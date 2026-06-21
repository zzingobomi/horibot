# Scan Interactive Workflow — Design (2026-06-21, 논의 중)

> SO-101 + D405 scan task 의 interactive UX design 결정문. **구현 전 단계** —
> architectural 결정만 완료, 구현은 다음 세션. 본 문서로 논의 이어가기.

## 1. 사용자 의도

- **자세 잡기 = 수동** (토크오프). 자동 motion (yaml 자세 순회) 불필요
- **흐름**: 자세 잡기 + live PC preview → [캡처] → 반복 → [build TSDF] → 결과 확인
- raw 데이터 최대한 저장 (디버깅용)
- frontend 빙글이/timeout 신경
- 결과 시각화 — frontend mesh viewer (PLY layer) vs CloudCompare

## 2. Architecture 결정 — Task DSL 안 interactive

### 검토한 옵션 3개

| | scan = task DSL? | interactive? | 코드 변경 |
|---|---|---|---|
| A. yaml teach + automated ScanTask | ✓ task keep | △ teach만 interactive, scan 자체는 자동 | 코드 0 |
| B. ScanTask 폐기 + 캘 패턴 (별도 node + mode) | ✗ task 밖 | ✓ 완전 interactive | 큼 (scan_node + RobotScanMode 신규) |
| **C. Task DSL 자리 interactive step 추가** ★ | **✓ task keep** | **✓ task 안에서 interactive** | 중간 |

### C 채택 이유

- **사용자 정정**: "Task DSL 안 interactive step 박는 거 자체는 패러다임 깨짐 아님 — 미래 다른 task 도 user 개입 필요" (pick&place 중간 "물체 배치 확인" 같은 step)
- **장기 시스템 = 로봇 작업을 Task 로 표현하는 플랫폼**. Calibration 외에 사용자 개입 작업 계속 생길 거 → 범용 primitive 가치 ★ Scan 일회성 기능보다 큼
- B 면 mode 가 계속 늘어남 (RobotCalibrate/RobotScan/RobotWhateverMode...) → TaskRunner 우회 길 생김. 패러다임 일관성 깨짐
- A 면 사용자 의도 (자세 자동 X) 와 안 맞음

## 3. 결정된 설계

### 3.1 새 Step — `WaitForUserInput(signal_name)`

범용 primitive (**Scan 전용 X**). lego 블럭:

```python
# Scan
RepeatUntil(signal="build_now", body=[
    WaitForUserInput("capture"),
    CaptureScan(),
])

# 미래 다른 task 예시
MoveJ("home")
WaitForUserInput("물체 배치 완료")
DetectObject(...)
Pick(...)

MoveJ(view1)
WaitForUserInput("capture")
MoveJ(view2)
WaitForUserInput("capture")
Build(...)
```

→ **Task DSL 표현력 자체 확장**. Scan 문제 해결이 *부산물*.

### 3.2 ScanTask 변형 (interactive workflow)

```
TaskRunner
    ↓
ScanTask
    ↓
NewScanSession
    ↓
RepeatUntil(signal="build_now", body=[
  WaitForUserInput("capture")
  → CaptureScan
])
    ↓
BuildReconstruction
```

기존 `ForEach(MoveJByName + CaptureScan)` (yaml 자세 순회) → 위 구조로 변형.

### 3.3 `CaptureScan` orchestration service

frontend 가 SCENE3D_SNAPSHOT + STORAGE_PUT_SCAN 두 service 를 직접 두드리는 거 X. backend 가 한 번에 묶음:

```
snapshot
→ validate
→ save (storage put)
→ metadata update
```

naming: `CaptureScanReq` / `CaptureScanRes` (verb-first + sub-domain prefix, [naming_conventions.md](naming_conventions.md) 만족)

### 3.4 안 만드는 것

- ❌ **새 mode (RobotScanMode 같은)** — TasksPage 가 entry. task step type 보고 interactive overlay (캡처/build 버튼) 표시
- ❌ **새 scan_node** — orchestration service 위치는 §4 미해결

### 3.5 ScanTask 폐기 X

ScanTask 자체는 keep, 내부만 interactive 로 변형. *Scan = Task 로 표현* 패러다임 유지.

## 4. 미해결 결정 항목

다음 세션에서 결정:

### 4.1 `CaptureScan` orchestration service 위치
- (a) **Step 안에서 직접 chain** — `CaptureScan.execute()` 가 SCENE3D_SNAPSHOT + STORAGE_PUT_SCAN 두 service 호출. 새 backend service 추가 X
- (b) **새 backend service `CaptureScanReq`** — Scene3DNode 확장 (CLAUDE.md 의 "primitive only" 규칙 살짝 풀거나 재정의) 또는 새 위치. step 은 한 번만 호출
- 사용자 의견 = (b) "CaptureScan orchestration service 있어도 됨" 쪽으로 기울었으나 최종 결정 X

### 4.2 `WaitForUserInput` signal 매커니즘
- TaskRunner 에 user input wait state 추가 + `TASK_USER_INPUT` topic subscribe
- payload schema design 필요:
  ```python
  class TaskUserInputEvent(StrictModel):
      task_id: str  # 어느 task instance
      signal_name: str  # "capture", "build_now" 등
      data: dict = {}  # 추가 페이로드 (옵션)
  ```
- signal name convention — kebab-case? snake_case? namespace prefix? 결정 필요

### 4.3 TasksPage interactive UI 디자인
- 현재 task step type 자리 보고 동적 버튼 표시:
  - `WaitForUserInput("capture")` → [캡처] 버튼
  - `WaitForUserInput("build_now")` → [완료 + Build] 버튼
- captured scans count + delete 버튼 (사용자 잘못 캡처 시 되돌리기)
- Live PC preview overlay (현 Scene3DLayer 그대로 + scan task 자리 자동 stream ON)
- Reconstruction progress bar (RECONSTRUCTION_PROGRESS topic 5 stage 자리)

### 4.4 결과 mesh viewer (PLY layer)
- frontend three.js PLYLoader + Scene3DLayer 옆 mesh group
- 1-2일 별도 작업
- 안 짜면 결과 검증을 CloudCompare 외부 툴로 (PLY blob 다운로드)

### 4.5 디버깅용 raw metadata 추가 항목
ScanRecord 에 추가 저장 권장:
- snapshot 시점의 `tcp_in_base` + `cam_in_base` 4×4 matrix (지금은 motor_positions 만 — 캘 변경 시 재계산 가능하나 *그 시점 cam pose* 자리 직접 보존)
- consensus 의 reject ratio (몇 frame outlier) — TSDF quality 진단

## 5. 작업 list (구현 순서)

1. `WaitForUserInput(signal_name)` step — [backend/modules/task/steps.py](backend/modules/task/steps.py)
2. TaskRunner 에 user input wait state + TASK_USER_INPUT topic
3. `CaptureScan` orchestration (§4.1 결정 후 service 또는 step)
4. ScanTask 변형 — [backend/modules/task/tasks/scan.py](backend/modules/task/tasks/scan.py)
5. Frontend TasksPage interactive overlay
6. (옵션) PLY mesh viewer layer

## 6. 다음 세션 진입점

이 문서 읽고 — **§4 미해결 결정 항목** 부터 답 받기:
- 4.1: orchestration service 위치 (a vs b)
- 4.2: TASK_USER_INPUT topic schema 확정
- 4.3: TasksPage UI design 디테일

§5 작업 순서대로 진행.

## 7. 관련 문서

- [scan_pipeline_readiness.md](scan_pipeline_readiness.md) — SO-101 scan 시작 전 코드 검토. **단 robot_poses.yaml missing 항목은 본 design 의 ScanTask interactive 변형으로 무효** (자세 yaml 안 씀)
- [step_dsl.md](step_dsl.md) — typed Slot Step DSL. WaitForUserInput 도 같은 패러다임 안에서 새 step
- [naming_conventions.md](naming_conventions.md) — `CaptureScanReq` naming 규칙 적용
- [tsdf_pipeline.md](tsdf_pipeline.md) — ICP + TSDF 빌드 결정사항
- [calibration_workflow.md](calibration_workflow.md) — Calibration 의 user-driven capture flow. scan 은 같은 user-driven 이지만 *Task DSL 안* 패러다임 (별도 node X)
