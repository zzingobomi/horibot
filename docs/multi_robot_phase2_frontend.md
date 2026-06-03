# Multi-Robot Phase 2 — Frontend / Namespace / UX

Phase 1 (foundation) 완료 후 남은 자리. [multi_robot_architecture.md §12](multi_robot_architecture.md) 의 Phase 2 의 *frontend / UX / namespace* 슬라이스.

본 문서는 **entry point** — 본격 논의는 새 세션에서 이 문서 보고 시작. 각 섹션은 *현 상태 + 쟁점 + 옵션 sketch* 수준이고 *결정* 은 보류.

## §0. 컨텍스트

### 완료된 자리 (본 작업의 *전제*)

- **Phase 1 foundation** ([multi_robot_walkthrough.md](multi_robot_walkthrough.md))
  - 폴더 split: `robot/<type>/` (URDF/mesh) + `robot/instances/<id>/` (calibration / scans / logs)
  - `RobotRegistry` ([robot/robots.yaml](../robot/robots.yaml) SSOT) → `RobotConfig` / `get_iksolver(id)` / `get_motor_backend(id)` / `get_camera_capture(id)`
  - Protocol abstraction: `IKSolver` / `MotorBackend` / `CameraCapture`
  - Coordinates → `dict[robot_id]` (joint/link/sag/tool)
  - **N=1 환경에서 SO-101 도착 시 SWAP 가능** (entry + FeetechBackend adapter 추가)

- **Typed messaging** ([typed_messaging.md](typed_messaging.md))
  - 모든 토픽/서비스 payload pydantic (면제 자리 제외)
  - `backend/api_contract.py` SSOT → frontend codegen 자동 (`pnpm gen:types`)
  - `BridgeClient` generic 화 (subscribe / publish / callService typed)

### 본 문서 scope (Phase 2 의 일부)

| § | 항목 | 성격 |
|---|---|---|
| §1 | Zenoh namespace 개편 | 기술 (transport 설계) |
| §2 | 페이지 역할 기획 | 기획 (UX 요구사항) |
| §3 | 멀티로봇 UX | 기획 + UX |
| §4 | 프론트 데이터 플로우 재정비 | 기술 (§1+§2+§3 의 downstream) |

§2/§3 (기획) 이 §1/§4 (코드 결정) 의 *upstream* — 기획 먼저 잡고 코드 결정 내려가는 게 자연스러움.

### Phase 2 의 *다른* 자리 (본 문서 scope 밖)

- [distributed_topology.md](distributed_topology.md) — hori1/2/3 Pi 3대 분산
- Coordinator (multi-robot 동시 동작 조율) — 별도 슬라이스
- dual-arm 시퀀스 (e.g. 한 로봇이 picks, 다른 로봇이 places) — 별도

## §1. Zenoh Namespace 개편

### 현 상태

토픽 / 서비스 키가 single robot 가정:

```
omx/motor/state/joint         ← 어느 로봇?
omx/motor/srv/enable          ← 어느 로봇?
omx/motion/srv/move_tcp       ← 어느 로봇?
omx/camera/state/status       ← 어느 카메라? (현재는 omx_f_0 의 D405)
omx/calib/srv/handeye/capture ← 어느 로봇의 hand-eye?
omx/task/state                ← global vs 로봇별?
omx/system/heartbeat          ← global (노드별 발행, robot 무관)
omx/pointcloud/stream         ← 어느 카메라?
```

[`backend/core/transport/topic_map.py`](../backend/core/transport/topic_map.py) 의 모든 키가 `omx/<domain>/...` 형태. 로봇 식별자 없음.

### 쟁점

1. **prefix 만 추가**: `omx/<robot_id>/motor/state/joint`
   - Pro: 변경 최소, 기존 키 끝 부분 보존
   - Con: 모든 노드 publish 자리에서 `robot_id` 채워야. global topic (heartbeat / system_log) 과 robot-scoped 토픽 구분 필요
2. **도메인별 차등**:
   - robot-scoped: `omx/robot/<id>/motor/state/joint`, `omx/robot/<id>/motion/srv/move_tcp`
   - global: `omx/system/heartbeat`, `omx/task/state` (task 가 robot 별인지 global 인지 별도 결정)
3. **flat 유지 + payload 안에 `robot_id` 필드** (현 [`BaseRobotMessage`](../backend/core/transport/messages/base.py) 패턴)
   - Pro: 키 안 바뀜 → frontend 변경 최소
   - Con: Zenoh subscriber 가 *robot 별 필터링* 못함 — 모든 메시지 받아서 payload 보고 filter. 분산 환경 부담 ↑
4. **하이브리드**: 키는 robot prefix, payload 도 `robot_id` (validation 용)

### 결정해야 할 자리

- robot-scoped vs global 토픽 분류 (전체 35+ 토픽/서비스)
- prefix 위치 (`omx/<id>/...` vs `omx/robot/<id>/...`)
- task 가 robot 별인지 global 인지 (한 task 가 N robot 조작 시?)
- Coordinator (Phase 2 의 별도 슬라이스) 토픽이 robot prefix 위에 어떻게 얹히나
- 마이그레이션 — 한 번에 갈아엎기 vs 점진 (backward compat 토픽 2벌 publish?)

### 참조

- [multi_robot_architecture.md §토픽 namespace 재설계 candidate](multi_robot_architecture.md)
- ROS2 의 namespace 패턴 (`/<robot_name>/joint_states`) — 산업 표준 참고

## §2. 페이지 역할 기획

### 현 상태

[frontend/src/pages/](../frontend/src/pages/) 의 메뉴들 — 책임 / 사용자 동작이 *명확하지 않음*.

| 페이지 | 추정 책임 | 모호한 자리 |
|---|---|---|
| Dashboard | 시스템 상태 한눈에 | 어떤 *상태* 가 중요한가? 로봇별 모터/모션 상태? 노드 heartbeat? 시각화? |
| Workspace3D | 3D 디지털 트윈 + teleop | dashboard 와 겹침? teleop 어디서? |
| Motion | move_j / move_l 컨트롤 | Workspace3D 의 부분집합? 독립 페이지? |
| Calibration | 캘 캡처 / commit | intrinsic / hand-eye / 표시 / TSDF mesh build — 한 페이지에 다 들어가나? |
| Task | task 디버거 / 실행 | task tree 시각화? RUN/PAUSE/STEP 컨트롤? log? |
| ... | | |

### 쟁점

1. **페이지 = 사용자 작업 단계** vs **페이지 = 기능 그룹**:
   - 전자: "캘 캡처 중인 사용자", "task 디버깅 중인 사용자" 등 시나리오 기반
   - 후자: 현재 — 도메인별 (motion / calibration / task) 묶음
2. **Workspace3D 의 위상** — 다른 페이지의 *상위* (모든 페이지가 그 위에 패널)? 아니면 독립 페이지?
3. **Dashboard 의 위치** — 진짜 *시작 화면* (어디로 갈지 결정) vs 단순 status overview
4. **사용자 시나리오 명문화 필요**:
   - "캘리브레이션 새로 잡는 시나리오" — 어떤 페이지 흐름?
   - "teleop 으로 데모하는 시나리오" — 어떤 페이지?
   - "task 디버깅 시나리오" — 어떤 페이지?
   - "scan + TSDF build 시나리오" — 어떤 페이지?
   - "여러 로봇 동시 운영 시나리오" — 어떤 페이지? (§3 와 연결)

### 결정해야 할 자리

- 사용자 시나리오 enumeration (5–10개)
- 각 시나리오의 페이지 흐름
- 페이지 간 *상태 공유* 자리 (선택된 로봇 / pose / detection 등이 페이지 이동해도 유지?)
- 페이지 추가/제거/통합 결정

## §3. 멀티로봇 UX

### 현 상태

UI 가 *single robot 가정* — 로봇 선택 자리 없음. 모든 store / 컴포넌트가 *그* 로봇 (현 `omx_f_0`) 만 가정.

### 쟁점

1. **로봇 selector 위치**:
   - global header (모든 페이지 공통, app 전체에서 *active robot* 하나)
   - per-page selector (페이지마다 다른 로봇 가능)
   - 페이지 안에 split view (N 로봇 동시 표시, selector 없음)
2. **동시 운영 시각화**:
   - tab 전환 (active = 1 robot at a time)
   - split / multi-pane (N robot 옆에 옆에)
   - card grid (dashboard 류)
3. **페이지별 패턴 차이**:
   - Workspace3D 의 3D scene — N robot URDF 같은 scene 에 띄움? 별도 scene?
   - Calibration — robot 별로 독립 (한 로봇 캘 잡는 동안 다른 로봇 unrelated)
   - Task — task 가 multi-robot 인 경우 selector 무의미 (task 가 로봇 지정)
4. **3D scene 의 좌표계**:
   - N 로봇이 같은 world frame? 각자 frame?
   - calibration 결과 (hand-eye / pointcloud) 의 frame 정합

### 결정해야 할 자리

- 페이지별 single-robot vs multi-robot view 패턴
- "active robot" 개념의 layer (URL? store? per-page?)
- 새 로봇 추가 시 UI 어떻게 자동 인식 (robots.yaml fetch?)
- 분산 환경 (PC + Pi N대) 의 UI 영향 (latency / 연결 끊김 표시)

### 참조 (산업 사례)

- ROS rqt — robot namespace selector + tool 별 dropdown
- Foxglove Studio — multi-source panel 추가, robot 별 topic prefix
- 산업용 로봇 컨트롤러 (Teach Pendant) — 보통 single robot per pendant

## §4. 프론트 데이터 플로우 재정비

### 현 상태

[frontend/src/store/](../frontend/src/store/) 의 Zustand store 들이 *single robot 가정*:

```
robotStore       — joints / configs / torque (어느 로봇?)
cameraStore      — status (어느 카메라?)
motionStore      — trajectory state (어느 로봇?)
detectorStore    — detections (어느 카메라/로봇?)
pointCloudStore  — voxel / scans (어느 카메라?)
taskStore        — state / tree (global? robot 별?)
sceneStore       — 3D scene config
systemStore      — bridge / nodes / logs
```

`useBridge.ts` 가 모든 토픽 1회 subscribe → store 1개씩 푸시. 토픽 = robot global → store = single.

### 쟁점

1. **store 재설계** — robot-scoped 부분을 `dict[robot_id]` 로 펴기 vs *active robot* 만 들고 selector 로 swap
2. **subscribe 패턴** — 모든 robot 토픽 한 번에 vs active robot 만
3. **codegen 의 추가 layer** — contract.ts 에 robot prefix 도 자동 반영?
4. **점진 리팩 vs rebuild**:
   - 점진: §1 namespace 결정 후 store 1개씩 multi-robot 화. 기존 single-robot UI 동작 유지.
   - rebuild: §2 / §3 결정 후 페이지 / store 통째 재설계.
5. **현 코드 *억지로 끼워맞춤* 자리**:
   - 무엇이 억지? (사용자 지적 — 구체 자리 enumeration 필요)
   - openapi codegen 은 type 만 — 데이터 흐름 / 컴포넌트 책임 / 라우팅은 별도

### 결정해야 할 자리

- store 의 단위 (per-robot dict vs active robot)
- subscribe / unsubscribe 라이프사이클 (페이지 mount / robot 선택 변경 / 분산 머신 끊김)
- codegen 산출물의 위상 — types / contract 외에 추가 layer 필요한지
- 점진 vs rebuild

## §5. 의존성 / 작업 순서 (잠정)

```
§2 페이지 역할 ─┐
                ├─→ §3 멀티로봇 UX ─→ §4 프론트 재정비
                ┘                       ↑
                                §1 Zenoh namespace
                                (§4 의 backend dependency)
```

- §2/§3 (기획) 먼저
- §1 namespace 는 §4 와 같은 슬라이스 또는 약간 먼저 (frontend codegen 영향)
- §4 가 가장 큰 작업 (점진 리팩 시 여러 슬라이스)

## 다음 세션 시작 시 첫 prompt 추천

> docs/multi_robot_phase2_frontend.md 보고 §2 (페이지 역할 기획) 부터 시작.
> 사용자 시나리오 5–10개 enumeration → 각 시나리오의 페이지 흐름 정의.

`§2` / `§3` 의 *기획* 자리는 코드 작업 아니라 *요구사항 정리* — 화이트보드 / 종이에 시나리오 + UI 흐름 그리며 가는 게 자연스러움. 결정 fix 되면 본 문서에 채우고 §1 / §4 코드 작업 진입.
