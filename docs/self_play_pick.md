# Self-play Pick Loop — 설계 (WIP)

> 이 문서는 같이 채워나가는 living design doc. 작성 시점에 결론 박지 말고, **옵션 / 질문 / TODO 위주**로.
> 결정된 사항은 § "결정 로그" 아래에 날짜와 함께 박는다.

## 왜 self-play 인가 (피벗 정리)

원래 방향: pick 실패 → TCP 절대 정확도 측정/개선 → 정밀 캘 / peg / 중력 처짐 모델.

폐기 이유:

- "잘 집기"의 진짜 metric 은 **TCP 절대 정확도가 아니라 pick 성공률**. 카메라 → IK → sag → gripper 전체 체인의 end-to-end 정확도가 결정함.
- 사용자 환경: **DIY (연구소 아님)**, 강한 제약 = **사용자 개입 최소화**.
- 정밀 캘 / GT 측정 / peg 같은 hardware-side 개선은 ROI 낮음. self-play 로그가 쌓이면 잔차(residual) 보정 → bandit → RL 순서로 자연 진화 가능 (산업도 실제로 이 순서).

→ 한 줄 결론: **사용자가 큐브 1개 던져놓으면 로봇이 스스로 집고/떨어뜨리고/실패 기록하면서 데이터 누적 → 거기서 보정.**

---

## 메인 루프 — 큰 그림

공통 골격 (두 mode 동일):

```
loop:
    target = detect_object()                # GroundingDINO + D405
    move_above(target, +Δz)                 # hover (그리퍼 open)
    s1 = descend_until_target_z(target)     # stage 1: 무사 도달 vs current spike
    if s1 != "OK":
        log_and_continue
    s2 = close_gripper_and_check()          # stage 2: gripper position 으로 잡힘 확인
    s3 = lift_and_check() if s2 == "OK"     # stage 3: lift 후 position 변화로 떨어짐 확인
                         else None
    log_attempt(target, s1, s2, s3)
    if mode == "loop" and s3 == "OK":
        drop_at(random_workspace_xy())      # loop mode 만 후속 step
    else:
        retreat_home()
```

세부 분기는 § "단계" / § "Attempt 의 3-stage 측정" 참고.

---

## 단계 (mode 2개)

**구현 방식 (결정됨)**: 단일 task `self_play_pick` + `mode` 인자로 분기 (§ "결정 로그" #1). ChatGPT 원안의 Phase 0 (touch) / Phase 1 (push) 은 별도 mode 가 아니라 `grasp` attempt 내부의 측정 stage 로 흡수 (§ "결정 로그" #3).

| mode | 동작 |
| --- | --- |
| `grasp` | 사용자가 큐브 놓아두면 한 번 시도. attempt 마다 3-stage 결과 로깅 (§ "Attempt 의 3-stage 측정") |
| `loop` | `grasp` success → random drop → 다시 grasp 무한 반복 |

두 mode 모두 공통 골격 `detect → approach → descend → close → lift → log`. 차이는 `loop` 가 success 시 random drop 후속 step 을 추가하는 것뿐.

---

## Attempt 의 3-stage 측정

한 번의 grasp 시도가 다음 3 stage 의 결과를 모두 기록. **어디서 깨졌나로 잔차 source 가 자동 분리됨**.

### 정상 시퀀스 (그림)

```
  Step 1: hover (그리퍼 open)
       | |
       | |
        o          ← 큐브
      ████████
       책상

  Step 2: descend (큐브 옆을 따라 손가락 사이로 큐브 진입)
       | |
       |o|         ← 손가락 사이에 큐브 진입 (공중, 책상 안 닿음)
      ████████

  Step 3: 공중에서 close → 큐브 옆을 잡음
       |o|
      ████████

  Step 4: lift
        o          ← 큐브가 들림
      ████████
```

### Stage 별 판정

| Stage | 정상 | 비정상 | 측정 방법 |
| --- | --- | --- | --- |
| 1. descend 무사 도달 | current spike 없이 목표 z 도달 | 도중 spike = 어딘가 부딪힘 (xy 어긋남 or 큐브 위 찍음) | joint `Present_Current` 실시간 모니터링, baseline + threshold 초과 시 break (결정 로그 #4) |
| 2. close 후 큐브 잡힘 | gripper `Present_Position` 이 `GRIPPER_CLOSE_RAW`(1800) 보다 한참 큼 (큐브 두께만큼만 닫힘) | position ≈ 1800 = 끝까지 닫힘 = 빈손 | close 명령 후 `get_present_positions()` 1회 (결정 로그 #5) |
| 3. lift 후 안 떨어짐 | lift 후 gripper position 변화 거의 없음 | position 이 1800 가까이 가버림 = 떨어짐 | lift 직후 position 다시 읽음 (결정 로그 #5) |

### Stage 결과 → 잔차 진단

| 결과 | 의미 |
| --- | --- |
| Stage 1 자주 깨짐 | 카메라가 본 위치 자체가 어긋남 (detect + hand_eye + IK + sag 종합) |
| Stage 1 OK, Stage 2 빈손 | 위치는 맞는데 살짝 옆 — finger 가 큐브 옆을 비껴감 (**사용자가 처음 겪은 케이스**) |
| Stage 2 OK, Stage 3 떨어짐 | 잡긴 잡았는데 grip 약함 (torque / 모터 한계 / 미끄러짐) |

### 그리퍼 hardware/firmware 셋업 (이미 완료)

| 항목 | 값 | 위치 |
| --- | --- | --- |
| Operating Mode | 5 (Current-based Position) | Wizard EEPROM (사용자 박음) |
| Goal Current default | 200 mA | [motor_node.py:19](../backend/nodes/motor_node.py#L19) |
| Goal Position OPEN | 2600 | [motor_node.py:17](../backend/nodes/motor_node.py#L17) |
| Goal Position CLOSE | 1800 ("여유있게") | [motor_node.py:18](../backend/nodes/motor_node.py#L18) |
| `set_goal_current` 호출처 | 매 `_srv_gripper` 호출 시 자동 | [motor_node.py:231](../backend/nodes/motor_node.py#L231) |

Mode 5 + Goal Current 200mA 덕분에 close 가 큐브에 막히면 그 자리에서 멈춤, 빈손이면 1800 까지 끝까지 닫힘 → position 차이로 잡힘 여부 자동 판정. **self-play 진입을 위한 hardware/firmware 변경 X**.

---

## 셋업 (사용자 손)

### 객체 단계별 ramp-up (결정됨, § 결정 로그 #6)

쉬운 객체부터 어려운 객체로 순차 진행. 각 단계는 self-play 자동 루프, 데이터는 통합 누적, 보정 모델은 하나.

| 단계 | 객체 | 허용오차 | 진행 조건 |
| --- | --- | --- | --- |
| 1 | 종이컵 (~80mm 직경) | 큼 (~30mm) | 잘 잡히면 다음 단계 |
| 2 | 50mm 갈색 종이박스 | 중간 (~15mm) | 잘 잡히면 다음 단계 |
| 3 | 20mm 캘리브레이션 큐브 | 작음 (~3mm, 가장 어려움) | 누적 보정 모델로 잘 잡힐 때까지 학습 |

각 단계마다 셋업 점검:

- Detector prompt 변경 (`"paper cup"` / `"brown paper box"` / `"white calibration cube"`)
- gripper Goal Current 조정 (종이컵 찌그러짐 방지 위해 100mA 정도? — 1단계 진입 시 결정)
- gripper open 폭 — `GRIPPER_OPEN_RAW`=2600 이 50mm 박스 감쌀 만큼 벌어지는지 1회 확인

### 1회 calib

- 그리퍼 `Present_Position`: 객체 잡았을 때 값 (stage 2/3 판정 threshold). 객체별 측정 (객체마다 폭이 다르니).
- joint `Present_Current` baseline + spike threshold (stage 1 판정). 자세별 baseline 한 번씩 측정 권장.

### 추가로 의심되는 것들 (논의 필요)

- [ ] 카메라 각도 고정 / 책상 위 큐브 외 잡동사니 정리 정도는 사용자 책임?
- [ ] workspace 경계 정의 (random drop 범위) — 책상 끝 / 로봇 reach 안쪽 어떻게 잡을지?
- [ ] 실패 시 self-recovery (큐브가 reach 밖으로 굴러갔을 때) — 일단 사용자가 손으로 되돌리기?

---

## 로깅 스키마 (TBD)

각 attempt 마다 자동 수집할 후보:

- timestamp
- phase / attempt_id
- joint state (raw + urdf rad)
- predicted_xyz (detector 출력, base frame)
- depth median / depth ROI (디버그용)
- gripper close 시점 모터 current
- attempt result: `success` / `touched_only` / `missed` / `closed_empty` / `fail_other`
- fail_reason free text (선택)
- 사진/ROI 저장 여부

저장:

- 포맷 옵션: `jsonl` (append-only, 분석 쉬움) / `sqlite` (쿼리 강함) / `npz dump` (수치 위주)
- 위치: `robot/logs/self_play/<session>/...` 후보
- 사진 저장 시 용량 — full frame 30Hz 다 저장 X, attempt event 시점만

→ 일단 **jsonl + attempt 별 ROI thumbnail** 정도가 단순할 듯. 확정은 같이 결정.

---

## 메트릭 (자가 개선 효과 측정용)

- Phase 별 성공률 (전체)
- workspace XY heatmap 성공률 (bin)
- 시간/누적 attempt 수에 따른 성공률 곡선
- 보정 적용 전/후 비교 (residual 학습 후)

이게 사실상 옛 AccuracyTestPanel 의 "health check" 역할을 대체. 사용자 개입 0.

---

## 분석 → 보정 (Phase 2 이후)

- **Residual regression**: input = `(joint, pred_xyz, depth_features)` / output = `grasp_offset_correction (dx,dy,dz)`. 회귀 모델 (작은 MLP 또는 그냥 bin 평균) 충분.
- **Workspace bin 별 평균 오프셋** — heatmap 만들어 어디서 큰지 시각화.
- 큰 오차 위치 → (a) 캘 의심 (사용자 통보) / (b) 학습으로 흡수.

---

## Open Questions (먼저 결정해야 할 것들)

- [x] ~~`touch` mode success 판정~~ → **grasp Stage 1 (joint current spike)** 으로 흡수 (결정 로그 #3, #4)
- [x] ~~verify_grasped() 판정~~ → **gripper `Present_Position`** (결정 로그 #5)
- [x] ~~task 노드 통합 방식: step list vs 전용 task 클래스?~~ → **전용 task 클래스** (결정 로그 #2)
- [ ] 다양한 위치 만들기: `loop` mode 의 pick→random drop self-play vs 사용자가 가끔 흩뿌리기?
- [ ] gripper 정렬 (yaw rotation) 학습 포함? 일단 fixed yaw 로 시작?
- [ ] 안전 한계 — max attempts / stuck timeout / force limit?
- [x] ~~detect 실패 처리 — retry / search pose / stuck halt 정책~~ → **3회 retry → search pose 순회 (`search_*`) → stuck 시 task 중단** (결정 로그 #7)

---

## 관련 코드 (현재 상태)

- [backend/modules/task/tasks/pick_named_object.py](../backend/modules/task/tasks/pick_named_object.py) — 현재 단발 pick task (Grounded detection 사용)
- [backend/modules/task/step_executor.py](../backend/modules/task/step_executor.py) — step 실행기
- [backend/modules/task/step_types.py](../backend/modules/task/step_types.py) — `MoveTCPStep` / `DetectStep` / `GripperStep` 등
- [backend/nodes/task_node.py](../backend/nodes/task_node.py) — `TASK_REGISTRY`
- [backend/modules/detector/grounded_detector.py](../backend/modules/detector/grounded_detector.py) — Grounding DINO
- [backend/core/joint_state_cache.py](../backend/core/joint_state_cache.py) — joint state 공유
- [backend/core/frame_cache.py](../backend/core/frame_cache.py) — camera frame 공유

---

## 결정 로그

(날짜 + 결정 내용 + 근거. 같이 채움.)

### 2026-05-24 #1 — Phase 분리 방식: 단일 task + `mode` 인자

**결정**: Phase 0~3 을 각각 별 task 로 만들지 않고, 단일 task `self_play_pick` 에 `mode: "touch" | "push" | "grasp" | "loop"` 인자로 분기.

**근거**:

- 네 phase 모두 `detect → approach → action → judge → log` 공통 골격. 차이는 `action` / `judge` 두 strategy 뿐 → 별도 task 로 쪼개면 90% 중복.
- 자동 phase 진화(state machine) 는 매력적이지만 전환 임계값 정의가 어렵고 초기 디버깅 비용 큼 — 일단 사람이 trigger 하는 단순한 형태부터.
- 사용자 개입은 phase 전환 시(며칠에 한 번)만 발생 — 개입 최소화 제약 안에 들어옴.
- 자동 진화가 필요해지면 외부 wrapper `self_play_auto` 가 metric 보고 mode 승급하는 layer 로 추가 가능. 즉 단일 task = 그 layer 의 기반.

### 2026-05-24 #2 — Task 통합 방식: 전용 task 클래스

**결정**: 기존 `step list` (e.g. `[MoveTCPStep, DetectStep, GripperStep, ...]`) 방식 대신 **전용 task 클래스** `SelfPlayPickTask` 신규.

**근거**:

- loop / mode 별 branching / random_drop 같은 제어 흐름이 step list 로 표현하기 어색.
- mode 별 strategy 객체 plug-in 이 클래스 구조에 자연.
- [pick_named_object](../backend/modules/task/tasks/pick_named_object.py) 같은 기존 단발 task 는 step list 가 적합하지만, self-play 는 attempt 단위 loop 가 본질이라 패턴이 다름.
- [task_node.py](../backend/nodes/task_node.py) 의 `TASK_REGISTRY` 에 추가하는 방식은 동일.

### 2026-05-24 #3 — Touch / Push 별도 mode 폐기, 3-stage 측정으로 흡수

**결정**: ChatGPT 원안의 Phase 0 (touch) / Phase 1 (push) 별도 mode 폐기. grasp attempt 한 번이 이미 multi-stage 라 stage 별 결과만 기록하면 동일 정보 획득. mode 수 4 → 2 (`grasp`, `loop`).

**근거**:

- 한 attempt = `descend → close → lift` 세 단계 모두 success/fail 판정 가능. touch 만 따로 돌릴 이유 없음.
- 사용자 직관과 일치 — 최종 목표가 "잘 집기" 인데 "터치만 하는 task" 가 생기는 게 어색.
- ChatGPT 우려("초반 성공률 0% → 데이터 없음") 도 해소됨: stage 별 부분 success 자체가 데이터.
- 안전성: descend 중 current spike → 즉시 break 라 처음부터 grasp 가도 충돌 위험 안 늘어남.

### 2026-05-24 #4 — Stage 1 (descend) 판정: joint current spike (safety break + 어긋남 감지)

**결정**: descend 중 joint `Present_Current` 실시간 모니터링. baseline + threshold 초과 시 break + log. **정상 = spike 없이 목표 z 도달**.

**근거**:

- D405 가 wrist-mounted 라 EE 가 큐브에 다가갈수록 카메라도 같이 가까워짐 → depth 측정 객관성 ↓. depth-based 판정 부적합.
- Dynamixel X-series 는 `Present_Current` 무료 제공. 별도 sensor 불필요.
- 천천히 descend (5~10mm/s) 하면 가속 current 가 거의 0 → baseline 평평 → contact spike 명확.
- spike 시점의 EE z 값으로 어긋남 type 구분 가능 (큐브 위 찍음 vs 큐브 옆 부딪힘 등). **spike 자체가 풍부한 데이터**.

**Threshold calibration**: 첫 1회 사용자가 자세별 baseline + spike 값 측정. 큐브 무게/자세 범위 안 변하니 안정적.

### 2026-05-24 #5 — Stage 2/3 (잡힘 / 떨어짐) 판정: gripper `Present_Position`

**결정**: close / lift 직후 그리퍼 모터(ID 6) 의 `Present_Position` 읽기.

- close 후 position 이 `GRIPPER_CLOSE_RAW`(1800) 보다 한참 큼 → 큐브에 막혀 멈춤 → **잡힘 ✅**
- close 후 position ≈ 1800 → 끝까지 닫힘 → **빈손 ❌**
- lift 후 position 변화 없음 → 잡고 있음 ✅
- lift 후 position 이 1800 가까이 가버림 → **떨어짐 ❌**

**근거**:

- Wizard EEPROM 에 그리퍼(ID 6) operating mode 5 (Current-based Position) 박혀 있음. Goal Current default 200 mA.
- 코드는 매 `_srv_gripper` 호출 시 `set_goal_current` + `set_goal_position` 같이 set ([motor_node.py:231](../backend/nodes/motor_node.py#L231)).
- → close 명령에서 큐브 닿으면 200 mA 도달 시 그 자리에서 멈춤, 큐브 없으면 1800 까지 끝까지 닫힘.
- 큐브 잡힌 상태의 position 값 = 1회 calib 으로 측정해서 threshold (§ "셋업").
- self-play 진입을 위한 추가 hardware/firmware 변경 불필요.

### 2026-05-24 #7 — Detect 실패 처리: retry → search pose → stuck halt

**결정**: 카메라에 객체 안 보일 때 처리 정책.

```
detect 시도 →  ┬─ 성공 → 다음 stage
              ├─ workspace 밖 → skip + log (다음 attempt)
              └─ fail
                 ↓
              1초 간격 3회 retry
                 ↓ 모두 fail
              search pose 순회 (robot_poses.yaml 의 `search_*`, lexical 정렬)
                 ↓ 모두 fail
              task 중단 + log 경고 + `TASK_STATE` 토픽으로 사용자 알림
                 ↓
              사용자가 책상 정리 후 재시작
```

**근거**:

- 큐브가 reach 밖으로 굴러간 경우 등 자동 복구 불가 케이스는 사용자 개입이 결국 필요 — "개입 최소화 ≠ 0" 원칙 적용.
- Search pose 는 사용자가 `robot_poses.yaml` 에 `search_1`, `search_2`, ... 형태로 추가 → self-play runner 가 자동으로 lexical 정렬 list 만들어 순회. search pose 가 0개여도 (yaml 추가 안 함) graceful fallback — 3회 retry 후 바로 중단.
- 사용자 환경에선 hover 자세에서 보통 바로 보임. search pose 는 책상 가장자리/뒤 같은 사각지대 커버용. 3~4개면 충분.

**구현**:

- [robot_poses.py](../backend/core/robot_poses.py) 의 `list_pose_names("search_")` 헬퍼로 search list 수집.
- `SelfPlayRunner` 가 시작 시 list 캐시, detect fail 시 순회.
- workspace 경계 check: 별도 config 또는 hand_eye 기반 reach radius 추정 (1차 구현 시 단순 box 한계로 시작).

### 2026-05-24 #6 — 객체 단계별 ramp-up: 종이컵 → 50mm 박스 → 20mm 큐브

**결정**: self-play 를 객체 단계로 진행. 셋 다 self-play 자동 루프 돌림. 데이터는 통합 누적, 보정 모델은 하나.

| 단계 | 객체 | 진행 조건 |
| --- | --- | --- |
| 1 | 종이컵 (~80mm) | 잘 잡히면 다음 |
| 2 | 50mm 갈색 박스 | 잘 잡히면 다음 |
| 3 | 20mm 캘리브레이션 큐브 | 누적 보정으로 잘 잡힐 때까지 |

**근거**:

- 기술적으로는 가장 어려운 20mm 만 해도 보정 학습 충분 (보정 모델 입력이 객체 무관: `joint_state + pred_xyz` 만 — 객체 폭/색/크기 feature 사용 X. 학습되는 패턴은 로봇 자체의 오차 = 객체와 무관).
- 다만 사용자 명시 선호로 큰 객체부터 ramp-up:
  - 시스템 단계 확인 (종이컵도 못 잡으면 self-play 이전에 디버깅 필요)
  - 초기 0% 성공률 회피 (큰 객체로 데이터 수집 시작 → 점점 어려운 케이스로)
  - 정신적 진행감
- 데이터 통합 누적 → 단계 진행될수록 보정 모델 강해짐 → 20mm 큐브 단계에선 누적 데이터 기반 보정이 작은 허용오차도 흡수.

**연관 셋업 변수** (각 단계 진입 시 결정):

- Detector prompt — `"paper cup"` / `"brown paper box"` / `"white calibration cube"`
- gripper Goal Current — 종이컵 100mA 정도? (찌그러짐 방지)
- gripper open 폭 — `GRIPPER_OPEN_RAW`=2600 이 50mm 박스 충분히 벌어지는지 확인

---

## 진행 메모

(세션별 작업 기록.)

- _다음 세션에서 채움._
