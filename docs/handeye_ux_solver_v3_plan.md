# Hand-Eye Calibration UX + Solver Pipeline v3 — Reconciliation Plan

> **이 문서의 위치** — 사용자가 준 "Hand-Eye Calibration UX + Solver Pipeline Specification v3" (이하 **스펙**) 은 *일반 수학 / 산업 표준 레벨* 로 쓰여있다. 본 문서는 그 스펙을 **실제 horibot 코드** (IRLS+Huber / physical_sag / JointPerturbationStrategy / PnP gate / coach / mean-based board pose) 와 대조해, "갈아엎기" 가 아니라 "버무리기" 로 가는 reconciliation plan 이다. 스펙의 *의도* 는 살리되, 우리 코드가 이미 더 정교한 자리는 그 이유를 박제하고 유지한다.
>
> **원칙** (사용자 강조): SSOT / 편법 금지 / 원칙대로. 산업 표준 도구·수식 우선 (study output = "표준 단계 다 밟기"). 구현 후 단위/e2e/분산 테스트 + 문서 업데이트 필수.
>
> **관련 문서** — [calibration_apply_flow.md](calibration_apply_flow.md) (4종 산출물 적용), [handeye_robust_irls_plan.md](handeye_robust_irls_plan.md) (IRLS sprint 박제), [hand_eye_extended_ba.md](hand_eye_extended_ba.md) (확장 BA + sag), [pose_library_design.md](pose_library_design.md) (ghost primitive design — 미구현), [calibration_workflow.md](calibration_workflow.md) (캡처 절차).

## 0. 핵심 결론 요약 (TL;DR)

| 스펙이 원하는 것 | 실제 코드 현실 | 본 plan 결정 |
|---|---|---|
| 단일 `T_base_board` (capture별 board pose 금지) | `bundle_adjust.py:21-24` 이미 mean-based 단일 T_b, 게다가 gauge freedom 회피까지 | ✅ **유지** — 스펙보다 정교. board를 변수로 두는 naive 안 채택 |
| Phase 1 = geometry only, RMS/BA 금지 | UI `manualModeActive` 로 σ를 *숨기지만* 백엔드는 n≥3 부터 auto-BA 실제 실행 | ⚠️ **solve를 진짜 뒤로 미룸**. Phase 1 = 순수 geometry traffic light |
| parameter별 observability `{he_rot, he_t, joint, sag}` | `observability.py` 는 pose 다양성 geometry metric → 단일 A/B/mid | ➕ **신설**: Fisher 정보행렬 `JᵀJ` 블록 conditioning. 기존 geometry는 Phase 1 신호로 재배치 |
| BA gating = observability별 unlock (개수 금지) | 개수 gating (`MIN_POSES` 3/8) + reg-only 약한 흡수. all-at-once | ➕ **staged freeze**: 블록별 observability < threshold → freeze |
| Ghost = 공통 primitive (calib 전용 X) | ghost 없음, `RobotModel` tint 없음, pose_library 미구현 | ➕ **신설** (option A): 재사용 가능 primitive + `CalibrationCandidateProvider` |
| capture metadata: 이미지+코너+관측 | `joint_angles` + `board_in_cam` 만 저장 (코너/이미지 X) | ➕ **확장**: ChArUco 코너(+선택적 이미지) 영속화 |
| Traffic light G/Y/R | binary 빨강/초록 overlay + post-solve 4-state coach | ➕ **Phase1 G/Y/R** 신설 (geometry only) |

**보존 자산** (스펙은 모름, 절대 버리지 말 것): IRLS+Huber robust BA / lumped-mass physical sag / `JointPerturbationStrategy` (5DOF) / PnP RMS reject gate / coach verdict / mean-based board pose.

## 구현 상태 (2026-06-18) — MVP1·2·3 완료

| 항목 | 상태 | 코드 / 테스트 |
|---|---|---|
| **MVP1 Phase 1 Traffic Light (실시간, 다양성 포함)** | ✅ | `capture_quality.py` (검출+tilt+pose/rotation/translation diversity → G/Y/R), preview loop `_add_capture_quality`, `CALIB_HANDEYE_PREVIEW.capture_verdict/reasons`, frontend `CaptureGuideOverlay` |
| MVP1 Phase 1/2 분리 (collection=BA 없음) | ✅ | `_RobotState.phase`, collection 엔 auto-BA 차단 (backlog 해소 + 스펙) |
| MVP2 per-param observability (Fisher/CRLB, 5블록) | ✅ | `observability_params.py`, `bundle_adjust.physical_sag_data_residual` SSOT |
| MVP2 staged gating (frozen_blocks) | ✅ | `bundle_adjust_hand_eye_physical_sag_irls(frozen_blocks=)`, `hand_eye.compute_with_diagnostics` |
| sag joint SSOT (robots.yaml, 5/6축 동일 코드) | ✅ | `RobotConfig.sag_joint_motor_ids` → BA/hand_eye/sag_corrected |
| `CALIB_HANDEYE_PARAM_OBSERVABILITY` topic + contract | ✅ | topic_map / api_contract / `HandeyeParamObservabilityState` / `ParamObservabilityCard` |
| MVP3 Ghost primitive (공통, 단일 클릭 선택) | ✅ | `RobotModel.tint` + `RobotPreviewLayer` + `previewStore.setGhost` |
| MVP3 캘 추천 = 클릭 → 단일 고스트 → 토크오프 수동 | ✅ | `PoseCandidates` (자동주행 `[이동]` + 명시신호 `[👎]` 제거) |
| 네이밍 정리 | ✅ | `NextPoseCard→PoseCandidates`, `CheckerboardOverlay→CaptureGuideOverlay`, `multi_start→begin_refinement` |
| capture metadata (corners 영속화) | ⏸ deferred | board_in_cam(PnP결과) 이미 저장 — corners는 재PnP용, 가치 낮아 보류 |
| **테스트** | ✅ | `test_capture_quality.py` + `test_observability_params.py`(실 SO-101 σ<GOOD) + headless e2e `test_calibration_e2e.py` + 분산 sim e2e `test_calibration_distributed_sim.py` |

**부수 성과**: 버그 2개 fix (geometry observability `np.array(dict)` TypeError, recommendations `KeyError(0)`). mock 카메라 ChArUco eye-in-hand 시뮬(`sim_board.py`, `CALIB_SIM_BOARD` env)로 캘 전 파이프라인 headless 검증 가능해짐. `recommendation_fail` 서비스/모델/상태 ([👎] 백엔드) 완전 제거.

**스펙 대비 변경 결정 (사용자 합의)**:
- **추천 = 동시 다중 고스트 X → 클릭한 1개만** ghost 표시 (여러 개 겹치면 3D 에서 헷갈림). 자동주행/[👎] 제거 — 토크오프 수동 매칭만 (스펙 "시스템이 자세 대신 정하지 않음" 철학 강화).
- **Phase 1 Traffic Light 가 핵심 UX** — 검출+tilt 뿐 아니라 *기존 캡처와의 다양성*까지 실시간 판정 ("지금 찍으면 좋은 데이터셋이 되나"). 캡처 0개일 땐 비교 대상 없어 거의 🟢.

## 최종 UX 플로우 (구현됨)

1. **[캘 시작]** (collection phase). SO-101 D405 공장 intrinsic 자동 seed (내부캘 X — OMX USB 만 내부캘).
2. 토크오프 → 손으로 자세. **카메라 overlay 실시간 G/Y/R** — 🔴 미검출/tilt/기존과 동일, 🟡 회전·거리 더, 🟢 새 자세+새 시야. 🟢에서 **[캡처]**.
3. **[자동 추천 시작 (N/8)]** — 8장 도달 시 활성 → `begin_refinement` (초기 solve) → Phase 2.
4. **σ 배지 + 식별성 카드(5블록: 카메라회전/위치 + 관절영점 + 링크기하 + 처짐)**. 약한 블록 자동 freeze.
5. **추천 후보 목록 → 클릭 = 그 1개 주황 고스트** → 토크오프 수동 매칭 → overlay 🟢 → [캡처] 반복.
6. **[COMMIT]** → DB 저장+active → **백엔드 재시작**으로 적용.

§2.1 의 "Phase 1 백엔드 강제" 는 collection 에서 auto-BA 차단으로 달성. capture metadata(corners) 만 deferred (board_in_cam 이미 저장돼 캘 동작에 불필요).

---

## 1. 현재 코드의 진실 (md 아닌 .py 읽고 확정)

### 1.1 솔버 (`bundle_adjust.py`)

- **board pose `T_b` = 명시 변수 아님.** 매 iteration 모든 포즈의 `T_base←board` 평균 ([bundle_adjust.py:206-210](../backend/modules/calibration/bundle_adjust.py#L206-L210)). 주석 21-24 이 이유 박제: *"T_b를 변수로 두면 X·T_b 결합 gauge가 ridge로 안 잡혀 X가 헤맴 — 실측 확인"*. → **스펙의 "B as variable" 은 reject, 의도(단일 board)는 충족.**
- **잔차 = consistency residual, reprojection 아님.** 포즈마다 `T_base←board` 가 평균에서 벗어난 양 (rot axis-angle 3 + trans 3, 미터). σ_rot/σ_t = 이 편차의 RMS. → 스펙의 "reprojection error" 는 우리에게 *consistency error* 로 매핑됨 (Phase 2 metric 으로 동일 역할).
- **운영 BA = `bundle_adjust_hand_eye_physical_sag_irls`** ([bundle_adjust.py:657](../backend/modules/calibration/bundle_adjust.py#L657)). 변수 layout (J=arm DOF):
  - `[0:J]` joint_offset · `[J:4J]` link_trans · `[4J:7J]` link_rot · `[7J:7J+2]` sag_k(J2,J3) · `[+3]` rod(handeye R) · `[+3]` t(handeye t).
  - OMX-F 5DOF = 43 DOF, SO-101 6DOF = 51 DOF.
  - 모든 블록을 **항상 동시 최적화** + soft regularization (`joint_offset_reg=0.5`, `link_*_reg=1.0`, `sag_k_reg=0.0`). **블록 freeze 메커니즘 없음** (basic BA 의 `estimate_joint_offsets` on/off 만 존재).
  - IRLS: outer loop 5회, σ̂=MAD/0.6745, κ=1.345·σ̂, w_i=min(1,κ/r_i). per-pose weight 만 (reg 항은 weight 안 곱). → **유지**.

### 1.2 관측성 (`observability.py`)

- **순수 geometry, solver 무관** (라인 11: *"BA/FK/hardware 모두 불필요"*). PnP `R_target2cam` + raw motor 만 사용. → **이건 그 자체로 Phase-1 호환** 진단이다.
- metric 4종: ① 카메라 광축 펼침 (axis_spread_deg) ② board tilt 분포 ③ relative-motion 회전축 span σ₃/σ₁ (Tsai degeneracy) ④ wrist roll raw 범위.
- `verdict()`: A (다양성 충분) / B (구조적 부족) / 중간. **단일 aggregate verdict — parameter별 분해 아님.**
- → **재배치**: 이 geometry observability 는 Phase 1 traffic light 의 핵심 입력. parameter별 observability (Phase 2) 는 별도 신설 (§3).

### 1.3 thresholds (`thresholds.py`) — SSOT

- 모든 노브가 여기 1곳 + `as_dict()` 로 frontend 미러 (`CALIB_HANDEYE_THRESHOLDS` 서비스). **새 임계값도 전부 여기 추가** (SSOT 유지).
- count-based gating 의 현 위치: `MIN_POSES_FOR_COMPUTE=3`, `MIN_POSES_FOR_TRUSTED_SIGMA=8`. → §3 에서 observability-based 로 보강 (count 는 *하한 안전장치* 로만 남김, 식별성 판단은 observability 가).

### 1.4 capture 영속화 (`persistence_models.py`)

- `CalibrationCaptureRecord`: `joint_angles`(URDF rad) + `board_in_cam`(4×4) + residual + weight. **ChArUco 코너 / 원본 이미지 미저장.**
- 함의: 재현·디버깅·재솔브 시 코너 픽셀이 없어 PnP 재계산 불가. 스펙이 코너+이미지 저장을 요구하는 이유와 정합 → 확장 (§4).

### 1.5 frontend

- ghost / preview / tint / previewStore **전무**. `RobotModel` 은 opacity·visible·linkVisibility 만.
- Phase 1/2 UI 분기 `manualModeActive` 존재하나, **Phase 1 에 per-capture geometry traffic light 없음** (CheckerboardOverlay 의 binary 빨강/초록 tilt+detect 뿐).
- 추천 자세는 숫자 텍스트만, hover preview 없음.

---

## 2. Phase 모델 — geometry(1) / solver(2) 진짜 분리

### 2.1 결정

스펙의 "Phase 1 에선 RMS/BA residual 안 씀" 을 **백엔드 레벨에서 강제**한다. 현재는 UI 만 숨기고 BA 는 돈다.

- **Phase 1 (Data Collection)**: hand-eye solve **안 함**. 평가 = 순수 geometry. 매 capture 후 `observability.analyze_pose_data` (이미 solver-free) + 신규 per-candidate geometry score 로 **G/Y/R traffic light** 산출. σ / BA / coach 전부 미산출·미표시.
- **Phase 1 → 2 전이**: geometry coverage 가 "초기 solve 가능" 수준 도달 시 (관측성 A 또는 최소 자세수+다양성 충족). 전이 trigger 에서 **Initial Hand-Eye Solve** 1회 (cv2.calibrateHandEye seed → physical_sag_irls BA).
- **Phase 2 (Solve / Refinement)**: 이제부터 σ / per-pose residual / coach / per-parameter observability / staged gating / ghost 추천 활성.

### 2.2 코드 매핑

- `manualModeActive=true` ≈ Phase 1. 단, [calibration_node.py 의 auto-BA queue (`_auto_ba_and_publish`)](../backend/nodes/application/calibration_node.py) 를 **Phase 1 에선 skip** 하도록 gate 추가. Phase 1 에선 대신 `_publish_phase1_geometry_state` (신규) 만 publish.
- `exitManualMode()` ≈ Phase 1→2 전이 = initial solve trigger. 현재도 multi-start BA 를 여기서 함 → 그 자리에 "최초 solve" 의미 명확화.
- 전이 조건을 thresholds SSOT 로 (`PHASE2_MIN_POSES`, geometry verdict gate).

### 2.3 의도적 비변경

전이 후 Phase 2 에서 capture 더 쌓이면 매번 auto-BA + σ live 갱신 (현 동작 유지). "Phase 1 금지" 는 *최초 solve 이전* 에만 적용.

---

## 3. Per-parameter observability + staged BA gating (본 plan 의 심장)

### 3.1 스펙 의도 vs 우리가 할 일

스펙: `{hand_eye_rotation: 0.91, hand_eye_translation: 0.85, joint_offset: 0.42, link_sag: 0.15}` → 블록별 식별성 → 식별성 threshold 로 unlock. "50장이어도 같은 회전축이면 식별 불가" 라 **개수 gating 금지**.

우리 코드: 블록별 식별성 개념 자체가 없음. reg + count + IRLS 만.

### 3.2 산업 표준 방법 — Fisher 정보행렬 / CRLB

**원칙대로**: BA 의 잔차 Jacobian `J` (scipy `least_squares` 가 수렴 시 `result.jac` 로 제공, 또는 finite-diff) 에서 정보행렬 `H = JᵀJ` 를 만든다. 파라미터 블록 `b` (handeye_R / handeye_t / joint_offset / sag_k / link) 의 **observability score** 를:

- **방법 A (블록 conditioning)**: `H` 의 블록 `b` 에 해당하는 부분행렬 `H_bb` 의 최소 특이값 σ_min(H_bb) 을 nominal scale 로 정규화. → "이 블록이 데이터로 얼마나 제약되나".
- **방법 B (marginal CRLB, 권장)**: `C = H⁻¹` (Cramér-Rao 하한, 정칙화된 pseudo-inverse). 블록 `b` 의 marginal 공분산 `C_bb` 의 trace/eigen 으로 **추정 불확실성** 산출. observability = `1 / (1 + normalized_variance)` ∈ (0,1]. → 다른 블록과의 상관까지 marginalize 해 "진짜로 풀 수 있나" 를 봄. **방법 B 채택** (Borm-Menq observability index 류의 marginal 접근, hand-eye 캘 표준).

각 블록을 nominal 단위로 스케일링 (rad / m / rad·m⁻¹) 해 비교 가능하게 정규화 → [0,1] score. threshold 는 thresholds.py SSOT (`OBS_UNLOCK_*`).

> **검증 가능성** — 합성 데이터 (알려진 X/joint/sag 로 포즈 생성 → 노이즈 주입) 로 "회전축 1개만" 데이터셋 = sag/joint observability 낮게, "다양한 축" = 높게 나오는지 단위테스트. 스펙의 "같은 회전축 50장" 시나리오를 그대로 재현.

### 3.3 Staged BA gating

`physical_sag_irls` 에 **block freeze** 메커니즘 추가:

- 새 인자 `frozen_blocks: set[str]` (예: `{"sag", "link", "joint"}`). frozen 블록은 최적화 벡터에서 제외 (값 0 또는 prior 고정), residual 계산엔 고정값 사용.
- gating loop (calibration_node):
  1. handeye (R,t) **항상 unlock** (캘 1차 목적).
  2. 임시 BA (handeye만 또는 현 unlocked set) → Jacobian → per-block observability.
  3. 블록 observability ≥ `OBS_UNLOCK_<block>` 면 unlock 후보. 단 의존 순서: joint_offset → link → sag (sag 는 link·joint 가 풀린 위에서만 의미).
  4. unlocked set 으로 최종 physical_sag_irls.
- → reg-only 의 "약하게 누르기" 를 **식별 가능할 때만 hard unlock** 으로 대체. 식별 안 되는 블록은 0 고정 (prior) → 잘못된 흡수 방지.

### 3.4 스펙 예시 ↔ 동작

| 데이터 | observability | optimize | freeze |
|---|---|---|---|
| 부족 | he OK, joint WEAK, sag NONE | handeye | joint, link, sag |
| 중간 | he OK, joint OK, sag WEAK | handeye, joint(, link) | sag |
| 충분 | 전부 OK | handeye, joint, link, sag | — |

### 3.5 frontend 노출

per-parameter observability → `CALIB_HANDEYE_PARAM_OBSERVABILITY` topic (신규). UI 는 **숫자 노출 X, 블록별 상태만** (OK/WEAK/INSUFFICIENT 색 dot) — 기존 ObservabilityBanner / coach 의 "수치 숨기고 verdict만" 철학 유지. "지금 sag 까지 풀렸어요 / joint 아직 부족" 류 안내.

---

## 4. Capture metadata 확장

스펙: joint / robot pose / camera image / checkerboard corners / camera→board observation / timestamp 저장.

현 `CalibrationCaptureRecord` 에 추가:
- `charuco_corners: list[[x,y]]` + `charuco_ids: list[int]` (PnP 입력 — 재솔브·디버깅 핵심, 작음).
- `image_blob_key: str | None` (선택적 원본 JPEG → ObjectStore. 용량 커서 **opt-in** 토글 / 디버그 모드만. SSOT: storage_layer 의 ObjectStore 패턴 재사용).
- `board_in_cam` (이미 있음) = camera→board observation. `joint_angles` = robot pose (FK 로 환산). timestamp = 이미 `Pose.timestamp`.

> **편법 금지** — 코너는 small structured data 라 RDB row 에. 이미지는 blob 이라 ObjectStore (scan 패턴과 동형). 두 경로 섞지 않음.

---

## 5. Phase 1 geometry traffic light (G/Y/R)

스펙의 Phase 1 평가 metric: checkerboard visibility / detection quality / tilt / pose difference / rotation diversity / translation diversity / joint pose variation.

기존 재사용:
- visibility / detection / tilt → `board.py` + `CheckerboardOverlay` (이미 있음, binary).
- rotation diversity / axis spread → `observability.analyze_pose_data` (이미 있음).

신규 (per-capture-candidate, solver-free):
- **pose difference / novelty**: 직전 캡처들과의 joint-space 거리 + 카메라 광축 신규성. `next_pose_planner` 의 diversity score 로직 재사용.
- 종합 → **G/Y/R**:
  - 🟢 GREEN: detected + tilt in-range + 기존과 충분히 다른 새 viewpoint.
  - 🟡 YELLOW: 캡처 가능하나 개선 여지 (tilt 경계 / 다양성 부족 / 회전축 중복).
  - 🔴 RED: 미검출 / tilt out / 직전과 거의 동일.
- topic `CALIB_HANDEYE_PHASE1_GUIDE` (신규) — verdict + 부족 요소 텍스트 ("tilt 부족" / "회전 다양성 더").

---

## 6. Ghost Robot — 공통 primitive (option A, 재사용 고려)

사용자 결정: **A (primitive 만)** 단 "추후 공통 재사용 고려 설계". → calibration 에 가두지 않는다.

### 6.1 위치 / 책임 (스펙의 "권장 구조" 따름)

```
frontend/src/components/scene/
  RobotModel.tsx          # 기존. tint prop 추가 (material color overlay)
  RobotPreviewLayer.tsx   # 신규 — ghost 렌더 (공통)
frontend/src/domain/stores/
  preview.ts              # 신규 — previewStore (robotId → ghost joints). 공통
backend/modules/calibration/
  candidate_provider.py   # 신규 — CalibrationCandidateProvider: PoseCandidate[] 만 산출
```

- **RobotPreviewLayer / previewStore 는 calibration 무관** — `setGhost(robotId, joints, source)` 단일 API. caller (캘 추천 hover / 추후 MoveJ / pose library) 누구든 호출. pose_library_design.md §5 그대로.
- **Ghost 가 안 하는 것** (스펙 명시): calibration 계산 / hand-eye solver / IK / observability. Ghost 는 렌더링·투명도·pose 적용·multi-ghost·enable/disable·selection 만.
- **CalibrationCandidateProvider 가 하는 것**: dataset + observability vector + robot state → `PoseCandidate[]` (`{joints, score, reason}`). 기존 `next_pose_planner` 의 strategy (JointPerturbation/Geometry) 가 그 엔진. provider 는 그 출력을 candidate 로 wrapping + ghost 에 먹임. 즉 **next_pose_planner 재사용**, 이름·경계만 스펙에 맞춰 정리.

### 6.2 Phase별 candidate (스펙)

- Phase 1: reachable IK + board visibility + tilt + 기본 viewpoint diversity (현 `is_pose_visible` + diversity).
- Phase 2: observability 기반 — 부족 parameter excitation 증가 자세. 예: joint WEAK → joint excitation↑, sag NONE → workspace spread↑. (per-param observability 결과를 strategy 입력으로.)

### 6.3 구현 범위 한계

poses.yaml / plans.yaml / Poses 패널 / Scan Plan 패널 (pose_library Phase A/B/D) 은 **이번 범위 외**. ghost primitive (Phase C 등가) + candidate provider 만. 단 previewStore/RobotPreviewLayer 인터페이스는 추후 pose library 가 그대로 얹도록 설계.

---

## 7. SSOT 터치포인트 (편법 금지 체크리스트)

| 자리 | SSOT 파일 | 추가 |
|---|---|---|
| 임계값 | `thresholds.py` + `as_dict()` | `OBS_UNLOCK_*`, `PHASE2_MIN_POSES`, phase1 diversity 임계 |
| topic/service 키 | `topic_map.py` (`Topic`/`Service`) | `CALIB_HANDEYE_PARAM_OBSERVABILITY`, `CALIB_HANDEYE_PHASE1_GUIDE` |
| 공개 계약 | `api_contract.py` (`PUBLIC_*`) → `pnpm gen:types` | 위 topic + Pydantic 모델. **손 동기화 금지** |
| robot 식별 | `robots.yaml` | sag joint 일반화 (현 OMX hardcode `_OMX_SAG_JOINT_ARM_INDICES`) 는 별도 follow-up |
| capture schema | `persistence_models.py` | charuco_corners/ids, image_blob_key |
| 서비스 응답 봉투 | `{success, message, data}` | 신규 서비스 전부 준수 |

**Ghost primitive 는 calibration 모듈에 두지 않음** (스펙 핵심 원칙) — `scene/` + `stores/preview.ts` 공통 자리.

---

## 8. 구현 순서 (스펙 MVP1/2/3) + 테스트 + 문서

각 MVP 끝에 **단위 + e2e(host_mock) + 분산(sim) 테스트 + 문서 업데이트** 를 포함 (사용자 요구).

### MVP 1 — Data Collection Phase
구현: capture metadata 확장 (§4) · Phase 1/2 백엔드 분리 (§2, auto-BA gate) · Phase 1 geometry traffic light (§5) · `RobotModel` tint prop (ghost 토대).
테스트:
- 단위: capture record round-trip (코너 직렬화), phase1 verdict 함수 (G/Y/R 경계), observability solver-free 재현.
- e2e: `host_mock` 에서 mock capture → phase1 guide topic 수신.
- 문서: calibration_workflow.md 의 Phase 1 절 + 본 문서 §2/§4/§5 "구현됨" 마크.

### MVP 2 — Solve / Refinement Phase
구현: initial hand-eye solve trigger (§2.2) · per-parameter observability (Fisher/CRLB, §3.2) · staged BA gating (block freeze, §3.3) · `CALIB_HANDEYE_PARAM_OBSERVABILITY` topic + UI 블록 상태 dot.
테스트:
- 단위: **합성 데이터** observability 검증 — 단일 회전축 → sag/joint INSUFFICIENT, 다양축 → OK (스펙 "50장 같은 축" 재현). staged gating: 부족 데이터에서 sag freeze 확인. IRLS 기존 acceptance test 회귀 (8장+합성 outlier).
- e2e: `host_mock` capture 누적 → param observability topic + gating 동작.
- 문서: handeye_robust_irls_plan.md / hand_eye_extended_ba.md 에 staged gating 절 추가.

### MVP 3 — Ghost primitive + Candidate Provider
구현: `previewStore` · `RobotPreviewLayer` (orange tint, opacity 0.35) · Scene 마운트 · `CalibrationCandidateProvider` (next_pose_planner wrapping) · 캘 추천 row hover → setGhost.
테스트:
- 단위: candidate provider PoseCandidate 스키마, IK reachable filter.
- e2e: 추천 hover → ghost joints store 반영 (component test). dev 서버 시각 검증 (`/robots/:id/calibrate`).
- 분산: sim 3-프로세스 (`host_pc_sim`/`pi_motor_sim`/`pi_camera_sim`) 에서 capture→observability→recommend topic 라우팅 확인.
- 문서: pose_library_design.md 에 "ghost primitive 구현됨 (calibration 첫 소비자)" 마크 + CLAUDE.md 캘 절 업데이트.

### 최종
- 전체 회귀 (`uv run ruff check` / `uv run pyright` / `pnpm lint` / `pnpm build`).
- CLAUDE.md 캘리브레이션 절 + 본 문서 status 갱신.

---

## 9. 미해결 / 합의 필요 자리

1. **per-parameter observability 방법** — 본 plan 은 **방법 B (marginal CRLB)** 채택. 방법 A (블록 conditioning) 보다 상관 marginalize 까지 해 정확하나 `H⁻¹` 비용. 5DOF·소수 포즈라 비용 무시 가능 → B 진행. (반대 시 짚어줄 것.)
2. **Phase 1→2 전이 조건** — geometry verdict A *또는* (min poses + diversity). 자동 전이 vs 사용자 [다음 단계] 버튼? 본 plan: **자동 전이 + 사용자 override 버튼** (현 exitManualMode UX 유지).
3. **원본 이미지 저장** — 기본 off (코너만), 디버그 토글 시 ObjectStore. (용량/프라이버시 고려 — 동의 시 그대로.)
4. **sag joint hardcode** (`_OMX_SAG_JOINT_ARM_INDICES`) — 본 plan 범위 밖 (SO-101 sag 진입 시 robots.yaml 일반화). MVP2 의 staged gating 은 현 OMX 가정 위에서.

**이 문서 OK 면 MVP1 부터 구현 시작.** 반대/수정은 §번호 + 한 줄로.
