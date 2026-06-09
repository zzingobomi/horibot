# Calibration UX Rewrite Plan

> ## 🎯 Success criteria (north star)
>
> 1. **재캘 거부감 0** — 평범한 user 가 UI 만으로, 외부 스크립트 / ad-hoc 디버깅
>    없이 처음부터 끝까지 자연스럽게 흘러갈 수 있어야 함.
> 2. **재현 가능한 품질** — 매 캘 라운드가 현재 floor (σ_rot 0.65° / σ_t 7.94mm)
>    와 거의 동일한 수준에 reliably 도달해야 함. "운 좋게 한 번 통과" X, "매번
>    통과" 가 표준.
>
> 이 둘이 본 rewrite 의 통과 기준. 어떤 UI/flow 결정이든 이 두 줄 위로 평가됨.
>
> ---
>
> **배경**: 현재 σ 결과는 자연스러운 user flow 로 도달한 게 아니라, 외부 검증
> 스크립트 + ad-hoc 디버깅을 거쳐 "별짓" 으로 맞춘 값. 사용자 트라우마의 출처.
> 새 ChArUco 보드 + SO-101 도착을 계기로 위 두 criteria 충족하도록 재설계.
>
> 본 문서는 한 세션의 논의 정리본 + 다음 세션 pickup anchor. 결정 X, scope frame + open questions.

---

## 1. Scope — 다시 짤 것 / 안 짤 것

### IN (UX/flow/검출/표현 layer)

- **검출 layer** — plain chessboard → ChArUco
  - `intrinsic.py`, `calibration_node.py:607` 근처 `findChessboardCornersSB` 콜
  - 보드 spec SSOT 한 곳 (config 파일 vs 모듈 상수 TBD)
- **Capture → Compute → Commit flow**
  - 멘탈 모델: 사용자는 "값 좋아질 때까지 COMMIT 반복" 이 자연스러움 → 시스템도
    그렇게 동작 (idempotent). 한 번 더 누르면 더 나빠지는 trap 없게.
  - 외부 스크립트 / UI 어느 경로로 들어와도 같은 semantics
- **추천 자세 visibility**
  - `next_pose_planner` 가 카메라 FOV + 보드 가시성 검증 안 함 (이번 세션 발견)
  - 추천한 자세에서 보드가 화면에 들어오는지 reproject 게이트
- **Panel UI 이주**
  - 이전 page 기반 → [RobotCalibrateMode](../frontend/src/pages/robotModes/) 패널
  - dockview 패널 셋 구성 (Capture / Compute / Commit / Validate 분리 또는 통합)
- **3D 결과 시각화**
  - [TaskResultLayer](../frontend/src/components/canvas/3d/TaskResultLayer.tsx) 같은
    typed dispatch 패턴 — 캡처한 보드 pose 들, per-pose 잔차 화살표, 현재 hand_eye
    카메라 시각화, COMPUTE 결과 미리보기

### Default-locked (검증된 자산, 기본은 손 X)

다음은 현재 σ floor 의 토대라서 **기본값은 그대로 둔다**. 절대 잠금은 아니고,
unlock 하려면 §1.1 의 조건 통과 필요.

- **BA solver** — `bundle_adjust.py`, standard/extended/physical_sag 3 모드.
  0.65°/7.94mm 결과 재현이 가장 큰 risk.
- **Offset apply pipeline** — `CorrectedIKSolver`, `JointCoordinates`,
  `LinkCoordinates`, `SagCoordinates`. URDF patch + FK/IK 적용 메커니즘.
- **npz 포맷 + 산출물 디스크 구조** — `robot/instances/<id>/calibration/`
- **Joint/link/sag offset 자체의 수학** — BA 가 출력하는 값의 정의

### 1.1 Default-locked 항목 unlock 조건

Implementation 중 위 항목을 건드리는 게 더 나은 길로 보이면, 다음 3 가지를 만족해야
unlock:

1. **Why** — 왜 IN 영역만으론 해결 불가한지 명시 (UX/flow/검출/UI/viz 로는 풀
   수 없는 근본 이유)
2. **Baseline 측정** — 변경 전/후 같은 캡처 셋으로 σ_rot/σ_t 비교. 후퇴 없거나
   개선이어야 함 (success criteria 2번에 직결).
3. **Rollback 경로** — 변경 단위가 독립 commit 으로 분리되어 후퇴 시 되돌리기
   쉬워야 함.

조건 안 만족하면 IN 영역에서 풀 방법 더 탐색.

---

## 2. 이번 세션에서 발견한 것 (다음 세션 컨텍스트)

### 2.1 새 ChArUco 보드 스펙 (calib.io)

- 5x7 / Checker 25mm / Marker 18mm / DICT_4X4 / Start Id 0
- 포맥스 5T 합지 (평탄성 OK, 라운드 모서리 의도된 안전 처리)
- `docs/calibration_workflow.md` §5 와 일치
- 현재 코드 ([backend/modules/calibration/intrinsic.py:9](../backend/modules/calibration/intrinsic.py#L9)) 는
  `CHECKERBOARD = (8, 5)` + plain `findChessboardCornersSB` — 보드 스펙과 불일치

### 2.2 트라우마 출처 — 4가지 페인 → 코드 매칭

| 사용자 페인 | 코드 측 흔적 |
|---|---|
| 추천 자세 중 캘판 안 보이는 게 있음 | `next_pose_planner.py` 에 visibility/FOV/board 키워드 0건 |
| 추천 따라가도 σ 수렴 X | (가설) 위 visibility 부재 결과 — 안 보이는 자세 캡처 → 검출 fail/noisy → BA outlier |
| 스크립트로 수렴 후 UI 재진입 → 수치 안 맞음 | 외부 도구와 UI 의 offset semantics 불일치 가능 |
| 다시 COMMIT 누르니 "누적되어 틀어짐" | **확정** — `joint_offsets` 는 cumulative 가산, `link/sag` 는 overwrite. 동일 bug 가 link/sag 에선 2026-05-28 에 잡혀 overwrite 로 전환됨 ([link_offsets.py:17](../backend/modules/calibration/link_offsets.py#L17), [sag_offsets.py:19](../backend/modules/calibration/sag_offsets.py#L19)) |

### 2.3 trap 의 구조

`joint_offsets` cumulative semantics + 외부 스크립트 / UI 가 같은 npz 를
absolute/delta 어느 쪽으로 다루는지 일관되지 않음 → 한 번이라도 경로 섞이면
과보정. 사용자 멘탈 모델 ("COMMIT = 현재 최선값으로 덮어쓰기") 와 정면 충돌.

---

## 3. Open questions (다음 세션에서 결정)

### Q1. `joint_offsets` semantics — cumulative 유지 vs overwrite 통일?

- link/sag 는 이미 overwrite 로 갔던 길. joint 만 cumulative 로 남은 게 의도인지
  빠뜨린 건지 불명.
- 의도라면 그 정당화 (BA 입력이 보정 후 joint state 위에서 동작한다는 가정?) 부터
  명문화 필요.
- 없다면 overwrite 로 통일 → "COMMIT 반복해도 idempotent" 가 무료로 따라옴.

### Q2. Capture/Compute/Commit 의 user contract

- COMMIT 은 idempotent ("같은 캡처 셋으로 N번 눌러도 동일 결과") 가 default?
- 아니면 "한 번의 commit = 한 번의 round" 라는 명시적 round 개념?
- COMPUTE preview 가 "이대로 COMMIT 하면 npz 안의 값이 정확히 X" 를 보여줘야 사용자가
  안심 — 현재는 BA 결과만 보여주고 disk 반영 후 어떻게 되는지 불투명.

### Q3. 추천 자세 planner 의 visibility gate

- 한 줄: "현재 (intrinsic + 현 hand_eye) 로 보드 4코너 reproject 시 모두 화면 안" 이
  추천 후보 필터. 하지만 hand_eye 아직 없는 첫 캡처는 어떻게?
- bootstrap 단계 (n<3) 는 사용자 자유 자세, 그 후부터 planner 활성 ?

### Q4. Panel UI 구조

- RobotCalibrateMode 안의 패널 단위 — Capture / Compute / Commit / Validate 4분할
  vs 통합 / Stepper UI / wizard ?
- 어느 시점에 panel 이 lock 되어 다음 단계 강제?
- Hand-Eye 외 (intrinsic / cross-robot) 도 같은 패널 셋에 들어가는지?

### Q5. 3D 결과 viz — 무엇을 그릴까

- 캡처된 보드 pose N 개를 base 프레임에 모두 그리기 (이상적이면 한 점에 모임)
- per-pose 잔차 = 보드 → re-projected 보드 차이 화살표
- 현재 hand_eye 로 카메라 콘 시각화
- COMPUTE 후 미리보기 (commit 전 상태로 disk 와 분리)

### Q6. 새 보드 + 새 코드 검증의 baseline

- 현 σ_rot 0.65° / σ_t 7.94mm 는 옛 보드 + 옛 코드 결과.
- 새 보드 + ChArUco 검출 + 같은 BA solver 로 베이스라인 다시 잡으면 얼마? (개선?
  비슷? 더 나쁨?)
- baseline 후퇴하면 어디서 후퇴했는지 분리 가능해야 (보드? 검출기? flow?)

---

## 4. References (이미 있는 문서)

- [calibration_workflow.md](calibration_workflow.md) — 현 캡처 절차 + 색 임계값 + 진단 룰
- [calibration_apply_flow.md](calibration_apply_flow.md) — 4종 산출물 적용 메커니즘
- [hand_eye_extended_ba.md](hand_eye_extended_ba.md) — BA 모드 + 물리 sag
- [accuracy_squeeze_plan.md](accuracy_squeeze_plan.md) — link/sag commit bug fix
  이력 (§1.6, 2026-05-28)
- [multi_robot_phase2_frontend.md](multi_robot_phase2_frontend.md) — RobotCalibrateMode 패널 시스템
- [so101_6dof_plan.md](so101_6dof_plan.md) — SO-101 도착 시 캘 동일 보드/flow 재사용

---

## 5. 다음 세션 진입점

위 Q1~Q6 중 어디서부터 풀지 우선순위 정하기. 추천 진입 순서 (가장 적은 결정으로 가장
큰 user pain 해소 → 점진적):

1. **Q1** (joint_offsets semantics) — 결정 안 하면 어떤 UI 짜도 trap 그대로
2. **Q3** (visibility gate) — 한 줄 추가로 user pain 절반 해소 가능
3. **Q2** (user contract) — Q1 정해지면 자연 follow-up
4. **Q4, Q5** (UI/viz) — 위 셋 정해진 후 구현 layer
5. **Q6** (baseline) — 새 코드 짜면서 동시에
