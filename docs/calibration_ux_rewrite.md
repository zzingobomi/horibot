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
> 본 문서는 한 세션의 논의 정리본 + 다음 세션 pickup anchor. 2026-06-10 세션 진행 후 § 6 추가됨 — 진단 정정 + 일부 결정 + 새 open question.

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
| 다시 COMMIT 누르니 "누적되어 틀어짐" | ~~**확정** — joint_offsets cumulative 가 bug~~ → **2026-06-10 세션에서 retract**. cumulative 는 BA math contract (의도된 design). 진짜 trauma source 는 § 6.2 (last_compute.delta stale double-add + tool_offset cascade) |

### 2.3 trap 의 구조

`joint_offsets` cumulative semantics + 외부 스크립트 / UI 가 같은 npz 를
absolute/delta 어느 쪽으로 다루는지 일관되지 않음 → 한 번이라도 경로 섞이면
과보정. 사용자 멘탈 모델 ("COMMIT = 현재 최선값으로 덮어쓰기") 와 정면 충돌.

---

## 3. Open questions (다음 세션에서 결정)

### Q1. `joint_offsets` semantics — cumulative 유지 vs overwrite 통일?

**답 (2026-06-10): disk overwrite 통일, BA math 내부 cumulative.** § 6.6 참조.

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

## 5. 다음 세션 진입점 (구 — 2026-06-10 이전)

위 Q1~Q6 중 어디서부터 풀지 우선순위 정하기. 추천 진입 순서 (가장 적은 결정으로 가장
큰 user pain 해소 → 점진적):

1. **Q1** (joint_offsets semantics) — 결정 안 하면 어떤 UI 짜도 trap 그대로
2. **Q3** (visibility gate) — 한 줄 추가로 user pain 절반 해소 가능
3. **Q2** (user contract) — Q1 정해지면 자연 follow-up
4. **Q4, Q5** (UI/viz) — 위 셋 정해진 후 구현 layer
5. **Q6** (baseline) — 새 코드 짜면서 동시에

→ § 6 에 2026-06-10 세션 진행 결과 + 새 open question 추가됨. 다음 세션은 § 6.10 에서 시작.

---

## 6. 2026-06-10 세션 진행 (facts only)

### 6.1. § 2.2 진단 retract — 코드 진실

까본 결과 cumulative joint_offset 은 **BA math contract**, bug 아님:

- [joint_offsets.py:11-12](../backend/modules/calibration/joint_offsets.py#L11-L12) — *"BA 결과는 delta — 기존 파일이 있으면 cumulative하게 합산해 저장하는 책임은 호출 측"*
- [link_offsets.py:13-18](../backend/modules/calibration/link_offsets.py#L13-L18) — *"commit semantics: overwrite (joint_offsets와 다름). BA의 link_t 출력은 original URDF 기준 absolute total 값이라 cumulative 가산 금지"*
- [hand_eye.py:46-52](../backend/modules/calibration/hand_eye.py#L46-L52) Pose docstring — *"raw 만 보관. 매 COMPUTE 시점에 현재 시스템 offset 으로 일관 재해석"*
- [hand_eye.py:103-108](../backend/modules/calibration/hand_eye.py#L103-L108) `_resolve_pose_arrays` — *"매 COMPUTE 시점에 현재 JointCoordinates offset 으로 모든 포즈를 재해석"*
- [hand_eye.py:166-178](../backend/modules/calibration/hand_eye.py#L166-L178) compute_with_diagnostics — *"joint_offset_rad: delta — ja에 disk값 이미 가산됨 → cumulative. link_trans_m / rot: absolute total → overwrite. sag_k_rad_per_m: absolute total → overwrite"*

즉 joint cumulative 는 *"입력 angle = raw + 현재 offset, 출력 delta = 잔여 추정"* 이라는 BA 의 의도된 contract. link/sag 도 cumulative 라는 단어는 같지만 BA 가 absolute total 을 출력 — 다른 contract.

### 6.2. 진짜 trauma source — 두 bug 확정

**Bug A — `last_compute.delta` stale + double-add** ([calibration_node.py:402-540](../backend/nodes/application/calibration_node.py#L402-L540)):

```
COMPUTE → st.last_compute = {joint_offset_delta: δ, ...}
COMMIT (1st) → existing + δ = absolute_v1 (disk)
              st.last_compute 그대로 남아 있음 (invalidate X)
COMMIT (2nd, COMPUTE 재실행 X) → existing(absolute_v1) + δ = absolute_v1 + δ ❌
```

사용자 인지 X 의 trap. UI button auto-disable 도 없음.

**Bug B — tool_offset cascade** ([link_coordinates.py:82-100](../backend/core/coords/link_coordinates.py#L82-L100)):

`LinkCoordinates.commit_offsets` 가 received offsets 만 저장, existing 무시. BA 는 joint_id 1-5 만 추정 → COMMIT 시 ID=6 (tool_offset, 5/28 에 [`backend/patch_ee_link_offset.py`](https://github.com/zzingo5/horibot/blob/884b381/backend/patch_ee_link_offset.py) 으로 박힌 행) 사라짐.

**git timeline 5/28** (trauma 진앙):
- `884b381` "test" — patch_ee_link_offset.py 추가 (smoking gun)
- `70df81b` "fix: calibration 값 및 로직 버그 수정"
- `669676b` Revert "fix"
- `7d8d8a7` "revert: npz disk만 OLD로 유지 (코드 fix는 보존)"

### 6.3. hand_eye_extended_ba.md 분석

- **§5 Gauge freedom**: 이미 reg 로 잡힘. `joint_offset_reg=0.5`, `link_trans_reg=1.0`, `link_rot_reg=1.0`, `sag_k_reg=0.0`. reg=0 시 link_t -60mm 폭주, reg=10 시 link 0 + joint_offset 흡수 — sweet spot 검증.
- **§3 J2/J3 같은 방향 같은 크기 offset = link 미스매치 signature** (motor horn error 아님). diag_handeye_floor.py 의 "sink fake" 의심은 이 분석으로 sink 아닌 진짜 system error 잡은 것으로 해석.
- **§15a Floor noise sources 5개** 분석 (D405 intrinsic / 보드 정확도 / PnP / 자세 다양성 / 모션 블러).
- **§8c stale**: "joint_offsets + link_offsets 둘 다 누적 저장" — 5/28 link → overwrite 전환 후 doc 안 갱신됨.

### 6.4. Hardware fact (이번 세션 [hardware.md](hardware.md) 박힘)

- 현재 (2026-06): D405 on OMX, SO-101 미도착
- 미래 swap (SO-101 도착 시): D405 → SO-101, OMX → 720P USB UVC (DFOV 120°)
- 작업대: 55×34cm
- OMX reach: 일직선 stretched 500mm, 자세 다양성 ~350-400mm sphere
- 현재 [intrinsic.py:9](../backend/modules/calibration/intrinsic.py#L9) `CHECKERBOARD = (8, 5)` plain — 새 ChArUco 보드 (5×7/25/18) 와 불일치, 전환 필요

### 6.5. 캘 보드 (오늘 도착)

5×7 / 25mm / 18mm ChArUco / DICT_4X4 / Start Id 0 ([calibration_workflow.md §5](calibration_workflow.md)). 세 시나리오 (D405+omx / USB+omx / D405+so101) 모두 보드 fit 분석 끝.

D405 intrinsic = factory seed 사용 ([calibration_workflow.md §4](calibration_workflow.md)). omx+D405 시나리오에선 Intrinsic 재캘 SKIP. Intrinsic panel 은 USB 시나리오 (so101 도착 후 omx 다운그레이드) 용으로 유지.

### 6.6. Q1 답

- **Disk semantics**: 4종 npz (joint / link / sag / hand_eye) 모두 **overwrite** 통일.
- **BA math 내부**: cumulative (BA contract 유지).
- **Reconcile**: COMPUTE 끝에서 `joint_offset_absolute = current + BA delta` 계산해 `last_compute` 에 저장. COMMIT 은 `commit_absolute(absolute)` 로 overwrite (link/sag 패턴과 동일).
- **효과**: COMMIT 두 번 누르기 안전 (idempotent), 외부 스크립트 mixing 후 다음 COMPUTE+COMMIT 한 번에 정상화.

### 6.7. Frontend 현재 상태 확인

- [RobotCalibrateMode.tsx](../frontend/src/pages/robotModes/RobotCalibrateMode.tsx) panel 셋: robot-state / calibration / calibration-actions / scene-controls
- [HandEyeTab.tsx](../frontend/src/components/panels/CalibrationActionsPanel/HandEyeTab.tsx) — 354 줄, 3-column in 320-width panel (cramped)
- NextPoseCard 이미 있음. **visibility gate (reproject) 없음** — Q3 자리
- 주석 (line 34-35): "사용자가 페이스 잡음" 의도된 design
- COMMIT button auto-disable 조건: `!compute || computeStale` — 첫 클릭 후 disable 안 됨 → Bug A 노출

### 6.8. 이번 commit scope 후보 (현재까지 정리, 28 items)

#### Backend
1. disk overwrite for joint (`JointCoordinates.commit_absolute` 신설)
2. tool_offset 별도 npz + idempotent migration ([tool_coordinates.py](../backend/core/coords/tool_coordinates.py) + [tool_offset.py](../backend/modules/calibration/tool_offset.py) 이미 존재, 현재 상태 미확인)
3. ChArUco 검출 (intrinsic.py + preview_loop + 보드 spec SSOT)
4. visibility gate (next_pose_planner reproject + 거리 + tilt)
5. 백그라운드 자동 COMPUTE + σ live publish
6. API contract 확장 (`visible` / `joint_offset_absolute` / sigma topic / snapshot meta)
25. 자동 timestamp backup (모든 commit_* 함수)
26. rollback service

#### Frontend (panel 재구성 포함)
7. panel registry 확장
8. RobotCalibrateMode panel 배열 갱신
9. 신규 panel 컴포넌트 5개 (Intrinsic / HandEyeLive / HandEyeCaptures / HandEyeResults / HandEyeSave)
10. CalibrationStatusPanel 확장 (tool_offset 포함)
11. NextPoseCard visibility 마크
12. 3D viz layer (HandEyeBoardLayer — TaskResultLayer 패턴)
13. status / 어휘 cleanup
14. `pnpm gen:types` 자동 동기
27. rollback button (가장 최근 backup)

#### Docs
15-20. ux_rewrite.md / hand_eye_extended_ba.md §8c / calibration_apply_flow.md / calibration_workflow.md / multi_robot_phase2_frontend.md / CLAUDE.md 갱신
28. backup + rollback 메커니즘 명시

#### Tests
21-24. idempotency / tool_offset migration / visibility gate / ChArUco detection

### 6.9. Open questions (다음 세션 결정 필요)

#### O1. 저장/rollback 메커니즘 — file backup vs DB

- **Option A (file)**: timestamp 백업 + `.json` metadata → `robot/instances/<id>/calibration/.history/` 폴더 누적. 코드 ~150줄. 현재 git-based 분산 동기 패턴 유지 ([joint_coordinates.py:107](../backend/core/coords/joint_coordinates.py#L107) "다른 머신 전파는 git pull + 재시작이 담당").
- **Option B (DB)**: SQLite (single-user) → PostgreSQL (NAS, 미래) adapter pattern. 코드 ~500-1000줄 + schema 설계 + migration. 현재 git-based 분산 동기 패턴 변경 필요. task history / config history 등 다른 use case 와 통합 가능성.
- 미결: 이번 commit 에 어느 option? DB 가 별도 vision 인지 calibration UX rewrite 안인지 다음 세션 판단.

#### O2. Intrinsic SKIP 확정

- omx+D405 시점에 Intrinsic 단계 SKIP (factory seed 사용)
- USB 카메라 시나리오 (so101 도착 후) 에서만 Intrinsic panel 사용
- 확정 시 docs 갱신 필요

#### O3. Rollback 단계

- 1단계 (가장 최근 backup 만) vs N단계 (timestamp list 에서 선택)
- backup cleanup policy (예: 10개 넘으면 오래된 거 자동 삭제)

#### O4. Panel 구성 OK?

- 8 panel (Robot State / Calib Status / Intrinsic / Hand-Eye Live / Captures / Results / Save / Scene Controls)
- 또는 다르게 (예: Live + Captures 합쳐 7 panel)

#### O5. 3D viz 마운트 위치

- RobotsLayout R3F scene (모든 mode 에서 보임) vs Calibrate mode 한정

#### O6. `commit_offsets(delta)` 기존 API

- deprecate (호환 유지) vs 제거 (caller = calibration_node 하나만 확인됨, grep 미실행)

#### O7. 백그라운드 자동 BA

- capture 마다 자동 BA 실행 (user-paced design 유지하되 button 누르는 부담 줄임) 동의?

### 6.10. 다음 세션 진입점

1. **O1 결정** (file vs DB) — plan scope 의 핵심
2. **O2-O7 답** — 7개 미해결
3. **자료 수집** (코드 짜기 전):
   - [tool_coordinates.py](../backend/core/coords/tool_coordinates.py) + [tool_offset.py](../backend/modules/calibration/tool_offset.py) 현재 상태
   - `JointCoordinates.commit_offsets` caller grep
   - 현재 disk 의 `robot/instances/omx_f_0/calibration/link_offsets.npz` 에 ID=6 행 박혀 있는지 (user 확인 필요)
4. **Hand-simulate user sequence** (모든 button 누르는 순서) + edge case 종이 trace
5. **코드 한 번에 박기** (한 commit)
6. **여기서 acceptance**: mock boot (`uv run python main.py --host mock`) + dev 서버 (`pnpm dev`) + lint/type (`uv run ruff check . && uv run pyright && pnpm lint && pnpm build`) + unit test 통과
7. **user 집에서 hardware 검증** — ChArUco 보드 + omx+D405 재캘 → σ ≤ 0.65°/7.94mm 재현 + 재현성 3-5 라운드

### 6.11. 세션 핵심 결정 요약

- ✅ § 2.2 진단 retract (cumulative bug 아님)
- ✅ Q1 답: disk overwrite 통일, BA math 내부 cumulative
- ✅ trauma source 두 bug 확정 (last_compute stale double-add + tool_offset cascade)
- ✅ 5/28 git timeline = trauma 진앙
- ✅ Hardware fact docs/hardware.md 박힘
- ✅ ChArUco 전환 필요 확정
- ✅ 자동 backup + rollback safety net 필요 확정 (메커니즘은 O1 미결)
- ⏸ O1-O7 다음 세션 판단

## 7. 2026-06-10 실 산출물 (이번 commit 박힌 것)

§ 6 까지 결정만 정리. 본 절은 실제 코드로 박힌 결과. § 6.8 의 28-item scope 와 비교용.

### 7.1 Backend
- **commit_absolute 4종 통일** ([joint/link/sag/tool_coordinates.py](../backend/core/coords/)) — 옛 `commit_offsets` (joint cumulative) / `commit_offset` (tool 단수) 제거. 4종 모두 absolute overwrite + memory reload + `reload()` 메서드 (rollback 용).
- **calibration_node Bug A fix** — `_srv_handeye_compute` 가 BA delta + 현재 disk 로 absolute 계산해 `last_compute["_joint_absolute_by_id"]` stash. `_srv_handeye_commit` 끝에 `st.last_compute = None` invalidate → 두 번 클릭 시 "먼저 COMPUTE" 응답 → disk idempotent.
- **shared `_run_ba_and_stash` helper** — 수동 COMPUTE / 자동 BA 동일 logic 한 자리.
- **ChArUco 전환** ([modules/calibration/board.py](../backend/modules/calibration/board.py)) — plain chessboard → ChArUco (5×7/25/18/DICT_4X4). 보드 spec SSOT + `detect()` / `match_object_points()` / `draw()` / `board_corner_points_3d()`. `intrinsic.py` + `calibration_node._preview_loop` + `_srv_handeye_capture` 모두 본 모듈 사용. 일부 가림에도 검출 살아남음 → 사용자 자세 자유도 ↑.
- **자동 BA + σ live publish** — `_srv_handeye_capture` 끝에 `pose_count >= MIN_POSES_FOR_COMPUTE` 면 자동 BA → `CALIB_HANDEYE_SIGMA` topic 으로 `HandeyeSigmaState` (σ_rot, σ_t, pose_count, ba_mode, coach_verdict) publish. 사용자 [COMPUTE] 안 눌러도 즉시 σ 확인 (criteria #1 핵심).
- **visibility gate** ([next_pose_planner.is_pose_visible](../backend/modules/calibration/next_pose_planner.py)) — 후보 자세에서 FK + hand_eye + 보드 base reproject → 4 코너가 카메라 frame margin (5%) 안인지. `_compute_recommendations` 가 (intrinsic + hand_eye + `_estimate_board_base_frame()`) 모두 있을 때 visibility_check 클로저 build → recommend_many 가 `visible` / `visibility_reason` 마크. UI 가 회색 처리 (hard filter 아님).
- **보드 base 자동 추정** — `_estimate_board_base_frame` 가 모든 capture pose 의 `target2cam` → `gripper2base · cam2gripper · target2cam = target2base` 평균 (origin 평균 + SVD R 평균). hand_eye 안정되며 같이 refine.
- **Backup `.history/` mechanism** ([modules/calibration/backup.py](../backend/modules/calibration/backup.py)) — `snapshot(calibration_dir, tag, meta)` / `list_snapshots()` / `restore(timestamp)`. 매 COMMIT 진입 시 자동 snapshot. restore 도 직전에 "pre-restore" snapshot → undo 가능. 4종 + intrinsic + handeye_poses 한 묶음.
- **CALIB_BACKUP_LIST / CALIB_BACKUP_RESTORE 서비스** — `BackupEntry` (timestamp, tag, σ_rot, σ_t, capture_count, ba_mode) 리스트 + restore (restart_required 표시). `restore` 후 4종 Coordinates `reload()` + hand_eye/intrinsic load + `last_compute = None`.

### 7.2 Frontend
- **NextPoseCard visibility 마크** ([NextPoseCard.tsx](../frontend/src/components/panels/CalibrationActionsPanel/NextPoseCard.tsx)) — `visible=false` 후보 회색 + opacity 0.6 + ⚠ 안보임 라벨 (`visibility_reason` tooltip). hard filter 아님 — [이동] 시도 가능.
- **σ live badge** ([HandEyeTab.tsx](../frontend/src/components/panels/CalibrationActionsPanel/HandEyeTab.tsx)) — `CALIB_HANDEYE_SIGMA` 구독 → "Hand-Eye — Capture" 헤더 옆 inline (σ_rot ° / σ_t mm). thresholds 기반 색깔 (good/warn/bad).
- **Rollback 탭** ([RollbackTab.tsx](../frontend/src/components/panels/CalibrationActionsPanel/RollbackTab.tsx)) — 신규. snapshot 리스트 (timestamp / tag / σ_rot / σ_t / capture_count) + restore button. `restart_required=true` 시 백엔드 재시작 안내. Calibration Actions 패널의 3번째 탭.
- **generated/contract.ts 자동 regen** — `pnpm gen:types` 가 `CALIB_HANDEYE_SIGMA` topic + `CALIB_BACKUP_LIST/RESTORE` service + `BackupEntry/ListRes/RestoreReq/Res` + `HandeyeSigmaState` schema 자동 포함.

### 7.3 Tests / 검증
- `backend/tests/test_backup.py` — 8 unit tests (snapshot 생성 / meta / list 정렬 / garbage skip / restore roundtrip / pre-restore 자동 / missing timestamp / 정확 복원).
- mock boot: backend 7 노드 정상 시작, no errors. frontend `pnpm lint` + `pnpm build` clean.

### 7.4 § 6.8 의 28-item 대비 deferred
- **3D viz layer (HandEyeBoardLayer)** — board pose 시각화. nice-to-have. defer.
- **panel 8 분할** — Live / Captures / Results / Save 등 panel 단위로 분리. 현재 monolith HandEyeTab + Rollback 탭 추가로 hardware 검증 가능. defer.
- **Intrinsic 단계 docs** — § 6.5 의 omx+D405 SKIP / USB 시 활성 정책은 코드/UI 그대로, docs 만 다음 commit.
- 위 셋은 hardware 검증 결과 보고 다음 commit 으로 결정.

### 7.5 다음 단계 — hardware 검증

ChArUco 보드 + omx+D405 재캘:
1. 첫 캡처 2-3 자유 자세 (보드 base 추정 안 됨, visibility "unchecked")
2. n=3+ 후 자동 BA 시작 → σ live badge 표시 / visibility gate 활성
3. 추천 후보 따라가며 capture → σ 수렴 관찰 → COMMIT
4. **목표**: σ_rot ≤ 0.65° / σ_t ≤ 7.94mm 재현. 3-5 라운드 일관 도달.
5. 실패 시 Rollback 탭에서 pre-commit snapshot 으로 되돌리기.
