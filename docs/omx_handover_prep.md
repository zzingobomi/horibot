# omx handover 투입 준비 — 현황·설계·실물 미지수

> **목적.** omx(giver)가 **자기 eye-in-hand 웹캠으로** 책상 위 얇고 긴 물체(매직펜류)를
> 보고 → 집어서 → so101(receiver)이 받기 좋게 공중에 제시 → so101이 받아가는
> handover 를, **처음부터 제대로 설계**하기 위한 근거 문서.
>
> **작성 원칙 (2026-07-23).** 이 문서의 사실은 전부 **코드/DB/debug 데이터 직독 또는
> git·docs 1차 사료**에 앵커가 달려 있다 (추측/요약 금지). "sim 으로 증명 가능한 것" 과
> "실물 첫 런에서만 풀리는 미지수" 를 §7 에서 정직하게 분리한다. **omx 실물 데이터는
> 현재 0** — 그래서 첫 런이 진단 가능하도록 관측성(§6)을 설계에 박아 넣는다.
>
> 관련 정본: [task.md §4](task.md) (PnP 실물 runbook), [motion.md §10~§12](motion.md)
> (IK 대수술/planner), [calibration.md §3](calibration.md) (σ floor),
> `grasp_debug_handoff_2026-07-16.md` (파지 실패 post-mortem 정본).

---

## 0. 한눈에 (준비도 + 목표)

**목표 = 비전 구동·적응·무티칭.** 펜의 위치/방향은 매 실행 미지 → omx 가 보고 파지
계획을 세우고(펜 방향 따라 J5 roll 매번 다름) 집는다. 그렇게 잡히면 제시된 펜의
자세도 매번 다르므로, so101 도 **보고 적응해서** 받는다. **정밀도 종착점은 so101**
— omx 는 mono·5축이라 완벽할 수 없고, "어떻게든 집어 대충 제시"하면 so101 의
depth+servo 가 흡수한다. (양쪽에 비전을 둔 이유 = 다양성 대응.)

| 영역 | 상태 | 핵심 갭 |
| --- | --- | --- |
| omx 팔 모션 (IK/MoveJ/MoveL) | 🟢 동작 | numeric IK (analytic 미적용 — §5.1) |
| omx 그리퍼 (약 70° 개구) | 🟢 제어 | held 임계가 Feetech 스케일 → 재특성화 필요 (§5.4) |
| omx 캘 (intrinsic/hand_eye/joint_offset) | 🟢 **양호** σ_rot 0.684°/σ_t 7.57mm | link_offset·sag 없음 |
| omx 카메라 = **eye-in-hand 웹캠** | 🟢 | URDF 에 카메라 link 없음 (so101 은 있음) |
| **omx 물체 인식 (mono→3D)** | 🔴 **경로 없음** | detector depth 전용 → z=0 평면 검출 신규 (§4-A, §5.3) |
| omx waypoint (search/observe·handover) | 🔴 home/rest 뿐 | **계산 가능** — 티칭 아님 (§4-C·§4-D) |
| handover task | 🟡 골격·mock only | "so101=눈, omx=blind, 티칭 포즈" 전제 → 재배선 |

**실질 작업 = ① omx mono z=0 검출(신규) ② omx look-then-move(신규) + 재배선 3건
(파지점 끝쪽 / 제시 포즈 계산 / so101 재검출 수취).** 나머지 골격·안전·실행
primitive 는 재사용.

---

## 1. 목표 아키텍처 (능력 A~E)

정밀도 역할 분담: **omx = 러프 적응 파지(mono, best-effort)**, **so101 = 정밀 폐루프
수취(depth+servo)**. omx 의 부정확성을 so101 이 뒤에서 흡수.

- **A. omx 가 펜을 본다** — observe 포즈(계산) → 2D 검출(mask/bbox) → **mono z=0
  평면 역투영**으로 (x,y) → mask 주축(PCA)으로 펜 방향.
- **B. omx 가 파지 계획을 세운다** — top-down + **J5 roll = 짧은 축에 조 정렬**.
  **파지점 = 펜의 먼 끝에서 N%** (가운데 물면 so101 이 받을 데 없음 — §1.1). grip z
  = z=0 + 펜 반경 가정.
- **C. omx 가 집는다** — **look-then-move**(eye-in-hand 로 가까이 재관측→XY refine) →
  마지막 몇 cm blind → close → verify → lift.
- **D. omx 가 제시한다** — **노출된 끝을 so101 쪽으로** 오리엔트한 제시 포즈(계산).
  든 펜의 정확한 자세는 omx 도 모름 (미끄러짐/파지점 오차) → 그래서 →
- **E. so101 이 보고 받아간다** — D405 로 **공중의 펜을 재검출** → **노출 세그먼트
  겨냥** → closed-loop 파지. 수취 순서 불변식(so101 held 판정 뒤에만 omx open) +
  cross-robot 충돌 게이트.

### 1.1 파지점 = handover 전체를 좌우하는 커플링

omx 가 어디를 무느냐가 so101 이 받을 수 있느냐를 결정한다.

- 파지점 = 한쪽 끝에서 ~25~35% (실물 튜닝). 너무 끝 = 모멘트 암↑ 펜이 조 안에서
  돌거나 빠짐 / 너무 가운데 = so101 노출 부족. **안정성 ↔ 노출 길이** 트레이드오프.
- omx 는 so101 에서 **먼 끝을 물고, 노출부를 so101 쪽으로** 제시.
- **mono 로 가능**: 펜은 z=0 평면에 누워 있으니 mask 주축 양 끝점을 z=0 역투영하면
  base frame 에서 펜 양 끝점·길이·방향이 나온다 (depth 불필요 — 평면 기하).
- **신규 실패 모드 (명시 실패)**: (omx 파지폭 + margin + so101 최소 파지 길이) > 펜
  길이 → "펜이 짧아 handover 불가". 파지 계획 단계에서 사전 판정.

---

## 2. 현황 (검증된 사실 — 앵커 포함)

### 2.1 omx 하드웨어·캘 (DB/설정 직독)
- capability `[move, calibrate]`, rgbd 없음 ([robots.yaml](../robot/robots.yaml)).
- 5축: **J1 yaw(z) · J2·J3·J4 평행 pitch(y) · J5 roll(x)** — 손목 yaw 없음, roll 있음
  ([omx_f.urdf](../robot/omx_f/urdf/omx_f.urdf)).
- 카메라 = **eye-in-hand UVC 웹캠** (1280×720, DFOV 120° 광각 → barrel distortion 큼,
  factory intrinsic 없음 — intrinsic 캘 필수). 근거: hand_eye 결과 `t_cam2gripper`
  크기 **93mm**(손목 근처) + 캘 파이프라인이 eye-in-hand 전용
  ([sim_board.py](../backend/modules/calibration/vision/sim_board.py):
  `T_cam_base = FK(joints) @ X_cam2gripper`) + 그걸로 σ_t 7.57mm 양호값 → 고정
  카메라면 불가능.
- 캘 활성 (horibot.db `calibration_results`): intrinsic ✅ / **hand_eye ✅
  (effective σ_rot 0.684° / σ_t 7.57mm — so101 0.82°/7.54mm 과 동급)** / joint_offset ✅.
  **없음**: link_offset, sag (so101 은 보유).
- waypoint (horibot.db): **`home`/`rest` 뿐**. search/observe/handover 포즈, search 그룹
  전무. (so101 은 `search` 그룹 + `search_auto_0..5` — [plan_search_poses.py](../backend/scripts/plan_search_poses.py) 로 계산 생성.)
- 그리퍼: XL330 모터, raw 1800~2600, 약 70° 개구 (omx_f.urdf/motors.yaml).

### 2.2 소프트웨어 경로
- [detector](../backend/modules/detector/): **depth 필수** — mask→depth→base 점군
  (object-centric). mono 경로/ray→평면 투영 **없음** (과거 폐기, geometry.py docstring).
- IK: numeric pybullet 로 동작. [analytic.py](../backend/modules/motion/adapters/analytic.py)
  (EAIK) 는 so101 6R 클래스 전용.
- [handover task](../backend/modules/tasks/handover/): 존재·mock 동작. **전제가 목표와
  상이** — 검출을 so101 D405 가 하고([steps.py](../backend/modules/tasks/handover/steps.py)
  `detect(so101,...)`), omx 는 blind open-loop pick, 제시는 티칭된 `handover` waypoint.
  pc.yaml 은 주석 TODO.

---

## 3. so101 여정 흉터 → omx 적용 (1차 사료 채굴)

so101 pick_and_place 를 실물 성공시키기까지의 흉터. 전부 commit hash / docs 앵커.
**omx handover 는 open-loop 이 3중첩(omx pick + so101 수취 + so101 place)** 이라
아래 흉터 상당수가 재발한다.

### 총론 — 왜 open-loop 을 버리고 closed-loop servo 로 갔나
근본원인 = **기구학 절대정확도가 자세의존적으로 ~1~2cm 부정확**
(`grasp_debug_handoff_2026-07-16.md` post-mortem):
- 정지한 같은 2.5cm 큐브가 카메라 자세마다 중심 ±7~10mm·크기 20~56mm 로 찍힘.
  **거리 비례** (30cm 서 ~40mm, 15cm 서 ~1cm) = 잔여 회전오차의 lever arm.
- FK 체인은 내부 일관(offline BA vs motion FK 0.05°, analytic vs pybullet 0.01°),
  hand_eye 불변 → 포팅 회귀 아님. 캘은 **정밀(σ 0.8°/7.5mm)하나 자세의존적으로
  부정확** — 고정 hand_eye 보정으로 평균은 잡아도 자세의존 잔차는 못 잡음.
- **"정적 인식을 더 똑똑하게 후처리"로는 ~1cm 를 못 넘는다.** 강건해지려면 피드백
  (closed-loop). eye-in-hand 의 진짜 이점 = 측정과 실행이 **같은 자세의 FK 오차를
  공유해 common-mode 상쇄**. 착수 전 offline 실증: 관측 편차 ∝ 카메라 거리 **r=0.95**.
- 전환 = `b7e8368`, 첫 실물 성공 = `f3eacff` (7/17). 실측 효과 (servo_pick trace
  `error_history_mm`): lateral **39 → 1.6mm** 수렴.

> **omx 함의 (가장 중요).** omx 는 mono(depth 없음)+open-loop pick 이라 so101 이 데인
> "정적 인식으로는 ~1cm 자세의존 오차 못 넘음"이 giver 측에 그대로 남는다. depth 가
> 없어 servo 의 z 앵커(윗면 centroid)도 못 씀. → **정밀도를 so101 수취(closed-loop)로
> 넘기는 설계가 필연.** handover docstring 가정② 도 "크로스캘 σ_t ~8mm, 2cm 큐브면
> 턱걸이"라 인정.

### 흉터 → omx 재발 여부 (요약, 상세 근거는 각 커밋/docs)

| # | 흉터 | 고침 (노브) | 근거 | omx 재발? |
| --- | --- | --- | --- | --- |
| 1 | base_z 앵커가 nip 튕김 | 파지 z=윗면−`grip_below_top_m=0.010` | `a47bdfe`, task.md §4.3 | **이식 불가** (mono z 못 봄) — z 는 z=0+반경 가정으로 |
| 2 | 접촉 인접 이동이 물체 흘림 | `speed_scale`+`gentle_speed_scale=0.25` (~2.5cm/s) | `a47bdfe` | **재발** — 공중 랑데부 상대속도=이젝션 |
| 3 | IK deepening 예산=선호 역전+전멸 정지 | group-major `_GROUP_IK_BUDGET=40` | `3d9ed25` | **재발** — omx=수치 IK 경로 |
| 4 | 단발 IK 위치잔차 cm급 | conditional refine `>0.003m`, best 추적(발산 방어) | `055ba51`, motion.md §10.E | **재발** |
| 5 | 워크스페이스 전멸=standoff 가 먼저 죽음 | 해석 IK+yaw 격자; standoff 재설계 | motion.md §10.F | **재발·최중요** — 랑데부를 공통 워크스페이스 안쪽에 |
| 6 | 수치 IK "해 놓쳐도 증명 못 함" | EAIK 해석 IK (so101 6R) | `591d999`, motion.md §11 | **omx=5축→해석 IK 불가→수치 폴백** (§5.1) |
| 7 | 바닥 스침 미검출 | `getClosestPoints(FLOOR_MARGIN_M=0.006)` | `ddb6ca8` | **재발** + cross-robot 충돌 새 축 |
| 8 | aspect 문턱 침묵 fallback | 절대 yaw 15° 격자 + width 물리 게이트 | `591d999`, motion.md §11.B | **재발** — omx 는 roll 격자로 동형 |
| 9 | 파지판정 gap 단독=물고도 EMPTY | gap **OR** load; `gripper_characterize.py` | `47924fa`, task.md §4.2 | **재발·핵심** (§5.4, 수취 순서 불변식의 토대) |
| 10 | commit blind 하강 바닥 스침 | 2단 하강 midstop 재앵커 | task.md §4.3 | **이식 불가** (depth 전용) — 관측성만 이식 |
| 11 | 엉뚱한 물체 집으러 감 | `_PICK_SCORE_MIN=0.45` + 로봇 베이스 기하 제외 | task.md §4.3 | **재발·TODO 명시** — "handover pick 도 동일 게이트 필요" |
| 12 | place open-loop 모서리 적치 | `_fuse_place_center` (관측 융합) | task.md §4.3.2 | **재발** — 최종 적치 open-loop |
| 13 | home 허브=쥔 채 최장 스윙 | RRT-Connect planner `PLAN_PATH` | `f27c816`, motion.md §12 | **부분** — cross-robot 확장 필요 |
| 14 | 관측 자세를 파지 가족서 파생 | 카메라 반구 배치+hand-eye 역변환 | `b1de9fa` | **부분** (depth+6DOF 전제) |
| 15 | 나쁜 앵커가 좋은 관측 기각 | servo gate `min_score_floor=0.30` 등 | task.md §4.3 | **이식 가능** (so101 수취 측만) |

**설계 시 최우선 3가지**: ① 랑데부를 **두 팔 공통 워크스페이스 안쪽**에 (흉터 5,
히트맵 미구현=첫 런 특성화) ② **침묵 fallback 금지** — 모든 스칼라 스위치에 로그+명시
실패 (흉터 8·프로젝트 대원칙) ③ omx 는 `IK=수치` 폴백 → 6DOF 성공 재사용 가정 금지,
walk/restart 전멸 예산 처음부터 고려 (흉터 6).

---

## 4. 능력별 델타 (현재 → 목표 → 재사용/개선/신규)

기존 [handover/steps.py](../backend/modules/tasks/handover/steps.py) 골격은 재사용 가치가
크다. 아래는 "so101=눈, omx=blind, 티칭 포즈" 전제를 목표(omx=눈, 적응, 무티칭)로
돌리는 델타.

### A. omx 가 펜을 본다 — **신규**
- 현재: omx 검출 경로 없음. detector 는 depth 전용이라 omx 로 호출해도 실패.
- 재사용: GDINO/SAM2 2D 백엔드(mask/bbox), `OrientedDetection` 스키마
  (grasp_yaw/footprint — 펜에 적합), [projection.py](../backend/modules/detector/projection.py)
  좌표변환, [plan_search_poses `look_point`](../backend/scripts/plan_search_poses.py)(ray∩평면).
- 신규: **mono z=0 검출** — `unproject_to_base(u,v,z_cam)` 의 z_cam 을 **ray∩(z=table_z)**
  로 대체하는 함수 1개 (`plane_point_from_pixel`). 순수 numpy → **오피스 단위테스트
  가능** (projection.py docstring 이 명시하는 성격). + omx observe 포즈 계산.

### B. omx 파지 계획 — **개선**
- 현재: [plan_omx_pick](../backend/modules/tasks/handover/steps.py) 이 **물체 중심**을
  집음(`g_world = obj.position - grip_below_top`), tilt(0,15,30)×yaw{90,0}.
- 개선: 파지점 **"먼 끝 + N%" + 노출 길이 판정** / grip z = z=0+펜 반경 / **top-down
  전용**(tilt≈0 — §5.1 5축 도달성) / yaw→**J5 roll** 매핑.
- 재사용: 그룹 resolve 패턴, `_grasp_quat`/`_approach_of`, 짧은변 우선.

### C. omx 집기 — **신규**
- 현재: [omx_pick](../backend/modules/tasks/handover/steps.py) 순수 open-loop.
- 재사용: `verify_grasp`(gap OR load), lift, close settle.
- 신규: **mono look-then-move** (관측 자세→가까이 재관측→XY refine→blind 하강).

### D. omx 제시 — **개선**
- 현재: [omx_present](../backend/modules/tasks/handover/steps.py) 이 **티칭된 `handover`
  waypoint** 로 이동 (`_HANDOVER_WAYPOINT="handover"`).
- 개선: 티칭 제거 → **제시 포즈 계산** (노출 세그먼트를 so101 도달영역 향해; base_pose
  크로스캘 + collision.py + IK). 근거: so101 수취가 이미 omx TCP(FK)에서 물체 위치를
  역산하므로 handover 포즈를 so101 이 미리 알 필요 없음.

### E. so101 수취 — **개선**
- 현재: [plan_receive](../backend/modules/tasks/handover/steps.py) 가 omx TCP **FK** 로
  위치 추정 (재검출 X). **코드에 이미 TODO**: "so101 D405 재검출로 정밀화 — 공중 물체
  base_z 대역을 이 task 전용으로 열어야 함."
- 개선: 그 TODO 구현 — so101 이 **제시된 펜 재검출** + **노출 세그먼트 겨냥** +
  closed-loop. base_z 게이트(현재 `-0.01~0.08`)를 공중 대역으로 개방.
- 재사용: `receive()` 실행부, **수취 순서 불변식**, 크로스로봇 충돌 게이트, 접근 부채꼴.

### 횡단 전제 (신규 config — 계산으로 못 없앰)
- **omx workcell ROI + z=0 테이블 앵커** — omx instance.yaml 에 `workcell:` 블록 없음.
  [shared_config](../backend/modules/shared_config/contract.py) 에 per-robot AABB(z 볼륨
  포함) 스키마·자리는 있음. omx 는 depth 가 없어 테이블을 스스로 못 봄 → **1회
  설정/측정**이 유일하게 못 없애는 수작업(매 실행 아님).
- **펜 지름 가정값** (grip z 계산).

---

## 5. 핵심 기술 검증 결과 (직독)

### 5.1 omx 5축 IK — top-down 은 정확 도달, tilt 은 대체로 불가 (확정)
- [resolve_reachable `_solve`](../backend/modules/motion/module.py#L636) 는 **항상 full
  quat 을 IK 로 정확히 맞춘다** (position-only 는 `_screen` 단계뿐).
- omx = 5축 ([kinematics.py](../backend/modules/motion/kinematics.py): `omx_f=5`). 임의
  6D 자세는 불가(자세 1자유도 부족). **그러나 top-down 파지**는: 접근축=수직(-z)이면
  방위각이 degenerate → 제약이 (3위치 + 2자세) = **정확히 5개** = 관절 5개 → 정확
  도달. J1(방위) · J2·3·4(r,z,아래향) · J5(roll=jaw yaw). tilt(≠수직)은 방위각 제약이
  되살아나 6번째 제약 → 대체로 도달 불가.
- **함의**: omx pick 은 **top-down 전용**으로 설계. resolve API 는 그대로(top-down quat
  은 정확 도달). 사용자 토크오프 관찰("yaw 안 되나 roll 돌리면 매직 집힘")과 수학 일치.
- **analytic IK**: EAIK 는 6R/so101 클래스용 → omx 5축은 `try_build`→None → numeric
  폴백. numeric 도 동작하지만, **결정성(도달불가 ms 확정, false-negative 소멸 —
  흉터 5·6)** 때문에 **OMX 고전 closed-form 을 선제 작성하기로 확정**(§8-2, 구현순서
  3). 부팅 로그 `IK=해석적` 확인, 침묵 폴백 금지.

### 5.2 omx tcp 축 규약 (가정 ①) — 구조적 성립 (확정)
[omx_f.urdf](../robot/omx_f/urdf/omx_f.urdf): tcp_joint 는 link5 자식·rpy 0·X 로 +0.092
오프셋 → **tcp X = joint5(roll)축 = 접근축**. gripper joint 축 Z, 핑거가 Y-gap 으로
닫힘 → **tcp Y = jaw 축**. so101 `_TOPDOWN`(x=approach, y=jaw) 규약과 일치. top-down 시
J5 roll = tcp X(수직) 둘레 회전 = jaw yaw. (남는 실물 미지수 = 물리 조립이 URDF 와
일치하는지.)

### 5.3 mono z=0 검출 확장지점 (확정)
[projection.py `unproject_to_base`](../backend/modules/detector/projection.py#L22) 가
`(u,v,z_cam)+intrinsic+TCP+hand_eye → base 3D`. mono 는 z_cam 을 **카메라 ray 와 z=table_z
평면의 교점 파라미터**로 대체 (ray 원점=카메라 base 위치, 방향=R_be·R_ce·[(u-cx)/fx,
(v-cy)/fy, 1]). 동일 교차 수학이 [look_point](../backend/scripts/plan_search_poses.py) 에
이미 존재. 순수 numpy·결정적 → **오피스 단위테스트로 정확도 검증 가능**.
- **필수: dist_coeffs undistort 선행.** omx 웹캠은 barrel distortion 큼(k1=-0.48).
  pixel→ray 전에 `cv2.undistortPoints` 로 왜곡 보정해야 화각 전체(~94°×57°)가
  정확히 z=0 로 투영됨. 생략(순진한 pinhole)하면 유효 시야 62°×37.5° 로 축소 + 엣지
  물체 위치 오차. (§8-1 커버리지 계산이 이 왜곡 반영 전제.)

### 5.4 파지 판정 — omx load 임계는 Feetech 스케일, 재특성화 필요 (확정)
- omx 도 [resolve.py](../backend/apps/resolve.py#L217) `ho_specs` 에 자동 등록 (open/close
  raw = motors.yaml). held = `close + 0.05*(open-close)` (gap 5%) **OR** load ≥
  `_HELD_LOAD_MIN_RAW=80`.
- **그러나 gap 5% 와 load 80 은 so101 Feetech STS3215 실측 휴리스틱.** omx = Dynamixel
  XL330 → **load 단위/스케일이 달라 80 은 무의미**. 얇은 펜은 gap≈닫힘이라 load 가
  유일 판별자인데 그게 미검증.
- 실측 근거 (servo_pick summary): 헛잡음이 load 48~56 에 앉음(임계 80 미달로 EMPTY
  판정). 정상 파지는 load 300+ (task.md §4.2).
- **→ omx 는 [gripper_characterize.py](../backend/scripts/gripper_characterize.py) 로
  빈손/물림/펜 경계값을 본 작업 전 재도출.** mock stall seam 으로 헛잡음/잡았다-놓침
  시뮬 테스트도.

### 5.5 closed-loop 정량 + 실패 신호 (servo_pick trace 직독)
- lateral **39.2→1.6mm** 수렴 (so101 depth servo, `error_history_mm`).
- 특이점: MoveL "관절도약 38° 구성플립(특이점 근접)" 거부 — **5축 omx 는 특이점 더
  흔함**.
- 바닥: height 7mm 물체서 `floor_contact_suspect` — z=0 근처 파지(펜 지름 ~1cm 동일)의
  바닥 접촉 리스크. omx 는 floor 피드백도 없음.
- common-mode 보정 `comp_mm` 가 tick 마다 성장 [0,0,0]→[5.9,2.2,8.4] (상대명령 서보).

### 5.6 공중/thin 물체 검출 실현성 (capability E)
- detect JSON 실측 (blue box 세션): 공중 후보 base_z=0.163·footprint 짧은축 1.4cm·
  **points 260개**(테이블 후보 1182 대비 소수) — 얇은/공중 물체는 depth 점군이 성김.
- [geometry `_body_z_band`](../backend/modules/detector/geometry.py#L123) 는 "질량 최상단
  z-gap 군집"(min 20% mass) 전제 — compact 물체용. **얇은 공중 펜 + 그리퍼 가림**은
  군집이 파편화될 수 있어 미검증 리스크.
- [align_and_merge_views docstring](../backend/modules/detector/geometry.py#L233) 정직한
  한계: 뷰별 절대오차 1.5~3.3cm, "융합으로 못 짜냄, sub-3cm 안정 파지는 close-loop
  별도 필요" — so101 수취가 closed-loop 이어야 하는 이유.

---

## 6. 관측성 설계 (omx 데이터가 0 이므로 필수)

첫 omx 실물 런이 **그 데이터만으로 원인분석 가능**해야 한다 (task.md §4 trace 규약
계승).
- **handover trace (jsonl)** — omx pick 매 tick: observation(펜 끝점/길이/yaw/points),
  z=0 역투영 입력·출력, 파지점 선택 근거, IK 잔차, comp, tcp_joints, gate 사유.
  so101 수취: 재검출 결과, 노출 세그먼트, servo error_history, held 판정(gap/load 값).
- **summary.json** — 결과/실패 사유(사유+다음행동)/close_attempts/워크스페이스 전멸 시
  그룹별 기각 사유 zip.
- **debug 아티팩트** — omx observe color + mask + z=0 투영 오버레이 (depth 없으니 평면
  투영 시각화가 검증 핵심).
- **부팅 로그** — `IK=수치/해석적`, hand_eye 투영 확인(침묵 identity 금지), workcell
  ROI/table_z 로드값.

---

## 7. sim 으로 증명 가능 vs 실물에서만 풀리는 미지수 (정직 분리)

**sim(mock ctx + 합성/노이즈)으로 증명 가능:**
- mono z=0 역투영 로직 (합성 이미지 + 왜곡 모델, 오피스 단위테스트).
- 5축 top-down IK 도달성 / tilt 불가 (PyBullet).
- look-then-move 상태머신, 파지점 끝쪽 선택 + 노출 길이 판정 + 짧은 펜 명시 실패.
- handover 재배선(눈=omx), cross-robot 충돌 게이트, 수취 순서 불변식.
- 파지 판정 gap-OR-load 로직 (mock stall seam 으로 헛잡음/놓침 시뮬).

**실물 첫 런에서만 풀리는 미지수 (추측 금지 — 데이터로 튜닝):**
- z=0 평면 XY 절대정확도 (캘 + 광각 undistort 실측). ← 파지 성공률의 지배 인자.
- 마지막 blind 하강 구간 오차 (omx 는 midstop 재앵커 불가).
- **omx 가 얇은 펜을 실제로 물어내는 성공률** (mono·open-loop — 가장 취약).
- omx held 경계값 (Dynamixel load 스케일 — `gripper_characterize.py` 로 확보).
- 공중 펜에 대한 so101 검출 정확도 (thin+그리퍼 가림).
- 두 팔 공통 워크스페이스 (히트맵 미구현 — 첫 런 특성화).
- 물리 조립이 URDF tcp/그리퍼 규약과 일치하는지 (가정 ①).
- 그리퍼 개구 vs 펜 지름 + 고무 보강 마찰.

---

## 8. 열린 질문 / 결정 필요

1. **omx observe 포즈 — 계산 완료 (2026-07-23): 단일 포즈로 충분 가능성 높음.**
   omx intrinsic(캘 rms 0.19px) + dist_coeffs 로 실제 화각 계산: 순진한 pinhole 은
   62°×37.5° 지만 **왜곡 반영 시 ~94°(H)×57°(V)** (스펙 DFOV 120° 는 왜곡 포함 대각).
   z=0 수직하향 발자국: **30cm 높이서 64×33cm, 40cm 서 85×43cm** (왜곡 반영). so101
   책상 ~34×74cm 대비 → **top-down 1포즈로 omx 도달 영역 커버, sweep 불필요 (계산
   확정).** **필수 요건**: 검출이 dist_coeffs 로 **undistort 후** 투영해야 함 (순진한
   pinhole 이면 36×20cm 로 축소 — §5.3).
   - **omx IK 실행 결과 (2026-07-23, offline pybullet, table=base z=0 전제)**: omx z=0
     도달 영역 = **26cm(전후)×44cm(좌우)**, centroid ≈ (0.208, 0.0). 관측 후보 576개
     중 38개 도달. **최적 observe = 수직하향(nadir) 카메라 높이 25cm roll 90° →
     도달영역 100% 커버.** 카메라 base ≈ (0.208, 0.007, 0.250), **관절값(deg) joint1~5
     = [2.3, -20.8, 7.8, 66.6, -0.3]** (IK 위치잔차 7.1mm — numeric, analytic 작성 후
     개선). roll 90°=넓은 화각 축(가로 94°)을 도달영역 넓은 축(좌우 44cm)에 정렬.
   - **남은 미지수 (실물)**: table_z 실제값(베이스가 책상 위 전제 — 다르면 포즈 시프트),
     사용자 집 테스트 대조. 계산 스크립트 = scratchpad `omx_observe_pose.py`
     (table_z 확정 시 재실행).
2. **analytic IK — 결정: 선제 작성 (2026-07-23 확정).** omx 는 EAIK 미적용(5축→수치
   폴백)이라 "analytic 적용"=**OMX 고전 closed-form 솔버 신규 작성**(EAIK 스위치가
   아님). **결정성 보장(도달불가 ms 확정, false-negative 소멸 — 흉터 5·6) + 산업표준
   study 가치**로 pick 구현 전 선제 작성 (구현순서 3). J1=atan2 겨냥 → 수직평면 2R/3R
   → J5 roll 구조. [analytic.py](../backend/modules/motion/adapters/analytic.py) 의
   snap/polish 패턴·부팅 로그(`IK=해석적` 침묵 폴백 금지) 규약 계승.
3. **cross-robot 충돌 = 후행 아님 (안전 의무).** 충돌 **게이트 자체**는 이미
   [collision.py](../backend/modules/tasks/handover/collision.py) `CrossRobotChecker` 로
   존재하고 실물 런 내내 활성이어야 함. 이번 목표의 **근접 그리퍼-대-그리퍼 핸드오프**
   (같은 펜 cm 간격) 기하는 현 "그리퍼 최대개구 고정" 보수 근사로는 거칠어 **초반
   정밀화 필요**. **후행 가능한 것은 오직 "motion `resolve` ③b 로 승격"(아키텍처
   정리, task.md §4.7 TODO)** — 게이트 존재/활성/정밀화가 아니라 위치만.
4. **so101 재검출 = 확정 (열린 질문 아님).** 수취는 so101 이 펜을 **다시 봐서**
   closed-loop 로 잡는다. FK 짐작은 **옵션이 아님** — 그게 정확히 §3 총론이 실증한
   "정적 계산으로는 자세의존 ~1~2cm 오차를 못 넘는다"이고, so101 이 closed-loop 로
   간 이유 그 자체다. 현재 handover v1 이 omx TCP FK 로 위치를 짐작하는데
   ([plan_receive](../backend/modules/tasks/handover/steps.py)), **그게 바로 갈아엎어야
   할 미검증 코드.** (구현 난점은 §5.6 — 공중의 얇은 펜을 so101 이 검출해야 하는 것;
   depth 는 있으나 object-centric 기하가 compact 물체용이라 확장 필요. 이건 "어떻게"의
   문제이지 FK 로 후퇴할 이유가 아님.)

---

## 구현 순서 (제안)

1. omx workcell ROI + z=0 table 앵커 (instance.yaml `workcell:`) + 펜 지름 config.
2. mono z=0 검출 (`plane_point_from_pixel`) + 펜 끝점/yaw — 오피스 단위테스트.
3. **OMX closed-form analytic IK 선제 작성** (§8-2 확정) — J1=atan2 → 수직평면 2R/3R
   → J5 roll. 부팅 로그 `IK=해석적` 확인 + 5축 top-down 도달 manifold 검증(PyBullet
   대조). IK 잔차/false-negative probe 로그도 여기서 박음.
4. omx observe 포즈 계산 + `gripper_characterize.py` omx 특성화 (실물 2분).
5. omx look-then-move pick (top-down + J5 roll, 파지점 끝쪽) — sim 소진.
6. **cross-robot 충돌 게이트 정밀화** (근접 그리퍼-대-그리퍼 핸드오프 기하) — 실물 런
   전 활성·검증. **안전 게이트라 재배선과 동시, 실물 런 선행.**
7. handover 재배선 (눈=omx) + 제시 포즈 계산 + so101 재검출 수취 + base_z 대역 개방.
8. 관측성(trace/summary/debug) 완비 후 실물 첫 런 → 미지수(§7) 특성화.
9. [후행] cross-robot 충돌 **승격**(collision.py → motion resolve ③b), planner cross-robot.
