# Pick & Grasp / Motion 디버깅 handoff (2026-07-07~08 세션)

작은 흰 큐브 pick&place 가 계속 못 집던 문제를 파고든 세션 기록 + 남은 문제/버그 +
다음 세션 할 일. **모든 변경은 working tree 에 있고 커밋 안 됨** (사용자 검토 후 결정).

---

## 0. 한 줄 요약

`pick_and_place` 가 큐브를 못 집는 문제를 파다가 **접근 primitive 를 MoveL(Cartesian
직선) → MoveJPose(관절 공간) 로 전면 재설계** + IK 어댑터 **multi-restart** + **pinch
offset** 도입까지 왔음. 한 번 집혔지만 **운**이고 아직 신뢰성 없음. detection 이 가끔
엉뚱한 데 찍히고, 모션이 급하고, verify_grasp 가 별개 버그로 실패함.

---

## 1. 이번 세션 진행 히스토리 (어떻게 여기까지 왔나)

1. **시작**: `ApproachAlongTool` 의 MoveL 이 `MotionFailed: MoveL failed` — trajectory
   FAILED. 원인 = cartesian loop 의 IK 실패 (`MoveL IK 실패 | s=2.2cm`).
2. **오진 1 (내가)**: "SO-101 워크스페이스 한계로 도달 불가" 라고 docstring("FK 20만
   샘플, 수직 z≤0.05") 을 그대로 읊음. → 사용자 반박: "J2 150° 까지 가는데 왜 불가?"
3. **진짜 원인**: 실제 보정 kinematics 로 heavy sampling(seed 400) 해보니 — **위치는
   전부 도달**(position-only IK OK), 문제는 **PyBullet `calculateInverseKinematics`
   가 seed 1개짜리 local 솔버라 존재하는 해를 놓침**. straight-down 14° 자세도 seed
   400 개면 나옴. → **reach 문제 아님, IK 솔버 robustness 문제.**
4. **핵심 재정의 (사용자 주도)**: 목표는 "그리퍼를 -Z 수직으로" 가 아니라 "**큐브를
   잘 집는다**". 필요없는 자세 제약을 걸어 도달 가능한 위치의 IK 를 스스로 죽이고
   있었음. + MoveL(자세 고정 직선)은 **높이마다 도달 자세가 변하는** SO-101 에서
   구조적으로 실패 (heavy sampling 으로 확인 — 같은 자세로 z 2.3cm 만 OK, 4.3cm+ FAIL).
5. **재설계 (사용자 주도)**: "MoveL 에 갇히지 말고 다양한 primitive 써라. lift 도 월드
   Z 상승만이 답이냐?" → **pick 전체를 관절 공간 config 전이로**. approach/grasp/lift
   전부 `MoveJPose`(pose→IK→run_joint). lift = "위 config 로 복귀 = 자연 상승".
6. **단일 jaw 발견 (사용자)**: 그리퍼가 좌우 대칭이 아님 — **파란 jaw 고정, 검은 jaw
   (joint7) 만 회전**해 닫힘. 물체는 고정 jaw 쪽으로 눌려 잡힘. → tcp ≠ 파지점 →
   **pinch offset** 필요 (tool-frame, 그리퍼 상수, 큐브 무관).
7. **detection projection fix**: bbox 중심 픽셀 × 윗면 depth **불일치** → 파지 x/y 가
   카메라 쪽 모서리로 편향. → **윗면 픽셀 3D centroid** 로 교체.
8. **gripper URDF 시각화**: 프론트가 arm(TcpState)만 받아 gripper 안 움직임 → motion 이
   gripper rad 를 TcpState 별도 필드로 report.
9. **결과**: 한 번 집힘 — 근데 **운** (사용자: "운좋게 집은거야, 디텍팅도 가끔 이상한데
   찍힘"). 아직 미해결.

**교훈 (내 반복 실수)**: 검증 전에 "불가/미완/워크스페이스 한계" 를 단정함. 진단 도구
(IK 솔버) 가 "불가" 라 해도 물리 증거(손으로 도달)가 상충하면 **도구(seed 수)를 의심**.
[[feedback_verify_solver_not_reality]]

---

## 2. 구현된 것 (파일별, 전부 uncommitted)

### Motion
- `backend_v2/modules/motion/adapters/pybullet.py` — **IK multi-restart**. seeded 1회
  실패 시 random restart 24회 후 seed 에 가장 가까운 해 선택 (motion 연속성). single-seed
  local IK 가 존재하는 해 놓치는 것 방지. `_ik_from_seed` 로 분리.
- `backend_v2/modules/motion/contract.py` — `MOVE_J_POSE` 서비스 + `MoveJPoseRequest`
  (target_position, optional target_quaternion, optional **tool_offset**). TcpState 에
  `gripper_joint_name`/`gripper_rad` 필드 추가.
- `backend_v2/modules/motion/module.py` — `move_j_pose` 핸들러 (pose→IK(현재자세 seed,
  multi-restart)→run_joint). **tool_offset**: IK(target)→자세 R→`target - R·offset`
  재-IK (파지점을 target 에 맞춤, 검증 0.5mm). gripper rad report (units SSOT).

### Task
- `backend_v2/modules/task/steps.py` — `MoveToPose` step (MOVE_J_POSE 호출, optional
  tool_offset). 기존 `MoveTCP`(MoveL)는 남겨둠 (안 쓰임, Cartesian 필요시용).
- `backend_v2/modules/task/tasks/pick_and_place.py` — approach/grasp/lift/place 전부
  `MoveToPose` 로. `ApproachAlongTool`/`RetreatAlongTool` 삭제. **PINCH_OFFSET =
  (0.0, -0.015, 0.0)** (rough URDF 추정, grasp/place 에 적용, 튜닝 필요).

### Detector
- `backend_v2/modules/detector/projection.py` — `object_top_center_base` (윗면 픽셀
  3D centroid). 기존 z_cam_from_depth_bbox/unproject_to_base 는 test 만 씀.
- `backend_v2/modules/detector/module.py` — `object_top_center_base` 로 파지 x/y 산출.

### Frontend
- `frontend_v2/src/api/generated/contract.ts` — TcpState gripper 필드 (offline 재생성).
- `frontend_v2/src/components/scene/RobotLayer.tsx` — gripper joint 를 arm 뒤 append.

### Tests (통과)
- multi-restart 회귀 (`test_kinematics.py`), projection centroid (`test_detector_projection.py`),
  MoveJPose stack 도달 (`test_motion.py`), pick 이 MoveL 안 씀 (`test_pick_and_place.py`),
  gripper report (`test_motion.py`), RobotLayer gripper append.

---

## 3. 열린 문제 / 버그 (다음 세션 우선순위)

### ★ P1 — 모션이 급하고 안 스무스함 (사용자 2026-07-08 관찰)
> "moveJ 가 너무 빨리 움직여. 서치 후 다음 포인트 이동도 급해. 집으러 갈 때도. 빠른 게
> 중요한 게 아냐, 스무스한 느낌이 없어."

- 후보 원인:
  - `run_joint` 의 Ruckig joint max **velocity/accel/jerk** 가 큼 (robot motion.yaml).
    profile 값 확인 — 급가감속.
  - 각 step 이 **독립 MoveJ + 끝에서 정지** → start-stop-start (blending 없음).
  - position-only IK 가 **자세를 매번 자유롭게** 잡아 config 가 크게 튀면 큰 관절 이동
    = 휙 도는 느낌. (seed 근처 해 선택하지만 여전할 수 있음)
- 조사 시작점: `backend_v2/modules/motion/trajectory_runner.py` `run_joint`/`_joint_loop`
  + robot motion.yaml 의 joint profile + MoveJPose 가 넘기는 속도 한계.

### ★ P2 — detection 이 가끔 엉뚱한 데 찍힘
- 사용자: "디텍팅이 이상한데 찍히기도 했어." grasp 정확도의 뿌리 — 여기가 틀리면 나머지
  다 무의미.
- 조사: projection fix(`object_top_center_base`) 의 top-band 선택이 노이즈/테이블에
  민감한지, 아니면 GDINO bbox 자체가 가끔 오검출인지 분리 필요. depth top-percentile
  band(0.010m) 튜닝 여지. `backend_v2/modules/detector/projection.py` + `module.py`.

### P3 — verify_grasp "gripper 상태 미수신"
- grasp 물리 성공해도 `VerifyGrasp` 가 실패 (task 모듈이 gripper raw 캐시 못 함).
- 확인된 것: 프레임워크는 robot-scoped 구독을 wildcard(`stream/motor/*/raw_state`)로
  등록([app.py:280](../backend_v2/framework/runtime/app.py)) → task 모듈이 RAW_STATE
  받아야 정상. scan 모듈은 같은 패턴으로 잘 됨. gripper_index=`r.motors.index(grip)`=6
  (7모터), positions_raw 7개면 유효.
- **미확정**: 왜 캐시가 None 인가. 라이브 로그 필요 — `_on_motor_raw` 가 실제 호출되는지,
  cross-machine(PC task ← 모터 Pi RAW_STATE) 이 도착하는지, robot_id 매칭 되는지.
  `backend_v2/modules/task/module.py:66` `_on_motor_raw`.

### P4 — grasp 신뢰성 (아직 운)
- **pinch offset 값이 rough 추정** `(0,-0.015,0)` — 실측 필요. 큐브 문 자세(토크오프)
  에서 tcp pose vs 큐브 base 차이 = 정확한 tool-frame offset. 미스 방향이 튜닝 정보.
- **jaw-yaw 정렬 미구현** — detector 가 큐브 yaw 를 안 줌. top-face PCA 로 yaw 뽑아
  그리퍼 roll 을 큐브 면에 맞추면 회전된 큐브도 평평한 면 파지. 단 자세 제약이라
  도달성은 cube 90° 대칭 + 후보 search 로 처리 (motion 에 grasp-pose search 필요).
  **작은 큐브가 pinch offset 만으로 잡히면 불필요할 수도 — 테스트가 판단 근거.**

---

## 4. 검증된 사실 (다음 세션이 재유도 말 것)

- **reach 는 문제 아님**: position-only IK 가 큐브 위 z 2~16cm 전부 도달.
- **MoveL(자세 고정 직선)은 SO-101 pick 에 구조적으로 부적합**: 같은 자세로 z 2.3cm 만
  도달, 4.3cm+ 는 seed 400 개로도 FAIL. 높이마다 도달 자세가 바뀜. → 관절 이동만이 답.
- **single-seed PyBullet IK 는 존재하는 해를 놓침**: 14° 자세가 seed 몇 개론 FAIL,
  400개론 OK. → multi-restart 필수 (구현됨).
- **그리퍼 = 단일 가동 jaw** (파란 고정 / 검은 joint7 회전). 파지점 ≠ 기하 중앙 ≠ tcp.
- **tcp 는 gripper_center 에서 +X 0.04m** (URDF tcp_joint). fingertip ≈ tcp (접근축),
  파지 중심은 tcp 에서 고정 jaw 쪽(tool -Y).
- **tool_offset 메커니즘 검증**: pinch point 가 target 에 0.5mm 정렬 (재-IK 1회 근사).

## 5. 설계 결정 (박힌 것)

- pick 접근/파지/승강 = **MoveJPose(관절 공간)**, MoveL 아님. 월드축(Z-lift 포함) 명령 X.
- IK 어댑터 = multi-restart (seeded 우선 + 실패시 restart, seed 최근접 해).
- pinch offset = **그리퍼 상수(tool-frame), 큐브 치수 하드코딩 금지**. jaw 정렬 = 검출
  기하(cube yaw). 둘 다 큐브 크기 독립.
- gripper 관절 상태 = motion 이 TcpState 별도 필드로 report (arm `joints` 에 안 섞음 —
  waypoint 가 `.joints` 를 arm/IK 벡터로 소비하므로 섞으면 MoveJ dof 깨짐).

## 6. 배포 노트

- 변경 노드: **PC** (task, detector) + **모터 Pi** (motion, IK). 카메라 Pi 변경 없음.
- frontend: contract.ts + RobotLayer (재빌드). contract.ts 는 이미 offline 재생성됨.
- `MOVE_J_POSE` 는 frontend 미노출 (task 가 backend 에서 호출).

---

*세션 중 반복된 사용자 피드백: 검증 전 문제 단정 금지 / 한 primitive·한 축에 갇히지 말
것 / 목표(잘 집기)를 놓지 말 것 / 성급히 메모리·"미완" 선언 말 것.*
