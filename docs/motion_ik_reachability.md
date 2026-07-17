# MoveJ(pose) 도달 불가 — motion IK 완전성 이슈 (열린 논의, 2026-07-16)

> ✅ **해소됨 (2026-07-16 밤)**: `PybulletKinematics.ik` 에 **continuation walk**
> 구현 (①seed 1발 → ②위치-only 도달 후 TCP 고정 자세 slerp 걸음 + 스텝 재선형화
> → ③random restart). §3 의 [A] 실패 pose 성공 확인 + plan_pick 52가족 전멸
> 해소 + 실물 첫 완주 (backend.md handoff 2026-07-17 부). §6 의 TRAC-IK/analytical
> 은 불채택 — walk 가 §8.7 우려(빌드/hybrid) 없이 완전성을 실용 수준으로 올림.
> motion.md 로 접는 것은 후속 정리로 남김.
> 관련 메모리: `project_motion_preview_poc`.

---

## 1. 한 줄 요약 (논점)

같은 목표 pose 에 **MoveL 은 도달(OK), MoveJ(pose)는 도달 불가(INFEASIBLE)**. 원인은
PyBullet **수치(numerical, seed 의존) IK 가 도달 가능한 config 를 못 찾는 것** — seed
차이. 프리뷰가 이걸 보여준 건 정직함(실 motion IK 를 그대로 씀).

**사용자 논점 = 이거다**: "프리뷰가 못 간 건 OK (진짜 motion 을 돌린 거니까). 근데
**그럼 motion 자체가 잘못된 것 아니냐**?" 실제 산업로봇은 지금 자세에서 멀다고
도달 가능한 목표를 못 푸는 일이 **없다** (analytical/closed-form IK, seed 자체가
없음). 우리 motion 은 범용 수치 IK 라 sub-industry-standard.

**다음 세션이 결정할 것**: motion 의 IK 를 산업 표준(TRAC-IK / analytical)으로
올릴지 (= 예전에 미룬 "방향 B" = motion 계약/구현 손대기). 프로젝트 원칙 "study 는
industry standard 우선" 과 정확히 맞는 케이스.

---

## 2. 정확한 재현 (수치 박제 — 다른 세션에서 그대로 재현 가능)

### 환경
- robot: `so101_6dof_0`, deployment: `mock` (driver_mode=mock, 하드웨어 불필요).
- mock 부팅 자세 (= `apps/resolve.py::_MOCK_READY_POSE_RAW["so101_6dof"]`):
  - raw: `[2077, 2421, 1461, 2757, 2048, 3083]`
  - deg: `[2.5, 32.8, -51.6, 62.3, 0.0, 91.0]` (J6≈91° = D405 top-down)
  - 이 자세 TCP: `(0.234, -0.038, 0.117)` m
  - (사용자 실측 Robot State 패널 자세 그대로)

### 문제의 목표 (use_orientation=True, 목표지정)
- position: `(0.154, 0.112, 0.097)` m
- quaternion: `[-0.9286, 0.0209, 0.3706, -0.0029]` (= **현재 자세와 동일** — 목표
  RPY 를 현재값에서 안 바꿈. 그래서 MoveL 진단에 `ori=0.0°`)

### 관측 결과 (2026-07-16 16:15 로그, pid=23224)
```
MoveL 진단: start(0.234,-0.038,0.117)->end(0.154,0.112,0.097) ori=0.0° ticks=108 dur=2.20s
preview robot=so101_6dof_0 mode=move_l      use_ori=True → OK         frames=133
preview INFEASIBLE ... mode=move_j_pose ... pos=[0.154,0.112,0.097]
   quat=[-0.9286,0.0209,0.3706,-0.0029] pos_only_ok=True with_ori_ok=False frames=0
preview robot=so101_6dof_0 mode=move_j_pose use_ori=True → INFEASIBLE frames=0
```
- **MoveL → 도달 성공 (133 프레임).** 현재 자세 유지한 채 위치만 이동.
- **MoveJ(pose) → 도달 불가 (0 프레임).** 목표 pose IK 가 현재 seed 로 실패.
- 진단: `pos_only_ok=True` (위치만이면 됨), `with_ori_ok=False` (그 자세는 현재
  seed 로 못 찾음).

---

## 3. seed 의존 증명 (config 는 존재 = motion 이 놓친 것)

아래 스크립트 결과 (mock, so101):
```
[A] MoveJ 방식: IK(target, seed=start)        = None (실패)   ← 200 restart 포함
[B] MoveL 방식: seed 연쇄로 직선 걸어감         = 도달! config 존재
[C] IK(target, seed=목표 근처 config)          = 성공          ← 같은 목표, seed만 바꿈
```
→ **그 자세 config 는 실재한다([B],[C]). start seed 한 방으론 못 찾을 뿐([A]).**
도달 불가가 아니라 **solver 가 놓친 false negative**.

### 재현 스크립트 (backend/ 에서)
```python
import numpy as np
from apps.config import load_robots, DeploymentConfig, DriverMode
from apps.resolve import resolve_host_deps, _MOCK_READY_POSE_RAW
from modules.motion import units
from modules.motor.contract import MotorKind
robots = load_robots(); r = robots['so101_6dof_0']
arm = [m for m in r.motors if m.kind != MotorKind.GRIPPER]
spec = resolve_host_deps('motion_preview', robots,
                         DeploymentConfig(driver_mode=DriverMode.MOCK))['robots']['so101_6dof_0']
kin = spec.kinematics_factory(spec.urdf_path); kin.initialize()
start = units.joints_raw_to_rad(_MOCK_READY_POSE_RAW['so101_6dof'], arm, None)
start_pos, start_quat = kin.fk(start)
target_pos = (0.154, 0.112, 0.097); target_quat = start_quat  # ori=0 케이스
print('[A]', kin.ik(target_pos, target_quat, start))            # None
p0, p1 = np.array(start_pos), np.array(target_pos); seed = list(start); end = None
for i in range(1, 31):
    wp = p0 + (p1 - p0) * (i / 30)
    sol = kin.ik((wp[0], wp[1], wp[2]), target_quat, seed)
    if sol is None: break
    seed = sol; end = sol
print('[B] reached', end is not None)                           # True
print('[C]', kin.ik(target_pos, target_quat, end))              # 성공
```

---

## 4. 근본 원인

- IK = "TCP 목표 pose → 관절각". 우리는 PyBullet `calculateInverseKinematics`
  = **local iterative 수치 IK** (damped least squares). **seed(출발 관절 추측)에서
  가까운 해로 수렴** → seed 멀면 못 찾음.
- MoveJ(pose): `kin.ik(pos, quat, current)` — 현재 자세 seed 한 방
  ([motion_preview/module.py:187](../backend/modules/motion_preview/module.py#L187),
  실 motion 은 [motion/module.py:483](../backend/modules/motion/module.py#L483)).
- MoveL: 직선 샘플마다 **직전 해 seed 연쇄**
  ([trajectory_runner.py:265,299,303](../backend/modules/motion/trajectory_runner.py#L265))
  → 목표를 정답 옆에서 품 → 좁은 basin 도 찾음.
- fallback: seed 실패 시 **200 random restart** 있음
  ([adapters/pybullet.py:183](../backend/modules/motion/adapters/pybullet.py#L183)),
  근데 **rng seed 0 고정**([pybullet.py:180](../backend/modules/motion/adapters/pybullet.py#L180))
  → 매번 같은 200점 → 좁은 basin 못 덮으면 **결정적으로 항상 실패** (재현성 위해
  의도한 것, flaky 아님). 도달 성공 판정 = FK 잔차 <1cm
  ([pybullet.py:238](../backend/modules/motion/adapters/pybullet.py#L238)).

---

## 5. 왜 이게 motion 문제냐 (사용자 논점 = 정당)

- **산업로봇(UR/ABB/KUKA/FANUC)은 analytical(closed-form) IK** — 방정식으로 해를
  전부(≤8개) 직접 구하고 그중 고름. **seed 자체가 없음** → "멀어서 못 찾음" 이
  원천적으로 없음. 도달 가능하면 반드시 찾음.
- 즉 "MoveJ 이 자세 못 감" 은 산업로봇의 진실이 아니라 **우리가 고른 수치 IK 의
  한계**. 프리뷰의 "faithful" 은 *우리 so101+PyBullet 스택엔* 맞지만 산업 팔엔 안 맞음.
- 사용자의 앞선 직관 "MoveJ 는 seed 가 없어야 하는 것 아니냐" = **맞았음** — 산업
  표준 IK 엔 seed 가 없다. (수치 IK 안에서만 "seedless 없음"이 성립.)

---

## 6. 선택지 (다음 세션 결정)

1. **TRAC-IK** (추천) — ROS/산업 사실상 표준 수치 IK. KDL + 비선형 최적화 병렬 →
   "reachable 인데 못 찾음" 거의 제거. 임의 URDF 에 적용. **범용 유지 + 완전성↑.**
2. **analytical IK** — so101 손목 3축이 한 점에서 만나면(spherical wrist, Pieper)
   closed-form 가능. **so101 은 커스텀 팔이라 그 구조인지 확인 필요** (미확인).
3. band-aid: restart 수↑ / rng 다양화 / 스마트 시딩 — 근본 아님.

**제약**:
- 프리뷰는 motion IK 를 **그대로 써야 함** (faithful). 프리뷰만 IK 올리면 실
  move_j 는 여전히 실패 → 프리뷰가 거짓말. **고치면 motion 레벨** (move_j/move_l/
  preview/scan/resolve_reachable 전부 영향 — Kinematics protocol 구현체 교체).
- 이건 "방향 B" (motion 계약/구현 손대기). 지금까지는 "방향 A"(프리뷰만, motion
  무접촉)로 진행했음.

**다음 세션 착수 순서**:
1. 기존에 TRAC-IK / analytical IK 논의·기각된 게 있는지 **docs/motion.md,
   docs/calibration.md, 메모리 먼저 확인** (재론 금지 원칙).
2. 없으면 TRAC-IK 도입 타당성 검토 (Kinematics protocol 뒤에 어댑터 교체 —
   `PybulletKinematics` 옆에 `TracIkKinematics`, factory 주입 지점은
   `apps/resolve.py::_motion_deps` / scan / motion_preview).
3. 결정되면 이 문서를 docs/motion.md § 로 병합.

---

## 7. 이번 세션에서 확정/구현된 것 (배경)

- motion_preview POC 모듈 (plan-only 프리뷰) 구현 완료 — `project_motion_preview_poc`
  메모리 참조. 자세 축 토글(목표지정/위치만), 진단 로그(pos_only/with_ori),
  성공/실패 preview 로그 1줄, mock ready 자세(사용자 실측), dated 로그파일 등.
- 이 IK 이슈는 그 POC 검증 중 발견 — 프리뷰가 **정확히 제 역할(실 motion 동작을
  드러냄)을 해서** motion IK 의 한계가 보인 것. POC 성공 사례이기도 함.

---

## 8. RNG seed 고정 — 의미와 "아직 안 바꾸는" 이유 (2026-07-16 추가)

### 8.1 현 상태
- [pybullet.py:180](../backend/modules/motion/adapters/pybullet.py#L180) `np.random.default_rng(0)`
  — restart fallback 의 **200 샘플이 모든 target 에 동일**. 단 `ik()` 는
  seed=현재로 **1발 먼저** 시도([pybullet.py:172](../backend/modules/motion/adapters/pybullet.py#L172))하고
  실패했을 때만 이 rng 루프로 넘어감 → **rng 영향권 = fallback(어려운 자세)뿐**,
  쉬운 자세는 rng 를 안 탐.
- 효과: 모든 pose 의 성공/실패가 **결정적**. Pose A 항상 실패 / B·C 항상 성공 식.

### 8.2 제안된 변경 (미적용)
- `default_rng(0)` → `default_rng(hash(반올림한 target pose))` 류 **target 파생 seed**.
- 결과: pose 마다 다른 200 샘플 → 지금 사각(A)이 풀릴 수 있으나, 대신 다른
  pose(C)가 **새 사각**이 됨.

### 8.3 이게 뭘 의미하나 (핵심 — 쉬운 말)
```
지금:     A 항상 실패 / B 성공 / C 성공   (전 pose 가 같은 200 시작점)
바꾼 뒤:  A 성공      / B 성공 / C 실패   (pose 마다 다른 200 시작점)
```
- **사각(blind spot)을 없애는 게 아니라 재배치(reshuffle).** 근본(PyBullet 수치
  local 솔버)이 그대로라 "도달 가능한데 못 찾음"은 남고, **실패하는 자세 집합만 바뀜.**
- **복권 버그(2026-07-09)는 재발 안 함**: 그 버그 원인은 전역 rng 의 *history
  의존성*(같은 pose 가 오늘 성공/내일 실패)이지 seed 상수값이 아님. target 파생
  seed 는 요청의 pure function → 같은 pose 는 언제나 같은 결과 = **재현성 유지**
  (디버깅 재현·plan↔execute 일관성·테스트 안정성 그대로).
- 요약: 값싼 개선(특정 실패 자세 구제) 가능 ✅ / 다른 곳 새 실패 가능 ✅ /
  **근본 해결 아님** ✅.

### 8.4 곁다리로 발견한 별개 결함 (같이 기록 — RNG 와 함께 손볼 후보)
- 수용 판정이 **위치잔차만 검사**([pybullet.py:234-238](../backend/modules/motion/adapters/pybullet.py#L234)):
  `target_quaternion` 을 줘도 orientation 을 검증 안 함 → **자세 틀린 해도 통과**
  (false-negative 와 반대로 *너무 관대* — 별건이나 실 결함). 엔진 교체 없이
  PyBullet 안에서 고치는 싼 fix.

### 8.5 왜 아직 안 바꾸나 (결정)
- RNG 변경 = reshuffle 반창고. 단독으로는 "우리가 실제 쓰는 자세들이 더 나은
  쪽에 떨어지느냐"는 **도박** → 실물 데이터 없이 지금 바꿀 근거 약함.
- 완전성(reachable→반드시)은 **솔버 레벨에서만** 얻음: analytical = 이 로봇
  기하로 막힘(§8.6), TRAC-IK = 중간 코드작업 + 고약한 Windows 빌드 + 여전히
  수치(부분 보상, §8.7).
- 그래서 "값싸게 사각 옮기기"와 "비싸게 완전성 사기" **둘 다 지금 당장의 답이
  아님** → **실물 첫 런 데이터로 실패 자세 분포를 본 뒤 결정** (프로젝트 원칙:
  임계·전략은 추측 말고 실물 데이터로 튜닝).

### 8.6 SO-101 손목 기하 확정 (이 세션 — §6 옵션2 "미확인" 해소)
- URDF 판별([so101_6dof.urdf:404-418](../robot/so101_6dof/urdf/so101_6dof.urdf#L404)):
  joint4 축은 `wrist_yaw_back` 원점 통과 z, joint5 축은 x=0.029 에 고정된 채
  뻗음 → **두 축이 skew(~29mm 어긋남), 안 만남.** 세 손목축이 concurrent 도
  parallel 도 아님 → **spherical wrist 아님 → Pieper closed-form 불가.**
- 즉 "numpy 수학만, 빌드 0" 의 깨끗한 analytical 경로는 이 로봇엔 **없음.**

### 8.7 TRAC-IK 통합 현실 (§6 "옆에 factory 주입" 보정)
- Kinematics Protocol([kinematics.py:25](../backend/modules/motion/kinematics.py#L25))이
  `ik` 외에 `self_collision`/`floor_collision`/`obstacle_collision`/
  `set_obstacle_points`/`fk` 를 요구 = **전부 PyBullet 충돌세계 전용, TRAC-IK 엔
  없음**(순수 kinematics 솔버). → **drop-in 교체 불가.**
- 현실적 형태 = **합성(hybrid)**: `ik()` 만 TRAC-IK, 충돌·fk 는 PyBullet 유지
  (엔진 2개 구동) + TRAC-IK 해는 self-collision 미검사라 **PyBullet 재검증 배선
  필요**. TRAC-IK 도 seed 기반 수치라 **완전성 보장은 아님**(성공률↑).

### 8.8 MoveJ(pose) 깊이 = 프리뷰와 실물 동일 (세션 중 오해 정리)
- "실물은 다발 IK, 프리뷰만 단발"은 **오해.** 같은 동작이면 동일 코드:
  MoveJ(pose)는 프리뷰([motion_preview/module.py:187](../backend/modules/motion_preview/module.py#L187))·
  실물([motion/module.py:483](../backend/modules/motion/module.py#L483)) 모두
  `kin.ik` 1회 seed=현재(+내부 200 restart). MoveL 도 양쪽 seed 연쇄 동일.
- 단 실물 PnP **집기**는 MoveJ(pose)를 안 씀 → `resolve_reachable`(seed 연쇄 +
  52 후보 group-major 스캔) + MoveL 로 **더 두꺼운 경로**. 그래서 프리뷰 MoveJ
  실패가 PnP 집기 실패로 **직결되진 않음**(밑바닥 솔버가 같으니 면역은 아님).
  (2026-07-17: resolve 의 budget-major deepening(10,40,200) 폐지 → 그룹당
  walk+40 단일 예산, 선호 순서 엄격 — 근거/수치는 `motion/module.py`
  `_GROUP_IK_BUDGET` 주석. 전멸 후보 36.5s→~8s, 선호 역전 구멍 제거.)
