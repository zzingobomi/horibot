# 캘리브레이션 결과가 실제 로봇에 적용되는 흐름

OMX_F 6DOF 로봇팔의 캘리브레이션은 4개의 산출물을 만든다:

1. **hand_eye** — camera ↔ end-effector 외부 보정
2. **joint_offset** — 모터 zero 위치 오차 (per joint, scalar rad)
3. **link_offset** — URDF link 기하 오차 (per joint, trans 3 + rot 3)
4. **sag_offset** — 자세 의존 중력 처짐 stiffness (per joint, scalar k)

이 문서는 *각 산출물이 실제 로봇 동작에 어디서 어떻게 끼어드는지* 를 코드 흐름 중심으로 정리한다. 수식은 최소화하고, "이 함수가 호출되는 순간 무슨 보정이 들어가는가"에 초점.

전체 그림을 한 줄로:

> **joint_offset**은 raw↔rad 변환 *옆에*, **link_offset**은 URDF *안에*, **sag_offset**은 FK/IK *입출력 양쪽에*, **hand_eye**는 카메라 좌표를 base로 옮길 때 *별도 step*으로 적용된다.

각 산출물은 `robot/calibration/*.npz` 파일에 저장되고, 부팅 시 싱글톤 캐시(`*Coordinates`)에 1회 로드돼 메모리에서 읽힌다. 분산 머신 동기화는 git이 담당 (npz는 git 추적).

---

## 0. 공통 패턴 — *Coordinates 싱글톤

세 offset(joint/link/sag)은 모두 동일한 패턴을 따른다:

```
robot/calibration/<name>.npz  ─ 디스크 (git 추적)
          │
          │  부팅 시 1회 load
          ▼
<Name>Coordinates 싱글톤      ─ 메모리 캐시
          │
          │  코드가 호출 시점에 snapshot()/get_*() 로 읽음
          ▼
   FK/IK/raw↔rad 변환
```

관련 코드:
- [backend/core/coords/joint_coordinates.py](../backend/core/coords/joint_coordinates.py)
- [backend/core/coords/link_coordinates.py](../backend/core/coords/link_coordinates.py)
- [backend/core/coords/sag_coordinates.py](../backend/core/coords/sag_coordinates.py)

COMMIT 시 `commit_absolute()`가 디스크 *overwrite* + 메모리 reload를 한 트랜잭션으로 처리한다. 4종 (joint / link / sag / tool) 모두 같은 API + overwrite semantic — joint 만 cumulative 였던 시절은 [calibration_ux_rewrite.md §6.6](calibration_ux_rewrite.md) 의 Bug A fix 로 통일됨. 같은 PC에서는 즉시 반영되나 다른 머신은 `git pull` + 재시작 필요.

자동 백업: 매 COMMIT 진입 시 [backup.py](../backend/modules/calibration/backup.py) 가 현재 live disk 를 `robot/instances/<id>/calibration/.history/<ts>_pre-commit/` 로 통째 snapshot. `CALIB_BACKUP_LIST` / `CALIB_BACKUP_RESTORE` 서비스 → frontend Rollback 탭에서 picker + 복원. `.history/` 는 git ignored (machine-local).

| 산출물 | 메모리 적용 | PyBullet/URDF 재로드 | 백엔드 재시작 |
|---|---|---|---|
| hand_eye | 즉시 (load_calibration 매 호출마다 디스크 재참조 X — 노드 init에서 1회 load) | X | 재로드 필요 (DetectorNode init) |
| joint_offset | 즉시 (JointCoordinates 메모리 갱신) | X | 불필요 |
| link_offset | 즉시 (LinkCoordinates 메모리 갱신) | **필요** (PyBullet은 URDF 부팅 시 1회 로드) | **필요** |
| sag_offset | 즉시 (SagCoordinates + `solver._reload_sag_cache()`) | X (apply_gravity_sag는 numpy로 매번 계산) | 불필요 |

이 표가 4개 산출물의 적용 메커니즘 차이를 압축적으로 보여준다.

---

## 1. joint_offset — 모터 raw에 더해지는 스칼라

### 무엇이 문제인가

Dynamixel 모터의 raw 값(0~4095)은 **물리 0° 위치(URDF zero)와 정확히 일치한다는 보장이 없다**. 모터 혼(horn) 조립할 때 한 톱니 어긋나면 ~1°, 3D프린트 파트 끼우는 각도가 미세하게 어긋나면 ~0.5° 식의 systematic 오차가 매 joint마다 따라붙는다.

BA(Bundle Adjustment)가 이걸 풀어내서 *"joint i는 raw 값을 URDF rad로 환산한 뒤 +δᵢ를 더해야 진짜 URDF 각도"* 라는 보정값 δ를 5개 추정한다.

### 어디서 적용되는가 — 두 방향

핵심은 **모터 raw ↔ URDF rad 변환 자체가 offset을 흡수**한다는 점. 이게 [backend/core/coords/joint_coordinates.py](../backend/core/coords/joint_coordinates.py)의 두 함수:

```python
# 모터 → URDF (FK 입력으로 쓸 때)
def motor_to_urdf(self, raw, cfg) -> float:
    rad = raw_to_rad(raw, reverse=cfg.reverse)
    return rad + self._offsets.get(cfg.id, 0.0)     # ← + offset

# URDF → 모터 (IK 결과를 명령으로 쓸 때)
def urdf_to_motor(self, rad, cfg, ...) -> int:
    corrected = rad - self._offsets.get(cfg.id, 0.0)  # ← − offset
    return rad_to_raw(corrected, ...)
```

부호가 정확히 반대 — 한쪽은 가산, 한쪽은 차감. 이게 "+δᵢ 만큼 URDF 쪽이 더 크다"는 모델을 일관되게 적용한다.

### 호출 사이트 두 곳

**입력 방향 (state → FK):**

[backend/core/cache/joint_state_cache.py:48](../backend/core/cache/joint_state_cache.py#L48) — `JointStateCache.get_joint_angles_rad()`가 motor state 토픽에서 읽은 raw를 `motor_to_urdf()`로 환산. motion / task / detector / calibration 노드가 현재 EE pose 알고 싶을 때 거치는 단일 진입점.

**출력 방향 (IK → motor cmd):**

[backend/nodes/device/motion_node.py:130](../backend/nodes/device/motion_node.py#L130) — `MotionNode._publish_cmd()`가 trajectory runner의 joint angle을 `urdf_to_motor()`로 환산해 `MOTOR_CMD_JOINT` 토픽으로 publish.

```
[raw 모터] ─raw_to_rad─► [naive rad] ─+offset─► [URDF rad] ─► FK/IK
[URDF rad] ─-offset─► [corrected rad] ─rad_to_raw─► [raw 모터]
```

### COMMIT 동작

[calibration_node.py:320](../backend/nodes/application/calibration_node.py#L320) — BA가 estimate한 **delta**를 cumulative 합산해 디스크 저장 + 메모리 갱신. 즉 같은 PC에서는 **재시작 없이 다음 토픽부터 자동 반영**. delta는 누적이라 BA를 N번 돌리면 N번의 보정이 합쳐진다.

> 주의: `motor_to_urdf`는 `JointStateCache`가 호출하고, `urdf_to_motor`는 `MotionNode`가 호출한다. 분산 모드에서 둘이 다른 머신에 있을 수 있으나 둘 다 git이 동기화한 같은 `joint_offsets.npz`를 본다.

---

## 2. link_offset — URDF link 기하 자체를 수정

### 무엇이 문제인가

OMX_F는 Robotis OMX의 커스텀 변형이라 URDF의 `<joint><origin xyz rpy/>` 값이 물리 조립과 미세하게 어긋날 수 있다. 3D프린트 파트면 ±0.3mm, 도면과 다른 워셔를 끼웠다면 더. 이 오차는 **joint 각도와 무관하게 항상 같은 방향**으로 작용하니까 angle offset이 아니라 *link 기하 자체* 를 보정해야 한다.

확장 BA가 매 joint마다 6개 자유도를 풀어낸다: `link_trans = (dx, dy, dz)` (m), `link_rot = (rx, ry, rz)` (rad rotvec).

### 어디서 적용되는가 — URDF 파일을 만들어버린다

joint/sag offset과 결정적으로 다른 점: link offset은 **함수 호출 시점에 끼어드는 게 아니라 URDF 자체를 patch**한다. PyBullet이 URDF를 한 번 로드한 뒤 link transform이 부팅 후 고정되기 때문 — 매 IK마다 다른 link geometry를 시뮬에 박을 방법이 없다.

해결책: [backend/core/coords/urdf_patcher.py](../backend/core/coords/urdf_patcher.py)

```python
# patch_urdf_text() — 원본 URDF 텍스트를 읽어
for joint_el in root.findall("joint"):
    origin_el = joint_el.find("origin")
    xyz = _parse_xyz(origin_el.get("xyz", "0 0 0"))
    rpy = _parse_xyz(origin_el.get("rpy", "0 0 0"))
    origin_el.set("xyz", _fmt_xyz(xyz + d_trans))   # ← 가산
    origin_el.set("rpy", _fmt_xyz(rpy + d_rot))     # ← 가산
```

즉 `patch_urdf_text` 가 in-memory 에서 수정된 URDF text 를 만들어 PyBullet 이 그걸 로드한다. 원본은 그대로 두니까 git status 가 노이지하지 않고, BA 를 다시 돌릴 때 baseline 이 항상 같다.

> **2026-06-15 업데이트** — 이전에는 `robot/<type>/urdf/.patched/` 디스크에 영속 저장하던 패턴 (git push/pull 시대 잔재). storage_node 도입으로 폐기. 매 부팅 시 in-memory render → tempfile 1회성 (PyBullet `loadURDF` path-only 우회) → load 직후 unlink. 상세 [storage_layer.md §13](storage_layer.md).

### 부팅 시 흐름

[backend/modules/kinematics/adapters/pybullet_kinematics.py](../backend/modules/kinematics/adapters/pybullet_kinematics.py) `initialize()`:

```python
patched_text = patch_urdf_text(self._urdf_path, self._link_offsets)  # ← in-memory string
fd, temp_path = tempfile.mkstemp(suffix=".urdf", prefix="horibot_")
try:
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        f.write(patched_text)
    self._robot = p.loadURDF(temp_path, ...)        # ← PyBullet 가 1회 파싱
finally:
    os.unlink(temp_path)                            # ← load 끝나면 즉시 삭제
```

게다가 link offset은 numpy로 직접 구현한 FK([backend/modules/kinematics/fk_chain.py](../backend/modules/kinematics/fk_chain.py))에도 그대로 전달돼야 한다 — sag 계산이 numpy fk를 쓰는데, PyBullet의 patched URDF와 *수학적으로 동일한 transform*을 써야 두 경로의 EE 위치가 일치한다:

```python
# solver.py:61
self._link_trans_array = np.array(
    [link_offsets.get_trans(i + 1) for i in range(_ARM_DOF)], dtype=np.float64
)
self._link_rot_array = np.array(
    [link_offsets.get_rot(i + 1) for i in range(_ARM_DOF)], dtype=np.float64
)
```

### COMMIT 동작

[calibration_node.py:352](../backend/nodes/application/calibration_node.py#L352) — 디스크 저장 + 메모리 갱신은 즉시지만, **PybulletSolver는 부팅 시 1회 URDF를 로드**하므로 **백엔드 재시작이 필요**하다. COMMIT 응답에 `restart_required: true`로 표시.

> 왜 reload하지 않나: PyBullet API상 동적 URDF 교체는 가능하지만 진행 중인 trajectory / IK seed / joint state cache가 다 깨질 위험이 크고, 캘리브레이션은 자주 하지 않으니 재시작 cost가 더 안전.

---

## 3. sag_offset — FK/IK 호출 시점에 양쪽에서 끼어든다

### 무엇이 문제인가

XL430 모터 그룹(joint 1~3)이 11V로 운용 중인데 정격 하한(10V) 근처라 토크 마진이 작다. joint 2, 3는 중력 부하가 큰 자세에서 **commanded 각도와 실제 도달 각도가 다르다** — 모터가 명령한 각도까지 못 올라가고 sag한다.

이 sag는 자세 의존적: EE가 base에서 멀고 위에 있을수록 joint 2/3 토크가 크고 sag도 크다. 그러니까 *고정된 offset이 아니라* 자세마다 다른 값.

모델 ([fk_chain.py:172](../backend/modules/kinematics/fk_chain.py#L172) `gravity_torque_lumped`):

```
sag_offset_J = k_J × τ_J
   where τ_J = (ee_pos − joint_origin) × gravity_dir · joint_axis
```

EE에 lumped mass가 있다고 가정하고 J2/J3 회전축에 걸리는 중력 토크를 계산. `k_J`(스칼라 stiffness)는 BA가 추정. J1/J4/J5는 sag가 noise 수준이라 모델에서 제외.

### 어디서 적용되는가 — FK는 정방향, IK는 역방향

여기가 흥미로운 부분이다. FK와 IK가 정확히 **반대 방향**으로 sag를 적용한다:

#### FK: commanded → actual (sag 추가)

[solver.py:139](../backend/modules/kinematics/registry.py#L139) — `_commanded_to_actual()`:

```python
def fk(self, joint_angles):
    actual = self._commanded_to_actual(joint_angles)   # ← sag 더하기
    self._set_joint_positions(actual)
    return self._get_ee_state()
```

해석: 모터 encoder가 읽어주는 각도(=commanded)는 "내가 이 각도로 명령했다"는 값이지 실제 link 끝 각도가 아니다. URDF는 link 끝 기하를 모델링하니까 FK에 넣기 전에 sag 만큼 더해서 actual을 만들어야 한다.

```python
# apply_gravity_sag — fk_chain.py:193
out[1] += k[0] * tau_J2   # J2
out[2] += k[1] * tau_J3   # J3
```

#### IK: actual → commanded (sag 빼기)

[solver.py:149](../backend/modules/kinematics/registry.py#L149) — `_actual_to_commanded()`:

```python
def ik(self, target_position, ...):
    # 1. seed: commanded → actual (PyBullet IK는 actual 공간에서 푼다)
    current_actual = self._commanded_to_actual(current_joint_angles)
    ...
    # 2. PyBullet IK: target_position을 만족하는 actual angle 계산
    result = p.calculateInverseKinematics(...)
    actual_angles = list(result[:n])
    ...
    # 3. actual → commanded: 모터에 명령할 값으로 역변환
    return self._actual_to_commanded(actual_angles)
```

해석: PyBullet IK가 "이 자세면 EE가 target에 도달한다"는 **actual** angle을 준다. 하지만 모터에 그 각도를 명령하면 sag만큼 처지니까, 미리 sag를 *빼서* commanded로 변환해 명령해야 sag 후 실제 target에 도달한다.

[fk_chain.py:229](../backend/modules/kinematics/fk_chain.py#L229) `actual_to_commanded` — 1차 근사:

```python
# 정확한 모델: actual = commanded + sag(commanded)
# 1차 근사: sag(commanded) ≈ sag(actual)   (sag 변화가 작아서 OK)
# → commanded ≈ actual − sag(actual)
```

이 1차 근사 오차는 sag ~2°에서 < 0.05° 수준이라 fixed-point iteration 없이도 충분.

### 정리: 두 함수가 대칭

```
fk:  encoder_reading ──+sag──► URDF angle ──PyBullet FK──► EE pose
ik:  target EE ──PyBullet IK──► URDF angle ──−sag──► motor command
```

부호가 반대인 이유: FK는 "물리적으로 일어나는 현상"을 모델링(commanded면 실제 actual이 됨), IK는 "모터에 뭘 명령해야 원하는 EE가 나오는가"를 계산(역보정).

### COMMIT 동작

[calibration_node.py:385](../backend/nodes/application/calibration_node.py#L385) — 디스크 저장 + 메모리 갱신 + `solver._reload_sag_cache()` 호출. PybulletSolver의 `apply_gravity_sag`는 매 FK/IK 호출마다 numpy로 새로 계산하니까 **재시작 불필요, 즉시 반영**. link offset과 달리 URDF에 박힌 게 아니다.

---

## 4. hand_eye — 카메라 측정을 base 좌표로 옮길 때

### 무엇이 문제인가

D405 카메라는 그리퍼에 마운트돼 있어 EE와 함께 움직인다. 카메라가 "물체가 (xn, yn, Z_cam) 픽셀/깊이에 보인다"고 했을 때, 이게 로봇 base 좌표로는 어디인가? — 그러려면 카메라 frame ↔ 그리퍼 frame의 6DOF 변환이 필요하다. 이게 `T_cam2gripper = (R, t)`.

joint/link/sag와 달리 **운동학에 들어가지 않는다**. 단지 카메라 측정값을 다른 frame으로 옮기는 후처리.

### 어디서 적용되는가 — detector의 단일 step

[backend/nodes/application/detector_node.py:120](../backend/nodes/application/detector_node.py#L120):

```python
# 1. YOLO가 픽셀 (cx, cy) 찾음
# 2. cv2.undistortPoints로 normalize → (xn, yn)
# 3. 현재 EE pose 호출 (MOTION_GET_TCP → 이 안에 sag 보정이 이미 포함됨)
R_be = _quat_to_rot(quat)   # end-effector → base
t_be = np.array(pos)

# 4. hand-eye matrix
R_ce = self._calib.hand_eye.R       # camera → end-effector
t_ce = self._calib.hand_eye.t.flatten()

# 5. base 평면 Z=0 제약으로 depth Z_cam 역산
R_total = R_be @ R_ce               # camera → base
t_total = R_be @ t_ce + t_be
Z_cam = -t_total[2] / (...)

# 6. 카메라 → base
obj_in_cam = np.array([xn * Z_cam, yn * Z_cam, Z_cam])
obj_in_ee = R_ce @ obj_in_cam + t_ce
obj_in_base = R_be @ obj_in_ee + t_be
```

핵심은 **`R_be @ R_ce` chain**. 카메라 frame → EE frame (hand-eye) → base frame (FK from motor state). FK 자체에 joint/link/sag 보정이 이미 다 반영돼 있으니, hand_eye는 그 위에 한 단계 더 얹는 모양.

### 라이브 포인트클라우드에서의 적용

[frontend/src/components/workspace3d/3d/PointCloudLayer.tsx](../frontend/src/components/workspace3d/3d/PointCloudLayer.tsx)에서는 백엔드가 camera frame xyz를 그대로 publish하고, 프론트에서 `<group position quaternion>` 부모 transform으로 `cameraMatrix = tcpMatrix · handEyeMatrix`를 적용. 같은 수학이지만 GPU에 떠넘긴 것.

### COMMIT 동작

[calibration_node.py:308](../backend/nodes/application/calibration_node.py#L308) — `hand_eye.npz` 저장. DetectorNode는 init에서 `load_calibration()` 1회 호출하므로 **재시작 필요** (저장 즉시 디스크에는 있지만 메모리 상태가 옛 값). 분산 모드에서는 PC 측 detector만 재시작하면 됨.

---

## 5. 전체 적용 흐름 — 한 번의 motor command가 끝까지 가는 길

명령 사이클을 따라가면 4개 보정이 어디서 들어오는지 자연스럽게 보인다.

### A. 사용자가 "EE를 (x, y, z)로 이동"이라고 함

```
WebSocket → MotionNode._srv_move_l(target)
   │
   ▼
MotionCommand가 Kinematics.ik(target, current=encoder reading) 호출
   │
   ├─ current(encoder reading=commanded) → actual로 변환  ◄── sag (+ joint_offset은 이미 적용된 상태)
   │     ※ current는 JointStateCache.get_joint_angles_rad → motor_to_urdf 거친 값
   │       이 시점에서 joint_offset 이미 적용
   ├─ PyBullet IK가 patched URDF (link_offset 적용된)에서 target 도달 angle 계산  ◄── link_offset
   └─ 결과(actual angle) → commanded로 sag 역보정  ◄── sag
   │
   ▼
trajectory_runner가 100Hz로 보간하며 _publish_cmd 호출
   │
   ▼
MotionNode._publish_cmd(angles_rad)
   │
   ├─ JointCoordinates.urdf_to_motor(angle)  ◄── joint_offset (차감)
   └─ rad_to_raw로 0..4095 변환
   │
   ▼
MOTOR_CMD_JOINT 토픽
   │
   ▼
MotorNode가 Dynamixel에 raw 값 쓰기
```

### B. 모터가 움직인 결과를 다시 읽음

```
MotorNode가 20Hz로 encoder reading raw publish (MOTOR_STATE_JOINT)
   │
   ▼
JointStateCache._on_motor_state(raw 저장)
   │
   ▼
누군가 get_joint_angles_rad() 호출 (motion/task/detector/calibration이 EE pose 알고 싶을 때)
   │
   ├─ raw_to_rad
   └─ JointCoordinates.motor_to_urdf  ◄── joint_offset (가산)
   │
   ▼
Kinematics.fk(angles)
   │
   ├─ commanded → actual 변환  ◄── sag (가산)
   └─ patched URDF로 PyBullet FK  ◄── link_offset
   │
   ▼
EE pose (R, t) in base frame
```

### C. 카메라가 본 물체를 base 좌표로 옮김

```
DetectorNode._handle_detect()
   │
   ├─ FrameCache.get_frame() → YOLO → 픽셀 (cx, cy)
   ├─ cv2.undistortPoints → 정규화 (xn, yn)  ◄── intrinsic
   ├─ MOTION_GET_TCP 호출 → 위 B의 흐름으로 EE pose (R_be, t_be)
   │     이 안에 joint/link/sag 보정이 모두 녹아 있음
   └─ hand_eye (R_ce, t_ce) 곱해서 base 좌표 산출  ◄── hand_eye
```

---

## 6. 왜 이렇게 4개로 쪼갰나

캘리브레이션을 한 덩어리(예: "5DOF 전체 보정 행렬")로 두지 않고 4종으로 분리한 이유:

| 분리 | 이유 |
|---|---|
| joint_offset vs link_offset | joint_offset은 *각도* 오차(혼 조립 한 톱니), link_offset은 *기하* 오차(링크 길이/방향). 물리 원인이 다르고 자세 의존성도 다름. 합치면 BA의 자유도가 모호해져 식별 불가능 (gauge freedom) |
| 정적 offset vs sag | joint/link offset은 자세 무관, sag는 자세 의존. 같은 자유도로 묶으면 자세별 BA 잔차가 둘 사이에서 누설 |
| hand_eye 분리 | 운동학(FK/IK)과 무관. 카메라 measurement 후처리. 운동학 캘 따로, 카메라 캘 따로 진행해도 됨 |

각 보정이 **다른 메커니즘**으로 적용되는 것도 이 분리에서 자연스럽게 따라온다 — joint_offset은 raw↔rad 옆에 붙고, link_offset은 URDF에 박히고, sag는 매번 계산하고, hand_eye는 좌표 변환만.

---

## 7. 디버깅 노트

- 어느 보정이 적용 중인지 확인하려면 부팅 로그에 다음 줄들을 본다:
  - `joint_offsets 적용: {1: 0.012, ...}` ([joint_coordinates.py:57](../backend/core/coords/joint_coordinates.py#L57))
  - `link_offsets 적용: N joints` ([link_coordinates.py:54](../backend/core/coords/link_coordinates.py#L54))
  - `sag_offsets 적용: J2=..., J3=...` ([sag_coordinates.py:57](../backend/core/coords/sag_coordinates.py#L57))
  - `patched URDF 로드: .../omx_f.urdf` ([solver.py:57](../backend/modules/kinematics/registry.py#L57))
- `JointStateCache.get_joint_angles_rad_uncorrected()`로 offset 적용 *전* 값도 볼 수 있다 (캘 진단용)
- sag만 disable하고 싶으면 `sag_offsets.npz`를 지우거나 빈 값으로 commit. `_sag_enabled = False`가 되면 fk/ik가 no-op
