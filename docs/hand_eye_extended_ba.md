# Hand-Eye 확장 BA — 원리와 코드

> σ_rot 1.5° / σ_t 17mm floor → 확장 BA(link offset, §1~§14)로 1.30°/9.3mm →
> 물리 sag 모델(§16)로 **0.65°/7.94mm**까지 내림. 수식 최소, **실제 코드 스니펫 + 줄별 설명** 중심.

---

## 1. 무엇이 문제였나

OMX_F의 Hand-Eye 캘리브레이션 결과가 **σ_rot ≈ 1.5° / σ_t ≈ 17mm**에서
정체. 자세 32개까지 캡처해도 안 떨어짐.

캘 σ가 의미하는 건 _"체커보드는 실제로 한 위치에 있는데, 캘 결과로 자세
마다 예측한 체커보드 위치가 얼마나 흩어지나"_. σ가 작을수록 모든 자세에서
일관된 EE 위치를 잡는다는 뜻 → detector pick&place, TSDF 정밀도에 직결.

TSDF/ICP 깔끔하게 돌리려면 σ_rot < 1° / σ_t < 10mm 필요.
**floor가 모델 한계라는 게 의심스러웠다.**

---

## 2. 진단 — 코드로 어떻게 알아냈나

기존 BA는 [bundle_adjust.py:81](../backend/modules/calibration/bundle_adjust.py)
의 `bundle_adjust_hand_eye()` — **11자유도** (joint_offset 5 + R/t 6).

11자유도가 진짜 한계인지 확인하려면 *같은 데이터*에 모드 4가지를 돌려
σ 비교 진단 실시.

핵심 부분 (4가지 시나리오 호출):

```python
# baseline=0 (디스크 offset 무시) — angles_zero
# baseline=현재 commit — angles_current (= angles_zero + JointCoordinates._offsets)
def run(label, angles, R_seed, t_seed, estimate):
    ba = bundle_adjust_hand_eye(
        joint_angles_per_pose=angles,
        R_target2cam=R_tc_list, t_target2cam=t_tc_list,
        X_init=(R_seed, t_seed), fk_fn=fk,
        estimate_joint_offsets=estimate,   # ← 핵심: 11자유도 ↔ 6자유도
    )
    sigma_rot = float(np.sqrt(np.mean(ba.residual_rot_deg**2)))
    sigma_t   = float(np.sqrt(np.mean(ba.residual_t_mm**2)))
    print(f"[{label}] σ_rot={sigma_rot} σ_t={sigma_t} offset={...}")

run("(1) est=True ", angles_zero,    R_seed_zero, t_seed_zero, True)   # 11 DOF
run("(2) est=False", angles_zero,    R_seed_zero, t_seed_zero, False)  # 6 DOF
run("(3) est=True ", angles_current, R_seed_cur,  t_seed_cur,  True)
run("(4) est=False", angles_current, R_seed_cur,  t_seed_cur,  False)
```

결과 표:

| 시나리오                   | σ_rot     | σ_t        | 의미                        |
| -------------------------- | --------- | ---------- | --------------------------- |
| (1) baseline=0, est=ON     | 2.05°     | 19.7mm     | joint_offset 흡수 효과 있음 |
| (2) baseline=0, est=OFF    | 3.45°     | 24.9mm     | 아무 보정 없는 raw 한계     |
| (3) baseline=현재, est=ON  | **1.50°** | **16.9mm** | 한 라운드 commit 후 floor   |
| (4) baseline=현재, est=OFF | 1.50°     | 17.1mm     | (3)과 같음                  |

결정적 두 줄:

- (1) vs (2): joint_offset이 진짜 systematic 흡수 (3.45→2.05, 1.4° 차이 = 진짜 효과)
- **(3) ≈ (4)**: 현재 baseline에서는 est ON/OFF가 같음 → **joint_offset 자유도가 이미 소진**

→ 알고리즘 문제 아니라 **모델 자유도 부족**.

---

## 3. 진짜 원인은 URDF의 link 기하학

1차 commit 결과를 보면 J2/J3 offset이 **+5.75° / +3.67°로 같은 방향, 비슷한
크기**. horn 오차라면 모터마다 독립이라 _같은 방향으로 함께 어긋날 일이 거의 없음_.
이건 다른 원인의 signature:

- **URDF link 길이 미스매치** — 3D프린트 부품 실측 vs URDF 수치 불일치
- **link frame 기울기** — 조립 시 약간 비스듬, URDF는 rpy="0 0 0" 가정
- **중력 처짐** — XL430이 11V(정격 하한)에서 동작, joint 2/3 토크 크면 sag

이 셋은 *joint 회전축*이 아니라 _link 본체의 transform_ 오차. joint_offset
하나당 1자유도는 "모든 자세에 일정한 보정"인데, link 오차는 자세에 따라
EE 위치에 다르게 영향 → 11자유도가 그걸 어거지로 흡수하다 limit 도달.

---

## 4. 해결 — link offset을 BA 변수로

URDF의 각 joint origin은:

```xml
<joint name="joint2" type="revolute">
  <origin rpy="0 0 0" xyz="0 0 0.0635"/>   <!-- 이 두 값을 변수화 -->
  <axis xyz="0 1 0"/>
  ...
</joint>
```

`xyz` 3개 + `rpy` 3개 = joint마다 6자유도 추가. 5 joint × 6 = **30 자유도 추가** → 총 **41자유도**.

### 4a. numpy FK chain — PyBullet 우회

PyBullet은 URDF 로드 후 transform 변경 불가. 근데 BA는 link_offset을
*변수로 매 iteration마다 다른 값으로 평가*해야 함.

[fk_chain.py](../backend/modules/kinematics/fk_chain.py) — URDF chain을
numpy 행렬 곱으로 직접 구현:

```python
# URDF에서 추출한 상수 (motor id 1~5와 일치)
JOINT_ORIGINS = np.array([
    [-0.01125, 0.0, 0.034],     # joint1 (link0→link1)
    [0.0, 0.0, 0.0635],          # joint2
    [0.0415, 0.0, 0.11315],      # joint3
    [0.162, 0.0, 0.0],            # joint4
    [0.0287, 0.0, 0.0],           # joint5
])
JOINT_AXES = np.array([
    [0, 0, 1],  [0, 1, 0], [0, 1, 0], [0, 1, 0], [1, 0, 0],
])
EE_ORIGIN = np.array([0.09193, -0.0016, 0.0])  # link5→ee fixed


def fk_chain(joint_angles, link_trans=None, link_rot=None):
    """link_trans/link_rot이 BA 변수로 들어가는 entry point."""
    T = np.eye(4)
    for i in range(5):
        # (1) joint i의 origin transform — URDF base + BA delta
        T_o = np.eye(4)
        T_o[:3, :3] = rotvec_to_R(link_rot[i])    # ← BA가 푸는 회전 보정
        T_o[:3, 3]  = JOINT_ORIGINS[i] + link_trans[i]  # ← 위치 보정
        T = T @ T_o
        # (2) joint i 회전 (revolute axis만큼)
        T_r = np.eye(4)
        T_r[:3, :3] = axis_angle_to_R(JOINT_AXES[i], joint_angles[i])
        T = T @ T_r
    # (3) fixed end_effector_joint
    T_ee = np.eye(4); T_ee[:3, 3] = EE_ORIGIN
    Tee = T @ T_ee
    return Tee[:3, :3], Tee[:3, 3]
```

`link_trans=None / link_rot=None`이면 zero로 처리 → URDF 원본 그대로 FK.
BA에서는 `link_trans/link_rot`이 매번 다른 변수 값으로 들어감.

### 4b. 확장 BA — bundle_adjust_hand_eye_extended

[bundle_adjust.py](../backend/modules/calibration/bundle_adjust.py)에 신규 추가.
변수 layout:

```python
# 변수 layout (총 41):
#   [0:5]    joint_offset (rad)
#   [5:20]   link_translation (5×3, m)   ← 신규
#   [20:35]  link_rotation (5×3, rad)    ← 신규
#   [35:38]  rod (cam2gripper)
#   [38:41]  t (cam2gripper, m)

def unpack(x):
    return (
        x[:5],                        # joint_offset
        x[5:20].reshape(5, 3),         # link_translation
        x[20:35].reshape(5, 3),        # link_rotation
        x[35:38],                      # rod
        x[38:41],                      # t
    )
```

핵심 함수 — *체커보드는 한 위치*라는 제약을 잔차로 표현:

```python
def compute_T_target_in_base(x):
    """현재 변수 값으로 모든 포즈의 체커보드 위치 계산."""
    offset, link_t, link_r, rod, t_x = unpack(x)
    R_x = cv2.Rodrigues(rod)[0]
    T_x = make_T(R_x, t_x)          # T_cam2gripper (hand-eye)
    out = []
    for i in range(N):
        # joint angle에 offset 더한 후 FK (link 변형 반영)
        R_gb, t_gb = fk_chain(angles_arr[i] + offset, link_t, link_r)
        T_gb = make_T(R_gb, t_gb)    # T_gripper2base
        # T_target2base = T_gb @ T_cam2gripper @ T_target2cam (PnP 결과)
        out.append(T_gb @ T_x @ T_tc_list[i])
    return out


def residual(x):
    """모든 포즈의 T_target2base가 *평균*과 얼마나 다른지 = 흩어짐."""
    offset, link_t, link_r, _, _ = unpack(x)
    T_list = compute_T_target_in_base(x)
    positions = np.array([T[:3, 3] for T in T_list])
    mean_pos = positions.mean(axis=0)                       # 모든 포즈의 평균 위치
    mean_R   = _mean_rotation([T[:3,:3] for T in T_list])    # SVD chordal mean

    res = np.empty(6 * N + n_off + n_lt + n_lr)
    for i, T in enumerate(T_list):
        # 회전 편차 (axis-angle 형태)
        R_dev = T[:3,:3] @ mean_R.T
        rod_dev, _ = cv2.Rodrigues(R_dev)
        res[6*i : 6*i+3]   = rod_dev.flatten()              # 잔차[0:3]
        # 위치 편차
        res[6*i+3 : 6*(i+1)] = T[:3, 3] - mean_pos          # 잔차[3:6]

    # regularization 잔차 (다음 섹션)
    res[6*N : 6*N + n_off]                   = joint_offset_reg * offset
    res[6*N + n_off : 6*N + n_off + n_lt]    = link_trans_reg  * link_t.flatten()
    res[6*N + n_off + n_lt :]                = link_rot_reg    * link_r.flatten()
    return res

# scipy LM이 잔차 norm 최소화로 x를 푼다
result = least_squares(residual, x0, method="lm", ...)
```

**왜 mean 기준 잔차?** 체커보드의 "진짜 위치"를 변수로 두면 X(hand-eye)와
T_b(보드 위치)가 곱 형태로 entwine돼서 BA가 잘못된 minimum에 빠짐(gauge
freedom). 매 iter에서 *현재 추정의 평균*을 진짜 위치로 가정하면 그 자유도가
사라지고 LM이 안정적으로 수렴. 이게 hand_eye.py 주석에 적힌 'mean-based BA'.

결과 — 같은 32포즈에서:

|              | σ_rot     | σ_t       |
| ------------ | --------- | --------- |
| 11자유도     | 1.50°     | 16.9mm    |
| **41자유도** | **1.30°** | **9.3mm** |

σ_t가 거의 절반. TSDF GOOD threshold(10mm) 진입.

---

## 5. Gauge freedom — 왜 regularization이 필요한가

자유도 늘릴 때 위험: **link 길이 줄이고 hand-eye t 늘리면 같은 EE 위치**가
나옴. BA가 어느 값이 맞는지 못 정하고 어느 쪽으로든 흘러감.

증거 — regularization 없이 풀었더니:

```
joint2 link_translation dx = -60.97mm    ← 원본 link 길이 113mm의 절반!
joint2 joint_offset    = +22.83°          ← 비정상적으로 큼
σ_rot = 1.40°, σ_t = 9.36mm               ← fit은 좋음
```

fit은 좋은데 _값 자체는 의미 없음_. 다른 자세에 generalize 안 함.

해결 — 잔차에 _penalty 항_ 추가. 변수가 작은 값에 머물도록.

```python
# bundle_adjust.py — residual() 끝부분
res[6*N : 6*N + n_off]                = joint_offset_reg * offset       # weight=0.5
res[6*N + n_off : 6*N + n_off + n_lt] = link_trans_reg  * link_t        # weight=1.0
res[6*N + n_off + n_lt :]             = link_rot_reg    * link_r        # weight=1.0
```

`least_squares`는 잔차의 합을 최소화 → 이 항이 크면 그 변수도 작게 유지하려 함.
**weight 의미:** `link_trans_reg=1.0`이면 link_t가 0.01m(=10mm)일 때 잔차에
0.01 기여 → 데이터 잔차(보통 ~0.01 m) 비교해서 같은 수준. 즉 _10mm 부근에서
중립_. 그보다 큰 값을 쓰려면 데이터 fit이 추가로 그만큼 좋아져야 함.

weight 튜닝 실험:

| `link_trans_reg` | 결과                                                    |
| ---------------- | ------------------------------------------------------- |
| 10 (너무 강)     | link 모두 ≈0, BA가 joint_offset에 다시 흡수 (J2 +14.4°) |
| 5                | link ±3mm 정도, σ_t 14.9mm                              |
| 1                | link ±15mm, σ_t **9.3mm** ← sweet spot                  |
| 0 (없음)         | link 60mm 폭주, σ_t 9.4mm지만 의미 없음                 |

---

## 6. URDF patch — 변경 결과를 production에 어떻게 적용하나

BA가 풀어준 link_offset을 production code (motion/detector/task)에도 반영해야
함. 이들은 `PybulletSolver`로 FK/IK를 푸는데 PyBullet은 URDF 로드 후 변경 불가.

해결: **URDF 텍스트를 patch한 파일을 따로 만들고 PyBullet에 그걸 로드**.

[urdf_patcher.py](../backend/core/coords/urdf_patcher.py) 핵심:

```python
def patch_urdf_text(source_urdf_path, offsets, joint_id_map=None):
    """원본 URDF를 읽어 link_offsets patch한 텍스트 반환."""
    tree = ET.parse(str(source_urdf_path))
    root = tree.getroot()

    # (1) mesh 상대경로 → 절대경로 (patched URDF가 다른 폴더로 가니까)
    urdf_dir = src.parent.resolve()
    for mesh_el in root.iter("mesh"):
        filename = mesh_el.get("filename")
        if filename and not filename.startswith(("package://","file://","/")):
            abs_path = (urdf_dir / filename).resolve()
            mesh_el.set("filename", str(abs_path).replace("\\", "/"))

    # (2) joint origin patch
    for joint_el in root.findall("joint"):
        name = joint_el.get("name")
        if name not in joint_id_map: continue            # joint1~joint5만
        jid = joint_id_map[name]
        origin_el = joint_el.find("origin")

        d_trans = offsets.get_trans(jid)                  # 예: J2 [-0.02861, 0.00041, 0]
        d_rot   = offsets.get_rot(jid)                    # 예: J2 [-0.0108, 0.0035, 0]

        xyz = _parse_xyz(origin_el.get("xyz", "0 0 0"))
        rpy = _parse_xyz(origin_el.get("rpy", "0 0 0"))
        origin_el.set("xyz", _fmt_xyz(xyz + d_trans))     # 원본 + delta
        origin_el.set("rpy", _fmt_xyz(rpy + d_rot))

    return ET.tostring(root, encoding="unicode")
```

`(1)` mesh 절대경로화가 _중요_ — patched URDF가 `.patched/omx_f.urdf`에 저장되는데,
mesh가 원본의 `../../meshes/...`라면 상대 위치가 어긋나 PyBullet이 mesh 못 찾음.

`(2)` `xyz + d_trans`는 그냥 가산. `rpy + d_rot`는 _small-angle 가정_. URDF rpy는
ZYX 오일러 (`R = Rz·Ry·Rx`), `d_rot`는 BA의 rotation vector. 다른 표현이지만
각이 작으면 (<5°) 차이 무시 가능 (실제 v3 결과 최대 0.85°). 정확한 변환이
필요해지면 별도 함수.

저장 — [urdf_patcher.py](../backend/core/coords/urdf_patcher.py)의 `write_patched_urdf`:

```python
def write_patched_urdf(source_urdf_path, offsets, ...):
    src = Path(source_urdf_path)
    out = src.parent / ".patched" / src.name   # robot/urdf/omx_f/.patched/omx_f.urdf
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(patch_urdf_text(src, offsets), encoding="utf-8")
    return out
```

`.patched/`는 [.gitignore](../.gitignore)에 추가 → push 안 됨. 머신마다 자체 생성.

---

## 7. 백엔드 통합 — HandEyeCalibration 분기

`bundle_adjust_hand_eye_extended()`를 만들어둬도 호출돼야 의미가 있다.
[hand_eye.py](../backend/modules/calibration/hand_eye.py)의 `compute_with_diagnostics()`
가 mode 따라 기존 BA / 확장 BA로 분기.

### 7a. import 추가 + 헬퍼 함수

```python
# hand_eye.py 맨 위
from .bundle_adjust import (
    BundleAdjustExtendedResult,           # ← 확장 BA 결과 타입
    BundleAdjustResult,                    # 기존
    FkFn,
    bundle_adjust_hand_eye,
    bundle_adjust_hand_eye_extended,       # ← 확장 BA 함수
)
```

기존 `_run_ba_lists()` / `_multiseed_ba_lists()` 패턴 그대로 확장 버전 추가:

```python
@staticmethod
def _run_ba_extended_lists(*, ja_list, R_tc_list, t_tc_list, seed):
    """확장 BA 한 번 실행 — fk_fn 인자 없음 (내부에서 numpy fk_chain 호출)."""
    try:
        return bundle_adjust_hand_eye_extended(
            joint_angles_per_pose=[list(a) for a in ja_list],
            R_target2cam=R_tc_list,
            t_target2cam=[np.asarray(t).reshape(3) for t in t_tc_list],
            X_init=(seed.R_cam2gripper, seed.t_cam2gripper),
        )
    except Exception as e:
        logger.exception("확장 BA 실패: %s", e)
        return None


def _multiseed_ba_extended_lists(self, *, ja_list, R_gb_list, t_gb_list,
                                  R_tc_list, t_tc_list):
    """TSAI/PARK/DANIILIDIS 3 seed로 확장 BA 실행, cost 최소 채택."""
    best_ba, best_seed_name = None, None
    for method in _COMPARE_METHODS:
        R, t = cv2.calibrateHandEye(R_gb_list, t_gb_list, R_tc_list, t_tc_list,
                                     method=method)
        seed = HandEyeResult(R_cam2gripper=R, t_cam2gripper=t,
                              method=_METHOD_NAMES[method])
        ba = self._run_ba_extended_lists(ja_list=ja_list, R_tc_list=R_tc_list,
                                          t_tc_list=t_tc_list, seed=seed)
        if ba is None or not ba.success: continue
        if best_ba is None or ba.cost < best_ba.cost:
            best_ba, best_seed_name = ba, _METHOD_NAMES[method]
    return best_ba, best_seed_name
```

cv2 seed 3개로 돌리는 이유 — BA가 nonlinear라 seed 따라 다른 local minimum.
cost 최소를 채택하면 robust.

### 7b. compute_with_diagnostics 분기

기존 메서드에 `use_extended_ba` 인자 추가:

```python
def compute_with_diagnostics(self, *, fk_fn, arm_motor_cfgs, joint_limits_rad,
                              estimate_joint_offsets=True,
                              use_extended_ba=False):   # ← 신규
    """
    use_extended_ba=True면 확장 BA(41 DOF) 사용.
    fk_fn 대신 fk_chain.fk_chain 내부 호출.
    """
    ...
```

BA 호출 분기:

```python
# ── 2. 1차 BA (multiseed) — outlier 식별용 ────────────────
ba_first: BundleAdjustResult | BundleAdjustExtendedResult | None
if use_extended_ba:
    ba_first, ba_first_seed = self._multiseed_ba_extended_lists(
        ja_list=ja_list, R_gb_list=R_gb_list, t_gb_list=t_gb_list,
        R_tc_list=R_tc_list, t_tc_list=t_tc_list,
    )
else:
    ba_first, ba_first_seed = self._multiseed_ba_lists(
        ja_list=ja_list, R_gb_list=R_gb_list, t_gb_list=t_gb_list,
        R_tc_list=R_tc_list, t_tc_list=t_tc_list,
        fk_fn=fk_fn, estimate_joint_offsets=estimate_joint_offsets,
    )
```

`Union 타입`을 쓰는 이유: 두 BA 결과 공통 인터페이스(residual_rot_deg,
residual_t_mm, R_cam2gripper, t_cam2gripper, joint_offset_rad)는 동일.
**link_trans_m, link_rot_rad는 BundleAdjustExtendedResult만 가짐** → isinstance로 분기.

### 7c. 결과 처리 (joint_offset + link_offset 추출)

outlier 자동 제거 후 ba_final 결과에서 변수 추출:

```python
# ── 5. 최종 X / 잔차 / σ 결정 ────────────────────────────
joint_offset_rad      = np.zeros(len(arm_motor_ids))
joint_offsets_estimated = False
link_trans_delta = np.zeros((5, 3))
link_rot_delta   = np.zeros((5, 3))
link_offsets_estimated = False

if ba_final is not None and ba_final.success:
    final_R = ba_final.R_cam2gripper
    final_t = ba_final.t_cam2gripper.reshape(3, 1)

    # method_name 분기 (UI 표시용)
    if isinstance(ba_final, BundleAdjustExtendedResult):
        method_name = f"BA(+offset+link, seed={ba_final_seed})"
    elif ba_final.n_joint_vars > 0:
        method_name = f"BA(+offset, seed={ba_final_seed})"
    else:
        method_name = f"BA(seed={ba_final_seed})"

    # 변수 추출 분기
    if isinstance(ba_final, BundleAdjustExtendedResult):
        joint_offset_rad = ba_final.joint_offset_rad.copy()
        joint_offsets_estimated = True
        link_trans_delta = ba_final.link_trans_m.copy()        # ← 확장 BA만
        link_rot_delta   = ba_final.link_rot_rad.copy()         # ← 확장 BA만
        link_offsets_estimated = True
    elif ba_final.n_joint_vars > 0:
        joint_offset_rad = ba_final.joint_offset_rad.copy()
        joint_offsets_estimated = True
```

### 7d. 응답 dict — Frontend에 link offset 전달

```python
n_link = min(5, len(arm_motor_ids))
link_trans_list = [
    {
        "motor_id": int(arm_motor_ids[i]),
        "x_mm": float(link_trans_delta[i, 0] * 1000.0),
        "y_mm": float(link_trans_delta[i, 1] * 1000.0),
        "z_mm": float(link_trans_delta[i, 2] * 1000.0),
        "x_m":  float(link_trans_delta[i, 0]),                  # 정밀 저장용
        "y_m":  float(link_trans_delta[i, 1]),
        "z_m":  float(link_trans_delta[i, 2]),
    }
    for i in range(n_link)
]
# link_rot도 비슷한 dict 리스트 (rx_deg/rx_rad 둘 다)
...

return {
    ...                                          # 기존 필드
    "joint_offset_estimated": joint_offsets_estimated,
    "joint_offset_delta": joint_offset_list,
    "link_offset_estimated":  link_offsets_estimated,    # ← 신규
    "link_trans_delta":       link_trans_list,            # ← 신규
    "link_rot_delta":         link_rot_list,              # ← 신규
    ...
}
```

`mm`과 `m` 둘 다 보내는 이유: UI는 mm으로 표시(사람 친화), commit 시
정밀 저장은 m(np.float64 손실 없음).

---

## 8. 백엔드 통합 — CalibrationNode 핸들러

[calibration_node.py](../backend/nodes/application/calibration_node.py)는 Zenoh 서비스
핸들러를 들고 있다. compute / commit 둘 다 수정.

### 8a. import + commit 핸들러에서 LinkCoordinates 사용

```python
# calibration_node.py 맨 위
from core.joint_coordinates import JointCoordinates
from core.link_coordinates import LinkCoordinates                  # ← 신규
from modules.calibration.link_offsets import LinkOffsets           # ← 신규
```

### 8b. compute 핸들러 — mode 인자 + use_extended_ba 전달

```python
def _srv_handeye_compute(self, req: dict) -> dict:
    arm_motor_ids = [cfg.id for cfg in self._arm_cfgs]
    joint_limits = self.solver.joint_limits(len(arm_motor_ids))

    # mode: "extended" (기본) / "standard" (회귀 진단용 fallback)
    mode = str(req.get("mode", "extended")).lower()
    use_extended_ba = mode != "standard"

    diag = self.hand_eye.compute_with_diagnostics(
        fk_fn=self.solver.fk_to_matrix,
        arm_motor_cfgs=self._arm_cfgs,
        joint_limits_rad=joint_limits,
        use_extended_ba=use_extended_ba,                  # ← 신규 인자
    )
    ...
```

기본을 `"extended"`로 둔 이유 — validation으로 generalize 확인됐고,
σ 모든 면에서 더 좋음. Frontend는 mode 인자 안 보내면 자동 extended.

### 8c. commit 핸들러 — joint_offsets + link_offsets 둘 다 누적 저장

```python
def _srv_handeye_commit(self, req: dict) -> dict:
    ...
    # 1) hand_eye.npz — 카메라↔그리퍼 외부 보정
    self.hand_eye.save(hand_eye_path)

    # 2) joint_offsets.npz — 기존 패턴 그대로 (cumulative 합산)
    if self._last_compute.get("joint_offset_estimated"):
        delta_by_id = {int(e["motor_id"]): float(e["offset_rad"])
                       for e in self._last_compute["joint_offset_delta"]}
        applied = JointCoordinates().commit_offsets(delta_by_id,
                                                     method=self.hand_eye.result.method)

    # 3) link_offsets.npz — 신규 (cumulative 합산)
    link_msg = ""
    restart_required = False
    if self._last_compute.get("link_offset_estimated"):
        trans_list = self._last_compute["link_trans_delta"]
        rot_list   = self._last_compute["link_rot_delta"]
        # 응답 dict → LinkOffsets dataclass 변환
        delta = LinkOffsets(
            trans={int(e["motor_id"]): np.array([e["x_m"], e["y_m"], e["z_m"]])
                   for e in trans_list},
            rot={int(e["motor_id"]): np.array([e["rx_rad"], e["ry_rad"], e["rz_rad"]])
                 for e in rot_list},
        )
        # 디스크 누적 + PC 메모리 갱신
        link_applied = LinkCoordinates().commit_offsets(delta,
                                                         method=self.hand_eye.result.method)
        restart_required = True
        link_msg = f" + link_offsets 갱신 (n={len(link_applied.trans)})"

    return {
        "success": True,
        "message": f"저장 완료{offset_msg}{link_msg}",
        "data": {
            "joint_offsets_applied": ...,
            "link_offsets_applied":  link_offsets_estimated,
            "link_offsets":          link_applied_meta,
            "restart_required":      restart_required,      # ← UI에 표시
        },
    }
```

**`restart_required: true`가 중요** — `PybulletSolver`는 URDF를 부팅 시 1회만
로드하므로 commit 후 메모리 자동 갱신 X. 다음 부팅에 적용. UI가 사용자에게
"백엔드 재시작 필요" 알림.

---

## 9. 프론트엔드 통합 — 타입 + 결과 UI

[frontend/src/components/calibration/](../frontend/src/components/calibration/)
의 types.ts + HandEyeResults.tsx 수정.

### 9a. 타입 추가 — types.ts

```typescript
/** link translation 보정. URDF <joint><origin xyz/>에 더할 dx,dy,dz. */
export type LinkTransDelta = {
  motor_id: number;
  x_mm: number;
  y_mm: number;
  z_mm: number; // UI 표시용
  x_m: number;
  y_m: number;
  z_m: number; // commit 정밀 저장용
};

/** link rotation 보정 (small-angle 가정으로 rpy ≈ rotvec). */
export type LinkRotDelta = {
  motor_id: number;
  rx_deg: number;
  ry_deg: number;
  rz_deg: number;
  rx_rad: number;
  ry_rad: number;
  rz_rad: number;
};
```

기존 `ComputeData` 타입에 필드 추가:

```typescript
export type ComputeData = {
  ...                                              // 기존 필드
  joint_offset_estimated: boolean;
  joint_offset_delta: JointOffsetDelta[];
  // 확장 BA에서만 채워짐. standard fallback이면 false + 빈 배열.
  link_offset_estimated: boolean;                 // ← 신규
  link_trans_delta: LinkTransDelta[];              // ← 신규
  link_rot_delta:   LinkRotDelta[];                // ← 신규
  recommendations: NextPoseRecommendation[];
};
```

### 9b. 결과 테이블 — HandEyeResults.tsx

기존 `JointOffsetTable` 패턴 따라 두 컴포넌트 추가:

```typescript
/** link translation. |값| > 20mm면 gauge freedom 의심 — 노랑. */
function linkTransColor(mm: number): string {
  const mag = Math.abs(mm);
  if (mag < 5)  return "text-muted-foreground";
  if (mag < 20) return "text-foreground";
  return "text-amber-500";          // 의심 시 사용자에게 시각적 경고
}

function fmtSigned(v: number, frac: number): string {
  return (v >= 0 ? "+" : "") + v.toFixed(frac);
}

function LinkTransTable({ rows }: { rows: LinkTransDelta[] }) {
  return (
    <div>
      <p className="text-[10px] text-muted-foreground font-mono mb-1">
        link translation delta (mm) — joint origin xyz 보정, COMMIT 시 누적
      </p>
      <table className="w-full text-[11px] font-mono">
        <tbody>
          {rows.map((r) => (
            <tr key={r.motor_id}>
              <td className="py-0.5 text-muted-foreground">J{r.motor_id}</td>
              <td className={`py-0.5 text-right ${linkTransColor(r.x_mm)}`}>
                x {fmtSigned(r.x_mm, 2)}
              </td>
              <td className={`py-0.5 text-right ${linkTransColor(r.y_mm)}`}>
                y {fmtSigned(r.y_mm, 2)}
              </td>
              <td className={`py-0.5 text-right ${linkTransColor(r.z_mm)}`}>
                z {fmtSigned(r.z_mm, 2)}
              </td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
}

// LinkRotTable도 비슷 — 임계 0.5°/2.0°
```

색 임계의 의미:

- `<5mm`: 회색(정상 — 가산해도 시스템 무영향)
- `<20mm`: 흰색(주의 — 확인 필요)
- `≥20mm`: 노랑(gauge freedom 의심 — 진짜 link 미스매치인지 한 번 더 검토)

`ComputePreview`에서 렌더링:

```typescript
{data.joint_offset_estimated && data.joint_offset_delta.length > 0 && (
  <JointOffsetTable rows={data.joint_offset_delta} />
)}
{/* ↓ 신규 — 확장 BA일 때만 보임 */}
{data.link_offset_estimated && data.link_trans_delta.length > 0 && (
  <LinkTransTable rows={data.link_trans_delta} />
)}
{data.link_offset_estimated && data.link_rot_delta.length > 0 && (
  <LinkRotTable rows={data.link_rot_delta} />
)}
```

`link_offset_estimated`가 false면 (standard fallback) 자동으로 안 보임 →
기존 UI 회귀 없음.

---

## 10. 부팅 시 흐름 — LinkCoordinates + PybulletSolver

`link_offsets.npz`(디스크) → 메모리 캐시 → patched URDF → PyBullet 로드.

### 10a. LinkCoordinates (JointCoordinates 패턴 그대로)

[link_coordinates.py](../backend/core/coords/link_coordinates.py) — 싱글톤:

```python
LINK_OFFSETS_PATH = Path(__file__).parents[2] / "robot" / "calibration" / "link_offsets.npz"

class LinkCoordinates:
    _instance = None
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized: return
        self._initialized = True
        self._offsets: LinkOffsets = link_offsets_io.load(LINK_OFFSETS_PATH)  # 부팅 시 1회 로드

    def snapshot(self) -> LinkOffsets:
        with self._cache_lock:
            return LinkOffsets(trans=dict(self._offsets.trans), rot=dict(self._offsets.rot))

    def commit_absolute(self, offsets, method):
        """COMMIT 시 atomic 갱신: 디스크 *overwrite* + 메모리 reload."""
        link_offsets_io.save(LINK_OFFSETS_PATH, offsets, method=method)
        with self._cache_lock:
            self._offsets = LinkOffsets(trans=dict(offsets.trans), rot=dict(offsets.rot))
        return self.snapshot()
```

> **2026-06-10 정정**: 이 위 § 8c 의 "cumulative" 표현은 stale.
> `commit_absolute` 4종 (joint/link/sag/tool) 모두 overwrite contract. caller (calibration_node) 가 (현재 disk + BA delta) 를 absolute 로 변환한 후 한 번에 덮어쓴다.
> joint 만 옛 `commit_offsets(delta)` cumulative 였던 시절은 [calibration_ux_rewrite.md §6](calibration_ux_rewrite.md) 의 Bug A (last_compute stale double-add) 노출 자리 → API 통일로 제거.
> 진짜 수렴 신호는 "다음 라운드 BA 가 추정하는 *delta* 가 0 에 가까움" — 이건 그대로 (BA math 내부).

### 10b. PybulletSolver 수정

[solver.py:30~](../backend/modules/kinematics/solver.py) — 부팅 시 patched URDF
생성하고 그걸 로드:

```python
class PybulletSolver:
    def __init__(self):
        if self._initialized: return
        self._initialized = True

        # ← 신규: 디스크 link_offsets → patched URDF 생성
        link_offsets = LinkCoordinates().snapshot()
        urdf_to_load = write_patched_urdf(URDF_PATH, link_offsets)
        if not link_offsets.is_empty():
            logger.info(f"patched URDF 로드: {urdf_to_load}")

        self._client = p.connect(p.DIRECT)
        self._robot = p.loadURDF(str(urdf_to_load), useFixedBase=True, ...)
        # ↑ 원본 URDF_PATH 아니라 patched 경로
```

`link_offsets`가 비어있어도 `write_patched_urdf`는 호출됨 — 그러면 mesh 절대화만
적용된 URDF가 `.patched/`에 생성, joint origin은 원본 그대로. 즉 link_offsets
없을 때도 정상 동작.

### 10c. 전체 흐름

```
[Frontend] [COMPUTE]
  → calibration_node._srv_handeye_compute
  → HandEyeCalibration.compute_with_diagnostics(use_extended_ba=True)
  → bundle_adjust_hand_eye_extended()
  → 응답 dict: { joint_offset_delta, link_trans_delta, link_rot_delta, ... }

[Frontend] [COMMIT]
  → calibration_node._srv_handeye_commit
  → JointCoordinates().commit_offsets(...)  → joint_offsets.npz (누적)
  → LinkCoordinates().commit_offsets(...)   → link_offsets.npz   (누적, 신규)
  → 응답: restart_required=true

[Backend 재시작]
  → PybulletSolver() 부팅
  → LinkCoordinates() 새 값 로드
  → write_patched_urdf(...) → .patched/omx_f.urdf 갱신
  → p.loadURDF(patched_path) → FK/IK가 새 모델로 동작
```

---

## 11. 검증 — patched URDF가 numpy fk_chain과 일치하는가

BA는 numpy `fk_chain`으로 푸는데 production은 PyBullet의 patched URDF.
**두 경로가 수치적으로 같아야** BA가 풀어준 값이 시스템에 그대로 반영됨.

URDF patcher 일치 검증 — 같은 random angles로 양쪽 FK 호출:

```python
for k in range(30):
    angles = rng.uniform(-np.pi/2, np.pi/2, 5)

    # (A) PyBullet (patched URDF)
    for j, idx in enumerate(arm_indices):
        p.resetJointState(robot, idx, float(angles[j]), ...)
    state = p.getLinkState(robot, ee_index, computeForwardKinematics=True, ...)
    pb_pos = np.array(state[4])
    pb_R   = quat_to_R(state[5])

    # (B) numpy fk_chain (같은 link_offset)
    np_R, np_t = fk_chain(angles, LINK_TRANS, LINK_ROT)

    pos_err_mm  = np.linalg.norm(pb_pos - np_t) * 1000
    rot_err_deg = ... # axis-angle 차이

결과: max pos_err = 0.047mm,  max rot_err = 0.012°
```

수치 정밀도 수준에서 일치. 즉:

- BA가 numpy로 푼 link_offset 값 = 같은 link_offset으로 patched URDF 만들면 같은 FK
- 자세 시뮬에서 BA가 예측한 EE 위치 = production code에서 본 EE 위치
- 시스템 일관성 보장

---

## 12. 진짜 system 보정인지 vs overfit인지 — Hold-out validation

41자유도가 32포즈에만 fit한 overfit일 수 있음. Hold-out validation:

```python
# 32포즈를 train(24)/test(8) random split — 3 seed 반복
for seed in range(3):
    rng = np.random.default_rng(seed)
    idx = rng.permutation(n)
    train_idx, test_idx = idx[:24], idx[24:]

    # train으로 BA 풀기
    x_opt = fit_train(angles_train, R_tc_train, t_tc_train)

    # train의 mean_pos / mean_R을 기준으로 test 포즈들의 σ 측정
    T_train = compute_T_list(x_opt, angles_train)
    T_test  = compute_T_list(x_opt, angles_test)

    mean_pos = positions_train.mean(axis=0)
    mean_R   = mean_rotation(...)

    sr_train, st_train = sigma_against_train(T_train)
    sr_test , st_test  = sigma_against_train(T_test)   # test가 train과 얼마나 다르나
    ratio = sr_test / sr_train

평균:
  train σ=(1.28°, 9.82mm)
  test  σ=(1.35°, 9.88mm)
  ratio = 1.06× / 1.01×    ← 1.5× 이내면 양호
```

test ≈ train → overfit 아님. BA가 진짜 system 파라미터 잡은 거.

---

## 13. 분산 동기화 — CLAUDE.md 패턴 그대로

`link_offsets.npz`는 git tracked → 모든 머신이 같은 commit = 같은 파일.
Zenoh 토픽 전파 X.

```
PC에서 [COMMIT]
  → robot/calibration/link_offsets.npz 저장
  → PC의 LinkCoordinates 메모리 즉시 갱신
  → 단 PybulletSolver는 부팅 시 로드라 *재시작 필요*

PC에서 git add + commit + push

모터 Pi (motion_node)
  → ssh + git pull + backend 재시작
  → PybulletSolver 부팅 시 새 link_offsets로 patched URDF 자동 생성
  → motion_node의 IK가 patched URDF로 풀음

카메라 Pi (camera_node)
  → 영향 없음 (PybulletSolver 안 씀)
```

`.patched/`가 .gitignore된 게 핵심:

- 머신마다 자기 link_offsets로 자체 생성
- git에 보이는 URDF는 원본 하나뿐
- push되는 건 `link_offsets.npz` 하나 → 분산 모드 깔끔

---

## 14. 결과 해석 가이드

확장 BA 결과 dict의 어떤 값을 보고 무엇을 판단하나:

| 필드                 | 색 임계 ([HandEyeResults.tsx](../frontend/src/components/calibration/HandEyeResults.tsx)) | 의미                                               |
| -------------------- | ----------------------------------------------------------------------------------------- | -------------------------------------------------- |
| `sigma_rot_deg`      | <1° good, <2° warn                                                                        | 회전 floor — link_rot가 흡수                       |
| `sigma_t_mm`         | <10mm good, <20mm warn                                                                    | 위치 floor — link_trans가 흡수                     |
| `joint_offset_delta` | abs<2° 정상, ≥2° 주의                                                                     | horn-level 보정. 첫 commit 후 잔여는 ≈0            |
| `link_trans_delta`   | abs<5mm 정상, <20mm 노랑, ≥20mm 주의                                                      | mm 단위. ≥20mm면 gauge freedom 의심                |
| `link_rot_delta`     | abs<0.5° 정상, <2° 노랑                                                                   | small-angle 가정 안. 2°↑면 ZYX vs rotvec 변환 검토 |

**정상 수렴 시그니처:** σ가 GOOD 안 + 다음 라운드 `*_delta`가 모두 ≈0.
그게 BA가 "더 흡수할 게 없음" 신호.

---

## 15. 더 정밀하게 — 0.5° / 5mm 도달 경로

확장 BA가 모델 변수로 풀 수 있는 만큼 다 풀었는데도 1.3°/9mm 가 남았다 →
**모델 *밖*의 노이즈가 floor를 결정한다.** 그 노이즈 출처를 줄이면 σ도 같이 떨어짐.

### 15a. floor 노이즈 출처 분석

| 노이즈 출처                             | 현재 영향                     | 개선 시 효과                  |
| --------------------------------------- | ----------------------------- | ----------------------------- |
| D405 color **intrinsic** (factory seed) | PnP에 0.1~0.3° 회전 노이즈    | σ_rot 0.2~0.4° ↓              |
| **체커보드 인쇄/평탄도**                | corner 위치 ±0.5mm            | σ_rot 0.2~0.3° ↓, σ_t 1~2mm ↓ |
| **PnP corner detection** 정밀도         | refine 안 하면 ~0.1°          | σ_rot 0.1~0.2° ↓              |
| **자세 다양성** (J1/J4/J5 std)          | 부족 시 ill-conditioned       | σ_rot 0.1~0.3° ↓              |
| **모션 블러**                           | 캡처 전 0.5s 대기로 무시 가능 | —                             |

각 출처가 *독립적으로 누적*되니까 여러 개 잡으면 곱하기로 효과 — 셋만
잡아도 σ_rot 1.3° → 0.5° 가능.

### 15b. 1순위 — 체커보드 정확도 (ROI 최대)

지금 일반 종이 인쇄면 격자 ±0.5~1mm 오차가 그대로 PnP에 반영. corner
검출이 sub-pixel 정확해도 _진짜 좌표가 틀린_ 거라 BA로 못 푼다.

대안:

- **레이저 컷 아크릴/금속판** + 정밀 인쇄 부착 (또는 plotter 인쇄 후
  유리/아크릴 마운트로 평탄도 확보)
- 격자 크기 **25~30mm 정사각형** (더 크면 PnP 안정성 ↑, 9×6 정도)
- D405 작업 거리 **30~40cm 고정** — 너무 가까우면 시야 부분만 차지, 너무
  멀면 corner 해상도 부족

이 하나로 σ_rot 0.3° / σ_t 2mm 빠질 가능성. **하드웨어 투자 필요** (3D프린트 또는 외주).

### 15c. 2순위 — D405 intrinsic 재캘리브

[intrinsic.npz](../robot/calibration/intrinsic.npz)는 factory seed로 채워졌다
(`rms_error=0.0` — factory 값을 그대로 적은 거라 0. 진짜 재캘 잔차가 아님).

D405는 factory 캘이 일반적으로 정확하지만, 0.1~0.3° 수준의 미세 노이즈는 남음.
체커보드로 재캘하면 그걸 잡을 수 있다.

코드는 이미 다 있음 — [backend/modules/calibration/intrinsic.py](../backend/modules/calibration/intrinsic.py):

```python
# cv2.findChessboardCornersSB (sector-based)
#   조명/블러에 강하고 sub-pixel 정확도까지 내장.
# cv2.calibrateCamera(obj_points, img_points, image_size, ...)
#   K, dist 추정 + per-image rms_error 반환
```

Frontend의 Intrinsic 탭에서:

1. 다양한 각도/거리/회전으로 **15~20장** 캡처 (체커보드가 화면 다른 위치를 골고루)
2. COMPUTE → `rms_error < 0.3px`면 좋은 결과
3. COMMIT → `intrinsic.npz` 갱신

새 intrinsic 적용된 후 Hand-Eye 캘 다시 돌리면 σ_rot 0.2~0.3° 추가 감소.

### 15d. 3순위 — PnP refineLM (검증 결과: 효과 0, 변경 X)

> 처음에는 "코드 10줄로 σ 0.1~0.2° 추가 감소" 후보로 검토했으나
> 합성 잡음 Monte Carlo로
> 효과가 0임을 확인. 현재 코드 유지.

가설은 — [pose_estimator.py:24](../backend/modules/calibration/pose_estimator.py)
의 `cv2.solvePnP(obj, img, K, dist)`가 "초기 해"만 풀고, 그 위에
`cv2.solvePnPRefineLM`을 추가하면 LM이 재투영 잔차를 더 최소화한다 — 였음.

검증: 기존 41포즈의 (R, t)를 ground truth로 취급, `cv2.projectPoints`로
ideal corner 생성 후 가우시안 0.05~0.5px 잡음 추가, 4가지 변형 비교.

| 변형                                    | rot 평균 오차 | t 평균 오차 |
| --------------------------------------- | ------------- | ----------- |
| (i) ITERATIVE alone _(= 현재 코드)_     | **0.3252°**   | **0.711mm** |
| (ii) EPNP alone                         | 1.0241°       | 2.314mm     |
| (iii) EPNP + refineLM                   | 0.3294°       | 0.717mm     |
| (iv) ITERATIVE + refineLM _(원래 제안)_ | **0.3252°**   | **0.711mm** |

**원인 — default flag**: `cv2.solvePnP`는 flag 미지정 시 `SOLVEPNP_ITERATIVE`로
폴백. 이 모드는 _내부에서 이미 LM refinement를 수행_. 그 위에 `solvePnPRefineLM`을
호출하면 같은 잔차에 같은 LM을 같은 seed에서 다시 돌리는 셈 → no-op. 모든
노이즈 레벨에서 (i) ≡ (iv) 가 numerically 정확히 일치.

(iii)이 (i)과 비슷한 건 LM이 EPNP의 거친 해(iii의 (ii) seed)를 ITERATIVE 수준까지
끌어올렸기 때문. 즉 LM은 _좋은 seed가 없을 때만_ 의미 있고, 우리 코드는 이미
ITERATIVE라 좋은 seed가 보장돼 있음.

합성 잡음 가정은 실제 corner 검출 노이즈와 분포가 다를 수 있어 _절대 σ 수치_
는 신뢰 X, 다만 *알고리즘 우위*는 잘 드러남 — refineLM 추가의 우위가 정확히
0이면 실측에서도 0.

**결론**: pose_estimator.py 변경 없음. § 15g 권장 경로에서 이 행 제거.

### 15e. 4순위 — 자세 다양성 점검

확장 BA가 잘 풀려면 J1/J4/J5(회전 추정 주요 축) 자세가 _고르게 흩어져_ 있어야 함.
한 축이 좁은 범위에 몰려 있으면 BA가 그 방향 정보를 못 받아 ill-conditioned.

기존 진단 코드 — [coach.py](../backend/modules/calibration/coach.py)의
`axis_distributions`가 각 축 std와 추천 추가 캡처 영역을 알려줌:

```python
# COMPUTE 응답의 coach.axis_distributions
# 각 항목: {motor_id, std_deg, min_deg, max_deg, is_low_diversity, suggested_deg, ...}
```

`is_low_diversity=true`인 축 있으면 그쪽 자세 5~10개 추가 캡처 후 재캘.
[thresholds.py](../backend/modules/calibration/thresholds.py)의
`JOINT_DIVERSITY_THRESHOLD_DEG=(25, 15, 15, 25, 30)` 미만이 low.

### 15f. 5순위 이하 — 장기/하드웨어

| 액션                                | 효과                                        | 비용                 |
| ----------------------------------- | ------------------------------------------- | -------------------- |
| 모터 horn 정밀 재조립 (각도 게이지) | J2/J3 큰 joint_offset 제거                  | 분해 필요            |
| 링크 부품 실측 → URDF 직접 갱신     | link_trans/rot이 0에 가까워짐, 모델 더 깨끗 | 캘리퍼스 + URDF 수정 |
| 더 큰 / 정밀한 체커보드 (50mm)      | PnP 안정성 한 단계 ↑                        | 새 보드 제작         |
| 멀티 자세 누적 ICP refinement       | 캘 결과 추가 검증                           | 별도 알고리즘        |

### 15g. 현실적 권장 경로

| 순서    | 액션                                                               | 누적 σ_rot / σ_t   | 비용            |
| ------- | ------------------------------------------------------------------ | ------------------ | --------------- |
| 0       | 확장 BA 적용                                                       | 1.30° / 9.3mm      | 완료            |
| ~~1~~   | ~~+ PnP refineLM~~ — § 15d 검증으로 폐기 (효과 0)                  | —                  | —               |
| ~~1.5~~ | ~~+ Robust loss (huber f=0.010)~~ — § 15i 검증으로 폐기 (artifact) | —                  | —               |
| **0.5** | **+ 물리 sag 모델 (§16, default)**                                 | **0.65° / 7.94mm** | 완료            |
| 2       | + intrinsic 재캘리 (UI에서)                                        | ~0.5° / ~6mm       | 1시간           |
| 3       | + 정밀 체커보드 (아크릴 마운트)                                    | **~0.4° / ~5mm**   | 3D프린트 / 외주 |
| 4       | + 자세 다양성 보강 (J2 수평 자세 추가)                             | ~0.3° / ~4mm       | 추가 캡처 30분  |

§16의 물리 sag로 step 0.5에서 σ_rot 절반(1.30→0.65). **2+3 조합으로 산업
정밀도 도달.** 4는 보너스이자 §16의 연속 split extrapolation 한계 해소책.

각 단계 적용 후 확장 BA 다시 돌려서 σ 측정 → 진짜로 떨어졌는지 검증.
한 단계씩 확인하면서 가는 게 안전 (어디서 효과 가장 큰지 데이터로).

### 15i. 검증 폐기 — Robust loss (huber/cauchy/soft_l1)

> Robust loss 진단(6변형 + hold-out + reg sweep)에서
> 으로 검증. σ_t 1mm 개선이 _모델 파라미터 폭주에 의존하는 numerical artifact_
> 임을 확인 → 채택 X. bundle_adjust.py default는 lm/linear 유지.

가설: 현재 잔차 분포에 약한 outlier 있음 (σ_t max/RMS = 2.19×). robust loss
(huber/cauchy/soft_l1)로 그 영향 down-weight하면 σ 추가 감소 가능. method='lm'은
robust loss 미지원이라 'trf' (Trust Region Reflective)로 함께 교체.

1차 결과 (41 포즈 fit):

| Variant               | σ_rot  | σ_t        | X Δt vs linear |
| --------------------- | ------ | ---------- | -------------- |
| (현재) lm/linear      | 1.295° | 9.17mm     | —              |
| trf/linear (sanity)   | 1.295° | 9.17mm     | 0mm            |
| **trf/huber f=0.010** | 1.294° | **8.29mm** | **16.80mm**    |
| trf/cauchy f=0.010    | 1.293° | 8.17mm     | 25.48mm        |

σ_t 1mm 개선처럼 보임 — but X(cam2gripper)가 17~25mm 움직임.

Hold-out (train 33 / test 8, 3 random seed) — generalize 확인:

| Variant        | train σ_t | test σ_t | ratio_t |
| -------------- | --------- | -------- | ------- |
| huber f=0.010  | 8.24mm    | 8.74mm   | 1.06× ✓ |
| cauchy f=0.010 | 8.02mm    | 9.31mm   | 1.16× ⚠ |

huber는 ratio 1.06× — 확장 BA 원본 1.01×와 비슷한 generalize. _통계적으로_
는 진짜 개선처럼 보임. 그러나:

**진짜 원인 발견 — reg weight sweep**:

| link_reg          | σ_rot  | σ_t         | maxLT (mm)  |
| ----------------- | ------ | ----------- | ----------- |
| 1.0/1.0 (default) | 1.294° | **8.29**    | **49.17** ⚠ |
| 2.0/1.0           | 1.295° | 9.03        | 38.56       |
| 2.0/2.0           | 1.301° | 9.14        | 38.46       |
| 3.0/2.0           | 1.302° | **10.14** ⚠ | 26.67       |

linear (현재)의 maxLT는 ~29mm. robust huber default는 maxLT 49mm로 link offset
이 **70% 폭주**. link_reg를 강화해 maxLT를 linear 수준(27mm)으로 묶으면 σ_t는
역으로 10.14mm — _linear(9.17mm)보다 나빠짐_.

즉: **robust loss는 1mm σ를 줄이지만 그건 link 모델이 50mm 더 풀린 결과**.
같은 link 크기에서는 linear가 우월. patched URDF가 link2 origin을 49mm 옮긴
상태로 IK/FK 풀게 되는 게 _physical sense_. → 다음 라운드부터 운동학 모델이
점점 비현실적으로 발산할 위험.

**결론**: bundle_adjust.py default = lm/linear 유지. method/loss/f_scale 파라
미터는 인터페이스에 남겨 비교 실험 가능 (회귀 진단/디버깅 용도).

**일반 교훈**: σ만 보고 robust loss 판단 금지. 모델 파라미터(여기선 link offset)
가 _물리적으로 안 변하는지_ 함께 확인. σ ↓ + 파라미터 폭주 = numerical artifact
(자유도가 다른 변수로 흘러감). σ ↓ + 파라미터 안정 = 진짜 개선.

### 15h. 한계 — 모터 zero point 측정의 어려움

위 액션 다 해도 σ_rot < 0.2° 가려면 *모터 zero point의 물리적 정확도*가 필요.
Dynamixel raw 2048이 URDF의 0°와 정확히 일치한다는 보장이 없는데, 이건 외부
정밀 측정기 (각도 게이지, encoder 등)로만 검증 가능. DIY 환경에선 BA가
joint_offset으로 보정하는 게 한계.

물리 sag(§16)로 자세 의존 부분을 분리하면서 *조립/모터/링크 정밀도*가 진짜
floor가 됨. 산업 로봇이 0.1°까지 가는 건 *외부 정밀 측정 장비를 캘 단계에
사용*하기 때문. DIY 5축에선 **0.3° 정도가 현실적 한계** (sag 적용 + intrinsic
재캘 + 정밀 체커보드 조합 기준). 그 이하는 비용 곡선이 가파르게 올라감.

---

## 16. 물리 sag 모델 — 자세 의존 중력 처짐 분리

> 확장 BA 이후 σ_rot 1.30°/σ_t 9.3mm 정체. 한 사용자 관찰이 다음 단계의 단서를 줌:
> _"토크 OFF → 자세 잡음 → 토크 ON → 살짝 처짐. 자세마다 다 다름 (팔 펴면 더 처짐)."_
> 이건 link offset(자세 _무관_ 상수 보정)으로는 표현 불가능 — _자세 의존_ 오차.

### 16.1. 진단 — 외부 의견을 데이터로 검증

가설: "확장 BA의 link offset이 흡수한 것 중 일부는 사실 자세 의존 중력 sag."

Sag 가설 진단 — 41 포즈에 6 시나리오 BA fit:

| 시나리오                           | σ_rot  | σ_t    | DOF    | link_t_max |
| ---------------------------------- | ------ | ------ | ------ | ---------- |
| (4) link on, sag off **[현 prod]** | 1.296° | 9.29mm | 41     | 29.1mm     |
| (6) link on, sag sincos            | 0.633° | 7.76mm | 45     | **5.6mm**  |
| (3) link off, sag sincos           | 0.814° | 9.55mm | **15** | —          |

발견 두 가지:

- **(3) vs (4)**: DOF 15짜리가 DOF 41 prod보다 σ_rot에서 압도 (0.81° vs 1.30°).
  link offset 30개가 사실상 *자세 의존 오차의 빈약한 대용품*이었다는 시사.
- **(6) → link_t_max 29.1→5.6mm**: sag 추가하니 link offset이 5배 줄어듦.
  § 15i robust loss와 정반대 signature (그땐 σ↓ + 파라미터 폭주 = artifact).
  여기는 **σ↓ + link 폭주 감소 = 진짜 분리되는 변수**.

### 16.2. sin/cos basis는 왜 부족한가

처음엔 `sag_J = a·sin(θ_J) + b·cos(θ_J)` 임의 sinusoidal basis로 시작. random
hold-out(1.05× ratio)에선 통과했지만 robustness 진단의 *J2 연속 split*에서 폭주:

| split                                        | sag off       | sag sincos           |
| -------------------------------------------- | ------------- | -------------------- |
| lower 70% → upper 30% (큰 sag로 extrapolate) | 2.35× / 2.79× | **6.61× / 7.84×** ❌ |

train J2 [-82°, -60°]만 보고 test의 J2 -5° (수평) 예측 → sin/cos가 잘못된 방향
폭주. 이유: 같은 J2 각도라도 _팔 펴짐 정도(J3, J4, J5 자세)에 따라 모멘트 암이
다른데_, single-joint angle basis는 그걸 못 잡음.

→ **물리 모델로 가야 함** — `error = f(자세 전체)`가 표현되도록.

### 16.3. 물리 모델 — 모멘트 암 ∝ 처짐

중력 토크의 1차 모델:

```
τ_J = (r × g_dir) · axis_J     where r = ee_pos - joint_origin   (base frame)
sag_J = k_J · τ_J              (k = 1/effective_stiffness, BA 변수)
```

`r`은 base frame에서 ee 위치 - joint 회전축 위치 = 모멘트 암 벡터. 같은 J2 각도라도
J3/J4/J5 자세가 다르면 ee 위치가 달라 `r`도 달라짐 → sag 다름. _전체 자세 의존성을
2 params(k_J2, k_J3)로 표현_.

[fk_chain.py](backend/modules/kinematics/fk_chain.py) helper:

```python
def fk_chain_with_axes(angles, link_trans=None, link_rot=None):
    """fk_chain + 각 joint origin/axis (base frame). 중력 토크 계산용."""
    T = np.eye(4)
    joint_origins_base = np.zeros((5, 3))
    joint_axes_base = np.zeros((5, 3))
    for i in range(5):
        T_o = np.eye(4)
        T_o[:3, :3] = rotvec_to_R(link_rot[i])
        T_o[:3, 3] = JOINT_ORIGINS[i] + link_trans[i]
        T = T @ T_o
        # 회전 적용 *전* 위치/방향이 토크 계산용
        joint_origins_base[i] = T[:3, 3]
        joint_axes_base[i] = T[:3, :3] @ JOINT_AXES[i]
        T = T @ axis_angle_to_R(JOINT_AXES[i], angles[i])
    ...
    return R_ee, t_ee, joint_origins_base, joint_axes_base


_GRAVITY_DIR = np.array([0.0, 0.0, -1.0])

def gravity_torque_lumped(ee_pos_base, joint_origin_base, joint_axis_base):
    """ee에 lumped mass 가정. τ = (r × g) · axis."""
    r = ee_pos_base - joint_origin_base
    return float(np.dot(np.cross(r, _GRAVITY_DIR), joint_axis_base))


def apply_gravity_sag(joint_angles, k_stiff, link_trans=None, link_rot=None):
    """commanded → sag 적용 actual. J2, J3에만 적용."""
    if k_stiff.size == 0 or float(np.max(np.abs(k_stiff))) < 1e-12:
        return joint_angles.copy()
    _, ee_pos, jo, ja = fk_chain_with_axes(joint_angles, link_trans, link_rot)
    out = joint_angles.copy()
    out[1] += k_stiff[0] * gravity_torque_lumped(ee_pos, jo[1], ja[1])
    out[2] += k_stiff[1] * gravity_torque_lumped(ee_pos, jo[2], ja[2])
    return out
```

J2, J3에만 sag — J1/J4/J5의 sag는 측정 noise 수준이라 모델 단순성 위해 제외
(물리 sag 진단에서 검증).

### 16.4. PyBullet vs lumped — URDF mass 부정확성 발견

URDF의 link5 mesh가 `follower_06_pan_Revised_d405.stl` (D405 버전)이지만
**inertial 데이터가 D405 무게 반영 안 됐을 가능성** 발견 — link5 mass=44g인데
D405 자체가 ~42g (마운트만 2g = 비현실).

PyBullet 비교 진단 — PyBullet의
`calculateInverseDynamics` (URDF mass 기반 정확 토크) vs lumped (mass × 모멘트
암 가정) 비교:

| 모델                     | σ_rot      | σ_t        |
| ------------------------ | ---------- | ---------- |
| **lumped**               | **0.651°** | **7.94mm** |
| PyBullet inverseDynamics | 0.766°     | 10.48mm    |

PyBullet이 lumped보다 σ_rot 0.115° 더 _나쁨_. 원인: URDF mass 부정확 → 토크
underestimate. lumped는 _k가 (1/stiffness × effective_mass) 비율을 통째로 흡수_
해서 mass 부정확성에 robust.

**결론**: 검증된 라이브러리(PyBullet)가 항상 우월하지 않음. URDF mass의
*정확도*가 보장 안 될 때는 lumped + k의 자유도로 mass 오차도 함께 fit하는 게
실용적. 미래 D405 mass center/inertia 측정 → URDF 업데이트 시 PyBullet 재검토.

### 16.5. 확장 BA + sag = 43 DOF

[bundle_adjust.py:bundle_adjust_hand_eye_physical_sag](backend/modules/calibration/bundle_adjust.py) — extended(41) + sag_k 2개 = 43:

```python
# 변수 layout:
#   [0:5]    joint_offset (rad)
#   [5:20]   link_translation (5×3, m)
#   [20:35]  link_rotation (5×3, rad rotvec)
#   [35:37]  sag_k (J2, J3) (rad / (m·g_unit))     ← 신규
#   [37:40]  rod (cam2gripper)
#   [40:43]  t (cam2gripper, m)

def compute_T_target_in_base(x):
    offset, link_t, link_r, sag_k, rod, t_x = unpack(x)
    R_x = cv2.Rodrigues(rod)[0]
    T_x = make_T(R_x, t_x)
    out = []
    for i in range(N):
        # joint angle에 offset + sag 둘 다 적용 후 FK
        a_corr = apply_gravity_sag(
            angles_arr[i] + offset, sag_k, link_t, link_r
        )
        R_gb, t_gb = fk_chain(a_corr, link_t, link_r)
        T_gb = make_T(R_gb, t_gb)
        out.append(T_gb @ T_x @ T_tc_list[i])
    return out
```

잔차 + reg는 extended와 동일 + sag_k에 `sag_k_reg=0.0` (default). reg sweep으로
0~0.1 sweet spot 확인 (robustness 진단 §F). k_J2/k_J3 자체가
작은 양수(~0.27, ~0.14)라 reg 없이도 폭주 안 함.

### 16.6. IK 역방향 — actual_to_commanded

FK는 `commanded → actual` (motor encoder reading → 실제 link end 자세). IK는
역방향이라 _implicit equation_:

```
actual = commanded + sag(commanded)   [BA 모델]
→ commanded = actual - sag(commanded)   [IK가 풀어야 할 것]
```

implicit이라 fixed-point. 1차 Taylor 근사 (sag ~2°라 잔차 < 0.05°):

```
commanded ≈ actual - sag(actual)
```

[fk_chain.py:actual_to_commanded](backend/modules/kinematics/fk_chain.py)가 이걸 처리. [PybulletSolver.ik](backend/modules/kinematics/solver.py)가 PyBullet IK 결과(`actual`)를 받아서 `actual_to_commanded` 한 번 호출 → motor 명령으로 변환.

### 16.7. SagCoordinates — joint/link와 다른 점

`sag_offsets.npz`는 joint/link와 같은 git tracked + cumulative merge 패턴
([SagCoordinates](backend/core/coords/sag_coordinates.py)). 차이점은 **PC 내부는
재시작 불필요**:

|                 | 어떻게 적용되나                                | 재시작                       |
| --------------- | ---------------------------------------------- | ---------------------------- |
| joint_offsets   | `raw_to_rad` 호출 시 JointCoordinates에서 읽음 | X (이미 즉시)                |
| link_offsets    | PyBullet URDF 로드 시점                        | **필요** (URDF 한 번만 로드) |
| **sag_offsets** | **매 FK/IK 호출 시 메모리에서 읽음**           | **X**                        |

[calibration_node.py:\_srv_handeye_commit](backend/nodes/application/calibration_node.py)에서 COMMIT 시 `solver._reload_sag_cache()` 호출 — 다음 FK/IK부터 자동 반영. 다른
머신은 git pull + 재시작 (joint/link와 동일).

### 16.8. 검증 한계 — 캡처 자세 범위 안 OK, 밖 미검증

물리 sag 진단의 연속 split:

| split                                               | sag off       | sag physical        |
| --------------------------------------------------- | ------------- | ------------------- |
| lower 70% → upper 30% (큰 sag 영역으로 extrapolate) | 2.35× / 2.79× | **4.44× / 6.35× ⚠** |
| upper 70% → lower 30% (작은 sag 영역)               | 2.45× / 2.86× | **2.08× / 2.28× ✓** |
| middle 60% → edges 40%                              | 2.34× / 2.21× | 2.86× / 2.88×       |

physical 모델은 sin/cos보다 _모든_ split에서 개선 (특히 작은 sag 영역으로
extrapolate는 sag off보다도 좋음). 그러나 가장 어려운 lower→upper는 여전히
4.44× — *J2 수평 자세를 캡처 안 한 영역으로 extrapolate*는 미검증.

**현재 41 포즈의 J2 분포 [-82°, -5°]**. OMX_F 일상 deployment(pick&place,
TSDF)는 책상 위 작업 → J2 ~-30°~-60° 정도가 보통. 즉 **캡처 영역 안**이라
production에선 평균적으로 σ_rot 0.65° 도달. 캡처 영역 밖(예: 팔 더 펴기)으로
가면 부분적으로 sag off보다 나쁠 수 있음.

이게 §15g의 step 4 "자세 다양성 보강" (J2 수평 자세 5~8개 추가 캡처)의
이유. 해결되면 lower→upper ratio도 2× 안으로 들어올 것.

### 16.9. 검증 — production BA가 진단 결과와 일치

smoke test로 새 `bundle_adjust_hand_eye_physical_sag`가 inline 진단과
_정확히_ 같은 결과 내는지 확인:

```
σ_rot = 0.651°   (diag 기대: 0.651°)   ✓
σ_t   = 7.94mm   (diag 기대: 7.94mm)   ✓
sag_k = (+0.26523, +0.14126)            ✓
max sag (deg): J2=4.67, J3=2.26         ✓
link_t_max = 7.7mm                      ✓
```

소수점 자리까지 일치. 진단 → production 이식 회귀 없음.

### 16.10. 결과 + 다음 단계

- σ_rot **1.30° → 0.65°** (절반)
- σ_t **9.29mm → 7.94mm** (15% 감소)
- link offset 폭주 감소 (29.1mm → 5.6mm) — 모델이 _물리적으로_ 더 깨끗

다음 단계는 §15g 표 그대로:

- step 2: intrinsic 재캘리브 (1시간)
- step 3: 정밀 체커보드 (3D프린트 / 외주)
- step 4: J2 수평 자세 캡처 (§16.8의 extrapolation 한계 해소)

추가로 — D405 mass center/inertia 정확히 측정 → URDF 업데이트하면 PyBullet
inverseDynamics 모델로 재검토 가능. 단 σ 0.1° 차이라 _라이브 PC 시각 검증_ 후
필요 시.

---

## 17. 미해결 항목 (다음 작업 후보)

§16까지로 backend 캘 정확도는 σ_rot 0.65°/σ_t 7.94mm 도달. 단 _그 정확도가
사용자 측에서 실제로 보이게_ 하려면 두 가지 작업이 남음.

### 17.1. 12V 전압 변경 (선택)

현재 메인 PSU 11V — XL430 정격(10~14.8V, 권장 12V) **하한 근처**. 12V로 올리면:

- XL430 모터 토크 출력 ~9% 증가
- 자세 의존 sag 크기 ~9% 감소 (max sag J2 4.67° → ~4.2°)
- 모터 안정성 ↑

단 **σ에 미치는 영향은 작음** (~0.05~0.1°) — §16 sag 모델이 11V 처짐을
_이미 보정_ 중이라 그 차이는 모델이 흡수. 캘 정확도 측면만 보면 12V 변경의
ROI는 작고, 진짜 가치는 _모터 토크 마진 + 안정성_.

변경 시 절차 — sag k 값이 11V에 fit돼 있어서 cumulative 누적으로 갈 경우 첫
라운드 delta가 큼 → **clean slate가 더 깔끔**:

```
1. 백업: robot/calibration/ 의 5 파일 (joint/link/sag_offsets.npz +
   handeye_poses.npz + hand_eye.npz)을 *_11v_backup.npz로 복사
2. 원본 5 파일 삭제 (intrinsic.npz는 카메라 자체 캘이라 유지)
3. backend 재시작 → SagCoordinates/JointCoordinates/LinkCoordinates 빈 상태
4. Hand-Eye 탭에서 30~40 자세 재캡처 → COMPUTE → COMMIT
5. 결과 검증:
   - σ_rot 0.6~0.7° (11V와 비슷한 수준이면 모델 재현성 OK)
   - sag k_J2 ~0.24, k_J3 ~0.13 (11V의 ~91%면 전압 효과 정량 확인)
   - joint_offset 11V와 비슷한 값 (전압 무관 변수의 안정성 검증)
```

**caveat**: XL330 그룹(J4/J5/그리퍼)는 XL4015 강압 모듈로 _5V 그대로_ 유지
([CLAUDE.md] 전원 토폴로지). 12V 변경은 XL430 (J1/J2/J3)만 영향 — sag 모델이
J2/J3에 적용된 것과 정확히 일치.

### 17.2. frontend 라이브 PC 시각화에 link/sag 반영 (중요)

§16의 sag 모델까지 적용됐지만 **frontend의 workspace3d 라이브 PC 시각화는
여전히 sag/link offset을 못 봄**. 사용자가 σ 0.65° 결과를 _시각으로_ 확인할
때 어긋남 잔존 가능.

#### 현재 반영 상태

|                  | frontend 반영 | 이유                                                                                                                       |
| ---------------- | ------------- | -------------------------------------------------------------------------------------------------------------------------- |
| **joint_offset** | ✅            | [Workspace3D.tsx](../frontend/src/pages/Workspace3D.tsx) 의 jointAngles 계산에서 `baseRad + jointOffsetsRad[id]` 적용      |
| **hand_eye**     | ✅            | calibration .npz fetch로 cameraMatrix 계산에 buildMatrix4 적용 — BA가 sag+link 적용해 fit한 값                             |
| **link_offset**  | ❌            | [RobotModel.tsx:53](../frontend/src/components/workspace3d/3d/RobotModel.tsx) 이 _원본 omx_f.urdf_ fetch. `.patched/` 아님 |
| **sag**          | ❌            | frontend URDF가 commanded angle로 시각화. `actual_to_commanded` 적용 X. URDF 정적이라 자세 의존 sag 표현 자체 불가         |

#### 결과로 일어나는 어긋남

- backend FK = `fk(commanded + sag, patched URDF)` → 정확한 actual ee 위치
- frontend FK (urdf-loader) = `fk(commanded, 원본 URDF)` → commanded ee + 미보정 link
- 둘이 다른 위치를 봄. cameraMatrix = tcpMatrix · handEyeMatrix 곱셈이 _어긋난 tcpMatrix_ 위에 곱해져서 PC가 commanded 자세 기준으로 렌더링 → 실제와 sag/link만큼 차이.

#### 해결 옵션

**옵션 A — patched URDF 두 버전 생성**

[urdf_patcher.py:write_patched_urdf](../backend/core/coords/urdf_patcher.py)에 `for_web: bool` 플래그 추가. True면 mesh path를 _상대 유지_ (PyBullet은 false로 절대화). frontend는 `.patched/omx_f_web.urdf` fetch.

단점: 두 버전 관리 (cache invalidation 등).

**옵션 B — backend FastAPI가 동적 변환** ⭐ 추천

[zenoh_bridge.py](../backend/bridge/zenoh_bridge.py)의 `/robot` 정적 마운트 대신, `/robot/urdf/omx_f/omx_f.urdf` 요청에 한해 _동적 라우트_ 추가:

```python
@app.get("/robot/urdf/omx_f/omx_f.urdf")
def serve_patched_urdf_for_web():
    link_offsets = LinkCoordinates().snapshot()
    text = patch_urdf_text(URDF_PATH, link_offsets, absolute_mesh=False)
    return Response(text, media_type="application/xml")
```

`patch_urdf_text`에 `absolute_mesh=False` 옵션 추가하면 mesh path 상대 유지. ETag/cache로 link_offsets 변경 시에만 무효화.

장점: frontend 코드 변경 X. backend의 단일 진실에서 동적으로 응답.

**옵션 C — sag 적용 ee pose를 backend가 publish** (옵션 A/B와 별개로 필요)

§17.2 옵션 C가 진짜 깔끔한 이유 — frontend의 RobotModel.tsx:92에서 emitTCP를 자체 계산 대신 backend가 publish한 ee_pose_actual을 받아 그대로 쓰면 됨. RobotModel은 시각화용 로봇 모양만 그리고, 진짜 TCP는 backend 권위.
그럼 주기는..? 주기가 문제일거 같긴한데..

sag는 자세 의존이라 URDF로 표현 불가. backend가 [PybulletSolver.fk](../backend/modules/kinematics/solver.py) 결과(actual ee pose)를 새 토픽으로 발행:

```
omx/motor/state/ee_pose_actual   # T_base_ee (sag 적용된 actual)
```

frontend는 자체 FK 안 하고 그 매트릭스를 tcpMatrix로 사용:

```typescript
// RobotScene.tsx — RobotModel 안 쓰고 받은 ee_pose 직접 사용
const tcpMatrix = useEEPoseStore((s) => s.eeMatrix);
```

장점: 모든 정확도가 backend 단일 진실에서 옴. URDF는 *robot 모양 시각화*만 담당, ee 자세는 backend 권위.

단점: backend가 motor state publish할 때마다 PybulletSolver.fk 호출 (현재 20Hz 정도면 부담 작음).

#### 작업 우선순위

라이브 PC 시각 검증 먼저:

- _충분히 정렬_ — 작업 보류, sag 모델 backend 적용으로 충분
- _명확한 어긋남_ — 옵션 B + 옵션 C 둘 다 진행. ROI 큰 순서로:
  1. 옵션 C (sag 적용 ee pose publish) — 가장 큰 어긋남 원인 (~2-4°)
  2. 옵션 B (동적 patched URDF) — 작은 어긋남 (~30mm link translation)

옵션 A는 두 버전 관리 부담이라 B로 통일이 깔끔.

#### 우회 — hand_eye가 이미 BA 결과 반영

부분 위안: **hand_eye.npz는 BA가 sag/link 적용해 fit**한 결과 (T_cam2gripper at _actual_ ee). 그래서 frontend가 _commanded_ tcpMatrix에 그 hand_eye를 곱해도 _부분적으로_ 보정됨 — hand_eye matrix가 sag만큼 카운터-오프셋 가지고 있어서. 단 자세에 따라 sag 변화량이 다르니 _완전 보정은 안 됨_. 옵션 C가 진짜 해결.

---

## 부록 — 진단 방법론 요약

본문에 인용된 진단들은 BA 락인 후 정리. 동일 데이터에 시나리오를 바꿔가며 σ 비교 + hold-out으로 generalize 확인 + 연속 split으로 robustness 확인이 핵심. 다른 robot이나 다른 BA 모델로 확장할 때 같은 방법론으로 재검증.

검증된 시나리오 요약:

- joint_offset ON/OFF, baseline 0/현재 비교 → §2의 표
- link translation/rotation 자유도 + regularization sweep → §6, §7
- Hold-out train/test split → §9
- Patched URDF vs numpy fk_chain 수치 일치 → §8
- bundle_adjust_hand_eye_extended sanity (진단 결과 재현) → §14
- 폐기 근거 — PnP refineLM (σ 변화 0) → §15d, robust loss (link offset 폭주 의존) → §15i
- Sag 모델 진화 — sin/cos basis(폭주) → lumped 물리 sag(채택) → PyBullet inverseDynamics(URDF mass D405 누락으로 σ 손해) → §16
