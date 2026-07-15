"""결정적: 전체 보정 FK 를 두 방식으로 계산해 TCP orientation 차이 측정.
 A = offline BA 방식(hand_eye 기준): raw URDF + link_offset 을 FkChain '파라미터'로
 B = v2 motion 방식(클라우드): link_offset 을 URDF에 patch 후 FkChain
둘 다 joint_offset 가산 + sag(apply_gravity_sag) 동일 적용.
A≠B (특히 orientation) 이면 = 사선 원인 (hand_eye 는 A 기준인데 클라우드는 B).
하드웨어 0."""
import sys, os, json, sqlite3, tempfile
import numpy as np

sys.path.insert(0, ".")
from apps.config import load_robots, _ROBOT_DIR  # noqa
from modules.motion.units import raw_to_rad  # noqa
from modules.motor.contract import MotorKind  # noqa
from modules.motion.fk_chain import FkChain  # noqa
from modules.motion.urdf_patch import patch_urdf_link_offsets  # noqa
from pathlib import Path  # noqa


def ang(Ra, Rb):
    c = (np.trace(Ra.T @ Rb) - 1) / 2
    return np.degrees(np.arccos(np.clip(c, -1, 1)))


ROBOT = "so101_6dof_0"
robot = load_robots()[ROBOT]
arm = [m for m in robot.motors if m.kind != MotorKind.GRIPPER]
arm_names = [m.name for m in arm]
n = len(arm)
urdf_raw = _ROBOT_DIR / robot.type / "urdf" / f"{robot.type}.urdf"

# DB 보정값
c = sqlite3.connect("horibot.db")
def load(kind):
    return json.loads(c.execute(
        "SELECT result_data FROM calibration_results WHERE robot_id=? AND kind=? AND is_active=1",
        (ROBOT, kind)).fetchone()[0])
jo = load("joint_offset")["offsets"]
lo = load("link_offset")["offsets"]
sg = load("sag")["k_rad_per_m"]

joint_off = np.array([jo.get(str(m.id), 0.0) for m in arm])
link_trans = np.zeros((n, 3)); link_rot = np.zeros((n, 3))
id_to_idx = {m.id: i for i, m in enumerate(arm)}
by_name = {}
for e in lo:
    i = id_to_idx.get(e["joint_id"])
    if i is None: continue
    link_trans[i] = e["trans_m"]; link_rot[i] = e["rot_rad"]
    by_name[arm[i].name] = (e["trans_m"], e["rot_rad"])
sag_idx = [id_to_idx[int(k)] for k in sg if int(k) in id_to_idx]
k_stiff = np.zeros(n)
for k in sg:
    if int(k) in id_to_idx:
        k_stiff[id_to_idx[int(k)]] = sg[k]

# A: raw urdf, link 은 파라미터
fk_A = FkChain(urdf_raw, arm_names)
# B: patched urdf, link 은 구움
patched = patch_urdf_link_offsets(Path(urdf_raw), ROBOT, by_name)
fk_B = FkChain(patched, arm_names)

scans = {
    "scan1": [1509, 2543, 1082, 3037, 2051, 3073],
    "scan2": [2062, 2451, 1106, 3189, 2051, 3073],
    "scan3": [1615, 3404, 366, 3189, 2051, 3073],
    "scan4": [2305, 2332, 1179, 3221, 2051, 3073],
}
print("=== A(BA/hand_eye 기준) vs B(v2 motion 클라우드) 전체 보정 FK ===")
for name, raw in scans.items():
    ja = np.array([raw_to_rad(raw[i], arm[i]) for i in range(n)]) + joint_off
    # sag: 각 방식의 geometry 로
    jaA = fk_A.apply_gravity_sag(ja, k_stiff[sag_idx], sag_idx) if sag_idx else ja
    jaB = fk_B.apply_gravity_sag(ja, k_stiff[sag_idx], sag_idx) if sag_idx else ja
    Ra, ta = fk_A.fk(jaA, link_trans, link_rot)
    Rb, tb = fk_B.fk(jaB)
    dpos = (np.asarray(tb) - np.asarray(ta)) * 1000
    print(f"{name}: pos차={np.linalg.norm(dpos):5.1f}mm ({dpos[0]:+.0f},{dpos[1]:+.0f},{dpos[2]:+.0f})  "
          f"ori차={ang(Ra, Rb):5.2f}°")
