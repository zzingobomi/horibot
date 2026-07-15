"""결정적 회귀 진단: hand_eye 를 만든 offline BA 는 FkChain(analytic numpy)으로 TCP 를
계산하고, v2 motion(라이브 클라우드/검출)은 PybulletKinematics 로 계산한다.
두 엔진이 같은 URDF·같은 관절각에서 TCP orientation 이 다르면 = 사선의 원인.

7/6 scan 관절각(DB, 그때 클라우드 평평했음)으로 두 엔진 TCP 를 나란히 비교.
하드웨어 0."""
import sys, os
import numpy as np

sys.path.insert(0, ".")
from apps.config import load_robots  # noqa: E402
from modules.motion.units import raw_to_rad  # noqa: E402
from modules.motor.contract import MotorKind  # noqa: E402
from modules.motion.fk_chain import FkChain  # noqa: E402
from modules.motion.adapters.pybullet import PybulletKinematics  # noqa: E402


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)],
    ])


def ang(Ra, Rb):
    Rrel = Ra.T @ Rb
    c = (np.trace(Rrel) - 1) / 2
    return np.degrees(np.arccos(np.clip(c, -1, 1)))


ROBOT = "so101_6dof_0"
robots = load_robots()
robot = robots[ROBOT]
arm = [m for m in robot.motors if m.kind != MotorKind.GRIPPER]
arm_names = [m.name for m in arm]
print(f"arm joints: {arm_names}")

# URDF 경로 — raw (엔진차 격리; 둘 다 같은 파일)
urdf_path = robot.urdf_path if hasattr(robot, "urdf_path") else None
if urdf_path is None:
    # fallback 탐색
    import glob
    cand = glob.glob(f"../robot/**/{robot.type if hasattr(robot,'type') else 'so101'}*.urdf", recursive=True)
    cand = [c for c in cand if "calibrated" not in c]
    urdf_path = cand[0] if cand else None
print(f"URDF: {urdf_path}")

fkchain = FkChain(urdf_path, arm_names)
pybk = PybulletKinematics(urdf_path)
pybk.initialize()

# 7/6 scan 관절각 (raw ticks, DB) — 4 자세
scans = {
    "scan1": [1509, 2543, 1082, 3037, 2051, 3073],
    "scan2": [2062, 2451, 1106, 3189, 2051, 3073],
    "scan3": [1615, 3404, 366, 3189, 2051, 3073],
    "scan4": [2305, 2332, 1179, 3221, 2051, 3073],
}
spec_by_name = {m.name: m for m in arm}
print("\n=== FkChain(analytic, hand_eye 기준) vs PyBullet(motion 클라우드) — 같은 raw URDF ===")
for name, raw in scans.items():
    jr = [raw_to_rad(raw[i], arm[i]) for i in range(len(arm))]
    Rc, tc = fkchain.fk(np.array(jr))          # FkChain: (R 3x3, t 3)
    pp, qp = pybk.fk(jr)                        # PyBullet: (pos 3, quat 4)
    tc = np.asarray(tc); pp = np.asarray(pp)
    Rp = quat_to_R(qp)
    dpos = (pp - tc) * 1000
    dang = ang(Rc, Rp)
    print(f"{name}: pos차={np.linalg.norm(dpos):5.1f}mm ({dpos[0]:+.0f},{dpos[1]:+.0f},{dpos[2]:+.0f})  "
          f"ori차={dang:5.2f}°")
