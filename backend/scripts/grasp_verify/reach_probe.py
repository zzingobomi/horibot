"""깨끗한 큐브(검출 OBB)에 대해, 각 tilt 파지가 IK 도달 + 바닥(몸통 포함) 클리어 인지
오프라인으로 직접 찍는다. '위에서 사선 파지가 되는데 코드가 못 찾은 거냐 vs 물리 한계냐'.
raw kinematics(캘 미세보정 무시 — 도달성 coarse 판정엔 충분). 하드웨어 0."""
import sys, math, json, glob, os
import numpy as np
sys.path.insert(0, ".")
from apps.config import load_robots, _ROBOT_DIR
from modules.motor.contract import MotorKind
from modules.motion.adapters.pybullet import PybulletKinematics
from modules.tasks.pick_and_place import antipodal, geometry

ROBOT = "so101_6dof_0"
robot = load_robots()[ROBOT]
urdf = _ROBOT_DIR / robot.type / "urdf" / f"{robot.type}.urdf"
kin = PybulletKinematics(urdf)
kin.initialize()

# 최신 세션 큐브 검출 → 중심/yaw
sess = sorted(glob.glob("debug/detect/*/*cube*.json"))
m = json.load(open(sorted(glob.glob("debug/detect/20260715_230340/000[6]_*cube*.json"))[0], encoding="utf-8"))
c0 = m["candidates"][0]
cx, cy, _ = c0["position"]
yaw = c0.get("grasp_yaw", 0.0)
h = c0["height"]
zc = h / 2  # 중심 높이
print(f"큐브: pos=({cx:.3f},{cy:.3f}) yaw={math.degrees(yaw):.0f}° height={h*1000:.0f}mm 중심z={zc*1000:.0f}mm")

# 합성 antipodal 쌍 (깨끗) — 큐브 옆면 두 방향(yaw 정렬), 폭 20mm, 중심에서
W = 0.020
mid = np.array([cx, cy, zc])
pairs = []
for face in (0, math.pi/2):
    a = yaw + face
    jaw = np.array([math.cos(a), math.sin(a), 0.0])
    pairs.append(antipodal.AntipodalPair(mid=tuple(mid), jaw_axis=tuple(jaw), width=W))

cands = geometry.plan_grasp(pairs)
print(f"\n생성 후보 {len(cands)}개. tilt별 도달/바닥클리어:")
by_tilt = {}
for c in cands:
    # label 에서 tilt 추출
    tl = c.label.split("tilt=")[1].split(" ")[0]
    gpos = list(c.grasp); gq = list(c.quat)
    sol = kin.ik(gpos, gq)
    reach = sol is not None
    floor = kin.floor_collision(sol, 0.0) if sol is not None else None
    key = tl
    prev = by_tilt.get(key, (False, None))
    ok = reach and (floor is False)
    if ok or (reach and not prev[0]):
        by_tilt[key] = (reach, floor)
    by_tilt.setdefault(key, (reach, floor))

for tl in ["+0","+15","-15","+30","-30","+45","-45","+60","-60","+75","-75","+90","-90"]:
    if tl in by_tilt:
        reach, floor = by_tilt[tl]
        tag = "✓도달+바닥OK" if (reach and floor is False) else ("도달O 바닥충돌" if reach else "IK실패")
        print(f"  tilt={tl:>4}: {tag}")
kin.close()
