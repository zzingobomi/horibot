"""hang(z-up) 개정 전체 시퀀스 오프라인 검증 — 오늘 밤 실물 런 시나리오 그대로.

pick(먼 끝, tool z ∥ -u) → present(hang) → receive(so101) 를 production
기하/IK/충돌 체커로 재현. 각 단계 관절해(특히 omx J5)와 게이트 통과를 출력.
로봇/transport 0 — 안전.
"""

from __future__ import annotations

import json
import math
import sqlite3
import sys
from pathlib import Path


ROOT = Path(r"d:/Study/horibot")
sys.path.insert(0, str(ROOT / "backend"))

from modules.motion.adapters.pybullet import PybulletKinematics  # noqa: E402
from modules.tasks.handover import block, frames, steps  # noqa: E402
from modules.tasks.handover.collision import BasePose, CrossRobotChecker  # noqa: E402
from modules.shared_config.contract import WorkcellRoi  # noqa: E402
import yaml  # noqa: E402

BASE = BasePose(x=0.0342, y=0.2702, z=-0.0094, yaw_rad=math.radians(-3.33))
RB = ROOT / "robot"


def roi(instance: str) -> WorkcellRoi:
    y = yaml.safe_load(
        (RB / "instances" / instance / "instance.yaml").read_text(encoding="utf-8")
    )
    return WorkcellRoi(**y["workcell"])


def home(robot_id: str) -> list[float]:
    con = sqlite3.connect(str(ROOT / "backend" / "horibot.db"))
    row = con.execute(
        "SELECT joint_values FROM waypoints WHERE robot_id=? AND name='home'",
        (robot_id,),
    ).fetchone()
    con.close()
    return json.loads(row[0])


def deg(sol):
    return [round(math.degrees(v), 1) for v in sol]


kin_omx = PybulletKinematics(RB / "omx_f" / "urdf" / "omx_f.urdf")
kin_omx.initialize()
kin_so = PybulletKinematics(
    RB / "so101_6dof" / "urdf" / "so101_6dof.so101_6dof_0.calibrated.urdf"
)
kin_so.initialize()
checker = CrossRobotChecker(
    RB / "so101_6dof" / "urdf" / "so101_6dof.urdf",
    RB / "omx_f" / "urdf" / "omx_f.urdf",
    BASE,
)
roi_so, roi_omx = roi("so101_6dof_0"), roi("omx_f_0")
home_omx, home_so = home("omx_f_0"), home("so101_6dof_0")

# ── 1. pick — 오늘 밤 검출 그대로 (center 0.171,0.029 yaw 2.2°, known len 8cm)
grasp = block.plan_block_grasp(
    (0.171, 0.029), math.radians(2.2), (0.08, 0.02),
    grasp_frac=steps._BLOCK_GRASP_FRAC, jaw_along_m=steps._OMX_JAW_ALONG_M,
    exposed_frac=steps._BLOCK_EXPOSED_FRAC,
    min_exposed_m=steps._SO_MIN_GRASP_M + steps._EXPOSED_MARGIN_M,
    len_min_m=steps._BLOCK_LEN_MIN_M, len_max_m=steps._BLOCK_LEN_MAX_M,
)
print("=== 1. pick (tool z ∥ -u, 자연손목 정렬 + J5 게이트 = production) ===")
pick_sol = None
pick_meta = None
ends = sorted(grasp.ends, key=lambda eu: eu[1][0] * eu[0][0] + eu[1][1] * eu[0][1])
for (gx, gy), u in ends:
    yaw = math.atan2(-u[1], -u[0])
    quat = steps._grasp_quat(yaw, 0)
    got = None
    for dz in steps._PICK_DZ_LADDER:
        sol = kin_omx.ik((gx, gy, steps._OMX_TABLE_Z_M + dz), quat, home_omx, 40)
        if sol is not None and abs(sol[-1]) <= steps._WRIST_NATURAL_MAX_RAD:
            got = (dz, sol)
            break
    tag = "먼끝" if gx > 0.171 else "가까운끝"
    if got:
        print(f"  {tag} g=({gx:.3f},{gy:.3f}) u=({u[0]:+.2f},{u[1]:+.2f}) "
              f"dz={got[0]*1000:.0f}mm → {deg(got[1])}")
        if pick_sol is None:
            pick_sol, pick_meta = got[1], (gx, gy, u)
    else:
        print(f"  {tag} g=({gx:.3f},{gy:.3f}) u=({u[0]:+.2f},{u[1]:+.2f}) → "
              "자연해 없음 (게이트 기각 포함)")

# ── 2. present — rendezvous 후보 × hang quat + 벽/충돌 게이트
print("\n=== 2. present hang(z↑) — 랑데부 후보 ===")
cands = frames.rendezvous_candidates(
    roi_so, roi_omx, BASE, steps._PRESENT_Z_WORLD,
    limit=steps._PRESENT_LIMIT, prefer_point=steps._RENDEZVOUS_PREFER_XY,
)
present_sol = None
present_tcp = None
for tcp_w in cands:
    e = (tcp_w[0], tcp_w[1], tcp_w[2] - grasp.tcp_to_e_m)
    if not (roi_so.x_min <= e[0] <= roi_so.x_max
            and roi_so.y_min <= e[1] <= roi_so.y_max
            and roi_so.z_min <= e[2] <= roi_so.z_max):
        print(f"  tcp={tuple(round(v,3) for v in tcp_w)}: E ROI 밖")
        continue
    tcp_omx = steps.world_to_robot(tcp_w, BASE)
    alpha = math.atan2(tcp_omx[1], tcp_omx[0])
    quat = steps._present_quat_hang(alpha)
    sol = kin_omx.ik(tuple(tcp_omx), quat, pick_sol or home_omx, 40)
    if sol is None:
        print(f"  tcp={tuple(round(v,3) for v in tcp_w)}: IK 실패")
        continue
    wall = steps._behind_wall(checker, "b", sol)
    coll = steps._omx_path_collides(checker, home_so, [pick_sol or home_omx, sol])
    print(f"  tcp={tuple(round(v,3) for v in tcp_w)} α={math.degrees(alpha):.0f}° "
          f"→ {deg(sol)} 벽={wall} 충돌={coll}")
    if not wall and not coll and present_sol is None:
        present_sol, present_tcp = sol, tcp_w

# ── 3. receive — E 에서 so101 수취족 [pre, grasp] + 충돌
print("\n=== 3. so101 receive — 수직 조축 spin 사다리 ===")
if present_sol is None:
    print("  (present 후보 없음 — 중단)")
    sys.exit(1)
e = (present_tcp[0], present_tcp[1], present_tcp[2] - grasp.tcp_to_e_m)
print(f"  E={tuple(round(v,3) for v in e)}")
ok = 0
for label, quat, a in steps._grasp_orients(e, (0.0, 0.0, -1.0))[:8]:
    for clear_m in steps._RECV_PRE_CLEAR_LADDER:  # 접근 여유 사다리
        pre = tuple(e[i] - a[i] * clear_m for i in range(3))
        s1 = kin_so.ik(pre, quat, home_so, 40)
        if s1 is None:
            continue
        s2 = kin_so.ik(e, quat, s1, 40)
        if s2 is None:
            continue
        coll = checker.path_in_collision(
            [s1, s2], present_sol, grip_b=steps._OMX_HOLD_GRIP_FRAC,
            margin_m=steps._RECV_COLLISION_MARGIN_M,
        )
        wall = steps._behind_wall(checker, "a", s2)
        print(f"  {label} (여유 {clear_m * 100:.0f}cm): pre+grasp 도달, "
              f"충돌={coll} 벽={wall}"
              f"{'  ← 채택가능' if not coll and not wall else ''}")
        if not coll and not wall:
            ok += 1
        break
print(f"\n수취 가능 자세: {ok}개")
print("\n=== 요약: J5 여정 ===")
print(f"  home J5={deg(home_omx)[4]}° → pick J5={deg(pick_sol)[4] if pick_sol else '?'}°"
      f" → present J5={deg(present_sol)[4]}°")
