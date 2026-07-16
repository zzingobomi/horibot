"""어제 실물 debug 데이터의 **실제 관측 큐브 위치**에, 새 closed-loop(servo)
계획이 만드는 파지 자세 가족을 실 URDF IK 로 재생 — "plan_pick 이 IK 전멸로
못 집는가?" 를 집 가기 전 회사에서 데이터로 확인 (하드웨어 0).

각 큐브 관측마다:
  - servo.grasp_families 52개 × [standoff 사다리…, 파지] 를 실제로 생성
  - 실 URDF IK 로 각 pose 도달성 + 바닥 + 그리퍼(벌림)↔물체 점군 충돌 판정
  - "IK 만" / "IK+바닥+장애물" 두 층으로 나눠 몇 가족이 사는지 보고

두 층을 나누는 이유(핵심): post-mortem 가정 D = "게이트가 물리적으로 닿는 pose 를
거부". IK 만 통과인데 게이트에서 다 죽으면 그 층이 범인이고, IK 부터 전멸이면
그 위치·그 캘로는 자세 자체가 도달 불가 — 원인이 갈린다.

실행: backend 에서 .venv\\Scripts\\python.exe scripts\\grasp_verify\\servo_reach_replay.py [session]
"""
import glob
import json
import os
import sys

import numpy as np

sys.path.insert(0, ".")
from apps.config import _ROBOT_DIR, load_robots  # noqa: E402
from modules.motion.adapters.pybullet import PybulletKinematics  # noqa: E402
from modules.detector.contract import OrientedDetection  # noqa: E402
from modules.tasks.pick_and_place import servo  # noqa: E402

SESSION = sys.argv[1] if len(sys.argv) > 1 else "20260715_234827"
D = f"debug/detect/{SESSION}"
CFG = servo.ServoConfig()
FLOOR_MARGIN = 0.005


def load_ply(path: str) -> np.ndarray | None:
    if not os.path.exists(path):
        return None
    pts = []
    with open(path) as f:
        body = False
        for line in f:
            if body:
                p = line.split()
                if len(p) >= 3:
                    pts.append([float(p[0]), float(p[1]), float(p[2])])
            elif line.strip() == "end_header":
                body = True
    return np.array(pts) if pts else None


def build_coarse(meta: dict, ply_path: str) -> OrientedDetection | None:
    c = meta["candidates"][0]
    pts = load_ply(ply_path)
    return OrientedDetection(
        prompt=meta["prompt"],
        position=tuple(c["position"]),
        score=c["score"],
        base_z=c["base_z"],
        height=c["height"],
        grasp_yaw=c["grasp_yaw"],
        footprint=tuple(c["footprint"]),
        points=[tuple(p) for p in pts] if pts is not None else None,
    )


def probe(kin: PybulletKinematics, coarse: OrientedDetection):
    """52 가족 × 사다리 pose 를 실 URDF IK 로 찍는다 → (ik_ok, gated_ok, 상세)."""
    families = servo.grasp_families(coarse)
    g_point = servo.grasp_point(coarse, coarse, CFG)
    floor_z = coarse.base_z - FLOOR_MARGIN
    pts = coarse.points or []
    kin.set_obstacle_points([tuple(p) for p in pts] if pts else None)

    ik_ok, gated_ok = [], []
    try:
        for fam in families:
            width = servo.width_along(pts, fam.jaw_axis, coarse.footprint[1])
            lateral = servo.lateral_offset(width)
            g_tcp = servo.grasp_tcp(g_point, fam, lateral)
            poses = [servo.standoff(g_tcp, fam, s) for s in CFG.standoffs]
            poses.append(g_tcp)

            # ① 전 pose IK 도달 (seed 연쇄 — motion resolve 와 동형)
            seed = None
            sols = []
            for pos in poses:
                sol = kin.ik(pos, fam.quat, seed)
                if sol is None:
                    break
                sols.append(sol)
                seed = sol
            if len(sols) != len(poses):
                continue
            ik_ok.append(fam)

            # ② 바닥 + 그리퍼(벌림)↔물체 점군 충돌 (파지 해 기준)
            grasp_sol = sols[-1]
            if kin.floor_collision(grasp_sol, floor_z):
                continue
            if pts and kin.obstacle_collision(grasp_sol, gripper_open=True):
                continue
            gated_ok.append(fam)
    finally:
        kin.set_obstacle_points(None)
    return ik_ok, gated_ok


def tilt_summary(fams) -> str:
    if not fams:
        return "없음"
    tilts = sorted({f.tilt_deg for f in fams})
    return "tilt " + ",".join(f"{t:+d}" for t in tilts) + f"° ({len(fams)}가족)"


def main() -> None:
    robot = load_robots()["so101_6dof_0"]
    urdf = _ROBOT_DIR / robot.type / "urdf" / f"{robot.type}.urdf"
    kin = PybulletKinematics(urdf)
    kin.initialize()
    print(f"=== {SESSION}: 실제 관측 큐브 위치에서 servo 파지 가족 IK 도달성 ===")
    jsons = sorted(glob.glob(os.path.join(D, "*cube*.json")))
    any_reach = False
    for jf in jsons:
        meta = json.load(open(jf, encoding="utf-8"))
        ply = jf.replace(".json", "_c0.ply")
        coarse = build_coarse(meta, ply)
        if coarse is None:
            continue
        p = coarse.position
        ik_ok, gated_ok = probe(kin, coarse)
        tag = "✓ 도달" if gated_ok else ("△ IK만" if ik_ok else "✗ 전멸")
        print(
            f"\n{os.path.basename(jf)[:4]} pos=({p[0]:.3f},{p[1]:.3f},{p[2]:.3f}) "
            f"base_z={coarse.base_z:.3f} pts={len(coarse.points or [])}  [{tag}]"
        )
        print(f"    IK 도달       : {tilt_summary(ik_ok)}")
        print(f"    IK+바닥+장애물: {tilt_summary(gated_ok)}")
        if gated_ok:
            any_reach = True
    print("\n" + "=" * 60)
    print(
        "판정: 최소 한 위치라도 '도달' 가족이 있으면 → 그 위치에서 plan_pick 은 "
        "servo 진입점을 찾는다.\n전멸이면 → 그 위치·그 캘로는 새 설계도 못 집음 "
        "(손으로 닿으면 게이트 버그 = 가정 D, 집 가기 전 고칠 것)."
    )
    print("전체:", "일부 위치 도달 가능" if any_reach else "⚠ 전 위치 전멸")
    kin.close()


if __name__ == "__main__":
    main()
