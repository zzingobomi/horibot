"""handover 랑데부 **재특성화** — omx 를 물리적으로 옮긴 뒤 노브를 다시 뽑는다.

2026-07-28 신설. 배치를 바꾸면 `steps._RENDEZVOUS_PREFER_XY` / `_PRESENT_Z_WORLD`
가 통째로 무효가 된다 (옛 값 (0.21,0.16) 은 omx 가 so101 옆에 나란히 있던 배치의
probe 산출물). 이 스크립트는 **robots.yaml 의 현재 base_pose 를 그대로 읽어**
결합 게이트를 스윕하고 **그대로 붙여넣을 노브 값**을 출력한다.

실행 순서 (물리 이전 후):
  1. omx 를 새 자리에 마운트
  2. `uv run --no-sync python scripts/cross_calibrate.py` → 리포트의 base_pose
     블록을 robot/robots.yaml 에 반영 (실측값 — 손으로 짐작한 값 금지)
  3. `uv run --no-sync python scripts/handover_layout_tune.py`
     → 출력된 _RENDEZVOUS_PREFER_XY / _PRESENT_Z_WORLD 를 steps.py 에 반영
  4. `uv run --no-sync pytest tests/modules/test_handover_feasibility.py -q`

게이트 (production 과 동형 — steps.plan_omx_present / plan_receive):
  ① omx hang(z↑) 제시 도달 + 손목 뒤집힘 기각 (_WRIST_NATURAL_MAX_RAD)
  ② omx 벽(뒤) — 링크 world x ≥ _WALL_MIN_X_M
  ③ E(so101 파지점) ∈ so101 workcell ROI
  ④ so101 수취 도달 — _recv_orients × **접근 여유 사다리** [pre, grasp] IK
  ⑤ cross-robot 충돌 여유 (_RECV_COLLISION_MARGIN_M)
  ⑥ **노출부 전체** 시선 비가림 (⚠ 옛 probe 는 E±25mm 만 봐서 실물 가림을
     놓쳤다 — handover_block_probe.ray_occluded 주석)

⚠ pybullet 클라이언트를 3개(so101 IK / omx IK / 충돌 세계) 띄운다. 실행 중
backend 를 함께 띄우지 말 것 (CLAUDE.md 유령 backend 주의).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import pybullet as p
import yaml
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from modules.motion.adapters.pybullet import PybulletKinematics  # noqa: E402
from modules.tasks.handover import block, steps  # noqa: E402
from modules.tasks.handover.collision import (  # noqa: E402
    BasePose,
    CrossRobotChecker,
)
from modules.tasks.handover.frames import world_to_robot  # noqa: E402

_ROOT = Path(__file__).resolve().parents[2]
_ROBOT = _ROOT / "robot"

# 노출부 기하 — steps 노브에서 유도 (봉 스펙이 바뀌면 자동 추종)
_GEOM = block.plan_block_grasp(
    (0.20, 0.0), 0.0, (steps._BLOCK_LEN_M, steps._BLOCK_CROSS_M),
    grasp_frac=steps._BLOCK_GRASP_FRAC,
    jaw_along_m=steps._OMX_JAW_ALONG_M,
    exposed_frac=steps._BLOCK_EXPOSED_FRAC,
    min_exposed_m=steps._SO_MIN_GRASP_M + steps._EXPOSED_MARGIN_M,
    len_min_m=steps._BLOCK_LEN_MIN_M,
    len_max_m=steps._BLOCK_LEN_MAX_M,
)
IK_BUDGET = 40
OBS_SAMPLES = 11  # 노출부 시선 표본 수


def _load(name: str) -> dict:
    return yaml.safe_load(
        (_ROBOT / "instances" / name / "instance.yaml").read_text(encoding="utf-8")
    )["workcell"]


def _base_omx() -> BasePose:
    reg = yaml.safe_load((_ROBOT / "robots.yaml").read_text(encoding="utf-8"))
    bp = reg["robots"]["omx_f_0"]["base_pose"]
    return BasePose(bp["x"], bp["y"], bp["z"], math.radians(bp["yaw_deg"]))


def _hand_eye_so() -> np.ndarray:
    import sqlite3

    con = sqlite3.connect(str(_ROOT / "backend" / "horibot.db"))
    row = con.execute(
        "SELECT result_data FROM calibration_results "
        "WHERE robot_id='so101_6dof_0' AND kind='hand_eye' AND is_active=1",
    ).fetchone()
    con.close()
    if row is None:
        raise SystemExit("so101 active hand_eye 없음 — 관측 게이트 계산 불가")
    d = json.loads(row[0])
    x = np.eye(4)
    x[:3, :3] = np.array(d["R_cam2gripper"], dtype=float)
    x[:3, 3] = np.array(d["t_cam2gripper"], dtype=float).reshape(3)
    return x


def _exposed_targets(tcp_w: np.ndarray) -> list[np.ndarray]:
    """봉 노출부 표본 (world) — 조 끝부터 봉 아래끝까지 수직 매달림."""
    top = tcp_w[2] - steps._OMX_JAW_ALONG_M / 2.0
    return [
        np.array([tcp_w[0], tcp_w[1],
                  top - _GEOM.exposed_len_m * k / (OBS_SAMPLES - 1)])
        for k in range(OBS_SAMPLES)
    ]


def _blocked_frac(ck: CrossRobotChecker, sa, sb, cam, targets) -> float:
    ck.in_collision(sa, sb, grip_a=1.0, grip_b=steps._OMX_HOLD_GRIP_FRAC)
    cam = np.asarray(cam, float)
    st, en = [], []
    for t in targets:
        v = t - cam
        v = v / np.linalg.norm(v)
        st.append(tuple(cam + v * 0.02))  # 자기 wrist mesh 즉발 오탐 회피
        en.append(tuple(t))
    hits = p.rayTestBatch(st, en, physicsClientId=ck._client)
    return sum(1 for h in hits if h[0] >= 0 and h[2] < 0.97) / len(targets)


def _camera_poses(e: np.ndarray, x_ce: np.ndarray):
    """steps.plan_so_observe 와 같은 (az_off × elev × dist × ψ) 사다리."""
    az0 = math.atan2(e[1], e[0])
    x_inv = np.linalg.inv(x_ce)
    for az_off in steps._RECV_OBS_AZOFF_DEG:
        for elev_deg in steps._RECV_OBS_ELEV_DEG:
            for dist in steps._RECV_OBS_DIST_M:
                az, elev = az0 + math.radians(az_off), math.radians(elev_deg)
                c = np.array([
                    e[0] - math.cos(az) * dist * math.cos(elev),
                    e[1] - math.sin(az) * dist * math.cos(elev),
                    e[2] + dist * math.sin(elev),
                ])
                z = (e - c) / np.linalg.norm(e - c)
                horiz = np.cross(z, np.array([0.0, 0.0, 1.0]))
                x0 = (np.array([1.0, 0.0, 0.0]) if np.linalg.norm(horiz) < 1e-6
                      else horiz / np.linalg.norm(horiz))
                y0 = np.cross(z, x0)
                for psi_deg in steps._RECV_OBS_PSI_DEG:
                    psi = math.radians(psi_deg)
                    x = math.cos(psi) * x0 + math.sin(psi) * y0
                    tb = np.eye(4)
                    tb[:3, :3] = np.column_stack([x, np.cross(z, x), z])
                    tb[:3, 3] = c
                    t = tb @ x_inv
                    q = Rotation.from_matrix(t[:3, :3]).as_quat()
                    yield (
                        (float(t[0, 3]), float(t[1, 3]), float(t[2, 3])),
                        tuple(float(v) for v in q),
                        tuple(float(v) for v in c),
                    )


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--step", type=float, default=0.02, help="world 격자 간격 m")
    ap.add_argument("--json", type=Path, default=None)
    ap.add_argument(
        "--base", default=None, metavar="X,Y,YAW_DEG",
        help="base_pose 오버라이드 — **마운트 전 what-if 용**. 실측 대신 쓰지 말 것 "
             "(실측은 cross_calibrate.py → robots.yaml).",
    )
    args = ap.parse_args()

    if args.base:
        bx, by, byaw = (float(v) for v in args.base.split(","))
        base = BasePose(bx, by, _base_omx().z, math.radians(byaw))
        print(f"⚠ base_pose 오버라이드 (what-if — 실측 아님): "
              f"({bx}, {by}) yaw {byaw}°", flush=True)
    else:
        base = _base_omx()
    roi_so, roi_omx = _load("so101_6dof_0"), _load("omx_f_0")
    x_ce = _hand_eye_so()
    ks = PybulletKinematics(
        _ROBOT / "so101_6dof" / "urdf"
        / "so101_6dof.so101_6dof_0.calibrated.urdf")
    ks.initialize()
    ko = PybulletKinematics(_ROBOT / "omx_f" / "urdf" / "omx_f.urdf")
    ko.initialize()
    ck = CrossRobotChecker(
        _ROBOT / "so101_6dof" / "urdf" / "so101_6dof.urdf",
        _ROBOT / "omx_f" / "urdf" / "omx_f.urdf", base,
    )
    print(f"omx base_pose (robots.yaml) = ({base.x:.4f}, {base.y:.4f}, "
          f"{base.z:.4f}) yaw {math.degrees(base.yaw_rad):.2f}°", flush=True)
    print(f"봉 노출 {_GEOM.exposed_len_m * 1000:.0f}mm / "
          f"tcp→E {_GEOM.tcp_to_e_m * 1000:.1f}mm", flush=True)

    xs = np.arange(roi_so["x_min"], roi_so["x_max"] + 1e-9, args.step)
    ys = np.arange(roi_so["y_min"], roi_so["y_max"] + 1e-9, args.step)
    rows = []
    for tz in steps._PRESENT_Z_WORLD:
        for tx in xs:
            for ty in ys:
                t_w = np.array([tx, ty, tz])
                t_o = world_to_robot(tuple(t_w), base)
                if not (roi_omx["x_min"] <= t_o[0] <= roi_omx["x_max"]
                        and roi_omx["y_min"] <= t_o[1] <= roi_omx["y_max"]
                        and roi_omx["z_min"] <= t_o[2] <= roi_omx["z_max"]):
                    continue
                alpha = math.atan2(t_o[1], t_o[0])
                q_omx = steps._present_quat_hang(alpha)
                s_omx = ko.ik(t_o, q_omx, [0.0] * ko.dof, IK_BUDGET)
                if s_omx is None:
                    continue
                if abs(s_omx[-1]) > steps._WRIST_NATURAL_MAX_RAD:
                    continue  # ① 손목 뒤집힘 (케이블 안전)
                if ck.min_link_world_x(
                        "b", s_omx, grip=steps._OMX_HOLD_GRIP_FRAC
                ) < steps._WALL_MIN_X_M:
                    continue  # ② 벽(뒤)
                e = np.array([t_w[0], t_w[1], t_w[2] - _GEOM.tcp_to_e_m])
                if not (roi_so["x_min"] <= e[0] <= roi_so["x_max"]
                        and roi_so["y_min"] <= e[1] <= roi_so["y_max"]
                        and roi_so["z_min"] <= e[2] <= roi_so["z_max"]):
                    continue  # ③ E ∈ so101 ROI
                # ④ so101 수취 (자세족 × 접근 여유 사다리) — 통과 개수 = 강건성
                n_recv, s_so, best_clear = 0, None, None
                for _label, quat, a in steps._recv_orients(tuple(e)):
                    for clear in steps._RECV_PRE_CLEAR_LADDER:
                        pre = tuple(e[i] - a[i] * clear for i in range(3))
                        s1 = ks.ik(pre, quat, [0.0] * ks.dof, IK_BUDGET)
                        if s1 is None:
                            continue
                        s2 = ks.ik(tuple(e), quat, s1, IK_BUDGET)
                        if s2 is None:
                            continue
                        n_recv += 1
                        if s_so is None:
                            s_so, best_clear = s2, clear
                        break
                if s_so is None:
                    continue
                # ⑤ 충돌 여유
                if ck.in_collision(
                    s_so, s_omx, grip_a=1.0,
                    grip_b=steps._OMX_HOLD_GRIP_FRAC,
                    margin_m=steps._RECV_COLLISION_MARGIN_M,
                ):
                    continue
                # ⑥ 노출부 전체 비가림 관측 자세 수
                tg = _exposed_targets(t_w)
                n_clear = 0
                for tp, tq, cam in _camera_poses(e, x_ce):
                    s_obs = ks.ik(tp, tq, [0.0] * ks.dof, 10)
                    if s_obs is None:
                        continue
                    if _blocked_frac(ck, s_obs, s_omx, cam, tg) == 0.0:
                        n_clear += 1
                if n_clear == 0:
                    continue
                gap = min((c[8] for c in p.getClosestPoints(
                    bodyA=ck._a.body, bodyB=ck._b.body, distance=0.3,
                    physicsClientId=ck._client)), default=0.3)
                rows.append({
                    "tcp": [round(float(v), 4) for v in t_w],
                    "E": [round(float(v), 4) for v in e],
                    "n_recv": n_recv, "n_obs_clear": n_clear,
                    "gap_mm": round(gap * 1000, 1),
                    "pre_clear_m": best_clear,
                })
    ck.close()
    ks.close()
    ko.close()

    if not rows:
        raise SystemExit(
            "전 후보 전멸 — base_pose/workcell ROI 를 확인하세요. omx 를 so101 "
            "공통 워크스페이스가 생기는 자리로 옮겨야 합니다."
        )
    # 선호: 비가림 관측 자세 많고 → 수취 자세 많고 → 간격 넓은 순
    rows.sort(key=lambda r: (-r["n_obs_clear"], -r["n_recv"], -r["gap_mm"]))
    best = rows[0]
    print(f"\n통과 랑데부 {len(rows)}개. 상위 8개:", flush=True)
    for r in rows[:8]:
        print(f"  TCP {r['tcp']}  E {r['E']}  비가림관측 {r['n_obs_clear']}  "
              f"수취자세 {r['n_recv']}  간격 {r['gap_mm']}mm  "
              f"접근여유 {r['pre_clear_m'] * 100:.0f}cm", flush=True)
    zs = sorted({r["tcp"][2] for r in rows},
                key=lambda z: -sum(1 for r in rows if r["tcp"][2] == z))
    print("\n─── steps.py 에 반영할 값 ───", flush=True)
    print(f"_PRESENT_Z_WORLD = {tuple(zs)}", flush=True)
    print(f"_RENDEZVOUS_PREFER_XY = ({best['tcp'][0]}, {best['tcp'][1]})",
          flush=True)
    if args.json:
        args.json.write_text(json.dumps(rows, ensure_ascii=False, indent=2),
                             encoding="utf-8")
        print(f"\nJSON -> {args.json}", flush=True)


if __name__ == "__main__":
    main()
