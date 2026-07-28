"""handover 봉(block) 제시 자세 결합 probe — 수평(접선)족 vs 수직족을 숫자로 결정.

2026-07-27 신설 (8×2×2cm 봉 전환 설계 게이트). 큐브 시대의 벽(M2 도달 razor-thin
/ M3 도달↔가림 y축 정면충돌 — docs/omx_handover_realtest_handoff.md §T.3)이 봉
전환으로 실제로 풀리는지, 어느 제시 자세족이 결합 조건을 만족하는지 offline 로
판정한다. **모델이 실물과 같을 때만 유효** (§T.6 — omx 카메라 box 를 URDF 에
넣은 뒤에야 이 probe 를 신뢰할 수 있다. 2026-07-27 M1 반영 완료 상태로 실행).

⚠ 자세 후보는 omx 5DOF **도달 다양체 위에서 구성**한다 (1차 probe 의 교훈:
임의 세계 방향 d 에 tool z ∥ d 를 요구하면 5DOF 에선 measure-zero 라 전 후보
IK 실패 — 물리가 아니라 질문이 잘못된 것). ZYYYX 기구학상 TCP 가 정하는 팔
평면(방위 α)에서 가능한 방위 = tool x 는 평면 내 고도 θ 자유, tool z 는 J5
roll 자유. 따라서:
  - 수평 봉(A) = tool z ∥ **접선**(α±90°) — 펜-era 접선족의 부활 (봉 노출 끝이
    접선 방향을 향한다. so101 쪽을 향하려면 TCP 방위 α≈0, 즉 omx 정면).
  - 수직 봉(B) = tool z ∥ ∓z_world (tool x 수평일 때 J5 roll 90°) — 노출 끝
    아래(B-down)는 중력 모멘트 0 (매달림) 이라 파지 안정 보너스.

결합 판정 (한 후보 = TCP×자세 — E(so101 파지점)는 TCP+d·tcp_to_e 로 파생):
  ① omx 제시 도달 (analytic IK, 다양체 위 구성이라 실패 = 관절 리밋/자기충돌)
  ② omx 벽(뒤) 게이트 — 링크 world x ≥ _WALL_MIN_X
  ③ E 가 so101 workcell ROI 안
  ④ so101 수취 도달 — [pre, grasp] IK, tool z ∥ ±d, spin 사다리
  ⑤ cross-robot 충돌 — 채택 해 쌍 링크 간 margin (production checker 그대로)
  ⑥ so101 관측 도달 + 시선 가림 — 관측 카메라 pose IK + E 시선 rayTest
    (omx 몸통/카메라 box + so101 자기 팔 모두 차단원)

production 파트 재사용: PybulletKinematics(analytic IK 포함) / CrossRobotChecker /
캘 DB hand_eye / instance.yaml workcell ROI. IK budget = motion 모듈과 동일급.

실행 (backend/, 로봇 불필요 — 집 재특성화 시 그대로 재실행):
    uv run --no-sync python scripts/handover_block_probe.py
    uv run --no-sync python scripts/handover_block_probe.py --json out.json
"""

from __future__ import annotations

import argparse
import json
import math
import sqlite3
import sys
import time
from pathlib import Path

import numpy as np
import pybullet as p
import yaml
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from modules.motion.adapters.pybullet import PybulletKinematics  # noqa: E402
from modules.tasks.handover.collision import (  # noqa: E402
    BasePose,
    CrossRobotChecker,
)

_ROOT = Path(__file__).resolve().parents[2]
_ROBOT = _ROOT / "robot"
_DB = _ROOT / "backend" / "horibot.db"

# ─── 봉 기하 (steps.py 봉 노브와 같은 값 — 전환 구현의 기준치) ────────
BLOCK_LEN_M = 0.08
BLOCK_W_M = 0.02
GRASP_FRAC = 0.20  # omx 파지점 = 먼 끝에서 20% (1.6cm)
JAW_ALONG_M = 0.020  # omx 조가 봉 축 방향으로 차지하는 폭
EXPOSED_FRAC = 0.65  # so101 파지점 = 노출 세그먼트의 조-쪽 끝에서 65% 지점
# omx TCP → so101 파지점 E 거리 (봉 축 방향)
_g = GRASP_FRAC * BLOCK_LEN_M
_exposed = BLOCK_LEN_M - _g - JAW_ALONG_M / 2.0
TCP_TO_E_M = JAW_ALONG_M / 2.0 + EXPOSED_FRAC * _exposed

# ─── 후보 격자 (omx TCP, world) ──────────────────────────────────────
TCP_XS = (0.12, 0.15, 0.18, 0.21, 0.24)
TCP_YS = (0.08, 0.12, 0.16, 0.20, 0.24, 0.28, 0.32)
TCP_ZS = (0.28, 0.30, 0.32)
THETA_DEG = (0.0, 15.0, 30.0, 45.0, 60.0)  # tool x 평면 내 고도 (0=수평 접근)
SPIN_DEG = tuple(float(v) for v in range(0, 360, 45))  # so101 수취 spin
IK_BUDGET = 40  # motion _GROUP_IK_BUDGET 동급
WALL_MIN_X = -0.03  # steps._WALL_MIN_X_M
RECV_PRE_CLEAR = 0.07  # steps._RECV_PRE_CLEAR_LADDER 의 최대(선호) 값
COLLISION_MARGIN = 0.008  # steps._RECV_COLLISION_MARGIN_M
OMX_HOLD_GRIP = 0.2  # steps._OMX_HOLD_GRIP_FRAC
# 관측 후보 — steps._RECV_OBS_* 동일
OBS_DIST = (0.18, 0.15)
OBS_ELEV = (30.0, 25.0)
OBS_AZOFF = (20.0, 0.0, 40.0, -20.0)
OBS_PSI = (0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0)


def load_base_omx() -> BasePose:
    reg = yaml.safe_load((_ROBOT / "robots.yaml").read_text(encoding="utf-8"))
    bp = reg["robots"]["omx_f_0"]["base_pose"]
    return BasePose(bp["x"], bp["y"], bp["z"], math.radians(bp["yaw_deg"]))


def load_roi(instance: str) -> dict:
    y = yaml.safe_load(
        (_ROBOT / "instances" / instance / "instance.yaml").read_text(
            encoding="utf-8")
    )
    return y["workcell"]


def load_hand_eye(robot_id: str) -> np.ndarray:
    con = sqlite3.connect(str(_DB))
    row = con.execute(
        "SELECT result_data FROM calibration_results "
        "WHERE robot_id=? AND kind='hand_eye' AND is_active=1",
        (robot_id,),
    ).fetchone()
    con.close()
    if row is None:
        raise SystemExit(f"{robot_id} active hand_eye 없음 — probe 불가")
    d = json.loads(row[0])
    x = np.eye(4)
    x[:3, :3] = np.array(d["R_cam2gripper"], dtype=float)
    x[:3, 3] = np.array(d["t_cam2gripper"], dtype=float).reshape(3)
    return x


def world_to_omx(pt: tuple, base: BasePose) -> tuple:
    c, s = math.cos(base.yaw_rad), math.sin(base.yaw_rad)
    dx, dy, dz = pt[0] - base.x, pt[1] - base.y, pt[2] - base.z
    return (c * dx + s * dy, -s * dx + c * dy, dz)


def dir_omx_to_world(d: np.ndarray, base: BasePose) -> np.ndarray:
    c, s = math.cos(base.yaw_rad), math.sin(base.yaw_rad)
    return np.array([c * d[0] - s * d[1], s * d[0] + c * d[1], d[2]])


def present_orients(
    tcp_omx: tuple,
) -> list[tuple[str, tuple, np.ndarray]]:
    """omx 5DOF 도달 다양체 위 제시 자세 후보 → (라벨, quat(omx), tool z(omx)).

    tool x = 팔 평면(방위 α) 내 고도 θ / tool z = 접선(±, A족) 또는
    수직(∓z, B족 — tool x 수평일 때만). 전부 구성상 다양체 위 — IK 실패는
    관절 리밋/자기충돌 만이다.
    """
    alpha = math.atan2(tcp_omx[1], tcp_omx[0])
    out: list[tuple[str, tuple, np.ndarray]] = []

    def q_of(x: np.ndarray, z: np.ndarray) -> tuple:
        y = np.cross(z, x)
        q = Rotation.from_matrix(np.column_stack([x, y, z])).as_quat()
        return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))

    for theta_deg in THETA_DEG:
        th = math.radians(theta_deg)
        x = np.array(
            [math.cos(alpha) * math.cos(th),
             math.sin(alpha) * math.cos(th), -math.sin(th)]
        )
        for sgn, sl in ((1.0, "+t"), (-1.0, "-t")):
            z = sgn * np.array([-math.sin(alpha), math.cos(alpha), 0.0])
            out.append((f"A/{sl}/th{theta_deg:.0f}", q_of(x, z), z))
    x0 = np.array([math.cos(alpha), math.sin(alpha), 0.0])
    out.append(("B/down", q_of(x0, np.array([0.0, 0.0, -1.0])),
                np.array([0.0, 0.0, -1.0])))
    out.append(("B/up", q_of(x0, np.array([0.0, 0.0, 1.0])),
                np.array([0.0, 0.0, 1.0])))
    return out


def quat_tool_z(d: np.ndarray, spin_deg: float) -> tuple:
    """tool z ∥ d + spin(사다리) → quat [x,y,z,w]. spin 0 = tool x 최하향.
    so101(6DOF) 수취 자세용 — 6축은 임의 방위가 성립한다."""
    z = d / np.linalg.norm(d)
    down = np.array([0.0, 0.0, -1.0])
    x0 = down - z * np.dot(down, z)
    if np.linalg.norm(x0) < 1e-6:  # d 수직 → tool x 기준 world +x
        ref = np.array([1.0, 0.0, 0.0])
        x0 = ref - z * np.dot(ref, z)
    x0 = x0 / np.linalg.norm(x0)
    x = Rotation.from_rotvec(z * math.radians(spin_deg)).apply(x0)
    y = np.cross(z, x)
    q = Rotation.from_matrix(np.column_stack([x, y, z])).as_quat()
    return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))


def camera_tcp_poses(
    e: np.ndarray, t_tcp_cam: np.ndarray
) -> list[tuple[tuple, tuple, tuple]]:
    """E 를 보는 관측 카메라 pose 후보 → (tcp_pos, tcp_quat, cam_pos) 목록.

    steps.plan_so_observe 와 같은 (az_off × elev × dist × ψ) 사다리 —
    az0 = base 원점→E 방위 (so101 base = world 원점).
    """
    az0 = math.atan2(e[1], e[0])
    out = []
    x_inv = np.linalg.inv(t_tcp_cam)
    for az_off in OBS_AZOFF:
        for elev_deg in OBS_ELEV:
            for dist in OBS_DIST:
                az = az0 + math.radians(az_off)
                elev = math.radians(elev_deg)
                c = np.array(
                    [
                        e[0] - math.cos(az) * dist * math.cos(elev),
                        e[1] - math.sin(az) * dist * math.cos(elev),
                        e[2] + dist * math.sin(elev),
                    ]
                )
                z = (e - c) / np.linalg.norm(e - c)
                horiz = np.cross(z, np.array([0.0, 0.0, 1.0]))
                if np.linalg.norm(horiz) < 1e-6:
                    x0 = np.array([1.0, 0.0, 0.0])
                else:
                    x0 = horiz / np.linalg.norm(horiz)
                y0 = np.cross(z, x0)
                for psi_deg in OBS_PSI:
                    psi = math.radians(psi_deg)
                    x = math.cos(psi) * x0 + math.sin(psi) * y0
                    y = np.cross(z, x)
                    t_base_cam = np.eye(4)
                    t_base_cam[:3, :3] = np.column_stack([x, y, z])
                    t_base_cam[:3, 3] = c
                    t = t_base_cam @ x_inv
                    q = Rotation.from_matrix(t[:3, :3]).as_quat()
                    out.append(
                        (
                            (float(t[0, 3]), float(t[1, 3]), float(t[2, 3])),
                            (float(q[0]), float(q[1]),
                             float(q[2]), float(q[3])),
                            tuple(float(v) for v in c),
                        )
                    )
    return out


def ray_occluded(
    checker: CrossRobotChecker,
    so_joints: list[float],
    omx_joints: list[float],
    cam: tuple,
    e: np.ndarray,
    d: np.ndarray,
) -> float:
    """관측 카메라→**노출 세그먼트 전체** 시선의 차단 비율 (0=완전 비가림).

    checker 세계의 두 body 전부 차단원 (omx 몸통+카메라 box, so101 자기 팔).
    ray 시작을 카메라에서 2cm 전진 — 원점이 자기 wrist mesh 안이면 즉발 오탐.

    ⚠ 2026-07-28 수정 — **옛 판정은 E±25mm 5점만 샘플해서 실물 가림을 놓쳤다.**
    실물(debug/detect/20260727_230345)에서 omx 조가 가린 띠는 **E+21~33mm**
    (점군 z-gap 0.276~0.288) 인데 옛 범위(E-25~+25mm)가 그 밖이라 "23/23
    비가림" 오판이 나왔다. 노출부는 E 기준 −0.65·exposed ~ +0.35·exposed
    (EXPOSED_FRAC 규약) 이므로 그 전 구간을 11점 균등 샘플한다. 고친 판정은
    실물 차단 지점을 정확히 재현한다 (차단률 18%).
    """
    # 형상 배치 (probe 전용 — in_collision 부작용으로 config set)
    checker.in_collision(so_joints, omx_joints,
                         grip_a=1.0, grip_b=OMX_HOLD_GRIP)
    client = checker._client  # probe 한정 내부 접근 (판정 로직 재사용 목적)
    assert client is not None
    lo, hi = -EXPOSED_FRAC * _exposed, (1.0 - EXPOSED_FRAC) * _exposed
    targets = [e + d * (lo + (hi - lo) * k / 10) for k in range(11)]
    starts, ends = [], []
    for tgt in targets:
        v = tgt - np.asarray(cam)
        v = v / np.linalg.norm(v)
        starts.append(tuple(np.asarray(cam) + v * 0.02))
        ends.append(tuple(tgt))
    hits = p.rayTestBatch(starts, ends, physicsClientId=client)
    blocked = sum(1 for h in hits if h[0] >= 0 and h[2] < 0.97)
    return blocked / len(targets)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", type=Path, default=None)
    args = ap.parse_args()

    base_omx = load_base_omx()
    roi_so = load_roi("so101_6dof_0")
    roi_omx = load_roi("omx_f_0")
    t_tcp_cam_so = load_hand_eye("so101_6dof_0")

    so_urdf = _ROBOT / "so101_6dof" / "urdf" / \
        "so101_6dof.so101_6dof_0.calibrated.urdf"
    omx_urdf = _ROBOT / "omx_f" / "urdf" / "omx_f.urdf"
    kin_so = PybulletKinematics(so_urdf)
    kin_so.initialize()
    kin_omx = PybulletKinematics(omx_urdf)
    kin_omx.initialize()
    checker = CrossRobotChecker(
        _ROBOT / "so101_6dof" / "urdf" / "so101_6dof.urdf", omx_urdf, base_omx
    )

    def in_roi(pt: tuple, roi: dict) -> bool:
        return (
            roi["x_min"] <= pt[0] <= roi["x_max"]
            and roi["y_min"] <= pt[1] <= roi["y_max"]
            and roi["z_min"] <= pt[2] <= roi["z_max"]
        )

    # family 라벨 → 집계
    fams: dict[str, dict] = {}
    t0 = time.monotonic()
    print("\n=== handover block probe (manifold-aligned) ===", flush=True)
    for tx in TCP_XS:
        for ty in TCP_YS:
            for tz in TCP_ZS:
                t_w = np.array([tx, ty, tz])
                t_omx = world_to_omx(tuple(t_w), base_omx)
                if not in_roi(t_omx, roi_omx):
                    continue
                for label, q_omx, z_omx in present_orients(t_omx):
                    fam = fams.setdefault(
                        label.split("/th")[0],
                        {
                            "candidates": 0, "omx_reach": 0, "e_in_roi": 0,
                            "so_reach": 0, "pair_clear": 0, "observed": 0,
                            "hits": [],
                        },
                    )
                    fam["candidates"] += 1
                    # ① omx 제시 도달 (+② 벽)
                    sol = kin_omx.ik(t_omx, q_omx, [0.0] * kin_omx.dof,
                                     IK_BUDGET)
                    if sol is None:
                        continue
                    if checker.min_link_world_x(
                            "b", sol, grip=OMX_HOLD_GRIP) < WALL_MIN_X:
                        continue
                    fam["omx_reach"] += 1
                    # ③ E (노출 끝 파지점) — d 는 world 로
                    d_w = dir_omx_to_world(z_omx, base_omx)
                    e = t_w + d_w * TCP_TO_E_M
                    if not in_roi(tuple(e), roi_so):
                        continue
                    fam["e_in_roi"] += 1
                    # ④ so101 수취 도달 ([pre, grasp], tool z ∥ ±d)
                    so_sol = None
                    for sgn in (1.0, -1.0):
                        for spin in SPIN_DEG:
                            q = quat_tool_z(d_w * sgn, spin)
                            a = Rotation.from_quat(q).apply([1.0, 0.0, 0.0])
                            pre = tuple(e - a * RECV_PRE_CLEAR)
                            s1 = kin_so.ik(pre, q, [0.0] * kin_so.dof,
                                           IK_BUDGET)
                            if s1 is None:
                                continue
                            s2 = kin_so.ik(tuple(e), q, s1, IK_BUDGET)
                            if s2 is None:
                                continue
                            if checker.min_link_world_x("a", s2) < WALL_MIN_X:
                                continue
                            so_sol = s2
                            break
                        if so_sol is not None:
                            break
                    if so_sol is None:
                        continue
                    fam["so_reach"] += 1
                    # ⑤ cross 충돌 (수취 국면 margin)
                    if checker.in_collision(
                        so_sol, omx_sol := sol,
                        grip_a=1.0, grip_b=OMX_HOLD_GRIP,
                        margin_m=COLLISION_MARGIN,
                    ):
                        continue
                    fam["pair_clear"] += 1
                    # ⑥ 관측 도달 + 시선 비가림
                    seen = False
                    for tcp_pos, tcp_q, cam in camera_tcp_poses(
                            e, t_tcp_cam_so):
                        s_obs = kin_so.ik(tcp_pos, tcp_q, [0.0] * kin_so.dof,
                                          10)
                        if s_obs is None:
                            continue
                        occ = ray_occluded(checker, s_obs, omx_sol, cam, e,
                                           d_w)
                        if occ == 0.0:
                            seen = True
                            break
                    if not seen:
                        continue
                    fam["observed"] += 1
                    fam["hits"].append(
                        {
                            "label": label,
                            "tcp_world": [round(float(v), 3) for v in t_w],
                            "E": [round(float(v), 3) for v in e],
                            "d_world": [round(float(v), 2) for v in d_w],
                        }
                    )
    for label, fam in sorted(fams.items()):
        best = fam["hits"][0] if fam["hits"] else None
        print(
            f"{label:8s} cand {fam['candidates']:3d} > omx "
            f"{fam['omx_reach']:3d} > E-roi {fam['e_in_roi']:3d} > so "
            f"{fam['so_reach']:3d} > clear {fam['pair_clear']:3d} > seen "
            f"{fam['observed']:3d}   best={best}",
            flush=True,
        )

    print(f"\ntotal {time.monotonic() - t0:.0f}s", flush=True)
    if args.json:
        args.json.write_text(
            json.dumps(
                {
                    "block": {
                        "len": BLOCK_LEN_M,
                        "grasp_frac": GRASP_FRAC,
                        "tcp_to_e": TCP_TO_E_M,
                    },
                    "families": fams,
                },
                ensure_ascii=False,
                indent=2,
            ),
            encoding="utf-8",
        )
        print(f"JSON -> {args.json}")
    kin_so.close()
    kin_omx.close()
    checker.close()


if __name__ == "__main__":
    main()
