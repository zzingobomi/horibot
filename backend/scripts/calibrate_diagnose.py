"""캘 데이터 + BA 결과 진단 — RED flag 원인 추적.

확인 항목:
  1. Joint angle diversity (capture 별 자세 spread) — narrow 면 BA 가 handeye/joint
     disambiguate 불가
  2. Seed handeye 자리 T_target2base 일관성 — 보드 base 자리 자세 25 자리 자리 자리
     일치해야. 분산 크면 FK 또는 handeye seed 문제
  3. Per-pose RMS 와 joint angle 자리 correlation — 자세별 systematic vs random
  4. Strong-prior BA 결과 — link_offset 강제로 작게 → reproj 변화 (overfit 진단)
"""

from __future__ import annotations

import json
import sqlite3
import sys
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as Rot

BACKEND = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BACKEND))

from core.robot.robot_registry import RobotRegistry  # noqa: E402
from core.units import raw_to_rad  # noqa: E402
from modules.calibration import board as calib_board  # noqa: E402
from modules.motor.motor_config import load_motor_layout  # noqa: E402

ROBOT = "so101_6dof_0"
RUN_ID = 2
DB = BACKEND / "storage" / "horibot.db"


def main() -> None:
    try:
        sys.stdout.reconfigure(encoding="utf-8")  # type: ignore[attr-defined]
    except (AttributeError, Exception):
        pass
    # ─── Load ─────────────────────────────────────────────────
    con = sqlite3.connect(str(DB))
    cur = con.cursor()
    run_row = cur.execute(
        "SELECT algorithm_params FROM calibration_runs WHERE id=?", (RUN_ID,),
    ).fetchone()
    algorithm_params = json.loads(run_row[0])
    snap = algorithm_params["intrinsic_snapshot"]
    K = np.array(snap["camera_matrix"], dtype=np.float64)
    dist = np.array(snap["dist_coeffs"], dtype=np.float64)

    cap_rows = cur.execute(
        "SELECT pose_index, motor_positions, corners_2d, corner_ids, "
        "board_in_cam, reproj_rms_px, tilt_deg "
        "FROM calibration_captures WHERE run_id=? ORDER BY pose_index ASC",
        (RUN_ID,),
    ).fetchall()
    con.close()

    arm_cfgs = load_motor_layout(ROBOT).arm
    registry = RobotRegistry()
    fk_chain = registry.get_fk_chain(ROBOT)

    # 자세 dict.
    poses = []
    for (pi, mp_j, c2d_j, cid_j, bic_j, rms, tilt) in cap_rows:
        mp = {int(k): int(v) for k, v in json.loads(mp_j).items()}
        joint_angles = np.array(
            [raw_to_rad(mp[c.id], reverse=c.reverse) for c in arm_cfgs],
            dtype=np.float64,
        )
        T = np.asarray(json.loads(bic_j), dtype=np.float64)
        poses.append({
            "pose_index": pi,
            "joint_angles": joint_angles,
            "R_target2cam": T[:3, :3],
            "t_target2cam": T[:3, 3],
            "pnp_rms": rms,
            "tilt": tilt,
            "corners_2d": np.asarray(json.loads(c2d_j), dtype=np.float64),
            "corner_ids": np.asarray(json.loads(cid_j), dtype=np.int32),
        })

    print(f"=== {ROBOT} run_id={RUN_ID} ({len(poses)} poses) ===\n")

    # ─── 1. Joint diversity ───────────────────────────────────
    print("[1] Joint angle diversity (std per axis, 자세 spread):")
    joint_arr = np.array([p["joint_angles"] for p in poses])  # (N, 6)
    for i, c in enumerate(arm_cfgs):
        std_deg = float(np.rad2deg(joint_arr[:, i].std()))
        min_deg = float(np.rad2deg(joint_arr[:, i].min()))
        max_deg = float(np.rad2deg(joint_arr[:, i].max()))
        rng = max_deg - min_deg
        marker = ""
        if std_deg < 5:
            marker = " ← 매우 좁음 (BA 어려움)"
        elif std_deg < 10:
            marker = " ← 좁음"
        print(
            f"  J{c.id} ({c.name:8s}): std={std_deg:5.2f}°  "
            f"range=[{min_deg:+6.1f}°, {max_deg:+6.1f}°] = {rng:5.1f}°{marker}"
        )
    print()

    # ─── 2. T_target2base 일관성 (seed handeye 자리) ──────────
    R_g2b_list = [fk_chain.fk(p["joint_angles"])[0] for p in poses]
    t_g2b_list = [fk_chain.fk(p["joint_angles"])[1].reshape(3, 1) for p in poses]
    R_t2c_list = [p["R_target2cam"] for p in poses]
    t_t2c_list = [p["t_target2cam"].reshape(3, 1) for p in poses]

    print("[2] cv2.calibrateHandEye seed 자리 T_target2base 일관성")
    print("    (같은 보드 자리 모든 capture 자리 same pose 여야 — 분산 크면 FK 또는 handeye 문제)")
    print()
    R_he, t_he = cv2.calibrateHandEye(
        R_g2b_list, t_g2b_list, R_t2c_list, t_t2c_list,
        method=cv2.CALIB_HAND_EYE_PARK,
    )
    t_he = np.asarray(t_he).reshape(3)
    print(f"    Seed handeye: t = [{t_he[0]*1000:+.2f}, {t_he[1]*1000:+.2f}, "
          f"{t_he[2]*1000:+.2f}] mm")

    target_origins = []
    target_rots_euler = []
    for p, R_g2b, t_g2b in zip(poses, R_g2b_list, t_g2b_list):
        t_g2b_v = t_g2b.reshape(3)
        R_c2b = R_g2b @ R_he
        t_c2b = R_g2b @ t_he + t_g2b_v
        R_t2b = R_c2b @ p["R_target2cam"]
        t_t2b = R_c2b @ p["t_target2cam"] + t_c2b
        target_origins.append(t_t2b)
        target_rots_euler.append(Rot.from_matrix(R_t2b).as_euler("xyz", degrees=True))
    target_origins = np.array(target_origins)
    target_rots_euler = np.array(target_rots_euler)
    pos_std_mm = target_origins.std(axis=0) * 1000.0
    rot_std_deg = target_rots_euler.std(axis=0)
    pos_mean = target_origins.mean(axis=0)
    print(
        f"    Target origin 평균: [{pos_mean[0]*1000:+.1f}, {pos_mean[1]*1000:+.1f}, "
        f"{pos_mean[2]*1000:+.1f}] mm"
    )
    print(
        f"    Target origin std:  [{pos_std_mm[0]:.2f}, {pos_std_mm[1]:.2f}, "
        f"{pos_std_mm[2]:.2f}] mm  (자리 보드 자리 < 5mm 정상)"
    )
    print(
        f"    Target rot std:     [{rot_std_deg[0]:.2f}, {rot_std_deg[1]:.2f}, "
        f"{rot_std_deg[2]:.2f}]°  (< 2° 정상)"
    )
    total_pos_std = float(np.linalg.norm(pos_std_mm))
    total_rot_std = float(np.linalg.norm(rot_std_deg))
    if total_pos_std > 20 or total_rot_std > 10:
        print("    → RED: seed handeye 자리 일관성 매우 나쁨. FK 또는 motor direction 의심")
    elif total_pos_std > 10 or total_rot_std > 5:
        print("    → YELLOW: 보통. BA 자리 자세히 fit 해야 함")
    else:
        print("    → OK: seed 일관성 양호. BA 가 미세 조정")
    print()

    # ─── 3. Per-pose reprojection (seed handeye + 평균 target) ─
    print("[3] Per-pose reprojection (seed + mean target). 자세별 systematic err 자리.")
    target_R_mean = _avg_rot([Rot.from_matrix(R_t2c_list[i] @ R_g2b_list[i] @ R_he) for i in range(len(poses))])  # placeholder
    # 정확히는 위에서 계산한 mean target 사용.
    R_t2b_mean, t_t2b_mean = _avg_se3(
        [
            R_g2b_list[i] @ R_he @ R_t2c_list[i]
            for i in range(len(poses))
        ],
        target_origins.tolist(),
    )

    rms_list = []
    for p, R_g2b, t_g2b in zip(poses, R_g2b_list, t_g2b_list):
        t_g2b_v = t_g2b.reshape(3)
        R_c2b = R_g2b @ R_he
        t_c2b = R_g2b @ t_he + t_g2b_v
        R_b2c = R_c2b.T
        R_t2c = R_b2c @ R_t2b_mean
        t_t2c = R_b2c @ t_t2b_mean - R_b2c @ t_c2b

        # 보드 frame corner 자리 reproject.
        obj_pts, _ = calib_board.match_object_points(
            p["corners_2d"].reshape(-1, 1, 2).astype(np.float32),
            p["corner_ids"].reshape(-1, 1).astype(np.int32),
        )
        obj_pts = obj_pts.reshape(-1, 3).astype(np.float64)
        pts_cam = (R_t2c @ obj_pts.T).T + t_t2c
        z = pts_cam[:, 2]
        z_safe = np.where(z > 1e-6, z, 1e-6)
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]
        u = fx * pts_cam[:, 0] / z_safe + cx
        v = fy * pts_cam[:, 1] / z_safe + cy
        proj = np.stack([u, v], axis=-1)
        rms = float(np.sqrt(np.mean((proj - p["corners_2d"]) ** 2)))
        rms_list.append(rms)

    rms_arr = np.array(rms_list)
    print(
        f"    Reproj RMS — median={np.median(rms_arr):.2f}px  "
        f"mean={rms_arr.mean():.2f}px  std={rms_arr.std():.2f}px  "
        f"max={rms_arr.max():.2f}px"
    )
    # joint angle 자리 correlation (어느 축이 크면 RMS 큰가).
    print("    Joint axis 자리 correlation 자리 (|joint angle| vs RMS pearson r):")
    for i, c in enumerate(arm_cfgs):
        abs_ang = np.abs(joint_arr[:, i])
        if abs_ang.std() < 1e-6 or rms_arr.std() < 1e-6:
            r = 0.0
        else:
            r = float(np.corrcoef(abs_ang, rms_arr)[0, 1])
        marker = ""
        if abs(r) > 0.5:
            marker = " ← 강한 correlation (systematic model err)"
        elif abs(r) > 0.3:
            marker = " ← 약한 correlation"
        print(f"      J{c.id} ({c.name:8s}): r = {r:+.3f}{marker}")
    print()

    # ─── 4. Per-capture pnp RMS ───────────────────────────────
    print("[4] PnP RMS (raw 데이터 quality) — DB record 자리:")
    pnp_rmss = np.array([p["pnp_rms"] for p in poses])
    print(
        f"    median={np.median(pnp_rmss):.3f}px  mean={pnp_rmss.mean():.3f}px  "
        f"max={pnp_rmss.max():.3f}px"
    )
    print()

    # ─── 5. Hand-eye t magnitude reasonableness ───────────────
    print("[5] Hand-eye seed t check:")
    print(
        f"    |t| = {np.linalg.norm(t_he)*1000:.1f} mm  "
        f"({'OK' if 0.05 <= np.linalg.norm(t_he) <= 0.20 else 'WARN'})"
    )


def _avg_rot(rots):
    qs = np.array([r.as_quat() for r in rots])
    for i in range(1, len(qs)):
        if np.dot(qs[0], qs[i]) < 0:
            qs[i] = -qs[i]
    M = qs.T @ qs
    _, eigvecs = np.linalg.eigh(M)
    q_mean = eigvecs[:, -1]
    return Rot.from_quat(q_mean).as_matrix()


def _avg_se3(Rs, ts):
    qs = np.array([Rot.from_matrix(R).as_quat() for R in Rs])
    for i in range(1, len(qs)):
        if np.dot(qs[0], qs[i]) < 0:
            qs[i] = -qs[i]
    M = qs.T @ qs
    _, eigvecs = np.linalg.eigh(M)
    q_mean = eigvecs[:, -1]
    R_mean = Rot.from_quat(q_mean).as_matrix()
    t_mean = np.mean(np.array(ts), axis=0)
    return R_mean, t_mean


if __name__ == "__main__":
    main()
