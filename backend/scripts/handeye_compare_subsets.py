"""Backend production BA (IRLS + sag + link + offset) 로 subset 별 실측 σ 비교.

script handeye_diagnose_current.py 는 cv2 standard metric (모델 없는 단순 hand-eye).
이건 backend production BA 를 직접 호출해서 σ_rot/σ_t 실측 — subset 별 비교.
"""

from __future__ import annotations

import json
import sqlite3
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
DB_PATH = REPO_ROOT / "backend" / "storage" / "horibot.db"

# backend module import path
sys.path.insert(0, str(REPO_ROOT / "backend"))

from modules.calibration.hand_eye import HandEyeCalibration, Pose
from core.robot.robot_registry import RobotRegistry
from core.coords.joint_coordinates import JointCoordinates
from modules.motor.motor_config import load_motor_layout


def fetch_run(run_id: int = 2) -> tuple[list[tuple], dict, str]:
    con = sqlite3.connect(str(DB_PATH))
    cur = con.cursor()
    cur.execute(
        "SELECT robot_id FROM calibration_runs WHERE id=?", (run_id,)
    )
    robot_id = cur.fetchone()[0]
    cur.execute(
        "SELECT pose_index, joint_angles, board_in_cam "
        "FROM calibration_captures WHERE run_id=? ORDER BY pose_index",
        (run_id,),
    )
    captures = cur.fetchall()
    cur.execute(
        "SELECT result_data FROM calibration_results "
        "WHERE robot_id=? AND kind='intrinsic' AND is_active=1",
        (robot_id,),
    )
    intr = json.loads(cur.fetchone()[0])
    con.close()
    return captures, intr, robot_id


def build_handeye(captures, intr, robot_id, keep_indices: list[int] | None = None):
    """HandEyeCalibration 에 자세 add_pose. keep_indices=None 이면 전체."""
    reg = RobotRegistry()
    cfg = reg.get(robot_id)
    sag_arm_indices = [m - 1 for m in cfg.sag_joint_motor_ids]

    fk_chain = reg.get_fk_chain(robot_id)
    he = HandEyeCalibration(fk_chain, sag_arm_indices=sag_arm_indices)
    he.intrinsic_matrix = np.array(intr["camera_matrix"])
    he.dist_coeffs = np.array(intr["dist_coeffs"])

    arm_cfgs = load_motor_layout(robot_id).arm
    joints = JointCoordinates()

    pose_count = 0
    for pi, ja_s, bic_s in captures:
        if keep_indices is not None and pi not in keep_indices:
            continue
        if bic_s is None:
            continue
        ja_rad = json.loads(ja_s)
        bic = np.array(json.loads(bic_s))
        # joint_angles (URDF rad) → motor raw
        raw_positions = {
            cfg.id: joints.urdf_to_motor(rad, cfg, robot_id=robot_id)
            for cfg, rad in zip(arm_cfgs, ja_rad)
        }
        he.add_pose(Pose(
            raw_motor_positions=raw_positions,
            R_target2cam=bic[:3, :3],
            t_target2cam=bic[:3, 3].reshape(3, 1),
        ))
        pose_count += 1
    return he, pose_count


def run_ba(he: HandEyeCalibration, fk_fn, arm_cfgs, joint_limits_rad, mode: str = "physical_sag") -> dict | None:
    """compute_with_diagnostics 호출 → diag dict."""
    use_ext = mode in ("extended", "physical_sag")
    use_sag = mode == "physical_sag"
    return he.compute_with_diagnostics(
        fk_fn=fk_fn,
        arm_motor_cfgs=arm_cfgs,
        joint_limits_rad=joint_limits_rad,
        use_extended_ba=use_ext,
        use_physical_sag=use_sag,
    )


def main(run_id: int = 2) -> None:
    captures, intr, robot_id = fetch_run(run_id)
    print(f"== Run {run_id} ({robot_id}), {len(captures)} 자세 ==\n")

    # subset 비교 list
    keep_subsets = {
        "전체 16": None,
        "N=15 (#15 빼기 — LOOCV 1순위 outlier)": [i for i in range(16) if i != 15],
        "N=13 (#2,3,15 빼기)": [i for i in range(16) if i not in {2, 3, 15}],
        "N=10 (script best — #2,3,5,9,10,15 빼기)": [
            i for i in range(16) if i not in {2, 3, 5, 9, 10, 15}
        ],
        "N=9 (#2,3,4,5,9,10,15 빼기)": [
            i for i in range(16) if i not in {2, 3, 4, 5, 9, 10, 15}
        ],
        "N=8 (#2,3,4,5,9,10,12,15 빼기)": [
            i for i in range(16) if i not in {2, 3, 4, 5, 9, 10, 12, 15}
        ],
    }

    # FK + arm_cfgs + joint_limits (subset 무관, robot 단위로 1회 준비)
    reg = RobotRegistry()
    kin = reg.get_kinematics(robot_id)
    # PybulletKinematics 는 apply_link_offsets + initialize 필요 (storage_layer.md §7).
    # SagCorrected wrapper 를 unwrap 해서 inner 의 initialize 호출.
    inner = kin._inner if hasattr(kin, "_inner") else kin
    if hasattr(inner, "initialize"):
        try:
            from modules.kinematics.link_offsets import LinkOffsets
            inner.apply_link_offsets(LinkOffsets())
        except Exception:
            pass
        inner.initialize()
    fk_fn = kin.fk_to_matrix
    arm_cfgs = load_motor_layout(robot_id).arm
    joint_limits_rad = kin.joint_limits(len(arm_cfgs))

    print(f"{'subset':<60} {'N':>4} {'sigma_rot(deg)':>15} {'sigma_t(mm)':>12} {'verdict':>14}")
    results = []
    for label, keep in keep_subsets.items():
        try:
            he, N = build_handeye(captures, intr, robot_id, keep)
            diag = run_ba(he, fk_fn, arm_cfgs, joint_limits_rad, mode="physical_sag")
            if diag is None:
                print(f"  {label:<58} {N:>4} BA 실패")
                continue
            srot = diag.get("sigma_rot_deg")
            st_mm = diag.get("sigma_t_mm")
            verdict = (diag.get("coach", {}) or {}).get("verdict", "?")
            srot_s = f"{srot:.3f}" if srot is not None else "NA"
            st_s = f"{st_mm:.2f}" if st_mm is not None else "NA"
            print(f"  {label:<58} {N:>4} {srot_s:>15} {st_s:>12} {verdict:>14}")
            results.append((label, N, srot, st_mm, verdict))
        except Exception as e:
            print(f"  {label:<58} ERROR: {type(e).__name__}: {e}")

    if results:
        valid = [(l, n, sr, st, v) for l, n, sr, st, v in results if sr is not None]
        best = min(valid, key=lambda r: r[2])
        print(f"\n── 실측 backend BA 최적: {best[0]}")
        print(f"   N={best[1]}, σ_rot={best[2]:.3f}°, σ_t={best[3]:.2f}mm, verdict={best[4]}")


if __name__ == "__main__":
    run_id = int(sys.argv[1]) if len(sys.argv) > 1 else 2
    main(run_id)
