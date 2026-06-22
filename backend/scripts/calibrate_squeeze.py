"""LOOCV 최저점 자리 max-push 자리 — outlier iteration + prior sweep + stage 비교.

전략:
  1. baseline (drop 6, 현 commit) 부터 시작
  2. per-pose RMS top-K outlier 자리 추가 제거, LOOCV 재측정
  3. LOOCV 자리 더 안 줄면 STOP
  4. 동시에 prior 강도 / Huber threshold sweep
  5. effective σ (board pose std) 도 같이 측정 → LOOCV + effective σ 둘 다 본다
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

BACKEND = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BACKEND))

import sqlite3
from core.robot.robot_registry import RobotRegistry  # noqa: E402
from modules.motor.motor_config import load_motor_layout  # noqa: E402

import scripts.calibrate_offline as co  # noqa: E402

ROBOT = "so101_6dof_0"
DB_PATH = BACKEND / "storage" / "horibot.db"
BLOB_ROOT = BACKEND / "storage" / "blobs"


def run_config(captures, fk_chain, K, sag_arm_indices, arm_cfgs, cfg, name):
    """run_ba_stage + LOOCV + effective σ — 한 config 자리.

    run_ba_stage 가 이미 effective σ 자리 계산해서 res 에 박음 (sigma_dual_metric).
    """
    seed_R, seed_t, _ = co.seed_handeye(captures, fk_chain)
    res = co.run_ba_stage(
        captures, fk_chain, K, sag_arm_indices, cfg,
        name=name, seed_handeye_R=seed_R, seed_handeye_t=seed_t,
        arm_cfgs=arm_cfgs, irls_outer=3,
    )
    loocv = co.compute_loocv(
        captures, fk_chain, K, sag_arm_indices, cfg,
        seed_R, seed_t, arm_cfgs,
    )
    return (
        res, loocv,
        res.effective_sigma_handeye_rot_deg,
        res.effective_sigma_handeye_t_mm,
    )


def per_pose_rms(captures, fk_chain, K, sag_arm_indices, arm_cfgs, cfg, res):
    """현재 fit 자리 per-pose RMS — 다음 drop 후보 자리."""
    n_arm = fk_chain.n_arm
    n_sag = len(sag_arm_indices)
    # joint/link/sag arrays.
    joint_off = np.zeros(n_arm)
    link_t = np.zeros((n_arm, 3))
    link_r = np.zeros((n_arm, 3))
    sag_sparse = np.zeros(n_sag)
    if "joint" in res.estimated:
        for i, c in enumerate(arm_cfgs):
            joint_off[i] = res.joint_offsets.get(c.id, 0.0)
    if "link" in res.estimated:
        for i, c in enumerate(arm_cfgs):
            if c.id in res.link_trans: link_t[i] = res.link_trans[c.id]
            if c.id in res.link_rot: link_r[i] = res.link_rot[c.id]
    if "sag" in res.estimated:
        for i, ai in enumerate(sag_arm_indices):
            sag_sparse[i] = res.sag_k.get(arm_cfgs[ai].id, 0.0)
    x = co.pack_params(
        res.handeye_R, res.handeye_t, res.target_R, res.target_t,
        joint_off, link_t, link_r, sag_sparse, cfg,
    )
    r = co.compute_residuals(
        x, captures, fk_chain, K, sag_arm_indices, cfg,
        pose_weights=np.ones(len(captures)),
    )
    prior_len = 0
    if cfg.estimate_joint: prior_len += n_arm
    if cfg.estimate_link: prior_len += 6 * n_arm
    if cfg.estimate_sag: prior_len += n_sag
    r_data = r[:-prior_len] if prior_len else r
    rms_list, _ = co._per_pose_residual_breakdown(r_data, captures, cfg)
    return rms_list


def main():
    sys.stdout.reconfigure(encoding="utf-8")
    import logging
    logging.basicConfig(level=logging.WARNING)

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", default=ROBOT)
    parser.add_argument("--run-id", type=int, default=2)
    parser.add_argument(
        "--initial-drop", type=int, nargs="*", default=[],
        help="강제 제외할 pose_index list (선택). default empty = fresh squeeze."
    )
    args = parser.parse_args()

    robot_id: str = args.robot
    run_id: int = args.run_id

    # Strict prior (현 commit 사용).
    co.PRIOR_JOINT_RAD = np.deg2rad(1.0)
    co.PRIOR_LINK_T_M = 0.001
    co.PRIOR_LINK_R_RAD = np.deg2rad(0.2)

    arm_cfgs = load_motor_layout(robot_id).arm
    registry = RobotRegistry()
    fk_chain = registry.get_fk_chain(robot_id)
    sag_arm_indices = [
        m - 1 for m in registry.get(robot_id).sag_joint_motor_ids
    ]

    # 전체 captures 로드 + drop 가능한 set 확인.
    con = sqlite3.connect(str(DB_PATH))
    cur = con.cursor()
    all_indices = [
        r[0] for r in cur.execute(
            "SELECT pose_index FROM calibration_captures WHERE run_id=? ORDER BY pose_index",
            (run_id,),
        )
    ]
    con.close()
    print(f"전체 captures: {len(all_indices)}")

    # 초기 drop set — default empty (fresh 캘 outlier 자동 squeeze).
    # 특정 pose 강제 제외 필요하면 args.initial_drop 으로.
    drop: set[int] = set(args.initial_drop)
    if drop:
        print(f"\n초기 drop: {sorted(drop)} → {len(all_indices)-len(drop)} captures\n")
    else:
        print(f"\n초기 drop: empty → {len(all_indices)} captures (fresh squeeze)\n")

    cfg = co.BAConfig(estimate_joint=True, estimate_link=True, estimate_sag=True)

    print(f"{'iter':5s} {'drop_n':6s} {'caps':5s} {'train':6s} {'LOOCV':6s} "
          f"{'ratio':6s} {'σ_R':6s} {'σ_t':7s} {'next_drop':20s}")
    print("-" * 88)

    history = []
    for it in range(8):
        # captures 자리 drop 적용 + 로드.
        run, captures, intrinsic, arm_cfgs_l = co.load_data(
            BACKEND / "storage" / "horibot.db",
            BACKEND / "storage" / "blobs",
            ROBOT, 2, load_depth=False,
        )
        captures = [c for c in captures if c.pose_index not in drop]
        K = intrinsic["camera_matrix"]

        t0 = time.time()
        res, loocv, sig_R, sig_t = run_config(
            captures, fk_chain, K, sag_arm_indices, arm_cfgs, cfg,
            name=f"sweep_iter{it}",
        )
        elapsed = time.time() - t0

        # 다음 drop 후보 — 가장 RMS 큰 capture 1개.
        rms_list = per_pose_rms(captures, fk_chain, K, sag_arm_indices, arm_cfgs, cfg, res)
        # slot → orig pose_index 매핑.
        kept = [c.pose_index for c in captures]
        ranked = sorted(zip(kept, rms_list), key=lambda x: -x[1])
        next_drop = ranked[0]

        print(
            f"  {it:3d}  {len(drop):4d}   {len(captures):3d}  "
            f"{res.reproj_rms_px:5.2f}  {loocv:5.2f}  "
            f"{loocv/res.reproj_rms_px:4.2f}× "
            f"{sig_R:5.3f}° {sig_t:5.2f}mm  next: pose #{next_drop[0]} ({next_drop[1]:.2f}px)"
        )

        history.append({
            "iter": it,
            "drop": sorted(drop),
            "caps": len(captures),
            "train": res.reproj_rms_px,
            "loocv": loocv,
            "sigma_R_deg": sig_R,
            "sigma_t_mm": sig_t,
            "result": res,
        })

        # 이전 iter 대비 LOOCV 개선 < 0.1px 자리 stop.
        if it >= 2 and (history[-2]["loocv"] - loocv) < 0.1:
            print(f"\n  → LOOCV plateau 자리 (Δ < 0.1px). Stop iteration.")
            break

        # 다음 outlier drop 추가.
        drop.add(next_drop[0])

    # Best LOOCV 선정.
    best = min(history, key=lambda h: h["loocv"])
    print("\n=== BEST LOOCV ===")
    print(f"  iter {best['iter']}: drop={sorted(best['drop'])}")
    print(f"  caps={best['caps']}, train={best['train']:.2f}, LOOCV={best['loocv']:.2f}px")
    print(f"  effective σ_R={best['sigma_R_deg']:.3f}°  σ_t={best['sigma_t_mm']:.2f}mm")

    # Best effective σ — LOOCV best 와 다를 때만 참고용으로 표시.
    best_sig = min(history, key=lambda h: h["sigma_t_mm"] + h["sigma_R_deg"]*10)
    if best_sig is not best:
        print("\n=== Different best by effective σ ===")
        print(f"  iter {best_sig['iter']}: drop={sorted(best_sig['drop'])}")
        print(f"  caps={best_sig['caps']}, train={best_sig['train']:.2f}, LOOCV={best_sig['loocv']:.2f}px")
        print(f"  σ_R={best_sig['sigma_R_deg']:.3f}°  σ_t={best_sig['sigma_t_mm']:.2f}mm")

    drop_args = " ".join(map(str, sorted(best["drop"]))) or "(none)"
    print(
        "\nsqueeze 는 측정만 — best 결과 commit 은 사용자 명령:\n"
        f"  uv run python scripts/calibrate_offline.py "
        f"--robot {robot_id} --run-id {run_id} --commit "
        f"--drop-poses {drop_args}"
    )


if __name__ == "__main__":
    main()
