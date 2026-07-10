"""LOOCV 최저점 max-push — outlier iteration squeeze (backend 이월).

옛 backend/scripts/calibrate_squeeze.py 이월. calibrate_offline 의 짝꿍 진단 도구:
committed 캘 결과가 어떤 drop set 으로 나왔는지 재현 / 새 캘의 outlier 자동 제거.

전략:
  1. drop = {} (fresh) 부터 시작 (Stage D config = joint+link+sag)
  2. per-pose RMS 최대 outlier 1개씩 추가 제거, LOOCV 재측정
  3. LOOCV 개선 < 0.1px 면 STOP
  4. effective σ (board pose std) 도 같이 측정 → best LOOCV drop set + σ 안내

측정만 — commit 은 사용자가 `calibrate_offline.py --commit --drop-poses ...` 로.

CLI:
  uv run python scripts/calibrate_squeeze.py --robot so101_6dof_0 --run-id 2 \
      --db <path> --blobs <dir>
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

import numpy as np

BACKEND_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BACKEND_ROOT))

from apps.config import _ROBOT_DIR, load_robots  # noqa: E402
from infra.database.sqlite import open_sqlite  # noqa: E402
from infra.object_store.filesystem import FilesystemObjectStore  # noqa: E402
from modules.calibration.persistence.repository import CalibrationRepository  # noqa: E402
from modules.motion.fk_chain import FkChain  # noqa: E402
from modules.motor.contract import MotorKind  # noqa: E402

import scripts.calibrate_offline as co  # noqa: E402


def run_config(captures, fk_chain, K, sag_arm_indices, arm_cfgs, cfg, name):
    """run_ba_stage + LOOCV + effective σ — 한 config."""
    seed_R, seed_t, _ = co.seed_handeye(captures, fk_chain)
    res = co.run_ba_stage(
        captures, fk_chain, K, sag_arm_indices, cfg,
        name=name, seed_handeye_R=seed_R, seed_handeye_t=seed_t,
        arm_cfgs=arm_cfgs, irls_outer=3,
    )
    loocv = co.compute_loocv(
        captures, fk_chain, K, sag_arm_indices, cfg, seed_R, seed_t, arm_cfgs,
    )
    return (
        res, loocv,
        res.effective_sigma_handeye_rot_deg,
        res.effective_sigma_handeye_t_mm,
    )


def per_pose_rms(captures, fk_chain, K, sag_arm_indices, arm_cfgs, cfg, res):
    """현재 fit per-pose RMS — 다음 drop 후보."""
    n_arm = fk_chain.n_arm
    n_sag = len(sag_arm_indices)
    joint_off = np.zeros(n_arm)
    link_t = np.zeros((n_arm, 3))
    link_r = np.zeros((n_arm, 3))
    sag_sparse = np.zeros(n_sag)
    if "joint" in res.estimated:
        for i, c in enumerate(arm_cfgs):
            joint_off[i] = res.joint_offsets.get(c.id, 0.0)
    if "link" in res.estimated:
        for i, c in enumerate(arm_cfgs):
            if c.id in res.link_trans:
                link_t[i] = res.link_trans[c.id]
            if c.id in res.link_rot:
                link_r[i] = res.link_rot[c.id]
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
    if cfg.estimate_joint:
        prior_len += n_arm
    if cfg.estimate_link:
        prior_len += 6 * n_arm
    if cfg.estimate_sag:
        prior_len += n_sag
    r_data = r[:-prior_len] if prior_len else r
    rms_list, _ = co._per_pose_residual_breakdown(r_data, captures, cfg)
    return rms_list


def main() -> int:
    try:
        sys.stdout.reconfigure(encoding="utf-8")  # type: ignore[attr-defined]
    except (AttributeError, Exception):
        pass
    logging.basicConfig(level=logging.WARNING)

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", default="so101_6dof_0")
    parser.add_argument("--run-id", type=int, default=2)
    parser.add_argument("--db", type=str, required=True)
    parser.add_argument("--blobs", type=str, required=True)
    parser.add_argument(
        "--initial-drop", type=int, nargs="*", default=[],
        help="강제 제외 pose_index (선택). default empty = fresh squeeze.",
    )
    parser.add_argument("--max-iter", type=int, default=8)
    args = parser.parse_args()

    # Strict prior (현 commit 사용) — offline 과 동일 값 명시.
    co.PRIOR_JOINT_RAD = np.deg2rad(1.0)
    co.PRIOR_LINK_T_M = 0.001
    co.PRIOR_LINK_R_RAD = np.deg2rad(0.2)

    robots = load_robots()
    robot = robots[args.robot]
    arm_cfgs = [m for m in robot.motors if m.kind != MotorKind.GRIPPER]
    arm_names = [c.name for c in arm_cfgs]
    urdf = _ROBOT_DIR / robot.type / "urdf" / f"{robot.type}.urdf"
    fk_chain = FkChain(urdf, arm_names, tcp_link_name="tcp")
    sag_arm_indices = [m - 1 for m in robot.sag_joint_motor_ids]

    _engine, session_factory = open_sqlite(args.db)
    repo = CalibrationRepository(session_factory)
    object_store = FilesystemObjectStore(args.blobs)

    # 전체 captures 1회 로드 (deterministic) → iter 마다 drop 필터.
    _run, all_captures, intrinsic, _arm = co.load_data(
        repo, object_store, robot, args.run_id, load_depth=False,
    )
    K = intrinsic["camera_matrix"]
    all_indices = [c.pose_index for c in all_captures]
    print(f"전체 captures: {len(all_indices)}")

    drop: set[int] = set(args.initial_drop)
    if drop:
        print(f"\n초기 drop: {sorted(drop)} → {len(all_indices)-len(drop)} captures\n")
    else:
        print(f"\n초기 drop: empty → {len(all_indices)} captures (fresh squeeze)\n")

    cfg = co.BAConfig(estimate_joint=True, estimate_link=True, estimate_sag=True)

    print(
        f"{'iter':5s} {'drop_n':6s} {'caps':5s} {'train':6s} {'LOOCV':6s} "
        f"{'ratio':6s} {'σ_R':6s} {'σ_t':7s} {'next_drop':20s}"
    )
    print("-" * 88)

    history = []
    for it in range(args.max_iter):
        captures = [c for c in all_captures if c.pose_index not in drop]

        res, loocv, sig_R, sig_t = run_config(
            captures, fk_chain, K, sag_arm_indices, arm_cfgs, cfg,
            name=f"sweep_iter{it}",
        )

        rms_list = per_pose_rms(
            captures, fk_chain, K, sag_arm_indices, arm_cfgs, cfg, res
        )
        kept = [c.pose_index for c in captures]
        ranked = sorted(zip(kept, rms_list), key=lambda x: -x[1])
        next_drop = ranked[0]

        print(
            f"  {it:3d}  {len(drop):4d}   {len(captures):3d}  "
            f"{res.reproj_rms_px:5.2f}  {loocv:5.2f}  "
            f"{loocv/res.reproj_rms_px:4.2f}× "
            f"{sig_R:5.3f}° {sig_t:5.2f}mm  next: pose #{next_drop[0]} "
            f"({next_drop[1]:.2f}px)"
        )

        history.append({
            "iter": it, "drop": sorted(drop), "caps": len(captures),
            "train": res.reproj_rms_px, "loocv": loocv,
            "sigma_R_deg": sig_R, "sigma_t_mm": sig_t,
        })

        if it >= 2 and (history[-2]["loocv"] - loocv) < 0.1:
            print("\n  → LOOCV plateau (Δ < 0.1px). Stop iteration.")
            break

        drop.add(next_drop[0])

    best = min(history, key=lambda h: h["loocv"])
    print("\n=== BEST LOOCV ===")
    print(f"  iter {best['iter']}: drop={best['drop']}")
    print(
        f"  caps={best['caps']}, train={best['train']:.2f}, "
        f"LOOCV={best['loocv']:.2f}px"
    )
    print(
        f"  effective σ_R={best['sigma_R_deg']:.3f}°  σ_t={best['sigma_t_mm']:.2f}mm"
    )

    best_sig = min(history, key=lambda h: h["sigma_t_mm"] + h["sigma_R_deg"] * 10)
    if best_sig is not best:
        print("\n=== Different best by effective σ ===")
        print(f"  iter {best_sig['iter']}: drop={best_sig['drop']}")
        print(
            f"  σ_R={best_sig['sigma_R_deg']:.3f}°  σ_t={best_sig['sigma_t_mm']:.2f}mm"
        )

    drop_args = " ".join(map(str, best["drop"])) or "(none)"
    print(
        "\nsqueeze 는 측정만 — commit 은:\n"
        f"  uv run python scripts/calibrate_offline.py --robot {args.robot} "
        f"--run-id {args.run_id} --db <db> --blobs <dir> --commit "
        f"--drop-poses {drop_args}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
