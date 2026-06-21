"""One-off backfill — 기존 hand_eye result row 의 effective_sigma_rot/t 채움.

run_id=2 의 active set (id=6 hand_eye + id=7 joint + id=8 link + id=9 sag) 의
result_data 4개로 StageResult 복원 → 전체 34 cap 자리 measure_effective_sigma →
UPDATE row id=6. id=2 (옛 commit, joint_offset=id=3, link=id=4, sag=id=5) 도 동일.

drop set 미보존이라 squeeze stdout 의 25-cap σ 와 다른 값 — *전체 cap 기준 fit 평가*.
"""

from __future__ import annotations

import json
import sqlite3
import sys
from pathlib import Path

import numpy as np

BACKEND = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BACKEND))

from core.robot.robot_registry import RobotRegistry  # noqa: E402
from modules.motor.motor_config import load_motor_layout  # noqa: E402

import scripts.calibrate_offline as co  # noqa: E402


def _stage_from_rows(
    he_payload: dict,
    joint_payload: dict | None,
    link_payload: dict | None,
    sag_payload: dict | None,
) -> co.StageResult:
    """4 row 의 result_data → 최소 StageResult — measure 자리 필요한 필드만 채움."""
    handeye_R = np.array(he_payload["R_cam2gripper"])
    handeye_t = np.array(he_payload["t_cam2gripper"]).flatten()

    joint_offsets: dict[int, float] = {}
    if joint_payload is not None:
        for k, v in joint_payload.get("offsets", {}).items():
            joint_offsets[int(k)] = float(v)

    link_trans: dict[int, np.ndarray] = {}
    link_rot: dict[int, np.ndarray] = {}
    if link_payload is not None:
        for entry in link_payload.get("offsets", []):
            jid = int(entry["joint_id"])
            link_trans[jid] = np.array(entry["trans_m"], dtype=float)
            link_rot[jid] = np.array(entry["rot_rad"], dtype=float)

    sag_k: dict[int, float] = {}
    if sag_payload is not None:
        for k, v in sag_payload.get("k_rad_per_m", {}).items():
            sag_k[int(k)] = float(v)

    return co.StageResult(
        name="backfill",
        estimated={"handeye", "joint", "link", "sag"},
        handeye_R=handeye_R,
        handeye_t=handeye_t,
        target_R=np.eye(3),
        target_t=np.zeros(3),
        joint_offsets=joint_offsets,
        link_trans=link_trans,
        link_rot=link_rot,
        sag_k=sag_k,
    )


def backfill_run(robot_id: str, run_id: int, db_path: Path, blob_root: Path) -> None:
    arm_cfgs = load_motor_layout(robot_id).arm
    fk_chain = RobotRegistry().get_fk_chain(robot_id)
    _, captures, _, _ = co.load_data(
        db_path, blob_root, robot_id, run_id, load_depth=False
    )
    print(f"run_id={run_id} captures 로드: {len(captures)}장")

    con = sqlite3.connect(str(db_path))
    cur = con.cursor()
    # 각 active set + 각 inactive set 자리 자리 동일 robot+run 자리 hand_eye 별
    # 그룹화. created_at 자리 같은 그룹 자리 자리 자리 자리.
    rows = cur.execute(
        "SELECT id, kind, created_at, result_data FROM calibration_results "
        "WHERE run_id=? ORDER BY created_at, id",
        (run_id,),
    ).fetchall()

    groups: dict[float, dict[str, tuple[int, dict]]] = {}
    for rid, kind, ts, payload in rows:
        groups.setdefault(ts, {})[kind] = (rid, json.loads(payload))

    for ts, kinds in groups.items():
        if "hand_eye" not in kinds:
            continue
        he_id, he_payload = kinds["hand_eye"]
        stage = _stage_from_rows(
            he_payload,
            kinds.get("joint_offset", (None, None))[1],
            kinds.get("link_offset", (None, None))[1],
            kinds.get("sag", (None, None))[1],
        )
        sig_R, sig_t = co.measure_effective_sigma(
            captures, fk_chain, arm_cfgs, stage
        )
        cur.execute(
            "UPDATE calibration_results "
            "SET effective_sigma_rot=?, effective_sigma_t=? WHERE id=?",
            (sig_R, sig_t, he_id),
        )
        print(
            f"  ts={ts:.2f}  hand_eye id={he_id} ← "
            f"effective σ_R={sig_R:.3f}°  σ_t={sig_t:.2f}mm (전체 {len(captures)} cap)"
        )

    con.commit()
    con.close()


if __name__ == "__main__":
    backfill_run(
        "so101_6dof_0",
        2,
        BACKEND / "storage" / "horibot.db",
        BACKEND / "storage" / "blobs",
    )
