"""Effective σ — joint+link 보조 들어갔을 때 실제 board pose 예측 분산.

각 stage 의 BA 결과를 *전체 chain* 에 적용해 25 자세에서 board 의 base-frame
pose 를 예측 → 25개의 std 가 그 stage 의 effective σ (downstream 이 실제로 보는 값).

비교:
  - BA Hessian σ (개별 param marginal 불확실성, 큰 값)
  - Effective σ (조합된 chain 의 prediction 분산, 작은 값)
"""

from __future__ import annotations

import json
import sqlite3
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as Rot

BACKEND = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BACKEND))

from core.robot.robot_registry import RobotRegistry  # noqa: E402
from core.units import raw_to_rad  # noqa: E402
from modules.motor.motor_config import load_motor_layout  # noqa: E402

ROBOT = "so101_6dof_0"
RUN_ID = 2


def main() -> None:
    sys.stdout.reconfigure(encoding="utf-8")  # type: ignore

    # ─── BA 결과 로드 ─────────────────────────────────────────
    with open(BACKEND / "storage" / "calib_loocv.json") as f:
        results = json.load(f)

    # ─── 캡처 + raw joints + PnP 로드 ─────────────────────────
    con = sqlite3.connect(BACKEND / "storage" / "horibot.db")
    cur = con.cursor()
    cap_rows = cur.execute(
        "SELECT pose_index, motor_positions, board_in_cam "
        "FROM calibration_captures WHERE run_id=? ORDER BY pose_index ASC",
        (RUN_ID,),
    ).fetchall()
    con.close()

    arm_cfgs = load_motor_layout(ROBOT).arm
    poses = []
    for pi, mp_j, bic_j in cap_rows:
        mp = {int(k): int(v) for k, v in json.loads(mp_j).items()}
        joint_angles_raw = np.array(
            [raw_to_rad(mp[c.id], reverse=c.reverse) for c in arm_cfgs],
            dtype=np.float64,
        )
        T = np.asarray(json.loads(bic_j), dtype=np.float64)
        poses.append({
            "joint_angles_raw": joint_angles_raw,
            "R_target2cam": T[:3, :3],
            "t_target2cam": T[:3, 3],
        })

    fk_chain = RobotRegistry().get_fk_chain(ROBOT)
    n_arm = fk_chain.n_arm

    # ─── Stage 별 effective σ 계산 ────────────────────────────
    print("=" * 92)
    print("Effective σ — joint+link 보조 들어갈수록 prediction variance 가 작아지는가?")
    print("=" * 92)
    print()
    print(f"{'Stage':6s} | {'BA Hessian σ_he':25s} | {'Effective σ (prediction)':35s}")
    print(f"{'':6s} | {'(개별 param marginal)':25s} | {'(전체 chain 적용 후 board pose std)':35s}")
    print(f"{'':6s} | {'σ_R (°)':9s} {'σ_t (mm)':10s} | "
          f"{'σ_R (°)':9s} {'σ_t (mm)':10s} {'min':6s} {'max':6s}")
    print("-" * 92)

    for stage_name in ["A", "B", "C", "D"]:
        st = results["stages"][stage_name]
        # Hand-eye.
        he_R_euler = np.array(st["handeye_R_euler_deg"])
        he_R = Rot.from_euler("xyz", he_R_euler, degrees=True).as_matrix()
        he_t = np.array(st["handeye_t_mm"]) / 1000.0

        # Joint / link / sag deltas (motor_id key, int 캐스팅 필요).
        joint_off = np.zeros(n_arm)
        for i, c in enumerate(arm_cfgs):
            key = str(c.id)
            if key in st.get("joint_offsets_deg", {}):
                joint_off[i] = np.deg2rad(st["joint_offsets_deg"][key])
        link_trans = np.zeros((n_arm, 3))
        link_rot = np.zeros((n_arm, 3))
        for i, c in enumerate(arm_cfgs):
            key = str(c.id)
            if key in st.get("link_trans_mm", {}):
                link_trans[i] = np.array(st["link_trans_mm"][key]) / 1000.0
            if key in st.get("link_rot_deg", {}):
                link_rot[i] = np.deg2rad(st["link_rot_deg"][key])

        # 각 capture 자리 T_board_in_base 예측 → 25개 std.
        board_origins_base = []
        board_R_base = []
        for p in poses:
            angles = p["joint_angles_raw"] + joint_off
            R_g2b, t_g2b = fk_chain.fk(
                angles,
                link_trans if "link" in st["estimated"] else None,
                link_rot if "link" in st["estimated"] else None,
            )
            # T_cam2base = T_g2b @ T_handeye.
            R_c2b = R_g2b @ he_R
            t_c2b = R_g2b @ he_t + t_g2b
            # T_board_in_base = T_cam2base @ T_board_in_cam.
            R_t2b = R_c2b @ p["R_target2cam"]
            t_t2b = R_c2b @ p["t_target2cam"] + t_c2b
            board_origins_base.append(t_t2b)
            board_R_base.append(R_t2b)

        origins = np.array(board_origins_base)
        pos_std_mm = origins.std(axis=0) * 1000.0
        pos_std_total = float(np.linalg.norm(pos_std_mm))

        # Rotation std: 각 R 의 reference R 자리 angular distance (axis-angle).
        # reference = quaternion 평균.
        Rs = board_R_base
        qs = np.array([Rot.from_matrix(R).as_quat() for R in Rs])
        for i in range(1, len(qs)):
            if np.dot(qs[0], qs[i]) < 0:
                qs[i] = -qs[i]
        M = qs.T @ qs
        _, eigvecs = np.linalg.eigh(M)
        q_mean = eigvecs[:, -1]
        R_mean = Rot.from_quat(q_mean).as_matrix()
        # 각 R 자리 R_mean 자리 angular distance (rad).
        angles_diff_rad = []
        for R in Rs:
            R_rel = R @ R_mean.T
            tr = (np.trace(R_rel) - 1.0) * 0.5
            ang = np.arccos(np.clip(tr, -1.0, 1.0))
            angles_diff_rad.append(ang)
        rot_std_deg = float(np.rad2deg(np.std(angles_diff_rad)))
        rot_max_deg = float(np.rad2deg(np.max(angles_diff_rad)))

        # 출력.
        print(
            f"  {stage_name}    | "
            f"{st['sigma_handeye_rot_deg']:7.2f}°  {st['sigma_handeye_t_mm']:8.2f}mm | "
            f"{rot_std_deg:7.3f}°  {pos_std_total:8.2f}mm  "
            f"{(origins.std(axis=0).min()*1000):5.2f}  {(origins.std(axis=0).max()*1000):5.2f}"
        )

    print()
    print(
        "Hessian σ 는 *그 파라미터 자체* 의 자유도 (joint/link 와 trade 가능 → 큼).\n"
        "Effective σ 는 *전체 chain 의 board 예측 일관성* — downstream (detector) 가 보는 값."
    )


if __name__ == "__main__":
    main()
