"""HandEyeCalibration backend 통합 E2E 회귀 테스트.

세 가지 검증:
    1. PybulletSolver 부팅 — LinkCoordinates 비어있을 때도 patched URDF 정상 생성/로드
    2. compute_with_diagnostics(use_extended_ba=False) — 기존 11 DOF BA 동작 + dict에
       link_offset_estimated=False 가 포함되는지 (호환)
    3. compute_with_diagnostics(use_extended_ba=True) — 확장 41 DOF BA 동작 +
       link_trans/link_rot_delta 채워짐 + sanity 결과 재현
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))

from core.common import GRIPPER_ID  # noqa: E402
from modules.calibration.hand_eye import HandEyeCalibration  # noqa: E402
from modules.dynamixel.motor_config import load_motor_config  # noqa: E402
from modules.kinematics.solver import PybulletSolver  # noqa: E402

POSES = Path(__file__).parents[1] / "robot" / "calibration" / "handeye_poses.npz"


def main():
    print("=== 1. PybulletSolver 부팅 ===")
    solver = PybulletSolver()
    print(f"  EE link index={solver._ee_index}, joints={len(solver._joint_indices)}")

    print("\n=== 2. HandEyeCalibration 포즈 로드 ===")
    _, motor_cfgs = load_motor_config()
    arm_cfgs = [m for m in motor_cfgs if m.id != GRIPPER_ID]
    he = HandEyeCalibration()
    n = he.load_poses(POSES)
    limits = solver.joint_limits(len(arm_cfgs))
    print(f"  포즈 {n}개, n_joints={len(arm_cfgs)}")

    print("\n=== 3. 기존 BA (use_extended_ba=False) ===")
    res_std = he.compute_with_diagnostics(
        fk_fn=solver.fk_to_matrix,
        arm_motor_cfgs=arm_cfgs,
        joint_limits_rad=limits,
        use_extended_ba=False,
    )
    assert res_std is not None
    print(f"  σ_rot={res_std['sigma_rot_deg']:.3f}° σ_t={res_std['sigma_t_mm']:.2f}mm")
    print(f"  method={res_std['method']}")
    print(f"  joint_offset_estimated={res_std['joint_offset_estimated']}")
    print(f"  link_offset_estimated={res_std['link_offset_estimated']}")
    print(f"  link_trans_delta len={len(res_std['link_trans_delta'])}")
    assert res_std["link_offset_estimated"] is False, (
        "기존 BA는 link_offset 추정 안 함"
    )

    print("\n=== 4. 확장 BA (use_extended_ba=True) ===")
    res_ext = he.compute_with_diagnostics(
        fk_fn=solver.fk_to_matrix,
        arm_motor_cfgs=arm_cfgs,
        joint_limits_rad=limits,
        use_extended_ba=True,
    )
    assert res_ext is not None
    print(f"  σ_rot={res_ext['sigma_rot_deg']:.3f}° σ_t={res_ext['sigma_t_mm']:.2f}mm")
    print(f"  method={res_ext['method']}")
    print(f"  joint_offset_estimated={res_ext['joint_offset_estimated']}")
    print(f"  link_offset_estimated={res_ext['link_offset_estimated']}")
    assert res_ext["link_offset_estimated"] is True, (
        "확장 BA는 link_offset 추정해야 함"
    )

    print("  joint_offset_delta:")
    for e in res_ext["joint_offset_delta"]:
        print(f"    motor{e['motor_id']}: {e['offset_deg']:+.3f}°")
    print("  link_trans_delta (mm):")
    for e in res_ext["link_trans_delta"]:
        print(
            f"    motor{e['motor_id']}: "
            f"({e['x_mm']:+5.2f}, {e['y_mm']:+5.2f}, {e['z_mm']:+5.2f})"
        )
    print("  link_rot_delta (deg):")
    for e in res_ext["link_rot_delta"]:
        print(
            f"    motor{e['motor_id']}: "
            f"({e['rx_deg']:+5.3f}, {e['ry_deg']:+5.3f}, {e['rz_deg']:+5.3f})"
        )

    print("\n=== 5. 결론 ===")
    d_sigma_t = res_std["sigma_t_mm"] - res_ext["sigma_t_mm"]
    d_sigma_rot = res_std["sigma_rot_deg"] - res_ext["sigma_rot_deg"]
    print(
        f"  σ_t 개선: {res_std['sigma_t_mm']:.2f} → {res_ext['sigma_t_mm']:.2f}mm "
        f"(Δ={d_sigma_t:+.2f}mm)"
    )
    print(
        f"  σ_rot 개선: {res_std['sigma_rot_deg']:.3f} → {res_ext['sigma_rot_deg']:.3f}° "
        f"(Δ={d_sigma_rot:+.3f}°)"
    )
    if d_sigma_t > 0 and d_sigma_rot >= -0.05:
        print("  OK: 확장 BA가 σ_t 개선, σ_rot 동등 이상")
    else:
        print("  !! 확장 BA가 기존 BA보다 나쁨 (예상 외)")


if __name__ == "__main__":
    main()
