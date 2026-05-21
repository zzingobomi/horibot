"""patched URDF (PyBullet FK) vs numpy fk_chain 일치 검증.

목적:
    BA는 numpy fk_chain으로 link_offset과 hand-eye를 같이 풀고,
    production code는 PyBullet이 patched URDF를 로드해 FK/IK 수행한다.
    두 경로가 *수치적으로 같은* 결과를 줘야 BA가 풀어준 link_offset이 실제
    런타임 시스템에 그대로 반영됨.

방법:
    v3 final 결과의 link_trans/link_rot을 LinkOffsets에 채워 patched URDF 생성.
    여러 random joint angle에 대해:
      - PyBullet (patched URDF) FK → EE pose
      - numpy fk_chain (같은 link_offsets) → EE pose
    두 결과 차이가 0에 충분히 가까우면 일치.

쟁점 — URDF rpy 표현:
    urdf_patcher는 link_rot rotvec를 URDF rpy 슬롯에 그대로 가산 (small-angle).
    PyBullet은 URDF rpy를 ZYX 오일러(R = Rz·Ry·Rx)로 해석.
    numpy fk_chain은 rotvec → Rodrigues로 R 계산.
    작은 각(<5°)에서 두 표현 차이는 O(angle^3) — 0.85°에서 ~1e-9 무시 가능.
    검증으로 실측해서 확인.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pybullet as p

sys.path.insert(0, str(Path(__file__).parent))

from core.urdf_patcher import write_patched_urdf  # noqa: E402
from modules.calibration.link_offsets import LinkOffsets  # noqa: E402
from modules.kinematics.fk_chain import fk_chain  # noqa: E402
from modules.kinematics.solver import URDF_PATH  # noqa: E402

# v3 final 결과 (대략값)
LINK_TRANS = np.array(
    [
        [0.0, 0.0, 0.0],
        [-0.02907, 0.00025, 0.0],
        [+0.01112, 0.00025, -0.00445],
        [+0.01066, 0.00027, -0.01484],
        [-0.00011, 0.00052, -0.01056],
    ],
    dtype=np.float64,
)
LINK_ROT = np.deg2rad(
    np.array(
        [
            [0.0, 0.0, 0.0],
            [-0.61, +0.82, 0.0],
            [-0.39, +0.74, +0.06],
            [-0.65, +0.60, -0.43],
            [0.0, +0.59, +0.30],
        ],
        dtype=np.float64,
    )
)


def main():
    offsets = LinkOffsets(
        trans={i + 1: LINK_TRANS[i] for i in range(5)},
        rot={i + 1: LINK_ROT[i] for i in range(5)},
    )
    patched_path = write_patched_urdf(URDF_PATH, offsets)
    print(f"patched URDF: {patched_path}")

    client = p.connect(p.DIRECT)
    robot = p.loadURDF(
        str(patched_path), useFixedBase=True, physicsClientId=client
    )

    arm_indices: list[int] = []
    ee_index = -1
    n_joints = p.getNumJoints(robot, physicsClientId=client)
    for i in range(n_joints):
        info = p.getJointInfo(robot, i, physicsClientId=client)
        joint_type = info[2]
        link_name = info[12].decode()
        if joint_type == p.JOINT_REVOLUTE and len(arm_indices) < 5:
            arm_indices.append(i)
        if link_name == "end_effector_link":
            ee_index = i
    if len(arm_indices) < 5 or ee_index < 0:
        print(f"❌ joint/ee 찾기 실패: arm={arm_indices}, ee={ee_index}")
        return

    rng = np.random.default_rng(42)
    n_test = 30
    max_pos_err_mm = 0.0
    max_rot_err_deg = 0.0
    rms_pos = 0.0
    rms_rot = 0.0

    for k in range(n_test):
        angles = rng.uniform(-np.pi / 2, np.pi / 2, 5)

        for j, idx in enumerate(arm_indices):
            p.resetJointState(robot, idx, float(angles[j]), physicsClientId=client)
        state = p.getLinkState(
            robot, ee_index, computeForwardKinematics=True, physicsClientId=client
        )
        pb_pos = np.array(state[4], dtype=np.float64)
        pb_quat = np.array(state[5], dtype=np.float64)
        m = p.getMatrixFromQuaternion(pb_quat, physicsClientId=client)
        pb_R = np.array(m).reshape(3, 3)

        np_R, np_t = fk_chain(angles, LINK_TRANS, LINK_ROT)

        pos_err_mm = float(np.linalg.norm(pb_pos - np_t) * 1000.0)
        R_diff = pb_R @ np_R.T
        cos = (float(np.trace(R_diff)) - 1.0) * 0.5
        cos = max(-1.0, min(1.0, cos))
        rot_err_deg = float(np.degrees(np.arccos(cos)))

        rms_pos += pos_err_mm**2
        rms_rot += rot_err_deg**2
        max_pos_err_mm = max(max_pos_err_mm, pos_err_mm)
        max_rot_err_deg = max(max_rot_err_deg, rot_err_deg)

        if k < 3:
            print(
                f"  test {k}: pb_pos={pb_pos}  np_t={np_t}  "
                f"pos_err={pos_err_mm:.4f}mm  rot_err={rot_err_deg:.4f}°"
            )

    rms_pos = float(np.sqrt(rms_pos / n_test))
    rms_rot = float(np.sqrt(rms_rot / n_test))

    print()
    print(f"테스트 {n_test}회:")
    print(f"  pos_err: max={max_pos_err_mm:.4f}mm  rms={rms_pos:.4f}mm")
    print(f"  rot_err: max={max_rot_err_deg:.4f}°  rms={rms_rot:.4f}°")

    if max_pos_err_mm < 0.01 and max_rot_err_deg < 0.001:
        print("✓ 완벽 일치 (수치 오차 수준)")
    elif max_pos_err_mm < 0.5 and max_rot_err_deg < 0.05:
        print("✓ 일치 (small-angle 가정 안에서 무시 가능)")
    elif max_pos_err_mm < 2.0 and max_rot_err_deg < 0.2:
        print("△ 작은 차이 — small-angle 한계 또는 ZYX vs rotvec 차이. BA에 영향 미미.")
    else:
        print("✗ 큰 차이 — patcher 또는 fk_chain 검토 필요")

    p.disconnect(client)


if __name__ == "__main__":
    main()
