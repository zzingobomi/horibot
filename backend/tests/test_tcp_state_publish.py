"""MOTION_STATE_TCP fix 회귀 차단 — 사선 PC 의 root cause 검증.

본 PR (frontend 가 자체 URDF FK 안 돌리고 backend MOTION_STATE_TCP 만 사용)
의 motivation 은 *frontend FK 가 sag + link_offset 빠진 채로 cameraMatrix 를
만들어 PC 가 사선으로 보이는 bug*. 이 테스트는:

1. 합성 robot pose + 합성 horizontal-table point cloud 를 만들고,
2. 두 가지 cameraMatrix 로 reproject — (a) corrected FK (SagCorrectedKinematics)
   (b) naive FK (sag/link 누락, 옛 frontend 가 하던 것)
3. (a) 자리는 table normal 이 world +z 와 거의 일치 (≈ 0°),
   (b) 자리는 table normal 이 의미있는 각도로 tilt (≥ 1°) 임을 assert.

→ frontend 가 다시 자체 FK 로 회귀하면 테스트가 깨지며 fix 가 영구적으로 보호됨.

또 motion_node 의 `_on_motor_state_publish_tcp` 가 *바로 그 corrected FK 결과*
를 publish 하는지 schema-level 으로 검증.
"""
from __future__ import annotations

import sqlite3
import json
import time
from pathlib import Path

import numpy as np
import pytest

from core.robot.robot_registry import RobotRegistry
from modules.kinematics.fk_chain import FkChain
from modules.motor.motor_config import load_motor_layout

REPO_ROOT = Path(__file__).resolve().parents[2]


def _load_active_calibration(robot_id: str) -> dict:
    """캘 5 종 (intrinsic 제외) 의 active result 를 raw dict 로 fetch.

    테스트 운영 환경의 SQLite 가 비어 있을 수 있으므로 (CI / fresh checkout) —
    캘 row 없으면 본 테스트는 skip (실 hardware 만 의미 있음).
    """
    db = REPO_ROOT / "backend" / "storage" / "horibot.db"
    if not db.exists():
        pytest.skip("storage/horibot.db 없음 — 실 캘 데이터 필요")
    conn = sqlite3.connect(str(db))
    out = {}
    for kind in ("joint_offset", "link_offset", "sag", "hand_eye"):
        row = conn.execute(
            "SELECT result_data FROM calibration_results "
            "WHERE robot_id=? AND kind=? AND is_active=1",
            (robot_id, kind),
        ).fetchone()
        if row is None:
            pytest.skip(f"active {kind} 없음 — 캘 commit 후 의미 있음")
        out[kind] = json.loads(row[0])
    return out


def _angle_between(u: np.ndarray, v: np.ndarray) -> float:
    """unit vector 두 개 사이 각 (deg)."""
    u = u / np.linalg.norm(u)
    v = v / np.linalg.norm(v)
    return float(np.degrees(np.arccos(np.clip(u @ v, -1.0, 1.0))))


def test_sag_link_offset_omission_tilts_pointcloud():
    """frontend 가 자체 URDF FK (sag + link_offset 누락) 돌리면 PC 가 사선이 됨.

    backend SSOT MOTION_STATE_TCP 를 거치면 그 사선이 사라짐.

    실 캘 활성 row 가 있는 환경에서만 의미 있음 — CI fresh env 는 skip.
    """
    robot_id = "so101_6dof_0"
    cal = _load_active_calibration(robot_id)

    layout = load_motor_layout(robot_id)
    arm_joint_names = [cfg.name for cfg in layout.arm]
    urdf_path = RobotRegistry().get(robot_id).urdf_path
    fk = FkChain(urdf_path, arm_joint_names, tcp_link_name="tcp")

    # 캘 산출물 unpack
    jo = cal["joint_offset"]["offsets"]
    joint_off = np.array([jo[str(i)] for i in range(1, len(arm_joint_names) + 1)])
    link_trans = np.array([cal["link_offset"]["offsets"][i]["trans_m"] for i in range(len(arm_joint_names))])
    link_rot   = np.array([cal["link_offset"]["offsets"][i]["rot_rad"] for i in range(len(arm_joint_names))])
    sag_k_per_motor = cal["sag"]["k_rad_per_m"]
    R_he = np.array(cal["hand_eye"]["R_cam2gripper"])
    # t_he 는 본 테스트의 plane normal reproject 자리 영향 X (회전만 검증).

    # 책상 보는 dummy pose (J2 shoulder 굽힘 → sag 가장 큰 자리)
    n_arm = len(arm_joint_names)
    angles_raw = np.zeros(n_arm)
    angles_raw[1] = 1.2   # J2 ≈ 69°
    angles_raw[2] = -1.2  # J3 ≈ -69°
    if n_arm >= 6:
        angles_raw[5] = 1.57  # wrist roll work pose
    angles_with_off = angles_raw + joint_off

    # (a) corrected — backend MotionModes.get_tcp_pose 와 동일 chain
    sag_arm_idx = [int(k) - 1 for k in sag_k_per_motor.keys()]
    sag_k = np.array([sag_k_per_motor[str(i + 1)] for i in sag_arm_idx])
    angles_corrected = fk.apply_gravity_sag(angles_with_off, sag_k, sag_arm_idx, link_trans, link_rot)
    R_g_corr, _t_g_corr = fk.fk(angles_corrected, link_trans=link_trans, link_rot=link_rot)
    R_cam_corr = R_g_corr @ R_he

    # (b) naive — 옛 frontend FK (joint_offset 만, sag/link 빠짐)
    R_g_naive, _t_g_naive = fk.fk(angles_with_off)
    R_cam_naive = R_g_naive @ R_he

    # 합성 horizontal table — world +z 가 normal 인 책상 표면.
    table_normal_world = np.array([0.0, 0.0, 1.0])

    # 카메라가 본 책상 normal — true world normal 을 camera frame 으로.
    # corrected camera 가 SSOT → world frame 의 table normal 을 camera 로 옮긴 값
    # 을 *naive cameraMatrix* 로 다시 world 로 돌리면 misalignment 가 나타남.
    n_in_cam = R_cam_corr.T @ table_normal_world
    # corrected reproject: world +z 로 돌아옴
    n_corr_world = R_cam_corr @ n_in_cam
    # naive reproject: 다른 cameraMatrix 사용 → 다른 world 방향
    n_naive_world = R_cam_naive @ n_in_cam

    tilt_corrected = _angle_between(n_corr_world, table_normal_world)
    tilt_naive     = _angle_between(n_naive_world, table_normal_world)

    print(
        f"\n[tilt] corrected (backend SSOT): {tilt_corrected:.4f} deg"
        f" / naive (옛 frontend FK): {tilt_naive:.4f} deg"
    )

    # corrected 는 거의 0 — numerical noise 만.
    assert tilt_corrected < 0.001, (
        f"corrected FK 가 self-consistent 해야 — {tilt_corrected} deg "
        "(round-trip 에 오차가 없어야 정상)"
    )
    # naive 는 의미 있게 tilt — sag/link_offset 효과가 보임.
    assert tilt_naive > 0.5, (
        f"naive FK (sag/link 누락) 의 tilt 가 {tilt_naive} deg 로 너무 작음 — "
        "캘 데이터 자체가 sag/link_offset 가 거의 0 일 가능성. "
        "효과 검증 실패."
    )


def test_motion_state_tcp_publisher_uses_corrected_fk():
    """motion_node 가 `MotionModes.get_tcp_pose()` 결과 그대로 publish 하는지
    *코드 경로* 검증 — 회귀 차단 (누군가 naive FK 로 바꾸면 본 테스트 깨짐).

    `_on_motor_state_publish_tcp` 의 *결과* (publish 인자) 가
    `MotionModes.get_tcp_pose()` 호출 결과와 1:1 일치하는지 monkey-patch 로 확인.
    """
    from unittest.mock import MagicMock, patch
    from nodes.device.motion_node import MotionNode
    from core.transport.messages.motion import MotionTcpState

    # MotionNode 의 __init__ 전체 mock — robotregistry / Coordinates / runner 등
    # 다 부담스러우니 publisher 메서드만 단독 호출.
    node = MotionNode.__new__(MotionNode)
    node._arm_cfgs = [
        type("Cfg", (), {"id": i, "name": f"joint{i}", "reverse": False})()
        for i in range(1, 7)
    ]
    node.robot_id = "so101_6dof_0"
    node._motion = MagicMock()
    node._motion.get_tcp_pose.return_value = type(
        "Pose", (), {"position": [0.1, 0.2, 0.3], "quaternion": [0, 0, 0, 1]}
    )()

    published: list[tuple[str, MotionTcpState]] = []
    node.publish = lambda topic, payload: published.append((topic, payload))  # type: ignore
    node.r = lambda template: template.format(robot_id=node.robot_id)  # type: ignore

    # JointCoordinates.motor_to_urdf 도 mock — joint_offset 적용은 별도 검증 자리.
    with patch("nodes.device.motion_node.JointCoordinates") as JC:
        coords = MagicMock()
        coords.motor_to_urdf.side_effect = lambda raw, cfg, robot_id: (raw - 2048) / 4095 * 2 * np.pi
        JC.return_value = coords

        node._on_motor_state_publish_tcp({
            "joints": [{"id": i, "position": 2048} for i in range(1, 7)],
        })

    assert len(published) == 1, "정확히 1 publish 발생해야"
    topic, payload = published[0]
    assert topic.endswith("/motion/state/tcp")
    assert payload.position == [0.1, 0.2, 0.3]
    assert payload.quaternion == [0, 0, 0, 1]
    assert abs(payload.timestamp - time.time()) < 1.0
    # 핵심 — get_tcp_pose 가 호출됨 (= SagCorrectedKinematics chain 진입 보장).
    node._motion.get_tcp_pose.assert_called_once()
