"""MotionPreview plan_trajectory 결정론 test (fast loop — pybullet/URDF 미부팅).

의미 있는 검증 (통과용 X): 궤적 수집이 실제 Ruckig + seed-연쇄 IK 로 돌고,
도달 가능/불가가 feasible 로 정확히 갈리며, tcp_trace 가 프레임과 1:1 이라는 것.
FakeKinematics 로 "IK 실패 지점" 을 결정적으로 심어 각 구멍을 재현한다:
  - MoveL 전 구간 도달 → feasible, 프레임 다수
  - MoveL 중간 도달 불가 → feasible=False + 도달 가능 지점까지 부분 프레임
  - MoveJ(pose) 목표 도달 → feasible
  - MoveJ(pose) 목표 도달 불가 → feasible=False + 프레임 0
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Sequence

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from apps.config import DeploymentConfig, DriverMode, load_robots
from apps.resolve import resolve_host_deps
from framework.runtime.app import Runtime
from infra.transport.zenoh import ZenohTransport
from modules.motor.contract import MotorKind
from modules.motion_preview.contract import (
    MotionPreview,
    PlanPreviewRequest,
    PlanPreviewResponse,
    PreviewMode,
    PreviewPoseTarget,
)
from modules.motion_preview.module import (
    MotionPreviewModule,
    PreviewRobotSpec,
    plan_trajectory,
)

_LOCAL_CFG = {"mode": "peer", "scouting": {"multicast": {"enabled": False}}}

_SO101 = "so101_6dof_0"
# x 절대값이 이 값을 넘는 TCP 위치는 "도달 불가" — MoveL 이 중간에 벗어나는 걸 심음.
_REACH_LIMIT_X = 1.0


class _FakeKinematics:
    """Kinematics Protocol 최소 충족 fake. fk: tcp.x = joints[0] (결정적).

    ik: x 가 reach 범위 안이면 [x,0,...] 반환, 밖이면 None (도달 불가 주입)."""

    DOF = 6

    def initialize(self) -> None: ...

    def close(self) -> None: ...

    @property
    def dof(self) -> int:
        return self.DOF

    @property
    def tcp_link_name(self) -> str:
        return "tcp"

    def fk(self, joint_angles: Sequence[float]):  # noqa: ANN201
        return (float(joint_angles[0]), 0.1, 0.2), (0.0, 0.0, 0.0, 1.0)

    def ik(  # noqa: ANN201
        self,
        target_position,  # noqa: ANN001
        target_quaternion,  # noqa: ANN001
        current_joint_angles=None,  # noqa: ANN001
        restarts=None,  # noqa: ANN001
    ):
        if abs(target_position[0]) > _REACH_LIMIT_X:
            return None
        return [float(target_position[0])] + [0.0] * (self.DOF - 1)

    def fk_to_matrix(self, joint_angles: Sequence[float]):  # noqa: ANN201
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], (
            float(joint_angles[0]),
            0.1,
            0.2,
        )

    def joint_limits(self, n=None):  # noqa: ANN001, ANN201
        return [(-3.14, 3.14)] * self.DOF

    def self_collision(self, joint_angles: Sequence[float]) -> bool:
        return False

    def floor_collision(self, joint_angles: Sequence[float], floor_z: float) -> bool:
        return False

    def set_obstacle_points(self, points) -> None:  # noqa: ANN001
        ...

    def obstacle_collision(self, joint_angles, *, gripper_open=False) -> bool:  # noqa: ANN001
        return False


@pytest.fixture
def spec() -> PreviewRobotSpec:
    """실 so101 arm 이름은 재사용하되 한계는 test 용으로 높여 수집을 빠르게
    (실 궤적 로직은 동일, wall-clock 만 sub-second)."""
    robot = load_robots()[_SO101]
    arm = [m for m in robot.motors if m.kind != MotorKind.GRIPPER]
    n = len(arm)
    return PreviewRobotSpec(
        kinematics_factory=lambda p: _FakeKinematics(),
        urdf_path=Path("unused_in_fake.urdf"),
        arm_specs=arm,
        joint_max_velocity=[10.0] * n,
        joint_max_acceleration=[50.0] * n,
        joint_max_jerk=[500.0] * n,
        cartesian_max_velocity=5.0,
        cartesian_max_acceleration=20.0,
        cartesian_max_jerk=200.0,
    )


def _target(x: float) -> PreviewPoseTarget:
    return PreviewPoseTarget(position=(x, 0.1, 0.2), rpy_deg=(0.0, 0.0, 0.0))


def test_move_l_reachable(spec: PreviewRobotSpec) -> None:
    start = [0.0] * spec.arm_specs.__len__()
    res = plan_trajectory(
        _FakeKinematics(), spec, start, _target(0.5), PreviewMode.MOVE_L
    )
    assert res.feasible is True
    assert res.fail_at_sample is None
    assert len(res.frames) >= 2  # Ruckig 다수 틱
    # tcp_trace 는 프레임과 1:1 (각 프레임 FK).
    assert len(res.tcp_trace) == len(res.frames)
    assert res.joint_names == [s.name for s in spec.arm_specs]
    # 마지막 TCP 는 목표 x 근처 (fk: tcp.x = joints[0] = ik(x) = x).
    assert res.tcp_trace[-1][0] == pytest.approx(0.5, abs=1e-3)


def test_move_l_unreachable_midpath(spec: PreviewRobotSpec) -> None:
    start = [0.0] * len(spec.arm_specs)
    # 직선이 x:0→2 로 가다 x>1 에서 IK 불가 → 부분 프레임 + feasible=False.
    res = plan_trajectory(
        _FakeKinematics(), spec, start, _target(2.0), PreviewMode.MOVE_L
    )
    assert res.feasible is False
    assert res.frames, "도달 가능 지점까지는 프레임이 있어야 함"
    assert res.fail_at_sample == len(res.frames)
    assert len(res.tcp_trace) == len(res.frames)
    # 끊긴 지점의 TCP 는 reach 한계 안 (x <= 1).
    assert res.tcp_trace[-1][0] <= _REACH_LIMIT_X + 1e-6


def test_move_j_pose_reachable(spec: PreviewRobotSpec) -> None:
    start = [0.0] * len(spec.arm_specs)
    res = plan_trajectory(
        _FakeKinematics(), spec, start, _target(0.5), PreviewMode.MOVE_J_POSE
    )
    assert res.feasible is True
    assert len(res.frames) >= 2
    assert len(res.tcp_trace) == len(res.frames)


def test_move_j_pose_unreachable(spec: PreviewRobotSpec) -> None:
    start = [0.0] * len(spec.arm_specs)
    res = plan_trajectory(
        _FakeKinematics(), spec, start, _target(2.0), PreviewMode.MOVE_J_POSE
    )
    assert res.feasible is False
    assert res.frames == []
    assert res.tcp_trace == []
    assert res.fail_at_sample == 0
    assert "도달 불가" in res.message


# ─── e2e: 실 PybulletKinematics + 와이어 디스패치 (sim) ────────────
# fake 가 못 잡는 것 = "실 IK/URDF 로도 도는가" + "MoveL 이 진짜 직선인가" +
# resolve→module→runtime.call 의 실 배선. mock motor 불필요 (start_joints 를
# 요청으로 실음 = preview 의 stateless 계약 그대로).


@pytest.mark.sim
async def test_plan_real_so101_wire_e2e() -> None:
    transport = ZenohTransport(_LOCAL_CFG)
    time.sleep(0.05)
    runtime = Runtime(transport)
    robots = load_robots()
    deploy = DeploymentConfig(driver_mode=DriverMode.MOCK)
    deps = resolve_host_deps("motion_preview", robots, deploy)
    runtime.add_module(MotionPreviewModule, **deps)
    await runtime.start()
    try:
        spec = deps["robots"][_SO101]
        # 비특이(non-singular) 시작 자세 — home([0]*6) 은 IK-degenerate 라 시드가
        # 정확한 해여도 PyBullet 수치 솔버가 1cm 내로 못 맞춘다(=start 에서 IK None).
        # 이 경우 preview 는 정직하게 feasible=False (motion move_l 도 동일 IK). 그래서
        # e2e 는 일반 자세를 골라 "정상 경로가 실제로 도는가" 를 본다.
        start = [0.2, 0.6, -1.0, 0.4, 0.3, 0.0]
        # 도달 가능 목표 = 그 자세 FK 위치에서 현재 orientation 유지 + y +2cm.
        kin = spec.kinematics_factory(spec.urdf_path)
        kin.initialize()
        try:
            pos, quat = kin.fk(start)
            r0, r1, r2 = (
                float(v) for v in Rotation.from_quat(quat).as_euler("XYZ", degrees=True)
            )
        finally:
            kin.close()

        reach = PreviewPoseTarget(
            position=(pos[0], pos[1] + 0.02, pos[2]), rpy_deg=(r0, r1, r2)
        )
        res = await runtime.module_runtime.call(
            MotionPreview.Service.PLAN,
            PlanPreviewRequest(
                robot_id=_SO101,
                start_joints=start,
                target=reach,
                mode=PreviewMode.MOVE_L,
            ),
            PlanPreviewResponse,
        )
        assert res.feasible, res.message
        assert len(res.frames) >= 2
        assert len(res.tcp_trace) == len(res.frames)
        # MoveL = TCP 직선: 시작~끝 선분에서 최대 수직 이탈이 작아야 함.
        pts = np.array(res.tcp_trace, dtype=float)
        a, b = pts[0], pts[-1]
        ab = b - a
        length = float(np.linalg.norm(ab))
        if length > 1e-6:
            devs = np.linalg.norm(np.cross(pts - a, ab / length), axis=1)
            assert devs.max() < 5e-3, f"MoveL 트레이스가 직선 아님 (dev={devs.max()})"

        # 도달 불가 — workspace 밖 (real IK 도 None → feasible=False).
        far = PreviewPoseTarget(position=(5.0, 5.0, 5.0), rpy_deg=(0.0, 0.0, 0.0))
        res2 = await runtime.module_runtime.call(
            MotionPreview.Service.PLAN,
            PlanPreviewRequest(
                robot_id=_SO101,
                start_joints=start,
                target=far,
                mode=PreviewMode.MOVE_J_POSE,
            ),
            PlanPreviewResponse,
        )
        assert res2.feasible is False
    finally:
        await runtime.stop()
        transport.close()
