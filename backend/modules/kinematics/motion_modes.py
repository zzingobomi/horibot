from dataclasses import dataclass
from typing import cast

from core.robot.robot_registry import RobotRegistry

from .kinematics import Kinematics
from .registry import Position3, Quaternion, get_default_kinematics


@dataclass
class TCPPose:
    position: Position3
    quaternion: Quaternion


@dataclass
class MotionModes:
    def __init__(self, robot_id: str | None = None) -> None:
        """robot_id 명시 시 그 robot 의 kinematics 사용. None = default (N=1 편의).

        multi-robot 환경 (host_mock 자리에 enabled robot 이 2개 + motion_node 가
        self.robot_id ≠ default 인 경우) 에서 dof mismatch 차단 — 반드시 명시.
        """
        if robot_id is None:
            self._solver: Kinematics = get_default_kinematics()
        else:
            # RobotRegistry.get_kinematics 의 return 은 lazy import 위해 object —
            # Kinematics Protocol 충족하는 구현체 (SagCorrectedKinematics) 반환.
            self._solver = cast(Kinematics, RobotRegistry().get_kinematics(robot_id))

    # ─── FK ────────────────────────────────────────────────────

    def get_tcp_pose(self, joint_angles: list[float]) -> TCPPose:
        position, quaternion = self._solver.fk(joint_angles)
        return TCPPose(position=position, quaternion=quaternion)

    # ─── Servo TCP (직접 IK + publish, planner 우회) ───────────────

    def servo_tcp(
        self,
        target_position: Position3,
        target_quaternion: Quaternion | None,
        current_joint_angles: list[float],
    ) -> list[float] | None:
        """절대 TCP target 의 IK 만 풂 — trajectory planner 우회.

        - `target_quaternion=None` → position-only IK (5DOF 또는 6DOF orientation 무시)
        - 6DOF + quaternion → 6DOF IK (정확한 자세)
        - 반환: 관절 각도 (라디안), IK 실패 시 None.
        """
        return self._solver.ik(
            target_position, target_quaternion, current_joint_angles
        )
