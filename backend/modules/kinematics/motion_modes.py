from dataclasses import dataclass

from .solver import PybulletSolver, Position3, Quaternion


@dataclass
class TCPPose:
    position: Position3
    quaternion: Quaternion


@dataclass
class MotionModes:
    def __init__(self) -> None:
        self._solver = PybulletSolver()

    # ─── FK ────────────────────────────────────────────────────

    def get_tcp_pose(self, joint_angles: list[float]) -> TCPPose:
        position, quaternion = self._solver.fk(joint_angles)
        return TCPPose(position=position, quaternion=quaternion)

    # ─── Move TCP ─────────────────────────────────────────────────

    def move_tcp(
        self,
        target_position: Position3,
        current_joint_angles: list[float],
    ) -> list[float] | None:
        """
        TCP를 target_position으로 이동하는 관절 각도 반환.
        5-DOF arm이라 orientation 제약 없이 position only IK.
        반환: 관절 각도 (라디안), IK 실패 시 None.
        """
        return self._solver.ik(target_position, None, current_joint_angles)
