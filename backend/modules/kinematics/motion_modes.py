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
        target_quaternion: Quaternion | None = None,
    ) -> list[float] | None:
        """
        TCP를 target_position으로 이동하는 관절 각도 반환.
        target_quaternion=None 이면 5-DOF 답게 자세 자유 (기존 동작).
        값이 있으면 PyBullet IK 가 그 자세 충족하는 해 찾음.
        반환: 관절 각도 (라디안), IK 실패 시 None.
        """
        return self._solver.ik(target_position, target_quaternion, current_joint_angles)
