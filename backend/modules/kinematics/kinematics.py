"""Kinematics Protocol — kinematics adapter 의 통합 인터페이스.

multi_robot_architecture.md §3.1 / §3.2 참조.

책임 경계:
- 본 Protocol 구현체 (PybulletKinematics / MujocoKinematics) 는 *ideal URDF 기구학만*
- sag / joint_offset 같은 보정은 외부가 적용:
  · joint_offset → `JointCoordinates` (motor raw ↔ URDF rad 변환 시)
  · sag → `SagCorrectedKinematics` (좌표계 어댑터, fk 출력 / ik 입력 양쪽 wrap)
  · link_offset → adapter 생성자에서 patched URDF 로 로드 (이미 URDF 자체에 박힘)

사용:
    inner = PybulletKinematics(urdf_path)
    kin: Kinematics = SagCorrectedKinematics(inner, link_coords=..., sag_coords=...)
    pos, quat = kin.fk(joints)
"""

from __future__ import annotations

from typing import Protocol, Sequence, TypeAlias

Position3: TypeAlias = tuple[float, float, float]
Quaternion: TypeAlias = tuple[float, float, float, float]  # [x, y, z, w]
RotMatrix3x3: TypeAlias = list[list[float]]


class IKSolverError(Exception):
    """IK 알고리즘 관련 예외 base."""


class IKConvergenceError(IKSolverError):
    """IK 수렴 실패 (max_iter 도달 또는 pos_error_limit 초과).

    호환성 위해 현재 구현체는 None 반환 — exception raise 는 미래.
    """


class Kinematics(Protocol):
    """URDF 기반 forward/inverse kinematics + joint limit + collision.

    thread-safe — 내부 mutex 또는 immutable 자료 구조 사용.
    """

    @property
    def dof(self) -> int:
        """관절 자유도 (arm 만, gripper 제외). omx_f=5, so101_6dof=6."""
        ...

    @property
    def tcp_link_name(self) -> str:
        """TCP link 이름 (URDF link name). fk/ik 기준점."""
        ...

    def fk(
        self, joint_angles: Sequence[float]
    ) -> tuple[Position3, Quaternion]:
        """joints (dof,) URDF rad → ee position (3,) m + quaternion [x,y,z,w], base frame."""
        ...

    def ik(
        self,
        target_position: Position3,
        target_quaternion: Quaternion | None,
        current_joint_angles: Sequence[float] | None = None,
    ) -> list[float] | None:
        """target pose → joints (dof,) URDF rad.

        - current_joint_angles=None → 0 벡터 seed
        - target_quaternion=None → position-only IK
        - 수렴 실패 시 None
        """
        ...

    def fk_to_matrix(
        self, joint_angles: Sequence[float]
    ) -> tuple[RotMatrix3x3, Position3]:
        """fk(joints) 의 (R, position) 표현."""
        ...

    def joint_limits(
        self, n: int | None = None
    ) -> list[tuple[float, float]]:
        """URDF 조인트 limit (lower, upper) rad. n 지정 시 처음 n개만 (arm)."""
        ...

    def self_collision(self, joint_angles: Sequence[float]) -> bool:
        """주어진 자세에서 self-collision 여부."""
        ...
