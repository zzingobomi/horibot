"""Kinematics Protocol — kinematics 엔진 adapter 의 통합 인터페이스.

구현체(PybulletKinematics 등)는 *ideal URDF 기구학만*. calibration 보정은 외부:
- joint_offset → raw↔rad 변환 시 (Motion D2)
- sag → SagCorrected decorator (Motion D4)
- link_offset → adapter 가 patched URDF 로 로드 (Motion D4)

chain = tcp link 의 ancestor revolute joint 만 (gripper 등 sibling 가지 제외) →
dof = arm 자유도 (so101_6dof=6, omx_f=5).
"""

from __future__ import annotations

from typing import Protocol, Sequence, TypeAlias

Position3: TypeAlias = tuple[float, float, float]
Quaternion: TypeAlias = tuple[float, float, float, float]  # [x, y, z, w]
RotMatrix3x3: TypeAlias = list[list[float]]


class IKSolverError(Exception):
    """IK 알고리즘 관련 예외 base."""


class Kinematics(Protocol):
    """URDF 기반 forward/inverse kinematics + joint limit + self-collision.

    thread-safe — 내부 mutex 사용.
    """

    def initialize(self) -> None:
        """URDF 로드 + 엔진 준비. fk/ik 전에 호출 (boot 1회)."""
        ...

    def close(self) -> None:
        """엔진 자원 해제."""
        ...

    @property
    def dof(self) -> int:
        """arm 자유도 (tcp 체인의 revolute joint 수, gripper 제외)."""
        ...

    @property
    def tcp_link_name(self) -> str:
        """TCP link 이름 (URDF link name). fk/ik 기준점."""
        ...

    def fk(self, joint_angles: Sequence[float]) -> tuple[Position3, Quaternion]:
        """joints (dof,) rad → ee position (3,) m + quaternion [x,y,z,w], base frame."""
        ...

    def ik(
        self,
        target_position: Position3,
        target_quaternion: Quaternion | None,
        current_joint_angles: Sequence[float] | None = None,
        restarts: int | None = None,
    ) -> list[float] | None:
        """target pose → joints (dof,) rad. 수렴 실패 / self-collision 시 None.

        - current_joint_angles=None → 0 벡터 seed
        - target_quaternion=None → position-only IK
        - restarts=None → 구현체 default (실행용 최대 예산). 배치 판정
          (RESOLVE_REACHABLE deepening) 은 작은 예산으로 probe — 불가 후보 기각
          비용이 재시작 수에 비례해서 (실패만 풀비용을 냄).
        """
        ...

    def fk_to_matrix(
        self, joint_angles: Sequence[float]
    ) -> tuple[RotMatrix3x3, Position3]:
        """fk(joints) 의 (R, position) 표현."""
        ...

    def joint_limits(self, n: int | None = None) -> list[tuple[float, float]]:
        """chain joint limit (lower, upper) rad. n 지정 시 처음 n개만."""
        ...

    def self_collision(self, joint_angles: Sequence[float]) -> bool:
        """주어진 자세에서 self-collision 여부."""
        ...
