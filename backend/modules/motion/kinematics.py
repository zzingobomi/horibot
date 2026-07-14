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

    def floor_collision(self, joint_angles: Sequence[float], floor_z: float) -> bool:
        """주어진 자세에서 수평 바닥 평면(z=floor_z, base frame)을 침투하는지.

        planner 충돌 게이트 최소형 (grasp_redesign_journey.md §5.7 — 바닥은 평면
        하나). base 쪽 고정 링크는 제외 — 로봇이 그 평면 위에 설치돼 있어 상시
        접촉이 상수 (검출 floor_z 오차 ±cm 에 전 후보 영구 기각 방지).
        """
        ...

    def set_obstacle_points(
        self, points: Sequence[tuple[float, float, float]] | None
    ) -> None:
        """장애물 점군(base frame, m) scene 설정 — obstacle_collision 의 대상.

        관측 점군(물체/이웃)이 소스 (grasp_redesign_journey.md §10.4-3 그리퍼↔
        물체 충돌 게이트). None/빈 = 해제. 배치 판정(RESOLVE_REACHABLE) lifecycle:
        판정 시작에 set → 다수 자세 검사 → 끝나면 반드시 해제 (잔존 시 이후
        판정이 남의 물체에 기각되는 침묵 오동작).
        """
        ...

    def obstacle_collision(
        self, joint_angles: Sequence[float], *, gripper_open: bool = False
    ) -> bool:
        """주어진 자세에서 로봇 링크가 설정된 장애물 점군을 침투하는지.

        gripper_open=True 면 chain 밖 movable 관절(그리퍼 조)을 URDF 상한
        (= 벌림 — so101 규약: 양수가 open)에 두고 검사 — 파지 접근은 조를 벌린
        채라 그 부피가 실 충돌 형상. 점군 미설정이면 False.
        """
        ...
