"""SagCorrectedKinematics — 중력 sag 보정 Decorator (Motion D4).

kinematics.py 선언 ("sag → SagCorrected decorator") 의 구현. 옛 backend
SagCorrectedKinematics 패턴 — inner(PybulletKinematics)는 ideal 기구학만,
sag 는 여기서 양방향:

    fk(θ_meas)  = inner.fk(θ_meas + sagΔ(θ_meas))     # 실 자세 = 측정 + 처짐
    ik(pose)    = actual_to_commanded(inner.ik(pose))  # 처짐 선보상 명령

sag 수식 SSOT = FkChain.apply_gravity_sag / actual_to_commanded (offline BA 와
동일 모델 — 수학 중복 X). FkChain 은 inner 와 **같은 (patched) URDF** 로 빌드 —
link_offset 이 이미 구워진 기하 위에서 torque 계산 (BA 의 fk_with_sag 등가).

k 가 전부 0 이면 순수 delegate (mock / 캘 없는 robot 무비용).
"""

from __future__ import annotations

from typing import Sequence

import numpy as np

from .fk_chain import FkChain
from .kinematics import Kinematics, Position3, Quaternion, RotMatrix3x3


class SagCorrectedKinematics:
    """Kinematics Protocol 만족 — inner 위 sag 양방향 보정."""

    def __init__(
        self,
        inner: Kinematics,
        fk_chain: FkChain,
        k_stiff: Sequence[float],
        sag_joint_indices: list[int],
    ) -> None:
        self._inner = inner
        self._fk_chain = fk_chain
        self._k = np.asarray(k_stiff, dtype=np.float64)
        self._sag_idx = list(sag_joint_indices)

    # ── sag 변환 ──────────────────────────────────────────────

    def _to_actual(self, joint_angles: Sequence[float]) -> list[float]:
        out = self._fk_chain.apply_gravity_sag(
            np.asarray(joint_angles, dtype=np.float64), self._k, self._sag_idx
        )
        return [float(a) for a in out]

    def _to_commanded(self, actual: Sequence[float]) -> list[float]:
        out = self._fk_chain.actual_to_commanded(
            np.asarray(actual, dtype=np.float64), self._k, self._sag_idx
        )
        return [float(a) for a in out]

    # ── Kinematics Protocol ───────────────────────────────────

    def initialize(self) -> None:
        self._inner.initialize()

    def close(self) -> None:
        self._inner.close()

    @property
    def dof(self) -> int:
        return self._inner.dof

    @property
    def tcp_link_name(self) -> str:
        return self._inner.tcp_link_name

    def fk(self, joint_angles: Sequence[float]) -> tuple[Position3, Quaternion]:
        return self._inner.fk(self._to_actual(joint_angles))

    def ik(
        self,
        target_position: Position3,
        target_quaternion: Quaternion | None,
        current_joint_angles: Sequence[float] | None = None,
        restarts: int | None = None,
    ) -> list[float] | None:
        # seed 는 근사면 충분 (측정각 ≈ 실제각, sag ~수° — IK 수렴 seed 용도)
        actual = self._inner.ik(
            target_position, target_quaternion, current_joint_angles, restarts
        )
        if actual is None:
            return None
        return self._to_commanded(actual)

    def fk_to_matrix(
        self, joint_angles: Sequence[float]
    ) -> tuple[RotMatrix3x3, Position3]:
        return self._inner.fk_to_matrix(self._to_actual(joint_angles))

    def joint_limits(self, n: int | None = None) -> list[tuple[float, float]]:
        return self._inner.joint_limits(n)

    def self_collision(self, joint_angles: Sequence[float]) -> bool:
        return self._inner.self_collision(self._to_actual(joint_angles))

    def floor_collision(self, joint_angles: Sequence[float], floor_z: float) -> bool:
        return self._inner.floor_collision(self._to_actual(joint_angles), floor_z)

    def set_obstacle_points(
        self, points: Sequence[tuple[float, float, float]] | None
    ) -> None:
        self._inner.set_obstacle_points(points)

    def obstacle_collision(
        self, joint_angles: Sequence[float], *, gripper_open: bool = False
    ) -> bool:
        return self._inner.obstacle_collision(
            self._to_actual(joint_angles), gripper_open=gripper_open
        )
