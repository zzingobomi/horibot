"""CorrectedIKSolver — sag 보정을 inner IKSolver 의 fk 출력 / ik 입력에 적용.

multi_robot_architecture.md §3.2 참조.

Decorator pattern — inner 가 PybulletIKSolver / MujocoIKSolver 어느 쪽이든 sag
보정은 한 번만 작성 + 양쪽 다 적용. inner 가 *ideal* URDF 기구학이라 unit test
용이 (sag 끄고 inner 만으로 검증 가능).

link_offset 은 inner 의 URDF patch 로 이미 처리됨 — 여기서는 sag (J2/J3 자세 의존
중력 처짐) 만 wrap.

기존 `PybulletSolver` 에 박혀있던 sag 코드 ([solver.py](solver.py) 의 _commanded_to_actual /
_actual_to_commanded / _reload_sag_cache) 를 이 클래스로 분리.
"""

from __future__ import annotations

import logging
import threading
from typing import Sequence

import numpy as np

from core.coords.link_coordinates import LinkCoordinates
from core.coords.sag_coordinates import SagCoordinates
from modules.kinematics.fk_chain import (
    actual_to_commanded,
    apply_gravity_sag,
)
from modules.kinematics.iksolver import (
    IKSolver,
    Position3,
    Quaternion,
    RotMatrix3x3,
)

logger = logging.getLogger(__name__)

# sag 모델은 J2, J3 에만 적용 (motor id 2, 3). J1/J4/J5 의 sag 는 측정 noise
# 수준이라 모델 단순성 위해 제외.
_SAG_JOINT_IDS: list[int] = [2, 3]
_ARM_DOF: int = 5


class CorrectedIKSolver:
    """sag 보정 wrapper. inner `IKSolver` Protocol 그대로 만족.

    - fk: inner.fk(commanded → actual via sag)
    - ik: inner.ik(target, current → actual via sag); 결과를 actual → commanded
    """

    def __init__(
        self,
        inner: IKSolver,
        link_coords: LinkCoordinates,
        sag_coords: SagCoordinates,
    ):
        self._inner = inner
        self._link_coords = link_coords
        self._sag_coords = sag_coords
        self._cache_lock = threading.Lock()
        self._reload_caches()

    # ─── IKSolver Protocol ─────────────────────────────────────

    @property
    def dof(self) -> int:
        return self._inner.dof

    @property
    def ee_link_name(self) -> str:
        return self._inner.ee_link_name

    def fk(
        self, joint_angles: Sequence[float]
    ) -> tuple[Position3, Quaternion]:
        """encoder reading(commanded) → 실제 ee 자세 (sag 반영)."""
        actual = self._commanded_to_actual(list(joint_angles))
        return self._inner.fk(actual)

    def ik(
        self,
        target_position: Position3,
        target_quaternion: Quaternion | None,
        current_joint_angles: Sequence[float] | None = None,
    ) -> list[float] | None:
        """target ee pose → motor 명령 (commanded, sag 역보정 적용)."""
        current_actual = (
            self._commanded_to_actual(list(current_joint_angles))
            if current_joint_angles
            else None
        )
        result = self._inner.ik(target_position, target_quaternion, current_actual)
        if result is None:
            return None
        return self._actual_to_commanded(result)

    def fk_to_matrix(
        self, joint_angles: Sequence[float]
    ) -> tuple[RotMatrix3x3, Position3]:
        """fk(joints) 의 (R, position) 표현 — sag 반영된 자세 기준."""
        actual = self._commanded_to_actual(list(joint_angles))
        return self._inner.fk_to_matrix(actual)

    def joint_limits(
        self, n: int | None = None
    ) -> list[tuple[float, float]]:
        return self._inner.joint_limits(n)

    def self_collision(self, joint_angles: Sequence[float]) -> bool:
        # sag 가 self-collision 거리에는 미미 → inner 그대로
        return self._inner.self_collision(joint_angles)

    # ─── sag 보정 / 캐시 ──────────────────────────────────────────

    def _reload_caches(self) -> None:
        """LinkCoordinates / SagCoordinates 에서 array 재로드.

        COMMIT 후 외부에서 호출하면 재시작 없이 반영. 단 LinkCoordinates 는 URDF
        patch 영향도라 inner solver 도 재로드 필요 (실용상 process 재시작 권장).
        """
        with self._cache_lock:
            link_offsets = self._link_coords.snapshot()
            self._link_trans_array = np.array(
                [link_offsets.get_trans(i + 1) for i in range(_ARM_DOF)],
                dtype=np.float64,
            )
            self._link_rot_array = np.array(
                [link_offsets.get_rot(i + 1) for i in range(_ARM_DOF)],
                dtype=np.float64,
            )
            sag = self._sag_coords.snapshot()
            self._sag_k_array = sag.as_array_for_joints(_SAG_JOINT_IDS)
            self._sag_enabled = bool(
                self._sag_k_array.size > 0
                and float(np.max(np.abs(self._sag_k_array))) > 1e-12
            )
            if self._sag_enabled:
                ks = ", ".join(
                    f"J{jid}={k:+.5f}"
                    for jid, k in zip(_SAG_JOINT_IDS, self._sag_k_array)
                )
                logger.info(f"CorrectedIKSolver sag 적용: {ks}")

    def _commanded_to_actual(self, joint_angles: list[float]) -> list[float]:
        """모터 encoder reading(commanded) → 실제 link end 의 URDF angle (actual)."""
        with self._cache_lock:
            if not self._sag_enabled or len(joint_angles) < _ARM_DOF:
                return list(joint_angles)
            arm = np.asarray(joint_angles[:_ARM_DOF], dtype=np.float64)
            actual = apply_gravity_sag(
                arm,
                self._sag_k_array,
                self._link_trans_array,
                self._link_rot_array,
            )
            return list(actual) + list(joint_angles[_ARM_DOF:])

    def _actual_to_commanded(self, joint_angles: list[float]) -> list[float]:
        """IK 결과(actual, URDF static FK target) → 모터 명령 commanded. 1차 근사."""
        with self._cache_lock:
            if not self._sag_enabled or len(joint_angles) < _ARM_DOF:
                return list(joint_angles)
            arm = np.asarray(joint_angles[:_ARM_DOF], dtype=np.float64)
            commanded = actual_to_commanded(
                arm,
                self._sag_k_array,
                self._link_trans_array,
                self._link_rot_array,
            )
            return list(commanded) + list(joint_angles[_ARM_DOF:])

    # 호환: 기존 코드가 호출하던 `_reload_sag_cache` 이름 alias
    def _reload_sag_cache(self) -> None:
        self._reload_caches()
