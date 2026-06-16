"""SagCorrectedKinematics — sag 보정을 inner Kinematics 의 fk 출력 / ik 입력에 적용.

multi_robot_architecture.md §3.2 참조.

좌표계 어댑터 — inner 가 PybulletKinematics / MujocoKinematics 어느 쪽이든 sag
보정은 한 번만 작성 + 양쪽 다 적용. inner 가 *ideal* URDF 기구학이라 unit test
용이 (sag 끄고 inner 만으로 검증 가능).

link_offset 은 inner 의 URDF patch 로 이미 처리됨 — 여기서는 sag (자세 의존
중력 처짐) 만 wrap. 자체 numpy FK 는 `FkChain` 인스턴스에 위임.
"""

from __future__ import annotations

import logging
import threading
from typing import Sequence

import numpy as np

from core.coords.link_coordinates import LinkCoordinates
from core.coords.sag_coordinates import SagCoordinates
from modules.kinematics.fk_chain import FkChain
from modules.kinematics.kinematics import (
    Kinematics,
    Position3,
    Quaternion,
    RotMatrix3x3,
)

logger = logging.getLogger(__name__)

# sag 모델은 J2, J3 에만 적용 (motor id 2, 3). J1/J4/J5 의 sag 는 측정 noise
# 수준이라 모델 단순성 위해 제외. 본 hardcode 는 OMX-F 가정 — SO-101 sag 캘
# 진입 시 robot 별 일반화 필요 (storage_layer.md §13.6 (5.5)(a) follow-up).
_SAG_JOINT_MOTOR_IDS: list[int] = [2, 3]
# 위 motor id 를 arm 안 0-indexed position 으로 변환. arm motor id 가 1-base
# 연속이라는 가정 (motors.yaml 컨벤션) — 어긋나면 fk_chain.apply_gravity_sag
# 의 sag_joint_indices 인자에 명시 매핑 필요.
_SAG_JOINT_ARM_INDICES: list[int] = [i - 1 for i in _SAG_JOINT_MOTOR_IDS]


class SagCorrectedKinematics:
    """sag 보정 wrapper. inner `Kinematics` Protocol 그대로 만족.

    - fk: inner.fk(commanded → actual via sag)
    - ik: inner.ik(target, current → actual via sag); 결과를 actual → commanded
    """

    def __init__(
        self,
        inner: Kinematics,
        link_coords: LinkCoordinates,
        sag_coords: SagCoordinates,
        fk_chain: FkChain,
    ):
        self._inner = inner
        self._link_coords = link_coords
        self._sag_coords = sag_coords
        self._fk_chain = fk_chain
        self._cache_lock = threading.Lock()
        # 부팅 시 caches 는 empty (sag 비활성). calibration_node 가 set_offsets 후
        # `reload_calibration()` 호출하면 link/sag array 실제 값으로 채워짐.
        n_arm = fk_chain.n_arm
        self._link_trans_array = np.zeros((n_arm, 3), dtype=np.float64)
        self._link_rot_array = np.zeros((n_arm, 3), dtype=np.float64)
        self._sag_k_array = np.zeros(
            (len(_SAG_JOINT_MOTOR_IDS),), dtype=np.float64
        )
        self._sag_enabled = False

    # ─── Kinematics Protocol ────────────────────────────────────

    @property
    def dof(self) -> int:
        return self._inner.dof

    @property
    def tcp_link_name(self) -> str:
        return self._inner.tcp_link_name

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

    def reload_calibration(self) -> None:
        """calibration_node 가 LinkCoordinates / SagCoordinates 갱신 후 호출.

        link offset 의 URDF patch 부분은 inner (PybulletKinematics) 가 부팅 시
        1회 적용 — 본 reload 는 sag array + link rotvec/trans array (sag 보정 계산에
        사용) 만 갱신. URDF 자체는 재로드 X (런타임 reload 는 본 design 의 범위 밖).
        """
        with self._cache_lock:
            n_arm = self._fk_chain.n_arm
            link_offsets = self._link_coords.snapshot()
            self._link_trans_array = np.array(
                [link_offsets.get_trans(i + 1) for i in range(n_arm)],
                dtype=np.float64,
            )
            self._link_rot_array = np.array(
                [link_offsets.get_rot(i + 1) for i in range(n_arm)],
                dtype=np.float64,
            )
            sag = self._sag_coords.snapshot()
            self._sag_k_array = sag.as_array_for_joints(_SAG_JOINT_MOTOR_IDS)
            self._sag_enabled = bool(
                self._sag_k_array.size > 0
                and float(np.max(np.abs(self._sag_k_array))) > 1e-12
            )
            if self._sag_enabled:
                ks = ", ".join(
                    f"J{jid}={k:+.5f}"
                    for jid, k in zip(_SAG_JOINT_MOTOR_IDS, self._sag_k_array)
                )
                logger.info(f"SagCorrectedKinematics sag 적용: {ks}")

    def _commanded_to_actual(self, joint_angles: list[float]) -> list[float]:
        """모터 encoder reading(commanded) → 실제 link end 의 URDF angle (actual)."""
        with self._cache_lock:
            n_arm = self._fk_chain.n_arm
            if not self._sag_enabled or len(joint_angles) < n_arm:
                return list(joint_angles)
            arm = np.asarray(joint_angles[:n_arm], dtype=np.float64)
            actual = self._fk_chain.apply_gravity_sag(
                arm,
                self._sag_k_array,
                _SAG_JOINT_ARM_INDICES,
                self._link_trans_array,
                self._link_rot_array,
            )
            return list(actual) + list(joint_angles[n_arm:])

    def _actual_to_commanded(self, joint_angles: list[float]) -> list[float]:
        """IK 결과(actual, URDF static FK target) → 모터 명령 commanded. 1차 근사."""
        with self._cache_lock:
            n_arm = self._fk_chain.n_arm
            if not self._sag_enabled or len(joint_angles) < n_arm:
                return list(joint_angles)
            arm = np.asarray(joint_angles[:n_arm], dtype=np.float64)
            commanded = self._fk_chain.actual_to_commanded(
                arm,
                self._sag_k_array,
                _SAG_JOINT_ARM_INDICES,
                self._link_trans_array,
                self._link_rot_array,
            )
            return list(commanded) + list(joint_angles[n_arm:])
