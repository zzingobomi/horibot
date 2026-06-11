"""PyBullet 기반 IKSolver adapter — ideal URDF 기구학 only.

multi_robot_architecture.md §3.1 참조.

책임:
- URDF 로드 (link_offsets 적용된 patched URDF). PyBullet DIRECT 모드
- fk / ik / fk_to_matrix / joint_limits / self_collision
- thread-safe (`_sim_lock`)

책임 외 (외부 layer 가 적용):
- sag 보정 → `modules.kinematics.corrected.CorrectedIKSolver` (Decorator)
- joint_offset → `core.coords.joint_coordinates.JointCoordinates` (raw↔rad 변환 시)

기존 [solver.py](../solver.py) 의 `PybulletSolver` 에서 sag 관련 코드 제거 + singleton
제거 (per-robot 인스턴스 가능하게).
"""

from __future__ import annotations

import logging
import threading
from pathlib import Path
from typing import Sequence

import numpy as np
import pybullet as p

from core.coords.link_coordinates import LinkCoordinates
from core.coords.urdf_patcher import write_patched_urdf
from modules.kinematics.iksolver import (
    Position3,
    Quaternion,
    RotMatrix3x3,
)

logger = logging.getLogger(__name__)

IK_MAX_ITER = 100
IK_TOLERANCE = 1e-4
IK_POS_ERROR_LIMIT = 0.01

# Project-wide URDF 컨벤션: 모든 robot type 의 URDF 는 TCP 를 가리키는 `tcp` 라는
# 이름의 link 를 가져야 함 (UR `tool0` 와 같은 패턴 — link 이름이 그 robot type
# 의 운동학 SSOT 인 URDF 자체에서 표준화됨). 새 robot type 추가 시 URDF 에
# `<link name="tcp"/>` 를 fixed joint child 로 박아두면 frontend / backend
# 양쪽이 추가 config 없이 동작. 부팅 시 PyBullet 가 못 찾으면 즉시 fail-fast.
EE_LINK_NAME = "tcp"


class PybulletIKSolver:
    """PyBullet 기반 IKSolver. ideal URDF 기구학 only (sag 없음)."""

    def __init__(self, urdf_path: str | Path):
        """patched URDF 생성 후 PyBullet DIRECT 모드로 로드.

        link_offsets.npz 가 있으면 URDF joint origin patch 적용된 사본을 사용
        (`.patched/` 디렉토리). 없으면 mesh 경로만 절대화한 사본.
        """
        self._sim_lock = threading.Lock()

        link_offsets = LinkCoordinates().snapshot()
        urdf_to_load = write_patched_urdf(urdf_path, link_offsets)
        if not link_offsets.is_empty():
            logger.info(f"patched URDF 로드: {urdf_to_load}")

        self._client = p.connect(p.DIRECT)
        p.setGravity(0, 0, -9.81, physicsClientId=self._client)

        self._robot = p.loadURDF(
            str(urdf_to_load),
            useFixedBase=True,
            physicsClientId=self._client,
        )

        self._joint_indices: list[int] = []
        self._ee_index: int = -1
        self._lower_limits: list[float] = []
        self._upper_limits: list[float] = []
        self._joint_ranges: list[float] = []

        num_joints = p.getNumJoints(self._robot, physicsClientId=self._client)
        for i in range(num_joints):
            info = p.getJointInfo(self._robot, i, physicsClientId=self._client)
            joint_type = info[2]
            link_name: str = info[12].decode()
            if joint_type == p.JOINT_REVOLUTE:
                self._joint_indices.append(i)
                lower = info[8]
                upper = info[9]
                if lower >= upper:
                    lower, upper = -6.2832, 6.2832
                self._lower_limits.append(float(lower))
                self._upper_limits.append(float(upper))
                self._joint_ranges.append(float(upper - lower))
            if link_name == EE_LINK_NAME:
                self._ee_index = i

        if self._ee_index == -1:
            raise RuntimeError(f"{EE_LINK_NAME} not found in URDF")

    # ─── IKSolver Protocol ─────────────────────────────────────

    @property
    def dof(self) -> int:
        return len(self._joint_indices)

    @property
    def ee_link_name(self) -> str:
        return EE_LINK_NAME

    def fk(
        self, joint_angles: Sequence[float]
    ) -> tuple[Position3, Quaternion]:
        with self._sim_lock:
            self._set_joint_positions(list(joint_angles))
            return self._get_ee_state()

    def ik(
        self,
        target_position: Position3,
        target_quaternion: Quaternion | None,
        current_joint_angles: Sequence[float] | None = None,
    ) -> list[float] | None:
        with self._sim_lock:
            n = len(self._joint_indices)
            rest = (
                list(current_joint_angles) if current_joint_angles else [0.0] * n
            )
            if current_joint_angles:
                self._set_joint_positions(list(current_joint_angles))

            kwargs: dict = dict(
                bodyUniqueId=self._robot,
                endEffectorLinkIndex=self._ee_index,
                targetPosition=target_position,
                lowerLimits=self._lower_limits,
                upperLimits=self._upper_limits,
                jointRanges=self._joint_ranges,
                restPoses=rest,
                maxNumIterations=IK_MAX_ITER,
                residualThreshold=IK_TOLERANCE,
                physicsClientId=self._client,
            )
            if target_quaternion is not None:
                kwargs["targetOrientation"] = target_quaternion

            result = p.calculateInverseKinematics(**kwargs)
            angles = list(result[:n])

            # 수렴 검증
            self._set_joint_positions(angles)
            actual_pos, _ = self._get_ee_state()
            error = float(
                np.linalg.norm(np.array(actual_pos) - np.array(target_position))
            )
            if error > IK_POS_ERROR_LIMIT:
                return None

            return angles

    def fk_to_matrix(
        self, joint_angles: Sequence[float]
    ) -> tuple[RotMatrix3x3, Position3]:
        position, quaternion = self.fk(joint_angles)
        m = p.getMatrixFromQuaternion(quaternion, physicsClientId=self._client)
        rot: RotMatrix3x3 = [
            [m[0], m[1], m[2]],
            [m[3], m[4], m[5]],
            [m[6], m[7], m[8]],
        ]
        return rot, position

    def joint_limits(
        self, n: int | None = None
    ) -> list[tuple[float, float]]:
        pairs = list(zip(self._lower_limits, self._upper_limits))
        return pairs[:n] if n is not None else pairs

    def self_collision(self, joint_angles: Sequence[float]) -> bool:
        # PyBullet 의 self-collision 검사 — 현재 미구현. 미래.
        # 단순 stub: 항상 False (collision 없음 가정).
        return False

    # ─── 내부 ──────────────────────────────────────────────────

    def _set_joint_positions(self, joint_angles: list[float]) -> None:
        for idx, angle in zip(self._joint_indices, joint_angles):
            p.resetJointState(
                self._robot, idx, angle, physicsClientId=self._client
            )

    def _get_ee_state(self) -> tuple[Position3, Quaternion]:
        state = p.getLinkState(
            self._robot,
            self._ee_index,
            computeForwardKinematics=True,
            physicsClientId=self._client,
        )
        return tuple(state[4]), tuple(state[5])

    def close(self) -> None:
        if p.isConnected(self._client):
            p.disconnect(self._client)
