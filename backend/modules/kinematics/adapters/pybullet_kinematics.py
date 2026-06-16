"""PyBullet 기반 Kinematics adapter — ideal URDF 기구학 only.

multi_robot_architecture.md §3.1 참조. URDF patch 메커니즘 = storage_layer.md §13.

책임:
- URDF 로드 (link_offsets in-memory patch → tempfile 1회성 → loadURDF → unlink)
- fk / ik / fk_to_matrix / joint_limits / self_collision
- thread-safe (`_sim_lock`)

책임 외 (외부 layer 가 적용):
- sag 보정 → `modules.kinematics.adapters.sag_corrected.SagCorrectedKinematics`
- joint_offset → `core.coords.joint_coordinates.JointCoordinates` (raw↔rad 변환 시)
"""

from __future__ import annotations

import logging
import os
import tempfile
import threading
from pathlib import Path
from typing import Sequence

import numpy as np
import pybullet as p

from core.coords.urdf_patcher import patch_urdf_text
from modules.calibration.link_offsets import LinkOffsets
from modules.kinematics.kinematics import (
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
TCP_LINK_NAME = "tcp"


class PybulletKinematics:
    """PyBullet 기반 Kinematics. ideal URDF 기구학 only (sag 없음)."""

    def __init__(self, urdf_path: str | Path):
        """생성 즉시 끝남 — URDF load 안 함. calibration_node 가 `apply_link_offsets`
        + `initialize` 호출 (docs/storage_layer.md §7).

        부팅 시 storage 의존 차단 — Kinematics 는 calibration 도메인 모름.
        """
        self._urdf_path = Path(urdf_path)
        self._sim_lock = threading.Lock()
        self._initialized = False
        self._link_offsets: LinkOffsets = LinkOffsets()

        self._client: int = -1
        self._robot: int = -1
        self._joint_indices: list[int] = []
        self._ee_index: int = -1
        self._lower_limits: list[float] = []
        self._upper_limits: list[float] = []
        self._joint_ranges: list[float] = []

    def apply_link_offsets(self, offsets: LinkOffsets) -> None:
        """initialize() 호출 전에 link_offsets 주입. 이미 initialized 면 RuntimeError —
        URDF patch 가 부팅 시 1회만 (docs/storage_layer.md §7 원칙 4).
        """
        if self._initialized:
            raise RuntimeError(
                "PybulletKinematics 이미 initialize 완료 — link_offsets 재주입 X "
                "(런타임 reload 는 본 design 의 범위 밖)"
            )
        self._link_offsets = offsets

    def initialize(self) -> None:
        """URDF patch + PyBullet DIRECT 모드 로드 + joint info parse.

        calibration_node 가 apply_link_offsets 직후 호출. 두 번 호출 시 no-op.

        link_offsets 는 in-memory string patch (storage_layer.md §13). PyBullet
        `loadURDF` 가 path-only 라 OS temp 파일로 1회성 우회 — load 직후 unlink.
        mesh 는 patch_urdf_text 가 절대경로 rewrite 해두므로 unlink 후에도
        원본 mesh 파일에서 lazy load 가능.
        """
        with self._sim_lock:
            if self._initialized:
                return

            patched_text = patch_urdf_text(self._urdf_path, self._link_offsets)
            if not self._link_offsets.is_empty():
                logger.info("link_offsets 적용된 URDF in-memory render")

            self._client = p.connect(p.DIRECT)
            p.setGravity(0, 0, -9.81, physicsClientId=self._client)

            fd, temp_path = tempfile.mkstemp(suffix=".urdf", prefix="horibot_")
            try:
                with os.fdopen(fd, "w", encoding="utf-8") as f:
                    f.write(patched_text)
                self._robot = p.loadURDF(
                    temp_path,
                    useFixedBase=True,
                    physicsClientId=self._client,
                )
            finally:
                try:
                    os.unlink(temp_path)
                except OSError:
                    pass

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
                if link_name == TCP_LINK_NAME:
                    self._ee_index = i

            if self._ee_index == -1:
                raise RuntimeError(f"{TCP_LINK_NAME} not found in URDF")

            self._initialized = True

    # ─── Kinematics Protocol ────────────────────────────────────

    def _require_initialized(self) -> None:
        if not self._initialized:
            raise RuntimeError(
                "PybulletKinematics 미초기화 — calibration_node 가 부팅 시 "
                "apply_link_offsets + initialize 호출해야 함 "
                "(docs/storage_layer.md §7)"
            )

    @property
    def dof(self) -> int:
        self._require_initialized()
        return len(self._joint_indices)

    @property
    def tcp_link_name(self) -> str:
        return TCP_LINK_NAME

    def fk(
        self, joint_angles: Sequence[float]
    ) -> tuple[Position3, Quaternion]:
        self._require_initialized()
        with self._sim_lock:
            self._set_joint_positions(list(joint_angles))
            return self._get_ee_state()

    def ik(
        self,
        target_position: Position3,
        target_quaternion: Quaternion | None,
        current_joint_angles: Sequence[float] | None = None,
    ) -> list[float] | None:
        self._require_initialized()
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

    def tcp_twist_to_joint_vel(
        self,
        linear: Sequence[float],
        angular: Sequence[float],
        joint_angles: Sequence[float],
        frame: str,
    ) -> list[float] | None:
        """TCP twist → joint velocity (Jacobian pseudo-inverse).

        `frame`:
          - `"base"` — twist 가 base 좌표계 (world axes)
          - `"tcp"`  — twist 가 EE-local 좌표계, world 로 회전 후 J^+ 곱.

        dof < 6 (예: OMX-F 5DOF) 면 angular 무시 + linear-only 3xN J 만 사용 —
        wrist yaw 부재로 임의 orientation 추적 불가하므로 underdetermined 안 만듦.
        Singularity 자리 (J^TJ 가 ill-conditioned) → None 반환 → caller 가 안전 정지.
        """
        self._require_initialized()
        n = len(self._joint_indices)
        if len(joint_angles) < n:
            return None
        with self._sim_lock:
            self._set_joint_positions(list(joint_angles))
            zeros = [0.0] * n
            lin_J, ang_J = p.calculateJacobian(
                self._robot,
                self._ee_index,
                [0.0, 0.0, 0.0],
                list(joint_angles[:n]),
                zeros,
                zeros,
                physicsClientId=self._client,
            )
            ee_quat = self._get_ee_state()[1]
            ee_R = np.asarray(
                p.getMatrixFromQuaternion(
                    ee_quat, physicsClientId=self._client
                )
            ).reshape(3, 3)

        lin_J_np = np.asarray(lin_J)  # (3, n)
        ang_J_np = np.asarray(ang_J)  # (3, n)

        linear_arr = np.asarray(linear, dtype=np.float64)
        angular_arr = np.asarray(angular, dtype=np.float64)
        if frame == "tcp":
            linear_arr = ee_R @ linear_arr
            angular_arr = ee_R @ angular_arr

        if n < 6:
            J = lin_J_np
            twist = linear_arr
        else:
            J = np.vstack([lin_J_np, ang_J_np])
            twist = np.concatenate([linear_arr, angular_arr])

        try:
            joint_vel = np.linalg.pinv(J) @ twist
        except np.linalg.LinAlgError:
            return None
        if not np.all(np.isfinite(joint_vel)):
            return None
        return joint_vel.tolist()

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
