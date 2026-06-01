import logging
import threading
from typing import TypeAlias
import numpy as np
import pybullet as p

from core.link_coordinates import LinkCoordinates
from core.robot_registry import RobotRegistry
from core.sag_coordinates import SagCoordinates
from core.urdf_patcher import write_patched_urdf
from modules.kinematics.fk_chain import (
    actual_to_commanded,
    apply_gravity_sag,
)

logger = logging.getLogger(__name__)

# sag 모델은 J2, J3에만 적용 (motor id 2, 3). J1/J4/J5의 sag는 측정 noise
# 수준이라 모델 단순성 위해 제외.
_SAG_JOINT_IDS: list[int] = [2, 3]
_ARM_DOF: int = 5

# ─── 타입 별칭 ─────────────────────────────────────────────────
Position3: TypeAlias = tuple[float, float, float]  # [x, y, z] 미터
Quaternion: TypeAlias = tuple[float, float, float, float]  # [x, y, z, w]
RotMatrix3x3: TypeAlias = list[list[float]]  # 3x3 회전 행렬
IK_MAX_ITER = 100
IK_TOLERANCE = 1e-4
IK_POS_ERROR_LIMIT = 0.01


class PybulletSolver:
    _instance: "PybulletSolver | None" = None
    _lock = threading.Lock()

    def __new__(cls) -> "PybulletSolver":
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self) -> None:
        if self._initialized:
            return
        self._initialized = True
        self._sim_lock = threading.Lock()

        # link_offsets.npz가 있으면 patched URDF 생성 (없으면 mesh 절대경로화만).
        # patched URDF는 robot/urdf/omx_f/.patched/omx_f.urdf — gitignored.
        link_offsets = LinkCoordinates().snapshot()
        urdf_path = RobotRegistry().default().urdf_path
        urdf_to_load = write_patched_urdf(urdf_path, link_offsets)
        if not link_offsets.is_empty():
            logger.info(f"patched URDF 로드: {urdf_to_load}")

        # numpy fk_chain의 apply_gravity_sag에 전달할 link_offset (PyBullet의 patched URDF
        # 와 동일 값). 두 경로가 같은 ee 위치 → 같은 토크 계산 → 일관된 sag.
        self._link_trans_array = np.array(
            [link_offsets.get_trans(i + 1) for i in range(_ARM_DOF)], dtype=np.float64
        )
        self._link_rot_array = np.array(
            [link_offsets.get_rot(i + 1) for i in range(_ARM_DOF)], dtype=np.float64
        )

        # sag stiffness (rad/(m·g_unit)) — J2, J3만. SagCoordinates 비어있으면 0.
        # 0이면 sag 적용 없음 (legacy 동작과 동일).
        self._reload_sag_cache()

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
                # URDF limit이 없거나 역전된 경우 fallback
                if lower >= upper:
                    lower, upper = -6.2832, 6.2832
                self._lower_limits.append(float(lower))
                self._upper_limits.append(float(upper))
                self._joint_ranges.append(float(upper - lower))
            if link_name == "end_effector_link":
                self._ee_index = i

        if self._ee_index == -1:
            raise RuntimeError("end_effector_link not found in URDF")

    # ─── 내부 유틸 ──────────────────────────────────────────────

    def _set_joint_positions(self, joint_angles: list[float]) -> None:
        for idx, angle in zip(self._joint_indices, joint_angles):
            p.resetJointState(self._robot, idx, angle,
                              physicsClientId=self._client)

    def _get_ee_state(self) -> tuple[Position3, Quaternion]:
        state = p.getLinkState(
            self._robot,
            self._ee_index,
            computeForwardKinematics=True,
            physicsClientId=self._client,
        )
        return tuple(state[4]), tuple(state[5])

    def _reload_sag_cache(self) -> None:
        """SagCoordinates에서 k 배열 다시 로드. COMMIT 후 호출하면 재시작 없이 반영."""
        sag = SagCoordinates().snapshot()
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
            logger.info(f"PybulletSolver sag 적용: {ks}")

    def _commanded_to_actual(self, joint_angles: list[float]) -> list[float]:
        """모터 encoder reading(commanded) → 실제 link end의 URDF angle (actual)."""
        if not self._sag_enabled or len(joint_angles) < _ARM_DOF:
            return list(joint_angles)
        arm = np.asarray(joint_angles[:_ARM_DOF], dtype=np.float64)
        actual = apply_gravity_sag(
            arm, self._sag_k_array, self._link_trans_array, self._link_rot_array
        )
        return list(actual) + list(joint_angles[_ARM_DOF:])

    def _actual_to_commanded(self, joint_angles: list[float]) -> list[float]:
        """IK 결과(actual, URDF static FK target) → 모터 명령 commanded. 1차 근사."""
        if not self._sag_enabled or len(joint_angles) < _ARM_DOF:
            return list(joint_angles)
        arm = np.asarray(joint_angles[:_ARM_DOF], dtype=np.float64)
        commanded = actual_to_commanded(
            arm, self._sag_k_array, self._link_trans_array, self._link_rot_array
        )
        return list(commanded) + list(joint_angles[_ARM_DOF:])

    # ─── Public API ────────────────────────────────────────────

    def fk(self, joint_angles: list[float]) -> tuple[Position3, Quaternion]:
        """encoder reading(commanded) → 실제 ee 자세 (sag 반영).

        Sag 비활성(SagCoordinates 비어있음)이면 기존 동작과 동일.
        """
        actual = self._commanded_to_actual(joint_angles)
        with self._sim_lock:
            self._set_joint_positions(actual)
            return self._get_ee_state()

    def ik(
        self,
        target_position: Position3,
        target_quaternion: Quaternion | None,
        current_joint_angles: list[float] | None = None,
    ) -> list[float] | None:
        """target ee pose → motor 명령 (commanded, sag 역보정 적용).

        흐름:
          1. current(commanded)를 actual로 변환 → PyBullet IK의 seed
          2. PyBullet IK로 actual_result 계산 (URDF static fk가 target에 도달하는 angle)
          3. 수렴 검증 (actual_result로)
          4. actual_to_commanded로 motor 명령 변환해 return

        Sag 비활성이면 변환은 no-op (기존 동작과 동일).
        """
        # current_joint_angles는 encoder reading(commanded). IK seed로 쓸 때 actual 변환.
        current_actual = (
            self._commanded_to_actual(current_joint_angles)
            if current_joint_angles
            else None
        )

        with self._sim_lock:
            n = len(self._joint_indices)

            # restPoses: actual 기준으로 가장 가까운 해 선호
            # 없으면 0으로 초기화 (홈 포지션 근처에서만 시작할 때는 무방)
            rest = list(current_actual) if current_actual else [0.0] * n

            if current_actual:
                self._set_joint_positions(current_actual)

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
            actual_angles = list(result[:n])

            # 수렴 검증 — PyBullet의 fk가 target에 도달하는지 (actual angle 기준)
            self._set_joint_positions(actual_angles)
            actual_pos, _ = self._get_ee_state()
            error = float(
                np.linalg.norm(np.array(actual_pos) -
                               np.array(target_position))
            )
            if error > IK_POS_ERROR_LIMIT:
                return None

        # sag 역보정 → motor 명령으로 변환. IK는 sim_lock 안에서 끝났고
        # _actual_to_commanded는 numpy fk_chain만 호출하므로 lock 밖에서 OK.
        return self._actual_to_commanded(actual_angles)

    def joint_limits(self, n: int | None = None) -> list[tuple[float, float]]:
        """URDF 조인트 limit (lower, upper) — rad. n 지정 시 처음 n개만 (arm).

        next_pose_planner / coach가 모터 limit 안에서 추천 각도 계산할 때 사용.
        """
        pairs = list(zip(self._lower_limits, self._upper_limits))
        return pairs[:n] if n is not None else pairs

    def fk_to_matrix(self, joint_angles: list[float]) -> tuple[RotMatrix3x3, Position3]:
        position, quaternion = self.fk(joint_angles)
        m = p.getMatrixFromQuaternion(quaternion, physicsClientId=self._client)
        R: RotMatrix3x3 = [
            [m[0], m[1], m[2]],
            [m[3], m[4], m[5]],
            [m[6], m[7], m[8]],
        ]
        return R, position

    def close(self) -> None:
        if p.isConnected(self._client):
            p.disconnect(self._client)
        PybulletSolver._instance = None
        self._initialized = False
