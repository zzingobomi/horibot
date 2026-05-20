import threading
from pathlib import Path
from typing import TypeAlias
import numpy as np
import pybullet as p

# ─── 타입 별칭 ─────────────────────────────────────────────────
Position3: TypeAlias = tuple[float, float, float]  # [x, y, z] 미터
Quaternion: TypeAlias = tuple[float, float, float, float]  # [x, y, z, w]
RotMatrix3x3: TypeAlias = list[list[float]]  # 3x3 회전 행렬

# ─── 상수 ──────────────────────────────────────────────────────
URDF_PATH = Path(__file__).parents[3] / \
    "robot" / "urdf" / "omx_f" / "omx_f.urdf"
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

        self._client = p.connect(p.DIRECT)
        p.setGravity(0, 0, -9.81, physicsClientId=self._client)

        self._robot = p.loadURDF(
            str(URDF_PATH),
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

    # ─── Public API ────────────────────────────────────────────

    def fk(self, joint_angles: list[float]) -> tuple[Position3, Quaternion]:
        with self._sim_lock:
            self._set_joint_positions(joint_angles)
            return self._get_ee_state()

    def ik(
        self,
        target_position: Position3,
        target_quaternion: Quaternion | None,
        current_joint_angles: list[float] | None = None,
    ) -> list[float] | None:
        with self._sim_lock:
            n = len(self._joint_indices)

            # restPoses: current_joint_angles 기준으로 가장 가까운 해 선호
            # 없으면 0으로 초기화 (홈 포지션 근처에서만 시작할 때는 무방)
            rest = list(current_joint_angles) if current_joint_angles else [
                0.0] * n

            if current_joint_angles:
                self._set_joint_positions(current_joint_angles)

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
                np.linalg.norm(np.array(actual_pos) -
                               np.array(target_position))
            )
            if error > IK_POS_ERROR_LIMIT:
                return None

            return angles

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
