"""URDF chain 의 numpy FK — link_offset variable 박을 수 있는 자리.

PyBullet 은 `loadURDF` 후 link transform 정적 (`changeDynamics` 가 mass/inertia
만 변경, joint origin xyz/rpy 못 건드림). Hand-Eye BA 는 link origin offset 을
*변수* 로 풀어야 해서 매 LM iteration 마다 다른 link_offset 으로 FK 호출 필요
— PyBullet 우회.

URDF parse 는 `yourdfpy` 위임 (rpy 곱 / mimic / fixed joint 다 처리),
chain build 는 자체 numpy (BA hot path 성능 + link_offset variable 통제).

사용처:
    (a) `bundle_adjust.py` 의 확장 BA — link offset 자유도와 함께 FK 평가
    (b) `sag_corrected.py` 의 sag 보정 — apply_gravity_sag / actual_to_commanded

OMX-F (5DOF) / SO-101 (6DOF) robot type 무관 — `RobotConfig.urdf_path` +
`MotorLayout.arm()` 의 joint name list 받으면 chain 자동 build.
"""

from __future__ import annotations

import logging
from pathlib import Path

import numpy as np
from yourdfpy import URDF

logger = logging.getLogger(__name__)


# 중력 방향 — base frame 의 -z. SI 단위 g·m 로 normalize (k 가 rad/(m·g_unit) 흡수).
_GRAVITY_DIR = np.array([0.0, 0.0, -1.0], dtype=np.float64)


def axis_angle_to_R(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues — axis 는 정규화 안 돼있어도 OK."""
    a = np.asarray(axis, dtype=np.float64)
    norm = float(np.linalg.norm(a))
    if norm < 1e-12:
        return np.eye(3)
    a = a / norm
    c = np.cos(angle)
    s = np.sin(angle)
    K = np.array(
        [[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]], dtype=np.float64
    )
    return np.eye(3) * c + s * K + (1 - c) * np.outer(a, a)


def rotvec_to_R(rotvec: np.ndarray) -> np.ndarray:
    """rotation vector(= axis * angle) → 3x3. 작은 각(<5°)에서 정확."""
    rv = np.asarray(rotvec, dtype=np.float64)
    angle = float(np.linalg.norm(rv))
    if angle < 1e-12:
        return np.eye(3)
    return axis_angle_to_R(rv, angle)


def gravity_torque_lumped(
    ee_pos_base: np.ndarray,
    joint_origin_base: np.ndarray,
    joint_axis_base: np.ndarray,
) -> float:
    """ee 에 lumped mass 가정. joint 회전축에 작용하는 중력 토크 (sign + magnitude).

    τ = (r × g_dir) · axis   where r = ee - joint_origin   (모멘트 암 벡터)

    Units: r 은 m, g_dir/axis 는 unit → τ 는 m. k(=1/effective_stiffness) 곱하면 rad.

    URDF distributed mass 대신 lumped 가정 이유:
        - URDF mass 가 D405 카메라 교체 무게를 반영 못 함 (44g vs D405 42g)
        - PyBullet calculateInverseDynamics 보다 σ 에서 우월 (0.65° vs 0.77°)
        - k 가 effective (stiffness × mass) 비율을 통째로 흡수해 mass 부정확성에 robust
    """
    r = np.asarray(ee_pos_base) - np.asarray(joint_origin_base)
    return float(np.dot(np.cross(r, _GRAVITY_DIR), np.asarray(joint_axis_base)))


class FkChain:
    """URDF parse → arm chain numpy FK. robot type 무관.

    chain = `base_link` → ... → `tcp_link_name` 까지의 joint sequence. arm joint
    (revolute) 는 angle 적용, 중간 fixed joint 는 origin transform 만 흡수.
    `link_offset` 은 arm joint 의 origin xyz / rotation 에 variable 로 박힘.
    """

    def __init__(
        self,
        urdf_path: Path,
        arm_joint_names: list[str],
        tcp_link_name: str = "tcp",
    ):
        """Args:
            urdf_path: URDF 파일 경로
            arm_joint_names: actuated arm joint names, motor id 순서. gripper 제외.
                `MotorLayout.arm()` 의 `.name` 으로 추출 (motors.yaml SSOT).
            tcp_link_name: chain 끝 link 이름. CLAUDE.md 의 project convention 으로 `tcp`.
        """
        urdf = URDF.load(str(urdf_path), load_meshes=False)
        self._build_chain(urdf, tcp_link_name, arm_joint_names)
        self.n_arm = len(arm_joint_names)

    def _build_chain(
        self, urdf: URDF, tcp_link_name: str, arm_joint_names: list[str]
    ) -> None:
        # tcp 부터 base 까지 parent traverse (backward).
        if tcp_link_name not in urdf.link_map:
            raise ValueError(
                f"URDF 에 link '{tcp_link_name}' 없음. "
                f"CLAUDE.md project convention: 모든 robot type 의 URDF 는 'tcp' link 보유."
            )

        # link → parent_joint name 빠른 lookup
        child_to_joint: dict[str, str] = {}
        for jn, j in urdf.joint_map.items():
            child_to_joint[j.child] = jn

        chain_joints: list[str] = []  # tcp → base 방향 (reverse 전)
        cur = tcp_link_name
        while cur != urdf.base_link:
            if cur not in child_to_joint:
                raise ValueError(
                    f"chain build 실패 — link '{cur}' 가 root '{urdf.base_link}' "
                    f"로 연결 안 됨"
                )
            jn = child_to_joint[cur]
            chain_joints.append(jn)
            cur = urdf.joint_map[jn].parent
        chain_joints.reverse()  # base → tcp 순서

        # arm_joint_names 의 chain 안 index 매핑
        actuated_set = set(arm_joint_names)
        joint_to_arm_idx = {jn: i for i, jn in enumerate(arm_joint_names)}

        self._origins: list[np.ndarray] = []  # (4,4) per chain joint
        self._axes: list[np.ndarray] = []  # (3,) — revolute 만 valid, fixed 는 zeros
        self._types: list[str] = []  # 'revolute' or 'fixed'
        # chain pos → arm_joint_names 안 index (fixed joint 는 -1)
        self._arm_idx_in_angles: list[int] = []

        n_revolute = 0
        for jn in chain_joints:
            j = urdf.joint_map[jn]
            self._origins.append(np.asarray(j.origin, dtype=np.float64))
            if j.type == "revolute":
                self._axes.append(np.asarray(j.axis, dtype=np.float64))
                if jn not in actuated_set:
                    raise ValueError(
                        f"chain joint '{jn}' (revolute) 가 arm_joint_names 에 없음. "
                        f"motors.yaml 의 arm motor name 확인 — kind: gripper 제외 모두 포함 필요."
                    )
                self._arm_idx_in_angles.append(joint_to_arm_idx[jn])
                n_revolute += 1
            elif j.type == "fixed":
                self._axes.append(np.zeros(3, dtype=np.float64))
                self._arm_idx_in_angles.append(-1)
            else:
                raise ValueError(
                    f"joint '{jn}' type={j.type!r} unsupported. revolute / fixed 만 지원."
                )
            self._types.append(j.type)

        if n_revolute != len(arm_joint_names):
            raise ValueError(
                f"arm_joint_names 길이 ({len(arm_joint_names)}) ≠ "
                f"chain 안 revolute joint 수 ({n_revolute}). "
                f"motors.yaml 의 arm motor name 이 URDF chain 의 revolute joint 와 일치하는지 확인."
            )

        self._chain_joints = chain_joints
        logger.info(
            "FkChain: chain joints (base→tcp): %s — revolute %d, fixed %d",
            chain_joints,
            n_revolute,
            len(chain_joints) - n_revolute,
        )

    # ─── FK ──────────────────────────────────────────────────────

    def fk(
        self,
        joint_angles: np.ndarray,
        link_trans: np.ndarray | None = None,
        link_rot: np.ndarray | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        """arm joint angles → tcp 의 (R, t) base frame.

        Args:
            joint_angles: (n_arm,) rad
            link_trans: (n_arm, 3) or None — arm joint i origin xyz 에 더할 dx,dy,dz (m)
            link_rot: (n_arm, 3) or None — arm joint i origin frame 에 적용할 rotvec (rad)

        Returns:
            (R, t) — tcp link 의 base frame 자세. R is 3x3, t is (3,).
        """
        T = self._chain_transform(joint_angles, link_trans, link_rot)
        return T[:3, :3].copy(), T[:3, 3].copy()

    def fk_with_axes(
        self,
        joint_angles: np.ndarray,
        link_trans: np.ndarray | None = None,
        link_rot: np.ndarray | None = None,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """fk + 각 arm joint 의 origin/axis (base frame). 중력 토크 계산용.

        joint i 의 origin 은 *그 joint 의 회전 적용 전* base 좌표 (회전축 위치).
        axis 는 base 에서 본 단위 회전축 방향.

        Returns:
            (R_ee, t_ee, joint_origins_base, joint_axes_base)
            joint_origins_base: (n_arm, 3) m
            joint_axes_base: (n_arm, 3) unit vectors
        """
        ja, lt, lr = self._normalize_inputs(joint_angles, link_trans, link_rot)
        T = np.eye(4)
        joint_origins_base = np.zeros((self.n_arm, 3), dtype=np.float64)
        joint_axes_base = np.zeros((self.n_arm, 3), dtype=np.float64)
        for pos in range(len(self._chain_joints)):
            T_o = self._joint_origin_with_offset(pos, lt, lr)
            T = T @ T_o
            arm_idx = self._arm_idx_in_angles[pos]
            if arm_idx >= 0:
                # arm joint — 회전 적용 전 base frame origin / axis 캡처
                joint_origins_base[arm_idx] = T[:3, 3]
                joint_axes_base[arm_idx] = T[:3, :3] @ self._axes[pos]
                T_r = np.eye(4)
                T_r[:3, :3] = axis_angle_to_R(self._axes[pos], float(ja[arm_idx]))
                T = T @ T_r
            # fixed joint 는 회전 0 (이미 T_o 에 absorb)
        return (
            T[:3, :3].copy(),
            T[:3, 3].copy(),
            joint_origins_base,
            joint_axes_base,
        )

    def _chain_transform(
        self,
        joint_angles: np.ndarray,
        link_trans: np.ndarray | None,
        link_rot: np.ndarray | None,
    ) -> np.ndarray:
        ja, lt, lr = self._normalize_inputs(joint_angles, link_trans, link_rot)
        T = np.eye(4)
        for pos in range(len(self._chain_joints)):
            T_o = self._joint_origin_with_offset(pos, lt, lr)
            T = T @ T_o
            arm_idx = self._arm_idx_in_angles[pos]
            if arm_idx >= 0:
                T_r = np.eye(4)
                T_r[:3, :3] = axis_angle_to_R(self._axes[pos], float(ja[arm_idx]))
                T = T @ T_r
        return T

    def _normalize_inputs(
        self,
        joint_angles: np.ndarray,
        link_trans: np.ndarray | None,
        link_rot: np.ndarray | None,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        ja = np.asarray(joint_angles, dtype=np.float64)
        if ja.shape[0] != self.n_arm:
            raise ValueError(
                f"joint_angles 길이 {ja.shape[0]} ≠ n_arm {self.n_arm}"
            )
        lt = (
            np.zeros((self.n_arm, 3), dtype=np.float64)
            if link_trans is None
            else np.asarray(link_trans, dtype=np.float64)
        )
        lr = (
            np.zeros((self.n_arm, 3), dtype=np.float64)
            if link_rot is None
            else np.asarray(link_rot, dtype=np.float64)
        )
        return ja, lt, lr

    def _joint_origin_with_offset(
        self, pos: int, link_trans: np.ndarray, link_rot: np.ndarray
    ) -> np.ndarray:
        """chain pos 의 joint origin transform — arm joint 면 link_offset 가산."""
        T_o = self._origins[pos].copy()
        arm_idx = self._arm_idx_in_angles[pos]
        if arm_idx >= 0:
            T_o[:3, 3] = T_o[:3, 3] + link_trans[arm_idx]
            if np.any(link_rot[arm_idx]):
                T_o[:3, :3] = T_o[:3, :3] @ rotvec_to_R(link_rot[arm_idx])
        return T_o

    # ─── Sag 보정 ────────────────────────────────────────────────

    def apply_gravity_sag(
        self,
        joint_angles: np.ndarray,
        k_stiff: np.ndarray,
        sag_joint_indices: list[int],
        link_trans: np.ndarray | None = None,
        link_rot: np.ndarray | None = None,
    ) -> np.ndarray:
        """commanded joint angles → sag 적용된 actual angles.

        모델: actual = commanded + sag_offset(commanded)
            sag_offset_J = k_J * τ_J   where τ_J = gravity_torque_lumped(ee, joint_J)

        Args:
            joint_angles: (n_arm,) commanded (rad)
            k_stiff: (len(sag_joint_indices),) — 각 sag joint 의 stiffness
            sag_joint_indices: 0-indexed arm joint indices (sag 적용 자리).
                OMX-F: [1, 2] (J2 shoulder, J3 elbow — DIY 5축 중력 부하 가장 큰 두 joint)
                so101: 캘 도착 후 결정. k_stiff 가 [0, 0] 이면 본 함수 no-op.

        Returns:
            (n_arm,) sag 적용 angles.
        """
        k = np.asarray(k_stiff, dtype=np.float64)
        if k.size == 0 or float(np.max(np.abs(k))) < 1e-12:
            return np.asarray(joint_angles, dtype=np.float64).copy()
        if len(k) != len(sag_joint_indices):
            raise ValueError(
                f"k_stiff 길이 {len(k)} ≠ sag_joint_indices 길이 {len(sag_joint_indices)}"
            )
        _, ee_pos, joint_origins, joint_axes = self.fk_with_axes(
            joint_angles, link_trans, link_rot
        )
        out = np.asarray(joint_angles, dtype=np.float64).copy()
        for slot, arm_idx in enumerate(sag_joint_indices):
            tau = gravity_torque_lumped(
                ee_pos, joint_origins[arm_idx], joint_axes[arm_idx]
            )
            out[arm_idx] += k[slot] * tau
        return out

    def actual_to_commanded(
        self,
        actual_angles: np.ndarray,
        k_stiff: np.ndarray,
        sag_joint_indices: list[int],
        link_trans: np.ndarray | None = None,
        link_rot: np.ndarray | None = None,
    ) -> np.ndarray:
        """apply_gravity_sag 의 역방향 (IK 결과 → 모터 명령). 1차 근사.

        모델: actual = commanded + sag_offset(commanded)
        Implicit: commanded = actual - sag_offset(commanded)
        1st-order: commanded ≈ actual - sag_offset(actual)

        sag ~2° 수준에서 1차 근사 오차 < 0.05° (Taylor 잔차). fixed-point 1 iter
        면 완전 수렴하지만 perf vs 정확도 트레이드오프상 1차로 충분.
        """
        a = np.asarray(actual_angles, dtype=np.float64)
        sag_at_actual = (
            self.apply_gravity_sag(
                a, k_stiff, sag_joint_indices, link_trans, link_rot
            )
            - a
        )
        return a - sag_at_actual
