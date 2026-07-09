"""PybulletKinematics — PyBullet 기반 Kinematics (ideal URDF, sag 없음).

옛 backend/modules/kinematics/adapters/pybullet_kinematics.py port.
D1 = plain URDF load (link_offset patch 는 D4 Mirror[Bundle] 자리).

dof = tcp link 의 **ancestor revolute joint** 만 (gripper 등 sibling 가지 제외).
PyBullet 의 jointIndex == childLinkIndex 를 이용해 tcp 에서 base 로 거슬러 올라가며
chain 식별 → so101_6dof=6 (옛 코드의 "전체 revolute=7" 오포함 정정).
"""

from __future__ import annotations

import logging
import threading
from pathlib import Path
from typing import Sequence

import numpy as np
import pybullet as p

from ..kinematics import Position3, Quaternion, RotMatrix3x3

logger = logging.getLogger(__name__)

IK_MAX_ITER = 100
IK_TOLERANCE = 1e-4
IK_POS_ERROR_LIMIT = 0.01
# seed 1회로 수렴 못 하면 random restart (PyBullet 은 seed 에서 출발하는 local
# 솔버라 해가 존재해도 놓침 — "도달 가능한데 IK 실패" 방지). restart 중 seed 에
# 가장 가까운 해 선택 → motion 연속성 유지.
# 200 근거 (2026-07-09 SO-101 손 시연 자세 실측): orientation 붙은 top-down 파지
# 자세는 basin 이 좁아 균등 재시작 성공까지 median 8 / max 40회 — 옛 24 는 복권
# (같은 자세가 rng 이력 따라 됐다 안 됐다). 800회 ≈ 0.1s (DIRECT) 라 200 은 공짜.
IK_RESTARTS = 200

# 모든 robot type URDF 는 `tcp` 라는 link 를 가져야 함 (UR tool0 패턴, fail-fast).
TCP_LINK_NAME = "tcp"


class PybulletKinematics:
    """PyBullet DIRECT 모드 기구학. Kinematics Protocol 만족."""

    def __init__(self, urdf_path: str | Path) -> None:
        self._urdf_path = Path(urdf_path)
        self._lock = threading.Lock()
        self._initialized = False
        self._client = -1
        self._robot = -1
        self._ee_index = -1
        # chain = tcp ancestor revolute joints (base→tcp 순), = fk/ik 인터페이스 joints
        self._chain_indices: list[int] = []
        self._chain_lower: list[float] = []
        self._chain_upper: list[float] = []
        # 전체 movable revolute (gripper 포함) — IK solver 가 다 받으므로 필요
        self._movable_indices: list[int] = []
        self._movable_lower: list[float] = []
        self._movable_upper: list[float] = []
        self._movable_ranges: list[float] = []
        # chain joint 의 movable result vector 내 위치
        self._chain_in_movable: list[int] = []

    def initialize(self) -> None:
        with self._lock:
            if self._initialized:
                return
            self._client = p.connect(p.DIRECT)
            p.setGravity(0, 0, -9.81, physicsClientId=self._client)
            self._robot = p.loadURDF(
                str(self._urdf_path),
                useFixedBase=True,
                flags=(
                    p.URDF_USE_SELF_COLLISION
                    | p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT
                ),
                physicsClientId=self._client,
            )

            num = p.getNumJoints(self._robot, physicsClientId=self._client)
            movable: list[tuple[int, float, float]] = []
            link_names: dict[int, str] = {-1: "base"}
            for i in range(num):
                info = p.getJointInfo(self._robot, i, physicsClientId=self._client)
                link_names[i] = info[12].decode()
                if info[2] == p.JOINT_REVOLUTE:
                    lower, upper = float(info[8]), float(info[9])
                    if lower >= upper:
                        lower, upper = -6.2832, 6.2832
                    movable.append((i, lower, upper))
                if info[12].decode() == TCP_LINK_NAME:
                    self._ee_index = i
            if self._ee_index == -1:
                raise RuntimeError(f"{TCP_LINK_NAME} link not found in URDF")

            self._exclude_zero_pose_collisions(link_names)

            self._movable_indices = [m[0] for m in movable]
            self._movable_lower = [m[1] for m in movable]
            self._movable_upper = [m[2] for m in movable]
            self._movable_ranges = [u - lo for _, lo, u in movable]
            limit_by_idx = {m[0]: (m[1], m[2]) for m in movable}

            # tcp → base ancestor walk (jointIndex == childLinkIndex)
            chain: list[int] = []
            link = self._ee_index
            while link != -1:
                info = p.getJointInfo(self._robot, link, physicsClientId=self._client)
                if info[2] == p.JOINT_REVOLUTE:
                    chain.append(link)
                link = info[16]  # parentIndex
            chain.reverse()  # base→tcp
            self._chain_indices = chain
            self._chain_lower = [limit_by_idx[j][0] for j in chain]
            self._chain_upper = [limit_by_idx[j][1] for j in chain]
            self._chain_in_movable = [self._movable_indices.index(j) for j in chain]

            self._initialized = True
            logger.info(
                "PybulletKinematics: dof=%d (chain) / %d movable, tcp=%s",
                len(chain), len(movable), TCP_LINK_NAME,
            )

    # ── Protocol ──

    @property
    def dof(self) -> int:
        self._require_init()
        return len(self._chain_indices)

    @property
    def tcp_link_name(self) -> str:
        return TCP_LINK_NAME

    def fk(self, joint_angles: Sequence[float]) -> tuple[Position3, Quaternion]:
        self._require_init()
        with self._lock:
            self._set_chain(list(joint_angles))
            return self._ee_state()

    def ik(
        self,
        target_position: Position3,
        target_quaternion: Quaternion | None,
        current_joint_angles: Sequence[float] | None = None,
        restarts: int | None = None,
    ) -> list[float] | None:
        self._require_init()
        with self._lock:
            seed = (
                list(current_joint_angles)
                if current_joint_angles is not None
                else [0.0] * len(self._chain_indices)
            )
            # 1) seeded 1회 — 현재 자세 근처 해 (motion 연속성, 대부분 여기서 끝).
            sol = self._ik_from_seed(target_position, target_quaternion, seed)
            if sol is not None:
                return sol
            # 2) 실패 = local 솔버가 seed basin 에서 못 찾은 것일 수 있음 (해는 존재
            #    가능). random restart 후 seed 에 가장 가까운 해 선택 (jump 최소화).
            #    rng 는 호출마다 fresh — 프로세스 전역 rng 는 호출 이력에 따라 같은
            #    요청이 됐다 안 됐다 하는 복권이 됨 (2026-07-09 PnP 후보 전멸 원인).
            #    restarts 지정 = 작은 예산 probe (deepening — 실패 기각 비용 절감).
            rng = np.random.default_rng(0)
            best: list[float] | None = None
            best_dist = float("inf")
            for _ in range(restarts if restarts is not None else IK_RESTARTS):
                rand = [
                    float(rng.uniform(lo, hi))
                    for lo, hi in zip(self._chain_lower, self._chain_upper)
                ]
                cand = self._ik_from_seed(target_position, target_quaternion, rand)
                if cand is not None:
                    dist = sum((a - b) ** 2 for a, b in zip(cand, seed))
                    if dist < best_dist:
                        best_dist, best = dist, cand
            if best is None:
                logger.debug(
                    "IK 실패 (seed + restart %d 모두) target=%s",
                    IK_RESTARTS, target_position,
                )
            return best

    def _ik_from_seed(
        self,
        target_position: Position3,
        target_quaternion: Quaternion | None,
        seed_chain: list[float],
    ) -> list[float] | None:
        """chain-공간 seed 하나로 1회 IK + 수렴/충돌 검증 (호출자가 _lock 보유).

        수렴 검증은 reachability vs self-collision 원인 분리 (debug 로그).
        """
        rest = [0.0] * len(self._movable_indices)
        for k, angle in enumerate(seed_chain):
            rest[self._chain_in_movable[k]] = angle
        self._set_chain(seed_chain)

        kwargs: dict = dict(
            bodyUniqueId=self._robot,
            endEffectorLinkIndex=self._ee_index,
            targetPosition=target_position,
            lowerLimits=self._movable_lower,
            upperLimits=self._movable_upper,
            jointRanges=self._movable_ranges,
            restPoses=rest,
            maxNumIterations=IK_MAX_ITER,
            residualThreshold=IK_TOLERANCE,
            physicsClientId=self._client,
        )
        if target_quaternion is not None:
            kwargs["targetOrientation"] = target_quaternion

        result = p.calculateInverseKinematics(**kwargs)
        angles = [result[i] for i in self._chain_in_movable]

        self._set_chain(angles)
        actual_pos, _ = self._ee_state()
        error = float(
            np.linalg.norm(np.array(actual_pos) - np.array(target_position))
        )
        if error > IK_POS_ERROR_LIMIT:
            return None
        if self._self_collision_unlocked():
            return None
        return angles

    def fk_to_matrix(
        self, joint_angles: Sequence[float]
    ) -> tuple[RotMatrix3x3, Position3]:
        position, quat = self.fk(joint_angles)
        m = p.getMatrixFromQuaternion(quat, physicsClientId=self._client)
        rot: RotMatrix3x3 = [
            [m[0], m[1], m[2]],
            [m[3], m[4], m[5]],
            [m[6], m[7], m[8]],
        ]
        return rot, position

    def joint_limits(self, n: int | None = None) -> list[tuple[float, float]]:
        pairs = list(zip(self._chain_lower, self._chain_upper))
        return pairs[:n] if n is not None else pairs

    def self_collision(self, joint_angles: Sequence[float]) -> bool:
        self._require_init()
        with self._lock:
            self._set_chain(list(joint_angles))
            return self._self_collision_unlocked()

    def close(self) -> None:
        if p.isConnected(self._client):
            p.disconnect(self._client)

    # ── 내부 ──

    def _exclude_zero_pose_collisions(self, link_names: dict[int, str]) -> None:
        """URDF zero pose 에서 이미 침투한 link 쌍을 collision filter 에서 제외.

        MoveIt SRDF generator 의 "default collision matrix" 표준 패턴 — zero(=home)
        자세에서 침투한 쌍은 mesh 모델링 artifact 다 (실 로봇은 home 에 물리적으로
        존재하므로). 그대로 두면 그 쌍이 _모든_ 자세에서 self-collision 판정을
        오염시켜 IK 전부 reject (실례: OMX gripper 손가락 link6↔link7 -3.8mm 침투
        → home 포함 전 자세 IK 불가). 해당 쌍만 제외하고 나머지 쌍은 검사 유지.
        """
        p.performCollisionDetection(physicsClientId=self._client)
        contacts = p.getContactPoints(
            bodyA=self._robot, bodyB=self._robot, physicsClientId=self._client
        )
        excluded: set[tuple[int, int]] = set()
        for c in contacts:
            if c[8] >= 0.0:  # 침투(음수 거리)만 — 근접 접촉은 유지
                continue
            pair = (min(c[3], c[4]), max(c[3], c[4]))
            if pair in excluded:
                continue
            excluded.add(pair)
            p.setCollisionFilterPair(
                self._robot, self._robot, pair[0], pair[1], 0,
                physicsClientId=self._client,
            )
            logger.warning(
                "URDF zero-pose 침투 쌍 collision 제외: %s <-> %s (%.2fmm) — "
                "mesh artifact (urdf=%s)",
                link_names.get(pair[0], pair[0]),
                link_names.get(pair[1], pair[1]),
                c[8] * 1000.0,
                self._urdf_path.name,
            )

    def _require_init(self) -> None:
        if not self._initialized:
            raise RuntimeError("PybulletKinematics 미초기화 — initialize() 먼저")

    def _set_chain(self, angles: list[float]) -> None:
        for idx, angle in zip(self._chain_indices, angles):
            p.resetJointState(self._robot, idx, angle, physicsClientId=self._client)

    def _ee_state(self) -> tuple[Position3, Quaternion]:
        state = p.getLinkState(
            self._robot,
            self._ee_index,
            computeForwardKinematics=True,
            physicsClientId=self._client,
        )
        return tuple(state[4]), tuple(state[5])

    def _self_collision_unlocked(self) -> bool:
        p.performCollisionDetection(physicsClientId=self._client)
        contacts = p.getContactPoints(
            bodyA=self._robot, bodyB=self._robot, physicsClientId=self._client
        )
        return any(c[8] < 0.0 for c in contacts)
