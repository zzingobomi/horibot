"""PybulletKinematics — PyBullet 기반 Kinematics (ideal URDF, sag 없음).

옛 backend/modules/kinematics/adapters/pybullet_kinematics.py port.
D1 = plain URDF load (link_offset patch 는 D4 Mirror[Bundle] 자리).

dof = tcp link 의 **ancestor revolute joint** 만 (gripper 등 sibling 가지 제외).
PyBullet 의 jointIndex == childLinkIndex 를 이용해 tcp 에서 base 로 거슬러 올라가며
chain 식별 → so101_6dof=6 (옛 코드의 "전체 revolute=7" 오포함 정정).
"""

from __future__ import annotations

import logging
import math
import threading
from pathlib import Path
from typing import Sequence

import numpy as np
import pybullet as p

from ..kinematics import Position3, Quaternion, RotMatrix3x3
from .analytic import AnalyticIk
from .analytic_zyyyx import ZyyyxAnalyticIk

logger = logging.getLogger(__name__)

IK_MAX_ITER = 100
IK_TOLERANCE = 1e-4
IK_POS_ERROR_LIMIT = 0.01
# 주 경로 conditional refine (docs/motion.md §10.D). 단발 수치 IK 는 자세가 빡센
# 지점에서 위치를 내주고 자세를 맞추는 해로 수렴 → 위치잔차가 cm급까지 뜬다
# (§10.B 실측: 자세회전 120° 에서 단발 51.9mm, 결과-seed 재해 시 0.6mm 로 붕괴).
# 잔차가 이 임계를 넘을 때만 결과를 seed 로 재해해 회수한다. 쉬운 해(≤임계)는
# 재해 0회 = 비용 없음. 게이트(IK_POS_ERROR_LIMIT) 검사 앞에서 도므로 게이트에
# 걸릴 잔차(~15mm)도 회수해 통과시킨다 (refine 이 IK 실패를 줄이는 메커니즘 §6).
_IK_REFINE_THRESHOLD_M = 0.003
# 재해 상한 — §4 실측 "결과-seed 1회 재호출로 5mm/0.05° 수렴", walk 의
# _WALK_REFINE_ITERS(5) 와 동급. 임계 도달 시 조기 종료라 대개 1~2회.
_IK_REFINE_ITERS = 5
# 바닥충돌 안전여유 — 관통(거리<0)만이 아니라 바닥에서 이 거리 이내로 그리퍼
# 링크(몸통 포함)가 접근하면 기각. 그리퍼 몸통이 바닥을 "스치는"(관통 아닌 접촉)
# 파지를 막는다 (2026-07-15: 몸통이 바닥 닿았는데 관통-only 체크가 통과시킨 사고).
FLOOR_MARGIN_M = 0.006
# seed 1회로 수렴 못 하면 random restart (PyBullet 은 seed 에서 출발하는 local
# 솔버라 해가 존재해도 놓침 — "도달 가능한데 IK 실패" 방지). restart 중 seed 에
# 가장 가까운 해 선택 → motion 연속성 유지.
# 200 근거 (2026-07-09 SO-101 손 시연 자세 실측): orientation 붙은 top-down 파지
# 자세는 basin 이 좁아 균등 재시작 성공까지 median 8 / max 40회 — 옛 24 는 복권
# (같은 자세가 rng 이력 따라 됐다 안 됐다). 800회 ≈ 0.1s (DIRECT) 라 200 은 공짜.
IK_RESTARTS = 200

# continuation walk (docs/motion_ik_reachability.md §3 [B] 의 솔버 내장화):
# 6D one-shot 은 목표 자세 basin 이 좁으면 seed 가 멀 때 결정적으로 실패하는
# false negative 가 있다 (§3 [A] — 2026-07-16 plan_pick 48/52 전멸 원인). 위치만
# 먼저 풀면(3D 구속 = 넓은 basin) 그 지점에서 TCP 위치를 고정한 채 자세를 조금씩
# 회전하는 각 스텝은 직전 해 바로 옆이라 local 솔버가 수렴한다. randomness 없음 —
# 같은 입력 = 같은 결과 (2026-07-09 복권 버그 재발 금지).
_WALK_STEP_RAD = math.radians(15.0)  # 자세 slerp 사다리 스텝 (~최대 12스텝/180°)
_WALK_POS_RESTARTS = 20  # 위치-only 단계 재시작 상한 (위치 basin 은 넓어 소예산)
_WALK_ORI_TOL_RAD = math.radians(5.0)  # 최종 자세 잔차 허용 — walk 는 자세도 검증
# 스텝 당 재선형화 반복 — PyBullet 단일 호출은 pos+ori 동시 수렴이 약해 자세를
# 맞추며 위치를 ~15mm 트레이드오프함 (2026-07-16 실측: 결과를 seed 로 1회 재호출
# 만으로 5mm/0.05° 수렴). 상한 5 는 보수 여유.
_WALK_REFINE_ITERS = 5

# 모든 robot type URDF 는 `tcp` 라는 link 를 가져야 함 (UR tool0 패턴, fail-fast).
TCP_LINK_NAME = "tcp"

# ── 장애물 점군 게이트 (grasping.md §1) ──────────
# 관측 점군은 표면 샘플 — voxel 대표점마다 작은 구를 놓아 "표면을 뚫고 들어가는"
# 링크를 잡는다. 조 안쪽 면 설계 여유(FIXED_JAW_CLEAR 5mm)와 depth 노이즈(σ~1mm)
# 사이: 반경 3mm 구 + 침투 2mm 임계 → 정상 파지(표면과 5mm 여유)는 +2mm 마진으로
# 통과, 물체 관통(표면점 중심 1mm 이내 진입)은 기각.
_OBSTACLE_VOXEL_M = 0.006
_OBSTACLE_RADIUS_M = 0.003
_OBSTACLE_PENETRATION_M = 0.002


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
        # 해석적 branch 열거기 (docs/motion.md §11) — initialize 에서 시도,
        # None = 수치 폴백. 침묵 금지 — 모드는 각 해석기가 부팅 로그 1줄로
        # 밝힌다. 순서: EAIK(so101 6R + omx 5축 실측 분해 확인) → Z·YYY·X
        # closed-form (EAIK 부재 환경 백업 — omx_handover_prep.md §8-2,
        # pi 소스빌드 실패 클래스 대비).
        self._analytic: AnalyticIk | ZyyyxAnalyticIk | None = None
        # 바닥 평면 body (floor_collision 게이트) — 첫 사용 시 lazy 생성
        self._plane = -1
        # 장애물 점군 body 들 (obstacle_collision 게이트) — set_obstacle_points 관리
        self._obstacle_bodies: list[int] = []
        self._obstacle_shape = -1  # 공유 구 collision shape (1회 생성)

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

            self._analytic = self._build_analytic()

            self._initialized = True
            logger.info(
                "PybulletKinematics: dof=%d (chain) / %d movable, tcp=%s, "
                "IK=%s",
                len(chain), len(movable), TCP_LINK_NAME,
                f"해석적({self._analytic.family})+polish"
                if self._analytic else "수치(walk+restart)",
            )

    def _build_analytic(self) -> AnalyticIk | ZyyyxAnalyticIk | None:
        """로드된 바디의 zero-pose 축/원점에서 해석기 구성 (파일 재파싱 없음).

        patched(캘) URDF 그대로 사용 — 캘 오차(~1.4° 스큐)는 snap 이 흡수하고
        정밀도는 polish(_ik_from_seed refine, 같은 캘 모델)가 회수한다.
        호출 시점: initialize 내부 (_lock 보유, zero pose 로 두고 추출).
        EAIK(6R 계열) 우선, 불가 시 Z·YYY·X 5축 closed-form (omx) 시도.
        """
        self._set_chain([0.0] * len(self._chain_indices))
        axes: list[tuple[float, float, float]] = []
        origins: list[tuple[float, float, float]] = []
        for j in self._chain_indices:
            info = p.getJointInfo(self._robot, j, physicsClientId=self._client)
            st = p.getLinkState(
                self._robot, j, computeForwardKinematics=True,
                physicsClientId=self._client,
            )
            rot = np.array(p.getMatrixFromQuaternion(st[5])).reshape(3, 3)
            a = rot @ np.asarray(info[13], dtype=float)
            a = a / np.linalg.norm(a)
            axes.append((float(a[0]), float(a[1]), float(a[2])))
            origins.append(
                (float(st[4][0]), float(st[4][1]), float(st[4][2]))
            )
        tcp_pos, tcp_quat = self._ee_state()
        r0 = np.array(p.getMatrixFromQuaternion(tcp_quat)).reshape(3, 3)
        ik: AnalyticIk | ZyyyxAnalyticIk | None = AnalyticIk.try_build(
            axes, origins, tcp_pos, r0, self._chain_lower, self._chain_upper
        )
        if ik is None:
            ik = ZyyyxAnalyticIk.try_build(
                axes, origins, tcp_pos, r0, self._chain_lower, self._chain_upper
            )
        return ik

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
            # 0) 해석적 경로 (§11) — 전 branch 열거 + polish. 완전(모든 basin)
            #    + 결정적 + "도달불가" 를 수 ms 에 증명. walk/restart 불필요.
            if target_quaternion is not None and self._analytic is not None:
                return self._ik_analytic(
                    target_position, target_quaternion, seed
                )
            # 1) seeded 1회 — 현재 자세 근처 해 (motion 연속성, 대부분 여기서 끝).
            sol = self._ik_from_seed(target_position, target_quaternion, seed)
            if sol is not None:
                return sol
            # 2) continuation walk — 위치만 먼저 도달한 뒤 TCP 위치를 고정한 채
            #    자세를 slerp 사다리로 회전 (관절만 움직임). one-shot 이 못 찾는
            #    좁은 basin 의 해를 결정적으로 찾는다 (상단 _WALK_* 주석).
            if target_quaternion is not None:
                sol = self._ik_walk(
                    target_position, target_quaternion, seed, restarts
                )
                if sol is not None:
                    return sol
            # 3) 실패 = local 솔버가 seed basin 에서 못 찾은 것일 수 있음 (해는 존재
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
                # probe 는 refine 없이 basin 만 탐색 (실패 그룹 200×5 낭비 차단).
                cand = self._ik_from_seed(
                    target_position, target_quaternion, rand, refine=False
                )
                if cand is not None:
                    dist = sum((a - b) ** 2 for a, b in zip(cand, seed))
                    if dist < best_dist:
                        best_dist, best = dist, cand
            if best is None:
                logger.debug(
                    "IK 실패 (seed + restart %d 모두) target=%s",
                    IK_RESTARTS, target_position,
                )
                return None
            # 승자만 refine — probe 에서 내준 위치(≤10mm)를 결과-seed 재해로 회수.
            refined = self._ik_from_seed(
                target_position, target_quaternion, best, refine=True
            )
            return refined if refined is not None else best

    def _ik_analytic(
        self,
        target_position: Position3,
        target_quaternion: Quaternion,
        seed: list[float],
    ) -> list[float] | None:
        """해석적 branch 열거 → seed 최근접 순 polish → 첫 통과 채택.

        - branch = snap 모델의 seed (±리밋 clamp) — 실 모델 정밀도는
          _ik_from_seed 의 conditional refine(=polish) 이 회수.
        - seed 거리순 = motion 연속성 (같은 목표라도 팔 위치에 따라 가까운
          basin 우선 — 구성 플립 최소화).
        - 자세 잔차 게이트: polish 는 위치만 검증하므로 (알려진 결함) 해석
          경로에서는 자세 오답 branch 채택을 여기서 차단.
        - 전 branch 실패 = 도달불가 확정 (수치처럼 "못 찾은 것일 수도" 가 없음).
        """
        branches = self._analytic.branches(  # type: ignore[union-attr]
            target_position, target_quaternion
        )
        branches.sort(
            key=lambda b: sum((a - s) ** 2 for a, s in zip(b, seed))
        )
        for b in branches:
            sol = self._ik_from_seed(target_position, target_quaternion, b)
            if sol is None:
                continue
            self._set_chain(sol)
            _, got = self._ee_state()
            dot = abs(sum(a * c for a, c in zip(got, target_quaternion)))
            if 2.0 * math.acos(min(1.0, dot)) > _WALK_ORI_TOL_RAD:
                continue
            return sol
        logger.debug(
            "IK 도달불가 확정 (해석 branch %d개 전부 기각) target=%s",
            len(branches), target_position,
        )
        return None

    def _ik_raw(
        self,
        target_position: Position3,
        target_quaternion: Quaternion | None,
        seed_chain: list[float],
    ) -> list[float]:
        """calculateInverseKinematics 1회 (검증 없음, 호출자가 _lock 보유)."""
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
        return [result[i] for i in self._chain_in_movable]

    def _ik_from_seed(
        self,
        target_position: Position3,
        target_quaternion: Quaternion | None,
        seed_chain: list[float],
        refine: bool = True,
    ) -> list[float] | None:
        """chain-공간 seed 하나로 1회 IK + 수렴/충돌 검증 (호출자가 _lock 보유).

        수렴 검증은 reachability vs self-collision 원인 분리 (debug 로그).
        refine=False 는 random-restart probe 용 — 거긴 "아무 basin 이나 찾기"가
        목적이라 정밀 refine 이 낭비(실패 그룹에서 200×5 = 실물 전멸 27s 사고).
        승자만 호출자가 1회 refine 한다.
        """
        angles = self._ik_raw(target_position, target_quaternion, seed_chain)

        self._set_chain(angles)
        actual_pos, _ = self._ee_state()
        error = float(
            np.linalg.norm(np.array(actual_pos) - np.array(target_position))
        )
        # 조건부 refine — 잔차가 임계 초과일 때만 결과-seed 재해 (상단 상수 주석).
        # 게이트 검사 앞이라 15mm 단발도 ~sub-mm 로 회수해 통과. quat 포함 재해라
        # 자세도 함께 재수렴 (자세 잔차 검증은 기존대로 walk/호출자 책임).
        #
        # ★ best 추적 필수 — 수치 IK 재해는 pose-hard/특이점 근처에서 발산할 수
        #   있다(7mm→12mm). 마지막 반복을 그대로 쓰면 단발보다 나빠져 게이트를
        #   넘겨 None → 단발이면 통과했을 후보가 refine 후 실패로 뒤집힌다(§6 불변식
        #   "refine 은 실패를 늘리지 않는다" 위반, 2026-07-20 실물 52가족 전멸 사고).
        #   best_error ≤ 단발_error 를 보장해 refine 이 절대 나빠지지 않게 한다.
        if refine and error > _IK_REFINE_THRESHOLD_M:
            best_angles, best_error = angles, error
            seed_k = angles
            for _ in range(_IK_REFINE_ITERS):
                seed_k = self._ik_raw(target_position, target_quaternion, seed_k)
                self._set_chain(seed_k)
                actual_pos, _ = self._ee_state()
                error = float(
                    np.linalg.norm(np.array(actual_pos) - np.array(target_position))
                )
                if error < best_error:
                    best_angles, best_error = seed_k, error
                if error <= _IK_REFINE_THRESHOLD_M:
                    break
            # 발산 반복이 아닌 best 를 채택 + self-collision 검사가 볼 상태 복원.
            angles, error = best_angles, best_error
            self._set_chain(angles)
        if error > IK_POS_ERROR_LIMIT:
            return None
        if self._self_collision_unlocked():
            return None
        return angles

    def _ik_walk(
        self,
        target_position: Position3,
        target_quaternion: Quaternion,
        seed_chain: list[float],
        restarts: int | None,
    ) -> list[float] | None:
        """continuation IK (호출자가 _lock 보유) — 위치-only 도달 → 자세 slerp 걸음.

        docs/motion_ik_reachability.md §3 [B] ("seed 연쇄로 걸어가면 도달, config
        존재") 의 솔버 내장화. 스텝 실패(수렴/자세중 self-collision)는 보수 포기 —
        walk 는 실행 경로가 아니라 해 탐색 장치라 뒤의 random restart 가 받는다.
        """
        # a) 위치-only (자세 자유 3DOF = 넓은 basin). seed 실패 시 소예산 재시작.
        q = self._ik_from_seed(target_position, None, seed_chain)
        if q is None:
            budget = min(
                _WALK_POS_RESTARTS,
                restarts if restarts is not None else _WALK_POS_RESTARTS,
            )
            rng = np.random.default_rng(0)
            for _ in range(budget):
                rand = [
                    float(rng.uniform(lo, hi))
                    for lo, hi in zip(self._chain_lower, self._chain_upper)
                ]
                q = self._ik_from_seed(target_position, None, rand)
                if q is not None:
                    break
            if q is None:
                logger.debug(
                    "IK walk 실패 @위치-only: target=%s (재시작 %d 포함) — "
                    "위치 자체가 workspace 밖",
                    target_position, budget,
                )
                return None
        # b) 그 지점의 실제 자세 → 목표 자세 slerp 사다리 (TCP 위치 고정, 관절만
        #    회전). 각 스텝은 재선형화 refinement 루프 — PyBullet 단일 호출이
        #    자세를 맞추며 위치를 내주는 것을 결과-seed 재호출로 회수.
        self._set_chain(q)
        _, cur = self._ee_state()
        dot = sum(a * b for a, b in zip(cur, target_quaternion))
        tgt = (  # 반대 반구 표현이면 뒤집어 최단호 보간 (같은 회전)
            target_quaternion
            if dot >= 0
            else tuple(-v for v in target_quaternion)
        )
        angle = 2.0 * math.acos(min(1.0, abs(dot)))
        n = max(1, int(math.ceil(angle / _WALK_STEP_RAD)))
        for k in range(1, n + 1):
            step_quat = p.getQuaternionSlerp(cur, tgt, k / n)
            sol: list[float] | None = None
            seed_k = q
            pos_err = math.inf
            collided = False
            for _ in range(_WALK_REFINE_ITERS):
                cand = self._ik_raw(target_position, step_quat, seed_k)
                self._set_chain(cand)
                actual_pos, _ = self._ee_state()
                pos_err = float(
                    np.linalg.norm(
                        np.array(actual_pos) - np.array(target_position)
                    )
                )
                seed_k = cand
                if pos_err <= IK_POS_ERROR_LIMIT:
                    collided = self._self_collision_unlocked()
                    if not collided:
                        sol = cand
                        break
            if sol is None:
                logger.debug(
                    "IK walk 실패 @slerp %d/%d (총 %.0f°): %s target=%s",
                    k, n, math.degrees(angle),
                    "self-collision" if collided
                    else f"pos_err {pos_err * 1000:.1f}mm > "
                    f"{IK_POS_ERROR_LIMIT * 1000:.0f}mm "
                    f"(refine {_WALK_REFINE_ITERS}회)",
                    target_position,
                )
                return None
            q = sol
        # c) 최종 자세 잔차 검증 — _ik_from_seed 는 위치 잔차만 검사해서
        #    "자세 틀린 해" 도 통과시킨다 (알려진 결함). walk 결과만큼은 자세를
        #    검증해 오답을 성공으로 돌려주지 않는다.
        self._set_chain(q)
        _, got = self._ee_state()
        err = 2.0 * math.acos(
            min(1.0, abs(sum(a * b for a, b in zip(got, target_quaternion))))
        )
        if err > _WALK_ORI_TOL_RAD:
            logger.debug(
                "IK walk 실패 @최종검증: 자세 잔차 %.1f° > %.0f° target=%s",
                math.degrees(err), math.degrees(_WALK_ORI_TOL_RAD),
                target_position,
            )
            return None
        logger.debug(
            "IK walk 성공: %d스텝(%.0f°) 자세잔차 %.2f° target=%s",
            n, math.degrees(angle), math.degrees(err), target_position,
        )
        return q

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

    def floor_collision(self, joint_angles: Sequence[float], floor_z: float) -> bool:
        """수평 바닥 평면(z=floor_z) 침투 검사 — planner 충돌 게이트 최소형.

        평면은 별도 body 라 self-collision(robot↔robot 질의)/IK 에 영향 없음.
        base 쪽 고정 링크(첫 체인 관절 이전)는 제외 — 로봇이 그 평면 위에 설치돼
        상시 접촉이 상수 (검출 floor_z 의 ±cm 오차로 전 자세가 영구 기각되는 것
        방지). 링크 index 는 URDF 트리 순서라 체인 첫 관절 index 미만 = base 쪽.
        """
        self._require_init()
        with self._lock:
            if self._plane == -1:
                col = p.createCollisionShape(
                    p.GEOM_PLANE, physicsClientId=self._client
                )
                self._plane = p.createMultiBody(
                    baseCollisionShapeIndex=col,
                    basePosition=(0.0, 0.0, floor_z),
                    physicsClientId=self._client,
                )
            else:
                p.resetBasePositionAndOrientation(
                    self._plane,
                    (0.0, 0.0, floor_z),
                    (0.0, 0.0, 0.0, 1.0),
                    physicsClientId=self._client,
                )
            self._set_chain(list(joint_angles))
            # getClosestPoints(distance=margin) = 관통뿐 아니라 margin 이내 근접까지
            # 반환 (c[8]=거리, 음수=관통). 그리퍼 몸통이 바닥을 스치는 것도 잡는다.
            contacts = p.getClosestPoints(
                bodyA=self._robot,
                bodyB=self._plane,
                distance=FLOOR_MARGIN_M,
                physicsClientId=self._client,
            )
            first_moving = self._chain_indices[0]
            return any(c[8] < FLOOR_MARGIN_M and c[3] >= first_moving for c in contacts)

    def set_obstacle_points(
        self, points: Sequence[tuple[float, float, float]] | None
    ) -> None:
        self._require_init()
        with self._lock:
            for body in self._obstacle_bodies:
                p.removeBody(body, physicsClientId=self._client)
            self._obstacle_bodies.clear()
            if not points:
                return
            centers = self._voxel_centroids(np.asarray(points, dtype=float))
            if self._obstacle_shape == -1:
                self._obstacle_shape = p.createCollisionShape(
                    p.GEOM_SPHERE,
                    radius=_OBSTACLE_RADIUS_M,
                    physicsClientId=self._client,
                )
            for c in centers:
                self._obstacle_bodies.append(
                    p.createMultiBody(
                        baseCollisionShapeIndex=self._obstacle_shape,
                        basePosition=(float(c[0]), float(c[1]), float(c[2])),
                        physicsClientId=self._client,
                    )
                )

    def obstacle_collision(
        self, joint_angles: Sequence[float], *, gripper_open: bool = False
    ) -> bool:
        self._require_init()
        with self._lock:
            if not self._obstacle_bodies:
                return False
            self._set_chain(list(joint_angles))
            gripper_idx = [
                i for i in self._movable_indices if i not in self._chain_indices
            ]
            if gripper_open:
                # URDF 상한 = 벌림 (so101 gripper_jaw upper=1.746 — 양수 open 규약)
                for gi in gripper_idx:
                    upper = self._movable_upper[self._movable_indices.index(gi)]
                    p.resetJointState(
                        self._robot, gi, upper, physicsClientId=self._client
                    )
            try:
                p.performCollisionDetection(physicsClientId=self._client)
                for body in self._obstacle_bodies:
                    contacts = p.getContactPoints(
                        bodyA=self._robot, bodyB=body, physicsClientId=self._client
                    )
                    if any(c[8] < -_OBSTACLE_PENETRATION_M for c in contacts):
                        return True
                return False
            finally:
                if gripper_open:  # 그리퍼를 URDF zero 로 복원 — self_collision/IK
                    for gi in gripper_idx:  # 등 다른 질의의 전제 상태 오염 방지
                        p.resetJointState(
                            self._robot, gi, 0.0, physicsClientId=self._client
                        )

    @staticmethod
    def _voxel_centroids(pts: np.ndarray) -> np.ndarray:
        """점군 → voxel 당 centroid — 장애물 body 수 상한 (수천 점 → 수백 구)."""
        keys = np.floor(pts / _OBSTACLE_VOXEL_M).astype(np.int64)
        _, inverse, counts = np.unique(
            keys, axis=0, return_inverse=True, return_counts=True
        )
        sums = np.zeros((len(counts), 3), dtype=np.float64)
        np.add.at(sums, inverse, pts)
        return sums / counts[:, None]

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
