"""cross-robot 충돌 체커 — 두 URDF 를 한 PyBullet 세계에 놓고 링크 간 근접 판정.

handover task 전용의 **독립 유틸** (2026-07-17 신설, 실물 미검증). motion 모듈을
침범하지 않는다 — 각 robot 의 motion 모듈은 분산 host 에 자기 robot 만 알고,
cross-robot 게이트를 그 안에 넣으려면 peer 상태 구독 배선이 필요해 지금은
과하다. 정석 통합 자리는 motion resolve_reachable 의 ③b(장애물) 옆
(contract.py 가 예약해 둔 슬롯) — 아래 TODO 와 motion/module.py 주석 참조.

TODO(cross-robot): pick_and_place 실물 검증 완료 후 —
  ① motion resolve_reachable 에 peer robot 점유(joints+base_pose) 게이트 추가
  ② 이 체커는 그 게이트의 로컬 프로토타입으로 흡수/폐기
  ③ margin 은 실물 특성화로 튜닝 (지금 2cm 는 크로스캘 σ_t ~8mm + FK 오차의
     보수 기본값 — 추측이므로 실측 전 신뢰 금지)

판정 의미론: 두 로봇의 링크 표면 간 최소 거리가 margin 미만이면 "충돌 위험".
그리퍼 관절(체인 밖 movable)은 **벌림(상한)으로 고정** — 최악(최대 점유)
envelope 기준의 보수 판정.
"""

from __future__ import annotations

import logging
import math
import threading
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pybullet as p

logger = logging.getLogger(__name__)

# 링크 간 안전 여유 — 미만이면 충돌 위험 판정 (모듈 docstring TODO: 실측 튜닝 전
# 보수 기본값. 크로스캘 σ_t ~8mm + 양쪽 FK/backlash ~1cm 급 근거).
_DEFAULT_MARGIN_M = 0.02
# 경로 표본 간격 — 관절 최대 이동 기준 (충돌 검사만이라 촘촘해도 싸다).
_PATH_STEP_RAD = math.radians(6.0)


@dataclass(frozen=True)
class BasePose:
    """robot base 의 world(=so101 base) frame 자세 — robots.yaml base_pose 투영."""

    x: float
    y: float
    z: float
    yaw_rad: float


class CrossRobotChecker:
    """두 robot(a=world 원점, b=base_pose) 의 구성 쌍 충돌 판정.

    lazy init — 생성은 값싸고 첫 판정 때 PyBullet(DIRECT) 로드. thread-safe
    (판정은 짧은 lock 구간). joints 배열은 각 robot 의 **팔 관절**(TcpState.joints
    순서 = URDF movable 선두 — PybulletKinematics 체인 규약과 동일 가정) 이며,
    나머지 movable(그리퍼)은 벌림 상한으로 고정한다.
    """

    def __init__(
        self,
        urdf_a: Path | str,
        urdf_b: Path | str,
        base_b: BasePose,
        margin_m: float = _DEFAULT_MARGIN_M,
    ) -> None:
        self._urdf_a = str(urdf_a)
        self._urdf_b = str(urdf_b)
        self._base_b = base_b
        self.margin_m = margin_m
        self._client: int | None = None
        self._body_a = -1
        self._body_b = -1
        self._movable_a: list[int] = []
        self._movable_b: list[int] = []
        self._upper_a: list[float] = []
        self._upper_b: list[float] = []
        self._lock = threading.Lock()

    # ─── lifecycle ────────────────────────────────────────────────

    def _ensure_init(self) -> None:
        if self._client is not None:
            return
        self._client = p.connect(p.DIRECT)
        self._body_a = p.loadURDF(
            self._urdf_a, useFixedBase=True, physicsClientId=self._client
        )
        qz = math.sin(self._base_b.yaw_rad / 2.0)
        qw = math.cos(self._base_b.yaw_rad / 2.0)
        self._body_b = p.loadURDF(
            self._urdf_b,
            basePosition=(self._base_b.x, self._base_b.y, self._base_b.z),
            baseOrientation=(0.0, 0.0, qz, qw),
            useFixedBase=True,
            physicsClientId=self._client,
        )
        self._movable_a, self._upper_a = self._movable_joints(self._body_a)
        self._movable_b, self._upper_b = self._movable_joints(self._body_b)
        logger.info(
            "CrossRobotChecker init — a=%s b=%s base_b=(%.3f,%.3f,%.3f,%.1f°) "
            "margin=%.0fmm",
            Path(self._urdf_a).name, Path(self._urdf_b).name,
            self._base_b.x, self._base_b.y, self._base_b.z,
            math.degrees(self._base_b.yaw_rad), self.margin_m * 1000,
        )

    def _movable_joints(self, body: int) -> tuple[list[int], list[float]]:
        assert self._client is not None
        idx, upper = [], []
        for j in range(p.getNumJoints(body, physicsClientId=self._client)):
            info = p.getJointInfo(body, j, physicsClientId=self._client)
            if info[2] != p.JOINT_FIXED:
                idx.append(j)
                upper.append(float(info[9]))
        return idx, upper

    def close(self) -> None:
        with self._lock:
            if self._client is not None:
                p.disconnect(physicsClientId=self._client)
                self._client = None

    # ─── 판정 ─────────────────────────────────────────────────────

    def _set_config(
        self, body: int, movable: list[int], upper: list[float], joints: list[float]
    ) -> None:
        assert self._client is not None
        for k, j in enumerate(movable):
            if k < len(joints):
                val = float(joints[k])  # 팔 관절 (TcpState.joints 순서)
            else:
                val = upper[k]  # 그리퍼 등 잔여 movable — 벌림(최대 점유) 고정
            p.resetJointState(body, j, val, physicsClientId=self._client)

    def in_collision(
        self, joints_a: list[float], joints_b: list[float]
    ) -> bool:
        """구성 쌍의 링크 간 최소 거리 < margin 이면 True (충돌 위험)."""
        with self._lock:
            self._ensure_init()
            self._set_config(self._body_a, self._movable_a, self._upper_a, joints_a)
            self._set_config(self._body_b, self._movable_b, self._upper_b, joints_b)
            pts = p.getClosestPoints(
                bodyA=self._body_a,
                bodyB=self._body_b,
                distance=self.margin_m,
                physicsClientId=self._client,
            )
            return len(pts) > 0

    def path_in_collision(
        self,
        path_a: list[list[float]],
        joints_b: list[float],
    ) -> bool:
        """a 의 관절 경로(waypoint 열, 사이 lerp 표본) vs b 고정 구성.

        실행부(MoveJ/MoveL)의 실제 궤적과 동일하진 않다 — 관절 보간 근사
        (MoveJ 등가). MoveL 구간은 호출부가 IK 해 열을 path_a 로 넘길 것.
        """
        if not path_a:
            return False
        prev = path_a[0]
        if self.in_collision(prev, joints_b):
            return True
        for nxt in path_a[1:]:
            qa, qb = np.asarray(prev, float), np.asarray(nxt, float)
            n = max(1, int(math.ceil(float(np.max(np.abs(qb - qa))) / _PATH_STEP_RAD)))
            for k in range(1, n + 1):
                q = [float(v) for v in qa + (qb - qa) * (k / n)]
                if self.in_collision(q, joints_b):
                    return True
            prev = nxt
        return False
