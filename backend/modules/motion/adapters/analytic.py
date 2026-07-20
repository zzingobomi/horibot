"""해석적 IK branch 열거기 — EAIK(subproblem decomposition) 기반, optional.

docs/motion.md §11 (2026-07-20 IK 대수술): 수치 IK 는 해를 놓쳐도(존재하는데
못 찾음) 증명을 못 해 walk/restart 보상 기계가 자랐다. so101 은 Pieper
3평행축(J2∥J3∥J4) 충족 = 닫힌 해 존재 — 모든 branch(≤8)를 대수적으로 열거하면
"도달불가"가 수 ms 에 확정되고 false negative 클래스가 소멸한다.

역할 분담 (snap → 해석 → polish):
  - 축 이상화(snap): EAIK 분해는 정확한 평행/직교를 요구하는데 URDF 축은
    모델링/캘리브레이션으로 ~0.5-1.4° 어긋남 → 각 축을 최근접 좌표축으로 snap.
  - 해석해 = **seed 생성기** (snap 모델 기준, 실 모델 대비 ~1-3mm) — 정밀도가
    아니라 **완전성**(모든 basin 열거)이 이 계층의 계약.
  - 정밀도는 호출자(PybulletKinematics._ik_from_seed 의 conditional refine)가
    실(캘 적용) 모델에서 마무리.
  - branch 는 [-π,π] 정규화 + **리밋 clamp** (기각 아님 — 진짜 해가 리밋 경계에
    있으면 snap 오차가 경계 밖으로 밀어냄, clamp 후 polish 가 안으로 회수.
    2026-07-20 winner-debug: J4=1.61 vs 리밋 1.52 에서 정답 branch 를 리밋
    필터가 버리던 실측 사례).

optional 경계: EAIK 미설치(예: Pi 소스빌드 실패) 또는 분해 불가 구조(비-Pieper
robot)면 try_build 가 None — 호출자는 기존 수치 경로(walk+restart)로 폴백.
어느 모드인지 부팅 로그 1줄로 반드시 보인다 (침묵 폴백 금지).
"""

from __future__ import annotations

import logging
import math
from typing import Any

import numpy as np

logger = logging.getLogger(__name__)

try:  # optional 의존성 — 부재 시 수치 폴백 (아래 try_build)
    from eaik.IK_HP import HPRobot

    _EAIK_AVAILABLE = True
except ImportError:  # pragma: no cover — CI 는 EAIK 설치 환경
    HPRobot = None  # type: ignore[assignment]
    _EAIK_AVAILABLE = False

# snap 허용 상한 — 축이 좌표축에서 이보다 크게 어긋나면 "이 URDF 는 이 클래스가
# 아니다" (snap 이 기하를 왜곡해 seed 가 polish basin 을 벗어날 위험) → 폴백.
_SNAP_MAX_RAD = math.radians(5.0)
# branch dedup — clamp 후 동일 구성으로 붕괴한 branch 제거 (관절공간 L∞)
_DEDUP_ATOL_RAD = 1e-3


class AnalyticIk:
    """so101 클래스(3평행축) 해석적 IK branch 열거기.

    생성은 try_build 로만 — EAIK 부재/분해 불가/snap 과대 면 None.
    스레드 안전: EAIK Robot 호출은 상태 없음(read-only) — 락 불필요.
    """

    def __init__(
        self,
        robot: Any,  # eaik HPRobot — optional import 이라 정적 타입 미지정
        r0: np.ndarray,
        lower: list[float],
        upper: list[float],
        family: str,
    ) -> None:
        self._robot = robot
        self._r0_t = r0.T.copy()  # zero-pose tcp 회전 보정 (EAIK EE 기준 회전)
        self._lower = np.asarray(lower)
        self._upper = np.asarray(upper)
        self.family = family

    @staticmethod
    def try_build(
        axes: list[tuple[float, float, float]],
        origins: list[tuple[float, float, float]],
        tcp_origin: tuple[float, float, float],
        tcp_rot_zero: np.ndarray,
        lower: list[float],
        upper: list[float],
    ) -> "AnalyticIk | None":
        """zero-pose 월드 축/원점(+tcp) → 해석기. 불가하면 None (사유 로그).

        입력은 호출자(PybulletKinematics.initialize)가 로드된 바디에서 추출 —
        URDF 파일 재파싱 없음 (patched URDF 가 임시파일이어도 무관).
        """
        if not _EAIK_AVAILABLE:
            logger.info("AnalyticIk: EAIK 미설치 — 수치 IK 폴백")
            return None
        n = len(axes)
        H = np.zeros((n, 3))
        worst_snap = 0.0
        for i, a in enumerate(np.asarray(axes, dtype=float)):
            a = a / np.linalg.norm(a)
            k = int(np.argmax(np.abs(a)))
            H[i, k] = math.copysign(1.0, a[k])
            worst_snap = max(
                worst_snap,
                math.acos(min(1.0, abs(float(np.dot(a, H[i]))))),
            )
        if worst_snap > _SNAP_MAX_RAD:
            logger.info(
                "AnalyticIk: 축 snap %.2f° > %.1f° — 이 URDF 는 좌표축 정렬"
                " 클래스가 아님, 수치 IK 폴백",
                math.degrees(worst_snap), math.degrees(_SNAP_MAX_RAD),
            )
            return None
        org = np.asarray(origins, dtype=float)
        P = np.zeros((n + 1, 3))
        P[0] = org[0]
        for i in range(1, n):
            P[i] = org[i] - org[i - 1]
        P[n] = np.asarray(tcp_origin, dtype=float) - org[-1]
        try:
            assert HPRobot is not None  # _EAIK_AVAILABLE 가드 통과
            robot = HPRobot(H, P)
        except Exception as e:  # noqa: BLE001 — 외부 lib 실패는 폴백 사유
            logger.warning("AnalyticIk: EAIK 구성 실패 (%s) — 수치 IK 폴백", e)
            return None
        if not robot.hasKnownDecomposition():
            logger.info(
                "AnalyticIk: 분해 불가 (family=%s) — 수치 IK 폴백",
                robot.getKinematicFamily(),
            )
            return None
        ik = AnalyticIk(
            robot, np.asarray(tcp_rot_zero, dtype=float),
            list(lower), list(upper), robot.getKinematicFamily(),
        )
        logger.info(
            "AnalyticIk 활성: family=%s, 축 snap 최대 %.2f°",
            ik.family, math.degrees(worst_snap),
        )
        return ik

    def branches(
        self,
        position: tuple[float, float, float],
        quaternion: tuple[float, float, float, float],
    ) -> list[list[float]]:
        """목표 pose 의 모든 IK branch (정규화+리밋 clamp+dedup, 순서 무보장).

        LS(최소자승 근사) 해 포함 — snap 모델은 실 모델 pose 를 '정확히'는 못
        맞춰 정상 해도 LS 로 분류된다. 도달성 판정은 polish 후 호출자 몫.
        """
        x, y, z, w = quaternion
        # quat(xyzw) → 회전행렬 (scipy 없이 — hot path 의존 최소화)
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        T = np.eye(4)
        T[:3, :3] = R @ self._r0_t
        T[:3, 3] = position
        sol = self._robot.IK(T)
        out: list[list[float]] = []
        for q in sol.Q:
            qn = ((np.asarray(q) + math.pi) % (2.0 * math.pi)) - math.pi
            qc = np.clip(qn, self._lower, self._upper)
            if any(
                np.max(np.abs(qc - np.asarray(prev))) < _DEDUP_ATOL_RAD
                for prev in out
            ):
                continue
            out.append([float(v) for v in qc])
        return out
