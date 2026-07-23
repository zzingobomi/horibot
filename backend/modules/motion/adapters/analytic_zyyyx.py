"""OMX 5축 (Z·YYY·X) 고전 closed-form IK — 해석 branch 열거기.

docs/omx_handover_prep.md §8-2 확정 + docs/motion.md §11 계승: omx_f 는 5축
(J1 yaw(z) · J2·3·4 평행 pitch(y) · J5 roll(x)). 수치 폴백은 "해 놓쳐도 증명
못 함" (흉터 5·6 — walk/restart 보상 기계)이라 고전 기하 분해를 직접 쓴다.

자리 (2026-07-23 실측 정정): 문서 가정과 달리 **EAIK 도 이 5축을 분해한다**
(hasKnownDecomposition=True, FK-roundtrip 완전성 테스트 통과) — 그래서 빌드
순서는 EAIK 우선, 이 솔버는 **EAIK 부재 환경의 결정성 백업** (omx 가 도는
pi_hori3 는 소스빌드라 EAIK 설치 실패 클래스가 실존 — analytic.py optional
경계 주석). 두 경로 모두 같은 대조 테스트(test_motion_analytic_zyyyx)로 잠근다.
계약은 analytic.py(EAIK) 와 동일:

- **snap → 해석 seed → polish**: 축을 최근접 좌표축으로 snap, 미소 lateral
  오프셋(omx tcp y −1.6mm)을 평면으로 snap. 해석해 = seed 생성기 (완전성 계약
  — 모든 branch 열거). 정밀도는 호출자(PybulletKinematics._ik_from_seed 의
  conditional refine)가 실 모델에서 회수.
- branch = [-π,π] 정규화 + **리밋 clamp** (기각 아님) + dedup.
- 부팅 로그 1줄 (침묵 폴백 금지) — try_build 성공/실패 사유를 밝힌다.

기하 분해 (축 부호를 canonical +z/+y/+y/+y/+x 로 정규화한 뒤):
  θ1: 타깃 XY 가 J1 축을 지나는 수직 평면 위에 있어야 함 → atan2 겨냥 2 branch (±π)
  M = Rz(−θ1)·R_target·R0ᵀ = Ry(φ)·Rx(γ)  →  φ(=θ2+θ3+θ4), γ(=θ5) 추출
  손목점 = 타깃 − (o4→tcp 를 φ 만큼 평면 회전)  →  평면 2R (θ2,θ3 elbow ±)
  θ4 = φ − θ2 − θ3
= 최대 4 branch. 5축은 임의 6D pose 에 일반적으로 M ≠ Ry·Rx 꼴 — 그 경우 추출은
best-fit **seed** 일 뿐이고, 자세 오답은 호출자의 자세 잔차 게이트(_ik_analytic
의 _WALK_ORI_TOL)가 기각한다 → "전 branch 기각 = 도달불가 확정" 의미 유지
(top-down 은 제약 수 = 관절 수 = 정확 도달 — omx_handover_prep.md §5.1).
"""

from __future__ import annotations

import logging
import math

import numpy as np

logger = logging.getLogger(__name__)

# 축 snap 상한 — analytic.py 와 동일 근거 (URDF 모델링/캘 스큐 흡수 한도).
_SNAP_MAX_RAD = math.radians(5.0)
# 평면 이탈 상한 — J2..tcp 원점이 J1 수직 평면에서 이보다 벗어나면 이 클래스가
# 아니다 (omx 실측: tcp y −1.6mm — snap 이 흡수하고 polish 가 회수하는 수준).
_PLANE_OFF_MAX_M = 0.010
# tcp 가 J5(roll) 축 위에 있어야 위치가 θ5 와 분리된다 — 축에서 이보다 멀면 폴백.
_WRIST_OFF_MAX_M = 0.010
# 2R 도달 여유 — |cosΔ| 가 1 을 이만큼 넘으면 확정 불가로 branch 생략, 이내면
# 경계 clamp (snap 오차로 경계 밖에 밀린 진짜 해를 polish 가 회수 — analytic.py
# 리밋 clamp 와 같은 철학).
_REACH_EPS = 0.05
_DEDUP_ATOL_RAD = 1e-3
# canonical 축 패턴 — argmax 인덱스 (z, y, y, y, x)
_AXIS_PATTERN = (2, 1, 1, 1, 0)


class ZyyyxAnalyticIk:
    """omx_f 클래스 (Z·YYY·X 5축) 해석적 IK branch 열거기.

    생성은 try_build 로만. 스레드 안전 — 상태는 전부 불변 기하 상수.
    """

    def __init__(
        self,
        signs: list[float],
        p1: np.ndarray,
        s_x: float,
        s_z: float,
        l2: float,
        alpha2: float,
        l3: float,
        alpha3: float,
        l4: float,
        alpha4: float,
        r0: np.ndarray,
        lower: list[float],
        upper: list[float],
    ) -> None:
        self._signs = signs
        self._p1 = p1  # J1 축 앵커 (world zero-pose)
        self._s_x = s_x  # 어깨(J2 원점)의 평면 내 (x, z) — J1 앵커 기준
        self._s_z = s_z
        self._l2, self._a2 = l2, alpha2  # 상완 링크 (길이, zero-pose 평면각)
        self._l3, self._a3 = l3, alpha3  # 전완 링크
        self._l4, self._a4 = l4, alpha4  # 손목(o4)→tcp
        self._r0_t = r0.T.copy()
        self._lower = np.asarray(lower)
        self._upper = np.asarray(upper)
        self.family = "ZYYYX-5R(closed-form)"

    @staticmethod
    def try_build(
        axes: list[tuple[float, float, float]],
        origins: list[tuple[float, float, float]],
        tcp_origin: tuple[float, float, float],
        tcp_rot_zero: np.ndarray,
        lower: list[float],
        upper: list[float],
    ) -> "ZyyyxAnalyticIk | None":
        """zero-pose 월드 축/원점(+tcp) → 해석기. 불가하면 None (사유 로그).

        입력 계약은 analytic.AnalyticIk.try_build 와 동일 — 호출자
        (PybulletKinematics._build_analytic)가 로드된 바디에서 추출.
        """
        if len(axes) != 5:
            return None  # 이 클래스 아님 — 조용히 (EAIK 폴백 로그가 이미 찍힘)
        signs: list[float] = []
        worst_snap = 0.0
        for i, a in enumerate(np.asarray(axes, dtype=float)):
            a = a / np.linalg.norm(a)
            k = int(np.argmax(np.abs(a)))
            if k != _AXIS_PATTERN[i]:
                logger.info(
                    "ZyyyxAnalyticIk: 축 %d 최근접이 %s — 이 URDF 는 Z·YYY·X "
                    "클래스가 아님, 수치 IK 폴백", i + 1, "xyz"[k],
                )
                return None
            signs.append(math.copysign(1.0, a[k]))
            worst_snap = max(
                worst_snap, math.acos(min(1.0, abs(float(a[k]))))
            )
        if worst_snap > _SNAP_MAX_RAD:
            logger.info(
                "ZyyyxAnalyticIk: 축 snap %.2f° > %.1f° — 수치 IK 폴백",
                math.degrees(worst_snap), math.degrees(_SNAP_MAX_RAD),
            )
            return None
        org = np.asarray(origins, dtype=float)
        tcp = np.asarray(tcp_origin, dtype=float)
        p1 = org[0]
        # 평면성: zero pose 의 J2..J4·tcp 는 J1 축을 지나는 y=const 평면 위여야
        off = max(
            abs(float(o[1] - p1[1])) for o in (*org[1:], tcp)
        )
        if off > _PLANE_OFF_MAX_M:
            logger.info(
                "ZyyyxAnalyticIk: 평면 이탈 %.1fmm > %.0fmm — 수치 IK 폴백",
                off * 1000, _PLANE_OFF_MAX_M * 1000,
            )
            return None
        # tcp 가 J5(±x) 축 위 — 위치와 θ5 의 분리 조건
        w = tcp - org[4]
        wrist_off = math.hypot(float(w[1]), float(w[2]))
        if wrist_off > _WRIST_OFF_MAX_M:
            logger.info(
                "ZyyyxAnalyticIk: tcp 가 J5 축에서 %.1fmm — 수치 IK 폴백",
                wrist_off * 1000,
            )
            return None
        # 평면 (x, z) 기하 — 전부 J1 앵커 기준
        s_x = float(org[1][0] - p1[0])
        s_z = float(org[1][2] - p1[2])
        a2 = (float(org[2][0] - org[1][0]), float(org[2][2] - org[1][2]))
        a3 = (float(org[3][0] - org[2][0]), float(org[3][2] - org[2][2]))
        a4 = (float(tcp[0] - org[3][0]), float(tcp[2] - org[3][2]))
        l2, l3 = math.hypot(*a2), math.hypot(*a3)
        if l2 < 1e-6 or l3 < 1e-6:
            logger.info("ZyyyxAnalyticIk: 축퇴 링크 (L2/L3≈0) — 수치 IK 폴백")
            return None
        ik = ZyyyxAnalyticIk(
            signs=signs,
            p1=p1,
            s_x=s_x,
            s_z=s_z,
            l2=l2,
            alpha2=math.atan2(a2[1], a2[0]),
            l3=l3,
            alpha3=math.atan2(a3[1], a3[0]),
            l4=math.hypot(*a4),
            alpha4=math.atan2(a4[1], a4[0]),
            r0=np.asarray(tcp_rot_zero, dtype=float),
            lower=list(lower),
            upper=list(upper),
        )
        logger.info(
            "ZyyyxAnalyticIk 활성: family=%s, 축 snap 최대 %.2f°, 평면 이탈 "
            "%.1fmm", ik.family, math.degrees(worst_snap), off * 1000,
        )
        return ik

    def branches(
        self,
        position: tuple[float, float, float],
        quaternion: tuple[float, float, float, float],
    ) -> list[list[float]]:
        """목표 pose 의 모든 IK branch (정규화+리밋 clamp+dedup, 순서 무보장).

        5축이라 정확 도달이 안 되는 pose 도 best-fit seed 를 낸다 — 채택/기각은
        호출자 polish + 자세 잔차 게이트 몫 (모듈 docstring).
        """
        x, y, z, w = quaternion
        r = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        r_eff = r @ self._r0_t
        dx = float(position[0] - self._p1[0])
        dy = float(position[1] - self._p1[1])
        qz = float(position[2] - self._p1[2])
        base_yaw = math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-9 else 0.0
        out: list[list[float]] = []
        for th1 in (base_yaw, base_yaw + math.pi):
            c1, s1 = math.cos(th1), math.sin(th1)
            qx = c1 * dx + s1 * dy  # 평면 내 반경 좌표 (branch B 는 음수)
            # M = Rz(−θ1)·R_eff — Ry(φ)·Rx(γ) 꼴에서 φ/γ 추출
            m = np.array([
                [c1, s1, 0.0], [-s1, c1, 0.0], [0.0, 0.0, 1.0]
            ]) @ r_eff
            phi = math.atan2(-float(m[2, 0]), float(m[0, 0]))
            gamma = math.atan2(-float(m[1, 2]), float(m[1, 1]))
            # 손목점 = 타깃 − (o4→tcp 를 φ 회전) − 어깨 (평면 2R 입력)
            vx = self._l4 * math.cos(self._a4 - phi)
            vz = self._l4 * math.sin(self._a4 - phi)
            wx = qx - self._s_x - vx
            wz = qz - self._s_z - vz
            d = (wx * wx + wz * wz - self._l2**2 - self._l3**2) / (
                2.0 * self._l2 * self._l3
            )
            if abs(d) > 1.0 + _REACH_EPS:
                continue  # 확정 밖 — 이 θ1 branch 에서 2R 불가
            d = max(-1.0, min(1.0, d))
            for elbow in (1.0, -1.0):
                delta = elbow * math.acos(d)
                th3 = (self._a3 - self._a2) - delta
                a = math.atan2(wz, wx) - math.atan2(
                    self._l3 * math.sin(delta),
                    self._l2 + self._l3 * math.cos(delta),
                )
                th2 = self._a2 - a
                th4 = phi - th2 - th3
                th5 = gamma
                q = np.array([th1, th2, th3, th4, th5]) * np.asarray(self._signs)
                qn = ((q + math.pi) % (2.0 * math.pi)) - math.pi
                qc = np.clip(qn, self._lower, self._upper)
                if any(
                    np.max(np.abs(qc - np.asarray(prev))) < _DEDUP_ATOL_RAD
                    for prev in out
                ):
                    continue
                out.append([float(v) for v in qc])
        return out
