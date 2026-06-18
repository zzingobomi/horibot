"""실시간 capture-quality 판정 = Phase 1 Traffic Light (스펙 MVP1).

현재 라이브 pose 를 *기존 캡처 데이터셋* 과 비교해 GREEN / YELLOW / RED 판정.
순수 geometry (solver 무관) — checkerboard 검출 + tilt + pose/rotation/translation
diversity. 사용자가 토크오프로 자세 잡는 *동안* "지금 찍으면 좋은 데이터셋이 되나"
를 실시간(preview loop 5Hz)으로 안내 (docs/handeye_ux_solver_v3_plan.md §5).

스펙 Phase 1 Traffic Light:
  🟢 GREEN  : 검출 + tilt 충분 + 기존과 충분히 다른 pose + 새 viewpoint → 캡처 권장
  🟡 YELLOW : 캡처는 가능하나 개선 — more tilt / more rotation / more translation
  🔴 RED    : 미검출(lost) / tilt 부족·과다 / 기존과 너무 유사 (다양성 없음)

verdict 는 backend 가 계산 — frontend 는 색+사유만 표시 (수치 노출 X, 기존 철학).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from . import thresholds as T


@dataclass
class CaptureQuality:
    verdict: str  # "green" | "yellow" | "red"
    reasons: list[str] = field(default_factory=list)  # 사용자 안내 (한국어 짧게)
    # 진단 수치 (frontend 미노출, 디버그/테스트용).
    min_rot_deg: float | None = None
    min_trans_m: float | None = None
    min_joint_deg: float | None = None

    def to_dict(self) -> dict:
        return {
            "verdict": self.verdict,
            "reasons": list(self.reasons),
            "min_rot_deg": self.min_rot_deg,
            "min_trans_m": self.min_trans_m,
            "min_joint_deg": self.min_joint_deg,
        }


def _rot_diff_deg(Ra: np.ndarray, Rb: np.ndarray) -> float:
    R = Ra @ Rb.T
    cos = (np.trace(R) - 1.0) * 0.5
    return float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))))


def evaluate_capture_quality(
    *,
    detected: bool,
    tilt_deg: float | None,
    current_joints_rad: list[float] | None,
    current_R_t2c: np.ndarray | None,
    current_t_t2c: np.ndarray | None,
    existing_joints_rad: list[list[float]],
    existing_R_t2c: list[np.ndarray],
    existing_t_t2c: list[np.ndarray],
) -> CaptureQuality:
    """현재 pose 의 실시간 capture-quality 판정.

    Args:
        detected: ChArUco 검출 여부.
        tilt_deg: board tilt (PnP 필요). None = PnP 불가 (intrinsic 없음 등).
        current_joints_rad: 현재 arm joint (rad). diversity (joint variation) 용.
        current_R_t2c / current_t_t2c: 현재 board-in-cam 회전/이동 (PnP). diversity 용.
        existing_*: 기존 캡처들의 joint / R / t (같은 단위).
    """
    # 1. 검출 / tilt — RED gate
    if not detected:
        return CaptureQuality("red", ["보드 미검출"])
    if tilt_deg is None:
        return CaptureQuality("red", ["자세 추정 불가 (intrinsic 확인)"])
    if tilt_deg < T.TILT_MIN_DEG:
        return CaptureQuality("red", ["tilt 부족 (너무 정면)"])
    if tilt_deg > T.TILT_MAX_DEG:
        return CaptureQuality("red", ["tilt 과다 (너무 비스듬)"])

    # 2. 첫 자세 — 비교 대상 없음 → 검출+tilt OK 면 GREEN
    if not existing_joints_rad:
        return CaptureQuality("green", ["첫 자세 — 캡처 권장"])

    # 3. diversity 계산 (기존과 가장 유사한 것 기준)
    min_joint_deg: float | None = None
    if current_joints_rad is not None:
        per = [
            max(
                abs(c - e)
                for c, e in zip(current_joints_rad, ej)
            )
            for ej in existing_joints_rad
            if len(ej) == len(current_joints_rad)
        ]
        if per:
            min_joint_deg = float(np.degrees(min(per)))

    min_rot_deg: float | None = None
    min_trans_m: float | None = None
    if current_R_t2c is not None and existing_R_t2c:
        min_rot_deg = min(_rot_diff_deg(current_R_t2c, R) for R in existing_R_t2c)
    if current_t_t2c is not None and existing_t_t2c:
        ct = np.asarray(current_t_t2c, dtype=np.float64).reshape(3)
        min_trans_m = min(
            float(np.linalg.norm(ct - np.asarray(t, dtype=np.float64).reshape(3)))
            for t in existing_t_t2c
        )

    diag = dict(
        min_rot_deg=min_rot_deg, min_trans_m=min_trans_m, min_joint_deg=min_joint_deg
    )

    # 4. 기존과 거의 동일 → RED (찍어도 가치 없음)
    too_similar_joint = (
        min_joint_deg is not None and min_joint_deg < T.CAPTURE_SIMILAR_JOINT_DEG
    )
    weak_rot = min_rot_deg is not None and min_rot_deg < T.CAPTURE_ROT_DIVERSITY_DEG
    if too_similar_joint and (min_rot_deg is None or weak_rot):
        return CaptureQuality("red", ["기존과 거의 같은 자세"], **diag)

    # 5. YELLOW — 캡처 가능하나 특정 diversity 축 부족
    hints: list[str] = []
    if weak_rot:
        hints.append("회전 더 다양하게")
    if min_trans_m is not None and min_trans_m < T.CAPTURE_TRANS_DIVERSITY_M:
        hints.append("거리/위치 더 다양하게")
    if tilt_deg < T.TILT_MIN_DEG + T.CAPTURE_TILT_EDGE_MARGIN_DEG:
        hints.append("조금 더 기울이기")
    elif tilt_deg > T.TILT_MAX_DEG - T.CAPTURE_TILT_EDGE_MARGIN_DEG:
        hints.append("조금 덜 기울이기")
    if hints:
        return CaptureQuality("yellow", hints, **diag)

    # 6. GREEN
    return CaptureQuality("green", ["캡처 권장 — 새 자세 + 새 시야"], **diag)
