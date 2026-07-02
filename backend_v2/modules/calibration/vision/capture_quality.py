"""실시간 capture-quality 판정 = Phase 1 Traffic Light.

현재 라이브 pose 를 *기존 캡처 데이터셋* 과 비교해 GREEN / YELLOW / RED.
순수 geometry — ChArUco 검출 + tilt + pose/rotation/translation diversity.
verdict 는 backend 계산, frontend 는 색+사유만 표시 (수치 미노출).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from . import thresholds as T


@dataclass
class CaptureQuality:
    verdict: str  # "green" | "yellow" | "red"
    reasons: list[str] = field(default_factory=list)
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
    """현재 pose 의 실시간 capture-quality 판정."""
    # 1. 검출 / tilt — RED gate
    if not detected:
        return CaptureQuality("red", ["보드 미검출"])
    if tilt_deg is None:
        return CaptureQuality("red", ["자세 추정 불가 (intrinsic 확인)"])
    if tilt_deg < T.TILT_MIN_DEG:
        return CaptureQuality("red", ["tilt 부족 (너무 정면)"])
    if tilt_deg > T.TILT_MAX_DEG:
        return CaptureQuality("red", ["tilt 과다 (너무 비스듬)"])

    # 2. 첫 자세 — 비교 대상 없음
    if not existing_joints_rad:
        return CaptureQuality("green", ["첫 자세 — 캡처 권장"])

    # 3. diversity (기존과 가장 유사한 것 기준)
    min_joint_deg: float | None = None
    if current_joints_rad is not None:
        per = [
            max(abs(c - e) for c, e in zip(current_joints_rad, ej))
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

    # 4. 기존과 거의 동일 → RED
    too_similar_joint = (
        min_joint_deg is not None and min_joint_deg < T.CAPTURE_SIMILAR_JOINT_DEG
    )
    weak_rot = min_rot_deg is not None and min_rot_deg < T.CAPTURE_ROT_DIVERSITY_DEG
    if too_similar_joint and (min_rot_deg is None or weak_rot):
        return CaptureQuality("red", ["기존과 거의 같은 자세"], **diag)

    # 5. YELLOW — 특정 diversity 축 부족
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
