"""Hand-Eye 캘리브레이션 튜닝 노브 — 단일 출처.

capture-only 시나리오에 필요한 노브만. 한계 도달하면 여기만 조절하고 백엔드 재시작.
frontend 는 `get_thresholds` 서비스로 mount 시 fetch.
"""

from __future__ import annotations


# ─── Hand-Eye PnP 품질 gate ──────────────────────────────────────
# solvePnP 직후 reprojection error 임계. 이 이상이면 *capture 자동 reject*.
# ChArUco 코너 일부 가림 / blur / 광량 부족 / board 미세 움직임이 만든 안 좋은 PnP
# 자세를 애초에 안 들임. 사용자는 "캡처 거부됨, 더 또렷한 이미지로" 만 봄.
HANDEYE_PNP_RMS_WARN_PX: float = 1.0
HANDEYE_PNP_RMS_REJECT_PX: float = 1.5


# ─── Phase 1 Traffic Light (실시간 capture-quality) ──────────────
# 현재 pose 를 기존 캡처와 비교해 G/Y/R (capture_quality.py). 순수 geometry.
CAPTURE_SIMILAR_JOINT_DEG: float = 6.0
CAPTURE_ROT_DIVERSITY_DEG: float = 12.0
CAPTURE_TRANS_DIVERSITY_M: float = 0.03
CAPTURE_TILT_EDGE_MARGIN_DEG: float = 8.0


# ─── tilt 임계 ───────────────────────────────────────────────────
# tilt = 보드 normal vs 카메라 광축 각. 0°=정면(depth ambiguous), 90°=edge-on
# (corner 픽셀 정확도 ↓). 진짜 위험 구간만 red.
TILT_MIN_DEG: float = 20.0
TILT_MAX_DEG: float = 75.0


# ─── Intrinsic 캘리브레이션 ─────────────────────────────────────
INTRINSIC_RMS_GOOD_PX: float = 0.5
INTRINSIC_RMS_WARN_PX: float = 1.0
INTRINSIC_MIN_CAPTURES: int = 5
INTRINSIC_RECOMMENDED_CAPTURES: int = 10
INTRINSIC_GRID_COVERAGE_GOOD: int = 7


def as_dict() -> dict:
    """frontend service 응답용 직렬화."""
    return {
        "handeye_pnp_rms_warn_px": HANDEYE_PNP_RMS_WARN_PX,
        "handeye_pnp_rms_reject_px": HANDEYE_PNP_RMS_REJECT_PX,
        "capture_similar_joint_deg": CAPTURE_SIMILAR_JOINT_DEG,
        "capture_rot_diversity_deg": CAPTURE_ROT_DIVERSITY_DEG,
        "capture_trans_diversity_m": CAPTURE_TRANS_DIVERSITY_M,
        "capture_tilt_edge_margin_deg": CAPTURE_TILT_EDGE_MARGIN_DEG,
        "tilt_min_deg": TILT_MIN_DEG,
        "tilt_max_deg": TILT_MAX_DEG,
        "intrinsic_rms_good_px": INTRINSIC_RMS_GOOD_PX,
        "intrinsic_rms_warn_px": INTRINSIC_RMS_WARN_PX,
        "intrinsic_min_captures": INTRINSIC_MIN_CAPTURES,
        "intrinsic_recommended_captures": INTRINSIC_RECOMMENDED_CAPTURES,
        "intrinsic_grid_coverage_good": INTRINSIC_GRID_COVERAGE_GOOD,
    }
