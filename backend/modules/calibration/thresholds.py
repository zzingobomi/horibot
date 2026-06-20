"""Hand-Eye 캘리브레이션 튜닝 노브 — 단일 출처.

capture-only 시나리오 (online BA / 추천 / observability 폐기 후) 에 필요한 노브만
유지. 한계 도달하면 여기만 조절하고 백엔드 재시작. 프론트엔드는
`CALIB_HANDEYE_THRESHOLDS` 서비스로 mount 시 fetch.
"""

from __future__ import annotations


# ─── Hand-Eye PnP 품질 gate ──────────────────────────────────────
# solvePnP 직후 reprojection error 임계. 이 이상이면 *capture 자동 reject*.
# trauma source 차단 — ChArUco 코너 일부 가림 / blur / 광량 부족 / board 미세 움직임이
# 만든 안 좋은 PnP 자세를 *애초에 안 들임*. 사용자는 "캡처 거부됨, 더 또렷한 이미지로
# 다시 시도해 주세요" 만 봄 (RMS 숫자 안 보임).
#
# 기준 (D405 1280×720 sub-pixel ChArUco):
#   - 0.5px 이하 = excellent
#   - 1.0px 이하 = nominal
#   - 1.5px 이하 = acceptable (warn — capture 받되 미래 threshold 조정 후보)
#   - 1.5px 초과 = reject (capture 거부)
HANDEYE_PNP_RMS_WARN_PX: float = 1.0
HANDEYE_PNP_RMS_REJECT_PX: float = 1.5


# ─── Phase 1 Traffic Light (실시간 capture-quality) ──────────────
# 현재 pose 를 기존 캡처와 비교해 G/Y/R 판정 (capture_quality.py). 순수 geometry —
# 토크오프 이동 중 "지금 찍어도 좋은 데이터셋이 되나" 실시간 안내. preview loop 가
# 매 프레임 evaluate_capture_quality 호출.

# 기존과 거의 같은 자세 (max per-joint diff < 이 값) + 회전 다양성도 없으면 RED
# ("기존과 거의 동일 — 찍어도 가치 X").
CAPTURE_SIMILAR_JOINT_DEG: float = 6.0
# board orientation (board-in-cam) 의 기존 대비 최소 상대 회전. 이 미만이면
# rotation diversity 부족 → YELLOW ("회전 더 다양하게"). hand-eye 회전 관측의 핵심.
CAPTURE_ROT_DIVERSITY_DEG: float = 12.0
# board position (board-in-cam) 의 기존 대비 최소 거리 (m). 이 미만이면 translation
# diversity 부족 → YELLOW ("거리/위치 더 다양하게"). handeye_trans 관측.
CAPTURE_TRANS_DIVERSITY_M: float = 0.03
# tilt 가 권장 범위 경계에서 이 margin 안이면 YELLOW hint ("조금 더 기울이기").
CAPTURE_TILT_EDGE_MARGIN_DEG: float = 8.0


# ─── tilt 임계 ───────────────────────────────────────────────────
# tilt = 보드 normal vs 카메라 광축 각. 0° = 카메라가 보드 정면 (depth ambiguous),
# 90° = edge-on (corner 픽셀 정확도 ↓). capture-only 시나리오 자리 = 추천 없음 +
# offline BA 가 다양성 흡수 가능. 너무 타이트 (30/70) 면 모서리 자세가 다 빨강
# verdict → 사용자 캡처 안 함. 완화 — 진짜 위험 구간 (depth ambiguous / edge-on
# corner 손실) 만 red.
TILT_MIN_DEG: float = 20.0
TILT_MAX_DEG: float = 75.0


# ─── Intrinsic 캘리브레이션 ─────────────────────────────────────
# RMS reprojection error (pixels). cv2.calibrateCamera 결과.
# GOOD < 0.5px → distortion model 잘 맞춤, USB UVC plumb_bob (5-param) 충분.
# WARN < 1.0px → 일부 모서리에서 잔차 큼. 더 다양한 자세 or distortion model 검토.
# BAD ≥ 1.0px → 모델 불일치 (광각 + plumb_bob 한계) 또는 자세 다양성 부족.
INTRINSIC_RMS_GOOD_PX: float = 0.5
INTRINSIC_RMS_WARN_PX: float = 1.0

# Intrinsic 캡처 권장 수. 저장 가능 최소 5장 (수학적 한계). 권장 10장 (frame
# 9 영역 coverage 가능 + distortion 안정).
INTRINSIC_MIN_CAPTURES: int = 5
INTRINSIC_RECOMMENDED_CAPTURES: int = 10

# Frame 3×3 grid coverage 임계. 보드 중심이 떨어진 grid 셀 개수 ≥ 이 값이면 OK.
# 9 영역 다 채우면 perfect, 7 이상이면 acceptable.
INTRINSIC_GRID_COVERAGE_GOOD: int = 7


def as_dict() -> dict:
    """프론트엔드 service 응답용 직렬화."""
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
