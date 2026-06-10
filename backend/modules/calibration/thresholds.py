"""Hand-Eye 캘리브레이션 튜닝 노브 — 단일 출처.

DIY 5축 + TSDF 목표 컨텍스트라 산업로봇 기본값과 다름. 한계 도달하면 여기만
조절하고 백엔드 재시작. 프론트엔드는 `CALIB_HANDEYE_THRESHOLDS` 서비스로
mount 시 fetch하므로 미러링 불필요.
"""

from __future__ import annotations

# ─── Verdict + UI 색 임계값 ──────────────────────────────────────
# σ_rot / σ_t 가 GOOD 이하 → verdict=good + 초록.  WARN 이하 → needs_work + 노랑.
# 그 이상 → bad + 빨강.
#
# 기준 = **TSDF 최소사양** (DIY 5축, D405 작업거리 ~30cm, voxel 5mm 가정):
#   - GOOD: σ_rot 1° (작업거리에서 ~5mm 변위) + σ_t 10mm → TSDF 깔끔
#   - WARN: σ_rot 2° (~10mm 변위) + σ_t 20mm → TSDF 가능하지만 경계
#   - BAD : 위 초과 → 표면이 흐려져 detector 좌표 정확도 부족
# 산업로봇 정밀도 (0.5° / 5mm)는 외부 정밀 측정 도구가 필요한 영역이라 의도적으로 완화.
SIGMA_ROT_GOOD_DEG: float = 1.0
SIGMA_T_GOOD_MM: float = 10.0
SIGMA_ROT_WARN_DEG: float = 2.0
SIGMA_T_WARN_MM: float = 20.0

# ─── Outlier 자동 제거 ───────────────────────────────────────────
# Iglewicz-Hoaglin modified Z-score:
#     z_i = 0.6745 · (x_i − median) / MAD
#     |z_i| > THRESHOLD → outlier
# 3.5는 paper에서 명시. ASTM E178에서도 권장.
OUTLIER_MOD_Z_THRESHOLD: float = 3.5

# 절대 임계값 — SIGMA_*_GOOD의 1.5배. GOOD 도달 전엔 분포 기반(modified Z)만
# 작동하고, GOOD 근처로 수렴해야 절대 임계가 자(尺)로 작동. 이렇게 안 하면
# σ가 큰 초반 라운드에 잔차 다수가 절대 임계 위로 떠 cap_hit이 항상 발동.
OUTLIER_ABS_ROT_DEG: float = 1.5
OUTLIER_ABS_T_MM: float = 15.0

# 자동 제거 비율 상한. 이 이상 잘리면 outlier 문제가 아니라 BA가 FK floor를
# 흡수 못 한 신호 (자세 다양성 부족) → 자동 제거 중단, coach가 재캡처 가이드.
OUTLIER_REMOVAL_CAP_RATIO: float = 0.20

# ─── 자세 수 / 다양성 ────────────────────────────────────────────
# BA 의 수학적 최소. 이 시점부터 hand_eye 가 한 번 추정됨 → 보드 위치 자동 역산
# (`_estimate_board_base_frame`) → 추천 자세 활성. n<3 단계는 사용자 자유 자세
# (라이브 ChArUco overlay 가 시각 feedback).
MIN_POSES_FOR_COMPUTE: int = 3

# σ trust 임계 — BA 가 의미 있는 σ 를 내놓는 최소 자세 수.
# BA 자유도: standard 9 / extended 20 / physical_sag 22. per-pose residual dof ≈ 6.
# n=3 이면 residual 18 ≈ DOF 와 거의 동등 → BA 가 noise 까지 fit → σ 인위적으로 작아짐
# (사용자에게 false confidence). n=8 면 residual 48 ≈ DOF × 2.2 = trust 가능 영역.
# UI 는 n < MIN_POSES_FOR_TRUSTED_SIGMA 동안 σ 회색 + "신뢰도 낮음" 라벨.
# 추천 활성 임계 (MIN_POSES_FOR_COMPUTE=3) 와 별개 자리 — σ 색깔 표시 전용.
MIN_POSES_FOR_TRUSTED_SIGMA: int = 8

RECOMMENDED_POSES: int = 10

# 5DOF 아암 각 조인트의 std (deg) — 이 미만이면 다양성 부족.
# 캘에 가장 중요한 회전 축은 J1 (base yaw), J4 (wrist pitch), J5 (wrist roll).
# J2/J3는 ee 위치 변화엔 중요하지만 hand-eye 회전 추정엔 덜 중요해 임계값을 낮춤.
JOINT_DIVERSITY_THRESHOLD_DEG: tuple[float, ...] = (25.0, 15.0, 15.0, 25.0, 30.0)

# ─── tilt 임계 ───────────────────────────────────────────────────
# tilt = 보드 normal vs 카메라 광축 각. 0° = 카메라가 보드 정면 (depth ambiguous),
# 90° = edge-on (corner 픽셀 정확도 ↓). docs/calibration_workflow.md §2 권장 범위.
# next_pose_planner 의 visibility gate 와 frontend CheckerboardOverlay 의 캡처
# 가능 임계 둘 다 본 값 사용 — SSOT.
TILT_MIN_DEG: float = 30.0
TILT_MAX_DEG: float = 70.0


def as_dict() -> dict:
    """프론트엔드 service 응답용 직렬화."""
    return {
        "sigma_rot_good_deg": SIGMA_ROT_GOOD_DEG,
        "sigma_t_good_mm": SIGMA_T_GOOD_MM,
        "sigma_rot_warn_deg": SIGMA_ROT_WARN_DEG,
        "sigma_t_warn_mm": SIGMA_T_WARN_MM,
        "outlier_mod_z_threshold": OUTLIER_MOD_Z_THRESHOLD,
        "outlier_abs_rot_deg": OUTLIER_ABS_ROT_DEG,
        "outlier_abs_t_mm": OUTLIER_ABS_T_MM,
        "outlier_removal_cap_ratio": OUTLIER_REMOVAL_CAP_RATIO,
        "min_poses_for_compute": MIN_POSES_FOR_COMPUTE,
        "min_poses_for_trusted_sigma": MIN_POSES_FOR_TRUSTED_SIGMA,
        "recommended_poses": RECOMMENDED_POSES,
        "joint_diversity_threshold_deg": list(JOINT_DIVERSITY_THRESHOLD_DEG),
        "tilt_min_deg": TILT_MIN_DEG,
        "tilt_max_deg": TILT_MAX_DEG,
    }
