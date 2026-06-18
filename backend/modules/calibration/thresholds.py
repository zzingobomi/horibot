"""Hand-Eye 캘리브레이션 튜닝 노브 — 단일 출처.

DIY 5축 + TSDF 목표 컨텍스트라 산업로봇 기본값과 다름. 한계 도달하면 여기만
조절하고 백엔드 재시작. 프론트엔드는 `CALIB_HANDEYE_THRESHOLDS` 서비스로
mount 시 fetch하므로 미러링 불필요.
"""

from __future__ import annotations

import math

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

# Arm 각 조인트의 std (deg) — 이 미만이면 다양성 부족. caller 의 arm DOF 길이에
# 맞춰 사용 (joint_distribution.analyze 는 i >= len 일 때 fallback 15.0).
# 캘에 가장 중요한 회전 축은 J1 (base yaw), J4 (wrist pitch), J5 (wrist roll).
# J2/J3 는 ee 위치 변화엔 중요하지만 hand-eye 회전 추정엔 덜 중요해 임계값을 낮춤.
# omx_f (5DOF) 기준 J1~J5. so101_6dof (6DOF) 의 J6 (wrist yaw) 는 fallback 15.0
# 적용 — 별도 튜닝 시점에 robot type 별 분리 (multi_robot Phase 3).
JOINT_DIVERSITY_THRESHOLD_DEG: tuple[float, ...] = (25.0, 15.0, 15.0, 25.0, 30.0)

# ─── Hand-Eye PnP 품질 gate ──────────────────────────────────────
# solvePnP 직후 reprojection error 임계. 이 이상이면 *capture 자동 reject*.
# trauma source 차단 — ChArUco 코너 일부 가림 / blur / 광량 부족 / board 미세 움직임이
# 만든 안 좋은 PnP 자세를 *애초에 안 들임*. 사용자는 "캡처 거부됨, 더 또렷한 이미지로
# 다시 시도해 주세요" 만 봄 (RMS 숫자 안 보임).
#
# 기준 (D405 1280×720 sub-pixel ChArUco 기준):
#   - 0.5px 이하 = excellent
#   - 1.0px 이하 = nominal
#   - 1.5px 이하 = acceptable (warn 자리 — capture 받되 미래 threshold 조정 후보)
#   - 1.5px 초과 = reject (capture 거부)
HANDEYE_PNP_RMS_WARN_PX: float = 1.0
HANDEYE_PNP_RMS_REJECT_PX: float = 1.5


# ─── Phase 1 Traffic Light (실시간 capture-quality) ──────────────
# 현재 pose 를 기존 캡처와 비교해 G/Y/R 판정 (handeye_ux_solver_v3_plan.md §5,
# 스펙 MVP1). 순수 geometry — 토크오프 이동 중 "지금 찍어도 좋은 데이터셋이 되나"
# 실시간 안내. preview loop 가 매 프레임 evaluate_capture_quality 호출.
#
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
CAPTURE_TILT_EDGE_MARGIN_DEG: float = 5.0


# ─── tilt 임계 ───────────────────────────────────────────────────
# tilt = 보드 normal vs 카메라 광축 각. 0° = 카메라가 보드 정면 (depth ambiguous),
# 90° = edge-on (corner 픽셀 정확도 ↓). docs/calibration_workflow.md §2 권장 범위.
# next_pose_planner 의 visibility gate 와 frontend CheckerboardOverlay 의 캡처
# 가능 임계 둘 다 본 값 사용 — SSOT.
TILT_MIN_DEG: float = 30.0
TILT_MAX_DEG: float = 70.0

# ─── 추천 자세 sphere shell geometry ────────────────────────────
# next_pose_planner.recommend_geometry 가 보드 중심 주변에 anchor 5개 (정면/좌/우/
# 위/아래) 를 sphere shell 위에 배치. distance = 카메라 ↔ 보드 거리, side_offset =
# 정면 외 anchor 의 측면 변위. 작업대 (55×34cm) + D405 sweet spot 10-25cm 기준.
# 0.18m = 보드 ↔ wrist 가능 거리 중간값 (사용자 setup: 보드 x=240mm + wrist 가능
# x=40-160mm → 거리 80-200mm 범위 → 중간 ~18cm).
RECOMMEND_DISTANCE_M: float = 0.18
# side_offset 0.10 은 anchor 가 OMX-F (reach ~25cm) 작업공간 끝으로 가서 IK 풀림이
# 5개 중 1개만 성공 (audit 결과). 0.05 로 줄여 작업공간 안쪽 sphere shell. 사용자가
# 받는 추천이 거의 항상 empty 였던 trauma source.
RECOMMEND_SIDE_OFFSET_M: float = 0.05

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


# ─── Per-parameter observability (Fisher/Jacobian 식별성) ─────────
# docs/handeye_ux_solver_v3_plan.md §3. BA data residual 의 Jacobian 에서 블록별
# 식별성 score ∈ [0,1] 산출 → capture 개수가 아니라 식별성으로 BA 블록 unlock gating.
#
# 파라미터 정규화 nominal — Jacobian 컬럼을 "1 nominal 만큼 흔들면 residual 얼마
# 변하나" 로 무차원화 (블록 간 비교 가능). 단위 통일용이라 절대값보다 *상대 비율* 이
# score 를 지배 (degeneracy = collinear → score≈0 는 scale 무관).
OBS_SCALE_ANGLE_RAD: float = math.radians(1.0)  # joint_offset / link_rot / handeye rod
OBS_SCALE_TRANS_M: float = 0.001  # link_trans / handeye t (1mm)
OBS_SCALE_SAG_K: float = 0.1  # sag_k (rad / (m·g_unit))

# 측정 노이즈 바닥값 — Fisher 정보의 residual 행 정규화 (rot 행/σ_rot, trans 행/σ_trans).
# *fitted* residual 이 아니라 *고정* 노이즈 floor 여야 함: 포즈 적으면 overfit 으로
# residual→0 이지만 측정 노이즈는 그대로 → 정보 적음 → observability 낮게 (정상).
# D405 + ChArUco PnP 의 per-pose 자세 노이즈 추정.
OBS_NOISE_ROT_RAD: float = math.radians(0.5)
OBS_NOISE_TRANS_M: float = 0.003

# 블록별 unlock 임계 — score ≥ 임계면 BA 가 해당 블록 추정, 아니면 freeze (prior 0).
# handeye(R,t) 는 항상 unlock (캘 1차 목적). joint / link / sag 만 gate.
#
# **gating = 안전망** (handeye_ux_solver_v3_plan.md §3.2 measurement 결과):
# physical_sag 모델에선 현실적 데이터(자세 몇 개라도 다양)면 EE 위치 변화로 sag 까지
# 거의 항상 관측 가능 (실데이터: joint 0.22 / link 0.23 / sag 0.66, 전부 unlock = 현
# always-on BA 와 동일 → 무회귀). 임계를 안전망 수준 0.05 로 — 자세가 거의 동일하거나
# 극소수라 *정보가 사실상 없는* 병리적 블록만 freeze (잘못된 흡수 차단).
OBS_UNLOCK_JOINT: float = 0.05
OBS_UNLOCK_LINK: float = 0.05
OBS_UNLOCK_SAG: float = 0.05

# verdict 표시 (UI 안내) — gate 와 별개 band. score ≥ OK → "잘 잡힘", WEAK band →
# "보강 권장", < WEAK(=unlock) → "정보 부족 (freeze)".
# score = posterior 분산이 prior 대비 줄어든 비율 (information gain). 8 포즈 + tight
# prior 라 good data 도 블록별 0.08~0.6 — OK 임계는 measurement 기반 0.10
# (handeye_ux_solver_v3_plan.md §3.2).
OBS_VERDICT_OK: float = 0.10
OBS_VERDICT_WEAK: float = 0.05


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
        "handeye_pnp_rms_warn_px": HANDEYE_PNP_RMS_WARN_PX,
        "handeye_pnp_rms_reject_px": HANDEYE_PNP_RMS_REJECT_PX,
        "tilt_min_deg": TILT_MIN_DEG,
        "tilt_max_deg": TILT_MAX_DEG,
        "recommend_distance_m": RECOMMEND_DISTANCE_M,
        "recommend_side_offset_m": RECOMMEND_SIDE_OFFSET_M,
        "intrinsic_rms_good_px": INTRINSIC_RMS_GOOD_PX,
        "intrinsic_rms_warn_px": INTRINSIC_RMS_WARN_PX,
        "intrinsic_min_captures": INTRINSIC_MIN_CAPTURES,
        "intrinsic_recommended_captures": INTRINSIC_RECOMMENDED_CAPTURES,
        "intrinsic_grid_coverage_good": INTRINSIC_GRID_COVERAGE_GOOD,
    }
