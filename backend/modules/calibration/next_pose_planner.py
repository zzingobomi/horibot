"""반자동 Hand-Eye 다음 자세 후보 리스트 — [계산] 응답에 묶임.

추천의 본질:
    BA가 11개 자유도(joint_offset 5 + rod 3 + t 3) 안에서 최선을 짜냈는데도
    σ가 목표 미달이면 = "현재 데이터로는 여기까지". 그래서 추천 = 사용자에게
    *데이터 보완 요청*. 임의 자세가 아니라 *지금 추정의 약점이 드러나는 방향*
    이어야 한 라운드 추가로 σ가 실제로 줄어듦.

추천 = "한 점"이 아니라 "후보 리스트":
    각 후보가 정말 캡처 가능한지(체커보드 시야 안에 들어오는지)는 사용자만
    안다. 그래서 N개를 주고 사용자가 [이동] → 카메라 보고 보이는 것만 [캡처].
    안 보이면 다음 후보로. 가시성 판정은 사람 눈이 정확.

후보 생성:
    1) 잔차 큰 포즈(BA per-pose drot ≥ 임계) → 그 영역 J1/J4/J5 ±변주
       각 포즈당 최대 2개. 잔차 큰 포즈 우선.
    2) 분포 fallback (잔차 큰 포즈 부족할 때 채움)
       — joint_distribution이 가리킨 빈 축 1개당 1후보
    dedupe: 모든 축이 다른 후보와 5° 이내면 중복으로 보고 제외
    cap: 총 MAX_RECOMMENDATIONS개

향후 (H 강화 단계):
    BA의 최종 Jacobian (scipy result.jac)에서 H = J^T J 구성 → 가장 불확실한
    방향(eigenvector) 추정 → 후보 자세 sampling 후 H 업데이트 시 최소
    eigenvalue를 가장 크게 늘리는 자세 선택. 100~200줄, 외부 라이브러리 X.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from math import degrees, radians
from typing import Any, Callable, Protocol, Sequence

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from . import thresholds

logger = logging.getLogger(__name__)


VisibilityCheck = Callable[[list[float]], tuple[bool, str]]
"""후보 joint angles → (visible, reason). visibility_check 인자."""

# 후보 리스트 최대 길이. 너무 많으면 사용자 피로, 너무 적으면 다 가도 안 보일 수 있음.
MAX_RECOMMENDATIONS: int = 6

# 중복 판정 임계 (rad). 두 후보의 모든 축 차이가 이 이내면 같은 자세로 봄.
_DEDUPE_TOLERANCE_RAD: float = radians(5.0)


@dataclass
class NextPoseRecommendation:
    joints: list[dict]  # [{id, degree}] — motion/move_j 페이로드와 정렬
    reason: str  # 긴 설명 — 행 펼침 시 노출
    label: str  # 짧은 한 줄 — 리스트 행 헤드라인. "J4 위쪽 +25°" 형식.
    primary_axis: int  # 0..4 (어느 축이 주요 변경)
    source: str  # "high_residual" | "distribution"
    diagnostics: dict = field(default_factory=dict)
    visible: bool = True  # visibility_check 통과 여부 (기본 True — check 안 했으면 미상)
    visibility_reason: str = "unchecked"


# 추천 후보가 비었을 때 그 이유를 *분기별 분리*. UI 가 사용자에게 명확히 안내하도록.
# planner 단독으로 알 수 있는 분기만 다룸 — σ/verdict 기반 분기 (sigma_sufficient_*)
# 는 caller (calibration_node) 가 결정해서 덮어씀.
NoCandidatesReason = str
NO_REASON_BOARD_ESTIMATE_MISSING: NoCandidatesReason = "no_board_estimate"
NO_REASON_ALL_IK_FAIL: NoCandidatesReason = "all_ik_fail"
NO_REASON_ALL_INVISIBLE: NoCandidatesReason = "all_invisible"
NO_REASON_USER_MARKED_FAIL: NoCandidatesReason = "user_marked_fail"


def recommend_axis_diversify(
    *,
    current_joint_angles_rad: Sequence[float],
    arm_motor_ids: list[int],
    joint_limits_rad: list[tuple[float, float]],
    weak_axis_motor_ids: list[int],
    weak_axis_suggested_deg: dict[int, float | None],
    visibility_check: VisibilityCheck | None = None,
    excluded_ids: set[str] | None = None,
) -> list[NextPoseRecommendation]:
    """약한 axis 보충 후보 — current 자세에서 그 축만 suggested_deg 로 점프.

    Hybrid 자리: GeometryStrategy 의 sphere shell anchor 가 보드 주변 카메라
    위치 다양성 자리는 잡지만, *어느 축이 좁다* 정보 (coach.axis_distributions)
    는 안 받음. narrow_sigma_good 분기에서 약한 axis 별 1 후보를 추가해 사용자가
    그 자세로 ghost 따라가 캡처할 수 있게 함.

    "current 위 그 축만 점프" 자세는 보드 가시 영역에 가까우니 visibility 통과율
    높음. weak axis 별 1개씩 = 보통 1~3개 후보 추가.
    """
    out: list[NextPoseRecommendation] = []
    n_axes = min(len(arm_motor_ids), len(joint_limits_rad))
    excluded = excluded_ids or set()

    for motor_id in weak_axis_motor_ids:
        suggested_deg = weak_axis_suggested_deg.get(motor_id)
        if suggested_deg is None:
            continue
        try:
            axis_idx = arm_motor_ids.index(motor_id)
        except ValueError:
            continue
        if axis_idx >= n_axes or len(current_joint_angles_rad) < n_axes:
            continue

        rec_id = f"axis_diversify_{motor_id}"
        if rec_id in excluded:
            continue

        target = list(current_joint_angles_rad[:n_axes])
        target[axis_idx] = radians(suggested_deg)

        lo, hi = joint_limits_rad[axis_idx]
        if not (lo <= target[axis_idx] <= hi):
            continue

        visible = True
        v_reason = "unchecked"
        if visibility_check is not None:
            visible, v_reason = visibility_check(target)

        out.append(
            NextPoseRecommendation(
                joints=[
                    {"id": int(mid), "degree": float(degrees(ang))}
                    for mid, ang in zip(arm_motor_ids[:n_axes], target)
                ],
                reason=(
                    f"J{motor_id} 다양성 보충 — 현재 자세에서 J{motor_id} 만 "
                    f"{suggested_deg:+.0f}° 로 이동. 다른 축은 현재 유지."
                ),
                label=f"J{motor_id} 다양성 보충 → {suggested_deg:+.0f}°",
                primary_axis=axis_idx,
                source="axis_diversify",
                diagnostics={
                    "mode": "axis_diversify",
                    "motor_id": motor_id,
                    "suggested_deg": suggested_deg,
                },
                visible=visible,
                visibility_reason=v_reason,
            )
        )
    return out


@dataclass
class RecommendationResult:
    recommendations: list[NextPoseRecommendation]
    # 빈 list 일 때만 채워짐. caller 가 σ/verdict 결합해 덮어쓸 수 있음.
    no_candidates_reason: NoCandidatesReason | None = None



def is_pose_visible(
    joint_angles_rad: list[float],
    *,
    fk_fn: Callable[[list[float]], tuple[Any, Any]],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    image_size: tuple[int, int],
    hand_eye_R: np.ndarray,
    hand_eye_t: np.ndarray,
    board_corners_base: np.ndarray,
    margin: float = 0.05,
) -> tuple[bool, str]:
    """후보 자세에서 ChArUco 보드 4 외곽 코너가 카메라 frame 의 margin 영역 안인지.

    cam2base = gripper2base · cam2gripper (hand_eye) → 그 역으로 base-frame 보드
    코너를 cam frame 으로 → cv2.projectPoints → image frame. 4 코너 모두 margin
    안이고 모두 카메라 앞 (z > epsilon) 이면 visible.

    margin: 화면 가장자리 안전 영역 비율 (0.05 = 5%). 검출 신뢰성 위해 살짝 안쪽.
    """
    try:
        R_g2b, t_g2b = fk_fn(joint_angles_rad)
    except Exception as e:
        return False, f"FK 실패: {e}"

    T_g2b = np.eye(4)
    T_g2b[:3, :3] = np.asarray(R_g2b)
    T_g2b[:3, 3] = np.asarray(t_g2b).reshape(3)
    T_c2g = np.eye(4)
    T_c2g[:3, :3] = hand_eye_R
    T_c2g[:3, 3] = np.asarray(hand_eye_t).reshape(3)
    T_c2b = T_g2b @ T_c2g
    T_b2c = np.linalg.inv(T_c2b)

    homo = np.hstack(
        [board_corners_base, np.ones((board_corners_base.shape[0], 1))]
    )
    corners_cam = (T_b2c @ homo.T).T[:, :3]

    if np.any(corners_cam[:, 2] <= 0.01):
        return False, "보드가 카메라 뒤"

    img_pts, _ = cv2.projectPoints(
        corners_cam.reshape(-1, 1, 3),
        np.zeros(3),
        np.zeros(3),
        camera_matrix,
        dist_coeffs,
    )
    pts = img_pts.reshape(-1, 2)
    w, h = image_size
    margin_x = w * margin
    margin_y = h * margin

    if not np.all((pts[:, 0] >= margin_x) & (pts[:, 0] <= w - margin_x)):
        return False, "좌우 화면 벗어남"
    if not np.all((pts[:, 1] >= margin_y) & (pts[:, 1] <= h - margin_y)):
        return False, "상하 화면 벗어남"

    # tilt gate — 추천 자세에서 보드 normal vs 카메라 광축 각이 PnP 권장
    # 범위 (thresholds.TILT_MIN_DEG ~ TILT_MAX_DEG) 안인지. 밖이면 사용자가
    # 그 자세로 [이동] 해도 "캡처 금지" 떨어져서 헛걸음 — 미리 회색 마크.
    v1 = board_corners_base[1] - board_corners_base[0]
    v2 = board_corners_base[3] - board_corners_base[0]
    n_base = np.cross(v1, v2)
    n_norm = np.linalg.norm(n_base)
    if n_norm > 1e-9:
        n_base = n_base / n_norm
        n_cam = T_b2c[:3, :3] @ n_base
        cos_tilt = float(np.clip(abs(n_cam[2]), 0.0, 1.0))
        tilt_deg = float(np.degrees(np.arccos(cos_tilt)))
        if tilt_deg < thresholds.TILT_MIN_DEG:
            return False, f"tilt {tilt_deg:.0f}° 너무 정면"
        if tilt_deg > thresholds.TILT_MAX_DEG:
            return False, f"tilt {tilt_deg:.0f}° 너무 비스듬"

    return True, "visible"



def recommend_geometry(
    *,
    board_corners_base: np.ndarray,
    ik_fn: Callable[
        [
            tuple[float, float, float],
            tuple[float, float, float, float] | None,
            Sequence[float] | None,
        ],
        list[float] | None,
    ],
    hand_eye_R: np.ndarray,
    hand_eye_t: np.ndarray,
    arm_motor_ids: list[int],
    joint_limits_rad: list[tuple[float, float]],
    current_joint_angles_rad: Sequence[float] | None = None,
    distance_m: float | None = None,
    side_offset_m: float | None = None,
    outward_hint: np.ndarray | None = None,
    visibility_check: VisibilityCheck | None = None,
    excluded_ids: set[str] | None = None,
) -> RecommendationResult:
    """보드 sphere shell anchor 기반 EE pose IK sampling.

    Anchor 5개 (정면 / 좌측 / 우측 / 위쪽 / 아래쪽) — 보드 위 distance sphere shell.
    각 anchor 에서 camera z 축 → board center 향함 (look-at). inv(hand_eye) 로
    EE pose 자체 derive → IK 풀어 joint angles. fail (IK 풀림 X / joint limits 밖 /
    visibility fail) drop.

    Args:
        outward_hint: 보드 normal 의 + 방향을 결정하는 hint vector. 보통 카메라
            평균 위치 - 보드 중심. None 이면 fallback 으로 +Z 가정 (구 동작, 보드가
            책상에 누워있는 경우만 정상). 보드 수직/기울임 setup 에선 반드시 hint
            전달해야 anchor 가 *카메라가 있던 쪽* 으로 생성됨.
        distance_m / side_offset_m: None 이면 thresholds SSOT 사용.
        excluded_ids: 사용자가 명시 신호 ([👎]) 로 fail 표시한 추천 ID.

    Returns:
        RecommendationResult — recommendations + no_candidates_reason (빈 list 일 때).
    """
    if distance_m is None:
        distance_m = thresholds.RECOMMEND_DISTANCE_M
    if side_offset_m is None:
        side_offset_m = thresholds.RECOMMEND_SIDE_OFFSET_M

    if board_corners_base.shape != (4, 3):
        return RecommendationResult([], NO_REASON_BOARD_ESTIMATE_MISSING)

    board_center = board_corners_base.mean(axis=0)
    v1 = board_corners_base[1] - board_corners_base[0]
    v2 = board_corners_base[3] - board_corners_base[0]
    board_normal = np.cross(v1, v2)
    norm_n = np.linalg.norm(board_normal)
    if norm_n < 1e-9:
        return RecommendationResult([], NO_REASON_BOARD_ESTIMATE_MISSING)
    board_normal = board_normal / norm_n

    # 보드 normal 부호 — outward_hint (카메라가 있던 방향) 가 있으면 그 쪽 향하게.
    # 옛 동작 (board_normal[2] < 0 뒤집기) 은 *보드가 책상에 누워있다* 가정이라
    # 보드 수직/기울임 setup 에서 anchor 가 로봇 반대편 공중으로 생성되는 bug 원인.
    if outward_hint is not None:
        hint = np.asarray(outward_hint, dtype=float).reshape(3)
        if np.linalg.norm(hint) > 1e-9 and np.dot(board_normal, hint) < 0:
            board_normal = -board_normal
    else:
        # fallback: +Z 가정 (보드 누워있는 setup 만 정상)
        if board_normal[2] < 0:
            board_normal = -board_normal

    world_up = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(board_normal, world_up)) > 0.95:
        tangent_x = np.array([1.0, 0.0, 0.0])
    else:
        tangent_x = np.cross(world_up, board_normal)
        tangent_x = tangent_x / max(np.linalg.norm(tangent_x), 1e-9)
    tangent_y = np.cross(board_normal, tangent_x)
    tangent_y = tangent_y / max(np.linalg.norm(tangent_y), 1e-9)

    anchors: list[tuple[str, np.ndarray]] = [
        ("정면", board_center + distance_m * board_normal),
        (
            "좌측",
            board_center + distance_m * board_normal * 0.85 + side_offset_m * tangent_x,
        ),
        (
            "우측",
            board_center + distance_m * board_normal * 0.85 - side_offset_m * tangent_x,
        ),
        (
            "위쪽",
            board_center + distance_m * board_normal * 0.85 + side_offset_m * tangent_y,
        ),
        (
            "아래쪽",
            board_center + distance_m * board_normal * 0.85 - side_offset_m * tangent_y,
        ),
    ]

    # hand_eye inverse: gripper-to-camera
    R_g2c = hand_eye_R.T
    t_g2c = -R_g2c @ np.asarray(hand_eye_t).reshape(3)

    out: list[NextPoseRecommendation] = []
    excluded = excluded_ids or set()
    n_axes = min(len(arm_motor_ids), len(joint_limits_rad), 5)

    # fail 카운트 — 빈 list 일 때 *어느 분기로* 떨어졌는지 reason 결정용
    n_excluded = 0
    n_ik_fail = 0
    n_invisible = 0

    for idx, (label, cam_pos) in enumerate(anchors):
        rec_id = f"geometry_{idx}"
        if rec_id in excluded:
            n_excluded += 1
            continue

        cam_z = board_center - cam_pos
        cz_norm = np.linalg.norm(cam_z)
        if cz_norm < 1e-9:
            n_ik_fail += 1
            continue
        cam_z = cam_z / cz_norm

        if abs(np.dot(cam_z, world_up)) > 0.95:
            cam_x = np.array([1.0, 0.0, 0.0])
        else:
            cam_x = np.cross(world_up, cam_z)
            cam_x = cam_x / max(np.linalg.norm(cam_x), 1e-9)
        cam_y = np.cross(cam_z, cam_x)
        cam_y = cam_y / max(np.linalg.norm(cam_y), 1e-9)

        R_cam_in_base = np.column_stack([cam_x, cam_y, cam_z])
        R_ee_in_base = R_cam_in_base @ R_g2c
        t_ee_in_base = R_cam_in_base @ t_g2c + cam_pos

        target_pos = (
            float(t_ee_in_base[0]),
            float(t_ee_in_base[1]),
            float(t_ee_in_base[2]),
        )
        try:
            quat = Rotation.from_matrix(R_ee_in_base).as_quat()
        except ValueError:
            n_ik_fail += 1
            continue
        target_quat = (
            float(quat[0]),
            float(quat[1]),
            float(quat[2]),
            float(quat[3]),
        )

        # IK multi-seed retry — OMX-F 5DOF 의 좁은 joint manifold 에서 single seed IK
        # 실패하던 anchor 들 (좌/우/위/아래) 의 풀이 성공률 향상. base yaw 의 ±60°,
        # ±120° offset seed 들 시도 → 첫 풀린 + joint_limits 안 인 답 채택.
        # (audit: 5 anchor 중 1 개만 IK 풀리던 trauma source 직접 fix.)
        seeds_to_try: list[Sequence[float] | None] = [current_joint_angles_rad]
        if current_joint_angles_rad is not None:
            base = list(current_joint_angles_rad)
            for d_deg in (60.0, -60.0, 120.0, -120.0):
                alt = list(base)
                alt[0] = alt[0] + np.deg2rad(d_deg)
                seeds_to_try.append(alt)

        joint_angles = None
        for seed in seeds_to_try:
            try:
                ja = ik_fn(target_pos, target_quat, seed)
            except Exception:
                continue
            if ja is None or len(ja) < n_axes:
                continue
            if all(
                joint_limits_rad[i][0] <= ja[i] <= joint_limits_rad[i][1]
                for i in range(n_axes)
            ):
                joint_angles = ja
                break

        if joint_angles is None:
            n_ik_fail += 1
            continue

        visible = True
        visibility_reason = "unchecked"
        if visibility_check is not None:
            visible, visibility_reason = visibility_check(
                list(joint_angles[:n_axes])
            )
            if not visible:
                n_invisible += 1

        out.append(
            NextPoseRecommendation(
                joints=[
                    {"id": int(mid), "degree": float(degrees(ang))}
                    for mid, ang in zip(arm_motor_ids[:n_axes], joint_angles[:n_axes])
                ],
                reason=(
                    f"{label} — 보드 위 ~{distance_m * 100:.0f}cm sphere shell, "
                    f"카메라 z 축 보드 향함"
                ),
                label=label,
                primary_axis=0,
                source="geometry",
                diagnostics={
                    "mode": "geometry",
                    "anchor_id": rec_id,
                    "anchor_label": label,
                },
                visible=visible,
                visibility_reason=visibility_reason,
            )
        )

    # 빈 list 분기 reason — dominant fail mode 로 결정.
    # visible=false 인 추천도 list 에는 들어감 (UI 회색 hint) → 진짜 빈 list 는
    # n_invisible 이 모든 후보 였을 가능성은 X. invisible 자체로는 빈 list 안 만듦.
    # 빈 list 는 excluded + ik_fail 합쳐 5개 일 때만 발생.
    reason: NoCandidatesReason | None = None
    if not out:
        if n_excluded >= len(anchors):
            reason = NO_REASON_USER_MARKED_FAIL
        elif n_ik_fail > 0:
            reason = NO_REASON_ALL_IK_FAIL
        else:
            reason = NO_REASON_ALL_INVISIBLE  # safety fallback

    return RecommendationResult(out, reason)


def recommend_joint_sample(
    *,
    current_joint_angles_rad: Sequence[float],
    arm_motor_ids: list[int],
    joint_limits_rad: list[tuple[float, float]],
    fk_fn: Callable[[list[float]], tuple[Any, Any]],
    visibility_check: VisibilityCheck | None = None,
    existing_joint_angles: list[list[float]] | None = None,
    excluded_ids: set[str] | None = None,
    max_candidates: int = MAX_RECOMMENDATIONS,
) -> RecommendationResult:
    """5DOF robot 용 추천 자세 — current 자세 위 joint space perturbation.

    원리: "*좋은 카메라 자세 만들어서 로봇이 따라간다*" (recommend_geometry) 가 아니라
    "*로봇이 갈 수 있는 자세 중 좋은 걸 고른다*". 5DOF (OMX-F, wrist yaw 없음) 처럼
    임의 R 못 만드는 robot 에 자연스러움.

    추천 점수 (3 축):
      1. 갈 수 있음        — joint_limit + visibility 통과 (hard filter)
      2. 보드 잘 보임      — visibility_check 의 FOV + tilt 권장 범위
      3. 기존 pose 와 다름  — existing_joint_angles 와 joint-space min distance

    Strategy 패턴 — robots.yaml 의 `pose_recommend_strategy="joint_perturbation"`
    인 robot 만 사용. SO-101 (6DOF) 은 `recommend_geometry` 사용.

    Args:
        current_joint_angles_rad: 현재 자세 (J1..J5).
        arm_motor_ids: [1..5]
        joint_limits_rad: 각 joint 의 (lo, hi).
        fk_fn: angles → (R, t). visibility_check 의 forward 용.
        visibility_check: 보드 시야 안 + tilt 권장 범위 check. 통과 만 추천.
        existing_joint_angles: 이미 캡처된 pose 들의 joint angles. 다양성 점수 base.
            None 이면 다양성 점수 X (current 만 기준).
        excluded_ids: 사용자가 [👎] 표시한 추천 ID set.
        max_candidates: 최대 추천 개수.

    Returns:
        RecommendationResult — 5~8 개 안정 후보 목표.
    """
    n_axes = min(len(arm_motor_ids), len(joint_limits_rad), 5)
    if len(current_joint_angles_rad) < n_axes:
        return RecommendationResult([], None)

    excluded = excluded_ids or set()

    # axis 별 perturbation 후보 — board 가 23cm 가까이라 큰 yaw/pitch 가 시야 벗어남.
    # 작은 step (±10°) 부터 + 단계적으로 큼 (±25°). visible 한 step 부터 채택.
    # hand-eye 회전 추정 정보 큰 순: J1 > J4 > J5 > J2/J3.
    perturbations: list[tuple[int, str, float]] = [
        # ±10° step — board 시야 유지 확률 높음
        (0, "J1 yaw +10°", +10.0),
        (0, "J1 yaw -10°", -10.0),
        (3, "J4 pitch +10°", +10.0),
        (3, "J4 pitch -10°", -10.0),
        (4, "J5 roll +20°", +20.0),
        (4, "J5 roll -20°", -20.0),
        (1, "J2 shoulder +10°", +10.0),
        (1, "J2 shoulder -10°", -10.0),
        (2, "J3 elbow +10°", +10.0),
        (2, "J3 elbow -10°", -10.0),
        # ±20° step — 시야 유지되면 추가 다양성
        (0, "J1 yaw +20°", +20.0),
        (0, "J1 yaw -20°", -20.0),
        (3, "J4 pitch +20°", +20.0),
        (3, "J4 pitch -20°", -20.0),
        (4, "J5 roll +40°", +40.0),
        (4, "J5 roll -40°", -40.0),
        # ±30°+ step — fallback
        (0, "J1 yaw +30°", +30.0),
        (0, "J1 yaw -30°", -30.0),
        (4, "J5 roll +60°", +60.0),
        (4, "J5 roll -60°", -60.0),
    ]

    # candidates 모두 시도 → visibility 통과한 것만 후보. 이후 다양성 score 로 ranking.
    candidates: list[tuple[NextPoseRecommendation, list[float]]] = []
    n_excluded = 0
    n_limit_fail = 0
    n_invisible = 0

    for idx, (axis_idx, label, delta_deg) in enumerate(perturbations):
        rec_id = f"joint_sample_{idx}"
        if rec_id in excluded:
            n_excluded += 1
            continue
        if axis_idx >= n_axes:
            continue

        target = list(current_joint_angles_rad[:n_axes])
        target[axis_idx] = target[axis_idx] + radians(delta_deg)

        # joint limit
        lo, hi = joint_limits_rad[axis_idx]
        if not (lo <= target[axis_idx] <= hi):
            n_limit_fail += 1
            continue

        # visibility hard filter — joint_sample 은 visible 통과 자세만
        visible = True
        v_reason = "unchecked"
        if visibility_check is not None:
            visible, v_reason = visibility_check(target)
            if not visible:
                n_invisible += 1
                continue

        if _is_duplicate(target, [c[0] for c in candidates]):
            continue

        candidates.append(
            (
                NextPoseRecommendation(
                    joints=[
                        {"id": int(mid), "degree": float(degrees(ang))}
                        for mid, ang in zip(arm_motor_ids[:n_axes], target)
                    ],
                    reason=(
                        f"{label} — 현재 자세 위 변주, 보드 시야 안 + tilt 권장 범위."
                    ),
                    label=label,
                    primary_axis=axis_idx,
                    source="joint_sample",
                    diagnostics={
                        "mode": "joint_sample",
                        "delta_deg": delta_deg,
                        "axis_idx": axis_idx,
                    },
                    visible=visible,
                    visibility_reason=v_reason,
                ),
                target,
            )
        )

    # 다양성 score — 각 후보의 *기존 캡처 pose 들과의 joint-space minimum L2 distance* (rad).
    # 큼 = 기존과 다름 = 정보 기여 큼. existing 없으면 score = 1.0 (uniform).
    def diversity_score(target_angles: list[float]) -> float:
        if not existing_joint_angles:
            return 1.0
        ta = np.asarray(target_angles)
        existing = np.asarray(
            [e[: len(target_angles)] for e in existing_joint_angles]
        )
        dists = np.linalg.norm(existing - ta, axis=1)
        return float(dists.min())

    candidates.sort(key=lambda c: -diversity_score(c[1]))

    # diagnostics 에 다양성 점수 기록 (UI 가 표시 가능)
    for rec, ang in candidates:
        rec.diagnostics["diversity_score_rad"] = round(diversity_score(ang), 4)

    out = [c[0] for c in candidates[:max_candidates]]

    reason: NoCandidatesReason | None = None
    if not out:
        if n_excluded >= len(perturbations):
            reason = NO_REASON_USER_MARKED_FAIL
        elif n_invisible > 0 and n_invisible >= n_limit_fail:
            reason = NO_REASON_ALL_INVISIBLE
        else:
            reason = NO_REASON_ALL_IK_FAIL  # joint_limit 실패도 IK 실패 부류

    return RecommendationResult(out, reason)


# ─── Strategy Pattern — 로봇 kinematic 구조별 추천 전략 ──────────────────
#
# 발견: recommend_geometry (6DOF anchor 기반) 가 OMX-F (5DOF, wrist yaw 없음)
# 에선 anchor 5 중 1만 IK 풀림. "*좋은 카메라 자세 만들어 로봇이 따라간다*"
# 모델이 *5DOF manifold 안* 에 못 들어옴.
#
# 신규: JointPerturbationStrategy — "*로봇이 갈 수 있는 자세 중 좋은 걸 고른다*".
# IK 안 풀고 FK 만 사용 → 항상 robot 이 만들 수 있는 자세.
#
# robots.yaml 의 `pose_recommend_strategy` 가 SSOT.


@dataclass
class RecommendContext:
    """추천 전략 공통 입력. strategy 가 자기 필요한 필드만 사용."""

    current_joint_angles_rad: Sequence[float]
    arm_motor_ids: list[int]
    joint_limits_rad: list[tuple[float, float]]
    fk_fn: Callable[[list[float]], tuple[Any, Any]]
    ik_fn: Callable[
        [
            tuple[float, float, float],
            tuple[float, float, float, float] | None,
            Sequence[float] | None,
        ],
        list[float] | None,
    ]
    board_corners_base: np.ndarray  # (4, 3)
    hand_eye_R: np.ndarray  # (3, 3)
    hand_eye_t: np.ndarray  # (3,)
    outward_hint: np.ndarray | None = None
    visibility_check: VisibilityCheck | None = None
    existing_joint_angles: list[list[float]] | None = None
    excluded_ids: set[str] | None = None
    max_candidates: int = MAX_RECOMMENDATIONS
    # coach.axis_distributions 의 is_low_diversity=true 인 motor_id list.
    # narrow_sigma_good 분기에서 caller 가 채움. 빈 list 면 hybrid 자리 skip.
    weak_axis_motor_ids: list[int] = field(default_factory=list)
    # 위 motor_id → suggested_deg (절대 deg, 모터 limit 안). caller 가 같이 채움.
    weak_axis_suggested_deg: dict[int, float | None] = field(default_factory=dict)


class PoseRecommendationStrategy(Protocol):
    """robot 별 추천 전략 인터페이스."""

    def recommend(self, ctx: RecommendContext) -> RecommendationResult: ...


class GeometryStrategy:
    """6DOF (또는 wrist yaw 있는) robot 용 — anchor sphere shell + IK.

    임의 카메라 R 만들 수 있는 robot 에 자연 — SO-101, UR 등.

    Hybrid 자리 — narrow_sigma_good 분기에서 ctx.weak_axis_motor_ids 채워주면
    `recommend_axis_diversify` 결과를 list 앞에 prepend. sphere shell 5 anchor 만
    으로는 *어느 axis 좁다* 정보를 못 받아 다양성 보충 못 하던 자리 fix.
    """

    def recommend(self, ctx: RecommendContext) -> RecommendationResult:
        result = recommend_geometry(
            board_corners_base=ctx.board_corners_base,
            ik_fn=ctx.ik_fn,
            hand_eye_R=ctx.hand_eye_R,
            hand_eye_t=ctx.hand_eye_t,
            arm_motor_ids=ctx.arm_motor_ids,
            joint_limits_rad=ctx.joint_limits_rad,
            current_joint_angles_rad=ctx.current_joint_angles_rad,
            outward_hint=ctx.outward_hint,
            visibility_check=ctx.visibility_check,
            excluded_ids=ctx.excluded_ids,
        )

        # narrow_sigma_good 자리 — caller 가 weak axis 정보 채워줬으면 다양성 보충
        # 후보 prepend. 사용자 mental model "다양성 부족 안내 ↔ 어디 캡처할지 ghost"
        # 한 line 연결.
        if ctx.weak_axis_motor_ids:
            diversify = recommend_axis_diversify(
                current_joint_angles_rad=ctx.current_joint_angles_rad,
                arm_motor_ids=ctx.arm_motor_ids,
                joint_limits_rad=ctx.joint_limits_rad,
                weak_axis_motor_ids=ctx.weak_axis_motor_ids,
                weak_axis_suggested_deg=ctx.weak_axis_suggested_deg,
                visibility_check=ctx.visibility_check,
                excluded_ids=ctx.excluded_ids,
            )
            if diversify:
                # diversify 우선 — 사용자 mental model "약한 axis 보충" 자리 가장 먼저.
                result.recommendations = diversify + result.recommendations
                # 빈 list reason (no_board_estimate / all_ik_fail) 도 자연 해소
                result.no_candidates_reason = None
        return result


class JointPerturbationStrategy:
    """5DOF (또는 wrist yaw 없는) robot 용 — joint space perturbation + FK.

    robot kinematic manifold 안에서만 sample → 항상 reachable. OMX-F 등.
    """

    def recommend(self, ctx: RecommendContext) -> RecommendationResult:
        return recommend_joint_sample(
            current_joint_angles_rad=ctx.current_joint_angles_rad,
            arm_motor_ids=ctx.arm_motor_ids,
            joint_limits_rad=ctx.joint_limits_rad,
            fk_fn=ctx.fk_fn,
            visibility_check=ctx.visibility_check,
            existing_joint_angles=ctx.existing_joint_angles,
            excluded_ids=ctx.excluded_ids,
            max_candidates=ctx.max_candidates,
        )


def make_strategy(name: str) -> PoseRecommendationStrategy:
    """robots.yaml::pose_recommend_strategy 값으로 strategy 인스턴스 생성.

    유효값: "geometry" (6DOF, default), "joint_perturbation" (5DOF).
    """
    name_lower = (name or "geometry").lower()
    if name_lower == "geometry":
        return GeometryStrategy()
    if name_lower == "joint_perturbation":
        return JointPerturbationStrategy()
    raise ValueError(
        f"unknown pose_recommend_strategy='{name}'. "
        f"유효: 'geometry' | 'joint_perturbation'"
    )


def _is_duplicate(
    candidate_rad: list[float], existing: list[NextPoseRecommendation]
) -> bool:
    """모든 축 차이가 _DEDUPE_TOLERANCE_RAD 이내면 중복."""
    for rec in existing:
        rec_rad = [radians(j["degree"]) for j in rec.joints]
        if len(rec_rad) != len(candidate_rad):
            continue
        if all(
            abs(a - b) <= _DEDUPE_TOLERANCE_RAD
            for a, b in zip(candidate_rad, rec_rad)
        ):
            return True
    return False


def to_dict(rec: NextPoseRecommendation) -> dict:
    """프론트엔드 응답용 직렬화."""
    return {
        "joints": rec.joints,
        "reason": rec.reason,
        "label": rec.label,
        "primary_axis": rec.primary_axis,
        "source": rec.source,
        "diagnostics": rec.diagnostics,
        "visible": rec.visible,
        "visibility_reason": rec.visibility_reason,
    }
