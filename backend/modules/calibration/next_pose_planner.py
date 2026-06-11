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
from typing import Any, Callable, Sequence

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from . import joint_distribution as jd
from . import thresholds

logger = logging.getLogger(__name__)


VisibilityCheck = Callable[[list[float]], tuple[bool, str]]
"""후보 joint angles → (visible, reason). visibility_check 인자."""

# 잔차 큰 포즈를 base로 추천 자세를 만들 때 J1/J4/J5 중 어느 축을 변주할지
# 우선순위 (hand-eye 회전 추정 영향 큰 순). 0-indexed: 3=J4, 0=J1, 4=J5.
# cv2.calibrateHandEye는 카메라 광축에 수직한 축(J4 pitch, J1 yaw) 회전이
# 회전 추정에 가장 많은 정보를 줌. J5(wrist roll)은 광축 회전이라 정보 기여가
# 작아 fallback. (이전 [4,3,0]은 시야 안전 휴리스틱이었지만, J5 가동범위가
# ±150° 이상이라 우선순위 1위 + _VARIANTS_PER_BASE=2 조합이 한 base에서 J5
# 위/아래로 슬롯을 다 먹어 J4/J1이 영원히 추천 안 나오는 버그가 있었음.)
_AXIS_PRIORITY = [3, 0, 4]

# 잔차 큰 포즈 근처에서 한 축을 얼마나 변주할지 (deg). 너무 작으면 BA가 새 정보
# 못 받음. 너무 크면 체커보드 시야 벗어남.
_AXIS_PERTURBATION_DEG: float = 20.0

# 잔차 임계 (deg). 이 이상인 포즈가 "잔차 큰 영역 보강" 대상.
# 미만이면 "분포 다양성 보강"으로 자리 채움.
_HIGH_RESIDUAL_THRESHOLD_DEG: float = 0.5

# 후보 리스트 최대 길이. 너무 많으면 사용자 피로, 너무 적으면 다 가도 안 보일 수 있음.
MAX_RECOMMENDATIONS: int = 6

# 한 잔차-큰-포즈에서 뽑을 변주 개수. 1 = 한 base에서 한 축 한 방향만 →
# 다음 high-residual base로 넘어가 다른 축/시작점에서 변주가 나오도록 강제.
# (이전 2는 가동범위 넓은 축이 한 base 안에서 위/아래 두 슬롯을 다 먹어
# _AXIS_PRIORITY의 다음 축까지 못 닿는 문제 원인이었음.)
_VARIANTS_PER_BASE: int = 1

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


@dataclass
class RecommendationResult:
    recommendations: list[NextPoseRecommendation]
    # 빈 list 일 때만 채워짐. caller 가 σ/verdict 결합해 덮어쓸 수 있음.
    no_candidates_reason: NoCandidatesReason | None = None


def recommend_many(
    *,
    last_compute: dict | None,
    joint_angles_per_pose_at_compute: list[list[float]] | None,
    current_joint_angles_rad: list[float],
    arm_motor_ids: list[int],
    joint_limits_rad: list[tuple[float, float]],
    visibility_check: VisibilityCheck | None = None,
) -> list[NextPoseRecommendation]:
    """다음 캡처 후보 N개 반환.

    Args:
        last_compute: 직전 compute 결과. 없으면 분포 기반만.
        joint_angles_per_pose_at_compute: 직전 compute 의 해석된 joint angles
            (URDF rad). last_compute 의 per_pose_residual 과 같은 순서.
        current_joint_angles_rad: 분포 fallback 의 base 가 될 현재 모터 위치.
        arm_motor_ids: [1..5]
        joint_limits_rad: Kinematics.joint_limits(5)
        visibility_check: 후보 자세에서 보드가 카메라 frame 안인지 확인 함수.
            None 이면 visibility 표시만 "unchecked" 로 박힘. intrinsic + hand_eye +
            board base 추정 모두 있을 때만 caller 가 제공.
    """
    n_axes = min(len(arm_motor_ids), len(joint_limits_rad), 5)
    if len(current_joint_angles_rad) < n_axes:
        return []

    out: list[NextPoseRecommendation] = []

    out.extend(
        _from_high_residual_many(
            last_compute=last_compute,
            ja_at_compute=joint_angles_per_pose_at_compute,
            arm_motor_ids=arm_motor_ids[:n_axes],
            joint_limits_rad=joint_limits_rad[:n_axes],
            remaining=MAX_RECOMMENDATIONS,
        )
    )

    remaining = MAX_RECOMMENDATIONS - len(out)
    if remaining > 0:
        out.extend(
            _from_distribution_many(
                ja_per_pose=joint_angles_per_pose_at_compute or [],
                current=current_joint_angles_rad[:n_axes],
                arm_motor_ids=arm_motor_ids[:n_axes],
                joint_limits_rad=joint_limits_rad[:n_axes],
                already_chosen=out,
                remaining=remaining,
            )
        )

    # visibility 마크 — 각 후보의 보드 가시성. UI 는 visible=false 후보를 회색
    # 처리하되 사용자가 원하면 시도 가능 (gate 가 hard filter 가 아니라 hint).
    if visibility_check is not None:
        for rec in out:
            rec_rad = [radians(j["degree"]) for j in rec.joints]
            visible, reason = visibility_check(rec_rad)
            rec.visible = visible
            rec.visibility_reason = reason

    return out[:MAX_RECOMMENDATIONS]


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


def _from_high_residual_many(
    *,
    last_compute: dict | None,
    ja_at_compute: list[list[float]] | None,
    arm_motor_ids: list[int],
    joint_limits_rad: list[tuple[float, float]],
    remaining: int,
) -> list[NextPoseRecommendation]:
    """잔차 큰 포즈들 → 각 포즈에서 최대 _VARIANTS_PER_BASE개 변주."""
    if not last_compute or not ja_at_compute or remaining <= 0:
        return []
    per_pose = last_compute.get("per_pose_residual", [])
    if not per_pose:
        return []

    # excluded 제외, 잔차 큰 것부터 정렬
    candidates = [(i, r) for i, r in enumerate(per_pose) if not r.get("excluded")]
    if not candidates:
        return []
    candidates.sort(key=lambda x: -float(x[1].get("drot_deg", 0.0)))

    n_axes = len(arm_motor_ids)
    out: list[NextPoseRecommendation] = []

    for idx, res in candidates:
        if len(out) >= remaining:
            break
        drot = float(res.get("drot_deg", 0.0))
        if drot < _HIGH_RESIDUAL_THRESHOLD_DEG:
            # 더 이상 큰 잔차 없음 — 잔차 모드 종료
            break
        if idx >= len(ja_at_compute):
            continue
        base_angles_rad = list(ja_at_compute[idx][:n_axes])
        pose_id = res.get("id", "?")
        produced_for_base = 0

        # 이미 추천된 축은 뒤로 — base 간 축 다양성 강제. 같은 카운트 내에서는
        # _AXIS_PRIORITY 순 유지. (이전엔 모든 base가 우선순위 1위 축으로만 가서
        # 한 축이 추천 리스트를 도배하는 문제가 있었음.)
        axis_counts = {a: 0 for a in _AXIS_PRIORITY}
        for rec in out:
            if rec.primary_axis in axis_counts:
                axis_counts[rec.primary_axis] += 1
        axis_order = sorted(
            _AXIS_PRIORITY,
            key=lambda a: (axis_counts[a], _AXIS_PRIORITY.index(a)),
        )

        for axis_idx in axis_order:
            if produced_for_base >= _VARIANTS_PER_BASE:
                break
            if len(out) >= remaining:
                break
            if axis_idx >= n_axes:
                continue
            lo, hi = joint_limits_rad[axis_idx]
            cur = base_angles_rad[axis_idx]
            delta = radians(_AXIS_PERTURBATION_DEG)
            up_room = hi - cur
            down_room = cur - lo

            # 한 축에서 한 방향만 — 더 여유 있는 쪽 선택. 둘 다 부족하면 다음 축.
            if up_room >= delta and up_room >= down_room:
                dir_name, new_val = "위쪽", cur + delta
            elif down_room >= delta:
                dir_name, new_val = "아래쪽", cur - delta
            elif up_room >= delta:
                dir_name, new_val = "위쪽", cur + delta
            else:
                continue

            target = list(base_angles_rad)
            target[axis_idx] = new_val
            # 다른 축들 안전 클램프
            for i in range(n_axes):
                lo_i, hi_i = joint_limits_rad[i]
                target[i] = max(lo_i, min(hi_i, target[i]))

            if _is_duplicate(target, out):
                continue

            signed_deg = (
                _AXIS_PERTURBATION_DEG if dir_name == "위쪽"
                else -_AXIS_PERTURBATION_DEG
            )
            label = f"J{axis_idx + 1} {dir_name} {signed_deg:+.0f}°"
            reason = (
                f"포즈 #{pose_id} 잔차 큼 (Δrot={drot:.2f}°) — "
                f"그 영역 J{axis_idx + 1} {dir_name} "
                f"{_AXIS_PERTURBATION_DEG:.0f}° 변주."
            )
            out.append(
                NextPoseRecommendation(
                    joints=[
                        {"id": int(mid), "degree": float(degrees(ang))}
                        for mid, ang in zip(arm_motor_ids, target)
                    ],
                    reason=reason,
                    label=label,
                    primary_axis=axis_idx,
                    source="high_residual",
                    diagnostics={
                        "mode": "high_residual",
                        "base_pose_id": pose_id,
                        "base_residual_rot_deg": drot,
                        "direction": dir_name,
                    },
                )
            )
            produced_for_base += 1

    return out


def _from_distribution_many(
    *,
    ja_per_pose: list[list[float]],
    current: list[float],
    arm_motor_ids: list[int],
    joint_limits_rad: list[tuple[float, float]],
    already_chosen: list[NextPoseRecommendation],
    remaining: int,
) -> list[NextPoseRecommendation]:
    """빈 축 1개당 1후보. J5/J4/J1 우선, 그 다음 J2/J3."""
    if remaining <= 0:
        return []
    dists = jd.analyze(
        joint_angles_per_pose=ja_per_pose,
        arm_motor_ids=arm_motor_ids,
        joint_limits_rad=joint_limits_rad,
    )
    out: list[NextPoseRecommendation] = []
    axis_order = _AXIS_PRIORITY + [1, 2]
    for axis_idx in axis_order:
        if len(out) >= remaining:
            break
        if axis_idx >= len(dists):
            continue
        dist = dists[axis_idx]
        if not dist.is_low_diversity or dist.suggested_deg is None:
            continue
        target_rad = radians(dist.suggested_deg)
        lo, hi = joint_limits_rad[axis_idx]
        if not (lo <= target_rad <= hi):
            continue
        target = list(current)
        target[axis_idx] = target_rad
        for i in range(len(target)):
            lo_i, hi_i = joint_limits_rad[i]
            target[i] = max(lo_i, min(hi_i, target[i]))

        if _is_duplicate(target, already_chosen + out):
            continue

        label = f"J{axis_idx + 1} {dist.suggested_deg:+.0f}°"
        out.append(
            NextPoseRecommendation(
                joints=[
                    {"id": int(mid), "degree": float(degrees(ang))}
                    for mid, ang in zip(arm_motor_ids, target)
                ],
                reason=dist.suggestion_text,
                label=label,
                primary_axis=axis_idx,
                source="distribution",
                diagnostics={
                    "mode": "distribution",
                    "axis_distribution": jd.to_dict(dist),
                },
            )
        )
    return out


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

        try:
            joint_angles = ik_fn(target_pos, target_quat, current_joint_angles_rad)
        except Exception as e:
            logger.debug("[geometry] IK 실패 anchor=%s: %s", label, e)
            joint_angles = None
        if joint_angles is None or len(joint_angles) < n_axes:
            n_ik_fail += 1
            continue

        within_limits = all(
            joint_limits_rad[i][0] <= joint_angles[i] <= joint_limits_rad[i][1]
            for i in range(n_axes)
        )
        if not within_limits:
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
