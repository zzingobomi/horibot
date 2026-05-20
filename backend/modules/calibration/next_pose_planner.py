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

from . import joint_distribution as jd

logger = logging.getLogger(__name__)

# 잔차 큰 포즈를 base로 추천 자세를 만들 때 J1/J4/J5 중 어느 축을 변주할지
# 우선순위 (hand-eye 회전 추정 영향 큰 순). 0-indexed: 4=J5, 3=J4, 0=J1.
# J5(wrist roll)이 광축 회전이라 체커보드 시야 잃을 확률 가장 낮음 → 첫 시도.
_AXIS_PRIORITY = [4, 3, 0]

# 잔차 큰 포즈 근처에서 한 축을 얼마나 변주할지 (deg). 너무 작으면 BA가 새 정보
# 못 받음. 너무 크면 체커보드 시야 벗어남.
_AXIS_PERTURBATION_DEG: float = 20.0

# 잔차 임계 (deg). 이 이상인 포즈가 "잔차 큰 영역 보강" 대상.
# 미만이면 "분포 다양성 보강"으로 자리 채움.
_HIGH_RESIDUAL_THRESHOLD_DEG: float = 0.5

# 후보 리스트 최대 길이. 너무 많으면 사용자 피로, 너무 적으면 다 가도 안 보일 수 있음.
MAX_RECOMMENDATIONS: int = 6

# 한 잔차-큰-포즈에서 뽑을 변주 개수. 한 base만 우려먹지 않게 캡.
_VARIANTS_PER_BASE: int = 2

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


def recommend_many(
    *,
    last_compute: dict | None,
    joint_angles_per_pose_at_compute: list[list[float]] | None,
    current_joint_angles_rad: list[float],
    arm_motor_ids: list[int],
    joint_limits_rad: list[tuple[float, float]],
) -> list[NextPoseRecommendation]:
    """다음 캡처 후보 N개 반환. 빈 리스트면 추천 없음(σ 충분히 좋거나 변주 여유 없음).

    Args:
        last_compute: 직전 _srv_handeye_compute 결과 dict.
            없으면 분포 기반만 사용.
        joint_angles_per_pose_at_compute: 직전 compute의 *해석된* joint angles
            (URDF rad). last_compute의 per_pose_residual과 같은 순서.
        current_joint_angles_rad: 분포 fallback의 base가 될 현재 모터 위치.
        arm_motor_ids: [1..5]
        joint_limits_rad: PybulletSolver.joint_limits(5)
    """
    n_axes = min(len(arm_motor_ids), len(joint_limits_rad), 5)
    if len(current_joint_angles_rad) < n_axes:
        return []

    out: list[NextPoseRecommendation] = []

    # 1) 잔차 큰 포즈 기반 후보 — 우선순위 높음
    out.extend(
        _from_high_residual_many(
            last_compute=last_compute,
            ja_at_compute=joint_angles_per_pose_at_compute,
            arm_motor_ids=arm_motor_ids[:n_axes],
            joint_limits_rad=joint_limits_rad[:n_axes],
            remaining=MAX_RECOMMENDATIONS,
        )
    )

    # 2) 분포 fallback — 잔차 자리 채운 뒤 남은 슬롯에만
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

    return out[:MAX_RECOMMENDATIONS]


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

        for axis_idx in _AXIS_PRIORITY:
            if produced_for_base >= _VARIANTS_PER_BASE:
                break
            if axis_idx >= n_axes:
                continue
            lo, hi = joint_limits_rad[axis_idx]
            cur = base_angles_rad[axis_idx]
            delta = radians(_AXIS_PERTURBATION_DEG)
            up_room = hi - cur
            down_room = cur - lo

            # 더 여유 있는 쪽부터 시도. 양쪽 다 가능하면 둘 다 추가 시도(같은 base에서).
            directions: list[tuple[str, float]] = []
            if up_room >= delta:
                directions.append(("위쪽", cur + delta))
            if down_room >= delta:
                directions.append(("아래쪽", cur - delta))

            for dir_name, new_val in directions:
                if produced_for_base >= _VARIANTS_PER_BASE:
                    break
                if len(out) >= remaining:
                    break
                target = list(base_angles_rad)
                target[axis_idx] = new_val
                # 다른 축들 안전 클램프
                for i in range(n_axes):
                    lo_i, hi_i = joint_limits_rad[i]
                    target[i] = max(lo_i, min(hi_i, target[i]))

                if _is_duplicate(target, out):
                    continue

                label = (
                    f"J{axis_idx + 1} {dir_name} "
                    f"{_AXIS_PERTURBATION_DEG:+.0f}°"
                )
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
    }
