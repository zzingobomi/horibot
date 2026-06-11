"""Hand-Eye 캡처 분포 분석 — coach 메시지 + next_pose_planner 공통 입력.

기존 coach.py는 std 한 값만 보여줘서 "어디로 가야 하는지"가 안 보임.
여기서 각 축의 (min/max/std + 모터 limit 대비 어디가 비었는지)를 계산해
coach가 절대 각도로 "J4를 -10° 부근에서 추가 캡처" 식으로 안내할 수 있게 함.
같은 분석을 next_pose_planner가 후보 자세 sampling base로 사용.

대상 축: 5DOF arm (J1~J5). 임계값은 thresholds.JOINT_DIVERSITY_THRESHOLD_DEG.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from . import thresholds as T

# coach.py에서도 import 가능하도록 한 곳에 모음.
JOINT_NAMES_KO: list[str] = [
    "J1 (base yaw)",
    "J2 (shoulder)",
    "J3 (elbow)",
    "J4 (wrist pitch)",
    "J5 (wrist roll)",
]

# 모터 hard limit 안에서 추천 각도를 어느 정도 여유 둘지 (deg).
# 정확히 limit 끝으로 보내면 IK/구동 시 안전 마진 부족.
LIMIT_MARGIN_DEG: float = 5.0


@dataclass
class AxisDistribution:
    motor_id: int
    name_ko: str
    std_deg: float
    min_deg: float
    max_deg: float
    threshold_deg: float
    is_low_diversity: bool
    motor_limit_min_deg: float
    motor_limit_max_deg: float
    # 다음 캡처 추천 각도 (절대값, deg). None이면 추가 캡처 불필요(다양성 OK)
    # 또는 추천 영역이 모터 limit 밖이라 불가능.
    suggested_deg: float | None
    # 사용자 안내 텍스트. coach 메시지에 그대로 들어감.
    suggestion_text: str


def analyze(
    *,
    joint_angles_per_pose: list[list[float]],
    arm_motor_ids: list[int],
    joint_limits_rad: list[tuple[float, float]],
) -> list[AxisDistribution]:
    """캡처된 자세들의 axis별 분포 분석.

    Args:
        joint_angles_per_pose: 각 캡처 자세의 [J1, J2, J3, J4, J5, ...] (rad).
            joint_offset 적용 *후* 값 (= URDF rad).
        arm_motor_ids: 5DOF arm 모터 ID 리스트 (예: [1,2,3,4,5]).
        joint_limits_rad: Kinematics에서 가져온 (lower, upper) tuple 리스트.
            arm_motor_ids와 같은 순서/길이.

    캡처 0개여도 동작 — std=0, suggested_deg는 모터 limit 양 끝에서 결정.
    """
    n_axes = min(len(arm_motor_ids), len(joint_limits_rad), 5)
    thresholds = T.JOINT_DIVERSITY_THRESHOLD_DEG

    has_data = len(joint_angles_per_pose) > 0 and all(
        len(j) >= n_axes for j in joint_angles_per_pose
    )

    result: list[AxisDistribution] = []
    for i in range(n_axes):
        lim_min_deg = float(np.degrees(joint_limits_rad[i][0]))
        lim_max_deg = float(np.degrees(joint_limits_rad[i][1]))
        name = JOINT_NAMES_KO[i]
        thr = thresholds[i] if i < len(thresholds) else 15.0

        if has_data:
            angles_deg = np.degrees([j[i] for j in joint_angles_per_pose])
            std_deg = float(angles_deg.std())
            min_deg = float(angles_deg.min())
            max_deg = float(angles_deg.max())
        else:
            std_deg = 0.0
            min_deg = 0.0
            max_deg = 0.0

        low = std_deg < thr if has_data else True
        suggested, text = _suggest(
            has_data=has_data,
            name=name,
            std_deg=std_deg,
            min_deg=min_deg,
            max_deg=max_deg,
            threshold_deg=thr,
            limit_min_deg=lim_min_deg,
            limit_max_deg=lim_max_deg,
        )
        result.append(
            AxisDistribution(
                motor_id=arm_motor_ids[i],
                name_ko=name,
                std_deg=std_deg,
                min_deg=min_deg,
                max_deg=max_deg,
                threshold_deg=thr,
                is_low_diversity=low,
                motor_limit_min_deg=lim_min_deg,
                motor_limit_max_deg=lim_max_deg,
                suggested_deg=suggested,
                suggestion_text=text,
            )
        )
    return result


def _suggest(
    *,
    has_data: bool,
    name: str,
    std_deg: float,
    min_deg: float,
    max_deg: float,
    threshold_deg: float,
    limit_min_deg: float,
    limit_max_deg: float,
) -> tuple[float | None, str]:
    """이 축의 다음 캡처 추천 각도 + 사용자 안내 텍스트."""
    safe_lo = limit_min_deg + LIMIT_MARGIN_DEG
    safe_hi = limit_max_deg - LIMIT_MARGIN_DEG
    if safe_lo >= safe_hi:
        return None, f"{name}: 모터 limit 마진 부족 (제약 검토 필요)"

    if not has_data:
        # 캡처 0 → 양 극단 두 곳 안내, 일단 limit hi 쪽을 추천각으로.
        return (
            safe_hi,
            f"{name}: 첫 캡처. {safe_lo:.0f}°과 {safe_hi:.0f}° 양 극단을 포함하도록 분포 잡기.",
        )

    # 캡처된 분포 vs 모터 limit — 어느 쪽이 더 비었나
    gap_lo = min_deg - safe_lo  # 분포 min에서 안전 limit min까지 남은 공간
    gap_hi = safe_hi - max_deg  # 분포 max에서 안전 limit max까지

    if not (std_deg < threshold_deg):
        # 다양성 충분 — 추천 없음
        return (
            None,
            f"{name}: 분포 OK ({min_deg:+.0f}°~{max_deg:+.0f}°, std={std_deg:.1f}°).",
        )

    # 다양성 부족 — 더 큰 갭 쪽으로 확장 추천
    if gap_hi >= gap_lo and gap_hi > 0.0:
        target = max_deg + max(gap_hi * 0.5, 10.0)
        target = float(min(target, safe_hi))
        direction = "위쪽"
    elif gap_lo > 0.0:
        target = min_deg - max(gap_lo * 0.5, 10.0)
        target = float(max(target, safe_lo))
        direction = "아래쪽"
    else:
        # 양쪽 다 limit에 붙어있는데 std가 작으면 — 분포가 중앙에 몰림
        # 양 극단 중 모터 limit 더 여유있는 쪽으로
        target = safe_hi if (safe_hi - max_deg) > (min_deg - safe_lo) else safe_lo
        direction = "극단"

    text = (
        f"{name}: 현재 {min_deg:+.0f}°~{max_deg:+.0f}° (std={std_deg:.1f}° < "
        f"{threshold_deg:.0f}°) — {direction} {target:+.0f}° 부근에서 추가 캡처."
    )
    return target, text


def to_dict(dist: AxisDistribution) -> dict:
    """프론트엔드 응답용 직렬화."""
    return {
        "motor_id": dist.motor_id,
        "name_ko": dist.name_ko,
        "std_deg": dist.std_deg,
        "min_deg": dist.min_deg,
        "max_deg": dist.max_deg,
        "threshold_deg": dist.threshold_deg,
        "is_low_diversity": dist.is_low_diversity,
        "motor_limit_min_deg": dist.motor_limit_min_deg,
        "motor_limit_max_deg": dist.motor_limit_max_deg,
        "suggested_deg": dist.suggested_deg,
        "suggestion_text": dist.suggestion_text,
    }
