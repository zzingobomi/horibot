"""Hand-Eye 캘리브레이션 진단(coach) 메시지.

사용자는 캡처/계산/커밋만 함. outlier 식별/제거는 hand_eye.py가 자동으로
처리하고, coach는 그 결과를 사용자에게 안내:
    - 자동 제외된 포즈가 있으면 정보로 표시
    - 정확도가 부족하면 어떤 축을 더 캡처해야 하는지 구체적으로 안내
    - 정확도가 충분하면 COMMIT 안내

verdict 기준은 thresholds.py의 SIGMA_*_GOOD / SIGMA_*_WARN 참조.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field

import numpy as np

from . import thresholds as T

logger = logging.getLogger(__name__)

# 5DOF 아암 (ID 1~5) 이름 — 한국어
JOINT_NAMES_KO: list[str] = [
    "base yaw (J1)",
    "shoulder (J2)",
    "elbow (J3)",
    "wrist pitch (J4)",
    "wrist roll (J5)",
]


@dataclass
class CoachMessage:
    level: str  # "success" | "info" | "warn" | "error"
    text: str


@dataclass
class CoachReport:
    verdict: str  # "good" | "needs_work" | "bad"
    messages: list[CoachMessage] = field(default_factory=list)

    def to_dict(self) -> dict:
        return {
            "verdict": self.verdict,
            "messages": [{"level": m.level, "text": m.text} for m in self.messages],
        }


def diagnose(
    *,
    pose_count: int,
    joint_angles_per_pose: list[list[float]],
    per_pose_residuals: list[dict],  # [{id, drot_deg, dt_mm, excluded}]
    sigma_rot_deg: float,
    sigma_t_mm: float,
    method_compare: list[dict],
    excluded_pose_ids: list[int],
    excluded_cap_hit: bool,
) -> CoachReport:
    """캘 결과 진단 → coach 메시지 생성.

    sigma_rot_deg / sigma_t_mm 은 outlier 제거 후 깨끗한 set 위 RMS.
    """
    msgs: list[CoachMessage] = []

    # 1. 자동 제외 보고 (정보용, verdict 무관)
    if excluded_pose_ids:
        ids_str = ", ".join(f"#{i}" for i in excluded_pose_ids)
        # 제외된 포즈의 잔차 표시
        excl_residuals = [
            r for r in per_pose_residuals if r["id"] in set(excluded_pose_ids)
        ]
        if excl_residuals:
            max_rot = max(r["drot_deg"] for r in excl_residuals)
            max_t = max(r["dt_mm"] for r in excl_residuals)
            msgs.append(
                CoachMessage(
                    "info",
                    f"자동 제외된 포즈: {ids_str} "
                    f"(최대 잔차 rot={max_rot:.2f}°, t={max_t:.1f}mm). "
                    f"depth 모션블러/PnP 오검출 의심.",
                )
            )

    # 2. 비율 가드에 걸렸으면 — 자세 다양성 부족 신호
    if excluded_cap_hit:
        msgs.append(
            CoachMessage(
                "warn",
                f"잔차 큰 포즈가 {int(T.OUTLIER_REMOVAL_CAP_RATIO * 100)}%를 초과 — "
                f"BA가 FK floor를 흡수하지 못함. "
                f"자세 다양성을 늘려 추가 캡처가 필요합니다.",
            )
        )

    # 3. 자세 수
    if pose_count < T.RECOMMENDED_POSES:
        msgs.append(
            CoachMessage(
                "info",
                f"권장 자세 수 미달 ({pose_count}/{T.RECOMMENDED_POSES}) — "
                f"안정적 추정을 위해 더 캡처하세요.",
            )
        )

    # 4. 조인트 다양성 — verdict가 good이 아닐 때만 안내
    if joint_angles_per_pose and all(len(j) >= 5 for j in joint_angles_per_pose):
        joints = np.array([j[:5] for j in joint_angles_per_pose])  # (N, 5)
        std_deg = np.degrees(joints.std(axis=0))
        for s, thr, name in zip(
            std_deg, T.JOINT_DIVERSITY_THRESHOLD_DEG, JOINT_NAMES_KO
        ):
            if s < thr:
                msgs.append(
                    CoachMessage(
                        "warn",
                        f"{name} 다양성 부족 (std={s:.1f}° < {thr:.0f}°) — "
                        f"이 축을 더 회전시켜 캡처하세요.",
                    )
                )

    # 5. method self-consistency (입력 노이즈 진단)
    park = next((c for c in method_compare if c.get("method") == "PARK"), None)
    if park and not park.get("ref") and park.get("drot_deg", 0.0) >= 1.0:
        drot = park["drot_deg"]
        msgs.append(
            CoachMessage(
                "warn",
                f"알고리즘 간 결과 차이 큼 (PARK Δrot={drot:.2f}°) — "
                f"자세 다양성 부족 또는 입력 노이즈 가능성.",
            )
        )

    # 6. 종합 verdict
    if sigma_rot_deg < T.SIGMA_ROT_GOOD_DEG and sigma_t_mm < T.SIGMA_T_GOOD_MM:
        verdict = "good"
        msgs.insert(
            0,
            CoachMessage(
                "success",
                f"정확도 충분 (σ_rot={sigma_rot_deg:.2f}°, σ_t={sigma_t_mm:.1f}mm). "
                f"COMMIT 가능합니다.",
            ),
        )
    elif sigma_rot_deg < T.SIGMA_ROT_WARN_DEG and sigma_t_mm < T.SIGMA_T_WARN_MM:
        verdict = "needs_work"
        if not any(m.level in ("warn", "error") for m in msgs):
            msgs.append(
                CoachMessage(
                    "warn",
                    f"정확도 경계 (σ_rot={sigma_rot_deg:.2f}°, σ_t={sigma_t_mm:.1f}mm) — "
                    f"위 항목을 보완하거나 자세를 몇 개 더 캡처하세요.",
                )
            )
    else:
        verdict = "bad"
        msgs.insert(
            0,
            CoachMessage(
                "error",
                f"정확도 부족 (σ_rot={sigma_rot_deg:.2f}°, σ_t={sigma_t_mm:.1f}mm). "
                f"위 추천을 따라 다양한 자세를 더 캡처하세요.",
            ),
        )

    return CoachReport(verdict=verdict, messages=msgs)
