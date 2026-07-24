from __future__ import annotations

import math

import numpy as np
from pydantic import BaseModel
from scipy.spatial.transform import Rotation

from modules.detector.contract import OrientedDetection
from modules.motion.contract import TcpPose
from modules.tasks.core.errors import DetectionNotFound

from .antipodal import AntipodalPair

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]

# ── 그리퍼/전략 상수 (URDF mesh 실측 2026-07-09) ──
_TCP_TO_FIXED_JAW_M = 0.0079  # TCP → 고정 조 안쪽 면 (+y_tool 방향)
_FIXED_JAW_CLEAR_M = 0.005  # 진입 중 고정 조 vs 물체 옆면 여유
_APPROACH_CLEAR_M = 0.06  # pre pose: 파지점에서 접근축 후방 거리
_PLACE_DROP_CLEAR_M = 0.005  # 놓을 때 물체 바닥 vs 적치면 여유 (살짝 위서 release)

# 기준 자세 (tilt=0): 툴 x(approach)→base -z(수직 하향), y(조 축)→base +y.
_TOPDOWN = Rotation.from_matrix([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])

# reachable-orientation 파지 (2026-07-14 재설계 — grasping.md §1):
# top-down 강제 폐기. 조 축은 수평 유지(옆면 antipodal 파지 성립 조건)한 채 approach
# 를 조 축 둘레 tilt 0~±90°(수직→수평) 전체에서 probe — 도달 판정은 motion resolve.
# SO-101 은 먼 리치에서 손목을 수직으로 못 세움: 실물 실패 케이스 시뮬 재현에서
# top-down±40° 가족 전멸, tilt 30~60° base쪽 접근만 도달 (§3.2). 순서 = 선호
# (작은 tilt 우선 — 시야/낙하 안정에 유리, 도달만 되면 수직에 가까운 쪽 채택).
_TILTS_DEG = (0, 15, -15, 30, -30, 45, -45, 60, -60, 75, -75, 90, -90)


class GraspCandidate(BaseModel):
    """접근 후보 1개 — pre(접근)/grasp(파지) 는 같은 자세 (조 축 수평 유지).

    pre 는 grasp 에서 **접근축(툴 x) 후방** — 월드 +z 위가 아니다 (tilt=0 특수
    케이스에서만 위). 진입 = pre→grasp MoveL(접근축 직선), 후퇴 = grasp→pre 역방향
    (grasp-frame 상대 동작, grasping.md §1).

    lateral = 단일 가동 조 보정 (파란 고정 조는 모터로 안 움직임 — TCP 를 파지점
    이 아니라 고정 조 안쪽 면이 [접촉 폭/2 + 여유] 에 오는 자리로 조 축 방향
    횡이동). place 계획이 같은 값을 재사용 (물체가 TCP 에 이 오프셋으로 매달림).
    """

    label: str
    pre: Vec3
    grasp: Vec3
    quat: Quat
    lateral: float


class PlaceCandidate(BaseModel):
    label: str
    pre: Vec3
    place: Vec3
    quat: Quat


def select_target_by_score(
    cands: list[OrientedDetection], *, prompt: str
) -> OrientedDetection:
    """최고 score 후보. 0건이면 DetectionNotFound.

    height prior/하드게이트는 없다 (§10.4-6 폐기) — 관측 충분성의 심판은
    "실행 가능한 antipodal 파지가 섰나" (steps 의 adaptive 관측 루프).
    """
    if not cands:
        raise DetectionNotFound(prompt, candidates=0, reason="검출 0건")
    return max(cands, key=lambda c: c.score)


# (옛 adaptive 뷰 탐색축 view_directions/view_pose_groups 는 2026-07-16 closed-loop
# 전환으로 삭제 — 멀티뷰 관측 이동 자체가 servo 루프로 대체됨. git history 복원 가능.)


def plan_grasp(pairs: list[AntipodalPair]) -> list[GraspCandidate]:
    """접촉쌍 → 접근 후보 가족 — tilt(작은 것부터) × 쌍 × 조 축 flip.

    ⚠ production 소비자 없음 (2026-07-16 closed-loop 전환 — 파지 자세는
    servo.grasp_families 가 담당). scripts/grasp_verify/ 진단 스크립트
    (code_vs_data/reach_probe — post-mortem §9 검증 자산)가 소비해 유지.

    옛 footprint(윗면 윤곽) 파지 폐기 (§10.4-2 — prismatic 전용 추측) — 후보의
    파지점/조 축/폭이 전부 관측 표면의 antipodal 쌍에서 온다. 쌍의 mid 를 단일
    가동 조 TCP 로 환산 (lateral — 고정 조 안쪽 면이 [접촉 폭/2 + 여유] 자리),
    접근축은 조 축 둘레 tilt 스윕 (§3.2 — 작은 팔은 먼 리치에서 수직을 못 세움).
    tool frame: x=접근축, y=조 축, z=x×y. flip = 조 축 반전 (단일 가동 조라
    lateral 방향이 조 축에 묶여 두 방향이 서로 다른 후보 — 도달성도 다름).
    도달/바닥/그리퍼↔물체 충돌 판정은 motion resolve 게이트 몫.
    """
    down = np.array([0.0, 0.0, -1.0])
    out: list[GraspCandidate] = []
    for tilt_deg in _TILTS_DEG:
        for pi, pair in enumerate(pairs):
            for flip in (1.0, -1.0):
                y = np.asarray(pair.jaw_axis, dtype=float) * flip
                approach = Rotation.from_rotvec(y * math.radians(tilt_deg)).apply(down)
                rot_m = np.column_stack([approach, y, np.cross(approach, y)])
                lateral = pair.width / 2 + _FIXED_JAW_CLEAR_M - _TCP_TO_FIXED_JAW_M
                grasp = np.asarray(pair.mid) + rot_m @ np.array([0.0, lateral, 0.0])
                pre = grasp - approach * _APPROACH_CLEAR_M
                qx, qy, qz, qw = (
                    float(v) for v in Rotation.from_matrix(rot_m).as_quat()
                )
                out.append(
                    GraspCandidate(
                        label=(
                            f"pair{pi} tilt={tilt_deg:+d} "
                            f"flip={'+' if flip > 0 else '-'} "
                            f"w={pair.width * 1000:.0f}mm"
                        ),
                        pre=(float(pre[0]), float(pre[1]), float(pre[2])),
                        grasp=(float(grasp[0]), float(grasp[1]), float(grasp[2])),
                        quat=(qx, qy, qz, qw),
                        lateral=lateral,
                    )
                )
    return out


def grasp_ik_groups(plan: list[GraspCandidate]) -> list[list[TcpPose]]:
    """후보별 [pre, grasp] 쌍 — 같은 자세로 접근+파지 둘 다 풀려야 실행 가능."""
    return [
        [
            TcpPose(position=c.pre, quaternion=c.quat),
            TcpPose(position=c.grasp, quaternion=c.quat),
        ]
        for c in plan
    ]


# 놓기 yaw 후보.
# 정렬 방향을 우선 시도하고, 실패하면 자유 방향으로 확장한다.
#
# 정렬:
#   상자 방향에 맞춘 선호 후보.
#   배치 품질을 위해 먼저 시도한다.
#
# 자유:
#   정렬 후보가 모두 실패했을 때 사용하는 확장 후보.
#   더 많은 yaw 방향을 제공해 IK 도달 가능성을 높인다.
#
# 같은 위치라도 yaw가 달라지면 SO-101의 손목 자세(IK 해)가 달라질 수 있으므로
# 180° 등 방향 등가인 경우도 별도 후보로 유지한다.
_PLACE_ALIGNED_YAW_OFFSETS_DEG = (0.0, 180.0, 90.0, 270.0)

_PLACE_FREE_YAW_OFFSETS_DEG = (30.0, -30.0, 60.0, -60.0, 120.0, -120.0, 150.0, -150.0)

# 놓기 tilt 후보.
# 집기보다 후보 수를 줄여 성능을 확보하면서, 일반적인 접근 방향 범위를 커버한다.
# 순서는 선호도이며 resolve 단계에서 실제 도달 가능한 자세를 선택한다.
#
# 0°: 수직 접근 우선
# ±30~60°: 접근성 확보용 기울기 후보
_PLACE_TILTS_DEG = (0, 30, -30, 45, -45, 60, -60)


def plan_place(spot: OrientedDetection) -> list[PlaceCandidate]:
    """적치 후보 생성 (정렬 yaw).

    상자 방향에 맞춘 yaw 후보와 tilt 후보를 조합해 TCP 후보를 생성한다.
    정렬된 배치를 우선하지만, 최종 선택은 motion resolve의 도달성 판정에 따른다.
    """
    return _place_candidates(spot, yaw_offsets_deg=_PLACE_ALIGNED_YAW_OFFSETS_DEG)


def plan_place_free(spot: OrientedDetection) -> list[PlaceCandidate]:
    """적치 후보 생성 (자유 yaw).

    정렬 yaw 후보가 모두 실패한 경우 사용하는 폴백 후보.
    배치 방향보다 로봇 접근 가능성을 우선한다.
    """
    return _place_candidates(spot, yaw_offsets_deg=_PLACE_FREE_YAW_OFFSETS_DEG)


def _place_candidates(
    spot: OrientedDetection,
    *,
    yaw_offsets_deg: tuple[float, ...],
) -> list[PlaceCandidate]:
    """적치 위치와 자세 후보를 생성한다.

    검출된 적치 대상 중심 위에 place pose를 만들고,
    yaw/tilt 후보를 조합해 접근 가능한 TCP 자세 후보를 생성한다.

    현재는 물체 크기와 파지 오프셋을 고려하지 않고,
    상자 중심 위 고정 높이에 내려놓는 단순 모델을 사용한다.
    물체가 크거나 좁은 공간 적치가 필요한 경우 추후 lateral offset,
    물체 높이 보정 등을 추가할 수 있다.
    """
    sx, sy, spot_top_z = spot.position
    place_z = spot_top_z + _PLACE_DROP_CLEAR_M
    approach_dist = _APPROACH_CLEAR_M

    # tilt/yaw 조합으로 여러 접근 자세 후보 생성
    # 리스트 순서는 resolve에서 먼저 시도할 선호 순서를 결정한다.
    out: list[PlaceCandidate] = []
    for tilt_deg in _PLACE_TILTS_DEG:
        for off_deg in yaw_offsets_deg:
            yaw = spot.grasp_yaw + math.radians(off_deg)
            rot = (
                Rotation.from_euler("z", yaw)
                * _TOPDOWN
                * Rotation.from_euler("y", math.radians(tilt_deg))
            )
            qx, qy, qz, qw = (float(v) for v in rot.as_quat())
            ax, ay, az = (float(v) for v in rot.apply([1.0, 0.0, 0.0]))
            out.append(
                PlaceCandidate(
                    label=f"tilt={tilt_deg:+d} yaw={math.degrees(yaw):.0f}",
                    pre=(
                        sx - ax * approach_dist,
                        sy - ay * approach_dist,
                        place_z - az * approach_dist,
                    ),
                    place=(sx, sy, place_z),
                    quat=(qx, qy, qz, qw),
                )
            )
    return out


def place_ik_groups(plan: list[PlaceCandidate]) -> list[list[TcpPose]]:
    return [
        [
            TcpPose(position=c.pre, quaternion=c.quat),
            TcpPose(position=c.place, quaternion=c.quat),
        ]
        for c in plan
    ]
