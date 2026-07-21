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
                approach = Rotation.from_rotvec(
                    y * math.radians(tilt_deg)
                ).apply(down)
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


# 놓기 yaw 후보 — 정렬 가족 우선, 자유 가족 폴백 (2026-07-14 실물 진단):
# 놓기 yaw 는 본질적으로 자유(물체를 어느 방향으로 내려놓아도 됨)이고 상자 방위
# 정렬은 **선호**일 뿐이다. 그런데 SO-101 은 한 지점에서 닿는 손목 자세가 희박해
# — 집기는 antipodal 쌍이 조 방향을 수십 개 공급(쌍 10×flip 2×tilt 13=260)해서
# 그물이 넓은데, 놓기를 yaw 2개로 못 박으면 "위치 통과 26/26, 자세 IK 실패 26"
# (실물 로그 — 지점은 닿는데 내민 방향이 전부 사각). 그래서 두 가족:
# ① 정렬 = 상자 방위 + {0,180,90,270}° (180° 는 위치 등가지만 조·롤 방향이 달라
#   IK 가 다른 별개 자세 — 등가로 보고 떨군 게 커버리지 반토막 회귀였다)
# ② 자유 = 30° 격자의 나머지 8방향 — 정렬 전멸 시에만 (도달이 정렬을 이긴다).
_PLACE_ALIGNED_YAW_OFFSETS_DEG = (0.0, 180.0, 90.0, 270.0)
_PLACE_FREE_YAW_OFFSETS_DEG = (
    30.0, -30.0, 60.0, -60.0, 120.0, -120.0, 150.0, -150.0
)
# 놓기 tilt 사다리 — 파지(13단)보다 성기게 (perf, 2026-07-14 실측): 전멸 가족은
# 전 그룹이 풀예산 IK 를 태워 후보 수에 정비례로 느리다 (52+104 gr → 도합 수 분).
# resolve 는 "첫 통과" 를 고르므로 15° 해상도보다 도달 띠 커버(0~±60°)가 본질 —
# ±15 랑간 제거, ±75/±90(수평 삽입 = 상자 상면 쓸기 위험)도 놓기에선 제외.
# 실측 채택 이력: 파지 +30 / 놓기 +45 — 전부 이 사다리 안.
_PLACE_TILTS_DEG = (0, 30, -30, 45, -45, 60, -60)


def plan_place(spot: OrientedDetection) -> list[PlaceCandidate]:
    """적치 후보 (정렬 가족) — 상자(spot) 상면 **정중앙** 위 TCP 자리.

    yaw 는 **상자 방위(spot.grasp_yaw)** 정렬 4방향 (삐뚤어진 상자면 그 방향으로 —
    2방향은 자세 그물이 성겨 위치 닿아도 자세 전멸, 상수 주석). tilt 는 0~±60°
    도달 띠(_PLACE_TILTS_DEG — 소각 ±30 상한이 SO-101 top-down 사각지대 §3.2 에
    꽂혀 전멸, 사다리는 perf 로 성기게). 순서가 곧 선호: 수직(0°)·정렬(0°/180°)
    먼저. 전멸 시 소비자가 plan_place_free 폴백 (도달 판정은 motion resolve)."""
    return _place_candidates(spot, yaw_offsets_deg=_PLACE_ALIGNED_YAW_OFFSETS_DEG)


def plan_place_free(spot: OrientedDetection) -> list[PlaceCandidate]:
    """적치 후보 (자유 yaw 가족) — 정렬 가족 전멸 시 폴백 (30° 격자 나머지 8방향).

    정렬은 선호일 뿐 — 닿는 자유 yaw 가 안 닿는 정렬 yaw 를 이긴다 (놓기 실패로
    task 전체가 죽는 것보다 삐딱하게라도 상자 위에 놓는 게 낫다)."""
    return _place_candidates(spot, yaw_offsets_deg=_PLACE_FREE_YAW_OFFSETS_DEG)


def _place_candidates(
    spot: OrientedDetection,
    *,
    yaw_offsets_deg: tuple[float, ...],
) -> list[PlaceCandidate]:
    # 2026-07-21 단순화 (사용자 지시): 물건 폭(옆 치우침)·높이(손 밑 튀어나옴)는
    # 무시하고 **상자 정중앙(sx,sy) 위 고정 여유 높이**에 놓고 연다.
    # ⚠ TODO(추후 고려 — 빡빡한 상자/큰 물건이면 다시 반영):
    #   ① 물건이 조에 반폭만큼 옆으로 매달림 → 상자 중심에 놓으려면 lateral 보정
    #   ② 물건이 손 밑으로 height/2 만큼 튀어나옴 → 드롭 높이/접근 거리 반영
    #   (걷어내기 전 식 = git history: place_z += held.height/2 + clear,
    #    approach_dist += held.height/2, off = rot.apply([0, lateral, 0])).
    sx, sy, spot_top_z = spot.position
    place_z = spot_top_z + _PLACE_DROP_CLEAR_M  # 상자 상면 바로 위 (조금 위)
    approach_dist = _APPROACH_CLEAR_M

    out: list[PlaceCandidate] = []
    for tilt_deg in _PLACE_TILTS_DEG:  # 수직(0°) 먼저 (선호), 성긴 사다리 (perf)
        for off_deg in yaw_offsets_deg:  # 정렬 가까운 순 (선호)
            yaw = spot.grasp_yaw + math.radians(off_deg)
            rot = (
                Rotation.from_euler("z", yaw)
                * _TOPDOWN
                * Rotation.from_euler("y", math.radians(tilt_deg))
            )
            qx, qy, qz, qw = (float(v) for v in rot.as_quat())
            # pre = place 에서 접근축 후방 (tilt=0 이면 곧 월드 +z 바로 위)
            ax, ay, az = (float(v) for v in rot.apply([1.0, 0.0, 0.0]))
            out.append(
                PlaceCandidate(
                    label=f"tilt={tilt_deg:+d} yaw={math.degrees(yaw):.0f}",
                    pre=(
                        sx - ax * approach_dist,
                        sy - ay * approach_dist,
                        place_z - az * approach_dist,
                    ),
                    place=(sx, sy, place_z),  # 상자 정중앙
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
