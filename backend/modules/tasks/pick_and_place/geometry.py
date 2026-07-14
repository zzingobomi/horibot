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

# reachable-orientation 파지 (2026-07-14 재설계 — docs/grasp_redesign_journey.md §5.3):
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
    (grasp-frame 상대 동작, docs/grasp_redesign_journey.md §5.4).

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


# ─── 타깃 중심 멀티뷰 — adaptive 뷰 탐색축 (§10.4-1) ─────────────────
#
# 고정 궤도(옛 target_view_poses 6점) 폐기 — 반경/고도/방위는 **탐색축**이고,
# 어느 뷰가 도달·안전한지는 motion resolve(IK+self+floor+장애물)가 판정한다.
# 미리 티칭한 타깃-중심 자세는 존재 불가 (타깃 위치가 매번 다름) → 검출 위치
# 기준으로 계산. search waypoint 는 1단계 광역 탐색 전용 (재활용 X).

# 카메라-타깃 거리 — D405 depth 유효 최소(~7cm) 여유 + 좁은 workspace 절충.
_VIEW_RADII_M = (0.16, 0.13)
# 고도(수평 기준): 사선 — top-down 은 옆면 depth 가 안 잡혀 멀티뷰 의미 없음.
_VIEW_ELEVATIONS_DEG = (55.0, 40.0, 70.0)
# 방위 오프셋 (타깃→base 방위 기준) — spread-first: antipodal 은 서로 떨어진
# 방위 관측이 필요해 (§10.3-B 마주 보는 면), 한 방위 근처를 파기보다 벌어진
# 방위를 먼저 시도해야 파지 성립이 빠르다. base 쪽(0°)이 도달 가능성 최고 (§3.2).
_VIEW_AZIMUTH_OFFSETS_DEG = (
    0.0, 120.0, -120.0, 60.0, -60.0, 180.0, 90.0, -90.0, 30.0, -30.0, 150.0, -150.0
)
# 카메라 roll (광축 둘레) — 관측엔 자유 (광축이 타깃만 향하면 됨). 자연 roll
# (이미지 up≈월드 up) 우선, 안 닿는 뷰를 다른 roll 이 살린다 (§10.2 sim —
# roll 스윕이 도달 뷰 커버리지를 방위 180~300° 로 늘린 재료).
_VIEW_ROLLS_DEG = (0.0, 60.0, -60.0, 120.0, -120.0, 180.0)


def view_directions(target: Vec3) -> list[tuple[float, float, float]]:
    """관측 뷰 방향 후보 (radius_m, elev_deg, az_rad) — 시도 순서가 곧 선호.

    같은 (반경, 고도) 층에서 방위를 spread-first 로 전부 돈 뒤 다음 층 —
    도달 불가 뷰는 소비자(resolve 판정)가 스킵하므로 목록은 넉넉하게, 정지는
    "파지가 섰나"(adaptive)가 담당.
    """
    base_az = math.atan2(-float(target[1]), -float(target[0]))  # 타깃→base 방위
    out: list[tuple[float, float, float]] = []
    for radius in _VIEW_RADII_M:
        for elev in _VIEW_ELEVATIONS_DEG:
            for off in _VIEW_AZIMUTH_OFFSETS_DEG:
                out.append((radius, elev, base_az + math.radians(off)))
    return out


def view_pose_groups(
    target: Vec3,
    r_cam2ee: list[list[float]] | np.ndarray,
    t_cam2ee: list | np.ndarray,
    *,
    radius_m: float,
    elev_deg: float,
    az_rad: float,
) -> list[tuple[Vec3, Quat]]:
    """한 뷰 방향의 **TCP pose** 후보들 — roll 변형 순 (자연 roll 우선).

    카메라 pose: 위치 = 타깃 + 반경·(방위,고도) 단위벡터, 광축(+z_cam)이 타깃을
    향함. TCP 변환은 hand_eye(cam→ee): R_be = R_bc·R_ceᵀ, t_be = cam_pos − R_be·t_ce.
    도달·충돌 판정은 소비자 몫 — RESOLVE_REACHABLE 에 [pose] 그룹으로 묶어
    첫 가용 roll 을 채택한다 (§10.4-1: IK+self 만이 아니라 floor+장애물까지).
    """
    r_ce = np.asarray(r_cam2ee, dtype=float)
    t_ce = np.asarray(t_cam2ee, dtype=float).reshape(3)
    tgt = np.asarray(target, dtype=float)
    el = math.radians(elev_deg)
    up_dir = np.array(
        [
            math.cos(az_rad) * math.cos(el),
            math.sin(az_rad) * math.cos(el),
            math.sin(el),
        ]
    )
    cam_pos = tgt + radius_m * up_dir
    z_c = -up_dir  # 광축 → 타깃
    down = np.array([0.0, 0.0, -1.0])  # 자연 roll: 이미지 y(down) ≈ 월드 아래
    y0 = down - float(down @ z_c) * z_c
    norm = float(np.linalg.norm(y0))
    if norm < 1e-6:  # 수직 내려보기 축퇴 (elev≈90°) — 방위 방향으로
        y0 = np.array([math.cos(az_rad), math.sin(az_rad), 0.0])
    else:
        y0 = y0 / norm
    x0 = np.cross(y0, z_c)  # 우수계: x×y=z

    out: list[tuple[Vec3, Quat]] = []
    for roll_deg in _VIEW_ROLLS_DEG:
        rr = math.radians(roll_deg)
        x_c = math.cos(rr) * x0 + math.sin(rr) * y0  # z_c 둘레 roll 회전
        y_c = np.cross(z_c, x_c)
        r_bc = np.column_stack([x_c, y_c, z_c])
        r_be = r_bc @ r_ce.T
        t_be = cam_pos - r_be @ t_ce
        qx, qy, qz, qw = (float(v) for v in Rotation.from_matrix(r_be).as_quat())
        out.append(
            (
                (float(t_be[0]), float(t_be[1]), float(t_be[2])),
                (qx, qy, qz, qw),
            )
        )
    return out


def plan_grasp(pairs: list[AntipodalPair]) -> list[GraspCandidate]:
    """접촉쌍 → 접근 후보 가족 — tilt(작은 것부터) × 쌍 × 조 축 flip.

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


def _oriented_family(
    yaw_options: tuple[tuple[float, float], ...],
) -> list[tuple[str, Rotation, float]]:
    """tilt × yaw × flip 자세 가족 — (label, rot, across). place 접근 후보용."""
    out: list[tuple[str, Rotation, float]] = []
    for tilt_deg in _TILTS_DEG:
        for yaw_base, across in yaw_options:
            for flip in (0.0, math.pi):  # 조 대칭 — 180° flip 은 적치 등가
                rot = (
                    Rotation.from_euler("z", yaw_base + flip)
                    * _TOPDOWN
                    * Rotation.from_euler("y", math.radians(tilt_deg))
                )
                label = (
                    f"tilt={tilt_deg:+d} yaw={math.degrees(yaw_base):.0f} "
                    f"flip={math.degrees(flip):.0f}"
                )
                out.append((label, rot, across))
    return out


def plan_place(
    spot: OrientedDetection, *, held: OrientedDetection, lateral: float
) -> list[PlaceCandidate]:
    """적치 후보 생성 — spot 상면 중심 위에 held 물체 바닥이 오는 TCP 자리.

    TCP 는 파지 시 held 중간 높이 부근을 물었으므로 release z = spot 상면 +
    held/2 + 여유. lateral = 파지 때 확정된 보정값 재사용 (물체가 TCP 에 그
    오프셋으로 매달림) — 같은 tool-frame 오프셋을 적용해야 물체가 spot 위에 온다.
    """
    sx, sy, spot_top_z = spot.position
    place_z = spot_top_z + held.height * 0.5 + _PLACE_DROP_CLEAR_M
    approach_dist = _APPROACH_CLEAR_M + held.height * 0.5
    # 적치 yaw 는 자유 (물체를 어느 방향으로 놓아도 됨) — 0/90° 가족만 probe.
    yaw_options = ((0.0, 0.0), (math.pi / 2, 0.0))

    out: list[PlaceCandidate] = []
    for label, rot, _ in _oriented_family(yaw_options):
        off = rot.apply([0.0, lateral, 0.0])
        qx, qy, qz, qw = (float(v) for v in rot.as_quat())
        px, py = sx + float(off[0]), sy + float(off[1])
        # pre = place 에서 접근축 후방 (plan_grasp 와 동일 원리 — §5.4)
        ax, ay, az = (float(v) for v in rot.apply([1.0, 0.0, 0.0]))
        out.append(
            PlaceCandidate(
                label=label,
                pre=(
                    px - ax * approach_dist,
                    py - ay * approach_dist,
                    place_z - az * approach_dist,
                ),
                place=(px, py, place_z),
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
