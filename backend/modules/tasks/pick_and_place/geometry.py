from __future__ import annotations

import math

from pydantic import BaseModel
from scipy.spatial.transform import Rotation

from modules.detector.contract import OrientedDetection
from modules.motion.contract import TcpPose
from modules.tasks.core.errors import DetectionNotFound

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]

# ── 그리퍼/전략 상수 (URDF mesh 실측 2026-07-09) ──
_TCP_TO_FIXED_JAW_M = 0.0079  # TCP → 고정 조 안쪽 면 (+y_tool 방향)
_FIXED_JAW_CLEAR_M = 0.005  # 하강 중 고정 조 vs 물체 옆면 여유
_APPROACH_CLEAR_M = 0.06  # pre pose: 대상 윗면 위 접근 높이
_FINGER_TABLE_CLEAR_M = 0.008  # 손끝 vs 테이블 여유
_PLACE_DROP_CLEAR_M = 0.005  # 놓을 때 물체 바닥 vs 적치면 여유 (살짝 위서 release)

# height prior — 실측 2.3cm 큐브가 depth 노이즈로 경계 탈락해 하한 느슨 (2026-07-09).
# 상한은 테이블 모서리 ring 에 낮은 바닥이 섞여 height 부풀 때 잡는 안전망.
_MIN_HEIGHT_M = 0.015
_MAX_HEIGHT_M = 0.15

# top-down 파지 자세: 툴 x(approach)→base -z(수직 하향), y(조 축)→base +y.
_TOPDOWN = Rotation.from_matrix([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])

# 수직 approach 는 이 팔이 높은 z 에서 못 냄 (tilt<15° 는 z≤0.038 만 — 관절 리밋,
# 2026-07-09 FK 샘플링). 조 축은 수평 유지(파지 성립 조건)한 채 approach 만 조 축
# 둘레로 기울인 후보를 tilt 오름차순 probe — 판정은 motion 배치 IK (모션 0).
_TILTS_DEG = (0, 15, -15, 25, -25, 30, -30, 35, -35, 40, -40)


class GraspCandidate(BaseModel):
    """접근 후보 1개 — pre(접근)/grasp(파지) 는 같은 자세 (조 축 수평 유지).

    lateral = 단일 가동 조 보정 (파란 고정 조는 모터로 안 움직임 — TCP 를 물체
    중심이 아니라 고정 조 안쪽 면이 [물 변 + 여유] 에 오는 자리로 조 축 방향 횡이동).
    place 계획이 같은 값을 재사용 (물체가 TCP 에 이 오프셋으로 매달려 있음).
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


def select_pick_target(
    cands: list[OrientedDetection], *, prompt: str
) -> OrientedDetection:
    """height prior 통과 후보 중 최고 score. 통과 0 이면 DetectionNotFound."""
    ok = [c for c in cands if _MIN_HEIGHT_M <= c.height <= _MAX_HEIGHT_M]
    if not ok:
        reason = (
            "검출 0건"
            if not cands
            else f"height prior({_MIN_HEIGHT_M}~{_MAX_HEIGHT_M}m) 통과 0"
        )
        raise DetectionNotFound(prompt, candidates=len(cands), reason=reason)
    return max(ok, key=lambda c: c.score)


def _oriented_family(
    yaw_options: tuple[tuple[float, float], ...],
) -> list[tuple[str, Rotation, float]]:
    """tilt × yaw × flip 자세 가족 — (label, rot, across). across = 조가 무는 변."""
    out: list[tuple[str, Rotation, float]] = []
    for tilt_deg in _TILTS_DEG:
        for yaw_base, across in yaw_options:
            for flip in (0.0, math.pi):  # 조 대칭 — 180° flip 은 파지 등가
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


def plan_grasp(target: OrientedDetection) -> list[GraspCandidate]:
    """파지 접근 후보 생성 — tilt(작은 것부터) × yaw 가족 × flip.

    yaw 가족 = grasp_yaw(짧은 변 물기) + 90°(긴 변 물기) — 정사각형 근처 footprint
    는 minAreaRect yaw 가 노이즈 임의값이라 한 yaw 강제가 도달성 전멸을 만듦
    (2026-07-09 yaw=84° 사건). 둘 다 유효 파지.
    """
    x, y, top_z = target.position
    grasp_z = max(top_z - target.height * 0.5, target.base_z + _FINGER_TABLE_CLEAR_M)
    pre_z = top_z + _APPROACH_CLEAR_M
    long_side, short_side = target.footprint
    yaw_options = (
        (target.grasp_yaw, short_side),  # 조가 짧은 변을 가로질러 묾
        (target.grasp_yaw + math.pi / 2, long_side),  # 90° 돌려 긴 변을 묾
    )

    out: list[GraspCandidate] = []
    for label, rot, across in _oriented_family(yaw_options):
        lateral = across / 2 + _FIXED_JAW_CLEAR_M - _TCP_TO_FIXED_JAW_M
        off = rot.apply([0.0, lateral, 0.0])
        qx, qy, qz, qw = (float(v) for v in rot.as_quat())
        gx, gy = x + float(off[0]), y + float(off[1])
        out.append(
            GraspCandidate(
                label=label,
                pre=(gx, gy, pre_z),
                grasp=(gx, gy, grasp_z),
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


def plan_place(
    spot: OrientedDetection, *, held: OrientedDetection, lateral: float
) -> list[PlaceCandidate]:
    """적치 후보 생성 — spot 상면 중심 위에 held 물체 바닥이 오는 TCP 자리.

    TCP 는 파지 시 held 중간 높이를 물었으므로 release z = spot 상면 + held/2 + 여유.
    lateral = 파지 때 확정된 보정값 재사용 (물체가 TCP 에 그 오프셋으로 매달림) —
    같은 tool-frame 오프셋을 적용해야 물체 중심이 spot 중심에 온다.
    """
    sx, sy, spot_top_z = spot.position
    place_z = spot_top_z + held.height * 0.5 + _PLACE_DROP_CLEAR_M
    pre_z = place_z + _APPROACH_CLEAR_M
    # 적치 yaw 는 자유 (물체를 어느 방향으로 놓아도 됨) — 0/90° 가족만 probe.
    yaw_options = ((0.0, 0.0), (math.pi / 2, 0.0))

    out: list[PlaceCandidate] = []
    for label, rot, _ in _oriented_family(yaw_options):
        off = rot.apply([0.0, lateral, 0.0])
        qx, qy, qz, qw = (float(v) for v in rot.as_quat())
        px, py = sx + float(off[0]), sy + float(off[1])
        out.append(
            PlaceCandidate(
                label=label,
                pre=(px, py, pre_z),
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
