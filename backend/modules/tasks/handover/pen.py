"""펜(얇고 긴 물체) handover 의 순수 기하 — 하드웨어/wire 0, 오피스 단위테스트 대상.

책임 (docs/omx_handover_prep.md §1.1 — 파지점이 handover 전체를 좌우하는 커플링):
  - frame 변환 (world=so101 base ↔ robot base — robots.yaml base_pose 규약)
  - 펜 끝점/파지점 선택: omx 는 so101 에서 **먼 끝** 쪽 grasp_frac 지점을 물고
    노출부를 so101 쪽으로 남긴다. 안정성(모멘트 암) ↔ 노출 길이 트레이드오프.
  - **짧은 펜 명시 실패**: (파지점 + 조 폭 + 최소 노출) > 펜 길이 → 계획 단계
    에서 사유 있는 실패 (§1.1 신규 실패 모드 — 침묵 진행 금지).
  - 랑데부 후보: 두 팔 공통 워크스페이스(workcell ROI ∩) 안쪽 격자 (흉터 5 —
    "standoff 가 먼저 죽는" 워크스페이스 전멸의 예방. 히트맵 실측 전 기하 근사).

mono 검출 전제: 펜은 z=table 평면 위 → 기하는 전부 XY 평면 (끝점/방향/길이).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from modules.shared_config.contract import WorkcellRoi
from modules.tasks.core.errors import TaskError

from .collision import BasePose

Vec2 = tuple[float, float]
Vec3 = tuple[float, float, float]


# ─── frame 변환 (world = so101 base — robots.yaml base_pose 규약) ─────


def world_to_robot(p: Vec3, base: BasePose) -> Vec3:
    """world(so101 base) 좌표 → robot base 좌표 (base_pose 역변환)."""
    c, s = math.cos(base.yaw_rad), math.sin(base.yaw_rad)
    dx, dy, dz = p[0] - base.x, p[1] - base.y, p[2] - base.z
    return (c * dx + s * dy, -s * dx + c * dy, dz)


def robot_to_world(p: Vec3, base: BasePose) -> Vec3:
    """robot base 좌표 → world(so101 base) 좌표."""
    c, s = math.cos(base.yaw_rad), math.sin(base.yaw_rad)
    return (
        base.x + c * p[0] - s * p[1],
        base.y + s * p[0] + c * p[1],
        base.z + p[2],
    )


def yaw_to_world(yaw_robot: float, base: BasePose) -> float:
    """robot frame 평면각 → world 평면각 (base yaw 가산)."""
    return yaw_robot + base.yaw_rad


# ─── 펜 끝점 / 파지점 ────────────────────────────────────────────────


def pen_endpoints(center_xy: Vec2, yaw_rad: float, length_m: float) -> tuple[Vec2, Vec2]:
    """OBB (중심, 긴 축 yaw, 길이) → 양 끝점. mono z=0 검출의 footprint 소비."""
    hx = 0.5 * length_m * math.cos(yaw_rad)
    hy = 0.5 * length_m * math.sin(yaw_rad)
    return (
        (center_xy[0] - hx, center_xy[1] - hy),
        (center_xy[0] + hx, center_xy[1] + hy),
    )


@dataclass(frozen=True, slots=True)
class PenGrasp:
    """omx 파지 계획의 기하 산출 (robot frame — 호출자가 넣은 frame 그대로).

    u = 파지점 → 노출 끝(so101 쪽) 단위벡터: pick 의 jaw yaw(=J5 roll)와 present
    의 "펜을 so101 로 향하게" 가 전부 이 벡터에서 파생된다 (tool z ∥ u 규약).
    exposed_center_offset_m = 파지 중심 → 노출 세그먼트 중심 (so101 겨냥점 산출).
    """

    grasp_xy: Vec2
    tip_far: Vec2  # omx 쪽 (so101 에서 먼) 끝
    tip_near: Vec2  # so101 쪽 (노출) 끝
    u: Vec2  # 단위벡터 grasp → tip_near
    length_m: float
    width_m: float
    exposed_len_m: float
    exposed_center_offset_m: float


def plan_pen_grasp(
    center_xy: Vec2,
    yaw_rad: float,
    length_m: float,
    width_m: float,
    toward_xy: Vec2,
    *,
    grasp_frac: float,
    jaw_width_m: float,
    min_exposed_m: float,
) -> PenGrasp:
    """검출 OBB + so101 방향 → 파지점/노출 기하. 노출 부족은 **명시 실패**.

    grasp_frac: 먼 끝에서 이 비율 지점을 문다 (~0.25–0.35 실물 튜닝 — §1.1:
    너무 끝 = 모멘트 암↑ 회전/빠짐, 너무 가운데 = so101 노출 부족).
    jaw_width_m: omx 조가 펜 축 방향으로 차지하는 폭 (노출 계산에서 차감).
    min_exposed_m: so101 최소 파지 길이 + margin — 미만이면 handover 불가.
    """
    e1, e2 = pen_endpoints(center_xy, yaw_rad, length_m)
    d1 = math.hypot(e1[0] - toward_xy[0], e1[1] - toward_xy[1])
    d2 = math.hypot(e2[0] - toward_xy[0], e2[1] - toward_xy[1])
    tip_far, tip_near = (e1, e2) if d1 >= d2 else (e2, e1)
    ux = (tip_near[0] - tip_far[0]) / length_m
    uy = (tip_near[1] - tip_far[1]) / length_m
    g = grasp_frac * length_m
    exposed = length_m - g - jaw_width_m / 2.0
    if exposed < min_exposed_m:
        raise TaskError(
            f"펜이 짧아 handover 불가 — 길이 {length_m * 100:.1f}cm 에서 파지점 "
            f"{grasp_frac:.0%}({g * 100:.1f}cm) + 조 폭 절반 "
            f"{jaw_width_m / 2 * 100:.1f}cm 를 빼면 노출 {exposed * 100:.1f}cm < "
            f"필요 {min_exposed_m * 100:.1f}cm. 더 긴 물체로 교체하거나 파지 "
            "비율(_PEN_GRASP_FRAC)을 낮추세요"
        )
    return PenGrasp(
        grasp_xy=(tip_far[0] + ux * g, tip_far[1] + uy * g),
        tip_far=tip_far,
        tip_near=tip_near,
        u=(ux, uy),
        length_m=length_m,
        width_m=width_m,
        exposed_len_m=exposed,
        exposed_center_offset_m=jaw_width_m / 2.0 + exposed / 2.0,
    )


# ─── 랑데부 (두 팔 공통 워크스페이스) ────────────────────────────────


def rendezvous_candidates(
    roi_so: WorkcellRoi,
    roi_omx: WorkcellRoi,
    base_omx: BasePose,
    z_values: tuple[float, ...],
    *,
    step_m: float = 0.03,
    limit: int = 8,
    prefer_r_so: float | None = None,
) -> list[Vec3]:
    """world 격자 중 so101 ROI ∩ omx ROI(omx frame 변환) 교집합 점들 — 제시
    파지점(omx TCP)의 후보. 선호순: z_values 순서 → prefer_r_so 지정 시 so101
    원점 거리와의 차 (수취 sweet 반경 — so101 공중 도달이 좁은 환대 실측,
    steps._RENDEZVOUS_R_SO_M 주석), 미지정 시 교집합 중심 근접.

    흉터 5 (워크스페이스 전멸 — standoff 가 먼저 죽음) 예방: 랑데부를 애초에
    두 셀의 공통 영역 **안쪽**에 배치. 실 도달성 판정은 여전히 motion resolve
    몫 — 여기는 후보 생성/선호 정렬만. 교집합이 비면 [] (호출자가 명시 실패).
    """
    hits: list[tuple[int, float, float, float]] = []  # (z_idx, x, y, z)
    xs = _grid(roi_so.x_min, roi_so.x_max, step_m)
    ys = _grid(roi_so.y_min, roi_so.y_max, step_m)
    for zi, z in enumerate(z_values):
        if not (roi_so.z_min <= z <= roi_so.z_max):
            continue
        for x in xs:
            for y in ys:
                px, py, pz = world_to_robot((x, y, z), base_omx)
                if (
                    roi_omx.x_min <= px <= roi_omx.x_max
                    and roi_omx.y_min <= py <= roi_omx.y_max
                    and roi_omx.z_min <= pz <= roi_omx.z_max
                ):
                    hits.append((zi, x, y, z))
    if not hits:
        return []
    if prefer_r_so is not None:
        hits.sort(key=lambda h: (
            h[0], abs(math.hypot(h[1], h[2]) - prefer_r_so),
        ))
    else:
        cx = sum(h[1] for h in hits) / len(hits)
        cy = sum(h[2] for h in hits) / len(hits)
        hits.sort(key=lambda h: (h[0], math.hypot(h[1] - cx, h[2] - cy)))
    return [(x, y, z) for _zi, x, y, z in hits[:limit]]


def _grid(lo: float, hi: float, step: float) -> list[float]:
    n = max(1, int((hi - lo) / step))
    return [lo + (hi - lo) * k / n for k in range(n + 1)]
