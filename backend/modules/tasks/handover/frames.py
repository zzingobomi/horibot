"""handover 의 **객체-무관** 기하 — frame 변환 + 랑데부(두 팔 공통 워크스페이스).

물체(펜/큐브/봉) 모양에 의존하지 않는 부분만 여기 모은다 (물체별 파지 기하는
block.py). 하드웨어/wire 0 — 오피스 단위테스트 대상.

책임:
  - frame 변환 (world=so101 base ↔ robot base — robots.yaml base_pose 규약)
  - 랑데부 후보: 두 팔 공통 워크스페이스(workcell ROI ∩) 안쪽 격자 (흉터 5 —
    "standoff 가 먼저 죽는" 워크스페이스 전멸의 예방. 히트맵 실측 전 기하 근사).
"""

from __future__ import annotations

import math

from modules.shared_config.contract import WorkcellRoi

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


def world_dir_to_robot(d: Vec3, base: BasePose) -> Vec3:
    """world **방향벡터** → robot base 방향 (평행이동 없이 yaw 역회전만).

    위치와 달리 방향은 base 원점 이동에 불변이다 — 봉 축(노출 방향 w)처럼
    방향을 robot frame 자세 구성에 넘길 때 쓴다 (위치용 world_to_robot 을 쓰면
    base 오프셋만큼 틀린 축이 된다)."""
    c, s = math.cos(base.yaw_rad), math.sin(base.yaw_rad)
    return (c * d[0] + s * d[1], -s * d[0] + c * d[1], d[2])


def yaw_to_world(yaw_robot: float, base: BasePose) -> float:
    """robot frame 평면각 → world 평면각 (base yaw 가산)."""
    return yaw_robot + base.yaw_rad


# ─── 랑데부 (두 팔 공통 워크스페이스) ────────────────────────────────


def rendezvous_candidates(
    roi_so: WorkcellRoi,
    roi_omx: WorkcellRoi,
    base_omx: BasePose,
    z_values: tuple[float, ...],
    *,
    step_m: float = 0.03,
    limit: int = 8,
    prefer_point: Vec2 | None = None,
) -> list[Vec3]:
    """world 격자 중 so101 ROI ∩ omx ROI(omx frame 변환) 교집합 점들 — 제시
    파지점(omx TCP)의 후보. 선호순: z_values 순서 → **prefer_point 지정 시 그 world
    xy 에 가까운 순** (so101 도달이 razor-thin 이라 도달 robust 실측 지점을 직접
    겨냥 — steps._RENDEZVOUS_PREFER_XY) → 아니면 중심.

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
    if prefer_point is not None:
        px0, py0 = prefer_point
        hits.sort(key=lambda h: (h[0], math.hypot(h[1] - px0, h[2] - py0)))
    else:
        cx = sum(h[1] for h in hits) / len(hits)
        cy = sum(h[2] for h in hits) / len(hits)
        hits.sort(key=lambda h: (h[0], math.hypot(h[1] - cx, h[2] - cy)))
    return [(x, y, z) for _zi, x, y, z in hits[:limit]]


def _grid(lo: float, hi: float, step: float) -> list[float]:
    n = max(1, int((hi - lo) / step))
    return [lo + (hi - lo) * k / n for k in range(n + 1)]
