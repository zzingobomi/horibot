"""봉(8×2×2cm 주황 각봉) handover 의 순수 기하 — 하드웨어/wire 0, 단위테스트 대상.

⚠ 2026-07-27 신설 (큐브→봉 전환, 사용자 지시 — cube.py 삭제, git history 복원
가능). 2cm 큐브는 두 그리퍼가 같은 점에 모여야 해서 도달(razor-thin)↔가림이
y축에서 정면충돌했다 (M2/M3 — docs/omx_handover_realtest_handoff.md §T.3).
봉은 파지점을 두 개(양 끝)로 쪼개 그 충돌 자체를 없앤다.

채택 설계 = **hang(z↑) 매달기 제시** (수직 봉 기하는 probe 결합 스윕 그대로 —
수평(접선)족은 so101 공중 수취 도달 0. 2026-07-27 밤 개정: 옛 B/down(tool z ↓)
은 ZYYYX 기구학상 J5=±180 손목 뒤집기가 유일해라 실물에서 웹캠 USB 케이블이
감김 → 사용자 토크오프 데모(J5≈0) 실측으로 자세족 교체):
  - omx 가 봉의 **한쪽 끝**을 top-down 으로 물고 (tool z ∥ **−u**, 노출 반대 —
    자연손목에선 노출부가 팔 쪽을 향하므로 **먼 끝**이 채택된다),
  - 제시는 그리퍼를 위로 젖혀 (tool z ↑, J5=0 손목 중립) 봉이 그리퍼 아래로
    **수직 매달림** — 중력 모멘트 0 (조 안 회전력 없음), 수직 봉은 방위
    대칭이라 "so101 로 조준" 이 필요 없다 (펜-era 조준/접선 문제와 검출 yaw
    180° 모호성이 함께 소멸),
  - so101 은 **아래로 늘어진 노출부**를 수평 접근·수직 조축(tool z ∥ 봉 축)으로
    받는다 — omx 그리퍼/카메라는 E 위 ~4.5cm 라 시선(저각 측면)과 안 겹침.

mono 검출 전제: 봉은 z=table 평면 위 → 픽 기하는 XY 평면 (끝점/방향/길이).
봉은 **축대칭** — 펜의 "어느 끝이 far 인가" 의미가 없어 양 끝이 동등한 파지
후보다 (도달성이 고름).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from modules.tasks.core.errors import TaskError

Vec2 = tuple[float, float]


def block_endpoints(
    center_xy: Vec2, yaw_rad: float, length_m: float
) -> tuple[Vec2, Vec2]:
    """OBB (중심, 긴 축 yaw, 길이) → 양 끝점. mono z=table 검출의 footprint 소비."""
    hx = 0.5 * length_m * math.cos(yaw_rad)
    hy = 0.5 * length_m * math.sin(yaw_rad)
    return (
        (center_xy[0] - hx, center_xy[1] - hy),
        (center_xy[0] + hx, center_xy[1] + hy),
    )


@dataclass(frozen=True, slots=True)
class BlockGrasp:
    """omx 봉 파지 계획의 기하 산출 (robot frame — 호출자가 넣은 frame 그대로).

    ends: (파지점 xy, u) 두 후보 — u = 파지점→노출 끝(긴 자유부) XY 단위벡터.
      봉은 축대칭이라 양 끝이 동등 — pick resolve 가 도달 가능한 쪽을 채택.
      tool z ∥ **−u** 규약 (pick 의 조 축 방위 + 제시 hang(z↑)의 "노출부가
      아래"가 전부 이 벡터에서 파생 — steps.py 2026-07-27 손목 중립 개정).
    tcp_to_e_m: omx TCP(파지점) → so101 파지점 E 거리 (봉 축 방향) —
      제시(hang)에서 E = TCP − (0,0,tcp_to_e).
    below_e_m: E 아래로 남는 봉 길이 (봉 끝까지) — so101 적치/바닥 여유 계산용.
    """

    ends: list[tuple[Vec2, Vec2]]
    length_m: float
    width_m: float
    exposed_len_m: float
    tcp_to_e_m: float
    below_e_m: float


def plan_block_grasp(
    center_xy: Vec2,
    yaw_rad: float,
    footprint: Vec2,
    *,
    grasp_frac: float,
    jaw_along_m: float,
    exposed_frac: float,
    min_exposed_m: float,
    len_min_m: float,
    len_max_m: float,
) -> BlockGrasp:
    """검출 OBB → 봉 파지 기하 (robot frame). 순수 계산 — step 아님 (모션 0).

    grasp_frac: 잡는 끝에서 이 비율 지점을 문다 (모멘트 암↔조 접촉폭 트레이드
      오프 — hang 제시는 매달림이라 모멘트 0, 접촉폭 확보가 우선).
    jaw_along_m: omx 조가 봉 축 방향으로 차지하는 폭 (노출 계산에서 차감).
    exposed_frac: so101 파지점 E = 노출 세그먼트의 조-쪽 끝에서 이 비율 지점
      (두 그리퍼 이격↔끝 여백 트레이드오프 — probe 기준치 0.65).
    min_exposed_m: so101 최소 파지 길이 + margin — 미만이면 **명시 실패**
      (짧은 물체 침묵 진행 금지).
    """
    length = min(max(footprint[0], len_min_m), len_max_m)
    width = footprint[1]
    g = grasp_frac * length
    exposed = length - g - jaw_along_m / 2.0
    if exposed < min_exposed_m:
        raise TaskError(
            f"봉이 짧아 handover 불가 — 길이 {length * 100:.1f}cm 에서 파지점 "
            f"{grasp_frac:.0%}({g * 100:.1f}cm) + 조 폭 절반 "
            f"{jaw_along_m / 2 * 100:.1f}cm 를 빼면 노출 {exposed * 100:.1f}cm < "
            f"필요 {min_exposed_m * 100:.1f}cm. 더 긴 물체로 교체하거나 파지 "
            "비율(_BLOCK_GRASP_FRAC)을 낮추세요"
        )
    e1, e2 = block_endpoints(center_xy, yaw_rad, length)
    ends: list[tuple[Vec2, Vec2]] = []
    for tip, other in ((e1, e2), (e2, e1)):
        ux = (other[0] - tip[0]) / length
        uy = (other[1] - tip[1]) / length
        ends.append(((tip[0] + ux * g, tip[1] + uy * g), (ux, uy)))
    return BlockGrasp(
        ends=ends,
        length_m=length,
        width_m=width,
        exposed_len_m=exposed,
        tcp_to_e_m=jaw_along_m / 2.0 + exposed_frac * exposed,
        below_e_m=(1.0 - exposed_frac) * exposed,
    )
