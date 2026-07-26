"""큐브(2cm 정육면체) handover 의 순수 기하 — 하드웨어/wire 0, 오피스 단위테스트 대상.

⚠ 2026-07-26 신설 (펜→큐브 전환, 사용자 지시). 동그란 펜은 omx 평행조 그리퍼로
안정 파지가 어려워 폐기 (pen.py 삭제, git history 복원 가능). 큐브의 이점:
  - 평면 = 평행조 안정 파지 / footprint 오검출·"얇음+가림" 재검출 리스크 소멸.
  - **"다른 면 집기" 가 기하적으로 깔끔**: omx 가 두 대칭면(조 축)을 물면 나머지
    직교 면 쌍이 so101 몫 → 두 그리퍼 조 축이 90° 어긋나 서로 회피가 기본.

펜과의 근본 차이: 큐브는 **대칭** — 펜의 "먼 끝/노출 길이/짧음 실패" 개념이 없다.
omx 는 큐브 **중심**을 top-down 으로 물고, 조 축(어느 두 면)은 도달성이 고른다.
handover 의 다른 면 집기 = omx 옆면 / so101 **위·아래 면(수직 조축)** — 실제로
어느 자세가 나오나는 so101 수취 도달성이 결정하고(steps._RECV_FAMILY, 스윕
2026-07-26), 두 그리퍼가 안 부딪는 물리 보증은 충돌 게이트가 담당한다.

omx 는 depth 가 없다(웹캠) → 큐브도 mono DETECT_PLANAR(z=table)로 **XY 위치만**
잡고, 파지 높이(Z)는 큐브 크기 가정이 앵커 (steps 의 Z 사다리 — 2cm 큐브 →
중심 = 바닥+1cm, 사용자 지시 2026-07-26. footprint 는 XY 크기 clamp/로깅용).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

Vec2 = tuple[float, float]


@dataclass(frozen=True, slots=True)
class CubeGrasp:
    """omx 큐브 파지 계획의 기하 산출 (robot frame — 호출자가 넣은 frame 그대로).

    center_xy: 큐브 중심 (DETECT_PLANAR footprint 중심 — TCP 파지 XY).
    yaw_candidates: omx 조 축 후보 (rad) — 면 정렬(0/90) 우선, 대각(45/135) 폴백.
      square footprint 의 grasp_yaw 는 노이지하므로 여러 축을 열거해 resolve 가
      도달 가능한 것을 채택. 채택 조 축은 제시/수취의 "직교 면" 기준.
    size_m: 채택 큐브 변 (footprint 평균 clamp — mono 번짐 방어). Z 는 여기서
      안 정한다 (omx depth 없음 → 크기 가정이 앵커, 소비자 steps 의 Z 사다리).
    """

    center_xy: Vec2
    yaw_candidates: list[float]
    size_m: float


def plan_cube_grasp(
    center_xy: Vec2,
    grasp_yaw: float,
    footprint: Vec2,
    *,
    yaw_offsets_deg: tuple[float, ...],
    size_min_m: float,
    size_max_m: float,
) -> CubeGrasp:
    """검출 OBB → 큐브 파지 기하 (robot frame). 순수 계산 — step 아님 (모션 0).

    큐브는 대칭이라 펜의 "먼 끝/노출/짧음 실패" 가 없다 — 중심을 물고 조 축은
    도달성이 고른다. 조 축 후보 = 검출 yaw + yaw_offsets_deg (면 정렬 우선).
    size = footprint 평균을 [size_min, size_max] 로 clamp.
    """
    size = min(max((footprint[0] + footprint[1]) / 2.0, size_min_m), size_max_m)
    yaws = [grasp_yaw + math.radians(o) for o in yaw_offsets_deg]
    return CubeGrasp(center_xy=center_xy, yaw_candidates=yaws, size_m=size)
