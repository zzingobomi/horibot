"""pick_and_place @step 함수들 — closed-loop(servo) 파지 판 (2026-07-16 재설계,
2026-07-19 파일 분리 — 옛 단일 steps.py 1,850줄을 시나리오 phase 별로).

**집기 = closed-loop look-then-move** (docs/closed_loop_grasp_handoff.md 구현,
순수 계산·실측 근거·상태 전이 = ../servo.py, trace = ../servo_trace.py):

    찾기(search 스윕, coarse) → 계획(자세 가족 + standoff 사다리 resolve, 모션 0)
    → servo 루프 (rung 마다: 정지 관측 → tick gate → 상대 오차 보정 MoveL → 수렴
    시 하강) → commit (마지막 관측으로 blind 진입) → close → 파지 판정 (재시도 1)
    → 후퇴 → 판정 → home

파일 지도 (시나리오 흐름 순 — module.scenario 가 이 순서로 부른다):

    primitives.py  이동/그리퍼/home/파지 판정(verify_grasp) + 포맷 유틸 —
                   pick·place 공용 바닥층. servo 노브 SSOT(_SERVO_CFG)도 여기.
    search.py      detect — search waypoint 스윕 + 멀티 prompt 동시 검출 (coarse)
    world.py       WorldScan — 스윕 편승 월드 스캔 (빌드 백그라운드, best-effort)
    plan.py        plan_pick / plan_place / resolve_place — 모션 0 판정
                   (신뢰 컷/도달성 우선 순회/상자 중심 융합) + ServoPlan
    pick.py        servo_pick — closed-loop 실행 (tick 루프/commit/재플랜/재시도)
    place.py       execute_place — open-loop 적치 (접근/삽입/release/후퇴)

옛 open-loop 파지 (멀티뷰 융합 → 표면 antipodal → 일괄 실행) 는 **대체됨** —
팔 절대정확도(자세의존 ~1-2cm) ≈ 큐브(2.5cm) 라 구조적으로 실패했다 (2026-07-15
post-mortem, 성공 0). antipodal/plan_grasp 코드는 grasp_verify 진단 스크립트가
소비하므로 ../geometry.py·../antipodal.py 에 남아 있다. **놓기는 open-loop
유지** — 적치 대상(상자)이 크고 넓어 1-2cm 오차가 치명적이지 않다 (실측 도달
오차 12.8mm < 상자 여유).

handoff §2 실패 표 대비 구현 현황 (정직):
- 구현: 처음부터 못 봄 / 단발 드롭 vs 연속 소실 / mask 오검출(도약 gate) / depth
  붕괴(점군 gate) / 수렴 실패·발진(보정 상한) / 전체 timeout(tick 상한) / servo
  이동 IK 거부(MoveJ 폴백 후 실패) / close 후 EMPTY(재시도 상한) / 이송 중 놓침.
- 미구현 (알고 넘어감): FOV 부분 이탈(잘림) 전용 감지 — 응답에 이미지 크기가
  없어 bbox 경계 판정 불가. 점군 부족/도약 gate 가 간접 커버, 실물 데이터에서
  전용 gate 필요성이 보이면 계약 확장.
"""

from __future__ import annotations

# 소비자 편의 재수출 — module.scenario / 테스트 / 스크립트가 steps.X 로 접근
# (분리 전 단일 steps.py 표면 유지). 테스트의 노브 패치는 소유 모듈로
# (steps.primitives._SERVO_CFG 등 — 소비 코드가 모듈 참조로 읽는 자리).
from modules.detector.contract import OrientedDetection as OrientedDetection

from . import pick, place, plan, primitives, search, world
from .pick import servo_pick
from .place import execute_place, insert, pre_place, release, retreat
from .plan import (
    ServoPlan,
    _fuse_place_center,
    plan_pick,
    plan_place,
    resolve_place,
    servo_ladder_groups,
)
from .primitives import (
    _gripper_holding,
    close_gripper,
    go_home,
    home_waypoint,
    open_gripper,
    verify_grasp,
)
from .search import _SEARCH_GROUP, detect
from .world import WorldScan

__all__ = [
    "OrientedDetection",
    "ServoPlan",
    "WorldScan",
    "_SEARCH_GROUP",
    "_fuse_place_center",
    "_gripper_holding",
    "close_gripper",
    "detect",
    "execute_place",
    "go_home",
    "home_waypoint",
    "insert",
    "open_gripper",
    "pick",
    "place",
    "plan",
    "plan_pick",
    "plan_place",
    "pre_place",
    "primitives",
    "release",
    "resolve_place",
    "retreat",
    "search",
    "servo_ladder_groups",
    "servo_pick",
    "verify_grasp",
    "world",
]
