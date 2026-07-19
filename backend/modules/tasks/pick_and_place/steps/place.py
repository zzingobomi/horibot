"""놓기 실행 (open-loop 유지) — 적치 대상(상자)이 크고 넓어 1-2cm 오차가
치명적이지 않다 (실측 도달 오차 12.8mm < 상자 여유). coarse 오차의 안정화는
plan._fuse_place_center (스윕 관측 융합) 몫."""

from __future__ import annotations

import asyncio
import logging

from modules.tasks.core.context import TaskContext
from modules.tasks.core.step import step
from modules.waypoint.contract import WaypointRecord

from ..geometry import PlaceCandidate
from .primitives import (
    _fmt,
    _fmt_joints,
    _move_j_joints,
    _move_l,
    _set_gripper,
    close_gripper,
    go_home,
    verify_grasp,
)

logger = logging.getLogger(__name__)


@step(title="놓기 실행")
async def execute_place(
    ctx: TaskContext,
    robot_id: str,
    c: PlaceCandidate,
    pre_joints: list[float],
    home: WaypointRecord,
) -> None:
    """계획된 적치 후보로 실제 적치 — 접근 → 삽입 → 내려놓기 → 후퇴 → home.

    home 에서 시작 (servo_pick 이 home 으로 끝남). 종료도 home — 다음 run 의
    시작 자세가 일정하고 카메라 시야에서 팔이 빠진다."""
    await pre_place(ctx, robot_id, pre_joints)
    await insert(ctx, robot_id, c)
    # 파지 판정: 내려놓기 직전에도 물고 있나 (이송 중 놓쳤으면 여기서 실패 —
    # 빈 손으로 release 하는 허위 성공 방지).
    await verify_grasp(ctx, robot_id, phase="적치 직전", grasp_label=c.label)
    await release(ctx, robot_id)
    await retreat(ctx, robot_id, c, pre_joints)
    await go_home(ctx, robot_id, home)
    # 마무리: 그리퍼 닫아 정리 자세 (열린 조가 대기 중 걸리적/충돌 표면 —
    # 사용자 요청 2026-07-17)
    await close_gripper(ctx, robot_id)


@step(title="적치 접근")
async def pre_place(
    ctx: TaskContext, robot_id: str, pre_joints: list[float]
) -> None:
    logger.info("pre_place robot=%s → joints=%s", robot_id, _fmt_joints(pre_joints))
    await _move_j_joints(ctx, robot_id, pre_joints)


@step(title="삽입")
async def insert(ctx: TaskContext, robot_id: str, c: PlaceCandidate) -> None:
    logger.info("insert robot=%s → place %s", robot_id, _fmt(c.place))
    await _move_l(ctx, robot_id, c.place, c.quat)


@step(title="적치 후퇴")
async def retreat(
    ctx: TaskContext,
    robot_id: str,
    c: PlaceCandidate,
    pre_joints: list[float],
) -> None:
    """place → pre 직선 후퇴. MoveL 실패 시 계획 관절해(pre_joints) MoveJ 폴백.

    2026-07-17 실물: 사전 검증 통과한 retreat MoveL 이 실행 중 끝점(=pre)에서
    IK 실패 — pre 는 resolve 가 관절해까지 증명했고 20초 전 MoveJ 로 실제 도달한
    자세다 (좁은 basin — 실행 seed 연쇄가 다른 branch 로 흘러 못 푸는 복권).
    알려진 해가 있는데 재풀이 실패로, **적치까지 성공한** task 를 죽이지
    않는다. 이 시점 물체는 이미 release 됨 + 폴백 시작 관절은 insert 로 온 구성
    근방이라 관절 보간 스윕 위험 낮음. (정석은 insert 궤적 역재생 — motion 단
    지원 필요, 2026-07-17 분석 §근본3. 폴백은 그 전까지의 안전망.)
    """
    logger.info("retreat robot=%s → pre %s", robot_id, _fmt(c.pre))
    try:
        await _move_l(ctx, robot_id, c.pre, c.quat)
    except asyncio.CancelledError:
        raise
    except Exception as e:
        logger.warning("retreat MoveL 실패 (%s) — 계획 관절해(pre) MoveJ 폴백", e)
        await _move_j_joints(ctx, robot_id, pre_joints)


@step(title="내려놓기")
async def release(ctx: TaskContext, robot_id: str) -> None:
    await _set_gripper(ctx, robot_id, open_=True)
