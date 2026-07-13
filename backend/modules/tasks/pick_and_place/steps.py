from __future__ import annotations

import asyncio
import logging

from modules.detector.contract import (
    DetectOrientedResponse,
    DetectRequest,
    Detector,
    OrientedDetection,
)
from modules.motion.contract import (
    Motion,
    MoveJRequest,
    MoveJResponse,
    MoveLRequest,
    MoveLResponse,
    PoseTarget,
    ResolveReachableRequest,
    ResolveReachableResponse,
)
from modules.motor.contract import Motor, SetGripperRequest, SetGripperResponse
from modules.tasks.core.context import TaskContext
from modules.tasks.core.errors import NoReachableGrasp
from modules.tasks.core.step import step

from . import geometry
from .geometry import GraspCandidate, PlaceCandidate, Quat, Vec3

logger = logging.getLogger(__name__)

_GRIPPER_SETTLE_S = 1.2
_TOP_K = 5


# ─── scenario: 계획(plan) → 실행(execute) 분리 ─────────────────────
#
# 순서 규약 (2026-07-13): 물리 파지 **전에** 집기·놓기 도달성을 모두 검증한다.
# 옛 구조(집기 완주 후 놓기 검출/IK)는 놓을 곳이 도달 불가일 때 이미 물체를 쥔
# 채 실패해 로봇이 물체를 든 채 멈추는 corrupt 상태를 만들었다 (2026-07-13
# resolve_place IK 불가 실패). 계획 단계는 모션 0 (검출 + 배치 IK 판정뿐)이라,
# 어느 한쪽이라도 도달 불가면 아무것도 집기 전에 실패한다.


@step(title="집기 계획")
async def plan_pick(
    ctx: TaskContext, robot_id: str, prompt: str
) -> tuple[OrientedDetection, GraspCandidate]:
    """검출 + 파지 후보 IK 판정 (모션 0) → 실행 가능한 파지 후보."""
    cands = await detect(ctx, robot_id, prompt)
    target = geometry.select_pick_target(cands, prompt=prompt)
    plan = geometry.plan_grasp(target)
    best = plan[await resolve_grasp(ctx, robot_id, plan)]
    return target, best


@step(title="놓기 계획")
async def plan_place(
    ctx: TaskContext,
    robot_id: str,
    prompt: str,
    *,
    held: OrientedDetection,
    grasp: GraspCandidate,
) -> PlaceCandidate:
    """검출 + 적치 후보 IK 판정 (모션 0) → 실행 가능한 적치 후보. 물체 dims 는
    검출(held)에서 오므로 물리 파지 전에도 계획 가능."""
    spots = await detect(ctx, robot_id, prompt)
    spot = geometry.select_pick_target(spots, prompt=prompt)
    pplan = geometry.plan_place(spot, held=held, lateral=grasp.lateral)
    drop = pplan[await resolve_place(ctx, robot_id, pplan)]
    return drop


@step(title="집기 실행")
async def execute_pick(
    ctx: TaskContext, robot_id: str, c: GraspCandidate
) -> None:
    """계획된 파지 후보로 실제 파지 (접근→하강→파지→들어올리기)."""
    await pre_grasp(ctx, robot_id, c)
    await open_gripper(ctx, robot_id)
    await descend(ctx, robot_id, c)
    await close_gripper(ctx, robot_id)
    await lift(ctx, robot_id, c)


@step(title="놓기 실행")
async def execute_place(
    ctx: TaskContext, robot_id: str, c: PlaceCandidate
) -> None:
    """계획된 적치 후보로 실제 적치 (접근→내리기→내려놓기→후퇴)."""
    await pre_place(ctx, robot_id, c)
    await lower(ctx, robot_id, c)
    await release(ctx, robot_id)
    await retreat(ctx, robot_id, c)


# ─── planning ──────────────────────────────────────


@step(title="검출")
async def detect(
    ctx: TaskContext, robot_id: str, prompt: str
) -> list[OrientedDetection]:
    res = await ctx.call(
        Detector.Service.DETECT_ORIENTED,
        DetectRequest(robot_id=robot_id, prompt=prompt, top_k=_TOP_K),
        DetectOrientedResponse,
    )
    note = f"{len(res.candidates)}개 후보"
    if res.message:
        note += f" — {res.message}"
    logger.info("detect(%s): %s", prompt, note)
    return list(res.candidates)


@step(title="파지 후보 선별")
async def resolve_grasp(
    ctx: TaskContext, robot_id: str, plan: list[GraspCandidate]
) -> int:
    res = await ctx.call(
        Motion.Service.RESOLVE_REACHABLE,
        ResolveReachableRequest(groups=geometry.grasp_ik_groups(plan)),
        ResolveReachableResponse,
        robot_id=robot_id,
    )
    if res.index < 0:
        raise NoReachableGrasp(res.message)
    logger.info("resolve_grasp: group %d — %s", res.index, plan[res.index].label)
    return res.index


@step(title="적치 후보 선별")
async def resolve_place(
    ctx: TaskContext, robot_id: str, plan: list[PlaceCandidate]
) -> int:
    res = await ctx.call(
        Motion.Service.RESOLVE_REACHABLE,
        ResolveReachableRequest(groups=geometry.place_ik_groups(plan)),
        ResolveReachableResponse,
        robot_id=robot_id,
    )
    if res.index < 0:
        raise NoReachableGrasp(res.message)
    logger.info("resolve_place: group %d — %s", res.index, plan[res.index].label)
    return res.index


# ─── primitives ────────────


@step(title="파지 접근")
async def pre_grasp(ctx: TaskContext, robot_id: str, c: GraspCandidate) -> None:
    await _move_j_pose(ctx, robot_id, c.pre, c.quat)


@step(title="하강")
async def descend(ctx: TaskContext, robot_id: str, c: GraspCandidate) -> None:
    await _move_l(ctx, robot_id, c.grasp, c.quat)


@step(title="들어올리기")
async def lift(ctx: TaskContext, robot_id: str, c: GraspCandidate) -> None:
    await _move_l(ctx, robot_id, c.pre, c.quat)


@step(title="적치 접근")
async def pre_place(ctx: TaskContext, robot_id: str, c: PlaceCandidate) -> None:
    await _move_j_pose(ctx, robot_id, c.pre, c.quat)


@step(title="내리기")
async def lower(ctx: TaskContext, robot_id: str, c: PlaceCandidate) -> None:
    await _move_l(ctx, robot_id, c.place, c.quat)


@step(title="후퇴")
async def retreat(ctx: TaskContext, robot_id: str, c: PlaceCandidate) -> None:
    await _move_l(ctx, robot_id, c.pre, c.quat)


@step(title="그리퍼 열기")
async def open_gripper(ctx: TaskContext, robot_id: str) -> None:
    await _set_gripper(ctx, robot_id, open_=True)


@step(title="그리퍼 닫기")
async def close_gripper(ctx: TaskContext, robot_id: str) -> None:
    await _set_gripper(ctx, robot_id, open_=False)


@step(title="내려놓기")
async def release(ctx: TaskContext, robot_id: str) -> None:
    await _set_gripper(ctx, robot_id, open_=True)


# ─── internal helpers ──


async def _move_j_pose(
    ctx: TaskContext, robot_id: str, position: Vec3, quaternion: Quat
) -> None:
    await ctx.call(
        Motion.Service.MOVE_J,
        MoveJRequest(
            target=PoseTarget(kind="pose", position=position, quaternion=quaternion)
        ),
        MoveJResponse,
        robot_id=robot_id,
    )


async def _move_l(
    ctx: TaskContext, robot_id: str, position: Vec3, quaternion: Quat
) -> None:
    await ctx.call(
        Motion.Service.MOVE_L,
        MoveLRequest(
            target=PoseTarget(kind="pose", position=position, quaternion=quaternion)
        ),
        MoveLResponse,
        robot_id=robot_id,
    )


async def _set_gripper(ctx: TaskContext, robot_id: str, *, open_: bool) -> None:
    spec = ctx.spec(robot_id)
    raw = spec.gripper_open_raw if open_ else spec.gripper_close_raw
    await ctx.call(
        Motor.Service.SET_GRIPPER,
        SetGripperRequest(position_raw=raw),
        SetGripperResponse,
        robot_id=robot_id,
    )
    if not ctx.dry:  # dry-run(미리보기)엔 하드웨어 정착 대기 불필요
        await asyncio.sleep(_GRIPPER_SETTLE_S)
