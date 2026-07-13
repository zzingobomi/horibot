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


# ─── scenario ─────────────────────────────────


@step(title="집기")
async def pick(
    ctx: TaskContext, robot_id: str, prompt: str
) -> tuple[OrientedDetection, GraspCandidate]:
    cands = await detect(ctx, robot_id, prompt)
    target = geometry.select_pick_target(cands, prompt=prompt)
    plan = geometry.plan_grasp(target)
    best = plan[await resolve_grasp(ctx, robot_id, plan)]
    await pre_grasp(ctx, robot_id, best)
    await open_gripper(ctx, robot_id)
    await descend(ctx, robot_id, best)
    await close_gripper(ctx, robot_id)
    await lift(ctx, robot_id, best)
    return target, best


@step(title="놓기")
async def place(
    ctx: TaskContext,
    robot_id: str,
    prompt: str,
    *,
    held: OrientedDetection,
    grasp: GraspCandidate,
) -> PlaceCandidate:
    spots = await detect(ctx, robot_id, prompt)
    spot = geometry.select_pick_target(spots, prompt=prompt)
    pplan = geometry.plan_place(spot, held=held, lateral=grasp.lateral)
    drop = pplan[await resolve_place(ctx, robot_id, pplan)]
    await pre_place(ctx, robot_id, drop)
    await lower(ctx, robot_id, drop)
    await release(ctx, robot_id)
    await retreat(ctx, robot_id, drop)
    return drop


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
    await asyncio.sleep(_GRIPPER_SETTLE_S)
