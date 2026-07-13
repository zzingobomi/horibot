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
    JointTarget,
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
from modules.tasks.core.errors import NoReachableGrasp, TaskError
from modules.tasks.core.step import step
from modules.waypoint.contract import (
    ListGroupMembersRequest,
    ListGroupMembersResponse,
    ListGroupsRequest,
    ListGroupsResponse,
    Waypoint,
    WaypointRecord,
)

from . import geometry
from .geometry import GraspCandidate, PlaceCandidate, Quat, Vec3

logger = logging.getLogger(__name__)

_GRIPPER_SETTLE_S = 1.2
_TOP_K = 5

# 검색 자세 그룹 — 사용자가 티칭한 "search" waypoint 그룹 (robot 별). 이 자세들을
# 모두 돌며 관측한다 (§ multi-view). 값은 로봇마다 다른 관측 시점 = 사람이 배치.
_SEARCH_GROUP = "search"
_SEARCH_SETTLE_S = 0.3  # MoveJ 후 카메라 흔들림 정착 대기 (검출 품질)


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
    """search 그룹 자세를 **전부** 돌며 검출 → 후보 **누적** (첫 자세에서 안 멈춤).

    원리 (옛 SearchWaypointGroup 포팅): 단일 시점 검출은 가림/시야/각도로 놓치거나
    오검출한다. 사람이 티칭한 여러 관측 자세를 다 돌아 후보를 모으면(모두 base frame
    이라 비교 가능) 관측이 많아 강건하다. **선택은 안 함** — 누적만. "자세 다 돌고
    진짜 제일 점수 높은 것" 판정은 select_pick_target 이 누적 전체에서 (max score).
    """
    members = await _search_waypoints(ctx, robot_id)
    candidates: list[OrientedDetection] = []
    for wp in members:
        await _move_j_joints(ctx, robot_id, wp.joint_values)
        await asyncio.sleep(_SEARCH_SETTLE_S)  # MoveJ 후 카메라 정착 (검출 품질)
        res = await ctx.call(
            Detector.Service.DETECT_ORIENTED,
            DetectRequest(robot_id=robot_id, prompt=prompt, top_k=_TOP_K),
            DetectOrientedResponse,
        )
        if res.candidates:
            candidates.extend(res.candidates)
    logger.info(
        "detect(%s): search '%s' %d 자세 → 후보 누적 %d",
        prompt, _SEARCH_GROUP, len(members), len(candidates),
    )
    # 진단: 후보별 실제 height/base_z (height prior 1.5~15cm 통과 여부 확인용).
    # "통과 0" 일 때 왜인지 — 너무 얇게 읽혔나(<1.5cm) / 바닥 잘못 잡아 부풀었나(>15cm)
    # / depth·캘 문제인가 를 이 값으로 판별한다 (D405 depth → base-frame 투영 결과).
    for i, c in enumerate(candidates):
        ok = geometry._MIN_HEIGHT_M <= c.height <= geometry._MAX_HEIGHT_M
        logger.info(
            "  후보%d: score=%.2f height=%.1fcm base_z(바닥)=%.3fm top=%.3fm "
            "pos=(%.3f,%.3f) prior통과=%s",
            i, c.score, c.height * 100.0, c.base_z, c.position[2],
            c.position[0], c.position[1], ok,
        )
    return candidates


async def _search_waypoints(
    ctx: TaskContext, robot_id: str
) -> list[WaypointRecord]:
    """search 그룹 멤버(티칭 순서). 그룹 없음/빔 = 명시적 실패 (침묵 단일-뷰 폴백
    금지 — 사용자가 관측 자세를 티칭해야 multi-view 검색이 성립)."""
    groups = await ctx.call(
        Waypoint.Service.LIST_GROUPS,
        ListGroupsRequest(robot_id=robot_id),
        ListGroupsResponse,
    )
    grp = next((g for g in groups.groups if g.name == _SEARCH_GROUP), None)
    if grp is None or grp.id is None:
        raise TaskError(
            f"'{_SEARCH_GROUP}' waypoint 그룹 없음 (robot={robot_id}) — 검색 자세를 "
            "티칭해 '검색' 그룹으로 묶은 뒤 다시 실행하세요"
        )
    members = await ctx.call(
        Waypoint.Service.LIST_GROUP_MEMBERS,
        ListGroupMembersRequest(group_row_id=grp.id),
        ListGroupMembersResponse,
    )
    if not members.waypoints:
        raise TaskError(
            f"'{_SEARCH_GROUP}' 그룹이 비어있음 (robot={robot_id}) — 검색 자세를 "
            "이 그룹에 추가하세요"
        )
    return members.waypoints


async def _move_j_joints(
    ctx: TaskContext, robot_id: str, joints: list[float]
) -> None:
    """관절값으로 MoveJ (waypoint joint_values 그대로 — WaypointPanel 이동과 동일)."""
    await ctx.call(
        Motion.Service.MOVE_J,
        MoveJRequest(target=JointTarget(kind="joint", joints=list(joints))),
        MoveJResponse,
        robot_id=robot_id,
    )


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
