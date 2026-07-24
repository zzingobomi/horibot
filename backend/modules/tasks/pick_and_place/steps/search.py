from __future__ import annotations

import asyncio
import logging
import time

from modules.detector.contract import (
    DetectOrientedResponse,
    DetectRequest,
    Detector,
    OrientedDetection,
)
from modules.tasks.core.context import TaskContext
from modules.tasks.core.errors import TaskError
from modules.tasks.core.step import step
from modules.waypoint.contract import (
    ListGroupMembersByNameRequest,
    ListGroupMembersByNameResponse,
    Waypoint,
    WaypointRecord,
)

from .primitives import _TOP_K, _move_j

logger = logging.getLogger(__name__)


_SEARCH_GROUP = "search"
_SEARCH_SETTLE_S = 0.3  # 로봇 이동 후 카메라 흔들림 정착 대기 (검출 품질)


@step(title="오브젝트 찾기")
async def detect(
    ctx: TaskContext,
    robot_id: str,
    prompts: list[str],
) -> dict[str, list[OrientedDetection]]:
    """search 그룹 자세를 모두 순회하며 모든 prompt를 동시에 검출해 prompt별 후보를 누적한다.

    단일 관측은 가림이나 시야각으로 검출을 놓칠 수 있으므로 여러 자세의 관측을 합친다.
    여기서는 후보만 수집하며, 신뢰도와 도달성 판단은 plan 단계에서 수행한다.

    관측은 원거리(coarse) 위치 추정용이며, 최종 파지 정밀도는 servo의 근접 관측에서 보정한다.
    """
    t0 = time.monotonic()
    members = await _search_waypoints(ctx, robot_id)
    found: dict[str, list[OrientedDetection]] = {p: [] for p in prompts}
    for wp in members:
        await _move_j(ctx, robot_id, joints=wp.joint_values)
        await asyncio.sleep(_SEARCH_SETTLE_S)
        res = await ctx.call(
            Detector.Service.DETECT_ORIENTED,
            DetectRequest(robot_id=robot_id, prompts=list(prompts), top_k=_TOP_K),
            DetectOrientedResponse,
        )
        _bucket_by_prompt(found, res.candidates)
    logger.info(
        "detect(%s): search '%s' %d 자세 → 후보 누적 %s (%.1fs)",
        ", ".join(prompts),
        _SEARCH_GROUP,
        len(members),
        {p: len(cs) for p, cs in found.items()},
        time.monotonic() - t0,
    )
    for p, cs in found.items():
        for i, c in enumerate(cs):
            logger.info(
                "  [%s] 후보%d: score=%.2f height(단일뷰)=%.1fcm "
                "base_z(물체바닥)=%.3fm top=%.3fm pos=(%.3f,%.3f)",
                p,
                i,
                c.score,
                c.height * 100.0,
                c.base_z,
                c.position[2],
                c.position[0],
                c.position[1],
            )
    return found


def _bucket_by_prompt(
    found: dict[str, list[OrientedDetection]],
    candidates: list[OrientedDetection],
) -> None:
    for c in candidates:
        bucket = found.get(c.prompt)
        if bucket is None:
            logger.warning(
                "detect: 요청 밖 prompt 귀속 후보 무시 (%r ∉ %s)",
                c.prompt,
                list(found),
            )
            continue
        bucket.append(c)


async def _search_waypoints(ctx: TaskContext, robot_id: str) -> list[WaypointRecord]:
    res = await ctx.call(
        Waypoint.Service.LIST_GROUP_MEMBERS_BY_NAME,
        ListGroupMembersByNameRequest(robot_id=robot_id, name=_SEARCH_GROUP),
        ListGroupMembersByNameResponse,
    )
    if not res.found:
        raise TaskError(
            f"'{_SEARCH_GROUP}' waypoint 그룹 없음 (robot={robot_id}) — 검색 자세를 "
            "티칭해 '검색' 그룹으로 묶은 뒤 다시 실행하세요"
        )
    if not res.waypoints:
        raise TaskError(
            f"'{_SEARCH_GROUP}' 그룹이 비어있음 (robot={robot_id}) — 검색 자세를 "
            "이 그룹에 추가하세요"
        )
    return res.waypoints
