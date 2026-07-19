"""찾기 — search waypoint 그룹 스윕 + 멀티 prompt 동시 검출 (coarse 전용).

스윕 관측은 멀리서라 FK 오차가 크다 (실측: 카메라 31-33cm 에서 ~40mm) —
coarse 위치 전용, 파지 정밀도는 servo 루프(pick.py)가 close 관측으로 잡는다.
"""

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
    ListGroupMembersRequest,
    ListGroupMembersResponse,
    ListGroupsRequest,
    ListGroupsResponse,
    Waypoint,
    WaypointRecord,
)

from .primitives import _TOP_K, _move_j_joints
from .world import WorldScan

logger = logging.getLogger(__name__)

# 검색 자세 그룹 — 사용자가 티칭한 "search" waypoint 그룹 (robot 별). 이 자세들을
# 모두 돌며 관측한다 (coarse 찾기 전용 — 파지 정밀도는 servo 루프 몫).
_SEARCH_GROUP = "search"
_SEARCH_SETTLE_S = 0.3  # MoveJ 후 카메라 흔들림 정착 대기 (검출 품질)


@step(title="검출")
async def detect(
    ctx: TaskContext,
    robot_id: str,
    prompts: list[str],
    world: WorldScan | None = None,
) -> dict[str, list[OrientedDetection]]:
    """search 그룹 자세를 **전부** 돌며 **모든 prompt 동시** 검출 → prompt 별 누적.

    단일 시점 검출은 가림/시야/각도로 놓치거나 오검출한다 — 여러 관측 자세를 다
    돌아 모으면 강건. **선택은 안 함** — 판정(신뢰 컷/도달성)은 plan 단계가.
    스윕 관측은 멀리서라 FK 오차가 크다 (실측: 카메라 31-33cm 에서 ~40mm) —
    coarse 위치 전용, 파지 정밀도는 servo 루프가 close 관측으로 잡는다.

    **스윕 통합 (2026-07-19)**: 옛 구조는 pick/place 가 같은 자세를 두 번 돌았다
    (스윕 비용의 대부분 = 관측 자세 MoveJ). pose 당 wire 호출 1번에 prompts 를
    다 실어 pick 검출 + place 검출 + world 스캔이 한 스윕에 끝난다 — 후보는
    응답의 per-candidate prompt 귀속으로 나눈다 (detector contract).

    world: build_world 옵션의 편승 스캔 — 각 pose 정지 관측 직후 capture
    (빌드는 백그라운드 — WorldScan docstring, best-effort).
    """
    t0 = time.monotonic()
    members = await _search_waypoints(ctx, robot_id)
    found: dict[str, list[OrientedDetection]] = {p: [] for p in prompts}
    for wp in members:
        await _move_j_joints(ctx, robot_id, wp.joint_values)
        await asyncio.sleep(_SEARCH_SETTLE_S)  # MoveJ 후 카메라 정착 (검출 품질)
        res = await ctx.call(
            Detector.Service.DETECT_ORIENTED,
            DetectRequest(robot_id=robot_id, prompts=list(prompts), top_k=_TOP_K),
            DetectOrientedResponse,
        )
        _bucket_by_prompt(found, res.candidates)
        if world is not None:
            # capture 는 검출 **뒤** 직렬 (병렬화 원복 — 2026-07-19 22:13 실물:
            # capture∥detect 첫 런에서 pose1 스캔이 정합 전멸 (인접 1→0
            # fitness 0.09, loop 전부 0.00/보정 1.4m → 부챗살 메시). 유력
            # 기전 = 직렬일 땐 capture 가 "settle + 검출 2~3s" 뒤라 잔진동이
            # 죽은 뒤였는데, 병렬은 도착 0.3s 후 즉시 캡처 — 진입 대이동
            # 직후인 pose1 손목 진동이 depth 를 오염 (검출은 cm 관용이라
            # 무사). 재병렬화 조건 = 모터 스트림 속도 기반 **정지 판정
            # 게이트**를 capture 앞에 구현 (추측 지연 상수 금지). 빌드는
            # 백그라운드 유지 (WorldScan docstring).
            await world.capture()
    if world is not None:
        world.finalize()  # 마지막 캡처 포함 빌드 보장 (대기 안 함)
    logger.info(
        "detect(%s): search '%s' %d 자세 → 후보 누적 %s (%.1fs)",
        ", ".join(prompts), _SEARCH_GROUP, len(members),
        {p: len(cs) for p, cs in found.items()}, time.monotonic() - t0,
    )
    for p, cs in found.items():
        for i, c in enumerate(cs):
            logger.info(
                "  [%s] 후보%d: score=%.2f height(단일뷰)=%.1fcm "
                "base_z(물체바닥)=%.3fm top=%.3fm pos=(%.3f,%.3f)",
                p, i, c.score, c.height * 100.0, c.base_z, c.position[2],
                c.position[0], c.position[1],
            )
    return found


def _bucket_by_prompt(
    found: dict[str, list[OrientedDetection]],
    candidates: list[OrientedDetection],
) -> None:
    """스윕 응답 후보를 요청 prompt 버킷에 누적 — 명명 헬퍼인 이유: 프리뷰
    정적 인덱서가 `dict[key].append` 첨자 호출을 `<동적>` 노이즈 행으로 잡는다
    (_join_msgs 동형). 요청 밖 귀속 = detector 계약 위반 신호 — 버리되 침묵 금지."""
    for c in candidates:
        bucket = found.get(c.prompt)
        if bucket is None:
            logger.warning(
                "detect: 요청 밖 prompt 귀속 후보 무시 (%r ∉ %s)",
                c.prompt, list(found),
            )
            continue
        bucket.append(c)


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
