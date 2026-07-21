"""World 스캔 스윕 스텝 — 로봇이 'scan' waypoint 그룹을 돌며 캡처→빌드.

pick 편승 시절의 WorldScan(best-effort·백그라운드 juggling)을 대체 — 이제 스캔이
본업이라 **실패는 침묵하지 않고 raise**(사유+다음 행동), 빌드는 **foreground**
(진행바 보며 기다림). pose 마다 빌드 = 월드가 자라는 게 실시간으로 보인다
(scan BUILD_PROGRESS → 프론트 World 자동 갱신).

cross-task import 금지 — pick_and_place.steps 에 기대지 않고 자기 이동/그리퍼
헬퍼를 둔다 (공유 helper 추출은 실물 검증 후, resolve.py handover 분기와 동형).
"""

from __future__ import annotations

import asyncio
import logging
import time

from modules.motion.contract import (
    JointTarget,
    Motion,
    MoveJRequest,
    MoveJResponse,
)
from modules.motor.contract import (
    Motor,
    SetGripperRequest,
    SetGripperResponse,
)
from modules.scan.contract import (
    BuildRequest,
    BuildResponse,
    CaptureRequest,
    CaptureResponse,
    DeleteSessionRequest,
    DeleteSessionResponse,
    ListSessionsRequest,
    ListSessionsResponse,
    NewSessionRequest,
    NewSessionResponse,
    Scan,
)
from modules.tasks.core.context import TaskContext
from modules.tasks.core.errors import TaskError
from modules.tasks.core.step import step
from modules.waypoint.contract import (
    ListGroupMembersRequest,
    ListGroupMembersResponse,
    ListGroupsRequest,
    ListGroupsResponse,
    WaypointRecord,
    Waypoint,
)

logger = logging.getLogger(__name__)

# 스캔 자세 그룹 = 검출용 'search' 6포즈 재사용 (2026-07-21 확정). 전용 커버리지
# 포즈 생성을 시도했으나 실측 결론: **앞 근거리(X<~20cm)는 팔이 어떤 자세로도
# 못 봄 = 기구학 한계** (IK 전멸), 그 바깥은 search fan 이 이미 커버 (건강한
# 07-19 빌드로 실증). 앞구멍은 팔 길이 한계로 수용. 없으면 명시 실패 (침묵 폴백 금지).
_SCAN_GROUP = "search"
_SESSION_LABEL = "world_scan"
_SETTLE_S = 1.0  # MoveJ 후 손목 진동 정착 (depth 품질 — 실물 첫 런에서 튜닝).
_GRIPPER_SETTLE_S = 4.0  # 가동 조 풀스트로크 (pick primitives 와 동일 근거).
_CAPTURE_FRAMES = 10  # scan consensus 프레임 (scan CaptureRequest 기본과 동일).


@step(title="가동 조 열기")
async def open_gripper(ctx: TaskContext, robot_id: str) -> None:
    """관측 전 가동 조 open (§3.4) — 닫힌 조가 시야 하단 중앙을 가려 근거리
    앞을 오염시키는 것 방지. 고정 조·마운트는 물리적으로 남음 (robot self-filter
    가 정석 자리, 별개 이슈)."""
    spec = ctx.spec(robot_id)
    await ctx.call(
        Motor.Service.SET_GRIPPER,
        SetGripperRequest(position_raw=spec.gripper_open_raw),
        SetGripperResponse,
        robot_id=robot_id,
    )
    await asyncio.sleep(_GRIPPER_SETTLE_S)


@step(title="스캔 세션 시작")
async def start_session(ctx: TaskContext, robot_id: str) -> int:
    """새 world 스캔 세션 생성 + 이전 world 세션 프루닝 → session_row_id.

    프루닝 근거: world 배경은 latest-wins (옛 스캔은 새 스캔이 뜨면 가치 0) +
    rdb(horibot.db)가 git-tracked 라 매 실행 누적은 메타 bloat. label 매칭으로
    world_scan 세션만 지운다 (수동 스캔 세션은 건드리지 않음)."""
    res = await ctx.call(
        Scan.Service.NEW_SESSION,
        NewSessionRequest(robot_id=robot_id, label=_SESSION_LABEL),
        NewSessionResponse,
    )
    sid = res.session.id
    if sid is None:
        raise TaskError("스캔 세션 생성 실패 (session id 없음)")
    await _prune_old_sessions(ctx, robot_id, keep_id=sid)
    return sid


async def _prune_old_sessions(
    ctx: TaskContext, robot_id: str, *, keep_id: int
) -> None:
    """이전 world_scan 세션 삭제 (keep_id 제외). best-effort — 프루닝 실패가
    스캔을 막지 않게 경고만 (본 스캔은 새 세션에서 정상 진행)."""
    try:
        listed = await ctx.call(
            Scan.Service.LIST_SESSIONS,
            ListSessionsRequest(robot_id=robot_id),
            ListSessionsResponse,
        )
        stale = [
            s for s in listed.sessions
            if s.label == _SESSION_LABEL and s.id is not None and s.id != keep_id
        ]
        for s in stale:
            await ctx.call(
                Scan.Service.DELETE_SESSION,
                DeleteSessionRequest(session_row_id=s.id),  # type: ignore[arg-type]
                DeleteSessionResponse,
            )
        if stale:
            logger.info("world_scan: 이전 world 세션 %d개 프루닝", len(stale))
    except Exception as e:
        logger.warning("world_scan: 세션 프루닝 실패 (%s) — 스캔은 계속", e)


@step(title="스캔 스윕")
async def sweep(
    ctx: TaskContext,
    robot_id: str,
    session_row_id: int,
    voxel_size: float | None,
) -> None:
    """'search' 그룹 자세를 전부 돌며 pose 당 이동→정착→캡처, **빌드는 끝에 1번**.

    빌드 케이던스 (2026-07-21 확정): scan 빌드는 매번 **전체 재빌드**(누적 스캔
    전부 ICP+TSDF)라, pose 마다 빌드하면 낭비 + 느림 (~2-3분). 결과물은 마지막
    1회로 충분(완전한 mesh). **성장 UX 는 프론트가 포즈별 포인트클라우드 누적**으로
    보여준다 (빌드 없이 — 점 표시는 공짜). 여기선 스윕(캡처) + 최종 빌드만.

    실패는 침묵하지 않는다: 캡처/빌드 거부 = TaskError raise (사유 → 프론트 실패
    표시 + 재스캔)."""
    poses = await _scan_waypoints(ctx, robot_id)
    n = len(poses)
    t0 = time.monotonic()
    for i, wp in enumerate(poses):
        await _move_j(ctx, robot_id, wp.joint_values)
        await asyncio.sleep(_SETTLE_S)  # 손목 진동 정착 (depth 품질)
        cap = await ctx.call(
            Scan.Service.CAPTURE,
            CaptureRequest(session_row_id=session_row_id, num_frames=_CAPTURE_FRAMES),
            CaptureResponse,
            timeout=20.0,
        )
        if not cap.accepted:
            raise TaskError(
                f"스캔 캡처 거부 (pose {i + 1}/{n}: {wp.name}) — {cap.message}. "
                "카메라/자세 확인 후 다시 실행하세요"
            )
        logger.info(
            "world_scan sweep pose %d/%d 캡처 (총 %d장, %.1fs)",
            i + 1, n, cap.scan_count, time.monotonic() - t0,
        )
    # 스윕 끝 — mesh 1번 빌드 (전체 재빌드라 이 1회로 완전).
    build = await ctx.call(
        Scan.Service.BUILD,
        BuildRequest(session_row_id=session_row_id, voxel_size=voxel_size),
        BuildResponse,
        timeout=180.0,  # scan 은 timeout 미선언 — DEFAULT 로는 빌드가 잘림
    )
    if not build.accepted:
        raise TaskError(
            f"스캔 빌드 실패 — {build.message}. 정합 실패일 수 있음, 씬/자세 확인 "
            "후 다시 실행하세요"
        )
    rec = build.reconstruction
    logger.info(
        "world_scan 완료: %d 자세 캡처 + 최종 빌드 %d verts (%.1fs)",
        n, rec.vertex_count if rec else -1, time.monotonic() - t0,
    )


async def _scan_waypoints(ctx: TaskContext, robot_id: str) -> list[WaypointRecord]:
    """스캔 자세 그룹('search') 멤버(티칭 순서). 그룹 없음/빔 = 명시 실패 (침묵
    단일-뷰 폴백 금지 — 관측 자세를 티칭해야 스윕 성립)."""
    groups = await ctx.call(
        Waypoint.Service.LIST_GROUPS,
        ListGroupsRequest(robot_id=robot_id),
        ListGroupsResponse,
    )
    grp = next((g for g in groups.groups if g.name == _SCAN_GROUP), None)
    if grp is None or grp.id is None:
        raise TaskError(
            f"'{_SCAN_GROUP}' waypoint 그룹 없음 (robot={robot_id}) — 관측 자세를 "
            "티칭해 '검색' 그룹으로 묶은 뒤 다시 실행하세요"
        )
    members = await ctx.call(
        Waypoint.Service.LIST_GROUP_MEMBERS,
        ListGroupMembersRequest(group_row_id=grp.id),
        ListGroupMembersResponse,
    )
    if not members.waypoints:
        raise TaskError(
            f"'{_SCAN_GROUP}' 그룹이 비어있음 (robot={robot_id}) — 스캔 자세를 "
            "이 그룹에 추가하세요"
        )
    return members.waypoints


async def _move_j(ctx: TaskContext, robot_id: str, joints: list[float]) -> None:
    await ctx.call(
        Motion.Service.MOVE_J,
        MoveJRequest(target=JointTarget(kind="joint", joints=list(joints))),
        MoveJResponse,
        robot_id=robot_id,
    )
