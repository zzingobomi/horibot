"""handover 시나리오 step 들 — omx(giver)가 집어 든 물체를 so101(receiver)이
받아 상자에 적치.

⚠ **2026-07-17 신설, 실물 미검증** (사용자 지시: 코드만 — 실물 테스트는
pick_and_place 검증 완료 후). sim(mock ctx) 테스트만 통과한 상태이며, 실물
첫 런 전 확인 필수 가정 목록:
  ① OMX URDF tcp 링크의 축 규약이 so101 과 동일 (tool x=approach) — 다르면
     _grasp_quat 의 회전 구성이 통째로 틀어진다.
  ② 크로스캘 base_pose(robots.yaml) 정확도 σ_t ~8mm — omx open-loop pick 과
     so101 수취의 위치 예산이 여기에 걸려 있다 (open-loop 정확도 바닥 ≈ 물체
     크기 — 2cm 큐브면 턱걸이. 실패 시 so101 재검출 정밀화가 다음 단계).
  ③ 'handover' waypoint(omx) 는 물체가 so101 도달 영역(수평 접근 가능 높이,
     대략 z 0.08~0.15) 에 오도록 티칭되어야 한다 — 시나리오는 티칭을 검증할
     수 없고 fail-fast 안내만 한다.

설계 원칙 (pick_and_place 계승):
  - 계획(모션 0 resolve) 먼저, 실행은 그 관절해 그대로 (판정 해 == 실행 해).
  - 실패는 사유 + 다음 행동 안내 포함 명시 실패. 침묵 fallback 금지.
  - **수취 순서 불변식**: so101 이 물고(verify held) 난 **뒤에만** omx 가 연다
    — 순서가 뒤집히면 물체 낙하 (회귀 테스트 잠금).
  - cross-robot 충돌: so101 수취 접근/omx 후퇴 경로를 CrossRobotChecker 로
    계획 시점 검사 (collision.py — 독립 유틸, motion 모듈 불침범).
"""

from __future__ import annotations

import asyncio
import logging
import math
import time

import numpy as np
from scipy.spatial.transform import Rotation

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
    TcpPose,
    TcpSnapshotRequest,
    TcpState,
)
from modules.motor.contract import (
    JointState,
    Motor,
    ReadStateRequest,
    SetGripperRequest,
    SetGripperResponse,
)
from modules.tasks.core.context import TaskContext
from modules.tasks.core.errors import (
    DetectionNotFound,
    GraspFailed,
    NoReachableGrasp,
    TaskError,
)
from modules.tasks.core.step import step
from modules.waypoint.contract import (
    ListGroupMembersRequest,
    ListGroupMembersResponse,
    ListGroupsRequest,
    ListGroupsResponse,
    ListWaypointsRequest,
    ListWaypointsResponse,
    Waypoint,
    WaypointRecord,
)

from .collision import BasePose, CrossRobotChecker

logger = logging.getLogger(__name__)

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]

# ─── 상수 (노브 — 실물 첫 런 데이터로 튜닝, 전부 미검증 기본값) ────────

_SEARCH_GROUP = "search"  # so101 검출 스윕 waypoint 그룹 (pick_and_place 공유)
_HOME_WAYPOINT = "home"
_HANDOVER_WAYPOINT = "handover"  # omx 가 물체를 내미는 자세 (omx 티칭 필수)
_SEARCH_SETTLE_S = 0.6
_TOP_K = 3
_GRIPPER_SETTLE_S = 4.0  # close 완료 대기 (pick_and_place 와 동일 근거)
_HELD_LOAD_MIN_RAW = 80

# 검출 신뢰 게이트 — pick_and_place 와 같은 실측 근거 (2026-07-17).
_SCORE_MIN = 0.45
_MAX_WIDTH_M = 0.050  # 조 개구 35mm + 관측 번짐 여유 15mm
_BASE_Z_MIN_M = -0.01
_BASE_Z_MAX_M = 0.08

# omx pick 접근 기하 (open-loop — omx 는 카메라 없음, 크로스캘 정확도에 의존).
_OMX_PRE_CLEAR_M = 0.06  # grasp 에서 접근축 후방 pre 거리
_OMX_LIFT_M = 0.08  # 파지 후 수직 상승
_OMX_TILTS_DEG = (0, 15, 30)  # 수직부터 — 도달 판정은 omx motion resolve
_OMX_GRIP_BELOW_TOP_M = 0.010  # 윗면 앵커 파지 깊이 (pick_and_place 동일 규약)

# so101 수취 접근 기하 — 공중 물체는 옆(수평 계열) 파지 (so101 수직 approach
# 는 z≤0.038 물리 한계 + omx 조와의 간섭 회피는 충돌 체커가 최종 심판).
_RECV_PRE_CLEAR_M = 0.07
_RECV_TILTS_DEG = (90, 75, 60)  # 수평 우선 (공중 물체 옆 파지)
_RECV_YAWS_DEG = (0.0, 30.0, -30.0, 60.0, -60.0, 90.0)  # 물체 방향 기준 부채꼴
_RECV_WITHDRAW_M = 0.08
# 수취 계획 충돌 재시도 상한 — 채택 그룹이 omx 와 충돌하면 그 그룹을 빼고
# 재-resolve (전부 소진 = 명시 실패).
_RECV_COLLISION_RETRY = 3

# 적치 (pick_and_place plan_place 슬림판 — 상자 위 open-loop).
_PLACE_TILTS_DEG = (0, 30, -30, 45, -45)
_PLACE_YAW_OFFSETS_DEG = (0.0, 90.0, 180.0, 270.0)
_PLACE_DROP_CLEAR_M = 0.005
_PLACE_PRE_CLEAR_M = 0.06

# 기준 자세: 툴 x(approach)→base -z (수직 하향), y(조 축)→base +y — so101
# URDF tcp 규약 (pick_and_place geometry._TOPDOWN 동일. **omx 도 같다고 가정**
# — 모듈 docstring 가정 ①).
_TOPDOWN = Rotation.from_matrix(
    np.column_stack([[0, 0, -1], [0, 1, 0], [1, 0, 0]])
)


# ─── frame 변환 (world = so101 base — robots.yaml base_pose 규약) ─────


def world_to_robot(p: Vec3, base: BasePose) -> Vec3:
    """world(so101 base) 좌표 → robot base 좌표 (base_pose 역변환)."""
    c, s = math.cos(base.yaw_rad), math.sin(base.yaw_rad)
    dx, dy, dz = p[0] - base.x, p[1] - base.y, p[2] - base.z
    return (c * dx + s * dy, -s * dx + c * dy, dz)


def robot_to_world(p: Vec3, base: BasePose) -> Vec3:
    """robot base 좌표 → world(so101 base) 좌표."""
    c, s = math.cos(base.yaw_rad), math.sin(base.yaw_rad)
    return (
        base.x + c * p[0] - s * p[1],
        base.y + s * p[0] + c * p[1],
        base.z + p[2],
    )


def _grasp_quat(yaw: float, tilt_deg: float) -> Quat:
    """yaw(조 축 방위) × tilt(조 축 둘레 기울임) → TCP quat — pick_and_place
    회전 구성과 동일 규약 (tool x=approach, y=jaw)."""
    rot = (
        Rotation.from_euler("z", yaw)
        * _TOPDOWN
        * Rotation.from_euler("y", math.radians(tilt_deg))
    )
    qx, qy, qz, qw = (float(v) for v in rot.as_quat())
    return (qx, qy, qz, qw)


def _approach_of(yaw: float, tilt_deg: float) -> Vec3:
    rot = (
        Rotation.from_euler("z", yaw)
        * _TOPDOWN
        * Rotation.from_euler("y", math.radians(tilt_deg))
    )
    a = rot.apply([1.0, 0.0, 0.0])
    return (float(a[0]), float(a[1]), float(a[2]))


# ─── waypoint 조회 (fail-fast — 모션 0 시점) ──────────────────────────


@step(title="waypoint 조회")
async def named_waypoint(
    ctx: TaskContext, robot_id: str, name: str, teach_hint: str
) -> WaypointRecord:
    res = await ctx.call(
        Waypoint.Service.LIST,
        ListWaypointsRequest(robot_id=robot_id),
        ListWaypointsResponse,
    )
    wp = next((w for w in res.waypoints if w.name == name), None)
    if wp is None:
        raise TaskError(
            f"'{name}' waypoint 없음 (robot={robot_id}) — {teach_hint}"
        )
    return wp


# ─── 검출 (so101 카메라 스윕 — pick_and_place detect 계승) ────────────


@step(title="검출")
async def detect(
    ctx: TaskContext, so101: str, prompt: str
) -> list[OrientedDetection]:
    """search 그룹 자세 전부 순회 → 후보 누적 (so101 카메라 — omx 는 카메라
    없음, 검출은 전부 so101 몫)."""
    groups = await ctx.call(
        Waypoint.Service.LIST_GROUPS,
        ListGroupsRequest(robot_id=so101),
        ListGroupsResponse,
    )
    grp = next((g for g in groups.groups if g.name == _SEARCH_GROUP), None)
    if grp is None or grp.id is None:
        raise TaskError(
            f"'{_SEARCH_GROUP}' waypoint 그룹 없음 (robot={so101}) — 검색 자세를 "
            "티칭해 그룹으로 저장한 뒤 다시 실행하세요"
        )
    members = await ctx.call(
        Waypoint.Service.LIST_GROUP_MEMBERS,
        ListGroupMembersRequest(group_row_id=grp.id),
        ListGroupMembersResponse,
    )
    if not members.waypoints:
        raise TaskError(f"'{_SEARCH_GROUP}' 그룹이 비어있음 (robot={so101})")
    t0 = time.monotonic()
    cands: list[OrientedDetection] = []
    for wp in members.waypoints:
        await _move_j_joints(ctx, so101, wp.joint_values)
        await asyncio.sleep(_SEARCH_SETTLE_S)
        res = await ctx.call(
            Detector.Service.DETECT_ORIENTED,
            DetectRequest(robot_id=so101, prompts=[prompt], top_k=_TOP_K),
            DetectOrientedResponse,
        )
        cands.extend(res.candidates)
    logger.info(
        "detect(%s): %d 자세 → 후보 %d (%.1fs)",
        prompt, len(members.waypoints), len(cands), time.monotonic() - t0,
    )
    return cands


def select_pick_candidate(
    cands: list[OrientedDetection], prompt: str
) -> OrientedDetection:
    """신뢰 게이트(score/폭/base_z 대역) 후 최고 score — pick_and_place 와
    같은 실측 근거의 컷 (오검출/공중/개구 초과는 후보가 아니다)."""
    trusted = [
        c for c in cands
        if c.score >= _SCORE_MIN
        and c.footprint[1] <= _MAX_WIDTH_M
        and _BASE_Z_MIN_M <= c.base_z <= _BASE_Z_MAX_M
    ]
    if not trusted:
        raise DetectionNotFound(
            prompt,
            candidates=len(cands),
            reason=(
                f"검출 {len(cands)}건 전부 신뢰 컷 미달 (score≥{_SCORE_MIN}, "
                f"폭≤{_MAX_WIDTH_M * 1000:.0f}mm, base_z 대역) — 물체 위치/조명 "
                "확인 후 다시 실행하세요"
            ),
        )
    return max(trusted, key=lambda c: c.score)


# ─── omx pick (open-loop — 크로스캘 정확도 의존, 미검증) ──────────────


@step(title="omx 집기 계획")
async def plan_omx_pick(
    ctx: TaskContext, omx: str, obj_world: OrientedDetection, base_omx: BasePose
) -> tuple[list[list[float]], Quat, Vec3]:
    """world 검출 → omx frame 변환 → [pre, grasp, lift] 그룹 resolve.

    반환 = (관절해 3개, 자세 quat, omx frame 파지점). yaw 는 물체 OBB 기준
    2방향 × tilt 사다리 — 도달 판정은 omx motion resolve (omx 의 손목 제약을
    여기서 추측하지 않는다)."""
    g_world = (
        obj_world.position[0],
        obj_world.position[1],
        obj_world.position[2] - _OMX_GRIP_BELOW_TOP_M,
    )
    g_omx = world_to_robot(g_world, base_omx)
    groups: list[list[TcpPose]] = []
    metas: list[tuple[Quat, Vec3]] = []
    # 물체 yaw 도 omx frame 으로 (base 간 yaw 차 보정)
    obj_yaw_omx = obj_world.grasp_yaw - base_omx.yaw_rad
    for tilt in _OMX_TILTS_DEG:
        for yaw_off in (math.pi / 2, 0.0):  # 짧은 변 물기 우선
            yaw = obj_yaw_omx + yaw_off
            quat = _grasp_quat(yaw, tilt)
            a = _approach_of(yaw, tilt)
            pre = (
                g_omx[0] - a[0] * _OMX_PRE_CLEAR_M,
                g_omx[1] - a[1] * _OMX_PRE_CLEAR_M,
                g_omx[2] - a[2] * _OMX_PRE_CLEAR_M,
            )
            lift = (g_omx[0], g_omx[1], g_omx[2] + _OMX_LIFT_M)
            groups.append([
                TcpPose(position=pre, quaternion=quat),
                TcpPose(position=g_omx, quaternion=quat),
                TcpPose(position=lift, quaternion=quat),
            ])
            metas.append((quat, g_omx))
    res = await ctx.call(
        Motion.Service.RESOLVE_REACHABLE,
        ResolveReachableRequest(
            groups=groups,
            floor_z=world_to_robot(
                (0.0, 0.0, obj_world.base_z - 0.005), base_omx
            )[2],
            linear=True,
        ),
        ResolveReachableResponse,
        robot_id=omx,
    )
    if res.index < 0:
        raise NoReachableGrasp(
            f"omx 집기 후보 {len(groups)}개 전멸 — {res.message}. 물체를 omx "
            "쪽으로 옮기거나 handover 배치를 조정하세요"
        )
    quat, g = metas[res.index]
    logger.info(
        "plan_omx_pick: 그룹 %d/%d 채택 — grasp(omx)=(%.3f,%.3f,%.3f)",
        res.index, len(groups), g[0], g[1], g[2],
    )
    return res.solutions, quat, g


@step(title="omx 집기")
async def omx_pick(
    ctx: TaskContext, omx: str, sols: list[list[float]], quat: Quat, g_omx: Vec3
) -> None:
    """pre(관절해) → grasp(직선) → close → 판정 → lift(직선). open-loop —
    실측 오차는 크로스캘 + omx FK 의 합 (모듈 docstring 가정 ②)."""
    await _move_j_joints(ctx, omx, sols[0])
    await _move_l(ctx, omx, g_omx, quat)
    await set_gripper(ctx, omx, open_=False)
    await verify_grasp(ctx, omx, phase="omx close 직후")
    lift = (g_omx[0], g_omx[1], g_omx[2] + _OMX_LIFT_M)
    await _move_l(ctx, omx, lift, quat)
    await verify_grasp(ctx, omx, phase="omx lift 후")


@step(title="omx 내밀기")
async def omx_present(
    ctx: TaskContext, omx: str, handover_wp: WaypointRecord
) -> None:
    """물체를 든 채 handover 자세로 — 티칭된 waypoint (모듈 docstring 가정 ③)."""
    logger.info("omx_present → '%s'", handover_wp.name)
    await _move_j_joints(ctx, omx, handover_wp.joint_values)
    await verify_grasp(ctx, omx, phase="handover 자세 도달")


# ─── so101 수취 (충돌 게이트 포함) ────────────────────────────────────


@step(title="수취 계획")
async def plan_receive(
    ctx: TaskContext,
    so101: str,
    omx: str,
    base_omx: BasePose,
    checker: CrossRobotChecker | None,
) -> tuple[list[list[float]], Quat, Vec3, list[float]]:
    """omx TCP(FK)로 물체 world 위치 추정 → so101 수평 계열 접근 resolve →
    **충돌 게이트**: 채택 관절해가 omx 현재 구성과 근접하면 그 그룹을 빼고
    재-resolve (상한 _RECV_COLLISION_RETRY, 소진 = 명시 실패).

    v1 은 재검출 없이 FK 기반 (미검증 — 오차 예산은 모듈 docstring ②). TODO:
    so101 D405 재검출로 정밀화 (공중 물체 검출은 base_z 대역을 이 task 전용으로
    열어야 함 — pick_and_place 의 테이블 대역과 다름)."""
    omx_tcp = await ctx.call(
        Motion.Service.TCP_SNAPSHOT, TcpSnapshotRequest(), TcpState, robot_id=omx
    )
    p = omx_tcp.position
    obj_world = robot_to_world((p[0], p[1], p[2]), base_omx)
    omx_joints = list(omx_tcp.joints)
    logger.info(
        "plan_receive: 물체 world 추정=(%.3f,%.3f,%.3f) (omx FK 기반)",
        obj_world[0], obj_world[1], obj_world[2],
    )
    # 접근 부채꼴 기준 yaw = so101 base→물체 방향 (물체 너머로 뻗지 않게)
    toward = math.atan2(obj_world[1], obj_world[0])
    groups: list[list[TcpPose]] = []
    metas: list[tuple[Quat, Vec3]] = []
    for tilt in _RECV_TILTS_DEG:
        for yaw_off_deg in _RECV_YAWS_DEG:
            yaw = toward + math.radians(yaw_off_deg)
            quat = _grasp_quat(yaw, tilt)
            a = _approach_of(yaw, tilt)
            pre = (
                obj_world[0] - a[0] * _RECV_PRE_CLEAR_M,
                obj_world[1] - a[1] * _RECV_PRE_CLEAR_M,
                obj_world[2] - a[2] * _RECV_PRE_CLEAR_M,
            )
            groups.append([
                TcpPose(position=pre, quaternion=quat),
                TcpPose(position=obj_world, quaternion=quat),
            ])
            metas.append((quat, obj_world))
    alive = list(range(len(groups)))
    for attempt in range(_RECV_COLLISION_RETRY):
        res = await ctx.call(
            Motion.Service.RESOLVE_REACHABLE,
            ResolveReachableRequest(
                groups=[groups[i] for i in alive], linear=True
            ),
            ResolveReachableResponse,
            robot_id=so101,
        )
        if res.index < 0:
            raise NoReachableGrasp(
                f"수취 접근 후보 전멸 ({len(alive)}개) — {res.message}. "
                "'handover' waypoint 를 so101 쪽으로 조정 후 다시 실행하세요"
            )
        gi = alive[res.index]
        if checker is None or not checker.path_in_collision(
            res.solutions, omx_joints
        ):
            quat, obj = metas[gi]
            return res.solutions, quat, obj, omx_joints
        logger.warning(
            "plan_receive: 그룹 %d 채택안이 omx 와 충돌 위험 (margin %.0fmm) — "
            "제외 후 재시도 %d/%d",
            gi, checker.margin_m * 1000, attempt + 1, _RECV_COLLISION_RETRY,
        )
        alive.remove(gi)
        if not alive:
            break
    raise NoReachableGrasp(
        "수취 접근 전부 omx 와 충돌 위험 — 'handover' waypoint 를 두 로봇이 "
        "더 벌어지는 자세로 재티칭하세요"
    )


@step(title="수취")
async def receive(
    ctx: TaskContext,
    so101: str,
    omx: str,
    sols: list[list[float]],
    quat: Quat,
    obj_world: Vec3,
) -> None:
    """so101 접근 → close → **held 확인 후에만** omx open → so101 이탈.

    수취 순서 불변식 (모듈 docstring): so101 판정 전 omx 를 열면 물체 낙하 —
    회귀 테스트가 호출 순서를 잠근다."""
    await _move_j_joints(ctx, so101, sols[0])
    await _move_l(ctx, so101, obj_world, quat)
    await set_gripper(ctx, so101, open_=False)
    await verify_grasp(ctx, so101, phase="수취 close 직후")
    # so101 확보 확인 완료 — 이제 giver 가 놓는다
    await set_gripper(ctx, omx, open_=True)
    a = _approach_of_quat(quat)
    withdraw = (
        obj_world[0] - a[0] * _RECV_WITHDRAW_M,
        obj_world[1] - a[1] * _RECV_WITHDRAW_M,
        obj_world[2] - a[2] * _RECV_WITHDRAW_M,
    )
    await _move_l(ctx, so101, withdraw, quat)
    await verify_grasp(ctx, so101, phase="수취 이탈 후")


@step(title="omx 복귀")
async def omx_retreat(
    ctx: TaskContext,
    omx: str,
    so101: str,
    home_omx: WaypointRecord,
    checker: CrossRobotChecker | None,
) -> None:
    """omx home 복귀 — 복귀 관절 경로를 so101 현재 구성과 충돌 검사 (표본 lerp,
    collision.path_in_collision). 충돌 위험이면 **정지 유지 + 명시 실패**
    (so101 이 물체를 들고 있으므로 omx 가 멈추는 쪽이 안전)."""
    if checker is not None:
        so_tcp = await ctx.call(
            Motion.Service.TCP_SNAPSHOT, TcpSnapshotRequest(), TcpState,
            robot_id=so101,
        )
        omx_tcp = await ctx.call(
            Motion.Service.TCP_SNAPSHOT, TcpSnapshotRequest(), TcpState,
            robot_id=omx,
        )
        # 주의: path_in_collision 의 인자 순서는 (a 경로, b 구성) — checker 는
        # a=so101, b=omx 로 생성되므로 여기선 so101 고정/omx 경로를 뒤집어
        # b 경로 검사가 필요하다. v1 은 끝점 구성 쌍만 검사 (보수 margin 이
        # 커버) — TODO: 방향별 경로 검사 API (실물 검증 단계에서).
        if checker.in_collision(
            list(so_tcp.joints), list(home_omx.joint_values)
        ) or checker.in_collision(list(so_tcp.joints), list(omx_tcp.joints)):
            raise TaskError(
                "omx 복귀 경로가 so101 과 충돌 위험 — omx 정지 유지. so101 을 "
                "먼저 적치/이탈시킨 뒤 omx 를 수동 복귀하세요"
            )
    await _move_j_joints(ctx, omx, home_omx.joint_values)


def _approach_of_quat(quat: Quat) -> Vec3:
    a = Rotation.from_quat(quat).apply([1.0, 0.0, 0.0])
    return (float(a[0]), float(a[1]), float(a[2]))


# ─── 적치 (pick_and_place 슬림판 — open-loop) ─────────────────────────


@step(title="적치")
async def place_into(
    ctx: TaskContext,
    so101: str,
    prompt: str,
    held_height_m: float,
    home_so: WaypointRecord,
) -> None:
    """상자 검출 → [pre, place] resolve → 접근/삽입/release/후퇴.

    pick_and_place plan_place 의 슬림판 (정렬 4 yaw × tilt 5 — 폴백 자유 yaw
    가족은 생략, 필요해지면 그대로 이식). 후퇴는 pre 관절해 MoveJ (07-17
    retreat 실행 IK 실사고 회피 — 계획 해 재사용)."""
    spots = await detect(ctx, so101, prompt)
    ranked = sorted(
        (s for s in spots if -0.04 <= s.base_z <= _BASE_Z_MAX_M),
        key=lambda s: s.score,
        reverse=True,
    )
    if not ranked:
        raise TaskError(
            f"'{prompt}' 적치 대상 검출 0건 (타당 대역) — 상자 배치 확인 후 "
            "다시 실행하세요"
        )
    for spot in ranked:
        place_z = spot.position[2] + held_height_m * 0.5 + _PLACE_DROP_CLEAR_M
        groups: list[list[TcpPose]] = []
        metas: list[tuple[Quat, Vec3, Vec3]] = []
        for tilt in _PLACE_TILTS_DEG:
            for off in _PLACE_YAW_OFFSETS_DEG:
                yaw = spot.grasp_yaw + math.radians(off)
                quat = _grasp_quat(yaw, tilt)
                a = _approach_of(yaw, tilt)
                place = (spot.position[0], spot.position[1], place_z)
                pre = (
                    place[0] - a[0] * _PLACE_PRE_CLEAR_M,
                    place[1] - a[1] * _PLACE_PRE_CLEAR_M,
                    place[2] - a[2] * _PLACE_PRE_CLEAR_M,
                )
                groups.append([
                    TcpPose(position=pre, quaternion=quat),
                    TcpPose(position=place, quaternion=quat),
                ])
                metas.append((quat, place, pre))
        res = await ctx.call(
            Motion.Service.RESOLVE_REACHABLE,
            ResolveReachableRequest(
                groups=groups, floor_z=spot.base_z - 0.005, linear=True
            ),
            ResolveReachableResponse,
            robot_id=so101,
        )
        if res.index < 0:
            logger.info(
                "place_into: spot score=%.2f 전멸 — 다음 spot (%s)",
                spot.score, res.message,
            )
            continue
        quat, place, _pre = metas[res.index]
        await _move_j_joints(ctx, so101, res.solutions[0])
        await _move_l(ctx, so101, place, quat)
        await verify_grasp(ctx, so101, phase="적치 직전")
        await set_gripper(ctx, so101, open_=True)
        try:
            await _move_l(ctx, so101, _pre, quat)
        except asyncio.CancelledError:
            raise
        except Exception as e:
            logger.warning("place 후퇴 MoveL 실패 (%s) — pre 관절해 MoveJ 폴백", e)
            await _move_j_joints(ctx, so101, res.solutions[0])
        await _move_j_joints(ctx, so101, home_so.joint_values)
        return
    raise NoReachableGrasp(
        f"적치 spot {len(ranked)}건 전부 도달 불가 — 상자를 so101 쪽으로 "
        "옮긴 뒤 다시 실행하세요"
    )


# ─── 공용 primitive (pick_and_place 계승 — 계약 동일) ────────────────


@step(title="home 경유")
async def go_home(ctx: TaskContext, robot_id: str, home: WaypointRecord) -> None:
    logger.info("go_home robot=%s → '%s'", robot_id, home.name)
    await _move_j_joints(ctx, robot_id, home.joint_values)


@step(title="그리퍼")
async def set_gripper(ctx: TaskContext, robot_id: str, *, open_: bool) -> None:
    spec = ctx.spec(robot_id)
    raw = spec.gripper_open_raw if open_ else spec.gripper_close_raw
    logger.info(
        "gripper robot=%s → %s (raw=%d)",
        robot_id, "OPEN" if open_ else "CLOSE", raw,
    )
    await ctx.call(
        Motor.Service.SET_GRIPPER,
        SetGripperRequest(position_raw=raw),
        SetGripperResponse,
        robot_id=robot_id,
    )
    await asyncio.sleep(_GRIPPER_SETTLE_S)


@step(title="파지 확인")
async def verify_grasp(ctx: TaskContext, robot_id: str, *, phase: str) -> None:
    """gap OR load 판정 (pick_and_place _gripper_holding 동일 규약) — 미달이면
    GraspFailed. 판정 근거 전부 로깅 (실물 임계 튜닝 데이터)."""
    spec = ctx.spec(robot_id)
    state = await ctx.call(
        Motor.Service.READ_STATE, ReadStateRequest(), JointState, robot_id=robot_id
    )
    gi = spec.gripper_index
    achieved = state.positions_raw[gi]
    load = (
        state.loads_raw[gi]
        if state.loads_raw is not None and gi < len(state.loads_raw)
        else None
    )
    margin = abs(spec.gripper_held_threshold_raw - spec.gripper_close_raw)
    gap = abs(achieved - spec.gripper_close_raw)
    held = gap > margin or (load is not None and load >= _HELD_LOAD_MIN_RAW)
    logger.info(
        "verify_grasp[%s] robot=%s achieved=%d (close=%d thr=%d load=%s) → %s",
        phase, robot_id, achieved, spec.gripper_close_raw,
        spec.gripper_held_threshold_raw, load, "HELD" if held else "EMPTY",
    )
    if not held:
        raise GraspFailed(
            phase=phase,
            achieved_raw=achieved,
            close_raw=spec.gripper_close_raw,
            load_raw=load,
        )


async def _move_j_joints(
    ctx: TaskContext, robot_id: str, joints: list[float]
) -> None:
    await ctx.call(
        Motion.Service.MOVE_J,
        MoveJRequest(target=JointTarget(kind="joint", joints=list(joints))),
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
