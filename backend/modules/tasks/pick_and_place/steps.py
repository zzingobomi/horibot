from __future__ import annotations

import asyncio
import logging
import math

import numpy as np

from modules.calibration.contract import (
    Calibration,
    CalibrationBundle,
    SnapshotBundleRequest,
)
from modules.detector.contract import (
    DetectOrientedResponse,
    DetectRequest,
    Detector,
    FuseOrientedRequest,
    FuseOrientedResponse,
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
    ListWaypointsRequest,
    ListWaypointsResponse,
    Waypoint,
    WaypointRecord,
)

from . import antipodal, geometry
from .geometry import GraspCandidate, PlaceCandidate, Quat, Vec3

logger = logging.getLogger(__name__)

_GRIPPER_SETTLE_S = 1.2
_TOP_K = 5

# 검색 자세 그룹 — 사용자가 티칭한 "search" waypoint 그룹 (robot 별). 이 자세들을
# 모두 돌며 관측한다 (§ multi-view). 값은 로봇마다 다른 관측 시점 = 사람이 배치.
_SEARCH_GROUP = "search"
_SEARCH_SETTLE_S = 0.3  # MoveJ 후 카메라 흔들림 정착 대기 (검출 품질)

# 경유 자세 waypoint — 긴 이동(관측→접근, 후퇴→적치)과 관측 뷰 간 이동이 관절
# 공간으로 여기를 거친다 (pick↔place 도달성 분리 + 뷰 간 naive MoveJ 금지,
# grasp_redesign_journey.md §5.4/§10.4-4). 사용자가 티칭.
# motors.yaml home(2048 중심값)은 joint3 URDF limit 밖이라 못 씀 (2026-07-14 확인).
_HOME_WAYPOINT = "home"

# 바닥 충돌 게이트 평면을 검출 base_z 보다 살짝 내리는 버퍼 — 게이트는 cm-급
# 육안 충돌(수평 접근 시 그리퍼 몸통 등)용이고, mm-급 여유는 geometry 의
# clearance 상수 책임. 검출/캘 σ(~8mm)로 정상 파지가 영구 기각되는 것 방지.
_FLOOR_GATE_MARGIN_M = 0.005

# ─── 타깃 중심 adaptive 멀티뷰 관측 (§10.4-1) ───
# 같은 물체 판정 반경 — 다른 뷰의 검출을 coarse 타깃과 잇는다 (base frame 정렬 전제).
_VIEW_MATCH_RADIUS_M = 0.05
# 도달 성공 뷰 상한 — sim 검증(§10.3-G)은 2~4뷰면 파지가 선다. 이만큼 봐도 안
# 서면 관측으로 풀릴 문제가 아니다 → 사유 있는 실패.
_VIEW_MAX_REACHED = 6
# 이웃 장애물 수집 반경 — 이 안의 다른 검출 군집 점군을 그리퍼 충돌 게이트에
# 장애물로 넣는다 (§10.4-3 "이웃". 이보다 먼 물체는 파지 접근과 무관).
_NEIGHBOR_RADIUS_M = 0.15


# ─── scenario: 계획(plan) → 실행(execute) 분리 ─────────────────────
#
# 순서 규약 (2026-07-13): 물리 파지 **전에** 집기·놓기 도달성을 모두 검증한다.
# 옛 구조(집기 완주 후 놓기 검출/IK)는 놓을 곳이 도달 불가일 때 이미 물체를 쥔
# 채 실패해 로봇이 물체를 든 채 멈추는 corrupt 상태를 만들었다 (2026-07-13
# resolve_place IK 불가 실패). 계획 단계는 모션 0 (검출 + 배치 IK 판정뿐)이 아닌
# 관측 이동만 있고, 어느 한쪽이라도 도달 불가면 아무것도 집기 전에 실패한다.


@step(title="집기 계획")
async def plan_pick(
    ctx: TaskContext, robot_id: str, prompt: str, home: WaypointRecord
) -> tuple[OrientedDetection, GraspCandidate, list[float]]:
    """검출 → adaptive 멀티뷰 관측·파지 성립 검사 → (대상, 후보, pre 관절해).

    object-centric (§5.1-2): 대상 기하는 융합 점군에서, 파지는 관측 표면의
    antipodal 쌍에서 (§10.4-2). 관측 충분성의 심판 = "실행 가능한 파지가 섰나"
    (height 하드게이트 폐기 — §10.4-6). pre 관절해 = resolve 가 반환한 IK 해
    (실행부 재계산 없음, §5.5). home = 뷰 간/접근 이동의 경유 자세 (§10.4-4)."""
    cands = await detect(ctx, robot_id, prompt)
    coarse = geometry.select_target_by_score(cands, prompt=prompt)
    return await observe_and_plan_grasp(ctx, robot_id, cands, coarse, prompt, home)


@step(title="놓기 계획")
async def plan_place(
    ctx: TaskContext,
    robot_id: str,
    prompt: str,
    *,
    held: OrientedDetection,
    grasp: GraspCandidate,
    home: WaypointRecord,
) -> tuple[PlaceCandidate, list[float]]:
    """검출 + 적치 후보 게이트 판정 (모션 0) → (적치 후보, pre 관절해).

    적치엔 멀티뷰 불요 — place_z 는 spot **윗면**(단일 뷰로 실측)과 held(융합
    완료)의 height 에서 나온다. 물체 dims 는 검출(held)에서 오므로 물리 파지
    전에도 계획 가능."""
    spots = await detect(ctx, robot_id, prompt)
    spot = geometry.select_target_by_score(spots, prompt=prompt)
    pplan = geometry.plan_place(spot, held=held, lateral=grasp.lateral)
    idx, sols = await resolve_place(
        ctx, robot_id, pplan,
        floor_z=spot.base_z - _FLOOR_GATE_MARGIN_M,
        home=home,
    )
    return pplan[idx], sols[0]


@step(title="집기 실행")
async def execute_pick(
    ctx: TaskContext,
    robot_id: str,
    c: GraspCandidate,
    pre_joints: list[float],
    home: WaypointRecord,
) -> None:
    """계획된 파지 후보로 실제 파지 — home 경유 → 접근 → 진입 → 파지 → 후퇴 → home.

    진입/후퇴는 접근축(툴 x) 기준 MoveL (월드 +z 아님 — §5.4 grasp-frame 동작),
    긴 이동(관측 자세↔접근, 든 채 적치로)은 관절공간 home 경유 — resolve 의
    ④ 경로 게이트(path_from=home)가 이 home→pre MoveJ 를 계획 시점에 검증했다."""
    await go_home(ctx, robot_id, home)
    await pre_grasp(ctx, robot_id, pre_joints)
    await open_gripper(ctx, robot_id)
    await advance(ctx, robot_id, c)
    await close_gripper(ctx, robot_id)
    await withdraw(ctx, robot_id, c)
    await go_home(ctx, robot_id, home)


@step(title="놓기 실행")
async def execute_place(
    ctx: TaskContext,
    robot_id: str,
    c: PlaceCandidate,
    pre_joints: list[float],
    home: WaypointRecord,
) -> None:
    """계획된 적치 후보로 실제 적치 — 접근 → 삽입 → 내려놓기 → 후퇴 → home.

    home 에서 시작 (execute_pick 이 home 으로 끝남). 종료도 home — 다음 run 의
    시작 자세가 일정하고 카메라 시야에서 팔이 빠진다."""
    await pre_place(ctx, robot_id, pre_joints)
    await insert(ctx, robot_id, c)
    await release(ctx, robot_id)
    await retreat(ctx, robot_id, c)
    await go_home(ctx, robot_id, home)


# ─── planning ──────────────────────────────────────


@step(title="검출")
async def detect(
    ctx: TaskContext, robot_id: str, prompt: str
) -> list[OrientedDetection]:
    """search 그룹 자세를 **전부** 돌며 검출 → 후보 **누적** (첫 자세에서 안 멈춤).

    원리 (옛 SearchWaypointGroup 포팅): 단일 시점 검출은 가림/시야/각도로 놓치거나
    오검출한다. 사람이 티칭한 여러 관측 자세를 다 돌아 후보를 모으면(모두 base frame
    이라 비교 가능) 관측이 많아 강건하다. **선택은 안 함** — 누적만. "자세 다 돌고
    진짜 제일 점수 높은 것" 판정은 select_target_by_score 가 누적 전체에서.
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
    # 진단: 후보별 object-centric 기하 (물체 자기 점군 기준 — 단일 뷰 height 는
    # 옆면 depth 부재로 과소가 정상 — 파지 성립 검사가 멀티뷰 융합 후 심판).
    for i, c in enumerate(candidates):
        logger.info(
            "  후보%d: score=%.2f height(단일뷰)=%.1fcm base_z(물체바닥)=%.3fm "
            "top=%.3fm pos=(%.3f,%.3f)",
            i, c.score, c.height * 100.0, c.base_z, c.position[2],
            c.position[0], c.position[1],
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


@step(title="타깃 관측·파지 성립")
async def observe_and_plan_grasp(
    ctx: TaskContext,
    robot_id: str,
    cands: list[OrientedDetection],
    coarse: OrientedDetection,
    prompt: str,
    home: WaypointRecord,
) -> tuple[OrientedDetection, GraspCandidate, list[float]]:
    """adaptive 멀티뷰 (§10.4-1) — 관측을 누적하며 매번 파지 성립을 검사, 서면
    멈춘다 (sim: 2~4뷰). 안 서면 다음 뷰는 spread-first 방위 (§10.3-B — antipodal
    은 마주 보는 면 관측이 필요해 벌어진 방위부터).

    - 관측 자세 스크리닝 = motion resolve: IK+self 만이 아니라 **floor + 물체/이웃
      점군 충돌까지** (§10.3-H 원인 (1) 의 수정). roll 변형을 그룹으로 묶어 첫
      가용 roll 채택.
    - 뷰 간 이동 = home 경유 + resolve ④ 경로 게이트(path_from=home) — naive
      MoveJ 금지 (§10.3-H 원인 (2), §10.4-4).
    - 도달 불가 뷰 방향 = 스킵 (부정 데이터). 상한까지 봐도 파지가 안 서면
      사유 있는 명시 실패 (§10.4-3 "안전 파지 불가" — 맹목 파지 금지).
    """
    bundle = await ctx.call(
        Calibration.Service.SNAPSHOT_BUNDLE,
        SnapshotBundleRequest(robot_id=robot_id),
        CalibrationBundle,
    )
    if bundle.hand_eye is None:
        raise TaskError(
            f"hand_eye 캘 없음 (robot={robot_id}) — 멀티뷰 관측 불가, 캘 먼저"
        )
    he = bundle.hand_eye.result_data

    # 검색 스윕이 이미 여러 자세에서 본 타깃 관측 = 공짜 멀티뷰 시드.
    observations = [coarse] + [
        c
        for c in cands
        if c is not coarse
        and _xy_dist(c.position, coarse.position) <= _VIEW_MATCH_RADIUS_M
    ]
    neighbors = _neighbor_points(cands, coarse)
    floor_z = coarse.base_z - _FLOOR_GATE_MARGIN_M

    found = await try_plan_grasp(
        ctx, robot_id, observations, neighbors, prompt, home
    )
    if found is not None:
        logger.info("observe_and_plan_grasp(%s): 검색 스윕 관측만으로 파지 성립", prompt)
        return found

    reached = 0
    for radius, elev, az in geometry.view_directions(coarse.position):
        if reached >= _VIEW_MAX_REACHED:
            break
        poses = geometry.view_pose_groups(
            coarse.position, he.R_cam2gripper, he.t_cam2gripper,
            radius_m=radius, elev_deg=elev, az_rad=az,
        )
        res = await ctx.call(
            Motion.Service.RESOLVE_REACHABLE,
            ResolveReachableRequest(
                groups=[[TcpPose(position=p, quaternion=q)] for p, q in poses],
                floor_z=floor_z,
                obstacle_points=_observation_points(observations, neighbors),
                path_from=list(home.joint_values),
            ),
            ResolveReachableResponse,
            robot_id=robot_id,
        )
        if res.index < 0:
            continue  # 이 뷰 방향은 도달/안전 불가 — 다음 방향 (부정 데이터)
        reached += 1
        await go_home(ctx, robot_id, home)  # 뷰 간 이동 = home 경유 (§10.4-4)
        await _move_j_joints(ctx, robot_id, res.solutions[0])
        await asyncio.sleep(_SEARCH_SETTLE_S)
        det = await ctx.call(
            Detector.Service.DETECT_ORIENTED,
            DetectRequest(robot_id=robot_id, prompt=prompt, top_k=_TOP_K),
            DetectOrientedResponse,
        )
        near = _nearest_within(
            det.candidates, coarse.position, _VIEW_MATCH_RADIUS_M
        )
        if near is None:
            logger.info(
                "observe_and_plan_grasp: 뷰 %d(고도 %.0f° 방위 %.0f°)에서 타깃 "
                "미검출 — 새 정보 없음", reached, elev, math.degrees(az),
            )
            continue
        observations.append(near)
        found = await try_plan_grasp(
            ctx, robot_id, observations, neighbors, prompt, home
        )
        if found is not None:
            logger.info(
                "observe_and_plan_grasp(%s): 추가 뷰 %d/관측 %d건에서 파지 성립",
                prompt, reached, len(observations),
            )
            return found

    raise NoReachableGrasp(
        f"안전 파지 불가 — '{prompt}' 관측 {len(observations)}건(추가 뷰 {reached}"
        f"/{_VIEW_MAX_REACHED})으로도 실행 가능한 antipodal 파지가 안 섰습니다. "
        "물체가 workspace 경계이거나 주변이 빽빽할 수 있습니다 — 물체를 로봇 "
        "쪽으로 옮기거나 주변을 비운 뒤 다시 실행하세요"
    )


@step(title="파지 성립 검사")
async def try_plan_grasp(
    ctx: TaskContext,
    robot_id: str,
    observations: list[OrientedDetection],
    neighbors: list[Vec3],
    prompt: str,
    home: WaypointRecord,
) -> tuple[OrientedDetection, GraspCandidate, list[float]] | None:
    """현재 관측으로 파지가 서는지 — 융합 → 표면 antipodal → resolve 게이트.

    None = 아직 안 섬 (부정 데이터 — 관측을 더 쌓으라는 신호). 성립하면
    (융합 타깃, 채택 후보, pre 관절해). resolve 게이트: 끝점 IK + 바닥 +
    **그리퍼(벌림)↔물체·이웃 점군 충돌** + home→pre 관절 경로 + pre→grasp
    직선 경로 (§10.4-3)."""
    fused = await fuse_target(ctx, observations, prompt)
    if fused is None or not fused.points:
        return None
    pairs = await asyncio.to_thread(  # open3d 로드/법선 추정 — blocking
        antipodal.horizontal_antipodal_pairs, np.asarray(fused.points, dtype=float)
    )
    if not pairs:
        logger.info(
            "try_plan_grasp(%s): antipodal 쌍 0 (관측 %d건) — 마주 보는 면 미관측",
            prompt, len(observations),
        )
        return None
    plan = geometry.plan_grasp(pairs)
    res = await ctx.call(
        Motion.Service.RESOLVE_REACHABLE,
        ResolveReachableRequest(
            groups=geometry.grasp_ik_groups(plan),
            floor_z=fused.base_z - _FLOOR_GATE_MARGIN_M,
            linear=True,
            obstacle_points=[*fused.points, *neighbors],
            gripper_open=True,
            path_from=list(home.joint_values),
        ),
        ResolveReachableResponse,
        robot_id=robot_id,
    )
    if res.index < 0:
        logger.info(
            "try_plan_grasp(%s): 쌍 %d/후보 %d 전멸 — %s",
            prompt, len(pairs), len(plan), res.message,
        )
        return None
    logger.info(
        "try_plan_grasp(%s): group %d 채택 — %s (쌍 %d, height=%.1fcm "
        "base_z=%.3f)",
        prompt, res.index, plan[res.index].label, len(pairs),
        fused.height * 100.0, fused.base_z,
    )
    return fused, plan[res.index], res.solutions[0]


@step(title="관측 융합")
async def fuse_target(
    ctx: TaskContext, observations: list[OrientedDetection], prompt: str
) -> OrientedDetection | None:
    """멀티뷰 관측 융합 (detector FUSE — 점군 합쳐 기하 재계산) → 타깃 후보.

    None = 융합 결과에 타깃 위치 군집 없음 (관측 점군 부족 — 부정 데이터,
    관측을 더 쌓는다). height 하드게이트는 없다 (§10.4-6 — 심판은 파지 성립).
    robot 무관 순수 계산이라 robot_id 없음 (§2.7 agnostic)."""
    res = await ctx.call(
        Detector.Service.FUSE_ORIENTED,
        FuseOrientedRequest(candidates=observations),
        FuseOrientedResponse,
    )
    fused = _nearest_within(
        res.candidates, observations[0].position, _VIEW_MATCH_RADIUS_M
    )
    if fused is None:
        logger.info(
            "fuse_target(%s): 관측 %d건 융합에 타깃 군집 없음",
            prompt, len(observations),
        )
        return None
    logger.info(
        "fuse_target(%s): 관측 %d건 → height=%.1fcm base_z=%.3f points=%d",
        prompt, len(observations), fused.height * 100.0, fused.base_z,
        len(fused.points or []),
    )
    return fused


def _xy_dist(a: Vec3, b: Vec3) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _nearest_within(
    cands: list[OrientedDetection], anchor: Vec3, radius_m: float
) -> OrientedDetection | None:
    """anchor 와 XY 거리 radius_m 안의 최근접 후보 (없으면 None)."""
    best, best_d = None, radius_m
    for c in cands:
        d = _xy_dist(c.position, anchor)
        if d <= best_d:
            best, best_d = c, d
    return best


def _neighbor_points(
    cands: list[OrientedDetection], coarse: OrientedDetection
) -> list[Vec3]:
    """타깃 아닌 이웃 후보의 점군 — 그리퍼 충돌 게이트의 장애물 (§10.4-3 '이웃').

    같은 prompt 로 잡힌 다른 물체 군집 (매치 반경 밖 ~ _NEIGHBOR_RADIUS_M 안).
    다른 prompt 의 물체는 지금 관측 채널이 없다 — sim §10.3-I 의 빽빽 clutter
    fail-safe 는 관측된 이웃에 한해 성립 (미관측 장애물은 실물 몫)."""
    out: list[Vec3] = []
    for c in cands:
        d = _xy_dist(c.position, coarse.position)
        if d <= _VIEW_MATCH_RADIUS_M or d > _NEIGHBOR_RADIUS_M:
            continue
        out.extend(c.points or [])
    return out


def _observation_points(
    observations: list[OrientedDetection], neighbors: list[Vec3]
) -> list[Vec3]:
    """관측 자세 스크리닝용 장애물 점군 = 지금까지의 타깃 관측 + 이웃."""
    out: list[Vec3] = list(neighbors)
    for o in observations:
        out.extend(o.points or [])
    return out


@step(title="home 자세 조회")
async def home_waypoint(ctx: TaskContext, robot_id: str) -> WaypointRecord:
    """'home' waypoint 조회 (긴 이동 경유 자세). 없음 = 명시적 실패 — 모션 0
    시점(계획 전)에 걸리도록 시나리오 맨 앞에서 호출한다."""
    res = await ctx.call(
        Waypoint.Service.LIST,
        ListWaypointsRequest(robot_id=robot_id),
        ListWaypointsResponse,
    )
    wp = next((w for w in res.waypoints if w.name == _HOME_WAYPOINT), None)
    if wp is None:
        raise TaskError(
            f"'{_HOME_WAYPOINT}' waypoint 없음 (robot={robot_id}) — 픽↔플레이스"
            " 사이 경유할 안전 자세를 티칭해 'home' 으로 저장한 뒤 다시 실행하세요"
        )
    return wp


@step(title="적치 후보 선별")
async def resolve_place(
    ctx: TaskContext,
    robot_id: str,
    plan: list[PlaceCandidate],
    *,
    floor_z: float,
    home: WaypointRecord,
) -> tuple[int, list[list[float]]]:
    """게이트 판정 (위치→자세→바닥→home→pre 관절 경로→pre↔place 직선) — 모션 0.

    linear=True: 삽입(pre→place)이 MoveL 이므로 끝점만 풀리고 중간이 막히는
    후보를 계획 시점에 기각. path_from=home: execute_place 가 home 에서 pre 로
    MoveJ 하는 계약 (§10.4-4)."""
    res = await ctx.call(
        Motion.Service.RESOLVE_REACHABLE,
        ResolveReachableRequest(
            groups=geometry.place_ik_groups(plan),
            floor_z=floor_z,
            linear=True,
            path_from=list(home.joint_values),
        ),
        ResolveReachableResponse,
        robot_id=robot_id,
    )
    if res.index < 0:
        raise NoReachableGrasp(res.message)
    logger.info("resolve_place: group %d — %s", res.index, plan[res.index].label)
    return res.index, res.solutions


# ─── primitives ────────────


@step(title="home 경유")
async def go_home(ctx: TaskContext, robot_id: str, home: WaypointRecord) -> None:
    await _move_j_joints(ctx, robot_id, home.joint_values)


@step(title="파지 접근")
async def pre_grasp(
    ctx: TaskContext, robot_id: str, pre_joints: list[float]
) -> None:
    # resolve 가 반환한 관절 해 그대로 — 실행부 IK 재계산 없음 (§5.5)
    await _move_j_joints(ctx, robot_id, pre_joints)


@step(title="진입")
async def advance(ctx: TaskContext, robot_id: str, c: GraspCandidate) -> None:
    # pre→grasp 접근축 직선 (자세 동일 → slerp 가 자세 고정으로 수렴)
    await _move_l(ctx, robot_id, c.grasp, c.quat)


@step(title="후퇴")
async def withdraw(ctx: TaskContext, robot_id: str, c: GraspCandidate) -> None:
    # grasp→pre 접근축 역방향 — 월드 +z 들어올리기 폐기 (§5.4)
    await _move_l(ctx, robot_id, c.pre, c.quat)


@step(title="적치 접근")
async def pre_place(
    ctx: TaskContext, robot_id: str, pre_joints: list[float]
) -> None:
    await _move_j_joints(ctx, robot_id, pre_joints)


@step(title="삽입")
async def insert(ctx: TaskContext, robot_id: str, c: PlaceCandidate) -> None:
    await _move_l(ctx, robot_id, c.place, c.quat)


@step(title="적치 후퇴")
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
