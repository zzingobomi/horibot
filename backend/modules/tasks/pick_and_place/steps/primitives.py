"""로봇 기초 동작 + 파지 판정 — 집기(pick)와 놓기(place)가 함께 쓰는 바닥층.

이동(_move_l/_move_j_joints)·그리퍼(open/close/_set_gripper)·home 경유·파지
판정(verify_grasp — 물었나/놓쳤나)과 포맷 유틸. phase 파일(search/plan/pick/
place)은 전부 여기 위에 선다 — 역방향 import 금지 (계층: primitives ← phase).

servo 노브 SSOT(_SERVO_CFG)도 여기 — plan(사다리 구성)과 pick(servo 루프)이
같은 값을 봐야 하고, 테스트가 한 자리(steps.primitives._SERVO_CFG)만 패치하면
두 소비자에 다 걸리도록 소비 측은 `primitives._SERVO_CFG` 모듈 참조로 읽는다.
"""

from __future__ import annotations

import asyncio
import logging
import math

from modules.motion.contract import (
    JointTarget,
    Motion,
    MoveJRequest,
    MoveJResponse,
    MoveLRequest,
    MoveLResponse,
    PlanPathRequest,
    PlanPathResponse,
    PoseTarget,
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
from modules.tasks.core.errors import GraspFailed, TaskError
from modules.tasks.core.step import step
from modules.waypoint.contract import (
    ListWaypointsRequest,
    ListWaypointsResponse,
    Waypoint,
    WaypointRecord,
)

from modules.detector.contract import OrientedDetection

from .. import servo
from ..geometry import Quat, Vec3

logger = logging.getLogger(__name__)

# gripper 이동 완료 대기 — SET_GRIPPER 는 단발 goal 이라 완료 통지가 없다.
# 2026-07-17 close 30°/s 감속(타격 발사 방지)에 맞춰 1.2→4.0: open↔close 풀
# 스트로크 ~110° ≈ 3.7s. 짧으면 verify 가 닫히는 도중 raw 를 읽어 오판.
_GRIPPER_SETTLE_S = 4.0
_TOP_K = 5

# 경유 자세 waypoint — 긴 이동(관측→접근)이 관절 공간으로 여기를 거친다. 티칭 필수.
_HOME_WAYPOINT = "home"

# 같은 물체 판정 반경 — 융합 결과에서 타깃 군집을 찾을 때 (base frame 정렬 전제).
_VIEW_MATCH_RADIUS_M = 0.05

# servo 파라미터 SSOT (servo.ServoConfig docstring — 실물 첫 런 데이터로 튜닝).
_SERVO_CFG = servo.ServoConfig()


# ─── home / 이동 ──────────────────────────────────────────────────────


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


@step(title="home 경유")
async def go_home(ctx: TaskContext, robot_id: str, home: WaypointRecord) -> None:
    logger.info("go_home robot=%s → '%s'", robot_id, home.name)
    await _move_j_joints(ctx, robot_id, home.joint_values)


# ── 계획 이동 (transit) — home 허브 강등 (2026-07-22, docs/motion.md §12) ──
#
# 긴 이동(관측 전환/servo 진입/운반)의 기본 = PLAN_PATH 로 현재→목표 직접 계획,
# **실패(부정 결과/wire 예외)는 home 경유 폴백** — 옛 실행 계약 그대로라 동작
# 후퇴가 없고, resolve 게이트 ④(path_from=home)가 폴백 경로를 여전히 계획
# 시점에 증명한다 (플래너는 최적화지 의존성이 아니다 — 침묵 없이 로그).


@step(title="이동 (계획 경로)")
async def transit(
    ctx: TaskContext,
    robot_id: str,
    goal_joints: list[float],
    home: WaypointRecord,
    *,
    floor_z: float | None = None,
    obstacle_points: list[Vec3] | None = None,
    gripper_open: bool = False,
    tcp_min_z: float | None = None,
) -> None:
    """현재 자세 → goal_joints 계획 이동. 실행 = 계획 waypoint 순차 MoveJ
    (판정 경로 == 실행 경로 — resolve 의 판정 해 == 실행 해 원칙과 동일)."""
    res: PlanPathResponse | None = None
    try:
        res = await ctx.call(
            Motion.Service.PLAN_PATH,
            PlanPathRequest(
                goal_joints=list(goal_joints),
                floor_z=floor_z,
                obstacle_points=(
                    [(p[0], p[1], p[2]) for p in obstacle_points]
                    if obstacle_points else None
                ),
                gripper_open=gripper_open,
                tcp_min_z=tcp_min_z,
            ),
            PlanPathResponse,
            robot_id=robot_id,
        )
    except asyncio.CancelledError:
        raise
    except Exception as e:
        logger.warning(
            "transit robot=%s: PLAN_PATH 호출 실패 (%s) — home 경유 폴백",
            robot_id, e,
        )
    if res is not None and not res.found:
        logger.warning(
            "transit robot=%s: 경로 계획 실패 (%s, %.0fms) — home 경유 폴백",
            robot_id, res.message, res.planning_ms,
        )
    if res is None or not res.found:
        await go_home(ctx, robot_id, home)
        await _move_j_joints(ctx, robot_id, goal_joints)
        return
    logger.info(
        "transit robot=%s: %s (%.0fms, 검사 %d회) — 경유 %d + 목표",
        robot_id, "직선" if res.direct else "RRT",
        res.planning_ms, res.checks, len(res.waypoints),
    )
    for wp in res.waypoints:
        await _move_j_joints(ctx, robot_id, wp)
    await _move_j_joints(ctx, robot_id, goal_joints)


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


async def _move_l(
    ctx: TaskContext,
    robot_id: str,
    position: Vec3,
    quaternion: Quat,
    *,
    speed_scale: float = 1.0,
) -> None:
    await ctx.call(
        Motion.Service.MOVE_L,
        MoveLRequest(
            target=PoseTarget(kind="pose", position=position, quaternion=quaternion),
            speed_scale=speed_scale,
        ),
        MoveLResponse,
        robot_id=robot_id,
    )


async def _log_reached_tcp(
    ctx: TaskContext, robot_id: str, *, expected: Vec3, phase: str
) -> None:
    """도달 TCP snapshot 로깅 — 계획 vs 실제 위치 오차. 실패 시 "arm 이 목표에
    도달했나"를 "기하가 틀렸나"와 분리하는 진단 신호 (침묵 X, 서비스 실패해도
    파지 흐름은 계속 — 로깅은 부수)."""
    try:
        tcp = await ctx.call(
            Motion.Service.TCP_SNAPSHOT, TcpSnapshotRequest(), TcpState,
            robot_id=robot_id,
        )
    except Exception as e:  # 로깅 실패가 파지를 막지 않게
        logger.warning("_log_reached_tcp[%s] TCP snapshot 실패: %s", phase, e)
        return
    a = tcp.position
    dx, dy, dz = a[0] - expected[0], a[1] - expected[1], a[2] - expected[2]
    err_mm = math.sqrt(dx * dx + dy * dy + dz * dz) * 1000.0
    logger.info(
        "reached[%s] robot=%s 계획=(%.3f,%.3f,%.3f) 도달=(%.3f,%.3f,%.3f) "
        "오차=%.1fmm",
        phase, robot_id, expected[0], expected[1], expected[2],
        a[0], a[1], a[2], err_mm,
    )


# ─── 그리퍼 ───────────────────────────────────────────────────────────


@step(title="그리퍼 열기")
async def open_gripper(ctx: TaskContext, robot_id: str) -> None:
    await _set_gripper(ctx, robot_id, open_=True)


@step(title="그리퍼 닫기")
async def close_gripper(ctx: TaskContext, robot_id: str) -> None:
    await _set_gripper(ctx, robot_id, open_=False)


async def _set_gripper(ctx: TaskContext, robot_id: str, *, open_: bool) -> None:
    spec = ctx.spec(robot_id)
    raw = spec.gripper_open_raw if open_ else spec.gripper_close_raw
    logger.info(
        "gripper robot=%s → %s (raw=%d)", robot_id, "OPEN" if open_ else "CLOSE", raw
    )
    await ctx.call(
        Motor.Service.SET_GRIPPER,
        SetGripperRequest(position_raw=raw),
        SetGripperResponse,
        robot_id=robot_id,
    )
    await asyncio.sleep(_GRIPPER_SETTLE_S)


# ─── 파지 판정 (물었나/놓쳤나) ────────────────────────────────────────


# held 판정 부하 하한 — gap 이 작아도 (얇은 물체 / 슬립 후 조 끝 sliver 물림)
# 부하가 물체를 누르고 있으면 물림. 2026-07-17 실측 (so101 STS3215): 빈손 close
# = goal 도달이라 load 56~64 / sliver 물림 load 296 / 정상 물림 300~368 —
# 150 은 빈손×2 마진. ⚠ Feetech raw 기준 — 타 벤더(OMX Dynamixel) 부하 스케일
# 검증 전 (활성 robot 은 so101 뿐).
_HELD_LOAD_MIN_RAW = 150


def _gripper_holding(
    achieved_raw: int, load_raw: int | None, spec  # noqa: ANN001 — TaskRobotSpec
) -> bool:
    """물었나 판정 (벤더 무관). 신호 2개의 OR:

    ① gap = |achieved − close| > held margin (resolve.py, 5% range) — close 명령
      했는데 물체가 막아 완전히 못 닫힘 = 물림.
    ② 부하 ≥ _HELD_LOAD_MIN_RAW — 얇은 물체/슬립 sliver 는 gap 이 margin 아래로
      내려가지만 (2026-07-17 실물: gap 36 인데 실제로 물고 있었음 — 절대 gap
      문턱의 구조적 한계) 물체를 누르는 부하는 남는다. 빈손 close 는 goal 도달로
      부하가 낮아 (56~64) 구분된다.
    """
    margin = abs(spec.gripper_held_threshold_raw - spec.gripper_close_raw)
    gap = abs(achieved_raw - spec.gripper_close_raw)
    if gap > margin:
        return True
    return load_raw is not None and load_raw >= _HELD_LOAD_MIN_RAW


@step(title="파지 확인")
async def verify_grasp(
    ctx: TaskContext, robot_id: str, *, phase: str, grasp_label: str = ""
) -> dict:
    """실제 그리퍼 도달 위치로 물림 판정 — 빈 파지/놓침이면 GraspFailed raise.

    단일 시점·단일 신호의 허점(못 잡았는데 잡음/잡았다 놓침)을 줄이려 servo/place
    가 여러 시점(close 직후·withdraw 후·적치 직전)에서 이걸 부른다. 판정 근거(도달
    raw / close / threshold / load / 계획 폭)를 **전부 로깅** → 실패 시 원인분석 +
    실물 임계값 튜닝 데이터. fail-closed: 물림 확신 못 하면 실패로 기운다."""
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
    held = _gripper_holding(achieved, load, spec)
    logger.info(
        "verify_grasp[%s] robot=%s grip achieved=%d (close=%d open=%d held_thr=%d "
        "load=%s) 계획폭=%s → %s",
        phase, robot_id, achieved, spec.gripper_close_raw, spec.gripper_open_raw,
        spec.gripper_held_threshold_raw, load, grasp_label or "?",
        "HELD" if held else "EMPTY",
    )
    if not held:
        raise GraspFailed(
            phase=phase,
            achieved_raw=achieved,
            close_raw=spec.gripper_close_raw,
            load_raw=load,
        )
    # 성공 판정의 근거도 반환 — trace 에 남겨 임계 튜닝 데이터 (실패만 기록하면
    # "잡았을 때 raw/부하 분포"를 영영 못 본다. 2026-07-17 문턱 오판 진단 교훈).
    return {
        "achieved_raw": achieved,
        "gap_raw": abs(achieved - spec.gripper_close_raw),
        "load_raw": load,
    }


# ─── 포맷/기하 유틸 ───────────────────────────────────────────────────


def _fmt(pos: Vec3) -> str:
    return f"({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f})"


def _fmt_joints(joints: list[float]) -> str:
    return "[" + ",".join(f"{j:.3f}" for j in joints) + "]"


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


def _join_msgs(parts: list[str], sep: str = " / ") -> str:
    """실패 사유 조립 — 명명 헬퍼인 이유: 프리뷰 정적 인덱서가 문자열 리터럴
    `.join` 호출을 `<동적>` 노이즈 행으로 잡는다 (step 트리 오염 방지)."""
    return sep.join(parts)
