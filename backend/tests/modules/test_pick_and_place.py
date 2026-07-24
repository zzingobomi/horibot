"""Pick & Place task 테스트 — closed-loop(servo) 집기 판 (2026-07-16 재설계).

의미 (뒤집으면 회귀): servo 루프가 이동 중 관측 / 관측 없이 명령(맹목) / mask
오검출 tick 을 그대로 파지에 반영 / 관측 소실·수렴 실패가 침묵 진행 or 무한 대기 /
close 후 EMPTY 가 재시도 없이 즉사 or 무한 재시도 / servo 이동 거부가 침묵 통과 /
trace 미기록(실패 재구성 불가) / place 분기가 pick-only 에서 실행 / 놓기 도달
불가가 집은 뒤에 발견 (쥔 채 멈춤 corrupt) / RUN 동시 실행 허용.

servo 순수 계산(가족/gate/decide_tick) 잠금은 test_servo.py.
"""

from __future__ import annotations

import asyncio
import json
from datetime import UTC, datetime
from pathlib import Path

import numpy as np
import pytest
from pydantic import BaseModel

from framework.transport.protocol import RemoteError
from modules.calibration.contract import (
    Calibration,
    CalibrationBundle,
    HandEyeResultData,
    HandEyeResultRecord,
)
from modules.detector.contract import (
    DetectOrientedResponse,
    Detector,
    FuseOrientedResponse,
    OrientedDetection,
)
from modules.motion.contract import (
    Motion,
    MoveJResponse,
    MoveLResponse,
    PlanPathResponse,
    ResolveReachableResponse,
    StopResponse,
    TcpState,
)
from modules.motor.contract import JointState, Motor, SetGripperResponse
from modules.tasks.core.contract import (
    ControlRequest,
    PreviewRequest,
    TaskState,
    TaskStatus,
    ToggleBreakpointRequest,
)
from modules.tasks.core.errors import (
    DetectionNotFound,
    GraspFailed,
    NoReachableGrasp,
    ServoFailed,
    TaskError,
)
from modules.tasks.core.fake import FakeContext
from modules.tasks.core.spec import TaskRobotSpec
from modules.tasks.pick_and_place import geometry, servo, servo_trace, steps
from modules.tasks.pick_and_place.contract import (
    ListRobotsRequest,
    RunRequest,
    TaskMarkers,
)
from modules.tasks.pick_and_place.module import PickAndPlaceModule
from modules.waypoint.contract import (
    GetWaypointByNameResponse,
    ListGroupMembersByNameResponse,
    Waypoint,
    WaypointRecord,
)

_BOT = "so101_6dof_0"

_SPEC = TaskRobotSpec(
    gripper_open_raw=3186,
    gripper_close_raw=1935,
    gripper_index=5,
    gripper_held_threshold_raw=2100,
)

_DETECT = str(Detector.Service.DETECT_ORIENTED)
_FUSE = str(Detector.Service.FUSE_ORIENTED)
_SELECT = str(Motion.Service.RESOLVE_REACHABLE)
_PLAN_PATH = str(Motion.Service.PLAN_PATH)
_MOVE_J = str(Motion.Service.MOVE_J)
_MOVE_L = str(Motion.Service.MOVE_L)
_GRIP = str(Motor.Service.SET_GRIPPER)
_READ_STATE = str(Motor.Service.READ_STATE)
_TCP_SNAP = str(Motion.Service.TCP_SNAPSHOT)
_GET_WP_BY_NAME = str(Waypoint.Service.GET_WAYPOINT_BY_NAME)
_LIST_MEMBERS_BY_NAME = str(Waypoint.Service.LIST_GROUP_MEMBERS_BY_NAME)
_TS = datetime.fromtimestamp(0, UTC)

_HOME_JOINTS = [0.0, 0.5, -1.0, 0.0, 0.5, 1.5]  # 티칭된 home (임의 유효값)

# 테스트용 servo 설정 — 2단 사다리 + settle 0 (결정적·빠름). 실 기본값의
# 상태 전이 자체는 test_servo 가 잠근다. commit 2단 하강(midstop)은 실 기본
# 그대로 켠다 — 시나리오 테스트가 production 경로를 돌아야 한다.
_CFG = servo.ServoConfig(
    standoffs=(0.10, 0.05),
    eps_descend_m=(0.008, 0.004),
    corrections_per_rung=3,
    settle_s=0.0,
    commit_settle_s=0.0,
)


def _home_record() -> WaypointRecord:
    return WaypointRecord(
        id=99, robot_id=_BOT, name="home",
        joint_values=list(_HOME_JOINTS), joint_names=[], created_at=_TS,
    )


def _home_responses() -> dict:
    return {_GET_WP_BY_NAME: [GetWaypointByNameResponse(waypoint=_home_record())] * 2}


_HELD_RAW = 2400  # gap=|2400-1935|=465 > margin |2100-1935|=165 → HELD
_EMPTY_RAW = _SPEC.gripper_close_raw  # close 도달 = 빈 파지


def _joint_state(gripper_raw: int, *, load: int | None = None) -> JointState:
    pos = [0] * 6
    pos[_SPEC.gripper_index] = gripper_raw
    loads = None
    if load is not None:
        loads = [0] * 6
        loads[_SPEC.gripper_index] = load
    return JointState(
        robot_id=_BOT, seq=0, timestamp_unix=0.0,
        positions_raw=pos, velocities_raw=None, loads_raw=loads,
    )


def _tcp(position: tuple[float, float, float]) -> TcpState:
    return TcpState(
        robot_id=_BOT, seq=0, timestamp_unix=0.0, position=position,
        quaternion=(0.0, 0.0, 0.0, 1.0), joint_names=[], joints=[0.0] * 6,
    )


def _resolve_ok(index: int = 0) -> ResolveReachableResponse:
    """가용 응답 — solutions[0] = 첫 standoff(rung0) IK 해 (실행부가 MoveJ 에 씀)."""
    return ResolveReachableResponse(
        index=index, solutions=[[0.1] * 6, [0.2] * 6, [0.3] * 6]
    )


def _pts(n_side: float = 0.011) -> list[tuple[float, float, float]]:
    """관측 점군 — xy 로 ±n_side 스팬 (조 축 폭 측정의 관측 근거), 52점 ≥ min."""
    xs = np.linspace(0.2 - n_side, 0.2 + n_side, 13)
    out = []
    for x in xs:
        for y in (0.05 - n_side, 0.05 + n_side):
            for z in (0.01, 0.02):
                out.append((float(x), float(y), float(z)))
    return out


def _det(
    position: tuple[float, float, float] = (0.2, 0.05, 0.025),
    base_z: float = 0.0,
    height: float = 0.025,
    grasp_yaw: float = 0.0,
    footprint: tuple[float, float] = (0.025, 0.022),
    points: list | None = None,
    score: float = 0.9,
    prompt: str = "white cube",  # 스윕 통합 후 prompt 귀속으로 버킷 분리 — 기본
    # 은 시나리오 pick prompt 와 일치 (detector 가 요청 prompt 를 찍는 계약)
) -> OrientedDetection:
    return OrientedDetection(
        prompt=prompt, position=position, score=score, base_z=base_z,
        height=height, grasp_yaw=grasp_yaw, footprint=footprint,
        points=_pts() if points is None else points,
    )


# ── servo 기대값 (production 함수로 산출 — 배선 검증은 호출 순서/값 대조로) ──

_OBS = _det()
_FAM = servo.grasp_families(_OBS)[0]  # resolve index=0 → 첫 가족 (수직·jaw∥short)
_WIDTH = servo.width_along(_OBS.points, _FAM.jaw_axis, _OBS.footprint[1])
_LAT = servo.lateral_offset(_WIDTH)
_G_POINT = servo.grasp_point(_OBS, _OBS, _CFG)
_G_TCP = servo.grasp_tcp(_G_POINT, _FAM, _LAT, _CFG.engage_m)
_SO0 = servo.standoff(_G_TCP, _FAM, _CFG.standoffs[0])
_SO1 = servo.standoff(_G_TCP, _FAM, _CFG.standoffs[1])
_WITHDRAW = servo.standoff(_G_TCP, _FAM, _CFG.withdraw_standoff_m)
# commit 2단 하강 — midstop 시퀀스 [mid, mid+dither, mid], settle 실측점 = mid
_MIDSTOP = servo.standoff(_G_TCP, _FAM, _CFG.commit_midstop_m)


def _search_responses(n_members: int = 1, sweeps: int = 1) -> dict:
    members = ListGroupMembersByNameResponse(
        found=True,
        waypoints=[
            WaypointRecord(
                id=i + 1, robot_id=_BOT, name=f"s{i}",
                joint_values=[0.0] * 6, joint_names=[], created_at=_TS,
            )
            for i in range(n_members)
        ],
    )
    return {
        **_home_responses(),
        _LIST_MEMBERS_BY_NAME: [members] * (sweeps + 1),
    }


@pytest.fixture(autouse=True)
def _fast_servo(monkeypatch: pytest.MonkeyPatch, tmp_path: Path):
    # 노브는 소유 모듈에 패치 (steps 패키지 분리 2026-07-19) — 소비 코드
    # (plan_pick/servo_pick)가 primitives._SERVO_CFG 모듈 참조로 읽는다.
    monkeypatch.setattr(steps.primitives, "_GRIPPER_SETTLE_S", 0.0)
    monkeypatch.setattr(steps.search, "_SEARCH_SETTLE_S", 0.0)
    monkeypatch.setattr(steps.primitives, "_SERVO_CFG", _CFG)
    # trace 는 실제로 쓴다 (기록 자체가 요구사항) — 저장소만 tmp 로
    monkeypatch.setattr(servo_trace, "_TRACE_ROOT", tmp_path / "servo_pick")


# hand-eye 번들 — approach_observe 의 카메라 look-pose 생성 입력 (실측 마운트:
# 카메라가 TCP 후방 (−77,−9,−65)mm, servo.py 박제). R=I 는 기하 단순화 —
# resolve 는 스크립트라 테스트엔 유효 회전이기만 하면 된다.
def _bundle() -> CalibrationBundle:
    return CalibrationBundle(
        robot_id=_BOT,
        hand_eye=HandEyeResultRecord(
            run_id=1, robot_id=_BOT, created_at=_TS,
            result_data=HandEyeResultData(
                R_cam2gripper=[[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                t_cam2gripper=[[-0.077], [-0.009], [-0.065]],
                method="test",
            ),
        ),
    )


_BUNDLE_KEY = str(Calibration.Service.SNAPSHOT_BUNDLE)


def _plan_direct() -> PlanPathResponse:
    """transit 계획 성공 (직선, 경유점 0) — production 행복 경로 기본값."""
    return PlanPathResponse(found=True, direct=True)


def _ctx(script: dict) -> FakeContext:
    # SNAPSHOT_BUNDLE 은 approach_observe 가 매 호출 당기는 공통 의존 — 기본
    # 스크립트로 넉넉히 깔아준다 (개별 테스트가 넣으면 그쪽 우선).
    # PLAN_PATH(transit 계획, 2026-07-22 home 허브 강등)도 동일 — 기본 = 직선
    # 성공 (폴백 경로 검증 테스트는 명시 override).
    merged = dict(script)
    merged.setdefault(_BUNDLE_KEY, [_bundle()] * 8)
    merged.setdefault(_PLAN_PATH, [_plan_direct()] * 8)
    return FakeContext(robots=[_BOT], specs={_BOT: _SPEC}, service_script=merged)


def _module_for_scenario() -> PickAndPlaceModule:
    class _Rt:
        def publish(self, k: str, e: BaseModel) -> None: ...
        async def call(self, *a, **kw): ...  # noqa: ANN002, ANN003, ANN201

    return PickAndPlaceModule(_Rt(), {})  # type: ignore[arg-type]


def _pick_script(**overrides) -> dict:
    """pick 경로 성공 스크립트 — 스윕 1자세 → 접근·관측(look resolve + 3프레임 +
    융합) → 계획 resolve → servo 2 tick (tick1 rung0 수렴→하강, tick2 rung1
    수렴→commit) → close/withdraw 판정. 접근 관측은 _OBS 를 그대로 돌려줘 하류
    servo 기대값(_FAM/_G_TCP 등)이 불변 (2026-07-21 접근·관측 재구조)."""
    script = {
        **_search_responses(),
        _DETECT: [
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # 스윕
            *_APPROACH_DETECT,  # 접근·관측(1프레임)(coarse)
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # 접근·관측(1프레임)
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # tick1
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # tick2
        ],
        # [0] = 접근 look-pose resolve, [1] = 계획(plan_pick) resolve.
        _SELECT: [_resolve_ok(), _resolve_ok()],
        # tick1 TCP = rung0 standoff (오차 0 → 하강), tick2 = rung1 (→ commit),
        # midstop settle 실측(잔차 0), commit 후 touch-up (잔차 0) + 도달 로깅.
        _TCP_SNAP: [
            _tcp(_SO0), _tcp(_SO1), _tcp(_MIDSTOP), _tcp(_G_TCP), _tcp(_G_TCP),
        ],
        _FUSE: [FuseOrientedResponse(candidates=[_OBS])],  # servo tick2 (관측 2건부터)
        # 스윕 + 접근 look(transit 직선) + rung0(transit 직선) + 종료 home = 4.
        # (2026-07-22 home 허브 강등 — home 경유 MoveJ 는 transit 폴백에만)
        _MOVE_J: [MoveJResponse()] * 4,
        _MOVE_L: [MoveLResponse()] * 6,  # 하강 + midstop×3 + final + withdraw
        _GRIP: [SetGripperResponse()] * 2,  # open + close
        _READ_STATE: [_joint_state(_HELD_RAW)] * 2,  # close/withdraw 판정
    }
    script.update(overrides)
    return script


# ── 접근·관측(2026-07-21)이 스윕과 계획 사이에 넣는 wire ──────────────
# scenario override 시 스윕 _DETECT 뒤에 이 조각을 끼운다 (프레임 1 = _DETECT +1,
# look resolve = _SELECT +1 앞, transit(계획)+look = _MOVE_J +1 앞).
_APPROACH_DETECT = [DetectOrientedResponse(found=True, candidates=[_OBS])]  # 1프레임
# keys() prefix: 스윕 _DETECT 와 계획 _SELECT 사이
# (hand-eye 번들 → look resolve → transit 계획 → look → 관측).
_APPROACH_KEYS = [_BUNDLE_KEY, _SELECT, _PLAN_PATH, _MOVE_J, _DETECT]


# ─── servo 시나리오 (FakeContext — 하드웨어/wire 없음) ────────────────


async def test_scenario_servo_pick_only_sequence():
    mod = _module_for_scenario()
    ctx = _ctx(_pick_script())

    await mod.scenario(ctx, pick_object="white cube")

    assert ctx.keys() == [
        _GET_WP_BY_NAME,  # home 조회 (모션 0)
        _LIST_MEMBERS_BY_NAME, _MOVE_J, _DETECT,  # 스윕 (coarse 찾기)
        *_APPROACH_KEYS,  # 접근·관측: look resolve → transit 계획 → look → 관측
        _SELECT,  # servo 접근 계획 (가족+사다리, 모션 0)
        _PLAN_PATH, _MOVE_J, _GRIP,  # servo 진입: transit 계획 → rung0 → open
        _DETECT, _TCP_SNAP,  # tick1 (관측 1건 — 융합 생략)
        _MOVE_L,  # rung1 하강
        _DETECT, _TCP_SNAP, _FUSE,  # tick2 (관측 2건 융합)
        _MOVE_L, _MOVE_L, _MOVE_L, _TCP_SNAP,  # commit midstop×3 + settle 실측
        _MOVE_L, _TCP_SNAP, _TCP_SNAP,  # final 하강 + touch-up 검증 + 도달 로깅
        _GRIP, _READ_STATE,  # close + 판정 ①
        _MOVE_L, _READ_STATE,  # withdraw + 판정 ②
        _MOVE_J,  # 종료 home
    ]
    # MOVE_J 순서: [0]스윕 [1]접근 look [2]rung0 [3]종료 home (transit 직선 —
    # home 경유 MoveJ 소멸, 2026-07-22). rung0 = resolve 첫 standoff IK 해 그대로.
    assert ctx.calls(_MOVE_J)[2]["req"].target.joints == [0.1] * 6
    assert ctx.calls(_MOVE_J)[3]["req"].target.joints == _HOME_JOINTS
    # servo 이동 목표 = production servo 함수 산출값 (common-mode 상대 명령 배선)
    ml = [c["req"].target for c in ctx.calls(_MOVE_L)]
    assert ml[0].position == pytest.approx(_SO1, abs=1e-9)  # 하강
    # commit 2단: midstop → dither 후방 → midstop (하강방향 재안착) → final
    assert ml[1].position == pytest.approx(_MIDSTOP, abs=1e-9)
    assert ml[2].position[2] == pytest.approx(
        _MIDSTOP[2] + _CFG.commit_dither_m, abs=1e-9
    )
    assert ml[3].position == pytest.approx(_MIDSTOP, abs=1e-9)
    assert ml[4].position == pytest.approx(_G_TCP, abs=1e-9)  # final (잔차 0)
    assert ml[5].position == pytest.approx(_WITHDRAW, abs=1e-9)  # 후퇴
    # 접촉 인접 이동(commit 전 구간/후퇴)은 감속 — withdraw 중 흘림 실사고
    scales = [c["req"].speed_scale for c in ctx.calls(_MOVE_L)]
    assert all(s == _CFG.gentle_speed_scale for s in scales[1:6])
    assert all(m.quaternion == pytest.approx(_FAM.quat, abs=1e-9) for m in ml)
    # 계획 resolve 계약: 사다리+파지 직선(linear) + 그리퍼 벌림 충돌 + 바닥 +
    # home 경로 게이트, 그룹당 pose = standoff 2 + grasp 1.
    sel = ctx.calls(_SELECT)[1]["req"]  # [0]=접근 look resolve, [1]=계획 resolve
    assert sel.linear is True and sel.gripper_open is True
    assert sel.floor_z == pytest.approx(0.0 - 0.005)
    assert sel.path_from == _HOME_JOINTS
    # 장애물 = 이웃 점군만 — **자기 점군 금지** (2026-07-17: engage 설계상
    # grasp 자세의 조↔대상 겹침은 의도된 것 — 자기 점군을 장애물로 검사하면
    # 관측면 쪽 조가 걸려 같은 파지가 뷰에 따라 전멸. steps 주석 참조).
    # 이 시나리오는 이웃 없음(단일 후보) → 빈 리스트가 맞다.
    assert not sel.obstacle_points
    assert all(len(g) == len(_CFG.standoffs) + 1 for g in sel.groups)
    # 든 채 종료 (place 없음 — 마지막 gripper = close)
    grips = [c["req"].position_raw for c in ctx.calls(_GRIP)]
    assert grips == [_SPEC.gripper_open_raw, _SPEC.gripper_close_raw]


async def test_servo_outlier_tick_holds_without_motion():
    """mask 오검출 tick (실데이터 455mm 도약 클래스) = hold — 그 관측이 명령으로
    이어지면 로봇이 허공으로 간다. 기각 tick 과 다음 tick 사이 모션 0 잠금."""
    outlier = _det(position=(0.5, 0.45, 0.02))  # 기대 위치에서 먼 오검출
    ctx = _ctx(_pick_script(**{
        _DETECT: [
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # 스윕
            *_APPROACH_DETECT,  # 접근·관측(1프레임)
            DetectOrientedResponse(found=True, candidates=[outlier]),  # tick1 기각
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # tick2 채택
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # tick3
        ],
        _TCP_SNAP: [
            _tcp(_SO0), _tcp(_SO0), _tcp(_SO1), _tcp(_MIDSTOP), _tcp(_G_TCP),
        ],
        _FUSE: [FuseOrientedResponse(candidates=[_OBS])],
    }))
    await _module_for_scenario().scenario(ctx, pick_object="white cube")

    keys = ctx.keys()
    # tick1(기각) 과 tick2(채택) 사이 = 모션 없음 (DETECT,TCP 다음 바로 DETECT)
    i1 = keys.index(_DETECT, keys.index(_GRIP))  # servo 첫 DETECT
    assert keys[i1 : i1 + 5] == [_DETECT, _TCP_SNAP, _DETECT, _TCP_SNAP, _MOVE_L]
    # 파지는 정상 관측 기준 (오검출이 목표에 안 섞임)
    ml = [c["req"].target.position for c in ctx.calls(_MOVE_L)]
    assert ml[4] == pytest.approx(_G_TCP, abs=1e-9)  # final 하강


async def test_servo_lost_at_start_fails_with_reason_and_trace(tmp_path: Path):
    """servo 진입 후 물체를 한 번도 못 보면 (연속 소실) — 맹목 진행이 아니라
    ServoFailed (사유 포함) + 파지 모션 0 + trace/summary 기록."""
    ctx = _ctx(_pick_script(**{
        _DETECT: [
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # 스윕
            *_APPROACH_DETECT,  # 접근·관측(1프레임)
            DetectOrientedResponse(found=False, candidates=[]),  # tick1 miss
            DetectOrientedResponse(found=False, candidates=[]),  # tick2 → abort
        ],
        _TCP_SNAP: [_tcp(_SO0), _tcp(_SO0)],
    }))
    with pytest.raises(ServoFailed, match="소실"):
        await _module_for_scenario().scenario(ctx, pick_object="white cube")
    assert ctx.calls(_MOVE_L) == []  # 관측 없이 servo 이동/파지 없음
    grips = [c["req"].position_raw for c in ctx.calls(_GRIP)]
    assert grips == [_SPEC.gripper_open_raw]  # open 만 (close 없음)
    # trace 안전망: tick 기록 + 실패 summary 가 남는다 (실패 재구성 요구)
    runs = list((tmp_path / "servo_pick").iterdir())
    assert len(runs) == 1
    assert (runs[0] / "trace.jsonl").read_text(encoding="utf-8").count("\n") == 2
    summary = (runs[0] / "summary.json").read_text(encoding="utf-8")
    assert '"result": "failed"' in summary and "ServoFailed" in summary


async def test_servo_lost_after_convergence_commits_blind():
    """가까이서(rung1) 수렴 후 관측이 죽으면 (그리퍼 가림/근접 한계) — 직전
    관측으로 blind commit (후퇴·포기가 아니라 결단 — handoff §4)."""
    ctx = _ctx(_pick_script(**{
        _DETECT: [
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # 스윕
            *_APPROACH_DETECT,  # 접근·관측(1프레임)
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # tick1 → 하강
            DetectOrientedResponse(found=False, candidates=[]),  # tick2 miss(hold)
            DetectOrientedResponse(found=False, candidates=[]),  # tick3 → commit
        ],
        _TCP_SNAP: [
            _tcp(_SO0), _tcp(_SO1), _tcp(_SO1), _tcp(_MIDSTOP), _tcp(_G_TCP),
        ],
        _FUSE: [],  # 융합 없음 (채택 관측 1건뿐)
    }))
    await _module_for_scenario().scenario(ctx, pick_object="white cube")
    ml = [c["req"].target.position for c in ctx.calls(_MOVE_L)]
    assert ml[4] == pytest.approx(_G_TCP, abs=1e-9)  # 직전 관측 기준 commit


async def test_servo_empty_close_retries_from_standoff_then_succeeds():
    """close 후 EMPTY = 물체가 밀렸을 수 있다 — 놓고 rung1 로 물러나 재관측부터
    재시도 (상한 1회). 옛 open-loop 은 여기서 그냥 실패였다."""
    ctx = _ctx(_pick_script(**{
        _DETECT: [
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # 스윕
            *_APPROACH_DETECT,  # 접근·관측(1프레임)
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # tick1
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # tick2 → commit
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # 재시도 tick
        ],
        _TCP_SNAP: [
            _tcp(_SO0), _tcp(_SO1),  # tick1/2
            _tcp(_MIDSTOP), _tcp(_G_TCP), _tcp(_G_TCP),  # commit① settle+touchup+도달
            _tcp(_SO1),  # 재시도 tick
            _tcp(_MIDSTOP), _tcp(_G_TCP), _tcp(_G_TCP),  # commit② settle+touchup+도달
        ],
        _FUSE: [FuseOrientedResponse(candidates=[_OBS])],
        # 하강 + (midstop×3+final) + 후퇴 + (midstop×3+final) + withdraw
        _MOVE_L: [MoveLResponse()] * 11,
        _GRIP: [SetGripperResponse()] * 4,  # open,close, open(재시도),close
        _READ_STATE: [
            _joint_state(_EMPTY_RAW),  # close① = 빈 파지
            _joint_state(_HELD_RAW),  # close② = 물림
            _joint_state(_HELD_RAW),  # withdraw 판정
        ],
    }))
    await _module_for_scenario().scenario(ctx, pick_object="white cube")

    grips = [c["req"].position_raw for c in ctx.calls(_GRIP)]
    assert grips == [
        _SPEC.gripper_open_raw, _SPEC.gripper_close_raw,
        _SPEC.gripper_open_raw, _SPEC.gripper_close_raw,
    ]
    ml = [c["req"].target.position for c in ctx.calls(_MOVE_L)]
    # 실패 후 후퇴 = rung1 standoff (재관측 자리), 이후 재 commit
    # (하강0, midstop1-3, final4, 후퇴5, midstop6-8, final9, withdraw10)
    assert ml[5] == pytest.approx(_SO1, abs=1e-9)
    assert ml[9] == pytest.approx(_G_TCP, abs=1e-9)


async def test_servo_empty_close_exhausted_raises():
    """재시도 상한까지 EMPTY → GraspFailed (무한 재시도 금지 — handoff §2 표)."""
    ctx = _ctx(_pick_script(**{
        # 스윕 + 접근 + 두 attempt tick (접근·관측 1프레임 추가로 *4→*5)
        _DETECT: [DetectOrientedResponse(found=True, candidates=[_OBS])] * 5,
        _TCP_SNAP: [
            _tcp(_SO0), _tcp(_SO1),
            _tcp(_MIDSTOP), _tcp(_G_TCP), _tcp(_G_TCP),  # commit① settle+touchup+도달
            _tcp(_SO1),
            _tcp(_MIDSTOP), _tcp(_G_TCP), _tcp(_G_TCP),  # commit② settle+touchup+도달
        ],
        _FUSE: [FuseOrientedResponse(candidates=[_OBS])],
        _MOVE_L: [MoveLResponse()] * 10,  # 하강+(midstop×3+final)+후퇴+(midstop×3+final)
        _GRIP: [SetGripperResponse()] * 4,
        _READ_STATE: [_joint_state(_EMPTY_RAW)] * 2,  # 두 attempt 모두 빈 파지
    }))
    with pytest.raises(GraspFailed) as ei:
        await _module_for_scenario().scenario(ctx, pick_object="white cube")
    assert ei.value.phase == "close 직후"
    assert len(ctx.calls(_READ_STATE)) == 2  # attempt 2회에서 끝 (withdraw 판정 X)


async def test_servo_adaptive_ladder_empty_close_retry_survives():
    """적응 1단 진입 사다리(기본 전멸 → _ENTRY_LADDERS 폴백) + close EMPTY
    재시도 = 사다리 길이 무관하게 재관측부터 계속 (pnp_scenario_rework §8.6).

    옛 코드는 재시도 rung 을 `ServoState(rung=1)` 로 하드코딩 — 1단 사다리
    (standoffs=(0.05,)) 에서 다음 tick 의 standoffs[1] 이 IndexError = 의미
    있는 재시도 대신 엉뚱한 실패로 기록 (재시도 메커니즘 무력화).
    _retreat_for_retry 의 [1]→[-1] 수정(07-21)과 같은 클래스의 형제 자리."""
    ctx = _ctx(_pick_script(**{
        _DETECT: [
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # 스윕
            *_APPROACH_DETECT,  # 접근·관측(1프레임)
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # tick1 → commit
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # 재시도 tick
        ],
        _SELECT: [
            _resolve_ok(),  # 접근 look-pose
            ResolveReachableResponse(index=-1, message="기본 사다리 전멸"),
            _resolve_ok(),  # 진입 (0.05,) 채택 → ServoPlan.standoffs 1단
        ],
        _TCP_SNAP: [
            _tcp(_SO1),  # tick1 = rung0(5cm) 정위치 → 즉시 commit
            _tcp(_MIDSTOP), _tcp(_G_TCP), _tcp(_G_TCP),  # commit① settle+touchup+도달
            _tcp(_SO1),  # 재시도 tick (rung = 사다리 마지막 = 0)
            _tcp(_MIDSTOP), _tcp(_G_TCP), _tcp(_G_TCP),  # commit② settle+touchup+도달
        ],
        # (midstop×3+final) + 후퇴 + (midstop×3+final) + withdraw
        _MOVE_L: [MoveLResponse()] * 10,
        _GRIP: [SetGripperResponse()] * 4,  # open,close, open(재시도),close
        _READ_STATE: [
            _joint_state(_EMPTY_RAW),  # close① = 빈 파지 → 재시도
            _joint_state(_HELD_RAW),  # close② = 물림
            _joint_state(_HELD_RAW),  # withdraw 판정
        ],
    }))
    await _module_for_scenario().scenario(ctx, pick_object="white cube")

    # 계획 = 3 resolve (look + 기본 전멸 + 1단 채택), 채택 그룹 pose = 1+1
    sel = ctx.calls(_SELECT)[2]["req"]
    assert all(len(g) == 2 for g in sel.groups)
    grips = [c["req"].position_raw for c in ctx.calls(_GRIP)]
    assert grips == [
        _SPEC.gripper_open_raw, _SPEC.gripper_close_raw,
        _SPEC.gripper_open_raw, _SPEC.gripper_close_raw,
    ]
    ml = [c["req"].target.position for c in ctx.calls(_MOVE_L)]
    # (midstop0-2, final3, 후퇴4, midstop5-7, final8, withdraw9)
    assert ml[4] == pytest.approx(_SO1, abs=1e-9)  # 후퇴 = standoffs[-1]
    assert ml[8] == pytest.approx(_G_TCP, abs=1e-9)  # 재 commit
    assert ml[9] == pytest.approx(_WITHDRAW, abs=1e-9)


async def test_servo_move_rejected_falls_back_to_movej():
    """servo 이동 MoveL 거부 (경로 IK) → MoveJ 폴백으로 계속 — 거부가 침묵
    통과("명령은 항상 실행된다" 가정)하면 회귀."""
    obs2 = _det(position=(0.2, 0.062, 0.025))  # 12mm 이탈 → correct 유발
    ctx = _ctx(_pick_script(**{
        _DETECT: [
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # 스윕
            *_APPROACH_DETECT,  # 접근·관측(1프레임)
            DetectOrientedResponse(found=True, candidates=[obs2]),  # tick1 correct
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # tick2 하강
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # tick3 commit
        ],
        _TCP_SNAP: [
            _tcp(_SO0), _tcp(_SO0), _tcp(_SO1), _tcp(_MIDSTOP), _tcp(_G_TCP),
        ],
        _FUSE: [FuseOrientedResponse(candidates=[_OBS])] * 2,
        _MOVE_L: [
            RemoteError("MotionRejected", "경로 IK 실패"),  # correct 이동 거부
            MoveLResponse(),  # 하강
            MoveLResponse(), MoveLResponse(), MoveLResponse(),  # commit midstop×3
            MoveLResponse(),  # final
            MoveLResponse(),  # withdraw
        ],
        # 스윕 + 접근 look + rung0 + 폴백 + 종료 home
        _MOVE_J: [MoveJResponse()] * 5,
    }))
    await _module_for_scenario().scenario(ctx, pick_object="white cube")
    # 폴백 MoveJ = pose 타깃 (거부된 correct 목표 그대로).
    # MOVE_J: [0]스윕 [1]접근 look [2]rung0 [3]폴백 [4]종료 (transit 직선).
    fallback = ctx.calls(_MOVE_J)[3]["req"].target
    assert fallback.kind == "pose"


async def test_servo_move_both_rejected_replans_then_aborts_if_exhausted():
    """이동 거부 1회 = 재관측으로 계속 (오염 관측 가능성), 연속 2회 = 관측이
    진실 → **가족 재-resolve** (2026-07-17 저녁 실물: 헛집음이 큐브를 경계로
    밀면 재유도 가족이 그 자리서 IK 불가 — 불신 재생산은 사망). 재플랜도
    전멸이면 그때 명시 실패."""
    obs2 = _det(position=(0.2, 0.062, 0.025))  # 12mm 이탈 → 2번째 correct 유발
    ctx = _ctx(_pick_script(**{
        _DETECT: [
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # 스윕
            *_APPROACH_DETECT,  # 접근·관측(1프레임)
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # tick1
            DetectOrientedResponse(found=True, candidates=[obs2]),  # tick2
        ],
        _TCP_SNAP: [_tcp(_SO0), _tcp(_SO0)],
        _SELECT: [
            _resolve_ok(),  # [0] 접근 look-pose resolve
            _resolve_ok(),  # [1] plan_pick 채택
            # [2] 재플랜 (단일 resolve — §11 절대 yaw 격자) 전멸
            ResolveReachableResponse(index=-1, message="전멸"),
        ],
        _MOVE_L: [
            RemoteError("MotionRejected", "경로 IK 실패"),  # tick1 이동 거부①
            RemoteError("MotionRejected", "경로 IK 실패"),  # tick2 이동 거부②
        ],
        _MOVE_J: [
            # 스윕 + 접근 look + rung0 (transit 직선)
            MoveJResponse(), MoveJResponse(), MoveJResponse(),
            RemoteError("MotionRejected", "IK 실패"),  # 폴백 거부①
            RemoteError("MotionRejected", "IK 실패"),  # 폴백 거부②
        ],
    }))
    with pytest.raises(ServoFailed, match="이동 실패"):
        await _module_for_scenario().scenario(ctx, pick_object="white cube")
    assert len(ctx.calls(_SELECT)) == 3  # 접근 look + 계획 + 재플랜(전멸)
    assert ctx.calls(_GRIP)[-1]["req"].position_raw == _SPEC.gripper_open_raw
    assert ctx.calls(_READ_STATE) == []  # 파지 시도 없음


async def test_servo_move_rejected_twice_replans_family_and_continues():
    """2연속 이동 거부 → 가족 재-resolve 성공 → rung0 재진입 → 정상 수렴 →
    파지 성공까지 완주 (2026-07-17 저녁 실물 사망 시나리오의 생존 경로)."""
    # tick2 는 12mm 이탈 관측 — tick1 의 (실패한) descend 가 rung 을 이미
    # 전진시키므로, 수렴 관측이면 commit 으로 빠져 2번째 거부가 _servo_move
    # 를 안 탄다. correct 를 유도해 거부②가 같은 경로로 나게 한다.
    obs2 = _det(position=(0.2, 0.062, 0.025))
    ctx = _ctx(_pick_script(**{
        _DETECT: [
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # 스윕
            *_APPROACH_DETECT,  # 접근·관측(1프레임)
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # tick1 거부①
            DetectOrientedResponse(found=True, candidates=[obs2]),  # tick2 거부②
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # tick3 (재진입)
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # tick4 commit
        ],
        _SELECT: [_resolve_ok(), _resolve_ok(), _resolve_ok()],  # 접근 look+plan+재플랜
        _TCP_SNAP: [
            _tcp(_SO0), _tcp(_SO0),  # tick1/2 (거부 국면)
            _tcp(_SO0), _tcp(_SO1),  # tick3 rung0 수렴 → tick4 rung1 수렴
            _tcp(_MIDSTOP),  # commit settle 실측
            _tcp(_G_TCP), _tcp(_G_TCP),  # touch-up + 도달 로깅
        ],
        _FUSE: [FuseOrientedResponse(candidates=[_OBS])] * 2,  # tick3/4
        _MOVE_J: [
            # 스윕 + 접근 look + rung0 (transit 직선)
            MoveJResponse(), MoveJResponse(), MoveJResponse(),
            RemoteError("MotionRejected", "IK 실패"),  # 폴백 거부①
            RemoteError("MotionRejected", "IK 실패"),  # 폴백 거부②
            MoveJResponse(),  # 재플랜 rung0 재진입
            MoveJResponse(),  # 종료 home
        ],
        _MOVE_L: [
            RemoteError("MotionRejected", "경로 IK 실패"),  # 거부①
            RemoteError("MotionRejected", "경로 IK 실패"),  # 거부②
            MoveLResponse(),  # tick3 하강
            MoveLResponse(), MoveLResponse(), MoveLResponse(),  # commit midstop×3
            MoveLResponse(),  # final
            MoveLResponse(),  # withdraw
        ],
        _GRIP: [SetGripperResponse()] * 2,
        _READ_STATE: [_joint_state(_HELD_RAW)] * 2,
    }))
    await _module_for_scenario().scenario(ctx, pick_object="white cube")
    assert len(ctx.calls(_SELECT)) == 3  # 접근 look + 계획 + 재플랜
    # 재플랜 재진입 = resolve 가 반환한 rung0 관절해 그대로 (재계산 금지).
    # MOVE_J: [0]스윕 [1]look [2]rung0 [3]폴백① [4]폴백② [5]재진입 [6]종료.
    reentry = ctx.calls(_MOVE_J)[5]["req"].target
    assert reentry.kind == "joint" and reentry.joints == [0.1] * 6
    # 파지까지 완주 (close + withdraw 판정 2회)
    assert len(ctx.calls(_READ_STATE)) == 2


async def test_withdraw_rejected_falls_back_to_rung0_joints_while_holding():
    """**쥔 이후의 이동 실패는 task 를 죽일 수 없다** (2026-07-17 저녁 실물:
    HELD·부하 320 직후 withdraw 사전검증 거부 → 쥔 채 사망). 집은 자세는
    유효한 관절 구성 — 관절 공간(IK 0)으로 항상 탈출 가능하다. 폴백 = 계획이
    증명한 rung0 관절해 MoveJ, 이후 슬립 판정·이송 계속."""
    ctx = _ctx(_pick_script(**{
        _MOVE_L: [
            MoveLResponse(),  # 하강
            MoveLResponse(), MoveLResponse(), MoveLResponse(),  # commit midstop×3
            MoveLResponse(),  # final
            RemoteError("MotionRejected", "경로 IK 실패"),  # withdraw 거부
        ],
        # 스윕 + 접근 look + rung0 + **폴백** + 종료 home (transit 직선)
        _MOVE_J: [MoveJResponse()] * 5,
    }))
    await _module_for_scenario().scenario(ctx, pick_object="white cube")
    # MOVE_J: [0]스윕 [1]look [2]rung0 [3]폴백 [4]종료.
    fallback = ctx.calls(_MOVE_J)[3]["req"].target
    assert fallback.kind == "joint" and fallback.joints == [0.1] * 6
    assert len(ctx.calls(_READ_STATE)) == 2  # withdraw-후 슬립 판정까지 계속


# ── transit (계획 이동 — home 허브 강등, 2026-07-22) ─────────────────────


async def test_transit_plan_failure_falls_back_to_home_route():
    """★ 계획 실패(found=False) = home 경유 폴백 — 옛 실행 계약 그대로라 동작
    후퇴가 없다 (플래너는 최적화지 의존성이 아님). 뒤집으면 회귀: 폴백이 없으면
    플래너 전멸이 태스크 사망, 폴백이 침묵이면 로그 없는 성능 저하."""
    ctx = _ctx(_pick_script(**{
        _PLAN_PATH: [PlanPathResponse(found=False, message="경로 없음")] * 2,
        # 폴백 = home 경유 ×2 (접근 + servo 진입): 스윕1 + (home+look) +
        # (home+rung0) + 종료 = 6
        _MOVE_J: [MoveJResponse()] * 6,
    }))
    await _module_for_scenario().scenario(ctx, pick_object="white cube")
    mj = [c["req"].target.joints for c in ctx.calls(_MOVE_J)]
    # [1]=home(접근 폴백) [2]=look, [3]=home(진입 폴백) [4]=rung0
    assert mj[1] == _HOME_JOINTS and mj[3] == _HOME_JOINTS
    assert mj[2] == [0.1] * 6 and mj[4] == [0.1] * 6


async def test_transit_wire_error_falls_back_to_home_route():
    """PLAN_PATH wire 예외(RemoteError/구버전 motion)도 폴백 — 계획 서비스
    장애가 태스크를 못 죽인다."""
    ctx = _ctx(_pick_script(**{
        _PLAN_PATH: [RemoteError("ServiceUnavailable", "plan_path 없음")] * 2,
        _MOVE_J: [MoveJResponse()] * 6,
    }))
    await _module_for_scenario().scenario(ctx, pick_object="white cube")
    mj = [c["req"].target.joints for c in ctx.calls(_MOVE_J)]
    assert mj[1] == _HOME_JOINTS and mj[3] == _HOME_JOINTS


async def test_transit_executes_planned_waypoints_in_order():
    """계획이 중간 경유점을 반환하면 그 순서 그대로 MoveJ — 판정 경로 == 실행
    경로 (경유점을 건너뛰면 계획이 검증 안 한 직선을 실행하는 회귀)."""
    via = [0.05] * 6
    ctx = _ctx(_pick_script(**{
        _PLAN_PATH: [
            PlanPathResponse(found=True, waypoints=[via]),  # 접근 look transit
            _plan_direct(),  # servo 진입
        ],
        _MOVE_J: [MoveJResponse()] * 5,  # 스윕 + (via+look) + rung0 + 종료
    }))
    await _module_for_scenario().scenario(ctx, pick_object="white cube")
    mj = [c["req"].target.joints for c in ctx.calls(_MOVE_J)]
    assert mj[1] == via and mj[2] == [0.1] * 6  # 경유 → look 순서


async def test_pick_entry_transit_carries_plan_collision_model():
    """servo 진입 transit 요청 = plan_pick 게이트와 같은 충돌 모델 (바닥 +
    이웃 점군 + 조 벌림) — 모델이 빠지면 게이트는 통과했는데 실행 경로가
    검사 없이 나가는 침묵 구멍."""
    ctx = _ctx(_pick_script())
    await _module_for_scenario().scenario(ctx, pick_object="white cube")
    plans = ctx.calls(_PLAN_PATH)
    assert len(plans) == 2  # 접근 look + servo 진입
    entry = plans[1]["req"]
    assert entry.goal_joints == [0.1] * 6  # rung0 (resolve 해 그대로)
    assert entry.floor_z == pytest.approx(0.0 - 0.005)  # plan.floor_z
    assert entry.gripper_open is True
    assert entry.obstacle_points is None  # 단일 후보 — 이웃 없음
    # 접근 look transit 도 바닥 게이트 동봉 (검출 최저 base_z − 여유)
    look = plans[0]["req"]
    assert look.floor_z == pytest.approx(0.0 - 0.005)
    assert look.gripper_open is False


async def test_place_carry_transit_sets_held_clearance():
    """★ 운반 transit (쥔 채 servo 종료 → 적치 접근) — home 왕복 소멸 +
    tcp_min_z = 바닥 + 물체 높이 + 여유 (매달린 물체 바닥 여유의 보수 근사).
    뒤집으면 회귀: end_home 이 남으면 쥔 채 최장 스윙 왕복 부활, tcp_min_z
    가 빠지면 낮은 운반 경로가 물체를 테이블에 긁는다."""
    place_spot = _det(
        position=(0.25, -0.05, 0.04), height=0.04, grasp_yaw=0.3,
        prompt="red box",
    )
    ctx = _ctx(_pick_script(**{
        **_search_responses(),
        _DETECT: [
            DetectOrientedResponse(found=True, candidates=[_OBS, place_spot]),
            DetectOrientedResponse(found=True, candidates=[place_spot]),
            *_APPROACH_DETECT,
            DetectOrientedResponse(found=True, candidates=[_OBS]),
            DetectOrientedResponse(found=True, candidates=[_OBS]),
        ],
        _SELECT: [_resolve_ok()] * 4,
        _MOVE_J: [MoveJResponse()] * 6,
        _MOVE_L: [MoveLResponse()] * 8,
        _GRIP: [SetGripperResponse()] * 4,
        _READ_STATE: [_joint_state(_HELD_RAW)] * 3,
    }))
    await _module_for_scenario().scenario(
        ctx, pick_object="white cube", place_object="red box"
    )
    plans = ctx.calls(_PLAN_PATH)
    # place look + pick look + servo 진입 + 운반 = 4
    assert len(plans) == 4
    carry = plans[3]["req"]
    assert carry.goal_joints == [0.1] * 6  # pre 관절해 (resolve 그대로)
    assert carry.floor_z == pytest.approx(-0.005)  # pick 바닥 (plan.floor_z)
    # tcp_min_z = floor(-0.005) + 물체 높이(0.025) + 여유(0.02)
    assert carry.tcp_min_z == pytest.approx(-0.005 + 0.025 + 0.02)
    assert carry.gripper_open is False  # 쥔 채 (조 닫힘)
    # 쥔 채 home 왕복 소멸 — 운반 구간(withdraw 판정 후 ~ pre 도착)에 home
    # MoveJ 가 없다. MOVE_J home 은 스윕 앞/종료뿐.
    mj = [c["req"].target.joints for c in ctx.calls(_MOVE_J)]
    assert mj.count(_HOME_JOINTS) == 1  # 종료 1회만


async def test_commit_midstop_release_drops_stale_comp_from_final_command():
    """스틱션 release 회귀 (2026-07-17 42런 잔차 분석 — comp z ±5~13mm 널뜀):
    tick 국면에서 comp 가 +8mm 미달을 학습해도, midstop 실측이 "이미 해소"
    (측정 = 명령) 를 보이면 최종 하강은 g_tcp 그대로 — stale comp 가 최종
    명령에 남으면(과보상) 조 끝이 착지에서 어긋나는 메커니즘을 명령 수준에서
    차단한다."""
    low = 0.008
    so1_low = (_SO1[0], _SO1[1], _SO1[2] - low)  # tick2 실측 8mm 미달 → comp +8
    cmd1 = (_MIDSTOP[0], _MIDSTOP[1], _MIDSTOP[2] + low)  # midstop 명령 (comp 포함)
    ctx = _ctx(_pick_script(**{
        _TCP_SNAP: [
            _tcp(_SO0), _tcp(so1_low),
            _tcp(cmd1),  # settle 실측 = 명령 그대로 (release — 미달 소멸)
            _tcp(_G_TCP), _tcp(_G_TCP),
        ],
    }))
    await _module_for_scenario().scenario(ctx, pick_object="white cube")
    moves = [c["req"].target.position for c in ctx.calls(_MOVE_L)]
    # midstop 이동엔 comp(+8mm) 적용 (측정 전까지는 기존 보상 유지)
    assert moves[1][2] == pytest.approx(_MIDSTOP[2] + low, abs=1e-9)
    # 최종 하강 = 실측 재앵커 — stale comp 제거 (g_tcp 그대로)
    assert moves[4] == pytest.approx(_G_TCP, abs=1e-9)


async def test_commit_midstop_persisting_deficit_keeps_compensation():
    """반대 분기 — midstop 실측이 여전히 8mm 미달이면 최종 하강이 +8mm 보상
    (오늘과 동일 동작). 재앵커가 "보상을 없애는" 게 아니라 "실측으로 갱신하는"
    것임을 잠금 — 이게 뒤집히면 stall 국면에서 파지 z 가 낮아져 nip."""
    low = 0.008
    mid_low = (_MIDSTOP[0], _MIDSTOP[1], _MIDSTOP[2] - low)  # 여전히 미달
    ctx = _ctx(_pick_script(**{
        _TCP_SNAP: [
            _tcp(_SO0), _tcp(_SO1),
            _tcp(mid_low),
            _tcp(_G_TCP), _tcp(_G_TCP),
        ],
    }))
    await _module_for_scenario().scenario(ctx, pick_object="white cube")
    final = ctx.calls(_MOVE_L)[4]["req"].target.position
    assert final[2] == pytest.approx(_G_TCP[2] + low, abs=1e-9)


async def test_commit_midstop_move_rejected_falls_back_to_single_shot(
    tmp_path: Path,
):
    """midstop 경로 실패 = 오늘의 단발 하강으로 폴백 (이 수정으로 IK 사망
    경로가 생기지 않는다는 계약) — 사유는 trace midstop_skipped 로 남는다."""
    ctx = _ctx(_pick_script(**{
        _MOVE_L: [
            MoveLResponse(),  # 하강
            RemoteError("MotionRejected", "경로 IK 실패"),  # midstop 1구간 거부
            MoveLResponse(),  # 폴백 단발 하강
            MoveLResponse(),  # withdraw
        ],
        # settle 실측이 없다 (midstop 실패) — touch-up/도달 로깅만
        _TCP_SNAP: [_tcp(_SO0), _tcp(_SO1), _tcp(_G_TCP), _tcp(_G_TCP)],
    }))
    await _module_for_scenario().scenario(ctx, pick_object="white cube")
    moves = [c["req"].target.position for c in ctx.calls(_MOVE_L)]
    assert len(moves) == 4
    assert moves[2] == pytest.approx(_G_TCP, abs=1e-9)  # 단발 = comp.apply(g_tcp)
    rows = [
        json.loads(line)
        for p in (tmp_path / "servo_pick").glob("*/trace.jsonl")
        for line in p.read_text(encoding="utf-8").splitlines()
    ]
    assert any(r.get("action") == "midstop_skipped" for r in rows)


async def test_commit_descent_profile_records_samples_and_suspect(
    tmp_path: Path,
):
    """하강 프로파일 관측성 — 시간이 걸리는 실 이동 중 FK z/load 샘플이 trace
    에 남고, arm load 스파이크가 floor_contact_suspect 로 summary 에 표면화.
    (실패 시 "닿았는지/언제/얼마나"를 데이터가 답하게 하는 요구의 잠금.)"""
    held_arm_load = JointState(
        robot_id=_BOT, seq=0, timestamp_unix=0.0,
        positions_raw=[0, 0, 0, 0, 0, _HELD_RAW],
        velocities_raw=None,
        loads_raw=[0, 300, 0, 0, 0, 0],  # joint2 스파이크 (>150) — 접촉 의심
    )

    class _SlowFinalCtx(FakeContext):
        async def call(self, key, req, res_cls, *, robot_id=None, timeout=None):
            res = await super().call(
                key, req, res_cls, robot_id=robot_id, timeout=timeout
            )
            target = getattr(req, "target", None)
            if (
                str(key) == _MOVE_L
                and target is not None
                and abs(target.position[2] - _G_TCP[2]) < 1e-9
            ):
                await asyncio.sleep(0.06)  # 최종 하강만 실 소요 — 샘플링 창
            return res

    script = _pick_script(**{
        _TCP_SNAP: [_tcp(_SO0), _tcp(_SO1), _tcp(_MIDSTOP)]
        + [_tcp(_G_TCP)] * 10,  # 샘플/touch-up/도달 — 전부 동일값 (결정성)
        _READ_STATE: [held_arm_load] * 10,  # 샘플 + close/withdraw 판정
    })
    script.setdefault(_BUNDLE_KEY, [_bundle()] * 8)  # approach hand-eye (직접 생성 ctx)
    script.setdefault(_PLAN_PATH, [_plan_direct()] * 8)  # transit 계획 (직접 생성 ctx)
    ctx = _SlowFinalCtx(robots=[_BOT], specs={_BOT: _SPEC}, service_script=script)
    await _module_for_scenario().scenario(ctx, pick_object="white cube")

    rows = [
        json.loads(line)
        for p in (tmp_path / "servo_pick").glob("*/trace.jsonl")
        for line in p.read_text(encoding="utf-8").splitlines()
    ]
    prof = [r for r in rows if r.get("action") == "descent_profile"]
    assert prof and prof[-1]["samples"], "하강 프로파일 샘플이 안 남음"
    assert prof[-1]["floor_contact_suspect"] is True
    summaries = list((tmp_path / "servo_pick").glob("*/summary.json"))
    assert summaries
    summ = json.loads(summaries[-1].read_text(encoding="utf-8"))
    assert summ["floor_contact_suspect"] is True


async def test_servo_nonconvergence_aborts_with_history():
    """오차가 안 줄면 (발진/정체) 보정 상한 후 명시 중단 — 사유에 오차 이력."""
    cfg = servo.ServoConfig(
        standoffs=(0.10, 0.05), eps_descend_m=(0.008, 0.004),
        corrections_per_rung=1, settle_s=0.0,
    )
    far = _det(position=(0.2, 0.05 + 0.03, 0.025))  # lateral 30mm > capture 12mm
    ctx = _ctx(_pick_script(**{
        _DETECT: [
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # 스윕
            *_APPROACH_DETECT,  # 접근·관측(1프레임)
            DetectOrientedResponse(found=True, candidates=[far]),  # tick1 correct
            DetectOrientedResponse(found=True, candidates=[far]),  # tick2 → abort
        ],
        # TCP 가 목표를 안 따라감 (오차 유지) — 발진/정체 재현
        _TCP_SNAP: [_tcp(_SO0), _tcp(_SO0)],
        _FUSE: [FuseOrientedResponse(candidates=[far])],
        _MOVE_L: [MoveLResponse()],  # correct 1회
    }))
    steps.primitives._SERVO_CFG = cfg  # 이 테스트만 보정 상한 1
    try:
        with pytest.raises(ServoFailed, match="수렴 실패"):
            await _module_for_scenario().scenario(ctx, pick_object="white cube")
    finally:
        steps.primitives._SERVO_CFG = _CFG


async def test_scenario_with_place_branch_places_after_servo():
    # 스윕 통합 (2026-07-19): pick+place 가 **한 스윕** — pose 당 DETECT 1회가
    # 두 prompt 후보를 함께 반환, 귀속은 per-candidate prompt.
    place_spot = _det(
        position=(0.25, -0.05, 0.04), height=0.04, grasp_yaw=0.3,
        prompt="red box",
    )
    ctx = _ctx(_pick_script(**{
        **_search_responses(),
        _DETECT: [
            DetectOrientedResponse(
                found=True, candidates=[_OBS, place_spot]
            ),  # 통합 스윕 (pick+place)
            DetectOrientedResponse(found=True, candidates=[place_spot]),  # 접근·관측(place, 먼저)
            *_APPROACH_DETECT,  # 접근·관측(pick, 마지막)
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # tick1
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # tick2
        ],
        # [0]pick look + [1]pick 계획 + [2]place look + [3]place 정렬 가족
        _SELECT: [_resolve_ok(), _resolve_ok(), _resolve_ok(), _resolve_ok()],
        # 스윕 + place look + pick look + rung0 + 운반 pre + 종료 home = 6
        # (transit 직선 — servo 종료 home 은 end_home=False 로 소멸, 운반이 대체)
        _MOVE_J: [MoveJResponse()] * 6,
        _MOVE_L: [MoveLResponse()] * 8,  # servo 6 (midstop 포함) + insert + retreat
        _GRIP: [SetGripperResponse()] * 4,  # open/close/release/마무리 close
        _READ_STATE: [_joint_state(_HELD_RAW)] * 3,  # close/withdraw/적치 직전
    }))
    await _module_for_scenario().scenario(
        ctx, pick_object="white cube", place_object="red box"
    )
    grips = [c["req"].position_raw for c in ctx.calls(_GRIP)]
    assert grips == [
        _SPEC.gripper_open_raw, _SPEC.gripper_close_raw,
        _SPEC.gripper_open_raw,
        _SPEC.gripper_close_raw,  # 종료 정리 자세 (2026-07-17 사용자 요청)
    ]
    # 스윕 통합 불변식: 검출 wire 호출 = 스윕 pose 1 + servo tick 2 (place 전용
    # 재스윕 없음), 스윕 요청엔 두 prompt 가 함께 실린다.
    assert len(ctx.calls(_DETECT)) == 5  # 스윕 + 접근(pick+place) + servo tick 2
    sweep_req = ctx.calls(_DETECT)[0]["req"]
    assert sweep_req.prompts == ["white cube", "red box"]
    # 계획 우선 불변식: 놓기 도달성 판정까지 끝난 뒤에야 servo 진입(GRIP/MOVE_L).
    # RESOLVE = pick look + pick 계획 + place look + place 계획 = 4 (전부 servo 앞).
    keys = ctx.keys()
    assert keys[: keys.index(_GRIP)].count(_SELECT) == 4
    assert keys[: keys.index(_MOVE_L)].count(_SELECT) == 4


async def test_place_retreat_movel_failure_falls_back_to_pre_joints():
    """2026-07-17 실물 회귀 — 적치·release 까지 성공한 뒤 retreat MoveL 이 실행
    중 IK 실패로 죽어 task 전체가 실패 처리됐다 (사전 검증은 통과 — 실행 seed
    연쇄 복권). pre 는 resolve 가 관절해까지 증명한 자세 — MoveL 실패 시 그
    관절해(pre_joints) MoveJ 폴백으로 run 을 살린다."""
    place_spot = _det(
        position=(0.25, -0.05, 0.04), height=0.04, grasp_yaw=0.3,
        prompt="red box",
    )
    ctx = _ctx(_pick_script(**{
        **_search_responses(),
        _DETECT: [
            DetectOrientedResponse(
                found=True, candidates=[_OBS, place_spot]
            ),  # 통합 스윕
            DetectOrientedResponse(found=True, candidates=[place_spot]),  # 접근·관측(place, 먼저)
            *_APPROACH_DETECT,  # 접근·관측(pick, 마지막)
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # tick1
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # tick2
        ],
        _SELECT: [_resolve_ok()] * 4,  # pick look+계획 + place look+계획
        # 스윕 + look×2 + rung0 + 운반 pre + retreat 폴백 + 종료 home = 7
        _MOVE_J: [MoveJResponse()] * 7,
        _MOVE_L: [MoveLResponse()] * 7 + [  # servo 6 (midstop 포함) + insert
            RemoteError("MotionFailed", "MoveL failed"),  # retreat 실행 실패
        ],
        _GRIP: [SetGripperResponse()] * 4,
        _READ_STATE: [_joint_state(_HELD_RAW)] * 3,
    }))
    await _module_for_scenario().scenario(
        ctx, pick_object="white cube", place_object="red box"
    )  # raise 없음 = 적치 성공 run 생존
    # 폴백 = 마지막에서 두 번째 MoveJ (마지막은 go_home), 타깃 = 계획 관절해
    fallback = ctx.calls(_MOVE_J)[-2]["req"].target
    assert fallback.kind == "joint"
    assert list(fallback.joints) == [0.1] * 6  # _resolve_ok solutions[0]
    grips = [c["req"].position_raw for c in ctx.calls(_GRIP)]
    assert grips[-1] == _SPEC.gripper_close_raw  # 종료 정리까지 완주


async def test_scenario_place_unreachable_fails_before_pick():
    """놓을 곳 도달 불가 → 집기 **전에** 실패 (쥔 채 멈춤 corrupt 방지) —
    servo 모션·파지 0. 재배열(2026-07-21): 놓기 계획이 물건 관측·집기 계획보다
    먼저라, place 전멸이면 물건 접근·계획도 안 탄다 (더 일찍 실패)."""
    place_spot = _det(position=(0.15, 0.10, 0.22), base_z=0.20, prompt="red box")
    ctx = _ctx({
        **_search_responses(),
        _DETECT: [
            DetectOrientedResponse(
                found=True, candidates=[_OBS, place_spot]
            ),  # 통합 스윕
            DetectOrientedResponse(found=True, candidates=[place_spot]),  # 접근·관측(place)
        ],
        _SELECT: [
            _resolve_ok(),  # place 접근 look-pose
            ResolveReachableResponse(index=-1, message="정렬 전멸"),
            ResolveReachableResponse(index=-1, message="자유 전멸"),
        ],
        _MOVE_J: [MoveJResponse()] * 3,  # 스윕 + place 접근(home+look)
    })
    with pytest.raises(NoReachableGrasp, match="놓을 자리 도달 불가"):
        await _module_for_scenario().scenario(
            ctx, pick_object="white cube", place_object="red box"
        )
    assert ctx.calls(_GRIP) == []
    assert ctx.calls(_MOVE_L) == []


async def test_plan_pick_family_exhausted_fails_explicitly():
    """servo 접근 가족 전멸(-1) = 데이터 → step 이 치명 판정 (침묵 통과 금지),
    모션은 스윕뿐. 진입 사다리 4라운드(기본+_ENTRY_LADDERS)까지 전부 전멸해야
    실패 확정 (2026-07-21 적응 진입 — 68가족 매장 감사)."""
    ctx = _ctx({
        **_search_responses(),
        _DETECT: [
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # 스윕
            *_APPROACH_DETECT,  # 접근·관측(1프레임)
        ],
        _SELECT: [
            _resolve_ok(),  # 접근 look-pose
            # 계획 resolve — 진입 사다리 라운드 전멸 (기본 + 5/3/2cm)
            *[ResolveReachableResponse(index=-1, message="전멸")] * 4,
        ],
        _MOVE_J: [MoveJResponse()] * 3,  # 스윕 + 접근(home+look)
    })
    with pytest.raises(NoReachableGrasp, match="후보 1개 전부 전멸"):
        await _module_for_scenario().scenario(ctx, pick_object="white cube")
    # 접근 look 1 + 계획 진입 사다리 4라운드 전멸 확정
    assert len(ctx.calls(_SELECT)) == 5
    assert ctx.calls(_GRIP) == [] and ctx.calls(_MOVE_L) == []


async def test_scenario_detect_fail_raises_after_search():
    ctx = _ctx({
        **_search_responses(),
        _DETECT: [DetectOrientedResponse(found=False, candidates=[])] * 4,
        _MOVE_J: [MoveJResponse()] * 4,
    })
    with pytest.raises(DetectionNotFound):
        await _module_for_scenario().scenario(ctx, pick_object="white cube")
    assert ctx.calls(_GRIP) == []
    assert ctx.calls(_MOVE_L) == []


async def test_search_sweep_accumulates_and_selects_best_score():
    """검색 원리: search 자세를 전부 돌며 누적, select 는 누적 전체 최고 score
    (첫 자세서 안 멈춤). 스윕 통합 후 반환은 prompt 별 dict."""
    ctx = _ctx({
        **_search_responses(n_members=2),
        _DETECT: [
            DetectOrientedResponse(found=True, candidates=[_det(score=0.4)]),
            DetectOrientedResponse(found=True, candidates=[_det(score=0.95)]),
        ],
        _MOVE_J: [MoveJResponse()] * 2,
    })
    found = await steps.detect(ctx, _BOT, ["white cube"])
    cands = found["white cube"]
    assert len(ctx.calls(_MOVE_J)) == 2
    assert [c.score for c in cands] == [0.4, 0.95]
    coarse = geometry.select_target_by_score(cands, prompt="white cube")
    assert coarse.score == 0.95


async def test_search_sweep_buckets_by_prompt_single_pass():
    """★ 스윕 통합 계약 (2026-07-19): pose 당 DETECT wire 1호출에 두 prompt 가
    함께 실리고, 응답 후보는 per-candidate prompt 로 버킷 분리 — place 전용
    재스윕(같은 자세 MoveJ ×2)이 사라졌다. 요청 밖 prompt 귀속은 무시(로그)."""
    cube = _det(score=0.9)
    box = _det(position=(0.25, -0.05, 0.04), prompt="red box", score=0.7)
    alien = _det(position=(0.4, 0.3, 0.02), prompt="green ball", score=0.8)
    ctx = _ctx({
        **_search_responses(n_members=2),
        _DETECT: [
            DetectOrientedResponse(found=True, candidates=[cube, box, alien]),
            DetectOrientedResponse(found=True, candidates=[box]),
        ],
        _MOVE_J: [MoveJResponse()] * 2,
    })
    found = await steps.detect(ctx, _BOT, ["white cube", "red box"])
    assert len(ctx.calls(_DETECT)) == 2  # pose 당 1호출 (prompt 당 아님)
    for c in ctx.calls(_DETECT):
        assert c["req"].prompts == ["white cube", "red box"]
    assert [c.score for c in found["white cube"]] == [0.9]
    assert [c.score for c in found["red box"]] == [0.7, 0.7]  # 자세별 누적
    assert "green ball" not in found  # 요청 밖 귀속은 버킷 미생성 (무시)


async def test_search_group_missing_fails_explicitly():
    ctx = _ctx({
        **_home_responses(),
        _LIST_MEMBERS_BY_NAME: [ListGroupMembersByNameResponse(found=False)],
    })
    with pytest.raises(TaskError, match="search"):
        await _module_for_scenario().scenario(ctx, pick_object="white cube")
    assert ctx.calls(_MOVE_J) == []


async def test_scenario_home_waypoint_missing_fails_before_any_motion():
    ctx = _ctx({_GET_WP_BY_NAME: [GetWaypointByNameResponse(waypoint=None)]})
    with pytest.raises(TaskError, match="home"):
        await _module_for_scenario().scenario(ctx, pick_object="white cube")
    assert ctx.calls(_MOVE_J) == []
    assert ctx.calls(_LIST_MEMBERS_BY_NAME) == []


async def test_servo_success_writes_trace_and_summary(tmp_path: Path):
    """성공 run 도 tick trace + summary — "성공했는데 왜 성공했는지 모른다" 방지
    (실물 임계 튜닝의 데이터 소스)."""
    ctx = _ctx(_pick_script())
    await _module_for_scenario().scenario(ctx, pick_object="white cube")
    runs = list((tmp_path / "servo_pick").iterdir())
    assert len(runs) == 1
    lines = (runs[0] / "trace.jsonl").read_text(encoding="utf-8").splitlines()
    # tick1 + tick2 + commit + midstop_reanchor + close held + withdraw held
    # (성공 근거도 기록 — 실패만 기록하면 "잡았을 때 raw/부하 분포"를 못 본다)
    assert len(lines) == 6
    summary = (runs[0] / "summary.json").read_text(encoding="utf-8")
    assert '"result": "success"' in summary
    assert '"midstop_resid_mm"' in summary  # 재앵커 잔차가 summary 에 표면화


# ─── 놓기 geometry/step (open-loop 유지 — 기존 잠금 계승) ─────────────


def test_plan_place_release_height():
    # 2026-07-21 단순화: 상자 정중앙(spot XY) 위 고정 높이(spot_top + 5mm), 물건
    # 폭/높이 무시. place = spot 중심, pre = 접근축(tilt0=+z) 후방 _APPROACH_CLEAR.
    spot = _det(position=(0.25, -0.05, 0.04), height=0.04, grasp_yaw=0.3)
    pplan = geometry.plan_place(spot)
    assert pplan[0].place[0] == pytest.approx(0.25)  # 상자 정중앙 XY (lateral 없음)
    assert pplan[0].place[1] == pytest.approx(-0.05)
    assert pplan[0].place[2] == pytest.approx(0.04 + 0.005)  # spot_top + drop clear
    assert pplan[0].pre[0] == pytest.approx(pplan[0].place[0])
    assert pplan[0].pre[1] == pytest.approx(pplan[0].place[1])
    assert pplan[0].pre[2] == pytest.approx(pplan[0].place[2] + 0.06)  # +approach clear
    assert len(pplan) == 7 * 4
    assert pplan[0].label == "tilt=+0 yaw=17"
    for deg in (17, 107, 197, 287):
        assert any(f"yaw={deg}" in c.label for c in pplan)


def test_plan_place_free_family_disjoint_yaws():
    spot = _det(position=(0.25, -0.05, 0.04), height=0.04, grasp_yaw=0.3)
    aligned = geometry.plan_place(spot)
    free = geometry.plan_place_free(spot)
    assert len(free) == 7 * 8
    assert free[0].label == "tilt=+0 yaw=47"
    yaw_of = lambda c: c.label.split("yaw=")[1]  # noqa: E731
    assert {yaw_of(c) for c in aligned} & {yaw_of(c) for c in free} == set()


async def test_plan_place_falls_back_to_reachable_spot():
    high_score = _det(score=0.80, position=(0.15, 0.10, 0.03), base_z=0.005)
    low_score = _det(score=0.73, position=(0.24, -0.11, 0.05), base_z=0.005)
    ctx = _ctx({
        _SELECT: [
            ResolveReachableResponse(index=-1, message="IK 전멸"),
            ResolveReachableResponse(index=-1, message="IK 전멸"),
            _resolve_ok(),
        ],
    })
    chosen, _pre = await steps.plan_place(
        ctx, _BOT, "blue box", home=_home_record(),
        spots=[high_score, low_score],
    )
    assert chosen.place[2] == pytest.approx(0.05 + 0.005)  # spot_top + drop clear
    assert len(ctx.calls(_SELECT)) == 3


async def test_plan_place_defers_implausible_base_z_spots():
    """공중 부양(flying-pixel 오염) / 과침하 spot 은 score 가 높아도 후순위 —
    2026-07-17 실물: base_z=+0.156~0.175 오염 spot 이 score 상위로 spot 당
    resolve ~55s 를 먼저 태움 (최악 런은 plan_place 에만 3.5분). 기각 아님 —
    타당 spot 이 먼저 닿으면 resolve 1회로 끝. 하한은 pick(-0.01)이 아니라
    place 전용(-0.04) — 실상자 멀티뷰 바닥은 -0.02 대가 정상 관측이다."""
    floating = _det(score=0.95, position=(0.15, 0.10, 0.22), base_z=0.20)
    sunken = _det(score=0.90, position=(0.20, 0.10, 0.02), base_z=-0.10)
    box = _det(score=0.50, position=(0.24, -0.11, 0.03), base_z=-0.016)
    ctx = _ctx({
        _SELECT: [_resolve_ok()],
    })
    chosen, _pre = await steps.plan_place(
        ctx, _BOT, "blue box",
        home=_home_record(), spots=[floating, sunken, box],
    )
    # 타당 spot(box — base_z=-0.016 은 place 대역 안, score 최하)이 첫 시도
    assert len(ctx.calls(_SELECT)) == 1
    assert chosen.place[2] == pytest.approx(0.03 + 0.005)  # spot_top + drop clear


def test_fuse_place_center_averages_cluster_and_drops_garbage():
    """★ place 중심 융합 (2026-07-18 실물 모서리 적치): 정적 상자의 스윕 관측이
    부분-림 편향으로 2~3cm 흔들려(실측), 단일 검출은 작은 상자에서 모서리 적치를
    유발. plausible base_z 검출을 클러스터링해 score-가중 평균 중심을 쓴다.
    base_z 이상 garbage(공중 오검출)는 제외 — 안 그러면 융합 중심이 딴 데로 샌다."""
    box = [
        steps.OrientedDetection(
            prompt="blue box", position=(0.28, 0.11, 0.03), score=0.71,
            base_z=-0.01, height=0.04, grasp_yaw=0.0, footprint=(0.07, 0.05),
        ),
        steps.OrientedDetection(
            prompt="blue box", position=(0.29, 0.10, 0.03), score=0.89,
            base_z=-0.01, height=0.04, grasp_yaw=0.0, footprint=(0.07, 0.05),
        ),
        steps.OrientedDetection(
            prompt="blue box", position=(0.30, 0.09, 0.03), score=0.82,
            base_z=-0.02, height=0.04, grasp_yaw=0.0, footprint=(0.07, 0.05),
        ),
    ]
    garbage = steps.OrientedDetection(  # 공중 부양 오검출 (base_z 대역 밖)
        prompt="blue box", position=(0.15, -0.12, 0.14), score=0.42,
        base_z=0.13, height=0.015, grasp_yaw=0.0, footprint=(0.04, 0.02),
    )
    fused = steps._fuse_place_center([*box, garbage])
    assert fused is not None
    wsum = 0.71 + 0.89 + 0.82
    exp_x = (0.28 * 0.71 + 0.29 * 0.89 + 0.30 * 0.82) / wsum
    assert fused.position[0] == pytest.approx(exp_x, abs=1e-4)
    assert fused.position[0] > 0.27  # garbage(0.15)로 안 끌려감 = 제외 확인


def test_fuse_place_center_none_when_single_view():
    """융합할 이웃이 없으면(plausible 1개) None → 호출부가 기존 단일-spot 유지."""
    solo = steps.OrientedDetection(
        prompt="blue box", position=(0.28, 0.11, 0.03), score=0.9,
        base_z=-0.01, height=0.04, grasp_yaw=0.0, footprint=(0.07, 0.05),
    )
    assert steps._fuse_place_center([solo]) is None


def test_fuse_place_center_picks_dominant_cluster():
    """두 후보 군집이면 score 합 큰 쪽 채택 (5cm 밖 딴 물체로 안 샌다)."""
    near = [
        steps.OrientedDetection(
            prompt="blue box", position=(0.28, 0.11, 0.03), score=0.9,
            base_z=-0.01, height=0.04, grasp_yaw=0.0, footprint=(0.07, 0.05),
        ),
        steps.OrientedDetection(
            prompt="blue box", position=(0.29, 0.10, 0.03), score=0.9,
            base_z=-0.01, height=0.04, grasp_yaw=0.0, footprint=(0.07, 0.05),
        ),
    ]
    far = steps.OrientedDetection(  # 5cm 밖 = 딴 군집, score 낮음
        prompt="blue box", position=(0.10, 0.40, 0.03), score=0.5,
        base_z=-0.01, height=0.04, grasp_yaw=0.0, footprint=(0.07, 0.05),
    )
    fused = steps._fuse_place_center([*near, far])
    assert fused is not None
    assert fused.position[0] > 0.25 and fused.position[1] > 0.05  # near 군집


async def test_plan_pick_defers_floating_candidate():
    """공중 부양(base_z 상한 밖) 후보는 score 1등이어도 후순위 — 2026-07-17
    실물: flying-pixel 이 큐브 top 을 공중으로 들어올린 관측이 허공 목표를
    만들어 servo 이동 IK 거부 (04:24 태스크 사망). 타당 후보가 resolve 첫
    시도가 되어야 오염 뷰에 resolve 예산을 태우지 않는다."""
    floating = _det(score=0.95, position=(0.2, 0.05, 0.22), base_z=0.20)
    healthy = _det(score=0.50)  # base_z=0.0 — 타당
    ctx = _ctx({
        _SELECT: [_resolve_ok()],
    })
    plan = await steps.plan_pick(
        ctx, _BOT, "white cube", _home_record(), [floating, healthy]
    )
    assert plan.coarse.score == pytest.approx(0.50)  # 타당 후보 채택
    assert len(ctx.calls(_SELECT)) == 1  # 오염 후보에 resolve 소모 0


async def test_plan_pick_rejects_all_low_score_candidates():
    """저신뢰(오검출 가능) 후보만 남으면 명시 실패 — 2026-07-17 실물: 진짜
    큐브 전멸 후 순회가 score 0.31 오검출(로봇 옆 흰 물체)로 폴백, 엉뚱한
    물체를 집으러 가 사용자 STOP. 오동작(맹목 파지)보다 정직한 실패."""
    ctx = _ctx({})
    with pytest.raises(DetectionNotFound):
        await steps.plan_pick(ctx, _BOT, "white cube", _home_record(), [
            _det(score=0.35),
            _det(score=0.31, position=(0.1, 0.22, 0.012)),
        ])
    assert ctx.calls(_SELECT) == []  # 저신뢰 후보에 resolve 예산 소모 0


async def test_plan_pick_rejects_ungraspable_width():
    """조 개구 초과 물체는 score 가 높아도 후보가 아니다 — 2026-07-17 실물:
    손에 든 큐브 전멸 후 score 0.68 footprint 116mm blob 을 채택 (lateral
    47mm 계획) → '완전 다른 데' 주행. antipodal 쌍 필터는 쓰레기 점군 안
    우연 쌍으로 우회됐다 — 후보 레벨 물리 게이트가 마지막 방어선."""
    blob = _det(
        score=0.68, position=(0.09, -0.26, -0.04), base_z=-0.057,
        footprint=(0.120, 0.116),
    )
    cube = _det(score=0.50)  # footprint 기본 (0.025, 0.022) — 통과
    ctx = _ctx({
        _SELECT: [_resolve_ok()],
    })
    plan = await steps.plan_pick(
        ctx, _BOT, "white cube", _home_record(), [blob, cube]
    )
    assert plan.coarse.score == pytest.approx(0.50)  # blob 아닌 큐브
    assert len(ctx.calls(_SELECT)) == 1  # blob 에 resolve 소모 0

    ctx2 = _ctx({})
    with pytest.raises(DetectionNotFound):  # blob 만 = 명시 실패
        await steps.plan_pick(ctx2, _BOT, "white cube", _home_record(), [blob])
    assert ctx2.calls(_SELECT) == []


# (2026-07-22) test_plan_pick_excludes_robot_base_area_candidates 삭제 —
# exclude_xy(로봇 베이스 13cm 원기둥 컷) 폐지. 그 클래스("로봇을 집으러 감",
# 07-19 OMX 모터 오검출)는 detector ROI 컷이 상류에서 담당 — ROI 를 로봇이 안
# 들어오게 두는 규율은 ROI 패널이 가시화 (pnp_scenario_rework §9.1-5, 잠금 =
# test_detector_module ROI 컷 2종).


async def test_plan_pick_low_score_excluded_from_fallback():
    """진짜 후보가 전멸해도 저신뢰 후보로 **폴백하지 않는다** — 도달성 우선
    순회의 바닥은 신뢰 후보까지. 전멸이면 엉뚱한 물체 대신 명시 실패."""
    ctx = _ctx({
        # 진짜 후보의 진입 사다리 4라운드 전멸
        _SELECT: [ResolveReachableResponse(index=-1, message="전멸")] * 4,
    })
    with pytest.raises(NoReachableGrasp, match="후보 1개 전부 전멸"):
        await steps.plan_pick(ctx, _BOT, "white cube", _home_record(), [
            _det(score=0.76),
            _det(score=0.31, position=(0.1, 0.22, 0.012)),
        ])
    assert len(ctx.calls(_SELECT)) == 4  # 저신뢰 후보는 시도 대상 아님 (사다리만 4단)


async def test_plan_place_falls_back_to_free_yaw_family():
    ctx = _ctx({
        _SELECT: [
            ResolveReachableResponse(index=-1, message="정렬 yaw 전멸"),
            _resolve_ok(),
        ],
    })
    chosen, _pre = await steps.plan_place(
        ctx, _BOT, "cube",
        home=_home_record(), spots=[_det(grasp_yaw=0.3)],
    )
    assert len(ctx.calls(_SELECT)) == 2
    assert chosen.label == "tilt=+0 yaw=47"  # 자유 가족 첫 후보 (30°+17°)


# ─── module wire (runner 결합 e2e) ───────────────────────────────────


class _WireStub:
    def __init__(self) -> None:
        self.published: list[tuple[str, BaseModel]] = []
        self.responses: dict[str, BaseModel] = {}

    def publish(self, wire_key: str, event: BaseModel) -> None:
        self.published.append((str(wire_key), event))

    async def call(self, key, req, res_cls, *, robot_id=None, timeout=None):  # noqa: ANN001, ANN201
        r = self.responses.get(str(key))
        if r is None:
            raise AssertionError(f"call 스크립트 없음: {key}")
        return r


async def test_module_run_reports_failure_reason_and_allows_rerun():
    rt = _WireStub()
    rt.responses[_GET_WP_BY_NAME] = GetWaypointByNameResponse(waypoint=_home_record())
    rt.responses[_LIST_MEMBERS_BY_NAME] = ListGroupMembersByNameResponse(
        found=True,
        waypoints=[
            WaypointRecord(
                id=1, robot_id=_BOT, name="s0",
                joint_values=[0.0] * 6, joint_names=[], created_at=_TS,
            )
        ],
    )
    rt.responses[_MOVE_J] = MoveJResponse()
    rt.responses[_DETECT] = DetectOrientedResponse(found=False, candidates=[])
    rt.responses[str(Motion.Service.STOP)] = StopResponse(ok=True)
    mod = PickAndPlaceModule(rt, {})  # type: ignore[arg-type]

    res = await mod.run(RunRequest(pick_object="white cube"))
    assert res.accepted
    assert mod.task._run is not None and mod.task._run.handle is not None
    await mod.task._run.handle

    states = [e for k, e in rt.published if k.endswith("/state")]
    final = states[-1]
    assert isinstance(final, TaskState)
    assert final.status == TaskStatus.FAILED
    assert final.error is not None and "white cube" in final.error

    res2 = await mod.run(RunRequest(pick_object="white cube"))
    assert res2.accepted


async def test_scenario_republishes_grasp_marker_during_servo():
    """servo 채택 관측이 파지점을 갱신할 때마다 마커 스트림 재발행 — 계획
    시점 마커가 실행 내내 고정 표시되던 UI 구멍 (2026-07-17 사용자 리포트).
    on_grasp 배선을 빼면 발행이 계획 1회로 줄어 즉시 잡힌다."""
    rt = _WireStub()
    mod = PickAndPlaceModule(rt, {})  # type: ignore[arg-type]
    ctx = _ctx(_pick_script())
    await mod.scenario(ctx, pick_object="white cube")
    marker_events = [
        e for k, e in rt.published
        if k.endswith("/markers") and isinstance(e, TaskMarkers)
    ]
    # 계획 확정 1회 + servo 채택 tick 마다 (script = 2 tick 채택)
    assert len(marker_events) >= 3, [k for k, _ in rt.published]
    for ev in marker_events:
        m = ev.markers[0]
        assert m.label == "grasp"
        assert len(m.position) == 3
        # 파지 방향 동봉 (2026-07-19) — 화살표(approach)/조 축 바(jaw_axis)/
        # 자세(quat)의 시각화 소스. 계획·servo 갱신 발행 모두 실려야 한다
        # (빼면 "이 면을 이 방향으로" 오버레이가 죽는 회귀).
        assert m.approach == pytest.approx(_FAM.approach)
        assert m.jaw_axis == pytest.approx(_FAM.jaw_axis)
        assert m.quaternion == pytest.approx(_FAM.quat)
    # seq 단조 증가 (latest-wins 스트림 계약)
    seqs = [ev.seq for ev in marker_events]
    assert seqs == sorted(seqs)


async def test_module_control_without_run_says_why():
    mod = PickAndPlaceModule(_WireStub(), {})  # type: ignore[arg-type]
    r = await mod.pause(ControlRequest())
    assert not r.ok and r.message


async def test_module_preview_returns_static_tree_without_wire():
    mod = PickAndPlaceModule(_WireStub(), {})  # type: ignore[arg-type]
    res = await mod.preview(PreviewRequest())
    assert [e.name for e in res.entries if e.depth == 0] == [
        "home_waypoint", "detect", "approach_observe", "plan_place",
        "approach_observe", "plan_pick", "servo_pick", "execute_place",
    ]
    assert not mod._seq["state"]


async def test_module_toggle_breakpoint_before_run_publishes_state():
    rt = _WireStub()
    mod = PickAndPlaceModule(rt, {})  # type: ignore[arg-type]
    r = await mod.toggle_breakpoint(ToggleBreakpointRequest(name="servo_pick"))
    assert r.ok and "다음 실행" in r.message

    states = [e for k, e in rt.published if k.endswith("/state")]
    assert states, "run 밖 토글이 침묵 — STATE 미발행"
    final = states[-1]
    assert isinstance(final, TaskState)
    assert final.robot_id == _BOT
    assert final.status == TaskStatus.IDLE
    assert final.breakpoints == ["servo_pick"]


async def test_list_robots_returns_task_robots():
    mod = PickAndPlaceModule(_WireStub(), {})  # type: ignore[arg-type]
    res = await mod.list_robots(ListRobotsRequest())
    assert res.robot_ids == list(PickAndPlaceModule.TASK_ROBOTS)


# ─── 파지 판정 (물었나/놓쳤나) — 기존 잠금 계승 ───────────────────────


def test_gripper_holding_judgment():
    """gap OR 부하 판정 (2026-07-17 실측 기반 — steps._gripper_holding 주석)."""
    assert steps._gripper_holding(_HELD_RAW, None, _SPEC) is True
    # 빈손: gap≈0 + 저부하 (실측 56~64)
    assert steps._gripper_holding(_EMPTY_RAW, 60, _SPEC) is False
    # 얇은 물림/슬립 sliver: gap 은 margin 아래지만 부하가 누르는 중 (실측 296)
    thin = _SPEC.gripper_close_raw + 36
    assert steps._gripper_holding(thin, 296, _SPEC) is True
    # 같은 gap 인데 부하 낮음 = 빈손
    assert steps._gripper_holding(thin, 60, _SPEC) is False
    # 부하 신호 없는 모델 → gap 단독 (얇은 물림은 못 잡음 — 알려진 한계)
    assert steps._gripper_holding(thin, None, _SPEC) is False


def test_position_only_cannot_catch_false_stall():
    """알려진 한계 박제: 물체 없이 어중간히 stall 하면 위치 판정은 HELD 오판
    (false-positive) — load 병기 로그로 실물 튜닝이 해법."""
    false_stall = _SPEC.gripper_held_threshold_raw + 200
    assert steps._gripper_holding(false_stall, None, _SPEC) is True


async def test_verify_grasp_empty_raises_with_reason():
    ctx = _ctx({_READ_STATE: [_joint_state(_EMPTY_RAW, load=3)]})
    with pytest.raises(GraspFailed) as ei:
        await steps.verify_grasp(ctx, _BOT, phase="close 직후", grasp_label="w=22mm")
    assert "파지 실패" in str(ei.value) and "close 직후" in str(ei.value)
    assert ei.value.achieved_raw == _EMPTY_RAW


async def test_verify_grasp_held_passes():
    ctx = _ctx({_READ_STATE: [_joint_state(_HELD_RAW, load=280)]})
    await steps.verify_grasp(ctx, _BOT, phase="close 직후")
    assert len(ctx.calls(_READ_STATE)) == 1


# ── approach_observe 시점 품질 (2026-07-21 −60° 측면뷰 실사고 회귀망) ──────────


def test_camera_look_poses_frame_object_from_above():
    """★ look-pose 생성 = **카메라 중심** (2026-07-21 사용자 실증 자세가 규정).

    각 후보를 hand-eye 로 카메라 자세로 되돌려 검증: ① 광축이 물체를 정확히
    응시 ② 지정 거리 ③ 고각 ≥45° (옆/아래 뷰 배제 — 실사고 ratio 0.31/0.55)
    ④ 고각×방위 전 격자 (파지 가족 파생이 놓치던 "손목 꺾어 보기" 포함 —
    가장자리 전멸 회귀 방지)."""
    from modules.tasks.pick_and_place.steps import approach

    t_ee_cam = np.eye(4)
    t_ee_cam[:3, 3] = (-0.077, -0.009, -0.065)
    obj = (0.20, 0.05, 0.03)
    dist = approach._OBSERVE_CAM_DIST_M[0]
    poses = approach._camera_look_poses(obj, t_ee_cam, dist)
    assert len(poses) == len(approach._OBSERVE_ELEV_DEG) * approach._OBSERVE_AZIM_N
    from scipy.spatial.transform import Rotation as _R

    for i, p in enumerate(poses):
        t_be = np.eye(4)
        t_be[:3, :3] = _R.from_quat(p.quaternion).as_matrix()
        t_be[:3, 3] = p.position
        t_bc = t_be @ t_ee_cam  # 카메라 자세 복원
        campos, optical = t_bc[:3, 3], t_bc[:3, 2]
        ray = np.asarray(obj) - campos
        d = float(np.linalg.norm(ray))
        assert abs(d - dist) < 1e-9, f"후보{i} 거리 {d}"
        assert float(np.dot(optical, ray / d)) > 0.999999, f"후보{i} 광축 이탈"
        elev = float(np.degrees(np.arcsin(-(ray / d)[2])))
        assert elev >= min(approach._OBSERVE_ELEV_DEG) - 1e-6, f"후보{i} 저고각 {elev}"
    # 수직(90°)이 선호 1순위 — 첫 후보의 카메라는 물체 바로 위
    t_be = np.eye(4)
    t_be[:3, :3] = _R.from_quat(poses[0].quaternion).as_matrix()
    t_be[:3, 3] = poses[0].position
    cam0 = (t_be @ t_ee_cam)[:3, 3]
    assert abs(cam0[0] - obj[0]) < 1e-9 and abs(cam0[1] - obj[1]) < 1e-9


async def test_approach_observe_distance_ladder_retries_farther():
    """가장자리 물체: 기본 카메라 거리(13cm) 전멸이면 18cm 로 재시도해 **그래도
    가까이서 본다** (coarse 31cm 폴백은 최후) — "가장자리는 close 포기" 반려
    (2026-07-21 사용자 지시)."""
    coarse = _det(points=[(0.2, 0.05, 0.01)] * 300)
    close = _det(position=(0.201, 0.051, 0.025), points=[(0.2, 0.05, 0.01)] * 300)
    script = {
        _SELECT: [
            ResolveReachableResponse(index=-1, message="전멸"),  # 13cm
            ResolveReachableResponse(index=0, solutions=[[0.2] * 6]),  # 18cm
        ],
        _MOVE_J: [MoveJResponse(), MoveJResponse()],
        _DETECT: [DetectOrientedResponse(found=True, candidates=[close])],
    }
    ctx = _ctx(script)
    cands, joints, close_ok = await steps.approach_observe(
        ctx, _BOT, [coarse], "blue box", _home_record()
    )
    assert close_ok is True  # 18cm 에서라도 close 관측 성공
    assert joints == [0.2] * 6
    assert len(ctx.calls(_SELECT)) == 2  # 거리 사다리 2단 시도


def _approach_script(close_det) -> dict:
    """approach_observe 호출 시퀀스 스크립트 — resolve → home → look → 관측 1프레임."""
    return {
        _SELECT: [ResolveReachableResponse(index=0, solutions=[[0.1] * 6])],
        _MOVE_J: [MoveJResponse(), MoveJResponse()],
        _DETECT: [DetectOrientedResponse(found=True, candidates=[close_det])],
    }


async def test_approach_observe_distrusts_collapsed_close_points():
    """★ close 관측 물체 points 가 coarse 대비 급감(시점 불량)이면 그 관측을
    믿지 않고 coarse 폴백 (close=False → yaw 격자 유지). 실사고 재현: 397/1296
    = 0.31 을 '성공'으로 채택해 base_z 26mm 오차가 place 계획에 들어갔다."""
    coarse = _det(points=[(0.2, 0.05, 0.01)] * 300)
    poor_close = _det(points=[(0.2, 0.05, 0.01)] * 50)  # 50 < 300×0.7
    ctx = _ctx(_approach_script(poor_close))
    cands, joints, close = await steps.approach_observe(
        ctx, _BOT, [coarse], "blue box", _home_record()
    )
    assert close is False, "시점 불량 관측이 close 성공으로 채택됨"
    assert cands == [coarse]  # coarse 유지 (불신한 관측을 계획에 안 씀)
    assert joints == [0.1] * 6  # look 자세는 반환 (현재 위치 문맥)


async def test_approach_observe_trusts_healthy_close_points():
    """뒤집기 대조군: points 가 coarse 급(≥0.7×)이면 close 관측 채택 (close=True)."""
    coarse = _det(points=[(0.2, 0.05, 0.01)] * 300)
    good_close = _det(
        position=(0.201, 0.051, 0.025), points=[(0.2, 0.05, 0.01)] * 300
    )
    ctx = _ctx(_approach_script(good_close))
    cands, _joints, close = await steps.approach_observe(
        ctx, _BOT, [coarse], "blue box", _home_record()
    )
    assert close is True
    assert cands[0].position == (0.201, 0.051, 0.025)  # 정확본으로 교체


# ── 적응 진입 사다리 (2026-07-21 "68가족 매장" 감사 회귀망) ───────────────────


def test_servo_ladder_groups_standoffs_override():
    """standoffs override — 그룹 pose 수 = 진입 rung 수 + 파지 1. 1-rung 진입
    사다리(낮은 진입 폴백)가 기본과 같은 구성 규약으로 나온다."""
    g2, _ = steps.servo_ladder_groups(_OBS, _CFG, standoffs=(0.05,))
    assert all(len(g) == 2 for g in g2)  # [5cm, 파지]
    g3, _ = steps.servo_ladder_groups(_OBS, _CFG)  # 기본 = cfg.standoffs 2단
    assert all(len(g) == 3 for g in g3)


async def test_plan_pick_entry_ladder_falls_back_lower():
    """★ 기본 사다리 전멸 → 5cm 진입 라운드에서 채택. 실사고(01:28): 진입
    rung(8cm)이 같은 자세를 못 만들어 파지 가능 68가족이 "도달 불가"로 매장 —
    낮은 진입 폴백이 그 가족들을 살린다. 채택 사다리는 plan.standoffs 로
    servo 에 전달 (판정 사다리 == 실행 사다리)."""
    ctx = _ctx({
        _SELECT: [
            ResolveReachableResponse(index=-1, message="전멸"),  # 기본 (2-rung)
            _resolve_ok(index=0),  # 5cm 진입 라운드 채택
        ],
    })
    plan = await steps.plan_pick(
        ctx, _BOT, "white cube", _home_record(), [_det(score=0.9)]
    )
    assert plan.standoffs == (0.05,)
    calls = ctx.calls(_SELECT)
    assert len(calls) == 2
    # 1라운드 = 기본 2-rung(+파지 3 pose), 2라운드 = 1-rung(+파지 2 pose)
    assert len(calls[0]["req"].groups[0]) == 3
    assert len(calls[1]["req"].groups[0]) == 2


def test_effective_cfg_applies_plan_standoffs():
    """plan 이 채택한 진입 사다리 → 실행 cfg 반영 + eps 는 뒤에서부터 (마지막
    rung 최엄격 유지). None/동일 사다리는 cfg 원본 그대로 (기존 경로 무변)."""
    from modules.tasks.pick_and_place.steps import pick as pick_steps

    base_plan = steps.ServoPlan(
        coarse=_OBS, family=_FAM, rung0_joints=[0.1] * 6,
        grasp_point0=_G_POINT, grasp_tcp0=_G_TCP, lateral0=_LAT,
    )
    assert pick_steps._effective_cfg(_CFG, base_plan) is _CFG  # None = 무변
    from dataclasses import replace as _rep

    short = _rep(base_plan, standoffs=(0.05,))
    eff = pick_steps._effective_cfg(_CFG, short)
    assert eff.standoffs == (0.05,)
    assert eff.eps_descend_m == (_CFG.eps_descend_m[-1],)  # 최엄격 eps
    same = _rep(base_plan, standoffs=tuple(_CFG.standoffs))
    assert pick_steps._effective_cfg(_CFG, same) is _CFG
