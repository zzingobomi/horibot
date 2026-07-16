"""Pick & Place task 테스트 — closed-loop(servo) 집기 판 (2026-07-16 재설계).

의미 (뒤집으면 회귀): servo 루프가 이동 중 관측 / 관측 없이 명령(맹목) / mask
오검출 tick 을 그대로 파지에 반영 / 관측 소실·수렴 실패가 침묵 진행 or 무한 대기 /
close 후 EMPTY 가 재시도 없이 즉사 or 무한 재시도 / servo 이동 거부가 침묵 통과 /
trace 미기록(실패 재구성 불가) / place 분기가 pick-only 에서 실행 / 놓기 도달
불가가 집은 뒤에 발견 (쥔 채 멈춤 corrupt) / RUN 동시 실행 허용.

servo 순수 계산(가족/gate/decide_tick) 잠금은 test_servo.py.
"""

from __future__ import annotations

from datetime import UTC, datetime
from pathlib import Path

import numpy as np
import pytest
from pydantic import BaseModel

from framework.transport.protocol import RemoteError
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
from modules.tasks.pick_and_place.contract import ListRobotsRequest, RunRequest
from modules.tasks.pick_and_place.module import PickAndPlaceModule
from modules.waypoint.contract import (
    ListGroupMembersResponse,
    ListGroupsResponse,
    ListWaypointsResponse,
    Waypoint,
    WaypointGroupRecord,
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
_MOVE_J = str(Motion.Service.MOVE_J)
_MOVE_L = str(Motion.Service.MOVE_L)
_GRIP = str(Motor.Service.SET_GRIPPER)
_READ_STATE = str(Motor.Service.READ_STATE)
_TCP_SNAP = str(Motion.Service.TCP_SNAPSHOT)
_LIST_WP = str(Waypoint.Service.LIST)
_LIST_GROUPS = str(Waypoint.Service.LIST_GROUPS)
_LIST_MEMBERS = str(Waypoint.Service.LIST_GROUP_MEMBERS)
_TS = datetime.fromtimestamp(0, UTC)

_HOME_JOINTS = [0.0, 0.5, -1.0, 0.0, 0.5, 1.5]  # 티칭된 home (임의 유효값)

# 테스트용 servo 설정 — 2단 사다리 + settle 0 (결정적·빠름). 실 기본값의
# 상태 전이 자체는 test_servo 가 잠근다.
_CFG = servo.ServoConfig(
    standoffs=(0.10, 0.05),
    eps_descend_m=(0.008, 0.004),
    corrections_per_rung=3,
    settle_s=0.0,
)


def _home_record() -> WaypointRecord:
    return WaypointRecord(
        id=99, robot_id=_BOT, name="home",
        joint_values=list(_HOME_JOINTS), joint_names=[], created_at=_TS,
    )


def _home_responses() -> dict:
    return {_LIST_WP: [ListWaypointsResponse(waypoints=[_home_record()])] * 2}


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
) -> OrientedDetection:
    return OrientedDetection(
        prompt="cube", position=position, score=score, base_z=base_z,
        height=height, grasp_yaw=grasp_yaw, footprint=footprint,
        points=_pts() if points is None else points,
    )


# ── servo 기대값 (production 함수로 산출 — 배선 검증은 호출 순서/값 대조로) ──

_OBS = _det()
_FAM = servo.grasp_families(_OBS)[0]  # resolve index=0 → 첫 가족 (수직·jaw∥short)
_WIDTH = servo.width_along(_OBS.points, _FAM.jaw_axis, _OBS.footprint[1])
_LAT = servo.lateral_offset(_WIDTH)
_G_POINT = servo.grasp_point(_OBS, _OBS, _CFG)
_G_TCP = servo.grasp_tcp(_G_POINT, _FAM, _LAT)
_SO0 = servo.standoff(_G_TCP, _FAM, _CFG.standoffs[0])
_SO1 = servo.standoff(_G_TCP, _FAM, _CFG.standoffs[1])
_WITHDRAW = servo.standoff(_G_TCP, _FAM, _CFG.withdraw_standoff_m)


def _search_responses(n_members: int = 1, sweeps: int = 1) -> dict:
    grp = ListGroupsResponse(
        groups=[WaypointGroupRecord(id=1, robot_id=_BOT, name="search")]
    )
    members = ListGroupMembersResponse(
        waypoints=[
            WaypointRecord(
                id=i + 1, robot_id=_BOT, name=f"s{i}",
                joint_values=[0.0] * 6, joint_names=[], created_at=_TS,
            )
            for i in range(n_members)
        ]
    )
    return {
        **_home_responses(),
        _LIST_GROUPS: [grp] * (sweeps + 1),
        _LIST_MEMBERS: [members] * (sweeps + 1),
    }


@pytest.fixture(autouse=True)
def _fast_servo(monkeypatch: pytest.MonkeyPatch, tmp_path: Path):
    monkeypatch.setattr(steps, "_GRIPPER_SETTLE_S", 0.0)
    monkeypatch.setattr(steps, "_SEARCH_SETTLE_S", 0.0)
    monkeypatch.setattr(steps, "_SERVO_CFG", _CFG)
    # trace 는 실제로 쓴다 (기록 자체가 요구사항) — 저장소만 tmp 로
    monkeypatch.setattr(servo_trace, "_TRACE_ROOT", tmp_path / "servo_pick")


def _ctx(script: dict) -> FakeContext:
    return FakeContext(robots=[_BOT], specs={_BOT: _SPEC}, service_script=script)


def _module_for_scenario() -> PickAndPlaceModule:
    class _Rt:
        def publish(self, k: str, e: BaseModel) -> None: ...
        async def call(self, *a, **kw): ...  # noqa: ANN002, ANN003, ANN201

    return PickAndPlaceModule(_Rt(), {})  # type: ignore[arg-type]


def _pick_script(**overrides) -> dict:
    """pick 경로 성공 스크립트 — 스윕 1자세 → 계획 resolve → servo 2 tick
    (tick1 rung0 수렴→하강, tick2 rung1 수렴→commit) → close/withdraw 판정."""
    script = {
        **_search_responses(),
        _DETECT: [
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # 스윕
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # tick1
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # tick2
        ],
        _SELECT: [_resolve_ok()],
        # tick1 TCP = rung0 standoff (오차 0 → 하강), tick2 = rung1 (→ commit),
        # 마지막 = 도달 로깅 (commit 후).
        _TCP_SNAP: [_tcp(_SO0), _tcp(_SO1), _tcp(_G_TCP)],
        _FUSE: [FuseOrientedResponse(candidates=[_OBS])],  # tick2 (관측 2건부터)
        _MOVE_J: [MoveJResponse()] * 4,  # 스윕 + servo home + rung0 + 종료 home
        _MOVE_L: [MoveLResponse()] * 3,  # 하강 + commit + withdraw
        _GRIP: [SetGripperResponse()] * 2,  # open + close
        _READ_STATE: [_joint_state(_HELD_RAW)] * 2,  # close/withdraw 판정
    }
    script.update(overrides)
    return script


# ─── servo 시나리오 (FakeContext — 하드웨어/wire 없음) ────────────────


async def test_scenario_servo_pick_only_sequence():
    mod = _module_for_scenario()
    ctx = _ctx(_pick_script())

    await mod.scenario(ctx, pick_object="white cube")

    assert ctx.keys() == [
        _LIST_WP,  # home 조회 (모션 0)
        _LIST_GROUPS, _LIST_MEMBERS, _MOVE_J, _DETECT,  # 스윕 (coarse 찾기)
        _SELECT,  # servo 접근 계획 (가족+사다리, 모션 0)
        _MOVE_J, _MOVE_J, _GRIP,  # servo 진입: home 경유 → rung0 → open
        _DETECT, _TCP_SNAP,  # tick1 (관측 1건 — 융합 생략)
        _MOVE_L,  # rung1 하강
        _DETECT, _TCP_SNAP, _FUSE,  # tick2 (관측 2건 융합)
        _MOVE_L, _TCP_SNAP,  # commit (blind) + 도달 로깅
        _GRIP, _READ_STATE,  # close + 판정 ①
        _MOVE_L, _READ_STATE,  # withdraw + 판정 ②
        _MOVE_J,  # 종료 home
    ]
    # rung0 진입 = resolve 가 반환한 첫 standoff IK 해 그대로 (재계산 금지)
    assert ctx.calls(_MOVE_J)[1]["req"].target.joints == _HOME_JOINTS
    assert ctx.calls(_MOVE_J)[2]["req"].target.joints == [0.1] * 6
    # servo 이동 목표 = production servo 함수 산출값 (common-mode 상대 명령 배선)
    ml = [c["req"].target for c in ctx.calls(_MOVE_L)]
    assert ml[0].position == pytest.approx(_SO1, abs=1e-9)  # 하강
    assert ml[1].position == pytest.approx(_G_TCP, abs=1e-9)  # commit
    assert ml[2].position == pytest.approx(_WITHDRAW, abs=1e-9)  # 후퇴
    assert all(m.quaternion == pytest.approx(_FAM.quat, abs=1e-9) for m in ml)
    # 계획 resolve 계약: 사다리+파지 직선(linear) + 그리퍼 벌림 충돌 + 바닥 +
    # home 경로 게이트, 그룹당 pose = standoff 2 + grasp 1.
    sel = ctx.calls(_SELECT)[0]["req"]
    assert sel.linear is True and sel.gripper_open is True
    assert sel.floor_z == pytest.approx(0.0 - 0.005)
    assert sel.path_from == _HOME_JOINTS
    assert sel.obstacle_points
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
            DetectOrientedResponse(found=True, candidates=[outlier]),  # tick1 기각
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # tick2 채택
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # tick3
        ],
        _TCP_SNAP: [_tcp(_SO0), _tcp(_SO0), _tcp(_SO1), _tcp(_G_TCP)],
        _FUSE: [FuseOrientedResponse(candidates=[_OBS])],
    }))
    await _module_for_scenario().scenario(ctx, pick_object="white cube")

    keys = ctx.keys()
    # tick1(기각) 과 tick2(채택) 사이 = 모션 없음 (DETECT,TCP 다음 바로 DETECT)
    i1 = keys.index(_DETECT, keys.index(_GRIP))  # servo 첫 DETECT
    assert keys[i1 : i1 + 5] == [_DETECT, _TCP_SNAP, _DETECT, _TCP_SNAP, _MOVE_L]
    # 파지는 정상 관측 기준 (오검출이 목표에 안 섞임)
    ml = [c["req"].target.position for c in ctx.calls(_MOVE_L)]
    assert ml[1] == pytest.approx(_G_TCP, abs=1e-9)


async def test_servo_lost_at_start_fails_with_reason_and_trace(tmp_path: Path):
    """servo 진입 후 물체를 한 번도 못 보면 (연속 소실) — 맹목 진행이 아니라
    ServoFailed (사유 포함) + 파지 모션 0 + trace/summary 기록."""
    ctx = _ctx(_pick_script(**{
        _DETECT: [
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # 스윕
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
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # tick1 → 하강
            DetectOrientedResponse(found=False, candidates=[]),  # tick2 miss(hold)
            DetectOrientedResponse(found=False, candidates=[]),  # tick3 → commit
        ],
        _TCP_SNAP: [_tcp(_SO0), _tcp(_SO1), _tcp(_SO1), _tcp(_G_TCP)],
        _FUSE: [],  # 융합 없음 (채택 관측 1건뿐)
    }))
    await _module_for_scenario().scenario(ctx, pick_object="white cube")
    ml = [c["req"].target.position for c in ctx.calls(_MOVE_L)]
    assert ml[1] == pytest.approx(_G_TCP, abs=1e-9)  # 직전 관측 기준 commit


async def test_servo_empty_close_retries_from_standoff_then_succeeds():
    """close 후 EMPTY = 물체가 밀렸을 수 있다 — 놓고 rung1 로 물러나 재관측부터
    재시도 (상한 1회). 옛 open-loop 은 여기서 그냥 실패였다."""
    ctx = _ctx(_pick_script(**{
        _DETECT: [
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # 스윕
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # tick1
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # tick2 → commit
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # 재시도 tick
        ],
        _TCP_SNAP: [
            _tcp(_SO0), _tcp(_SO1), _tcp(_G_TCP),  # tick1/2 + 도달로깅
            _tcp(_SO1), _tcp(_G_TCP),  # 재시도 tick + 도달로깅
        ],
        _FUSE: [FuseOrientedResponse(candidates=[_OBS])],
        _MOVE_L: [MoveLResponse()] * 6,  # 하강+commit + 후퇴 + commit2 + withdraw
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
    assert ml[2] == pytest.approx(_SO1, abs=1e-9)
    assert ml[3] == pytest.approx(_G_TCP, abs=1e-9)


async def test_servo_empty_close_exhausted_raises():
    """재시도 상한까지 EMPTY → GraspFailed (무한 재시도 금지 — handoff §2 표)."""
    ctx = _ctx(_pick_script(**{
        _DETECT: [DetectOrientedResponse(found=True, candidates=[_OBS])] * 4,
        _TCP_SNAP: [
            _tcp(_SO0), _tcp(_SO1), _tcp(_G_TCP),
            _tcp(_SO1), _tcp(_G_TCP),
        ],
        _FUSE: [FuseOrientedResponse(candidates=[_OBS])],
        _MOVE_L: [MoveLResponse()] * 6,
        _GRIP: [SetGripperResponse()] * 4,
        _READ_STATE: [_joint_state(_EMPTY_RAW)] * 2,  # 두 attempt 모두 빈 파지
    }))
    with pytest.raises(GraspFailed) as ei:
        await _module_for_scenario().scenario(ctx, pick_object="white cube")
    assert ei.value.phase == "close 직후"
    assert len(ctx.calls(_READ_STATE)) == 2  # attempt 2회에서 끝 (withdraw 판정 X)


async def test_servo_move_rejected_falls_back_to_movej():
    """servo 이동 MoveL 거부 (경로 IK) → MoveJ 폴백으로 계속 — 거부가 침묵
    통과("명령은 항상 실행된다" 가정)하면 회귀."""
    obs2 = _det(position=(0.2, 0.062, 0.025))  # 12mm 이탈 → correct 유발
    ctx = _ctx(_pick_script(**{
        _DETECT: [
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # 스윕
            DetectOrientedResponse(found=True, candidates=[obs2]),  # tick1 correct
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # tick2 하강
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # tick3 commit
        ],
        _TCP_SNAP: [_tcp(_SO0), _tcp(_SO0), _tcp(_SO1), _tcp(_G_TCP)],
        _FUSE: [FuseOrientedResponse(candidates=[_OBS])] * 2,
        _MOVE_L: [
            RemoteError("MotionRejected", "경로 IK 실패"),  # correct 이동 거부
            MoveLResponse(),  # 하강
            MoveLResponse(),  # commit
            MoveLResponse(),  # withdraw
        ],
        _MOVE_J: [MoveJResponse()] * 5,  # 스윕+home+rung0 + 폴백 + 종료 home
    }))
    await _module_for_scenario().scenario(ctx, pick_object="white cube")
    # 폴백 MoveJ = pose 타깃 (거부된 correct 목표 그대로)
    fallback = ctx.calls(_MOVE_J)[3]["req"].target
    assert fallback.kind == "pose"


async def test_servo_move_both_rejected_aborts():
    ctx = _ctx(_pick_script(**{
        _DETECT: [
            DetectOrientedResponse(found=True, candidates=[_OBS]),
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # tick1 하강 시도
        ],
        _TCP_SNAP: [_tcp(_SO0)],
        _MOVE_L: [RemoteError("MotionRejected", "경로 IK 실패")],
        _MOVE_J: [
            MoveJResponse(), MoveJResponse(), MoveJResponse(),  # 스윕+home+rung0
            RemoteError("MotionRejected", "IK 실패"),  # 폴백도 거부
        ],
    }))
    with pytest.raises(ServoFailed, match="이동 실패"):
        await _module_for_scenario().scenario(ctx, pick_object="white cube")
    assert ctx.calls(_GRIP)[-1]["req"].position_raw == _SPEC.gripper_open_raw
    assert ctx.calls(_READ_STATE) == []  # 파지 시도 없음


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
            DetectOrientedResponse(found=True, candidates=[far]),  # tick1 correct
            DetectOrientedResponse(found=True, candidates=[far]),  # tick2 → abort
        ],
        # TCP 가 목표를 안 따라감 (오차 유지) — 발진/정체 재현
        _TCP_SNAP: [_tcp(_SO0), _tcp(_SO0)],
        _FUSE: [FuseOrientedResponse(candidates=[far])],
        _MOVE_L: [MoveLResponse()],  # correct 1회
    }))
    import modules.tasks.pick_and_place.steps as steps_mod
    steps_mod._SERVO_CFG = cfg  # 이 테스트만 보정 상한 1
    try:
        with pytest.raises(ServoFailed, match="수렴 실패"):
            await _module_for_scenario().scenario(ctx, pick_object="white cube")
    finally:
        steps_mod._SERVO_CFG = _CFG


async def test_scenario_with_place_branch_places_after_servo():
    place_spot = _det(position=(0.25, -0.05, 0.04), height=0.04, grasp_yaw=0.3)
    ctx = _ctx(_pick_script(**{
        **_search_responses(sweeps=2),
        _DETECT: [
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # pick 스윕
            DetectOrientedResponse(found=True, candidates=[place_spot]),  # place 스윕
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # tick1
            DetectOrientedResponse(found=True, candidates=[_OBS]),  # tick2
        ],
        _SELECT: [_resolve_ok(), _resolve_ok()],  # servo 계획 + place 정렬 가족
        _MOVE_J: [MoveJResponse()] * 8,
        _MOVE_L: [MoveLResponse()] * 5,  # servo 3 + insert + retreat
        _GRIP: [SetGripperResponse()] * 3,  # open/close/release
        _READ_STATE: [_joint_state(_HELD_RAW)] * 3,  # close/withdraw/적치 직전
    }))
    await _module_for_scenario().scenario(
        ctx, pick_object="white cube", place_object="red box"
    )
    grips = [c["req"].position_raw for c in ctx.calls(_GRIP)]
    assert grips == [
        _SPEC.gripper_open_raw, _SPEC.gripper_close_raw, _SPEC.gripper_open_raw,
    ]
    # 계획 우선 불변식: 놓기 도달성 판정(RESOLVE ×2)이 전부 끝난 뒤에야 servo
    # 진입(GRIP/MOVE_L) — 못 놓을 물체를 집지 않는다.
    keys = ctx.keys()
    assert keys[: keys.index(_GRIP)].count(_SELECT) == 2
    assert keys[: keys.index(_MOVE_L)].count(_SELECT) == 2


async def test_scenario_place_unreachable_fails_before_pick():
    """놓을 곳 도달 불가 → 집기 **전에** 실패 (쥔 채 멈춤 corrupt 방지) —
    servo 모션·파지 0."""
    place_spot = _det(position=(0.15, 0.10, 0.22), base_z=0.20)
    ctx = _ctx({
        **_search_responses(sweeps=2),
        _DETECT: [
            DetectOrientedResponse(found=True, candidates=[_OBS]),
            DetectOrientedResponse(found=True, candidates=[place_spot]),
        ],
        _SELECT: [
            _resolve_ok(),  # servo 계획
            ResolveReachableResponse(index=-1, message="정렬 전멸"),
            ResolveReachableResponse(index=-1, message="자유 전멸"),
        ],
        _MOVE_J: [MoveJResponse()] * 2,  # 스윕 2회 (pick/place)
    })
    with pytest.raises(NoReachableGrasp, match="놓을 자리 도달 불가"):
        await _module_for_scenario().scenario(
            ctx, pick_object="white cube", place_object="red box"
        )
    assert ctx.calls(_GRIP) == []
    assert ctx.calls(_MOVE_L) == []


async def test_plan_pick_family_exhausted_fails_explicitly():
    """servo 접근 가족 전멸(-1) = 데이터 → step 이 치명 판정 (침묵 통과 금지),
    모션은 스윕뿐."""
    ctx = _ctx({
        **_search_responses(),
        _DETECT: [DetectOrientedResponse(found=True, candidates=[_OBS])],
        _SELECT: [ResolveReachableResponse(index=-1, message="전멸")],
        _MOVE_J: [MoveJResponse()],
    })
    with pytest.raises(NoReachableGrasp, match="servo 접근 가족"):
        await _module_for_scenario().scenario(ctx, pick_object="white cube")
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
    (첫 자세서 안 멈춤)."""
    ctx = _ctx({
        **_search_responses(n_members=2),
        _DETECT: [
            DetectOrientedResponse(found=True, candidates=[_det(score=0.4)]),
            DetectOrientedResponse(found=True, candidates=[_det(score=0.95)]),
        ],
        _MOVE_J: [MoveJResponse()] * 2,
    })
    cands = await steps.detect(ctx, _BOT, "white cube")
    assert len(ctx.calls(_MOVE_J)) == 2
    assert [c.score for c in cands] == [0.4, 0.95]
    coarse = geometry.select_target_by_score(cands, prompt="white cube")
    assert coarse.score == 0.95


async def test_search_group_missing_fails_explicitly():
    ctx = _ctx({
        **_home_responses(),
        _LIST_GROUPS: [ListGroupsResponse(groups=[])],
    })
    with pytest.raises(TaskError, match="search"):
        await _module_for_scenario().scenario(ctx, pick_object="white cube")
    assert ctx.calls(_MOVE_J) == []


async def test_scenario_home_waypoint_missing_fails_before_any_motion():
    ctx = _ctx({_LIST_WP: [ListWaypointsResponse(waypoints=[])]})
    with pytest.raises(TaskError, match="home"):
        await _module_for_scenario().scenario(ctx, pick_object="white cube")
    assert ctx.calls(_MOVE_J) == []
    assert ctx.calls(_LIST_GROUPS) == []


async def test_servo_success_writes_trace_and_summary(tmp_path: Path):
    """성공 run 도 tick trace + summary — "성공했는데 왜 성공했는지 모른다" 방지
    (실물 임계 튜닝의 데이터 소스)."""
    ctx = _ctx(_pick_script())
    await _module_for_scenario().scenario(ctx, pick_object="white cube")
    runs = list((tmp_path / "servo_pick").iterdir())
    assert len(runs) == 1
    lines = (runs[0] / "trace.jsonl").read_text(encoding="utf-8").splitlines()
    assert len(lines) == 3  # tick1 + tick2 + commit
    summary = (runs[0] / "summary.json").read_text(encoding="utf-8")
    assert '"result": "success"' in summary


# ─── 놓기 geometry/step (open-loop 유지 — 기존 잠금 계승) ─────────────


def test_plan_place_release_height():
    spot = _det(position=(0.25, -0.05, 0.04), height=0.04, grasp_yaw=0.3)
    held = _det(height=0.023)
    pplan = geometry.plan_place(spot, held=held, lateral=0.008)
    assert pplan[0].place[2] == pytest.approx(0.04 + 0.023 / 2 + 0.005)
    assert pplan[0].pre[0] == pytest.approx(pplan[0].place[0])
    assert pplan[0].pre[1] == pytest.approx(pplan[0].place[1])
    assert pplan[0].pre[2] == pytest.approx(pplan[0].place[2] + 0.06 + 0.023 / 2)
    assert len(pplan) == 7 * 4
    assert pplan[0].label == "tilt=+0 yaw=17"
    for deg in (17, 107, 197, 287):
        assert any(f"yaw={deg}" in c.label for c in pplan)


def test_plan_place_free_family_disjoint_yaws():
    spot = _det(position=(0.25, -0.05, 0.04), height=0.04, grasp_yaw=0.3)
    held = _det(height=0.023)
    aligned = geometry.plan_place(spot, held=held, lateral=0.008)
    free = geometry.plan_place_free(spot, held=held, lateral=0.008)
    assert len(free) == 7 * 8
    assert free[0].label == "tilt=+0 yaw=47"
    yaw_of = lambda c: c.label.split("yaw=")[1]  # noqa: E731
    assert {yaw_of(c) for c in aligned} & {yaw_of(c) for c in free} == set()


async def test_plan_place_falls_back_to_reachable_spot():
    high_far = _det(score=0.80, position=(0.15, 0.10, 0.22), base_z=0.20)
    low_near = _det(score=0.73, position=(0.24, -0.11, 0.03), base_z=0.005)
    ctx = _ctx({
        **_search_responses(),
        _DETECT: [
            DetectOrientedResponse(found=True, candidates=[high_far, low_near])
        ],
        _SELECT: [
            ResolveReachableResponse(index=-1, message="선반 위 IK 전멸"),
            ResolveReachableResponse(index=-1, message="선반 위 IK 전멸"),
            _resolve_ok(),
        ],
        _MOVE_J: [MoveJResponse()],
    })
    held = _det(height=0.023)
    chosen, _pre = await steps.plan_place(
        ctx, _BOT, "blue box", held=held, lateral=0.008, home=_home_record()
    )
    assert chosen.place[2] == pytest.approx(0.03 + 0.023 / 2 + 0.005)
    assert len(ctx.calls(_SELECT)) == 3


async def test_plan_place_falls_back_to_free_yaw_family():
    ctx = _ctx({
        **_search_responses(),
        _DETECT: [
            DetectOrientedResponse(found=True, candidates=[_det(grasp_yaw=0.3)])
        ],
        _SELECT: [
            ResolveReachableResponse(index=-1, message="정렬 yaw 전멸"),
            _resolve_ok(),
        ],
        _MOVE_J: [MoveJResponse()],
    })
    chosen, _pre = await steps.plan_place(
        ctx, _BOT, "cube", held=_det(height=0.023), lateral=0.008,
        home=_home_record(),
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
    rt.responses[_LIST_WP] = ListWaypointsResponse(waypoints=[_home_record()])
    rt.responses[_LIST_GROUPS] = ListGroupsResponse(
        groups=[WaypointGroupRecord(id=1, robot_id=_BOT, name="search")]
    )
    rt.responses[_LIST_MEMBERS] = ListGroupMembersResponse(
        waypoints=[
            WaypointRecord(
                id=1, robot_id=_BOT, name="s0",
                joint_values=[0.0] * 6, joint_names=[], created_at=_TS,
            )
        ]
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


async def test_module_control_without_run_says_why():
    mod = PickAndPlaceModule(_WireStub(), {})  # type: ignore[arg-type]
    r = await mod.pause(ControlRequest())
    assert not r.ok and r.message


async def test_module_preview_returns_static_tree_without_wire():
    mod = PickAndPlaceModule(_WireStub(), {})  # type: ignore[arg-type]
    res = await mod.preview(PreviewRequest())
    assert [e.name for e in res.entries if e.depth == 0] == [
        "home_waypoint", "plan_pick", "plan_place", "servo_pick", "execute_place",
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


def test_task_robots_constant_matches_scenario_binding():
    assert PickAndPlaceModule.TASK_ROBOTS == ("so101_6dof_0",)


async def test_list_robots_returns_task_robots():
    mod = PickAndPlaceModule(_WireStub(), {})  # type: ignore[arg-type]
    res = await mod.list_robots(ListRobotsRequest())
    assert res.robot_ids == list(PickAndPlaceModule.TASK_ROBOTS)


# ─── 파지 판정 (물었나/놓쳤나) — 기존 잠금 계승 ───────────────────────


def test_gripper_holding_judgment():
    assert steps._gripper_holding(_HELD_RAW, _SPEC) is True
    assert steps._gripper_holding(_EMPTY_RAW, _SPEC) is False
    thin = _SPEC.gripper_close_raw + 100  # gap 100 < margin 165
    assert steps._gripper_holding(thin, _SPEC) is False


def test_position_only_cannot_catch_false_stall():
    """알려진 한계 박제: 물체 없이 어중간히 stall 하면 위치 판정은 HELD 오판
    (false-positive) — load 병기 로그로 실물 튜닝이 해법."""
    false_stall = _SPEC.gripper_held_threshold_raw + 200
    assert steps._gripper_holding(false_stall, _SPEC) is True


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
