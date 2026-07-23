"""handover task 검증 — mock(FakeContext) 레벨 (2026-07-23 재배선판).

잠그는 것:
  ① frame 변환 왕복 (base_pose 크로스캘 규약 — 정의는 pen.py 로 이동)
  ② 시나리오 happy path 의 호출 경로 — 특히 **수취 순서 불변식**: so101 이
     close + held 판정한 뒤에만 omx 가 연다 (뒤집히면 물체 낙하)
  ③ 명시 실패 클래스 — 짧은 펜 / workcell 미설정 / hand_eye 없음 /
     공중 재검출 실패 (FK 후퇴 금지)
  ④ 수취 계획의 cross-robot 충돌 게이트 — 충돌 그룹 제외 재시도 / 전멸 명시
     실패 + **근접 국면 파라미터** (omx 그리퍼 닫힘 fraction / margin 축소)
  ⑤ look-then-move — refine 채택 시 보정 이동, refine 실패 시 coarse blind
     진행 (침묵 아님 — trace/로그는 별도)
  ⑥ module 배선 (preview 정적 트리 / list_robots)
"""

from __future__ import annotations

import math
from datetime import UTC, datetime

import pytest
from pydantic import BaseModel

from modules.calibration.contract import (
    Calibration,
    CalibrationBundle,
    HandEyeResultData,
    HandEyeResultRecord,
)
from modules.detector.contract import (
    DetectOrientedResponse,
    Detector,
    OrientedDetection,
)
from modules.motion.contract import (
    Motion,
    MoveJResponse,
    MoveLResponse,
    ResolveReachableResponse,
    TcpState,
)
from modules.motor.contract import JointState, Motor, SetGripperResponse
from modules.shared_config.contract import SharedConfig, WorkcellBundle, WorkcellRoi
from modules.tasks.core.contract import PreviewRequest
from modules.tasks.core.errors import (
    DetectionNotFound,
    NoReachableGrasp,
    TaskError,
)
from modules.tasks.core.fake import FakeContext
from modules.tasks.core.spec import TaskRobotSpec
from modules.tasks.handover import pen, steps
from modules.tasks.handover.collision import BasePose
from modules.tasks.handover.contract import ListRobotsRequest
from modules.tasks.handover.module import HandoverModule
from modules.waypoint.contract import (
    ListWaypointsResponse,
    Waypoint,
    WaypointRecord,
)

SO = "so101_6dof_0"
OMX = "omx_f_0"
_TS = datetime.fromtimestamp(0, UTC)

_DETECT_PLANAR = str(Detector.Service.DETECT_PLANAR)
_DETECT_ORIENTED = str(Detector.Service.DETECT_ORIENTED)
_SELECT = str(Motion.Service.RESOLVE_REACHABLE)
_MOVE_J = str(Motion.Service.MOVE_J)
_MOVE_L = str(Motion.Service.MOVE_L)
_GRIP = str(Motor.Service.SET_GRIPPER)
_READ_STATE = str(Motor.Service.READ_STATE)
_TCP_SNAP = str(Motion.Service.TCP_SNAPSHOT)
_LIST_WP = str(Waypoint.Service.LIST)
_WORKCELL = str(SharedConfig.Service.SNAPSHOT_WORKCELL)
_CAL_BUNDLE = str(Calibration.Service.SNAPSHOT_BUNDLE)

_SPEC = TaskRobotSpec(
    gripper_open_raw=3186, gripper_close_raw=1935,
    gripper_index=5, gripper_held_threshold_raw=2100,
)
_SPECS = {SO: _SPEC, OMX: _SPEC}
_BASE_OMX = BasePose(x=0.0342, y=0.2702, z=-0.0094, yaw_rad=math.radians(-3.33))
_HELD_RAW = 2400  # gap > margin → HELD

_ROI_SO = WorkcellRoi(
    x_min=0.13, x_max=0.36, y_min=-0.16, y_max=0.39, z_min=-0.04, z_max=0.22
)
_ROI_OMX = WorkcellRoi(
    x_min=0.08, x_max=0.34, y_min=-0.22, y_max=0.22, z_min=-0.02, z_max=0.25
)
# happy path 의 제시 TCP 점 — 시나리오와 같은 계산으로 유도 (기대값 하드코딩
# 대신 같은 순수 함수: 랑데부 후보 [0] 이 첫 resolve 성공으로 채택된다).
# 실제 H(노출 중심)는 여기서 ~5cm 노출 방향 오프셋 — 매치 반경(8cm) 안이라
# det 를 이 점에 둬도 so_redetect/refine 매치가 성립한다.
_H = pen.rendezvous_candidates(
    _ROI_SO, _ROI_OMX, _BASE_OMX, steps._PRESENT_Z_WORLD,
    limit=steps._PRESENT_LIMIT, prefer_r_so=steps._RENDEZVOUS_R_SO_M,
)[0]


@pytest.fixture(autouse=True)
def _fast(monkeypatch: pytest.MonkeyPatch):
    monkeypatch.setattr(steps, "_GRIPPER_SETTLE_S", 0.0)
    monkeypatch.setattr(steps, "_SEARCH_SETTLE_S", 0.0)
    monkeypatch.setattr(steps, "_OBSERVE_SETTLE_S", 0.0)


@pytest.fixture(autouse=True)
def _trace_to_tmp(monkeypatch: pytest.MonkeyPatch, tmp_path):
    """trace 산출물을 tmp 로 — 테스트가 실 debug/handover/ 를 오염하지 않게
    (detector 덤프 오염 실사고 2026-07-19 와 같은 클래스)."""
    import modules.tasks.handover.trace as tmod

    monkeypatch.setattr(tmod, "_TRACE_ROOT", tmp_path / "handover")


def _wp(robot: str, name: str, rid: int = 1) -> WaypointRecord:
    return WaypointRecord(
        id=rid, robot_id=robot, name=name,
        joint_values=[0.1 * rid] * 6, joint_names=[], created_at=_TS,
    )


def _hand_eye_bundle(robot: str) -> CalibrationBundle:
    return CalibrationBundle(
        robot_id=robot,
        hand_eye=HandEyeResultRecord(
            run_id=1, robot_id=robot, created_at=_TS,
            result_data=HandEyeResultData(
                R_cam2gripper=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                t_cam2gripper=[[0.0], [0.0], [0.0]],
                method="test",
            ),
        ),
    )


def _pen_det(
    position=(0.20, 0.0, 0.0), score=0.9, footprint=(0.14, 0.012), yaw=0.0,
) -> OrientedDetection:
    """omx mono 검출 (omx base frame) — 펜 신뢰 게이트 통과 기본값."""
    return OrientedDetection(
        prompt="pen", position=position, score=score, base_z=position[2],
        height=0.0, grasp_yaw=yaw, footprint=footprint,
        points=[(position[0], position[1], position[2])] * 60,
    )


def _aerial_det(position, score=0.8) -> OrientedDetection:
    """so101 공중 재검출 (world frame)."""
    return OrientedDetection(
        prompt="pen", position=position, score=score, base_z=position[2],
        height=0.01, grasp_yaw=0.3, footprint=(0.09, 0.012),
        points=[(position[0], position[1], position[2])] * 60,
    )


def _joint_state(gripper_raw: int) -> JointState:
    pos = [0] * 6
    pos[_SPEC.gripper_index] = gripper_raw
    return JointState(
        robot_id=SO, seq=0, timestamp_unix=0.0,
        positions_raw=pos, loads_raw=None,
    )


def _tcp(position, joints) -> TcpState:
    return TcpState(
        robot_id=OMX, seq=0, timestamp_unix=0.0, position=position,
        quaternion=(0.0, 0.0, 0.0, 1.0), joint_names=[], joints=list(joints),
    )


class _Rt:
    def __init__(self) -> None:
        self.published: list[tuple[str, BaseModel]] = []

    def publish(self, wire_key: str, event: BaseModel) -> None:
        self.published.append((str(wire_key), event))

    async def call(self, key, req, res_cls, *, robot_id=None, timeout=None):  # noqa: ANN001, ANN201
        raise AssertionError("module runtime 호출 금지 — ctx 로만")


def _module(checker=None) -> HandoverModule:  # noqa: ANN001
    return HandoverModule(
        _Rt(), _SPECS, omx_base_pose=_BASE_OMX, checker=checker
    )  # type: ignore[arg-type]


def _happy_script() -> dict:
    """happy path 스크립트 — place_object="" (수취까지, 적치 생략)."""
    so_wps = ListWaypointsResponse(waypoints=[_wp(SO, "home", 1)])
    omx_wps = ListWaypointsResponse(waypoints=[_wp(OMX, "home", 2)])
    return {
        _LIST_WP: [so_wps, omx_wps],
        _WORKCELL: [WorkcellBundle(robots={SO: _ROI_SO, OMX: _ROI_OMX})],
        _CAL_BUNDLE: [_hand_eye_bundle(OMX), _hand_eye_bundle(SO)],
        # observe(1) + refine(1) — 같은 펜 (refine 채택 → 보정 이동 발생)
        _DETECT_PLANAR: [
            DetectOrientedResponse(found=True, candidates=[_pen_det()]),
            DetectOrientedResponse(found=True, candidates=[_pen_det()]),
        ],
        # so101 재검출(1) + 수취 refine(1) — 제시점 그대로
        _DETECT_ORIENTED: [
            DetectOrientedResponse(found=True, candidates=[_aerial_det(_H)]),
            DetectOrientedResponse(found=True, candidates=[_aerial_det(_H)]),
        ],
        _SELECT: [
            # omx 관측 자세 (ψ 격자 중 첫 그룹)
            ResolveReachableResponse(index=0, solutions=[[0.1] * 5]),
            # omx pick [pre, grasp, lift]
            ResolveReachableResponse(
                index=0, solutions=[[0.2] * 5, [0.25] * 5, [0.3] * 5]
            ),
            # omx 제시 자세
            ResolveReachableResponse(index=0, solutions=[[0.4] * 5]),
            # so101 수취 관측 자세
            ResolveReachableResponse(index=0, solutions=[[0.5] * 6]),
            # so101 수취 [pre, grasp]
            ResolveReachableResponse(index=0, solutions=[[0.6] * 6, [0.65] * 6]),
        ],
        # so home / omx home / omx observe / omx pick pre / omx present /
        # so observe / so pre / omx retreat home / so 종료 home = 9
        _MOVE_J: [MoveJResponse()] * 9,
        # refine XY 보정 / blind 하강 / lift / 수취 진입 / withdraw = 5
        _MOVE_L: [MoveLResponse()] * 5,
        # omx open / omx close / so open / so close / omx release open = 5
        _GRIP: [SetGripperResponse()] * 5,
        # omx close후 / omx lift후 / 제시 도달 / so close후 / so 이탈후 = 5
        _READ_STATE: [_joint_state(_HELD_RAW)] * 5,
        # 제시 계획(omx) / 수취 계획(omx) / retreat(so, omx) = 4
        _TCP_SNAP: [
            _tcp((0.25, 0.0, 0.10), [0.3] * 5),
            _tcp((0.25, 0.0, 0.12), [0.4] * 5),
            _tcp((0.2, 0.1, 0.1), [0.6] * 6),
            _tcp((0.25, 0.0, 0.12), [0.4] * 5),
        ],
    }


def _ctx(script: dict) -> FakeContext:
    return FakeContext(robots=[SO, OMX], specs=_SPECS, service_script=script)


# ─── ① frame 변환 ────────────────────────────────────────────────────


def test_base_pose_transform_roundtrip():
    p_world = (0.21, -0.09, 0.05)
    p_omx = steps.world_to_robot(p_world, _BASE_OMX)
    back = steps.robot_to_world(p_omx, _BASE_OMX)
    assert back == pytest.approx(p_world, abs=1e-12)
    # 회전 방향 sanity: omx base 는 world (0.034, 0.270) — omx 원점의 world 좌표
    assert steps.robot_to_world((0.0, 0.0, 0.0), _BASE_OMX) == pytest.approx(
        (0.0342, 0.2702, -0.0094)
    )


# ─── ② happy path + 수취 순서 불변식 ─────────────────────────────────


async def test_scenario_happy_path_and_release_order():
    ctx = _ctx(_happy_script())
    await _module().scenario(ctx, pick_object="pen")
    log = ctx.wire.call_log
    grip_events = [
        (i, c["robot_id"], c["req"].position_raw)
        for i, c in enumerate(log) if c["key"] == _GRIP
    ]
    # 순서: omx open(준비) → omx close(집기) → so open(수취 준비) →
    #       so close(수취) → omx open(release)
    robots = [(r, raw == _SPEC.gripper_open_raw) for _, r, raw in grip_events]
    assert robots == [
        (OMX, True), (OMX, False), (SO, True), (SO, False), (OMX, True)
    ], grip_events
    so_close_i = grip_events[3][0]
    omx_release_i = grip_events[4][0]
    # so close 와 omx release 사이에 so101 held 판정(READ_STATE)이 있어야 한다
    between = [
        c for c in log[so_close_i:omx_release_i]
        if c["key"] == _READ_STATE and c["robot_id"] == SO
    ]
    assert between, "so101 held 판정 전에 omx 가 열림 — 낙하 위험 순서 위반"
    # robot-scoped 라우팅: 양쪽 robot 모두 명령이 갔는지 (참여 명부 검증 경유)
    assert {c["robot_id"] for c in ctx.calls(_MOVE_L)} == {SO, OMX}
    # 검출 채널: omx = DETECT_PLANAR(mono) ×2, so101 = DETECT_ORIENTED ×2
    assert len(ctx.calls(_DETECT_PLANAR)) == 2
    assert all(
        c["req"].robot_id == OMX for c in ctx.calls(_DETECT_PLANAR)
    )
    assert len(ctx.calls(_DETECT_ORIENTED)) == 2
    assert all(
        c["req"].robot_id == SO for c in ctx.calls(_DETECT_ORIENTED)
    )
    # 접촉 인접 이동 감속 — blind 하강/수취 진입/withdraw 가 gentle
    gentle = [
        c for c in ctx.calls(_MOVE_L)
        if c["req"].speed_scale == steps._GENTLE_SPEED_SCALE
    ]
    assert len(gentle) >= 3, [c["req"].speed_scale for c in ctx.calls(_MOVE_L)]


async def test_scenario_refine_miss_proceeds_with_coarse():
    """look-then-move 의 재관측 실패 = coarse 로 blind 진행 (omx best-effort —
    so101 이 흡수). refine 보정 MoveL 1건이 줄어든다 (침묵이 아니라 로그/trace
    는 남는다 — 여기선 경로만 잠금)."""
    script = _happy_script()
    script[_DETECT_PLANAR] = [
        DetectOrientedResponse(found=True, candidates=[_pen_det()]),
        DetectOrientedResponse(found=False, candidates=[]),  # refine 미검출
    ]
    ctx = _ctx(script)
    await _module().scenario(ctx, pick_object="pen")
    assert len(ctx.calls(_MOVE_L)) == 4  # 보정 이동 1건 감소 (하강/lift/진입/이탈)


# ─── ③ 명시 실패 클래스 ──────────────────────────────────────────────


def test_short_pen_fails_explicitly():
    det = _pen_det(footprint=(0.07, 0.010))  # 7cm — 노출 부족
    with pytest.raises(TaskError, match="짧아"):
        steps.plan_pen_grasp_from(det, _BASE_OMX)


async def test_missing_workcell_fails_before_motion():
    script = _happy_script()
    script[_WORKCELL] = [WorkcellBundle(robots={SO: _ROI_SO})]  # omx 미설정
    ctx = _ctx(script)
    with pytest.raises(TaskError, match="workcell"):
        await _module().scenario(ctx, pick_object="pen")
    assert ctx.calls(_MOVE_J) == []  # 모션 0 시점 실패


async def test_missing_hand_eye_fails_before_motion():
    script = _happy_script()
    script[_CAL_BUNDLE] = [CalibrationBundle(robot_id=OMX)]  # hand_eye 없음
    ctx = _ctx(script)
    with pytest.raises(TaskError, match="hand_eye"):
        await _module().scenario(ctx, pick_object="pen")
    assert ctx.calls(_MOVE_J) == []


async def test_aerial_redetect_failure_is_explicit_no_fk_fallback():
    """공중 재검출 실패 = 명시 실패 — FK 짐작으로 후퇴하지 않는다 (§8-4).
    (v1 이 갈아엎은 미검증 코드가 바로 FK 기반 plan_receive 였다.)"""
    ctx = _ctx({
        _MOVE_J: [MoveJResponse()],
        _DETECT_ORIENTED: [DetectOrientedResponse(found=False, candidates=[])],
    })
    with pytest.raises(DetectionNotFound, match="재검출"):
        await steps.so_redetect(ctx, SO, "pen", [0.5] * 6, _H)
    # 실패 후 추가 모션/TCP 조회 없음 (FK 폴백 경로 부재)
    assert len(ctx.calls(_MOVE_J)) == 1
    assert ctx.calls(_TCP_SNAP) == []


async def test_pen_gate_rejects_untrusted_candidates():
    """관측 신뢰 게이트 — score/길이/폭 컷 미달은 명시 실패."""
    bad = [
        _pen_det(score=0.2),  # score 미달
        _pen_det(footprint=(0.02, 0.01)),  # 너무 짧음 (펜 아님)
        _pen_det(footprint=(0.14, 0.06)),  # 폭 초과
    ]
    ctx = _ctx({
        _MOVE_J: [MoveJResponse()],
        _DETECT_PLANAR: [DetectOrientedResponse(found=True, candidates=bad)],
    })
    with pytest.raises(DetectionNotFound, match="신뢰 컷"):
        await steps.omx_observe_detect(ctx, OMX, "pen", [0.1] * 5)


# ─── ④ 수취 충돌 게이트 (근접 국면 파라미터) ─────────────────────────


class _FakeChecker:
    margin_m = 0.02

    def __init__(self, hits: list[bool]) -> None:
        self.hits = hits
        self.calls: list[dict] = []

    def path_in_collision(
        self, path, joints_b, *, grip_a=1.0, grip_b=1.0, margin_m=None
    ) -> bool:  # noqa: ANN001
        self.calls.append({
            "grip_a": grip_a, "grip_b": grip_b, "margin_m": margin_m,
        })
        return self.hits.pop(0)

    def in_collision(self, ja, jb, **kw) -> bool:  # noqa: ANN001, ANN003
        return False


def _receive_script(n_resolve: int) -> dict:
    return {
        _TCP_SNAP: [_tcp((0.25, 0.0, 0.12), [0.4] * 5)],
        _SELECT: [
            ResolveReachableResponse(index=0, solutions=[[0.1] * 6, [0.2] * 6]),
        ] * n_resolve,
    }


async def test_plan_receive_retries_past_colliding_group():
    checker = _FakeChecker(hits=[True, False])
    ctx = _ctx(_receive_script(2))
    plan = await steps.plan_receive(
        ctx, SO, OMX, _aerial_det(_H), _BASE_OMX, checker  # type: ignore[arg-type]
    )
    assert len(checker.calls) == 2  # 1차 충돌 → 그룹 제외 재-resolve → 2차 통과
    assert len(ctx.calls(_SELECT)) == 2
    assert plan.sols[0] == [0.1] * 6
    # 근접 국면 파라미터 — omx 는 펜 든 상태(거의 닫힘), margin 축소 (정밀화 ③)
    assert checker.calls[0]["grip_b"] == steps._OMX_HOLD_GRIP_FRAC
    assert checker.calls[0]["margin_m"] == steps._RECV_COLLISION_MARGIN_M


async def test_plan_receive_all_colliding_fails_explicitly():
    checker = _FakeChecker(hits=[True, True, True])
    ctx = _ctx(_receive_script(3))
    with pytest.raises(NoReachableGrasp, match="충돌"):
        await steps.plan_receive(
            ctx, SO, OMX, _aerial_det(_H), _BASE_OMX, checker  # type: ignore[arg-type]
        )


# ─── ⑤ pen 기하 (순수) ───────────────────────────────────────────────


def test_pen_grasp_bites_far_end_and_exposes_toward_so101():
    det = _pen_det()  # 중심 (0.20, 0), yaw 0, 길이 0.14 → 끝점 (0.13,0)/(0.27,0)
    g = steps.plan_pen_grasp_from(det, _BASE_OMX)
    # so101 원점은 omx frame 에서 (−0.018, −0.272) 근방 → 먼 끝 = (0.27, 0)
    assert g.tip_far == pytest.approx((0.27, 0.0), abs=1e-9)
    assert g.tip_near == pytest.approx((0.13, 0.0), abs=1e-9)
    # 파지점 = 먼 끝에서 30% = 0.27 − 0.042
    assert g.grasp_xy == pytest.approx((0.228, 0.0), abs=1e-9)
    assert g.u == pytest.approx((-1.0, 0.0), abs=1e-9)
    # 노출 = 0.14 − 0.042 − 0.01 = 0.088
    assert g.exposed_len_m == pytest.approx(0.088, abs=1e-9)


def test_rendezvous_candidates_inside_both_rois():
    cands = pen.rendezvous_candidates(
        _ROI_SO, _ROI_OMX, _BASE_OMX, (0.12,), limit=100
    )
    assert cands, "공통 워크스페이스가 비어 있으면 안 됨 (설정 ROI 기준)"
    for x, y, z in cands:
        assert _ROI_SO.x_min <= x <= _ROI_SO.x_max
        assert _ROI_SO.y_min <= y <= _ROI_SO.y_max
        px, py, pz = pen.world_to_robot((x, y, z), _BASE_OMX)
        assert _ROI_OMX.x_min <= px <= _ROI_OMX.x_max
        assert _ROI_OMX.y_min <= py <= _ROI_OMX.y_max
        assert _ROI_OMX.z_min <= pz <= _ROI_OMX.z_max


def test_rendezvous_empty_when_no_overlap():
    far = WorkcellRoi(
        x_min=5.0, x_max=5.2, y_min=5.0, y_max=5.2, z_min=0.0, z_max=0.3
    )
    assert pen.rendezvous_candidates(_ROI_SO, far, _BASE_OMX, (0.12,)) == []


# ─── ⑥ module 배선 ───────────────────────────────────────────────────


async def test_module_list_robots_and_preview():
    mod = _module()
    robots = await mod.list_robots(ListRobotsRequest())
    assert robots.robot_ids == [SO, OMX]
    res = await mod.preview(PreviewRequest())
    top = [e.name for e in res.entries if e.depth == 0]
    # 시나리오 골격 잠금 — 구조를 바꾸면 이 목록도 같이 (계약 잠금, PnP 동형)
    assert top == [
        "named_waypoint", "named_waypoint", "load_workcells",
        "load_hand_eye", "load_hand_eye",
        "go_home", "go_home", "set_gripper",
        "plan_omx_observe", "omx_observe_detect",
        "plan_omx_pick_pen", "omx_pick_pen",
        "plan_omx_present", "omx_present",
        "plan_so_observe", "so_redetect", "plan_receive",
        "set_gripper", "receive", "omx_retreat",
        "place_into", "go_home",
    ]
