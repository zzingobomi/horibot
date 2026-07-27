"""handover task 검증 — mock(FakeContext) 레벨 (2026-07-27 큐브→봉 전환판).

잠그는 것:
  ① frame 변환 왕복 (base_pose 크로스캘 규약 — 정의는 frames.py)
  ② 시나리오 happy path 의 호출 경로 — 특히 **수취 순서 불변식**: so101 이
     close + held 판정한 뒤에만 omx 가 연다 (뒤집히면 물체 낙하)
  ③ 명시 실패 클래스 — 봉 신뢰 게이트 / workcell 미설정 / hand_eye 없음 /
     공중 재검출 실패 (FK 후퇴 금지) / 짧은 봉 (침묵 진행 금지)
  ④ 수취 계획의 cross-robot 충돌 게이트 — 충돌 그룹 제외 재시도 / 전멸 명시
     실패 + **근접 국면 파라미터** (omx 그리퍼 닫힘 fraction / margin 축소)
  ⑤ 봉 기하 (순수) — 양 끝 파지 후보 / 노출 오프셋 / B/down 제시 quat /
     수취 수직 조축족 (handover_block_probe 결론 회귀 잠금)
  ⑥ module 배선 (preview 정적 트리 / list_robots)
"""

from __future__ import annotations

import math
from datetime import UTC, datetime

import numpy as np
import pytest
from pydantic import BaseModel
from scipy.spatial.transform import Rotation

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
from modules.motor.contract import (
    JointState,
    Motor,
    SetGripperResponse,
    SetTorqueResponse,
)
from modules.shared_config.contract import SharedConfig, WorkcellBundle, WorkcellRoi
from modules.tasks.core.contract import PreviewRequest
from modules.tasks.core.errors import (
    DetectionNotFound,
    GraspFailed,
    NoReachableGrasp,
    TaskError,
)
from modules.tasks.core.fake import FakeContext
from modules.tasks.core.spec import TaskRobotSpec
from modules.tasks.handover import block, frames, steps
from modules.tasks.handover.collision import BasePose
from modules.tasks.handover.contract import ListRobotsRequest
from modules.tasks.handover.module import HandoverModule
from modules.waypoint.contract import (
    GetWaypointByNameResponse,
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
_SET_TORQUE = str(Motor.Service.SET_TORQUE)
_READ_STATE = str(Motor.Service.READ_STATE)
_TCP_SNAP = str(Motion.Service.TCP_SNAPSHOT)
_GET_WP_BY_NAME = str(Waypoint.Service.GET_WAYPOINT_BY_NAME)
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
    x_min=0.13, x_max=0.36, y_min=-0.16, y_max=0.39, z_min=-0.04, z_max=0.34
)
_ROI_OMX = WorkcellRoi(
    x_min=0.08, x_max=0.34, y_min=-0.22, y_max=0.22, z_min=-0.02, z_max=0.34
)
# happy path 의 봉 기하/제시점 — 시나리오와 같은 순수 계산으로 유도 (기대값
# 하드코딩 대신 같은 함수). 랑데부 후보 [0](TCP)이 첫 resolve 성공으로 채택되고,
# H(so101 재검출 겨냥점) = E = TCP − (0,0,tcp_to_e) (B/down 수직 제시).
_GEOM = block.plan_block_grasp(
    (0.20, 0.0), 0.0, (0.080, 0.020),
    grasp_frac=steps._BLOCK_GRASP_FRAC,
    jaw_along_m=steps._OMX_JAW_ALONG_M,
    exposed_frac=steps._BLOCK_EXPOSED_FRAC,
    min_exposed_m=steps._SO_MIN_GRASP_M + steps._EXPOSED_MARGIN_M,
    len_min_m=steps._BLOCK_LEN_MIN_M,
    len_max_m=steps._BLOCK_LEN_MAX_M,
)
_TCP_W = frames.rendezvous_candidates(
    _ROI_SO, _ROI_OMX, _BASE_OMX, steps._PRESENT_Z_WORLD,
    limit=steps._PRESENT_LIMIT, prefer_point=steps._RENDEZVOUS_PREFER_XY,
)[0]
_H = (_TCP_W[0], _TCP_W[1], _TCP_W[2] - _GEOM.tcp_to_e_m)


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


def _block_det(
    position=(0.20, 0.0, 0.0), score=0.9, footprint=(0.080, 0.020), yaw=0.0,
) -> OrientedDetection:
    """omx mono 검출 (omx base frame) — 봉 신뢰 게이트 통과 기본값 (8×2cm)."""
    return OrientedDetection(
        prompt="orange block", position=position, score=score, base_z=position[2],
        height=0.0, grasp_yaw=yaw, footprint=footprint,
        points=[(position[0], position[1], position[2])] * 60,
    )


def _aerial_det(position, score=0.8) -> OrientedDetection:
    """so101 공중 재검출 (world frame) — 수직 봉의 보이는 노출부 (평면 투영
    footprint 는 ~단면 크기)."""
    return OrientedDetection(
        prompt="orange block", position=position, score=score, base_z=position[2],
        height=0.05, grasp_yaw=0.3, footprint=(0.022, 0.020),
        points=[(position[0], position[1], position[2])] * 60,
    )


def _joint_state(gripper_raw: int) -> JointState:
    pos = [0] * 6
    pos[_SPEC.gripper_index] = gripper_raw
    return JointState(
        robot_id=SO, seq=0, timestamp_unix=0.0,
        positions_raw=pos, loads_raw=None,
    )


def _tcp(position, joints, quat=(0.0, 0.0, 0.0, 1.0)) -> TcpState:
    return TcpState(
        robot_id=OMX, seq=0, timestamp_unix=0.0, position=position,
        quaternion=quat, joint_names=[], joints=list(joints),
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
    so_home = GetWaypointByNameResponse(waypoint=_wp(SO, "home", 1))
    omx_home = GetWaypointByNameResponse(waypoint=_wp(OMX, "home", 2))
    return {
        _GET_WP_BY_NAME: [so_home, omx_home],
        _WORKCELL: [WorkcellBundle(robots={SO: _ROI_SO, OMX: _ROI_OMX})],
        _CAL_BUNDLE: [_hand_eye_bundle(OMX), _hand_eye_bundle(SO)],
        # omx 관측 검출 1건 (refine=look-then-move 폐기)
        _DETECT_PLANAR: [
            DetectOrientedResponse(found=True, candidates=[_block_det()]),
        ],
        # so101 재검출(1) + 수취 refine(1) — 제시점(E) 그대로
        _DETECT_ORIENTED: [
            DetectOrientedResponse(found=True, candidates=[_aerial_det(_H)]),
            DetectOrientedResponse(found=True, candidates=[_aerial_det(_H)]),
        ],
        _SELECT: [
            # omx 관측 자세 (ψ 격자 중 첫 그룹)
            ResolveReachableResponse(index=0, solutions=[[0.1] * 5]),
            # omx pick — 봉 끝 파지점 (양 끝 × z 사다리 중 첫 그룹, J5 자연해)
            ResolveReachableResponse(index=0, solutions=[[0.2] * 5]),
            # omx 제시 자세 (hang(z↑) 단일 — 랑데부 후보 [0] 채택)
            ResolveReachableResponse(index=0, solutions=[[0.4] * 5]),
            # so101 수취 관측 자세
            ResolveReachableResponse(index=0, solutions=[[0.5] * 6]),
            # so101 수취 [pre, grasp]
            ResolveReachableResponse(index=0, solutions=[[0.6] * 6, [0.65] * 6]),
        ],
        _SET_TORQUE: [SetTorqueResponse(ok=True)] * 2,  # so101 / omx enable
        # so home / omx home / omx observe / omx pick / omx present /
        # so observe / so pre / omx retreat home / so 종료 home = 9
        _MOVE_J: [MoveJResponse()] * 9,
        # 수취 진입 / withdraw = 2 (omx pick 은 move_j 스윙인 — MoveL 없음)
        _MOVE_L: [MoveLResponse()] * 2,
        # omx open / omx close / so open / so close / omx release open = 5
        _GRIP: [SetGripperResponse()] * 5,
        # omx close후 / 제시 도달 / so close후 / so 이탈후 = 4
        _READ_STATE: [_joint_state(_HELD_RAW)] * 4,
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
    await _module().scenario(ctx, pick_object="orange block")
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
    # omx pick 이 move_j 스윙인이라 MoveL 은 so101 수취만
    assert {c["robot_id"] for c in ctx.calls(_MOVE_L)} == {SO}
    # robot-scoped 라우팅: 양쪽 robot 모두 명령이 갔는지 (move_j 로 검증)
    assert {c["robot_id"] for c in ctx.calls(_MOVE_J)} == {SO, OMX}
    # 검출 채널: omx = DETECT_PLANAR(mono) ×1, so101 = DETECT_ORIENTED ×2
    assert len(ctx.calls(_DETECT_PLANAR)) == 1
    assert all(
        c["req"].robot_id == OMX for c in ctx.calls(_DETECT_PLANAR)
    )
    assert len(ctx.calls(_DETECT_ORIENTED)) == 2
    assert all(
        c["req"].robot_id == SO for c in ctx.calls(_DETECT_ORIENTED)
    )
    # 접촉 인접 이동 감속 — 수취 진입/withdraw 가 gentle
    gentle = [
        c for c in ctx.calls(_MOVE_L)
        if c["req"].speed_scale == steps._GENTLE_SPEED_SCALE
    ]
    assert len(gentle) == 2, [c["req"].speed_scale for c in ctx.calls(_MOVE_L)]


async def test_present_is_hang_z_up_and_h_below_tcp():
    """hang(z↑) 제시 잠금 — 채택 quat 의 tool z 가 world ↑ (pick 이 tool z ∥
    −u 로 물었으므로 노출부는 아래로 매달림, J5=0 손목 중립 — 2026-07-27
    케이블 감김 수정), H(재검출 겨냥점)는 제시 TCP 의 tcp_to_e 아래."""
    ctx = _ctx(_happy_script())
    script_pick = steps.BlockPick(
        sols=[[0.2] * 5], quat=(0.0, 0.0, 0.0, 1.0),
        grasp_omx=(0.2, 0.0, 0.016), u_omx=(1.0, 0.0),
        geom=_GEOM, chosen_dz=0.016,
    )
    plan = await steps.plan_omx_present(
        ctx, OMX, _ROI_SO, _ROI_OMX, _BASE_OMX, script_pick,
        [0.1] * 6, None,
    )
    # tool z (omx frame) ↑ — base yaw 는 z 축 회전이라 world 에서도 ↑
    tool_z = Rotation.from_quat(plan.quat).apply([0.0, 0.0, 1.0])
    assert tool_z[2] == pytest.approx(1.0, abs=1e-9)
    assert plan.h_world[2] == pytest.approx(
        _TCP_W[2] - _GEOM.tcp_to_e_m, abs=1e-9
    )
    assert (plan.h_world[0], plan.h_world[1]) == pytest.approx(
        (_TCP_W[0], _TCP_W[1]), abs=1e-9
    )


# ─── ③ 명시 실패 클래스 ──────────────────────────────────────────────


async def test_missing_workcell_fails_before_motion():
    script = _happy_script()
    script[_WORKCELL] = [WorkcellBundle(robots={SO: _ROI_SO})]  # omx 미설정
    ctx = _ctx(script)
    with pytest.raises(TaskError, match="workcell"):
        await _module().scenario(ctx, pick_object="orange block")
    assert ctx.calls(_MOVE_J) == []  # 모션 0 시점 실패


async def test_missing_hand_eye_fails_before_motion():
    script = _happy_script()
    script[_CAL_BUNDLE] = [CalibrationBundle(robot_id=OMX)]  # hand_eye 없음
    ctx = _ctx(script)
    with pytest.raises(TaskError, match="hand_eye"):
        await _module().scenario(ctx, pick_object="orange block")
    assert ctx.calls(_MOVE_J) == []


async def test_aerial_redetect_failure_is_explicit_no_fk_fallback():
    """공중 재검출 실패 = 명시 실패 — FK 짐작으로 후퇴하지 않는다 (§8-4).
    (v1 이 갈아엎은 미검증 코드가 바로 FK 기반 plan_receive 였다.)"""
    ctx = _ctx({
        _MOVE_J: [MoveJResponse()],
        _DETECT_ORIENTED: [DetectOrientedResponse(found=False, candidates=[])],
    })
    with pytest.raises(DetectionNotFound, match="재검출"):
        await steps.so_redetect(ctx, SO, "orange block", [0.5] * 6, _H)
    # 실패 후 추가 모션/TCP 조회 없음 (FK 폴백 경로 부재)
    assert len(ctx.calls(_MOVE_J)) == 1
    assert ctx.calls(_TCP_SNAP) == []


async def test_block_gate_rejects_untrusted_candidates():
    """관측 신뢰 게이트 — score/긴 변 대역/종횡비 하한 컷 미달은 명시 실패."""
    bad = [
        _block_det(score=0.2),  # score 미달
        _block_det(footprint=(0.020, 0.020)),  # 정사각 (큐브류 — 길이/종횡비 미달)
        _block_det(footprint=(0.20, 0.020)),  # 너무 김 (긴 변 상한 초과)
        _block_det(footprint=(0.080, 0.050)),  # 짧은 변 초과 (봉 단면 아님)
    ]
    ctx = _ctx({
        _MOVE_J: [MoveJResponse()],
        _DETECT_PLANAR: [DetectOrientedResponse(found=True, candidates=bad)],
    })
    with pytest.raises(DetectionNotFound, match="신뢰 컷"):
        await steps.omx_observe_detect(ctx, OMX, "orange block", [0.1] * 5)


def test_short_block_fails_explicitly():
    """짧은 봉 = 계획 단계 명시 실패 (노출 부족 침묵 진행 금지 — 사유에 수치)."""
    with pytest.raises(TaskError, match="짧아"):
        block.plan_block_grasp(
            (0.2, 0.0), 0.0, (0.055, 0.020),
            grasp_frac=0.3, jaw_along_m=0.02, exposed_frac=0.65,
            min_exposed_m=0.035, len_min_m=0.05, len_max_m=0.12,
        )


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

    def min_link_world_x(self, side, joints, *, grip=1.0) -> float:  # noqa: ANN001
        return 0.0  # 벽 침범 없음 (게이트 배선만 검증, 벽은 sim feasibility 몫)


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
    # 근접 국면 파라미터 — omx 는 봉 든 상태(거의 닫힘), margin 축소 (정밀화 ③)
    assert checker.calls[0]["grip_b"] == steps._OMX_HOLD_GRIP_FRAC
    assert checker.calls[0]["margin_m"] == steps._RECV_COLLISION_MARGIN_M


async def test_plan_receive_all_colliding_fails_explicitly():
    n = steps._RECV_COLLISION_RETRY
    checker = _FakeChecker(hits=[True] * n)
    ctx = _ctx(_receive_script(n))
    with pytest.raises(NoReachableGrasp, match="충돌"):
        await steps.plan_receive(
            ctx, SO, OMX, _aerial_det(_H), _BASE_OMX, checker  # type: ignore[arg-type]
        )


# ─── ⑤ 봉 기하 (순수) ────────────────────────────────────────────────


def test_plan_block_grasp_ends_and_offsets():
    g = _GEOM  # (0.20, 0.0), yaw 0, footprint (0.080, 0.020)
    assert g.length_m == pytest.approx(0.080)
    assert len(g.ends) == 2  # 양 끝 동등 후보 (축대칭)
    # 끝점 = 중심 ± 4cm, 파지점 = 끝에서 frac(20% = 1.6cm) 안쪽
    (g1, u1), (g2, u2) = g.ends
    assert g1 == pytest.approx((0.20 - 0.04 + 0.016, 0.0), abs=1e-12)
    assert u1 == pytest.approx((1.0, 0.0), abs=1e-12)  # 노출 = +x (반대 끝)
    assert g2 == pytest.approx((0.20 + 0.04 - 0.016, 0.0), abs=1e-12)
    assert u2 == pytest.approx((-1.0, 0.0), abs=1e-12)
    # 노출 = 8 − 1.6 − 1(조 절반) = 5.4cm / E 오프셋 = 1 + 0.65·5.4 = 4.51cm
    assert g.exposed_len_m == pytest.approx(0.054)
    assert g.tcp_to_e_m == pytest.approx(0.01 + 0.65 * 0.054)
    assert g.below_e_m == pytest.approx(0.35 * 0.054)


def test_plan_block_grasp_from_anchors_known_length():
    """2026-07-27 실물 회귀 — mono 검출 길이가 mask 번짐+측면 유입으로 과대
    (109mm, 실물 80)여도 파지/노출 기하는 known 길이(_BLOCK_LEN_M) 앵커.
    옛 코드는 검출 footprint 를 그대로 써 "검출 tip 에서 20%" 파지점이 실물
    끝 ~7mm 지점으로 밀림 = 헛집음 (E 겨냥점도 봉 끝 ~4mm 로 오염)."""
    det = _block_det(
        position=(0.171, 0.029, 0.02), footprint=(0.109, 0.030), yaw=0.0,
    )
    g = steps.plan_block_grasp_from(det, _BASE_OMX)
    assert g.length_m == pytest.approx(steps._BLOCK_LEN_M)  # 검출 109mm 무시
    # 파지점 = 검출 center ∓ (반길이 − frac·길이) = ∓(4 − 1.6)cm = ∓2.4cm
    (g1, _u1), (g2, _u2) = g.ends
    assert g1[0] == pytest.approx(0.171 - 0.024)
    assert g2[0] == pytest.approx(0.171 + 0.024)
    # E 오프셋도 known 기하 (실물 런에선 부푼 길이로 60mm → 봉 끝 지점)
    assert g.tcp_to_e_m == pytest.approx(0.01 + 0.65 * 0.054)


async def test_verify_grasp_counts_negative_load_magnitude():
    """Dynamixel Present_Load 는 2B signed (부호=방향 — 2026-07-27 omx 실물
    닫힘 스톨 −499). gap 이 margin 아래여도 |load| 로 물림 인정 (abs 회귀)."""
    pos = [0] * 6
    pos[_SPEC.gripper_index] = _SPEC.gripper_close_raw  # gap 0
    loads = [0] * 6
    loads[_SPEC.gripper_index] = -499
    st = JointState(
        robot_id=OMX, seq=0, timestamp_unix=0.0,
        positions_raw=pos, loads_raw=loads,
    )
    ctx = _ctx({_READ_STATE: [st]})
    await steps.verify_grasp(ctx, OMX, phase="test")  # raise 없음 = HELD


async def test_verify_grasp_empty_close_raises():
    """빈손 close (gap 0 + 저부하) = GraspFailed — 2026-07-27 실물 false-HELD
    (omx limit_min 이 물리 스톨 너머라 gap 177 로 오판)의 판정측 잠금: close
    ref 도달 + 부하 낮음이면 반드시 EMPTY."""
    pos = [0] * 6
    pos[_SPEC.gripper_index] = _SPEC.gripper_close_raw
    loads = [0] * 6
    loads[_SPEC.gripper_index] = 5
    st = JointState(
        robot_id=OMX, seq=0, timestamp_unix=0.0,
        positions_raw=pos, loads_raw=loads,
    )
    ctx = _ctx({_READ_STATE: [st]})
    with pytest.raises(GraspFailed):
        await steps.verify_grasp(ctx, OMX, phase="test")


def test_recv_orients_vertical_axis_and_approach_preference():
    """수취 자세족 잠금 — 전부 수직 조축(tool z ∥ 봉 축 = 수직) + 수평 접근,
    첫 후보의 접근이 base→E 방위 (so101 쪽 진입 선호). probe 2026-07-27 +
    큐브 시대 실측("도달해는 전부 수직 조축") 회귀 잠금."""
    e = (0.21, 0.16, 0.255)
    orients = steps._recv_orients(e)
    assert orients
    for _label, q, a in orients:
        tool_z = Rotation.from_quat(q).apply([0.0, 0.0, 1.0])
        assert abs(tool_z[2]) == pytest.approx(1.0, abs=1e-9)  # 봉 축 정렬
        assert abs(a[2]) < 1e-9  # 수평 접근
        # 조축(tool y)은 수평 — 봉 단면을 가로질러 문다
        tool_y = Rotation.from_quat(q).apply([0.0, 1.0, 0.0])
        assert abs(tool_y[2]) < 1e-9
    az_pref = math.atan2(e[1], e[0])
    a0 = orients[0][2]
    assert np.dot(a0, [math.cos(az_pref), math.sin(az_pref), 0.0]) == \
        pytest.approx(1.0, abs=1e-9)


def test_present_quat_hang_is_wrist_neutral():
    """hang(z↑) 제시 quat — tool z ↑ + tool x 가 팔 평면(방위 α) 수평 radial
    = 정확히 Rz(α) (ZYYYX 다양체에서 θ=0, **J5=0 손목 중립** — 옛 B/down 의
    J5=±180 케이블 감김 수정 회귀 잠금, 2026-07-27)."""
    alpha = math.radians(25.0)
    q = steps._present_quat_hang(alpha)
    r = Rotation.from_quat(q)
    assert r.apply([0.0, 0.0, 1.0]) == pytest.approx([0.0, 0.0, 1.0], abs=1e-9)
    assert r.apply([1.0, 0.0, 0.0]) == pytest.approx(
        [math.cos(alpha), math.sin(alpha), 0.0], abs=1e-9
    )
    # R == Rz(α) — 잉여 회전 0 (J5=0/θ=0 의 행렬 표현)
    expect = Rotation.from_euler("z", alpha)
    assert (r.inv() * expect).magnitude() == pytest.approx(0.0, abs=1e-9)


async def test_pick_prefers_natural_wrist_far_end():
    """pick 규약 회귀 (2026-07-27 케이블 감김 수정) — ends 는 자연손목 끝
    (노출 u 가 base 쪽 = dot(u,g)<0 = 먼 끝) 우선 정렬, quat 은 tool z ∥ −u."""
    ctx = _ctx({
        _SELECT: [ResolveReachableResponse(index=0, solutions=[[0.2] * 5])],
    })
    pick = await steps.plan_omx_pick_block(ctx, OMX, _GEOM)
    # _GEOM 중심 (0.20, 0) 봉 ∥ x — 먼 끝(0.224)이 채택, 노출은 base 쪽(−x)
    assert pick.u_omx[0] == pytest.approx(-1.0)
    assert pick.grasp_omx[0] == pytest.approx(0.20 + 0.04 - 0.016)
    tz = Rotation.from_quat(pick.quat).apply([0.0, 0.0, 1.0])
    assert tz == pytest.approx([1.0, 0.0, 0.0], abs=1e-9)  # tool z ∥ −u


async def test_pick_wrist_flip_gate_rejects_and_retries():
    """손목 뒤집힘(|J5|>90°) 채택안은 기각하고 그 그룹을 빼고 재-resolve —
    케이블 안전 불변식 (2026-07-27 offline probe: 가까운 끝이 J5≈−173° 뒤집힌
    해로 도달해 첫 그룹으로 채택될 뻔한 구멍)."""
    ctx = _ctx({
        _SELECT: [
            ResolveReachableResponse(index=0, solutions=[[0.1, 0.1, 0.1, 0.1, 3.0]]),
            ResolveReachableResponse(index=0, solutions=[[0.1] * 5]),
        ],
    })
    pick = await steps.plan_omx_pick_block(ctx, OMX, _GEOM)
    assert len(ctx.calls(_SELECT)) == 2  # 기각 → 재-resolve
    assert abs(pick.sols[0][-1]) <= math.radians(90.0)


def test_rendezvous_candidates_inside_both_rois():
    cands = frames.rendezvous_candidates(
        _ROI_SO, _ROI_OMX, _BASE_OMX, (0.12,), limit=100
    )
    assert cands, "공통 워크스페이스가 비어 있으면 안 됨 (설정 ROI 기준)"
    for x, y, z in cands:
        assert _ROI_SO.x_min <= x <= _ROI_SO.x_max
        assert _ROI_SO.y_min <= y <= _ROI_SO.y_max
        px, py, pz = frames.world_to_robot((x, y, z), _BASE_OMX)
        assert _ROI_OMX.x_min <= px <= _ROI_OMX.x_max
        assert _ROI_OMX.y_min <= py <= _ROI_OMX.y_max
        assert _ROI_OMX.z_min <= pz <= _ROI_OMX.z_max


def test_rendezvous_empty_when_no_overlap():
    far = WorkcellRoi(
        x_min=5.0, x_max=5.2, y_min=5.0, y_max=5.2, z_min=0.0, z_max=0.3
    )
    assert frames.rendezvous_candidates(_ROI_SO, far, _BASE_OMX, (0.12,)) == []


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
        "enable_torque", "enable_torque",
        "go_home", "go_home", "set_gripper",
        "plan_omx_observe", "omx_observe_detect",
        "plan_omx_pick_block", "omx_pick_block",
        "plan_omx_present", "omx_present",
        "plan_so_observe", "so_redetect", "plan_receive",
        "set_gripper", "receive", "omx_retreat",
        "place_into", "go_home",
    ]
