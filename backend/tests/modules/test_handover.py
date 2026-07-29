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


def _aerial_det(position, score=0.8, height=0.05) -> OrientedDetection:
    """so101 공중 재검출 (world frame) — 수직 봉의 보이는 노출부.

    `position` 은 **노출부 중심**(steps.aerial_target 이 돌려주는 값) 의미로 준다:
    detector 의 position 은 몸통 군집의 *윗면*, base_z 는 그 바닥이므로
    윗면 = 중심 + height/2 / base_z = 중심 − height/2 로 구성한다 (그래야
    aerial_target(det) == position 이 되어 호출부 의도와 맞는다)."""
    return OrientedDetection(
        prompt="orange block",
        position=(position[0], position[1], position[2] + height / 2.0),
        score=score, base_z=position[2] - height / 2.0,
        height=height, grasp_yaw=0.3, footprint=(0.022, 0.020),
        points=[(position[0], position[1], position[2])] * 60,
    )


def _joint_state(gripper_raw: int) -> JointState:
    pos = [0] * 6
    pos[_SPEC.gripper_index] = gripper_raw
    return JointState(
        robot_id=SO, seq=0, timestamp_unix=0.0,
        positions_raw=pos, loads_raw=None,
    )


_W_HANG = (0.0, 0.0, -1.0)  # 매달기 = 축 일반형의 특수case (노출 방향 = 아래)


def _present(w=_W_HANG, tcp=None) -> steps.PresentPlan:
    """PresentPlan 픽스처 — 노출 방향 w 와 제시 TCP 로 E 를 봉 축 위에 둔다."""
    t = tuple(_TCP_W if tcp is None else tcp)
    e = tuple(t[i] + w[i] * _GEOM.tcp_to_e_m for i in range(3))
    return steps.PresentPlan(
        sols=[[0.4] * 5], quat=(0.0, 0.0, 0.0, 1.0),
        h_world=e, tcp_world=t, w=tuple(w), label="test",  # type: ignore[arg-type]
    )


def _observe_pose(
    h, *, pos_off=(0.0, 0.0, 0.0)
) -> tuple[steps.ObservePlan, TcpState]:
    """(수취 관측 계획, 그 계획에 **도달한** TCP 상태) 쌍 — 도달 검증 통과용.

    채택 그룹 = 사다리 첫 조합 (az_off/elev/dist/ψ 각 [0], _SELECT index=0).
    hand_eye 가 identity(_hand_eye_bundle)라 카메라 pose == TCP pose.
    `pos_off` 를 주면 어긋난 자세가 되어 미도달 실패를 재현한다."""
    az = math.atan2(h[1], h[0]) + math.radians(steps._RECV_OBS_AZOFF_DEG[0])
    elev = math.radians(steps._RECV_OBS_ELEV_DEG[0])
    d = steps._RECV_OBS_DIST_M[0]
    c = np.array([
        h[0] - math.cos(az) * d * math.cos(elev),
        h[1] - math.sin(az) * d * math.cos(elev),
        h[2] + d * math.sin(elev),
    ])
    axis = np.asarray(h, dtype=float) - c
    axis = axis / np.linalg.norm(axis)
    groups, _ = steps._camera_pose_groups(
        c, axis, (steps._RECV_OBS_PSI_DEG[0],), np.eye(4)
    )
    pose = groups[0][0]
    plan = steps.ObservePlan(
        joints=[0.5] * 6,
        cam_pos=(float(c[0]), float(c[1]), float(c[2])),
        cam_axis=(float(axis[0]), float(axis[1]), float(axis[2])),
    )
    reached = _tcp(
        tuple(pose.position[i] + pos_off[i] for i in range(3)),
        [0.5] * 6,
        quat=pose.quaternion,
    )
    return plan, reached


def _aerial_at(offset, *, along_axis, w, tcp, points=60, score=0.8):
    """계획 봉 세그먼트 기준 (축 방향 along_axis, 축 수직 offset) 위치의 검출."""
    a = np.asarray(w, dtype=float)
    a = a / np.linalg.norm(a)
    base = np.asarray(tcp, dtype=float) + a * (steps._OMX_JAW_ALONG_M / 2.0)
    perp = np.asarray(offset, dtype=float)
    perp = perp - float(np.dot(perp, a)) * a  # 축 성분 제거
    c = base + a * along_axis + perp
    pos = (float(c[0]), float(c[1]), float(c[2]))
    return OrientedDetection(
        prompt="orange block", position=pos, score=score,
        base_z=float(c[2]) - 0.01, height=0.02, grasp_yaw=0.1,
        footprint=(0.022, 0.020),
        points=[pos] * points,
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


def _happy_script(e=None) -> dict:
    """happy path 스크립트 — place_object="" (수취까지, 적치 생략).

    `e` = 채택될 제시 파지점 E (관측 도달 검증 픽스처가 이걸 겨냥한다). 기본은
    매달기 기하의 E — 수평 제시를 돌리는 테스트는 그 w 로 계산한 E 를 준다."""
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
        # so101 재검출(1) + 수취 refine(1). 겨냥점은 **제시 TCP 위**에 둔다 —
        # 매치가 "봉 축 수직 거리 + 축 방향 세그먼트 범위" 라 TCP 점은 어느 w
        # 후보가 채택돼도 (perp=0, along=−jaw/2) 로 통과한다 (w-무관 픽스처).
        _DETECT_ORIENTED: [
            DetectOrientedResponse(found=True, candidates=[_aerial_det(_TCP_W)]),
            DetectOrientedResponse(found=True, candidates=[_aerial_det(_TCP_W)]),
        ],
        _SELECT: [
            # omx 관측 자세 (ψ 격자 중 첫 그룹)
            ResolveReachableResponse(index=0, solutions=[[0.1] * 5]),
            # omx pick — 봉 끝 파지점 (양 끝 × z 사다리 중 첫 그룹, J5 자연해)
            ResolveReachableResponse(index=0, solutions=[[0.2] * 5]),
            # omx 제시 자세 (w 후보 [0] — 랑데부 후보 [0] 채택)
            ResolveReachableResponse(index=0, solutions=[[0.4] * 5]),
            # so101 수취 결합 probe (제시 채택 게이트, 2026-07-29)
            ResolveReachableResponse(index=0, solutions=[[0.55] * 6, [0.6] * 6]),
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
        # 제시 계획(omx) / **관측 도달 검증(so)** / 수취 계획(omx) /
        # retreat(so, omx) = 5. checker=None 이라 plan_so_observe 의 충돌
        # 게이트용 스냅샷(omx+so)은 호출되지 않는다. 수취 계획의 omx TCP 는
        # 겨냥점 FK 앵커의 원천이라 제시점(_TCP_W)과 일관돼야 한다.
        _TCP_SNAP: [
            _tcp((0.25, 0.0, 0.10), [0.3] * 5),
            _observe_pose(_H if e is None else e)[1],
            _tcp(steps.world_to_robot(_TCP_W, _BASE_OMX), [0.4] * 5),
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


@pytest.mark.parametrize("elev", [-90.0, 15.0], ids=["매달기", "수평제시"])
async def test_scenario_happy_path_and_release_order(monkeypatch, elev):
    """전 시나리오 호출 경로 + 수취 순서 불변식 — **매달기/수평 두 제시족 모두**.

    elevation 사다리를 한 값으로 고정해 채택 w 를 결정론적으로 만들고, 그 w 로
    계산한 E 를 관측 도달 검증 픽스처에 준다 (w 는 production 함수에서 유도 —
    테스트가 선택 로직을 복제하지 않는다)."""
    monkeypatch.setattr(steps, "_PRESENT_W_ELEV_DEG", (elev,))
    # 채택될 w = E-ROI 게이트(production 과 동형)를 통과하는 첫 후보 — mock
    # resolve 는 항상 성공하므로 이 필터가 곧 채택 순서다 (omx-접선 후보의 E 가
    # 픽스처 ROI 밖이면 반대 접선이 채택된다 — 후보[0] 하드코딩 금지).
    def _e_of(wv):
        return tuple(_TCP_W[i] + wv[i] * _GEOM.tcp_to_e_m for i in range(3))

    w = next(
        wv for _lb, wv in steps._present_w_candidates(_TCP_W, _BASE_OMX)
        if _ROI_SO.x_min <= _e_of(wv)[0] <= _ROI_SO.x_max
        and _ROI_SO.y_min <= _e_of(wv)[1] <= _ROI_SO.y_max
        and _ROI_SO.z_min <= _e_of(wv)[2] <= _ROI_SO.z_max
    )
    e = _e_of(w)
    ctx = _ctx(_happy_script(e))
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


async def test_present_axis_invariants_tool_z_is_minus_w_and_e_on_axis():
    """제시 축 일반형 잠금 (2026-07-28 수평 제시 전환) — 어느 w 후보가 채택돼도:

    ① 채택 quat 의 **tool z (world) == −w** (pick 이 tool z ∥ −u 로 물었으므로
       노출 방향 = −tool z. 깨지면 봉이 반대로 뻗어 수취가 조용히 깨진다)
    ② **E == TCP + w·tcp_to_e** (파지점은 봉 축 위 — 매달기면 옛 "TCP 아래")
    ③ E 는 so101 workcell ROI 안 (두 팔 유효 대역 정합)"""
    ctx = _ctx(_happy_script())
    script_pick = steps.BlockPick(
        sols=[[0.2] * 5], quat=(0.0, 0.0, 0.0, 1.0),
        grasp_omx=(0.2, 0.0, 0.016), u_omx=(1.0, 0.0),
        geom=_GEOM, chosen_dz=0.016,
    )
    plan = await steps.plan_omx_present(
        ctx, OMX, SO, _ROI_SO, _ROI_OMX, _BASE_OMX, script_pick,
        [0.1] * 6, None,
    )
    # quat 은 omx frame — base yaw 로 world 로 돌려 비교
    to_world = Rotation.from_euler("z", _BASE_OMX.yaw_rad)
    tool_z_w = (to_world * Rotation.from_quat(plan.quat)).apply([0.0, 0.0, 1.0])
    assert tool_z_w == pytest.approx([-v for v in plan.w], abs=1e-6)
    expect_e = tuple(
        plan.tcp_world[i] + plan.w[i] * _GEOM.tcp_to_e_m for i in range(3)
    )
    assert plan.h_world == pytest.approx(expect_e, abs=1e-9)
    assert _ROI_SO.x_min <= plan.h_world[0] <= _ROI_SO.x_max
    assert _ROI_SO.y_min <= plan.h_world[1] <= _ROI_SO.y_max
    assert _ROI_SO.z_min <= plan.h_world[2] <= _ROI_SO.z_max


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
    plan, reached = _observe_pose(_H)
    ctx = _ctx({
        _MOVE_J: [MoveJResponse()],
        _TCP_SNAP: [reached],
        _DETECT_ORIENTED: [DetectOrientedResponse(found=False, candidates=[])],
    })
    with pytest.raises(DetectionNotFound, match="재검출"):
        await steps.so_redetect(
            ctx, SO, "orange block", plan, _present(), _GEOM, np.eye(4)
        )
    # 실패 후 추가 모션 없음 (FK 폴백 경로 부재). TCP 조회는 도달 검증 1회뿐
    assert len(ctx.calls(_MOVE_J)) == 1
    assert len(ctx.calls(_TCP_SNAP)) == 1


async def test_observe_pose_not_reached_fails_before_detection():
    """관측 자세 미도달 = **검출 전에** 명시 실패.

    ⚠ 2026-07-28 실물 회귀: so101 이 계획과 위치 14cm·광축 40° 다른 자세에
    있었는데 그대로 검출을 시도해, 시야에 있던 갈색 책상이 "orange block" 으로
    잡히고 실패 사유가 "검출 실패"로 위장됐다. 원인이 사유에 드러나야 한다."""
    plan, off = _observe_pose(_H, pos_off=(0.14, 0.0, 0.0))  # 14cm 어긋남
    ctx = _ctx({
        _MOVE_J: [MoveJResponse()],
        _TCP_SNAP: [off],
        _DETECT_ORIENTED: [
            DetectOrientedResponse(found=True, candidates=[_aerial_det(_H)]),
        ],
    })
    with pytest.raises(TaskError, match="도달하지 못"):
        await steps.so_redetect(
            ctx, SO, "orange block", plan, _present(), _GEOM, np.eye(4)
        )
    assert ctx.calls(_DETECT_ORIENTED) == []  # 검출 호출 자체가 없어야


def test_camera_pose_of_inverts_camera_pose_groups():
    """`_camera_pose_of`(TCP→카메라) 는 `_camera_pose_groups`(카메라→TCP) 의 역 —
    **같은 X** 로 왕복해야 한다. 깨지면 도달 검증이 정상 자세를 '미도달' 로
    오판(또는 그 반대)한다. hand_eye 를 비대각·비영으로 둬야 의미 있는 검증."""
    x = np.eye(4)
    x[:3, :3] = Rotation.from_euler("xyz", [20, -35, 50], degrees=True).as_matrix()
    x[:3, 3] = [0.02, -0.05, 0.07]
    c = np.array([0.10, -0.12, 0.33])
    axis = np.array([0.6, -0.5, -0.62])
    axis = axis / np.linalg.norm(axis)
    groups, _ = steps._camera_pose_groups(c, axis, (37.0,), x)
    pose = groups[0][0]
    cam_pos, cam_axis = steps._camera_pose_of(
        _tcp(pose.position, [0.0] * 6, quat=pose.quaternion), x
    )
    assert cam_pos == pytest.approx(tuple(c), abs=1e-9)
    assert steps._axis_error_deg(cam_axis, tuple(axis)) < 1e-6


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


async def test_inflated_short_side_still_trusted():
    """mono 가 부풀린 짧은 변(실물 20mm → 35mm)도 신뢰 게이트를 통과해야 한다.

    ⚠ 2026-07-28 실물 회귀: 상한 35mm 가 실측 분포(29/33/33/35mm) 위에 걸터앉아
    score 0.82·점군 421 의 **정상 검출을 컷했다**. 그리고 파지 기하는 검출
    footprint 가 아니라 known 스펙(_BLOCK_LEN_M/_BLOCK_CROSS_M)을 써야 한다."""
    det = _block_det(footprint=(0.102, 0.0354), score=0.82)
    ctx = _ctx({
        _MOVE_J: [MoveJResponse()],
        _DETECT_PLANAR: [DetectOrientedResponse(found=True, candidates=[det])],
    })
    got = await steps.omx_observe_detect(ctx, OMX, "orange block", [0.1] * 5)
    assert got.footprint[1] == pytest.approx(0.0354)
    grasp = steps.plan_block_grasp_from(got, _BASE_OMX)
    assert grasp.length_m == pytest.approx(steps._BLOCK_LEN_M)
    assert grasp.width_m == pytest.approx(steps._BLOCK_CROSS_M)  # 검출값 아님


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
        # omx TCP = 제시 자세 (omx frame) — 겨냥 z 의 **FK 앵커** 원천이라
        # world 제시점과 일관돼야 한다 (anchor = TCP_world + w·tcp_to_e = _H)
        _TCP_SNAP: [_tcp(steps.world_to_robot(_TCP_W, _BASE_OMX), [0.4] * 5)],
        _SELECT: [
            ResolveReachableResponse(index=0, solutions=[[0.1] * 6, [0.2] * 6]),
        ] * n_resolve,
    }


async def test_plan_receive_retries_past_colliding_group():
    checker = _FakeChecker(hits=[True, False])
    ctx = _ctx(_receive_script(2))
    plan = await steps.plan_receive(
        ctx, SO, OMX, _aerial_det(_H), _BASE_OMX, _present(), _GEOM,
        checker,  # type: ignore[arg-type]
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
            ctx, SO, OMX, _aerial_det(_H), _BASE_OMX, _present(), _GEOM,
            checker,  # type: ignore[arg-type]
        )


async def test_receive_aim_axis_component_uses_fk_anchor():
    """수취 겨냥점 = **축 성분은 FK 앵커 / 축 수직 성분은 검출**.

    ⚠ 2026-07-28 실물 회귀: omx 손목이 봉을 가려 아래 조각만 잡히자(점군 88,
    보이는 높이 2.5cm) 검출 centroid 가 축 방향 2.8cm 밀려 수취 IK 가 전멸했다.
    봉은 강체이므로 축 위 위치는 omx FK 가 정확하다 — 축 방향 밀림은 앵커로
    덮고, 축 수직 오차(omx 그립 오차)는 검출을 따라야 한다."""
    checker = _FakeChecker(hits=[False])
    ctx = _ctx(_receive_script(1))
    w = _W_HANG
    # 축 방향 2.8cm 밀림 + 축 수직 (xy) 로 (−24, +19)mm 어긋난 검출
    bad = _aerial_at((-0.024, 0.019, 0.0), along_axis=0.028, w=w, tcp=_TCP_W)
    plan = await steps.plan_receive(
        ctx, SO, OMX, bad, _BASE_OMX, _present(w), _GEOM,
        checker,  # type: ignore[arg-type]
    )
    assert plan.target[2] == pytest.approx(_H[2], abs=1e-9)  # 축 성분 = 앵커
    assert plan.target[0] == pytest.approx(_H[0] - 0.024, abs=1e-6)
    assert plan.target[1] == pytest.approx(_H[1] + 0.019, abs=1e-6)


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


@pytest.mark.parametrize(
    "axis",
    [(0.0, 0.0, -1.0), (-0.96, -0.064, 0.274), (1.0, 0.0, 0.0)],
    ids=["hang-수직", "실측-수평", "수평-x축"],
)
def test_recv_orients_aligns_tool_z_with_given_axis(axis):
    """수취 자세족 잠금 — **주어진 봉 축**에 tool z 를 정렬하고 축 둘레 spin.

    옛 코드는 수직을 하드코딩했다 (수평 제시에선 봉을 가로로 못 문다). 축이
    수직인 경우가 옛 "수직 조축 + 수평 접근" 족과 같아지는지도 같이 잠근다."""
    e = (0.21, -0.16, 0.255)
    orients = steps._recv_orients(e, axis)
    assert orients
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    for _label, q, ap in orients:
        r = Rotation.from_quat(q)
        tool_z = r.apply([0.0, 0.0, 1.0])
        assert abs(float(np.dot(tool_z, a))) == pytest.approx(1.0, abs=1e-6)
        # 접근(tool x)·조축(tool y)은 축에 수직 — 봉 단면을 가로질러 문다
        assert float(np.dot(ap, a)) == pytest.approx(0.0, abs=1e-6)
        tool_y = r.apply([0.0, 1.0, 0.0])
        assert float(np.dot(tool_y, a)) == pytest.approx(0.0, abs=1e-6)
    # 첫 후보 접근 = base→E 방위를 축 수직 평면으로 투영한 것 (so101 쪽 진입)
    rad = np.array([e[0], e[1], 0.0])
    rad = rad / np.linalg.norm(rad)
    expect = rad - float(np.dot(rad, a)) * a
    expect = expect / np.linalg.norm(expect)
    assert float(np.dot(orients[0][2], expect)) == pytest.approx(1.0, abs=1e-6)


def test_present_quat_axis_hang_is_wrist_neutral():
    """매달기(w=아래)는 축 일반형의 특수case — tool z ↑ + tool x 가 방위 α 의
    수평 radial = 정확히 Rz(α) (ZYYYX 에서 θ=0, **J5=0 손목 중립**. 옛 B/down
    의 J5=±180 케이블 감김 수정 회귀 잠금, 2026-07-27)."""
    alpha = math.radians(25.0)
    q = steps._present_quat_axis(_W_HANG, alpha)
    r = Rotation.from_quat(q)
    assert r.apply([0.0, 0.0, 1.0]) == pytest.approx([0.0, 0.0, 1.0], abs=1e-9)
    assert r.apply([1.0, 0.0, 0.0]) == pytest.approx(
        [math.cos(alpha), math.sin(alpha), 0.0], abs=1e-9
    )
    expect = Rotation.from_euler("z", alpha)
    assert (r.inv() * expect).magnitude() == pytest.approx(0.0, abs=1e-9)


def test_present_quat_axis_horizontal_has_vertical_jaw():
    """수평 제시 — tool z == −w 이고 **jaw(tool y)가 수직**.

    ⚠ 2026-07-28 사용자 토크오프 실측 근거: 두 로봇 조가 모두 수직(omx 18.5° /
    so101 9.7° off)이었다. 조가 봉을 위/아래로 물어야 중력이 조를 비틀지 않고
    so101 수취 자세족과 규약이 같아진다."""
    w = (-0.96, -0.064, 0.274)
    q = steps._present_quat_axis(w, alpha=0.0)
    r = Rotation.from_quat(q)
    wn = np.asarray(w, dtype=float)
    wn = wn / np.linalg.norm(wn)
    assert r.apply([0.0, 0.0, 1.0]) == pytest.approx(-wn, abs=1e-6)
    tool_y = r.apply([0.0, 1.0, 0.0])
    assert abs(float(tool_y[2])) > 0.95, f"jaw 가 수직이 아님 ({tool_y})"


# 실 배치 base (robots.yaml 2026-07-28 직각 재배치 미러) — 접선 frame 회귀
# 테스트 전용. 실측 봉 축 앵커와 같은 좌표계여야 의미가 있다 (fixture
# _BASE_OMX 는 옛 배치라 별도로 둔다). robots.yaml 배치가 바뀌면 같이 갱신.
_BASE_OMX_20260728 = BasePose(x=0.135, y=-0.40, z=-0.0094, yaw_rad=math.pi / 2)


def test_present_w_candidates_omx_tangent_and_hang_last():
    """w 후보 — **omx base 기준 접선** + hang 은 마지막 단일 폴백.

    2026-07-29 근인 회귀: 옛 코드는 접선을 so101(world 원점) 기준으로 계산해
    직각 배치에서 두 접선이 ~21° 어긋났고, 수평 후보 전원이 omx 5DOF 다양체
    밖 → 자세 IK 전멸 → 매달기 폴백이었다. 실측 봉 축 (-0.96,-0.064,0.274)
    은 omx 접선과 0.3° — 접선의 소속 frame 이 omx 임을 잠근다."""
    tcp = (0.126, -0.274, 0.204)  # 사용자 토크오프 실측 제시 TCP
    base = _BASE_OMX_20260728
    cands = steps._present_w_candidates(tcp, base)
    assert cands[0][0].startswith("-t")
    # 접선 = omx radial 에 수직 (so101 radial 에는 수직이 아니다 — 이 배치에서
    # 두 기준이 ~21° 어긋나는 것이 근인이었다)
    r_omx = np.array([tcp[0] - base.x, tcp[1] - base.y, 0.0])
    r_omx = r_omx / np.linalg.norm(r_omx)
    horiz = np.array([cands[0][1][0], cands[0][1][1], 0.0])
    horiz = horiz / np.linalg.norm(horiz)
    assert float(np.dot(horiz, r_omx)) == pytest.approx(0.0, abs=1e-6)
    r_so = np.array([tcp[0], tcp[1], 0.0])
    r_so = r_so / np.linalg.norm(r_so)
    assert abs(float(np.dot(horiz, r_so))) > 0.2, (
        "so101 radial 에도 수직 — 이 배치에선 두 접선이 달라야 회귀를 잡는다"
    )
    # 실측 봉 축과 같은 방향 분기(-t = so101 에서 멀어지는 쪽)가 첫 후보
    measured_w = np.array([-0.96, -0.064, 0.274])
    measured_w = measured_w / np.linalg.norm(measured_w)
    first_elev15 = next(v for lb, v in cands if lb == "-t/elev+15")
    assert float(np.dot(np.asarray(first_elev15), measured_w)) > 0.99
    # hang 은 마지막 **단일** 후보 (family 별 -90 중복 + 순서 버그 회귀 방지)
    labels = [c[0] for c in cands]
    assert labels[-1] == "hang"
    assert labels.count("hang") == 1
    assert not any("-90" in lb for lb in labels[:-1])
    assert not any(lb.startswith(("-r", "+r")) for lb in labels)


def test_present_candidates_on_omx_manifold():
    """수평 w 후보 × jaw-수직 quat 이 **omx 5DOF(ZYYYX) 다양체 위**임을 잠근다
    — tool x(omx frame) 가 팔 평면(TCP 방위 α) 안 (analytic_zyyyx 의 M=Ry·Rx
    분해 가능 조건). 이게 깨지면 자세 IK 가 조용히 전멸하고 매달기로 후퇴한다
    (2026-07-29 실물 전멸의 기하 근인 — 발견 시나리오 그대로)."""
    base = _BASE_OMX_20260728
    for tcp in [(0.138, -0.259, 0.20), (0.106, -0.259, 0.20), (0.171, -0.228, 0.22)]:
        tcp_o = steps.world_to_robot(tcp, base)
        alpha = math.atan2(tcp_o[1], tcp_o[0])
        n = np.array([-math.sin(alpha), math.cos(alpha), 0.0])  # 팔 평면 법선
        for label, w in steps._present_w_candidates(tcp, base):
            w_omx = frames.world_dir_to_robot(w, base)
            q = steps._present_quat_axis(w_omx, alpha)
            x = Rotation.from_quat(q).apply([1.0, 0.0, 0.0])
            assert abs(float(np.dot(x, n))) < 1e-6, (label, tcp)


async def test_present_rejects_candidate_when_receive_probe_dies():
    """수취 결합 게이트 (2026-07-29) — 제시 자기 게이트를 통과해도 so101 수취
    probe 가 전멸이면 그 w 를 기각하고 다음 후보로 간다. 수취 해가 랑데부
    지역에 1~2/16 뿐이라 이 게이트 없이는 '제시만 되는' 후보가 채택돼 수취
    전멸이 실행 후에야 드러난다 (20260729 실물 전멸의 파이프라인 근인).

    ROI 는 전 후보가 E-ROI 게이트를 통과하게 넓게 준다 — resolve 소비 순서가
    w 사다리 순서와 1:1 이 되어 스크립트가 결정론적이다."""
    wide = WorkcellRoi(
        x_min=-1.0, x_max=1.0, y_min=-1.0, y_max=1.0, z_min=-1.0, z_max=1.0
    )
    script = _happy_script()
    script[_SELECT] = [
        # 제시 w 후보 #1 도달
        ResolveReachableResponse(index=0, solutions=[[0.4] * 5]),
        # 그 후보의 수취 probe 전멸 → 기각돼야 한다
        ResolveReachableResponse(index=-1, message="전멸"),
        # 제시 w 후보 #2 도달
        ResolveReachableResponse(index=0, solutions=[[0.41] * 5]),
        # 수취 probe 통과 → 채택
        ResolveReachableResponse(index=0, solutions=[[0.6] * 6, [0.65] * 6]),
    ]
    ctx = _ctx(script)
    pick = steps.BlockPick(
        sols=[[0.2] * 5], quat=(0.0, 0.0, 0.0, 1.0),
        grasp_omx=(0.2, 0.0, 0.016), u_omx=(1.0, 0.0),
        geom=_GEOM, chosen_dz=0.016,
    )
    plan = await steps.plan_omx_present(
        ctx, OMX, SO, wide, wide, _BASE_OMX, pick, [0.1] * 6, None,
    )
    assert len(ctx.calls(_SELECT)) == 4
    # 첫 w 후보(-t/elev+15)가 아니라 둘째(-t/elev+0)가 채택됐다
    assert plan.label == "-t/elev+0"
    # probe resolve 는 so101 로 라우팅 (제시 resolve 는 omx)
    probe_calls = [c for c in ctx.calls(_SELECT) if c["robot_id"] == SO]
    assert len(probe_calls) == 2


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


# ─── 수취 겨냥점 / 게이트 정합 (2026-07-28 실물 회귀) ──────────────────


async def test_jaw_side_fragment_axis_error_absorbed_by_anchor():
    """2026-07-27 실물 실패(수취 IK 전멸)의 검출측 근인을 **앵커로 무해화**한다.

    omx 조 가림으로 점군이 갈렸을 때 검출된 **조 안쪽 조각** (base_z=0.2884 /
    height=0.0151 / position z=0.3035 — 실측값) 은 계획 E(z=0.2549) 대비 축
    방향으로 41mm 위다. 옛 코드는 그 z 를 그대로 겨냥해 수취 IK 가 전멸했다.

    축 일반형에서는 이 조각을 **기각하지 않는다** — 축 방향 오차는 가림에 따라
    항상 생기는 값이므로 FK 앵커로 덮고(그러면 무해), 축 수직 성분만 보정으로
    쓴다. 즉 잠글 것은 "기각" 이 아니라 **"겨냥점 축 성분이 앵커를 따른다"** 다."""
    frag = OrientedDetection(
        prompt="orange block", position=(0.2036, 0.1118, 0.3035), score=0.834,
        base_z=0.2884, height=0.0151, grasp_yaw=-0.46, footprint=(0.019, 0.009),
        points=[(0.2036, 0.1118, 0.296)] * 45,
    )
    checker = _FakeChecker(hits=[False])
    ctx = _ctx(_receive_script(1))
    plan = await steps.plan_receive(
        ctx, SO, OMX, frag, _BASE_OMX, _present(_W_HANG), _GEOM,
        checker,  # type: ignore[arg-type]
    )
    # 겨냥 z = FK 앵커 (검출의 41mm 축 오차가 실리지 않는다)
    assert plan.target[2] == pytest.approx(_H[2], abs=1e-9)
    assert abs(plan.target[2] - frag.position[2]) > 0.04, (
        "검출 z 가 겨냥점에 실렸다 — 앵커 규약 회귀"
    )


@pytest.mark.parametrize(
    "w,tcp",
    [((0.0, 0.0, -1.0), (0.29, -0.157, 0.28)),
     ((-0.96, -0.064, 0.274), (0.126, -0.274, 0.204))],
    ids=["hang-수직", "실측-수평"],
)
def test_match_aerial_axis_general_gates(w, tcp):
    """매치 게이트 축 일반형 — **축 수직 거리는 엄격 / 축 방향은 세그먼트 범위**.

    옛 게이트는 "계획점 xy 반경 + z 대역" 이라 봉이 수직일 때만 맞는 판정이었다.
    축 방향 밀림은 통과시켜야 하고(가림 의존 값 + 하류가 FK 앵커로 덮는다),
    축에서 **벗어난** 후보는 기각해야 한다 (그 성분이 수취 IK 를 죽인다)."""
    ok = _aerial_at((0, 0, 0), along_axis=0.03, w=w, tcp=tcp)
    assert steps._match_aerial([ok], tcp, w, _GEOM) is ok
    # 가림으로 축 방향 5.2cm 밀림 (노출 5.4cm 안) → 통과해야 (앵커가 덮는다)
    drift = _aerial_at((0, 0, 0), along_axis=0.052, w=w, tcp=tcp)
    assert steps._match_aerial([drift], tcp, w, _GEOM) is drift
    # 축에서 크게 벗어남 (반경 8cm 초과) → 기각
    off = _aerial_at((0.12, 0.05, 0.09), along_axis=0.03, w=w, tcp=tcp)
    assert steps._match_aerial([off], tcp, w, _GEOM) is None
    # 축 방향 세그먼트 훨씬 밖 → 기각
    far = _aerial_at((0, 0, 0), along_axis=0.20, w=w, tcp=tcp)
    assert steps._match_aerial([far], tcp, w, _GEOM) is None
    # 점군 게이트 유지
    thin = _aerial_at((0, 0, 0), along_axis=0.03, w=w, tcp=tcp, points=5)
    assert steps._match_aerial([thin], tcp, w, _GEOM) is None


def test_recv_pre_clear_ladder_prefers_long_standoff():
    """접근 여유 사다리는 **큰 것 우선** (긴 standoff 가 정렬/refine 에 유리) —
    특이점에 걸릴 때만 짧아진다. 순서가 뒤집히면 항상 최단으로 붙는다."""
    ladder = steps._RECV_PRE_CLEAR_LADDER
    assert len(ladder) >= 2
    assert list(ladder) == sorted(ladder, reverse=True), "사다리가 내림차순이 아님"
    assert ladder[0] == 0.07, "선호값이 옛 단일값(7cm)과 달라졌다 — 의도적 변경인가"
    assert min(ladder) <= 0.05, (
        "7cm pre 가 특이점 플립으로 죽는 프레임이 실물에 있었다 — 5cm 이하 단계 필수"
    )
