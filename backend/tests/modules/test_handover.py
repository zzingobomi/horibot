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


def _e_in_roi(pt):
    """production plan_omx_present 의 E-ROI 게이트 동형 (slack 포함)."""
    s_m = steps._E_ROI_SLACK_M
    return (
        _ROI_SO.x_min - s_m <= pt[0] <= _ROI_SO.x_max + s_m
        and _ROI_SO.y_min - s_m <= pt[1] <= _ROI_SO.y_max + s_m
        and _ROI_SO.z_min - s_m <= pt[2] <= _ROI_SO.z_max + s_m
    )



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
        # 제시 계획(omx) / **관측 도달 검증(so)** / **봉 실측(omx)** /
        # 수취 계획(omx) / retreat(so, omx) = 6. checker=None 이라
        # plan_so_observe 의 충돌 게이트용 스냅샷(omx+so)은 호출되지 않는다.
        # 봉 실측의 omx TCP 는 겨냥점 FK 앵커(폴백)의 원천이라 제시점(_TCP_W)과
        # 일관돼야 한다.
        _TCP_SNAP: [
            _tcp((0.25, 0.0, 0.10), [0.3] * 5),
            _observe_pose(_H if e is None else e)[1],
            _tcp(steps.world_to_robot(_TCP_W, _BASE_OMX), [0.4] * 5),
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
        if _e_in_roi(_e_of(wv))
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
    assert _e_in_roi(plan.h_world), "E 가 ROI+slack 밖 — E-ROI 게이트 회귀"


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
        # omx TCP = 제시 자세 (omx frame) — plan_receive 의 충돌 형상 원천
        _TCP_SNAP: [_tcp(steps.world_to_robot(_TCP_W, _BASE_OMX), [0.4] * 5)],
        _SELECT: [
            ResolveReachableResponse(index=0, solutions=[[0.1] * 6, [0.2] * 6]),
        ] * n_resolve,
    }


def _meas(target=None, axis=_W_HANG, fallback=False) -> steps.MeasuredBar:
    """MeasuredBar 픽스처 — plan_receive 게이트 단위 테스트용."""
    return steps.MeasuredBar(
        axis=tuple(axis), target=tuple(_H if target is None else target),
        span_m=0.04, n_points=60, axis_dev_deg=0.0, fallback=fallback,
    )


async def test_plan_receive_retries_past_colliding_group():
    checker = _FakeChecker(hits=[True, False])
    ctx = _ctx(_receive_script(2))
    plan = await steps.plan_receive(
        ctx, SO, OMX, _meas(), _present(), _GEOM,
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
    with pytest.raises(NoReachableGrasp, match="전멸"):
        await steps.plan_receive(
            ctx, SO, OMX, _meas(), _present(), _GEOM,
            checker,  # type: ignore[arg-type]
        )


async def test_measure_bar_axis_anchor_fallback():
    """봉 실측 폴백 = **축 성분은 FK 앵커 / 축 수직 성분은 검출** (옛 plan_receive
    앵커 규약 계승 — 점군이 빈약해 축을 못 재는 경우의 안전망).

    ⚠ 2026-07-28 실물 회귀: omx 손목이 봉을 가려 아래 조각만 잡히자(점군 88,
    보이는 높이 2.5cm) 검출 centroid 가 축 방향 2.8cm 밀려 수취 IK 가 전멸했다.
    봉은 강체이므로 축 위 위치는 omx FK 가 정확하다 — 축 방향 밀림은 앵커로
    덮고, 축 수직 오차(omx 그립 오차)는 검출을 따라야 한다."""
    ctx = _ctx(_receive_script(0))
    w = _W_HANG
    # 축 방향 2.8cm 밀림 + 축 수직 (xy) 로 (−24, +19)mm 어긋난 검출 — 점군이
    # 전부 동일점(_aerial_at 픽스처)이라 주축 span 0 → 폴백 경로
    bad = _aerial_at((-0.024, 0.019, 0.0), along_axis=0.028, w=w, tcp=_TCP_W)
    meas = await steps.measure_bar(
        ctx, OMX, _BASE_OMX, bad, _present(w), _GEOM
    )
    assert meas.fallback
    assert meas.axis == pytest.approx(w, abs=1e-9)  # 폴백 축 = 계획축
    assert meas.target[2] == pytest.approx(_H[2], abs=1e-9)  # 축 성분 = 앵커
    assert meas.target[0] == pytest.approx(_H[0] - 0.024, abs=1e-6)
    assert meas.target[1] == pytest.approx(_H[1] + 0.019, abs=1e-6)


async def test_measure_bar_rotated_bar_uses_measured_axis():
    """2026-07-29 실물 근인 회귀 — 봉이 omx 조 안에서 ~90° 돌면 (계획축 수직
    vs 실물 수평) 수취는 **실측축**을 따라야 한다. 옛 코드는 계획축 가정으로
    자세족+겨냥점을 만들어 전멸했다 (trace 20260729_232140: 점군 주축 vs
    계획축 84°, 검출은 정확 — 마스크 분홍 99%/span 85mm)."""
    w = _W_HANG  # 계획: 매달기 (수직)
    # 실물: 봉이 수평 +x 로 뻗음 — 자유단이 omx TCP 에서 +x 쪽
    axis_real = np.array([1.0, 0.0, 0.0])
    start = np.asarray(_TCP_W) + np.array([0.01, 0.0, -0.02])
    pts = [
        tuple(float(v) for v in start + axis_real * (0.046 * k / 39))
        for k in range(40)
    ]
    det = OrientedDetection(
        prompt="orange block", position=pts[20], score=0.7,
        base_z=float(start[2]) - 0.01, height=0.02, grasp_yaw=0.0,
        footprint=(0.046, 0.02), points=pts,
    )
    ctx = _ctx(_receive_script(0))
    meas = await steps.measure_bar(
        ctx, OMX, _BASE_OMX, det, _present(w), _GEOM
    )
    assert not meas.fallback
    assert meas.axis_dev_deg == pytest.approx(90.0, abs=3.0)  # 조 안 회전 지문
    assert meas.axis == pytest.approx((1.0, 0.0, 0.0), abs=0.05)
    # 겨냥점 = 실측 자유단(≈start+46mm)에서 inset 안쪽 — 계획 E(수직 아래)가 아님
    tip = start + axis_real * 0.046
    expect = tip - axis_real * steps._GRASP_TIP_INSET_M
    assert meas.target[0] == pytest.approx(float(expect[0]), abs=0.006)
    assert meas.target[2] == pytest.approx(float(expect[2]), abs=0.006)


# ─── ⑤ 봉 기하 (순수) ────────────────────────────────────────────────


def test_plan_block_grasp_ends_and_offsets():
    g = _GEOM  # (0.20, 0.0), yaw 0, footprint (0.080, 0.020)
    assert g.length_m == pytest.approx(0.080)
    assert len(g.ends) == 2  # 양 끝 동등 후보 (축대칭)
    # 끝점 = 중심 ± 4cm, 파지점 = 끝에서 frac(20% = 1.6cm) 안쪽
    (g1, u1), (g2, u2) = g.ends
    _g_off = steps._BLOCK_GRASP_FRAC * steps._BLOCK_LEN_M  # 잡는 끝에서 안쪽
    assert g1 == pytest.approx((0.20 - 0.04 + _g_off, 0.0), abs=1e-12)
    assert u1 == pytest.approx((1.0, 0.0), abs=1e-12)  # 노출 = +x (반대 끝)
    assert g2 == pytest.approx((0.20 + 0.04 - _g_off, 0.0), abs=1e-12)
    assert u2 == pytest.approx((-1.0, 0.0), abs=1e-12)
    # 노출 = L − g − 조절반 / E 오프셋 = 조절반 + 0.65·노출 — knob 파생
    _exp = steps._BLOCK_LEN_M - _g_off - steps._OMX_JAW_ALONG_M / 2
    assert g.exposed_len_m == pytest.approx(_exp)
    assert g.tcp_to_e_m == pytest.approx(0.01 + 0.65 * _exp)
    assert g.below_e_m == pytest.approx(0.35 * _exp)


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
    _off = steps._BLOCK_LEN_M / 2 - steps._BLOCK_GRASP_FRAC * steps._BLOCK_LEN_M
    assert g1[0] == pytest.approx(0.171 - _off)
    assert g2[0] == pytest.approx(0.171 + _off)
    # E 오프셋도 known 기하 (실물 런에선 부푼 길이로 60mm → 봉 끝 지점)
    _exp = steps._BLOCK_LEN_M * (1 - steps._BLOCK_GRASP_FRAC) - steps._OMX_JAW_ALONG_M / 2
    assert g.tcp_to_e_m == pytest.approx(0.01 + 0.65 * _exp)


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
def test_grasp_orients_jaw_perp_and_axis_superset(axis):
    """수취 자세족 잠금 — **잡기 조건 그리드** (2026-07-30 전면 교체).

    ① 모든 후보의 **조 닫힘축(tool y) ⊥ 봉 축** (정확) — 잡기의 유일한 물리
       제약. quat 과 접근 벡터의 정합(x = quat 의 tool x)도 같이 잠근다.
    ② roll0/roll180 후보의 tool z ∥ ±축 — 옛 정확 정렬족이 부분집합으로 보존
       (superset: 옛 해가 사라지지 않는다).
    ③ 첫 후보 접근 = base→E 방위 정렬 (so101 쪽 진입 선호 — omx 감아돌기 회피).
    ④ 후보 수 = φ(8) × ψ(len(_RECV_APPROACH_ROLL_DEG)) — 해 공간이 옛 16개에서
       실제로 넓어졌는지 (full_diagnosis.py: 과잉 제약 제거가 전멸의 해법)."""
    e = (0.21, -0.16, 0.255)
    orients = steps._grasp_orients(e, axis)
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    n_phi = int(360.0 / steps._RECV_SPIN_STEP_DEG)
    assert len(orients) == n_phi * len(steps._RECV_APPROACH_ROLL_DEG)
    for label, q, ap in orients:
        r = Rotation.from_quat(q)
        tool_y = r.apply([0.0, 1.0, 0.0])
        assert float(np.dot(tool_y, a)) == pytest.approx(0.0, abs=1e-6), label
        assert r.apply([1.0, 0.0, 0.0]) == pytest.approx(ap, abs=1e-9)
        if label.endswith("/roll0"):
            tool_z = r.apply([0.0, 0.0, 1.0])
            assert float(np.dot(tool_z, a)) == pytest.approx(1.0, abs=1e-6)
        if label.endswith("/roll180"):
            tool_z = r.apply([0.0, 0.0, 1.0])
            assert float(np.dot(tool_z, a)) == pytest.approx(-1.0, abs=1e-6)
    # 선호순 잠금 — 첫 후보의 접근(tool x)이 **base→E 진입 방위와 가장 정렬**
    # (so101 쪽 진입 선호 = omx 감아돌기 회피. 옛 "투영 = 첫 후보" 등식은
    # ψ 확장으로 더 정렬된 후보가 생기면 그쪽이 앞서는 게 맞다)
    rad = np.array([e[0], e[1], 0.0])
    rad = rad / np.linalg.norm(rad)
    aligns = [float(np.dot(ap, rad)) for _lb, _q, ap in orients]
    assert aligns[0] == pytest.approx(max(aligns), abs=1e-9)


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
    assert pick.grasp_omx[0] == pytest.approx(
        0.20 + 0.04 - steps._BLOCK_GRASP_FRAC * steps._BLOCK_LEN_M
    )
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
        "set_gripper",  # 집기 재시도 except 안 open (조건부)
        "plan_omx_present", "omx_present",
        "set_gripper",  # so101 open — 관측 전 (본인 조 가림 제거, 2026-07-29)
        "plan_so_observe", "so_redetect",
        # 재제시 보정 루프 = 제시 전면 재계획 (world_offset, look-then-move)
        "plan_omx_present", "omx_present", "plan_so_observe", "so_redetect",
        # 봉 실측 → 수취 계획 → (전멸 시) 협상: 이동 제안 → omx 재배치 →
        # 재관측/재검출/재실측 (2026-07-30)
        "measure_bar", "plan_receive",
        "find_receive_shift", "omx_nudge",
        "plan_so_observe", "so_redetect", "measure_bar",
        "receive", "omx_retreat",
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
    ctx = _ctx(_receive_script(0))
    meas = await steps.measure_bar(
        ctx, OMX, _BASE_OMX, frag, _present(_W_HANG), _GEOM
    )
    # 점군 전부 동일점(조각) → 축 실측 불가 → 폴백: 겨냥 z = FK 앵커
    # (검출의 41mm 축 오차가 실리지 않는다)
    assert meas.fallback
    assert meas.target[2] == pytest.approx(_H[2], abs=1e-9)
    assert abs(meas.target[2] - frag.position[2]) > 0.04, (
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

# ─── 재제시 보정 (look-then-move, 2026-07-29 실물 43mm 이탈 회귀) ──────


def test_represent_offset_perp_only_pure():
    """보정량 = 축 수직 성분의 **수평 투영**만 — 축 방향 밀림(가림 의존)과
    z(마스크 가장자리 depth 번짐이 centroid 를 끌어내림 — 21:11 실측)는
    실리지 않는다."""
    w = (0.0, 0.0, -1.0)
    h_ref = (0.20, 0.10, 0.25)
    det = _aerial_det((0.20 + 0.043, 0.10 - 0.02, 0.25 - 0.05))  # perp+along 혼합
    p = steps.represent_offset(det, h_ref, w)
    assert p[0] == pytest.approx(0.043, abs=1e-9)
    assert p[1] == pytest.approx(-0.02, abs=1e-9)
    assert p[2] == pytest.approx(0.0, abs=1e-9), "축(z) 성분이 보정에 실렸다"
    # 수평 축(봉 수평 제시)에서는 perp 평면에 z 가 포함되지만 — 번짐 오염이라
    # z 는 FK 앵커, 보정은 수평 성분만 (21:11 런: 검출 z 가 봉 바닥보다 아래)
    w_h = (-1.0, 0.0, 0.0)
    det2 = _aerial_det((0.20 - 0.03, 0.10 + 0.04, 0.25 - 0.013))
    p2 = steps.represent_offset(det2, h_ref, w_h)
    assert p2[1] == pytest.approx(0.04, abs=1e-9)
    assert p2[2] == pytest.approx(0.0, abs=1e-9), "번짐 오염 z 가 보정에 실렸다"


async def test_scenario_represent_loop_converges(monkeypatch):
    """실물 20:43/21:11 런 재현 — 재검출이 계획 E 대비 축 수직 40mm 이탈이면
    ① 오차를 world_offset 으로 **제시 전면 재계획** (so101 쪽 게이트 실물 평가)
    ② omx 이동 + 관측 재계획 + 재검출 ③ 수렴 후 수취 진행.
    (1차 구현 "같은 자세 단일 평행이동" 은 omx 가용 밴드 이탈로 IK 전멸 —
    21:11 실물. 겨냥점만 옮기는 옛 코드는 razor-thin 수취 밴드 밖 전멸 — 20:43)"""
    monkeypatch.setattr(steps, "_PRESENT_W_ELEV_DEG", (15.0,))

    def _e_of(wv, off=(0.0, 0.0, 0.0)):
        return tuple(
            _TCP_W[i] + wv[i] * _GEOM.tcp_to_e_m + off[i] for i in range(3)
        )

    def _first_w(off=(0.0, 0.0, 0.0)):
        return next(
            wv for _lb, wv in steps._present_w_candidates(_TCP_W, _BASE_OMX)
            if _e_in_roi(_e_of(wv, off))
        )

    w = _first_w()
    e = _e_of(w)
    # 축 수직·수평 40mm 이탈 (실물: FK 대비 so101 쪽으로 40.3mm — 21:11 trace)
    perp = np.cross(np.asarray(w, dtype=float), [0.0, 0.0, 1.0])
    perp = tuple(perp / np.linalg.norm(perp) * 0.04)
    det_off = _aerial_at(
        perp, along_axis=_GEOM.tcp_to_e_m - steps._OMX_JAW_ALONG_M / 2.0,
        w=w, tcp=_TCP_W,
    )
    # 재계획에서 채택될 w/E — offset 40mm > _REPRESENT_HANG_FIRST_M 이라
    # hang pass 가 먼저, 후보는 수취 밴드 근접순 (production 정렬과 동형)
    w2 = (0.0, 0.0, -1.0)
    cands = frames.rendezvous_candidates(
        _ROI_SO, _ROI_OMX, _BASE_OMX, steps._PRESENT_Z_WORLD,
        limit=steps._PRESENT_LIMIT, prefer_point=steps._RENDEZVOUS_PREFER_XY,
    )
    tcp2 = min(cands, key=lambda t: math.hypot(
        t[0] + perp[0] - steps._RECV_SWEET_XY[0],
        t[1] + perp[1] - steps._RECV_SWEET_XY[1],
    ))
    e2 = tuple(
        tcp2[i] + w2[i] * _GEOM.tcp_to_e_m + perp[i] for i in range(3)
    )  # h_world#2 = FK E + world_offset (실물 추정)
    script = _happy_script(e)
    # 재검출: ① 이탈 → ② 재계획 제시 후 실물 추정점 정착 (+ 수취 refine 1회)
    script[_DETECT_ORIENTED] = [
        DetectOrientedResponse(found=True, candidates=[det_off]),
        DetectOrientedResponse(found=True, candidates=[_aerial_det(e2)]),
        DetectOrientedResponse(found=True, candidates=[_aerial_det(e2)]),
    ]
    # 재계획 resolve 3건: 제시 / 수취 결합 probe / 관측 (so observe 다음에)
    sel = list(script[_SELECT])
    sel[5:5] = [
        ResolveReachableResponse(index=0, solutions=[[0.45] * 5]),
        ResolveReachableResponse(index=0, solutions=[[0.55] * 6, [0.6] * 6]),
        ResolveReachableResponse(index=0, solutions=[[0.5] * 6]),
    ]
    script[_SELECT] = sel
    script[_MOVE_J] = [MoveJResponse()] * 11  # +재제시 이동 +관측 재이동
    script[_READ_STATE] = [_joint_state(_HELD_RAW)] * 5  # +재제시 held 재확인
    snaps = list(script[_TCP_SNAP])
    snaps[2:2] = [
        _tcp(steps.world_to_robot(_TCP_W, _BASE_OMX), [0.4] * 5),  # 재계획 스냅
        _observe_pose(e2)[1],  # 재검출 ② 의 관측 도달 검증
    ]
    script[_TCP_SNAP] = snaps
    ctx = _ctx(script)
    await _module().scenario(ctx, pick_object="orange block")
    # 재제시 이동이 실제로 나갔다 — omx MOVE_J 가 한 번 더
    omx_moves = [c for c in ctx.calls(_MOVE_J) if c["robot_id"] == OMX]
    # home/observe/pick/present/**재제시**/retreat = 6
    assert len(omx_moves) == 6, [c["robot_id"] for c in ctx.calls(_MOVE_J)]
    assert len(ctx.calls(_DETECT_ORIENTED)) == 3
    # 수취 순서 불변식 유지 (so close → so held 판정 → omx release)
    grip_events = [
        (c["robot_id"], c["req"].position_raw == _SPEC.gripper_open_raw)
        for c in ctx.wire.call_log if c["key"] == _GRIP
    ]
    assert grip_events == [
        (OMX, True), (OMX, False), (SO, True), (SO, False), (OMX, True)
    ], grip_events


async def test_scenario_no_represent_when_converged(monkeypatch):
    """이탈 ≤ 임계면 재제시를 안 한다 — happy path 호출 수 불변 (비용 0 경로)."""
    monkeypatch.setattr(steps, "_PRESENT_W_ELEV_DEG", (15.0,))

    def _e_of(wv):
        return tuple(_TCP_W[i] + wv[i] * _GEOM.tcp_to_e_m for i in range(3))

    w = next(
        wv for _lb, wv in steps._present_w_candidates(_TCP_W, _BASE_OMX)
        if _e_in_roi(_e_of(wv))
    )
    ctx = _ctx(_happy_script(_e_of(w)))
    await _module().scenario(ctx, pick_object="orange block")
    assert len(ctx.calls(_DETECT_ORIENTED)) == 2  # 재검출 1 + refine 1
    omx_moves = [c for c in ctx.calls(_MOVE_J) if c["robot_id"] == OMX]
    assert len(omx_moves) == 5  # home/observe/pick/present/retreat — 재제시 없음
# ─── 수취 servo (look-then-move 수렴, 2026-07-29 22:06 허공 물기 회귀) ──


async def test_receive_servo_iterates_until_converged():
    """pre 측정이 흔들리면 **보정된 pre 로 재정렬 후 재측정** — 수렴(≤eps)
    후에만 진입한다. 옛 refine-1-tick+open-loop 진입은 연속 두 측정이
    12.5mm 어긋나는 산포(점군 52)에서 허공을 물었다 (22:06 실물)."""
    w = _W_HANG
    present = _present(w)
    e = present.h_world
    # 측정 시퀀스: ① lateral 15mm 이탈 → ② 같은 곳 재확인(수렴) → 진입
    off_det = _aerial_at(
        (0.015, 0.0, 0.0), along_axis=_GEOM.tcp_to_e_m - steps._OMX_JAW_ALONG_M / 2.0,
        w=w, tcp=_TCP_W,
    )
    ctx = _ctx({
        _DETECT_ORIENTED: [
            DetectOrientedResponse(found=True, candidates=[off_det]),
            DetectOrientedResponse(found=True, candidates=[off_det]),
        ],
        _MOVE_J: [MoveJResponse()],
        # servo 재정렬 pre / 진입 / withdraw = 3
        _MOVE_L: [MoveLResponse()] * 3,
        _GRIP: [SetGripperResponse()] * 2,  # so close + omx release
        _READ_STATE: [_joint_state(_HELD_RAW)] * 2,
    })
    plan = steps.ReceivePlan(
        sols=[[0.1] * 6, [0.2] * 6], quat=(0.0, 0.0, 0.0, 1.0),
        target=e, omx_joints=[0.4] * 5, pre_clear_m=0.03,
    )
    await steps.receive(ctx, SO, OMX, plan, "orange block", present, _GEOM)
    # 측정 2회 (이탈 → 수렴), 재정렬 pre 이동 1회 + 진입 + withdraw
    assert len(ctx.calls(_DETECT_ORIENTED)) == 2
    assert len(ctx.calls(_MOVE_L)) == 3
    # 진입 겨냥점 = 보정된 target (두 번째 MOVE_L)
    entry = ctx.calls(_MOVE_L)[1]["req"].target.position
    cen = steps.detection_centroid(off_det)
    assert entry[0] == pytest.approx(cen[0], abs=1e-6)
    assert entry[1] == pytest.approx(cen[1], abs=1e-6)
    assert entry[2] == pytest.approx(e[2], abs=1e-9), "z 는 FK 앵커여야"


async def test_receive_servo_zero_cost_when_converged():
    """첫 측정이 이미 수렴이면 재정렬 이동 없음 — 옛 happy path 와 호출 동수."""
    w = _W_HANG
    present = _present(w)
    e = present.h_world
    on_det = _aerial_at(
        (0.0, 0.0, 0.0), along_axis=_GEOM.tcp_to_e_m - steps._OMX_JAW_ALONG_M / 2.0,
        w=w, tcp=_TCP_W,
    )
    ctx = _ctx({
        _DETECT_ORIENTED: [DetectOrientedResponse(found=True, candidates=[on_det])],
        _MOVE_J: [MoveJResponse()],
        _MOVE_L: [MoveLResponse()] * 2,  # 진입 + withdraw
        _GRIP: [SetGripperResponse()] * 2,
        _READ_STATE: [_joint_state(_HELD_RAW)] * 2,
    })
    plan = steps.ReceivePlan(
        sols=[[0.1] * 6, [0.2] * 6], quat=(0.0, 0.0, 0.0, 1.0),
        target=e, omx_joints=[0.4] * 5, pre_clear_m=0.03,
    )
    await steps.receive(ctx, SO, OMX, plan, "orange block", present, _GEOM)
    assert len(ctx.calls(_DETECT_ORIENTED)) == 1
    assert len(ctx.calls(_MOVE_L)) == 2


# ─── 수취 협상 (2026-07-30 — "전멸=종료" 폐기, 발견 시나리오 그대로) ────


async def test_find_receive_shift_returns_first_alive_delta():
    """전멸 겨냥점 주변 격자에서 **수취 해가 사는 첫 δ** (작은 이동 우선) —
    δ 별 resolve 1콜, 실패는 다음 δ 로. 반환 δ 는 협상 반경 안."""
    ctx = _ctx({
        _SELECT: [
            ResolveReachableResponse(index=-1, message="전멸"),
            ResolveReachableResponse(index=-1, message="전멸"),
            ResolveReachableResponse(index=0, solutions=[[0.1] * 6, [0.2] * 6]),
        ],
    })
    delta = await steps.find_receive_shift(ctx, SO, _meas())
    assert delta is not None
    assert len(ctx.calls(_SELECT)) == 3  # 두 δ 실패 → 셋째 δ 채택
    norm = math.sqrt(sum(v * v for v in delta))
    assert 0.0 < norm <= steps._NEGOTIATE_RANGE_M + 1e-9


async def test_find_receive_shift_none_when_all_dead():
    """주변 전부 죽음 = None (호출자가 명시 실패) — 무한 탐색 금지."""
    ctx = _ctx({
        _SELECT: [ResolveReachableResponse(index=-1, message="전멸")] * 200,
    })
    assert await steps.find_receive_shift(ctx, SO, _meas()) is None
    assert 0 < len(ctx.calls(_SELECT)) <= 200


async def test_scenario_negotiates_when_receive_dead(monkeypatch):
    """협상 배선 잠금 — 수취 계획 전멸 → 이동 제안 → omx 재배치(자세 불변 평행
    이동) → 재관측/재검출/재실측 → 재계획 성공 → 수취. 수취 순서 불변식
    (so close → held 판정 → omx release)도 협상 경로에서 유지."""
    monkeypatch.setattr(steps, "_PRESENT_W_ELEV_DEG", (15.0,))

    def _e_of(wv):
        return tuple(_TCP_W[i] + wv[i] * _GEOM.tcp_to_e_m for i in range(3))

    w = next(
        wv for _lb, wv in steps._present_w_candidates(_TCP_W, _BASE_OMX)
        if _e_in_roi(_e_of(wv))
    )
    e = _e_of(w)
    delta = (0.02, 0.0, 0.0)

    async def _fixed_shift(ctx_, so, meas, trace=None):  # noqa: ANN001, ANN202
        return delta

    # find_receive_shift 는 단위 테스트가 잠근다 — 여기선 배선만 (결정론 δ)
    monkeypatch.setattr(steps, "find_receive_shift", _fixed_shift)
    e2 = tuple(e[i] + delta[i] for i in range(3))
    script = _happy_script(e)
    sel = list(script[_SELECT])
    # 수취 계획 #1 전멸 → (shift 는 고정) → omx_nudge resolve → 관측 #2 →
    # 수취 계획 #2 성공
    sel[5:] = [
        ResolveReachableResponse(index=-1, message="자세 IK 실패 전멸"),
        ResolveReachableResponse(index=0, solutions=[[0.42] * 5]),  # nudge
        ResolveReachableResponse(index=0, solutions=[[0.5] * 6]),  # 관측 #2
        ResolveReachableResponse(index=0, solutions=[[0.6] * 6, [0.65] * 6]),
    ]
    script[_SELECT] = sel
    script[_DETECT_ORIENTED] = [
        DetectOrientedResponse(found=True, candidates=[_aerial_det(_TCP_W)]),
        DetectOrientedResponse(
            found=True,
            candidates=[_aerial_det(tuple(
                _TCP_W[i] + delta[i] for i in range(3)
            ))],
        ),
        DetectOrientedResponse(
            found=True,
            candidates=[_aerial_det(tuple(
                _TCP_W[i] + delta[i] for i in range(3)
            ))],
        ),
    ]
    script[_MOVE_J] = [MoveJResponse()] * 11  # +nudge 이동 +관측 재이동
    script[_READ_STATE] = [_joint_state(_HELD_RAW)] * 5  # +nudge held 재확인
    snaps = list(script[_TCP_SNAP])
    tcp2_omx = steps.world_to_robot(
        tuple(_TCP_W[i] + delta[i] for i in range(3)), _BASE_OMX
    )
    # 순서: present / observe#1 / measure#1 / plan#1 / nudge(so,omx) /
    #       observe#2 / measure#2 / plan#2 / (retreat: checker=None 생략)
    snaps[4:4] = [
        _tcp((0.2, 0.1, 0.1), [0.6] * 6),  # nudge so 스냅
        _tcp(steps.world_to_robot(_TCP_W, _BASE_OMX), [0.4] * 5),  # nudge omx
        _observe_pose(e2)[1],  # 관측 #2 도달 검증
        _tcp(tcp2_omx, [0.42] * 5),  # measure #2 (이동 후 FK)
        _tcp(tcp2_omx, [0.42] * 5),  # plan #2
    ]
    script[_TCP_SNAP] = snaps
    ctx = _ctx(script)
    await _module().scenario(ctx, pick_object="orange block")
    # omx 이동: home/observe/pick/present/**nudge**/retreat = 6
    omx_moves = [c for c in ctx.calls(_MOVE_J) if c["robot_id"] == OMX]
    assert len(omx_moves) == 6, [c["robot_id"] for c in ctx.calls(_MOVE_J)]
    # 수취 순서 불변식 (so close → held → omx release)
    grip_events = [
        (c["robot_id"], c["req"].position_raw == _SPEC.gripper_open_raw)
        for c in ctx.wire.call_log if c["key"] == _GRIP
    ]
    assert grip_events == [
        (OMX, True), (OMX, False), (SO, True), (SO, False), (OMX, True)
    ], grip_events


async def test_scenario_negotiation_exhausted_fails_explicitly(monkeypatch):
    """협상 상한 소진 / 이동 제안 부재 = **명시 실패** (침묵 진행 금지)."""
    monkeypatch.setattr(steps, "_PRESENT_W_ELEV_DEG", (15.0,))

    async def _no_shift(ctx_, so, meas, trace=None):  # noqa: ANN001, ANN202
        return None

    monkeypatch.setattr(steps, "find_receive_shift", _no_shift)

    def _e_of(wv):
        return tuple(_TCP_W[i] + wv[i] * _GEOM.tcp_to_e_m for i in range(3))

    w = next(
        wv for _lb, wv in steps._present_w_candidates(_TCP_W, _BASE_OMX)
        if _e_in_roi(_e_of(wv))
    )
    script = _happy_script(_e_of(w))
    sel = list(script[_SELECT])
    sel[5:] = [ResolveReachableResponse(index=-1, message="전멸")]
    script[_SELECT] = sel
    ctx = _ctx(script)
    with pytest.raises(NoReachableGrasp, match="전멸"):
        await _module().scenario(ctx, pick_object="orange block")
