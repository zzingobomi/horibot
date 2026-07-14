"""Pick & Place task 테스트 — 순수 함수(geometry) / FakeContext 시나리오 / module wire.

의미 (뒤집으면 회귀): adaptive 관측이 "파지 성립" 을 심판으로 쓰지 않고 height
게이트/고정 뷰 수로 회귀 / 파지 후보가 관측 표면(antipodal)이 아니라 footprint
추측에서 나옴 / 뷰 이동이 resolve 스크리닝(floor+장애물+경로) 없이 naive MoveJ /
place 분기가 pick-only 에서 실행 / 실패가 침묵 성공 / place 검출 실패 시 release
(물체 낙하) / RUN 동시 실행 허용 / 도달 전멸(-1) 침묵 통과 / 관측 전멸이 맹목
파지로 이어짐 ("안전 파지 불가" 명시 실패 — §10.4-3).

antipodal/뷰 수식 순수 계산 잠금은 test_antipodal.py.
"""

from __future__ import annotations

from datetime import UTC, datetime

import numpy as np
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
    FuseOrientedResponse,
    OrientedDetection,
)
from modules.motion.contract import (
    Motion,
    MoveJResponse,
    MoveLResponse,
    ResolveReachableResponse,
    StopResponse,
)
from modules.motor.contract import Motor, SetGripperResponse
from modules.tasks.core.contract import (
    ControlRequest,
    PreviewRequest,
    TaskState,
    TaskStatus,
    ToggleBreakpointRequest,
)
from modules.tasks.core.errors import DetectionNotFound, NoReachableGrasp, TaskError
from modules.tasks.core.fake import FakeContext
from modules.tasks.core.spec import TaskRobotSpec
from modules.tasks.pick_and_place import geometry, steps
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
_CAL_BUNDLE = str(Calibration.Service.SNAPSHOT_BUNDLE)
_SELECT = str(Motion.Service.RESOLVE_REACHABLE)
_MOVE_J = str(Motion.Service.MOVE_J)
_MOVE_L = str(Motion.Service.MOVE_L)
_GRIP = str(Motor.Service.SET_GRIPPER)
_LIST_WP = str(Waypoint.Service.LIST)
_LIST_GROUPS = str(Waypoint.Service.LIST_GROUPS)
_LIST_MEMBERS = str(Waypoint.Service.LIST_GROUP_MEMBERS)
_TS = datetime.fromtimestamp(0, UTC)

_HOME_JOINTS = [0.0, 0.5, -1.0, 0.0, 0.5, 1.5]  # 티칭된 home (임의 유효값)


def _home_record() -> WaypointRecord:
    return WaypointRecord(
        id=99, robot_id=_BOT, name="home",
        joint_values=list(_HOME_JOINTS), joint_names=[], created_at=_TS,
    )


def _home_responses() -> dict:
    """'home' waypoint canned 응답 — 시나리오가 맨 앞에서 조회."""
    return {_LIST_WP: [ListWaypointsResponse(waypoints=[_home_record()])] * 2}


def _resolve_ok(index: int = 0) -> ResolveReachableResponse:
    """가용 응답 — solutions = [pre 해, grasp 해] (실행부가 [0] 을 관절 이동에 씀)."""
    return ResolveReachableResponse(
        index=index, solutions=[[0.1] * 6, [0.2] * 6]
    )


def _hand_eye_bundle() -> CalibrationBundle:
    return CalibrationBundle(
        robot_id=_BOT,
        hand_eye=HandEyeResultRecord(
            run_id=1, robot_id=_BOT, created_at=_TS, is_active=True,
            result_data=HandEyeResultData(
                R_cam2gripper=[[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                t_cam2gripper=[[0.0], [0.0], [0.05]],
                method="TSAI",
            ),
        ),
    )


def _cloud(both_sides: bool = True) -> list[tuple[float, float, float]]:
    """관측 점군 합성 — 옆면(±x, 간격 2.2cm) + 윗면 (중심 0.2,0.05, top z=0.023).

    both_sides=False = 단일 뷰 흉내 (마주 보는 면 중 먼 쪽 가림 → antipodal
    0쌍 — §10.3-B). 파지 성립 여부가 이 점군에서 **실제 antipodal 계산**으로
    갈린다 (canned bool 아님 — 코드 경로 그대로).
    """
    ys = np.linspace(0.05 - 0.011, 0.05 + 0.011, 12)
    zs = np.linspace(0.0, 0.023, 12)
    xs = np.linspace(0.2 - 0.011, 0.2 + 0.011, 12)
    pts: list[tuple[float, float, float]] = []
    for x in ((0.211, 0.189) if both_sides else (0.211,)):
        for y in ys:
            for z in zs:
                pts.append((float(x), float(y), float(z)))
    for x in xs:
        for y in ys:
            pts.append((float(x), float(y), 0.023))
    return pts


def _det(
    score: float = 0.9,
    height: float = 0.023,
    position: tuple[float, float, float] = (0.2, 0.05, 0.023),
    base_z: float = 0.0,
    footprint: tuple[float, float] = (0.023, 0.022),
    grasp_yaw: float = 0.3,
    points: list[tuple[float, float, float]] | None = None,
) -> OrientedDetection:
    return OrientedDetection(
        prompt="cube", position=position, score=score, base_z=base_z,
        height=height, grasp_yaw=grasp_yaw, footprint=footprint, points=points,
    )


def _fuse_full() -> FuseOrientedResponse:
    """파지가 서는 융합 결과 — 양쪽 옆면 점군 (antipodal 쌍 생성)."""
    return FuseOrientedResponse(candidates=[_det(points=_cloud(True))])


def _fuse_half() -> FuseOrientedResponse:
    """아직 안 서는 융합 결과 — 한쪽 옆면만 (antipodal 0쌍 → 뷰 더 필요)."""
    return FuseOrientedResponse(candidates=[_det(points=_cloud(False))])


def _search_responses(n_members: int = 1) -> dict:
    """search 그룹 canned 응답 ('search' 그룹 + n_members 자세)."""
    grp = ListGroupsResponse(
        groups=[WaypointGroupRecord(id=1, robot_id=_BOT, name="search")]
    )
    members = ListGroupMembersResponse(
        waypoints=[
            WaypointRecord(
                id=i + 1,
                robot_id=_BOT,
                name=f"s{i}",
                joint_values=[0.0] * 6,
                joint_names=[],
                created_at=_TS,
            )
            for i in range(n_members)
        ]
    )
    return {**_home_responses(), _LIST_GROUPS: [grp] * 4, _LIST_MEMBERS: [members] * 4}


@pytest.fixture(autouse=True)
def _no_settle(monkeypatch: pytest.MonkeyPatch):
    monkeypatch.setattr(steps, "_GRIPPER_SETTLE_S", 0.0)  # 테스트 즉시 진행
    monkeypatch.setattr(steps, "_SEARCH_SETTLE_S", 0.0)  # 스윕 정착 대기 제거


@pytest.fixture
def _two_view_dirs(monkeypatch: pytest.MonkeyPatch):
    """뷰 탐색축을 2방향으로 축소 — 전멸 시나리오의 canned 응답 수를 억제.

    (실 탐색축 순서/내용 잠금은 test_antipodal.test_view_directions_*.)"""
    monkeypatch.setattr(geometry, "_VIEW_RADII_M", (0.16,))
    monkeypatch.setattr(geometry, "_VIEW_ELEVATIONS_DEG", (55.0,))
    monkeypatch.setattr(geometry, "_VIEW_AZIMUTH_OFFSETS_DEG", (0.0, 120.0))


# ─── geometry 순수 함수 ──────────────────────────────────────────────


def test_select_target_by_score():
    """coarse 선택 = score 만 — height prior/하드게이트 없음 (§10.4-6 폐기,
    관측 충분성의 심판은 파지 성립)."""
    best = geometry.select_target_by_score(
        [_det(score=0.5), _det(score=0.9), _det(score=0.2, height=0.0)],
        prompt="cube",
    )
    assert best.score == 0.9

    with pytest.raises(DetectionNotFound, match="검출 0건"):
        geometry.select_target_by_score([], prompt="cube")


def test_plan_place_release_height():
    spot = _det(position=(0.25, -0.05, 0.04), height=0.04)
    held = _det(height=0.023)
    pplan = geometry.plan_place(spot, held=held, lateral=0.008)
    # release z = spot 상면 + held/2 + 여유(0.005) — 물체 바닥이 상면에 닿게
    assert pplan[0].place[2] == pytest.approx(0.04 + 0.023 / 2 + 0.005)
    # tilt=0 pre = place 에서 접근축 후방 (0.06 + held 반높이)
    assert pplan[0].pre[2] == pytest.approx(pplan[0].place[2] + 0.06 + 0.023 / 2)
    assert len(pplan) == 13 * 2 * 2


# ─── adaptive 관측·파지 성립 (step 직접 — FakeContext) ────────────────


async def test_observe_seeds_from_sweep_and_stands_without_views():
    """검색 스윕 관측(여러 자세에서 같은 물체)이 멀티뷰 시드 — 융합 점군에서
    파지가 서면 **추가 뷰 이동 0** 으로 계획 완료 (§10.3-G adaptive 정지)."""
    coarse = _det()
    near = _det(score=0.7, position=(0.21, 0.06, 0.023))  # 1.4cm — 같은 물체
    far = _det(score=0.99, position=(0.4, 0.3, 0.02), points=_cloud(False))
    ctx = FakeContext(
        robots=[_BOT],
        specs={_BOT: _SPEC},
        service_script={
            _CAL_BUNDLE: [_hand_eye_bundle()],
            _FUSE: [_fuse_full()],
            _SELECT: [_resolve_ok()],
        },
    )
    fused, grasp, pre = await steps.observe_and_plan_grasp(
        ctx, _BOT, [coarse, near, far], coarse, "white cube", _home_record()
    )
    assert pre == [0.1] * 6  # resolve 해 그대로 (재계산 금지 — §5.5)
    assert ctx.calls(_MOVE_J) == []  # 추가 뷰 이동 0

    # 융합 입력 = coarse + 근접 관측만 (far 는 다른 물체 — 배제)
    fuse_req = ctx.calls(_FUSE)[0]["req"]
    assert len(fuse_req.candidates) == 2
    assert fuse_req.candidates[0] is coarse

    # 파지 resolve 게이트 계약 (§10.4-3): 직선 경로 + 바닥 + 그리퍼 벌림 충돌
    # + home→pre 관절 경로, 장애물 = 융합 점군(+이웃)
    sel = ctx.calls(_SELECT)[0]["req"]
    assert sel.linear is True
    assert sel.floor_z == pytest.approx(0.0 - 0.005)
    assert sel.gripper_open is True
    assert sel.path_from == _HOME_JOINTS
    assert sel.obstacle_points  # 융합 점군이 장애물로 들어감
    # far(이웃 반경 0.15m 밖 아님 — 0.32m 밖이라 제외) 점군은 안 섞임
    assert len(sel.obstacle_points) == len(_cloud(True))


async def test_observe_adds_views_until_grasp_stands():
    """단일 뷰 점군(쌍 0)으로 시작 → 뷰 방향 1 도달 불가(스킵) → 뷰 방향 2 에서
    관측 추가 → 융합 점군에 마주 보는 면이 생겨 파지 성립. 뷰 이동은 home 경유
    + resolve 가 반환한 관절 해 (§10.4-4 naive MoveJ 금지)."""
    coarse = _det(points=_cloud(False))
    near = _det(score=0.7, position=(0.21, 0.06, 0.023), points=_cloud(False))
    view_sol = [0.3] * 6
    ctx = FakeContext(
        robots=[_BOT],
        specs={_BOT: _SPEC},
        service_script={
            _CAL_BUNDLE: [_hand_eye_bundle()],
            _FUSE: [_fuse_half(), _fuse_full()],  # 시드=쌍0 → 뷰 추가 후 성립
            _SELECT: [
                ResolveReachableResponse(index=-1, message="뷰 도달 불가"),
                ResolveReachableResponse(index=2, solutions=[view_sol]),
                _resolve_ok(),  # 파지 성립
            ],
            _MOVE_J: [MoveJResponse()] * 2,  # home 경유 + 뷰 이동
            _DETECT: [DetectOrientedResponse(found=True, candidates=[near])],
        },
    )
    fused, grasp, pre = await steps.observe_and_plan_grasp(
        ctx, _BOT, [coarse], coarse, "white cube", _home_record()
    )
    assert pre == [0.1] * 6
    # 호출 순서: 융합(쌍0 — resolve 없이 반려) → 뷰 resolve ×2 → home → 뷰
    # MoveJ → 검출 → 융합 → 파지 resolve
    assert ctx.keys() == [
        _CAL_BUNDLE, _FUSE, _SELECT, _SELECT, _MOVE_J, _MOVE_J, _DETECT,
        _FUSE, _SELECT,
    ]
    # 뷰 resolve 계약: roll 변형 그룹 (그룹당 pose 1) + floor + 장애물(관측 점군)
    # + home 경로 게이트. 파지가 아니므로 gripper_open/linear 없음.
    view_req = ctx.calls(_SELECT)[0]["req"]
    assert len(view_req.groups) == 6 and all(len(g) == 1 for g in view_req.groups)
    assert view_req.floor_z == pytest.approx(0.0 - 0.005)
    assert view_req.path_from == _HOME_JOINTS
    assert view_req.obstacle_points and view_req.gripper_open is False
    assert view_req.linear is False
    # 뷰 이동 = home 경유 후 resolve 해 그대로
    mjs = [c["req"].target.joints for c in ctx.calls(_MOVE_J)]
    assert mjs == [_HOME_JOINTS, view_sol]
    # 융합 #2 입력 = coarse + 새 관측
    fuse2 = ctx.calls(_FUSE)[1]["req"]
    assert len(fuse2.candidates) == 2 and fuse2.candidates[1] is near


async def test_observe_exhausted_fails_explicitly(_two_view_dirs):
    """모든 뷰 방향이 도달/안전 불가 + 파지 미성립 = "안전 파지 불가" 명시 실패
    (§10.4-3 — 맹목 파지 금지). 파지·실행 모션 0."""
    coarse = _det(points=_cloud(False))
    ctx = FakeContext(
        robots=[_BOT],
        specs={_BOT: _SPEC},
        service_script={
            _CAL_BUNDLE: [_hand_eye_bundle()],
            _FUSE: [_fuse_half()],
            _SELECT: [ResolveReachableResponse(index=-1)] * 2,  # 뷰 2방향 전멸
        },
    )
    with pytest.raises(NoReachableGrasp, match="안전 파지 불가"):
        await steps.observe_and_plan_grasp(
            ctx, _BOT, [coarse], coarse, "white cube", _home_record()
        )
    assert ctx.calls(_MOVE_J) == []  # 도달 불가 뷰는 이동 자체가 없다


async def test_observe_grasp_gate_exhausted_keeps_observing_then_fails(
    _two_view_dirs,
):
    """antipodal 쌍은 있으나 resolve 전멸(-1) = 부정 데이터 → 관측을 더 시도,
    끝까지 안 서면 명시 실패. -1 을 침묵 통과(맹목 파지)하면 회귀."""
    coarse = _det(points=_cloud(True))
    ctx = FakeContext(
        robots=[_BOT],
        specs={_BOT: _SPEC},
        service_script={
            _CAL_BUNDLE: [_hand_eye_bundle()],
            _FUSE: [_fuse_full()],
            _SELECT: [
                ResolveReachableResponse(index=-1, message="전멸"),  # 파지 게이트
                ResolveReachableResponse(index=-1),  # 뷰 방향 1
                ResolveReachableResponse(index=-1),  # 뷰 방향 2
            ],
        },
    )
    with pytest.raises(NoReachableGrasp, match="안전 파지 불가"):
        await steps.observe_and_plan_grasp(
            ctx, _BOT, [coarse], coarse, "white cube", _home_record()
        )
    # 첫 resolve 는 파지 시도 (gripper_open) — 이후는 뷰 스크리닝
    sels = ctx.calls(_SELECT)
    assert sels[0]["req"].gripper_open is True
    assert all(not s["req"].gripper_open for s in sels[1:])


async def test_observe_requires_hand_eye():
    ctx = FakeContext(
        robots=[_BOT],
        specs={_BOT: _SPEC},
        service_script={_CAL_BUNDLE: [CalibrationBundle(robot_id=_BOT)]},
    )
    with pytest.raises(TaskError, match="hand_eye"):
        await steps.observe_and_plan_grasp(
            ctx, _BOT, [_det()], _det(), "white cube", _home_record()
        )
    assert ctx.calls(_MOVE_J) == []  # 캘 없으면 모션 0


async def test_fuse_target_returns_none_off_cluster():
    """융합 결과에 coarse 근처 군집이 없으면 None (부정 데이터 — 관측을 더
    쌓으라는 신호, 침묵 오매칭 금지)."""
    ctx = FakeContext(
        robots=[_BOT], specs={_BOT: _SPEC},
        service_script={
            _FUSE: [
                FuseOrientedResponse(candidates=[_det(position=(0.5, 0.3, 0.02))])
            ]
        },
    )
    assert await steps.fuse_target(ctx, [_det()], "white cube") is None


# ─── 시나리오 (FakeContext — 하드웨어/wire 없음, step 은 게이트 없이 실행) ──


def _module_for_scenario() -> PickAndPlaceModule:
    class _Rt:
        def publish(self, k: str, e: BaseModel) -> None: ...
        async def call(self, *a, **kw): ...  # noqa: ANN002, ANN003, ANN201

    return PickAndPlaceModule(_Rt(), {})  # type: ignore[arg-type]


def _pick_script(**overrides) -> dict:
    """pick 경로 성공 스크립트 (search 자세 1개, 스윕 관측 융합만으로 파지 성립
    — 추가 뷰 0). 시나리오 = home 조회 → 스윕 → 융합 → 파지 성립 → 실행."""
    script = {
        **_search_responses(),
        _CAL_BUNDLE: [_hand_eye_bundle()] * 2,
        _DETECT: [
            DetectOrientedResponse(
                found=True, candidates=[_det(points=_cloud(False))]
            )
        ] * 4,
        _FUSE: [_fuse_full()] * 2,
        _SELECT: [_resolve_ok()] * 4,
        # MoveJ 소비 순서: 스윕 1 → 실행 (home/pre/home)
        _MOVE_J: [MoveJResponse()] * 8,
        _MOVE_L: [MoveLResponse()] * 4,  # advance/withdraw (+ insert/retreat)
        _GRIP: [SetGripperResponse()] * 3,  # open + close (+ release)
    }
    script.update(overrides)
    return script


async def test_scenario_pick_only_sequence():
    mod = _module_for_scenario()
    ctx = FakeContext(robots=[_BOT], specs={_BOT: _SPEC}, service_script=_pick_script())

    await mod.scenario(ctx, pick_object="white cube")

    # 호출 순서 = home 조회 → 검색 스윕(그룹 조회→자세 MoveJ→검출) → 관측·파지
    # (hand_eye 조회 → 융합 → 파지 resolve — 스윕 관측만으로 성립, 뷰 이동 0) →
    # 실행: home 경유 → pre 접근(관절 해) → open → 진입 → close → 후퇴 → home.
    assert ctx.keys() == [
        _LIST_WP,  # home waypoint 조회 (모션 0)
        _LIST_GROUPS, _LIST_MEMBERS, _MOVE_J, _DETECT,  # 스윕
        _CAL_BUNDLE, _FUSE, _SELECT,  # 관측 융합 + 파지 성립 (adaptive 정지)
        _MOVE_J, _MOVE_J, _GRIP, _MOVE_L, _GRIP, _MOVE_L, _MOVE_J,  # 실행
    ]
    # place 분기 안 탐 (detect 1회뿐) + 든 채 종료 (마지막 gripper = close raw)
    grips = [c["req"].position_raw for c in ctx.calls(_GRIP)]
    assert grips == [_SPEC.gripper_open_raw, _SPEC.gripper_close_raw]

    # resolve 게이트 계약: floor_z = 융합 base_z − 버퍼, linear + 그리퍼 벌림
    # 충돌 + home 경로 게이트 활성 (§10.4-3)
    sel = ctx.calls(_SELECT)[0]["req"]
    assert sel.linear is True
    assert sel.floor_z == pytest.approx(0.0 - 0.005)
    assert sel.gripper_open is True and sel.path_from == _HOME_JOINTS
    assert sel.obstacle_points
    # pre 접근 = resolve 가 반환한 관절 해 그대로 (실행부 IK 재계산 금지 — §5.5).
    # MoveJ 순서: [스윕, home, pre, home] — pre 는 index 2.
    pre_req = ctx.calls(_MOVE_J)[2]["req"]
    assert pre_req.target.kind == "joint" and pre_req.target.joints == [0.1] * 6
    # home 경유 = 티칭 waypoint 관절값
    home_req = ctx.calls(_MOVE_J)[1]["req"]
    assert home_req.target.joints == _HOME_JOINTS


async def test_scenario_with_place_branch():
    mod = _module_for_scenario()
    ctx = FakeContext(
        robots=[_BOT],
        specs={_BOT: _SPEC},
        service_script={
            **_search_responses(),
            _CAL_BUNDLE: [_hand_eye_bundle()] * 2,
            _FUSE: [_fuse_full()] * 2,
            _DETECT: [
                DetectOrientedResponse(  # pick 스윕
                    found=True, candidates=[_det(points=_cloud(False))]
                ),
                DetectOrientedResponse(  # place 스윕
                    found=True,
                    candidates=[_det(position=(0.25, -0.05, 0.04), height=0.04)],
                ),
            ],
            _SELECT: [_resolve_ok()] * 2,
            # 스윕(pick) → 스윕(place) + 실행 (home×3 + pre×2)
            _MOVE_J: [MoveJResponse()] * 8,
            _MOVE_L: [MoveLResponse()] * 4,  # advance/withdraw + insert/retreat
            _GRIP: [SetGripperResponse()] * 3,  # open/close + release
        },
    )

    await mod.scenario(ctx, pick_object="white cube", place_object="red box")

    grips = [c["req"].position_raw for c in ctx.calls(_GRIP)]
    assert grips == [
        _SPEC.gripper_open_raw, _SPEC.gripper_close_raw,
        _SPEC.gripper_open_raw,  # 마지막 open = release
    ]
    assert len(ctx.calls(_MOVE_L)) == 4
    # 관절 이동 = 스윕×2 + home 경유×3 (pick 진입 전 / 든 채 이송 / place 후)
    # + pre 접근×2 = 7
    assert len(ctx.calls(_MOVE_J)) == 7
    # 적치 resolve 도 home 경로 게이트 (execute_place 가 home→pre MoveJ 계약)
    place_sel = ctx.calls(_SELECT)[1]["req"]
    assert place_sel.path_from == _HOME_JOINTS and place_sel.linear is True
    # #2 불변식: 집기·놓기 도달성(RESOLVE) **둘 다** 끝난 뒤에야 첫 파지(GRIP)와
    # 실행 모션(MOVE_L)이 나간다 — 못 놓을 물체를 집는 일이 없도록. (검색 스윕은
    # 파지 아닌 관측이라 그 전에 MoveJ 가 있는 건 정상.)
    keys = ctx.keys()
    assert keys[: keys.index(_GRIP)].count(_SELECT) == 2
    assert keys[: keys.index(_MOVE_L)].count(_SELECT) == 2


async def test_search_sweep_accumulates_and_selects_best_score():
    """검색 원리 (옛 SearchWaypointGroup + SelectTarget 포팅): search 자세를 **전부**
    돌며 후보를 **누적**하고, 첫 자세에서 안 멈추고 **누적 전체의 최고 score** 를
    고른다. (뒤집으면 = 첫 자세서 멈추거나 pose별 최선만 보는 회귀.)"""
    ctx = FakeContext(
        robots=[_BOT],
        specs={_BOT: _SPEC},
        service_script={
            **_search_responses(n_members=2),  # 검색 자세 2개
            _CAL_BUNDLE: [_hand_eye_bundle()],
            _FUSE: [_fuse_full()],
            _DETECT: [
                DetectOrientedResponse(found=True, candidates=[_det(score=0.4)]),
                DetectOrientedResponse(found=True, candidates=[_det(score=0.95)]),
            ],
            _MOVE_J: [MoveJResponse()] * 2,  # 스윕 자세 2곳
            _SELECT: [_resolve_ok()],
        },
    )
    target, _grasp, _pre = await steps.plan_pick(
        ctx, _BOT, "white cube", _home_record()
    )

    assert target.score == 0.9  # 융합 결과 (canned) — 실패 아님이 요점
    # coarse(관측 융합 입력의 첫 항목) = 누적 전체 최고 score — 첫 자세(0.4)서
    # 안 멈춤. 같은 물체의 저score 관측도 융합 입력에 들어간다 (공짜 멀티뷰 시드).
    fuse_req = ctx.calls(_FUSE)[0]["req"]
    assert fuse_req.candidates[0].score == 0.95
    assert len(fuse_req.candidates) == 2
    assert len(ctx.calls(_MOVE_J)) == 2  # 스윕만 — 추가 뷰 이동 0
    assert len(ctx.calls(_DETECT)) == 2  # 스윕 자세마다 검출


async def test_search_group_missing_fails_explicitly():
    """search 그룹 없음 = 명시적 실패 (침묵 단일-뷰 폴백 금지 — 사용자가 관측 자세를
    티칭해야 함)."""
    ctx = FakeContext(
        robots=[_BOT],
        specs={_BOT: _SPEC},
        service_script={
            **_home_responses(),
            _LIST_GROUPS: [ListGroupsResponse(groups=[])],  # search 그룹 없음
        },
    )
    with pytest.raises(TaskError, match="search"):
        await mod_scenario_run(ctx)
    assert ctx.calls(_MOVE_J) == []  # 그룹 없으면 아무 데도 안 감


async def test_scenario_home_waypoint_missing_fails_before_any_motion():
    """'home' waypoint 없음 = 시나리오 맨 앞(모션 0)에서 명시적 실패 + 티칭 안내.
    검색 스윕조차 안 나간다 (실행 중간에 home 이 없어서 멈추는 corrupt 방지)."""
    ctx = FakeContext(
        robots=[_BOT],
        specs={_BOT: _SPEC},
        service_script={_LIST_WP: [ListWaypointsResponse(waypoints=[])]},
    )
    with pytest.raises(TaskError, match="home"):
        await mod_scenario_run(ctx)
    assert ctx.calls(_MOVE_J) == []  # 어떤 모션도 안 나감
    assert ctx.calls(_LIST_GROUPS) == []  # 검색 스윕 진입 전에 실패


async def mod_scenario_run(ctx: FakeContext) -> None:
    await _module_for_scenario().scenario(ctx, pick_object="white cube")


async def test_scenario_detect_fail_raises_after_search():
    """검색 스윕을 다 돌아도 후보 0 → DetectionNotFound. 스윕(관측 MoveJ)은 돌지만
    파지(GRIP)·실행 모션은 0 (아무것도 안 집음)."""
    mod = _module_for_scenario()
    ctx = FakeContext(
        robots=[_BOT], specs={_BOT: _SPEC},
        service_script={
            **_search_responses(),
            _DETECT: [DetectOrientedResponse(found=False, candidates=[])] * 4,
            _MOVE_J: [MoveJResponse()] * 4,  # 검색 스윕
        },
    )
    with pytest.raises(DetectionNotFound):
        await mod.scenario(ctx, pick_object="white cube")
    assert ctx.calls(_GRIP) == []  # 검출 실패면 파지 0
    assert ctx.calls(_MOVE_L) == []  # 실행 모션 0


async def test_scenario_ik_exhausted_raises(_two_view_dirs):
    """RESOLVE_REACHABLE 의 -1 은 데이터 — 파지가 끝내 안 서면 step 이 치명 판정
    (침묵 -1 통과 금지). 파지 전에 실패 (GRIP·MOVE_L 0)."""
    mod = _module_for_scenario()
    ctx = FakeContext(
        robots=[_BOT], specs={_BOT: _SPEC},
        service_script=_pick_script(
            **{
                _SELECT: [
                    ResolveReachableResponse(index=-1, message="전멸")
                ] * 3  # 파지 1 + 뷰 2방향
            }
        ),
    )
    with pytest.raises(NoReachableGrasp, match="안전 파지 불가"):
        await mod.scenario(ctx, pick_object="white cube")
    assert ctx.calls(_GRIP) == []  # 전멸이면 파지 0
    assert ctx.calls(_MOVE_L) == []


async def test_scenario_place_unreachable_fails_before_pick():
    """놓을 곳 IK 불가 → 집기 **전에** 실패 (#2). 물체를 쥔 채 멈추는 corrupt 상태를
    막는다 — 실물 실패 로그(resolve_place IK 불가) 그대로가 회귀 시나리오.

    계획 단계(집기·놓기 검출+IK)가 파지 없음이라, 놓기 IK 가 -1 이면 파지 전에
    raise → gripper·MOVE_L(실행) 0 (아무것도 안 집음). 검색 스윕 MoveJ 는 관측이라
    있는 게 정상."""
    mod = _module_for_scenario()
    ctx = FakeContext(
        robots=[_BOT],
        specs={_BOT: _SPEC},
        service_script={
            **_search_responses(),
            _CAL_BUNDLE: [_hand_eye_bundle()],
            _FUSE: [_fuse_full()],
            _DETECT: [
                DetectOrientedResponse(  # 집기 검출 OK
                    found=True, candidates=[_det(points=_cloud(False))]
                ),
                DetectOrientedResponse(  # 놓기 검출 OK
                    found=True,
                    candidates=[_det(position=(0.25, -0.05, 0.04), height=0.04)],
                ),
            ],
            _SELECT: [
                _resolve_ok(),  # 집기 도달 가능
                ResolveReachableResponse(index=-1, message="놓기 IK 전멸"),  # 놓기 불가
            ],
            # 스윕(pick) → 스윕(place)
            _MOVE_J: [MoveJResponse()] * 2,
        },
    )
    with pytest.raises(NoReachableGrasp, match="놓기 IK 전멸"):
        await mod.scenario(ctx, pick_object="white cube", place_object="red box")
    # 핵심 (#2): 파지·실행 모션이 하나도 안 나감 — 아무것도 안 집었으니 든 채 멈춤 없음.
    assert ctx.calls(_GRIP) == []
    assert ctx.calls(_MOVE_L) == []


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
    # home 조회 → 검색 스윕: search 그룹 1자세 → MoveJ → 검출(0건) → DetectionNotFound
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
    rt.responses[str(Motion.Service.STOP)] = StopResponse(ok=True)  # abort 안전 경로
    mod = PickAndPlaceModule(rt, {})  # type: ignore[arg-type]

    res = await mod.run(RunRequest(pick_object="white cube"))
    assert res.accepted
    assert mod.task._run is not None and mod.task._run.handle is not None
    await mod.task._run.handle

    states = [e for k, e in rt.published if k.endswith("/state")]
    final = states[-1]
    assert isinstance(final, TaskState)
    assert final.status == TaskStatus.FAILED
    assert final.error is not None and "white cube" in final.error  # 사유 표시

    # 실패 후 재실행 가능 (상태 corrupt 없음)
    res2 = await mod.run(RunRequest(pick_object="white cube"))
    assert res2.accepted


async def test_module_control_without_run_says_why():
    mod = PickAndPlaceModule(_WireStub(), {})  # type: ignore[arg-type]
    r = await mod.pause(ControlRequest())
    assert not r.ok and r.message  # 침묵 금지


async def test_module_preview_returns_static_tree_without_wire():
    """PREVIEW 서비스 — 실행/모킹 0 (wire stub 은 call 스크립트가 없어 호출되면
    즉사). 트리 상세는 test_task_preview 가 잠금 — 여기선 wire 노출만 확인."""
    mod = PickAndPlaceModule(_WireStub(), {})  # type: ignore[arg-type]
    res = await mod.preview(PreviewRequest())
    assert [e.name for e in res.entries if e.depth == 0] == [
        "home_waypoint", "plan_pick", "plan_place", "execute_pick", "execute_place",
    ]
    assert not mod._seq["state"]  # 프리뷰는 발행/실행 상태를 건드리지 않는다


async def test_module_toggle_breakpoint_before_run_publishes_state():
    """run 밖 breakpoint 토글(프리뷰에서 미리 박기)이 STATE 로 보인다 — robot
    라우팅은 참여 명부(TASK_ROBOTS) fallback (침묵 금지)."""
    rt = _WireStub()
    mod = PickAndPlaceModule(rt, {})  # type: ignore[arg-type]
    r = await mod.toggle_breakpoint(ToggleBreakpointRequest(name="advance"))
    assert r.ok and "다음 실행" in r.message

    states = [e for k, e in rt.published if k.endswith("/state")]
    assert states, "run 밖 토글이 침묵 — STATE 미발행"
    final = states[-1]
    assert isinstance(final, TaskState)
    assert final.robot_id == _BOT  # robot_ids 없음 → TASK_ROBOTS fallback
    assert final.status == TaskStatus.IDLE
    assert final.breakpoints == ["advance"]


def test_task_robots_constant_matches_scenario_binding():
    """TASK_ROBOTS = 바인딩 SSOT (scenario 도 여기서 파생) — 값이 바뀌면 프론트
    스트림 키/실 robot 대상이 같이 바뀌므로 명시 잠금."""
    assert PickAndPlaceModule.TASK_ROBOTS == ("so101_6dof_0",)


async def test_list_robots_returns_task_robots():
    """LIST_ROBOTS = 프론트가 {robot_id} 를 채우는 유일한 채널 — TASK_ROBOTS 와
    어긋나면 프론트가 존재하지 않는 스트림을 구독한다 (침묵 무데이터)."""
    mod = PickAndPlaceModule(_WireStub(), {})  # type: ignore[arg-type]
    res = await mod.list_robots(ListRobotsRequest())
    assert res.robot_ids == list(PickAndPlaceModule.TASK_ROBOTS)
