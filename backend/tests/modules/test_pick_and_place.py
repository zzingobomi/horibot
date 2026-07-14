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
    spot = _det(position=(0.25, -0.05, 0.04), height=0.04, grasp_yaw=0.3)
    held = _det(height=0.023)
    pplan = geometry.plan_place(spot, held=held, lateral=0.008)
    # release z = spot 상면 + held/2 + 여유(0.005) — 물체 바닥이 상면에 닿게
    assert pplan[0].place[2] == pytest.approx(0.04 + 0.023 / 2 + 0.005)
    # tilt=0(첫 후보) = 순수 수직 하강: pre 가 place 바로 위 (x,y 동일, z 만 위로)
    assert pplan[0].pre[0] == pytest.approx(pplan[0].place[0])
    assert pplan[0].pre[1] == pytest.approx(pplan[0].place[1])
    assert pplan[0].pre[2] == pytest.approx(pplan[0].place[2] + 0.06 + 0.023 / 2)
    # 7 tilt × 4 정렬 yaw — tilt 는 0~±60° 도달 띠 (소각 상한 = top-down 사각
    # 전멸 / 13단 풀사다리 = 전멸 가족 풀예산 IK 로 분 단위 지연, 둘 다 실물),
    # yaw 는 0/180/90/270° 4방향 (180° 는 조·롤 방향이 다른 별개 IK — 2방향으로
    # 줄이면 "위치 통과·자세 전멸" 실물 회귀, 2026-07-14). 수직·정렬이 첫 후보.
    assert len(pplan) == 7 * 4
    assert pplan[0].label == "tilt=+0 yaw=17"  # 수직 + 정렬 최우선
    # yaw 는 상자 방위(grasp_yaw=0.3rad≈17°) 기준 — 하드코딩 0/90° 아님.
    for deg in (17, 107, 197, 287):  # 정렬 가족 4방향 전부
        assert any(f"yaw={deg}" in c.label for c in pplan)


def test_plan_place_free_family_disjoint_yaws():
    """자유 yaw 가족 (정렬 전멸 폴백) — 30° 격자의 나머지 8방향 × 13 tilt.
    정렬과 겹치면 헛 IK (전멸 가족은 그룹당 풀예산 소모), 뒤집으면 = 커버리지
    구멍 or 중복 낭비."""
    spot = _det(position=(0.25, -0.05, 0.04), height=0.04, grasp_yaw=0.3)
    held = _det(height=0.023)
    aligned = geometry.plan_place(spot, held=held, lateral=0.008)
    free = geometry.plan_place_free(spot, held=held, lateral=0.008)
    assert len(free) == 7 * 8
    assert free[0].label == "tilt=+0 yaw=47"  # 정렬에 가장 가까운 자유 yaw 먼저
    yaw_of = lambda c: c.label.split("yaw=")[1]  # noqa: E731
    assert {yaw_of(c) for c in aligned} & {yaw_of(c) for c in free} == set()


# ─── adaptive 관측·파지 성립 (step 직접 — FakeContext) ────────────────


async def test_observe_uses_close_view_not_sweep_seed():
    """search 스윕은 '찾기'(coarse)만 — 파지는 close 뷰 관측에서 (2026-07-14 재구성).
    스윕 검출을 파지 융합에 넣지 않고, coarse 를 겨냥해 뷰를 찍어(MOVE_J) 그 close
    관측만 융합해 파지를 세운다. 옛 "스윕 시드만으로 파지 성립+뷰 이동 0" 조기
    종료(멀리서 본 걸로 파지 결정 → 큐브 끝 스침)의 회귀 잠금."""
    coarse = _det()
    sweep_near = _det(score=0.7, position=(0.21, 0.06, 0.023))  # 스윕 또 다른 관측
    close = _det(score=0.8, position=(0.205, 0.052, 0.024))  # 뷰에서 찍은 close 관측
    view_sol = [0.3] * 6
    ctx = FakeContext(
        robots=[_BOT],
        specs={_BOT: _SPEC},
        service_script={
            _CAL_BUNDLE: [_hand_eye_bundle()],
            _SELECT: [
                ResolveReachableResponse(index=0, solutions=[view_sol]),  # 뷰 도달
                _resolve_ok(),  # 파지 성립
            ],
            _MOVE_J: [MoveJResponse()] * 2,  # home 경유 + 뷰 이동
            _DETECT: [DetectOrientedResponse(found=True, candidates=[close])],
            _FUSE: [_fuse_full()],
        },
    )
    fused, grasp, pre = await steps.observe_and_plan_grasp(
        ctx, _BOT, [coarse, sweep_near], coarse, "white cube", _home_record()
    )
    assert pre == [0.1] * 6  # resolve 해 그대로 (재계산 금지 — §5.5)
    # 핵심: 스윕 시드 없이 close 뷰를 찍었다 (MOVE_J = home 경유 + 뷰 이동)
    assert [c["req"].target.joints for c in ctx.calls(_MOVE_J)] == [
        _HOME_JOINTS, view_sol,
    ]
    # 파지 융합 입력 = close 관측만 — 스윕의 coarse/sweep_near 는 안 들어감
    fuse_req = ctx.calls(_FUSE)[0]["req"]
    assert [c.position for c in fuse_req.candidates] == [close.position]
    assert coarse not in fuse_req.candidates
    assert sweep_near not in fuse_req.candidates

    # 파지 resolve 게이트 계약 (§10.4-3): 직선 + 바닥 + 그리퍼 벌림 + home 경로.
    # SELECT[0]=뷰 스크린, [1]=파지 게이트.
    grasp_sel = ctx.calls(_SELECT)[1]["req"]
    assert grasp_sel.linear is True and grasp_sel.gripper_open is True
    assert grasp_sel.floor_z == pytest.approx(0.0 - 0.005)
    assert grasp_sel.path_from == _HOME_JOINTS
    assert grasp_sel.obstacle_points  # 융합 점군이 장애물로


async def test_observe_accumulates_close_views_until_grasp_stands():
    """close 뷰 방향1 도달 불가(스킵) → 뷰2 관측 1개(한쪽 면, 쌍0)로는 안 섬 →
    뷰3 관측 추가 → 융합에 마주 보는 면이 생겨 파지 성립. 파지 융합 입력은 close
    관측만 누적(스윕 시드 없음). 뷰 이동 = home 경유 + resolve 해 (§10.4-4)."""
    coarse = _det()
    c1 = _det(score=0.8, position=(0.205, 0.052, 0.024))  # 뷰2 close 관측
    c2 = _det(score=0.8, position=(0.198, 0.048, 0.023))  # 뷰3 close 관측
    vs1, vs2 = [0.3] * 6, [0.35] * 6
    ctx = FakeContext(
        robots=[_BOT],
        specs={_BOT: _SPEC},
        service_script={
            _CAL_BUNDLE: [_hand_eye_bundle()],
            _FUSE: [_fuse_half(), _fuse_full()],  # 1뷰=쌍0 → 2뷰=성립
            _SELECT: [
                ResolveReachableResponse(index=-1, message="뷰 도달 불가"),  # 뷰1 스킵
                ResolveReachableResponse(index=0, solutions=[vs1]),  # 뷰2 도달
                ResolveReachableResponse(index=0, solutions=[vs2]),  # 뷰3 도달
                _resolve_ok(),  # 파지 성립
            ],
            _MOVE_J: [MoveJResponse()] * 4,  # (home 경유 + 뷰)×2
            _DETECT: [
                DetectOrientedResponse(found=True, candidates=[c1]),
                DetectOrientedResponse(found=True, candidates=[c2]),
            ],
        },
    )
    fused, grasp, pre = await steps.observe_and_plan_grasp(
        ctx, _BOT, [coarse], coarse, "white cube", _home_record()
    )
    assert pre == [0.1] * 6
    # 호출 순서: 뷰1 resolve(-1 스킵) → 뷰2 resolve → home → 뷰 MoveJ → 검출 →
    # 융합(쌍0) → 뷰3 resolve → home → 뷰 MoveJ → 검출 → 융합(성립) → 파지 resolve
    assert ctx.keys() == [
        _CAL_BUNDLE, _SELECT, _SELECT, _MOVE_J, _MOVE_J, _DETECT, _FUSE,
        _SELECT, _MOVE_J, _MOVE_J, _DETECT, _FUSE, _SELECT,
    ]
    # 뷰 resolve 계약: roll 그룹(그룹당 pose 1) + floor + home 경로, 파지 아님.
    view_req = ctx.calls(_SELECT)[0]["req"]
    assert len(view_req.groups) == 6 and all(len(g) == 1 for g in view_req.groups)
    assert view_req.gripper_open is False and view_req.linear is False
    assert view_req.path_from == _HOME_JOINTS
    # 뷰 이동 = home 경유 후 resolve 해 그대로 (도달한 뷰2/뷰3)
    assert [c["req"].target.joints for c in ctx.calls(_MOVE_J)] == [
        _HOME_JOINTS, vs1, _HOME_JOINTS, vs2,
    ]
    # 파지 융합 입력 = close 관측만 누적 (c1,c2) — 스윕 없음
    assert [c.position for c in ctx.calls(_FUSE)[1]["req"].candidates] == [
        c1.position, c2.position,
    ]


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
    """close 뷰에서 antipodal 쌍은 있으나 파지 resolve 전멸(-1) = 부정 데이터 →
    관측을 더 시도, 끝까지 안 서면 명시 실패. -1 을 침묵 통과(맹목 파지)하면 회귀.
    (뷰는 도달 성공시켜야 파지 게이트가 돈다.)"""
    coarse = _det()
    c = _det(score=0.8, position=(0.205, 0.052, 0.024))
    vs = [0.3] * 6
    ctx = FakeContext(
        robots=[_BOT],
        specs={_BOT: _SPEC},
        service_script={
            _CAL_BUNDLE: [_hand_eye_bundle()],
            _FUSE: [_fuse_full(), _fuse_full()],  # 뷰마다 쌍 있음
            _SELECT: [
                ResolveReachableResponse(index=0, solutions=[vs]),  # 뷰1 도달
                ResolveReachableResponse(index=-1, message="전멸"),  # 파지 게이트1
                ResolveReachableResponse(index=0, solutions=[vs]),  # 뷰2 도달
                ResolveReachableResponse(index=-1, message="전멸"),  # 파지 게이트2
            ],
            _MOVE_J: [MoveJResponse()] * 4,
            _DETECT: [
                DetectOrientedResponse(found=True, candidates=[c]),
                DetectOrientedResponse(found=True, candidates=[c]),
            ],
        },
    )
    with pytest.raises(NoReachableGrasp, match="안전 파지 불가"):
        await steps.observe_and_plan_grasp(
            ctx, _BOT, [coarse], coarse, "white cube", _home_record()
        )
    # 뷰 스크린(gripper_open False)과 파지 게이트(True)가 번갈아 — 파지 -1 은
    # 데이터라 다음 뷰로 계속 (침묵 통과 아님).
    assert [s["req"].gripper_open for s in ctx.calls(_SELECT)] == [
        False, True, False, True,
    ]


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
    """pick 경로 성공 스크립트 (search 자세 1개 → coarse, close 뷰 1개서 파지 성립).
    시나리오 = home 조회 → 스윕(찾기) → close 뷰 도달·검출·융합 → 파지 성립 → 실행.
    _SELECT 순서 = 뷰 스크린 → 파지 게이트 (→ place resolve)."""
    script = {
        **_search_responses(),
        _CAL_BUNDLE: [_hand_eye_bundle()] * 2,
        _DETECT: [
            DetectOrientedResponse(
                found=True, candidates=[_det(points=_cloud(False))]
            )
        ] * 4,
        _FUSE: [_fuse_full()] * 2,
        _SELECT: [_resolve_ok()] * 4,  # 뷰 도달 + 파지 (+ place)
        _MOVE_J: [MoveJResponse()] * 10,  # 스윕 + 뷰(home+이동) + 실행
        _MOVE_L: [MoveLResponse()] * 4,  # advance/withdraw (+ insert/retreat)
        _GRIP: [SetGripperResponse()] * 3,  # open + close (+ release)
    }
    script.update(overrides)
    return script


async def test_scenario_pick_only_sequence():
    mod = _module_for_scenario()
    ctx = FakeContext(robots=[_BOT], specs={_BOT: _SPEC}, service_script=_pick_script())

    await mod.scenario(ctx, pick_object="white cube")

    # 호출 순서 = home 조회 → 검색 스윕(찾기: 그룹→자세 MoveJ→검출) → 관측·파지
    # (hand_eye 조회 → 뷰 도달 resolve → home 경유+뷰 MoveJ → close 검출 → 융합 →
    # 파지 resolve) → 실행: home 경유 → pre 접근 → open → 진입 → close → 후퇴 → home.
    assert ctx.keys() == [
        _LIST_WP,  # home waypoint 조회 (모션 0)
        _LIST_GROUPS, _LIST_MEMBERS, _MOVE_J, _DETECT,  # 스윕(찾기)
        _CAL_BUNDLE, _SELECT, _MOVE_J, _MOVE_J, _DETECT, _FUSE, _SELECT,  # close 관측·파지
        _MOVE_J, _MOVE_J, _GRIP, _MOVE_L, _GRIP, _MOVE_L, _MOVE_J,  # 실행
    ]
    # place 분기 안 탐 + 든 채 종료 (마지막 gripper = close raw)
    grips = [c["req"].position_raw for c in ctx.calls(_GRIP)]
    assert grips == [_SPEC.gripper_open_raw, _SPEC.gripper_close_raw]

    # 파지 게이트(§10.4-3) = SELECT[1] (SELECT[0]=뷰 스크린): floor_z = 융합 base_z
    # − 버퍼, linear + 그리퍼 벌림 충돌 + home 경로 게이트.
    sel = ctx.calls(_SELECT)[1]["req"]
    assert sel.linear is True
    assert sel.floor_z == pytest.approx(0.0 - 0.005)
    assert sel.gripper_open is True and sel.path_from == _HOME_JOINTS
    assert sel.obstacle_points
    # pre 접근 = resolve 가 반환한 관절 해 그대로 (실행부 IK 재계산 금지 — §5.5).
    # MoveJ 순서: [스윕, 뷰-home, 뷰-이동, 실행-home, pre, 실행-home] — pre 는 index 4.
    pre_req = ctx.calls(_MOVE_J)[4]["req"]
    assert pre_req.target.kind == "joint" and pre_req.target.joints == [0.1] * 6
    # home 경유 = 티칭 waypoint 관절값 (뷰 이동 전 home = index 1)
    assert ctx.calls(_MOVE_J)[1]["req"].target.joints == _HOME_JOINTS


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
                DetectOrientedResponse(  # pick 스윕(찾기)
                    found=True, candidates=[_det(points=_cloud(False))]
                ),
                DetectOrientedResponse(  # pick close 뷰
                    found=True, candidates=[_det(points=_cloud(False))]
                ),
                DetectOrientedResponse(  # place 스윕
                    found=True,
                    candidates=[_det(position=(0.25, -0.05, 0.04), height=0.04)],
                ),
            ],
            _SELECT: [_resolve_ok()] * 3,  # pick 뷰 도달 + pick 파지 + place resolve
            _MOVE_J: [MoveJResponse()] * 10,
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
    # 관절 이동 = pick 스윕(1) + close 뷰 home+이동(2) + place 스윕(1) +
    # execute_pick home×2+pre(3) + execute_place pre+home(2) = 9
    assert len(ctx.calls(_MOVE_J)) == 9
    # 적치 resolve = SELECT[2] (뷰·파지 뒤): home 경로 게이트 + linear
    place_sel = ctx.calls(_SELECT)[2]["req"]
    assert place_sel.path_from == _HOME_JOINTS and place_sel.linear is True
    # #2 불변식: 집기·놓기 도달성(RESOLVE)이 **전부** 끝난 뒤에야 첫 파지(GRIP)와
    # 실행 모션(MOVE_L)이 나간다 — 못 놓을 물체를 집는 일이 없도록. 계획 단계
    # RESOLVE = pick 뷰(1) + pick 파지(1) + place(1) = 3, 전부 GRIP/MOVE_L 앞.
    keys = ctx.keys()
    assert keys[: keys.index(_GRIP)].count(_SELECT) == 3
    assert keys[: keys.index(_MOVE_L)].count(_SELECT) == 3


async def test_search_sweep_accumulates_and_selects_best_score():
    """검색 원리 (옛 SearchWaypointGroup + SelectTarget 포팅): search 자세를 **전부**
    돌며 후보를 **누적**하고, select_target_by_score 가 누적 전체의 최고 score 를
    고른다 (첫 자세서 안 멈춤). search 는 '찾기' 전용 — 파지는 이후 close 관측이
    판단(observe_and_plan_grasp)하므로 여기선 detect+선택만 검증."""
    ctx = FakeContext(
        robots=[_BOT],
        specs={_BOT: _SPEC},
        service_script={
            **_search_responses(n_members=2),  # 검색 자세 2개
            _DETECT: [
                DetectOrientedResponse(found=True, candidates=[_det(score=0.4)]),
                DetectOrientedResponse(found=True, candidates=[_det(score=0.95)]),
            ],
            _MOVE_J: [MoveJResponse()] * 2,  # 스윕 자세 2곳
        },
    )
    cands = await steps.detect(ctx, _BOT, "white cube")

    assert len(ctx.calls(_MOVE_J)) == 2  # 두 자세 다 돎 (첫서 안 멈춤)
    assert len(ctx.calls(_DETECT)) == 2  # 스윕 자세마다 검출
    assert [c.score for c in cands] == [0.4, 0.95]  # 누적 (선택 안 함)
    # 선택 = 누적 전체 최고 score (첫 자세 0.4 아님)
    coarse = geometry.select_target_by_score(cands, prompt="white cube")
    assert coarse.score == 0.95


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
                ] * 3  # 뷰 2방향 전멸(도달 불가) — close 뷰를 못 찍어 파지 미성립
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
                DetectOrientedResponse(  # 집기 스윕(찾기)
                    found=True, candidates=[_det(points=_cloud(False))]
                ),
                DetectOrientedResponse(  # 집기 close 뷰
                    found=True, candidates=[_det(points=_cloud(False))]
                ),
                DetectOrientedResponse(  # 놓기 스윕
                    found=True,
                    candidates=[_det(position=(0.25, -0.05, 0.04), height=0.04)],
                ),
            ],
            _SELECT: [
                ResolveReachableResponse(index=0, solutions=[[0.3] * 6]),  # 집기 뷰 도달
                _resolve_ok(),  # 집기 파지 성립
                # 놓기 불가 — 정렬 + 자유 yaw 두 가족 다 전멸
                ResolveReachableResponse(index=-1, message="놓기 IK 전멸"),
                ResolveReachableResponse(index=-1, message="놓기 IK 전멸"),
            ],
            # 집기 스윕(1) + close 뷰 home+이동(2) + 놓기 스윕(1)
            _MOVE_J: [MoveJResponse()] * 4,
        },
    )
    with pytest.raises(NoReachableGrasp, match="놓을 자리 도달 불가"):
        await mod.scenario(ctx, pick_object="white cube", place_object="red box")
    # 핵심 (#2): 파지·실행 모션이 하나도 안 나감 — 아무것도 안 집었으니 든 채 멈춤 없음.
    assert ctx.calls(_GRIP) == []
    assert ctx.calls(_MOVE_L) == []


async def test_plan_place_falls_back_to_reachable_spot():
    """놓기 타깃 = 점수 1등에 무조건 커밋하지 않고 **닿는 첫 spot** 채택 (실물 버그
    회귀: 점수 최고인 선반 위 통(도달 불가)에 커밋해 실패, 테이블 박스(도달 가능)를
    버림). 점수 높은 unreachable → 낮지만 reachable 로 폴백. 뒤집으면 = 점수-only
    커밋 회귀."""
    high_far = _det(score=0.80, position=(0.15, 0.10, 0.22), base_z=0.20)  # 선반 위
    low_near = _det(score=0.73, position=(0.24, -0.11, 0.03), base_z=0.005)  # 테이블
    ctx = FakeContext(
        robots=[_BOT],
        specs={_BOT: _SPEC},
        service_script={
            **_search_responses(),
            _DETECT: [
                DetectOrientedResponse(found=True, candidates=[high_far, low_near])
            ],
            _SELECT: [  # spot 점수순: high_far 정렬·자유 전멸 → low_near 정렬 가용
                ResolveReachableResponse(index=-1, message="선반 위 IK 전멸"),
                ResolveReachableResponse(index=-1, message="선반 위 IK 전멸"),
                _resolve_ok(),
            ],
            _MOVE_J: [MoveJResponse()],  # place 스윕 1자세
        },
    )
    held = _det(height=0.023)
    grasp = geometry.GraspCandidate(
        label="stub", pre=(0, 0, 0), grasp=(0, 0, 0),
        quat=(0, 0, 0, 1), lateral=0.008,
    )
    chosen, _pre = await steps.plan_place(
        ctx, _BOT, "blue box", held=held, grasp=grasp, home=_home_record()
    )
    # 채택된 놓기 자리 = 테이블 박스(low_near) 위 — 선반(high_far) 아님
    assert chosen.place[2] == pytest.approx(0.03 + 0.023 / 2 + 0.005)
    # resolve 소비 = 선반(정렬+자유 전멸 2회) → 테이블(정렬 채택 1회)
    assert len(ctx.calls(_SELECT)) == 3


async def test_plan_place_falls_back_to_free_yaw_family():
    """한 spot 안에서 정렬 yaw 전멸 → 자유 yaw 폴백 (실물 회귀: 위치 통과 26/26
    자세 IK 전멸 — 정렬 yaw 그물이 성겨 지점은 닿는데 자세를 못 찾음). 뒤집으면
    = 정렬 전멸이 곧 spot 포기(삐딱하게라도 놓기 상실) 회귀."""
    ctx = FakeContext(
        robots=[_BOT],
        specs={_BOT: _SPEC},
        service_script={
            **_search_responses(),
            _DETECT: [DetectOrientedResponse(found=True, candidates=[_det()])],
            _SELECT: [
                ResolveReachableResponse(index=-1, message="정렬 yaw 전멸"),
                _resolve_ok(),  # 자유 yaw 가족에서 성립
            ],
            _MOVE_J: [MoveJResponse()],
        },
    )
    grasp = geometry.GraspCandidate(
        label="stub", pre=(0, 0, 0), grasp=(0, 0, 0),
        quat=(0, 0, 0, 1), lateral=0.008,
    )
    chosen, _pre = await steps.plan_place(
        ctx, _BOT, "cube", held=_det(height=0.023), grasp=grasp,
        home=_home_record(),
    )
    assert len(ctx.calls(_SELECT)) == 2  # 정렬 → 자유 두 가족 순차
    assert chosen.label == "tilt=+0 yaw=47"  # 자유 가족 첫 후보 (index=0)


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
