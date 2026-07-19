from __future__ import annotations

import asyncio
import logging
import math
import time

from framework.contract.publisher import publishes
from framework.contract.service import service
from framework.runtime.api import ModuleRuntime
from modules.tasks.core.context import TaskContext, TaskContextFactory
from modules.tasks.core.contract import (
    ControlRequest,
    ControlResponse,
    PreviewRequest,
    PreviewResponse,
    RunResponse,
    RunToRequest,
    TaskState,
    TaskTrace,
    ToggleBreakpointRequest,
    TraceEntry,
)
from modules.tasks.core.preview import build_preview
from modules.tasks.core.runner import RunState, TaskRunner
from modules.tasks.core.spec import TaskRobotSpec

from . import servo, steps
from .contract import (
    ListRobotsRequest,
    ListRobotsResponse,
    PickAndPlace,
    RunRequest,
    TaskMarker,
    TaskMarkers,
)

logger = logging.getLogger(__name__)


def _grasp_marker(
    p: tuple[float, float, float], fam: servo.GraspFamily
) -> TaskMarker:
    """파지 마커 — 위치 + 가족 방향(approach/jaw_axis/quat) 동봉.

    계획 확정·servo 채택 갱신이 같은 구성을 쓴다 — "어느 면을 어느 방향으로
    무는지"의 시각화 소스는 항상 servo.GraspFamily (contract.TaskMarker)."""
    return TaskMarker(
        label="grasp", position=p,
        approach=fam.approach, jaw_axis=fam.jaw_axis, quaternion=fam.quat,
    )


def _unit_dir(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float] | None:
    """a→b 단위벡터 — 퇴화(0 길이)면 None (마커는 방향 표시 생략)."""
    d = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
    n = math.sqrt(d[0] ** 2 + d[1] ** 2 + d[2] ** 2)
    if n < 1e-9:
        return None
    return (d[0] / n, d[1] / n, d[2] / n)


@publishes(
    (PickAndPlace.Stream.STATE, TaskState),
    (PickAndPlace.Stream.TRACE, TaskTrace),
    (PickAndPlace.Stream.MARKERS, TaskMarkers),
)
class PickAndPlaceModule:
    TASK_ROBOTS = ("so101_6dof_0",)

    # 감독기 선언 — 훅 연결은 아래 발행 메서드의 @task.on_state/@task.on_trace
    # (@service/@publishes 와 같은 데코레이터 리듬). 실행 상태는 인스턴스별.
    task = TaskRunner()

    def __init__(
        self,
        runtime: ModuleRuntime,
        robots: dict[str, TaskRobotSpec] | None = None,
        robot_base_xy: list[tuple[float, float]] | None = None,
    ) -> None:
        self.runtime = runtime
        self.contexts = TaskContextFactory(runtime, robots)
        # 로봇 베이스 점유 XY (world) — pick 후보 구조 제외 (resolve.py 주입,
        # robots.yaml SSOT. None/빈 리스트 = 제외 없음 — 테스트/구버전 호환).
        self._robot_base_xy = list(robot_base_xy or [])
        self._seq = {"state": 0, "trace": 0, "markers": 0}

    async def stop(self) -> None:
        self.task.cancel()

    # ─── Services ──

    @service(PickAndPlace.Service.RUN)
    async def run(self, req: RunRequest) -> RunResponse:
        r = self.task.start(
            self.scenario,
            ctx=self.contexts.create(),
            robot_ids=list(self.TASK_ROBOTS),
            task_name="pick_and_place",
            pick_object=req.pick_object,
            place_object=req.place_object,
            build_world=req.build_world,
            world_voxel_size=req.world_voxel_size,
        )
        return RunResponse(accepted=r.accepted, message=r.message)

    @service(PickAndPlace.Service.STOP)
    async def stop_run(self, req: ControlRequest) -> ControlResponse:
        r = self.task.cancel()
        return ControlResponse(ok=r.ok, message=r.message)

    @service(PickAndPlace.Service.PAUSE)
    async def pause(self, req: ControlRequest) -> ControlResponse:
        r = self.task.pause()
        return ControlResponse(ok=r.ok, message=r.message)

    @service(PickAndPlace.Service.RESUME)
    async def resume(self, req: ControlRequest) -> ControlResponse:
        r = self.task.resume()
        return ControlResponse(ok=r.ok, message=r.message)

    @service(PickAndPlace.Service.STEP_ONCE)
    async def step_once(self, req: ControlRequest) -> ControlResponse:
        r = self.task.step_once()
        return ControlResponse(ok=r.ok, message=r.message)

    @service(PickAndPlace.Service.RUN_TO)
    async def run_to(self, req: RunToRequest) -> ControlResponse:
        r = self.task.run_to(req.name)
        return ControlResponse(ok=r.ok, message=r.message)

    @service(PickAndPlace.Service.TOGGLE_BREAKPOINT)
    async def toggle_breakpoint(self, req: ToggleBreakpointRequest) -> ControlResponse:
        r = self.task.toggle_breakpoint(req.name)
        return ControlResponse(ok=r.ok, message=r.message)

    @service(PickAndPlace.Service.LIST_ROBOTS)
    async def list_robots(self, req: ListRobotsRequest) -> ListRobotsResponse:
        return ListRobotsResponse(robot_ids=list(self.TASK_ROBOTS))

    @service(PickAndPlace.Service.PREVIEW)
    async def preview(self, req: PreviewRequest) -> PreviewResponse:
        # 정적 소스 읽기 — 시나리오/step 본문은 실행하지 않는다 (구조 인덱싱,
        # tasks/core/preview.py). getsource 의 파일 I/O 는 이벤트 루프 밖으로.
        entries = await asyncio.to_thread(build_preview, self.scenario)
        return PreviewResponse(entries=entries)

    # ─── Publishing ────

    @task.on_state
    def _publish_state(self, s: RunState) -> None:
        # run 밖 통지(idle breakpoint 토글)엔 robot_ids 가 없다 — 참여 명부
        # 상수로 라우팅 (실행 전 미리 박은 breakpoint 도 UI 에 보여야 함).
        for robot_id in s.robot_ids or self.TASK_ROBOTS:
            self.runtime.publish(
                PickAndPlace.Stream.STATE,
                TaskState(
                    robot_id=robot_id,
                    seq=self._next_seq("state"),
                    timestamp_unix=time.time(),
                    status=s.status,
                    task_name=s.task_name,
                    current_name=s.current_name,
                    current_title=s.current_title,
                    error=s.error,
                    breakpoints=list(s.breakpoints),
                ),
            )

    @task.on_trace
    def _publish_trace(self, s: RunState, entries: list[TraceEntry]) -> None:
        for robot_id in s.robot_ids:
            self.runtime.publish(
                PickAndPlace.Stream.TRACE,
                TaskTrace(
                    robot_id=robot_id,
                    seq=self._next_seq("trace"),
                    timestamp_unix=time.time(),
                    task_name=s.task_name,
                    entries=list(entries),
                ),
            )

    def _publish_markers(self, robot_id: str, markers: list[TaskMarker]) -> None:
        self.runtime.publish(
            PickAndPlace.Stream.MARKERS,
            TaskMarkers(
                robot_id=robot_id,
                seq=self._next_seq("markers"),
                timestamp_unix=time.time(),
                markers=list(markers),
            ),
        )

    def _next_seq(self, stream: str) -> int:
        seq = self._seq[stream]
        self._seq[stream] = seq + 1
        return seq

    # ─── Task Scenarios ─────────

    async def scenario(
        self,
        ctx: TaskContext,
        pick_object: str,
        place_object: str = "",
        build_world: bool = False,
        world_voxel_size: float | None = None,
    ) -> None:
        so101 = self.TASK_ROBOTS[0]
        markers: list[TaskMarker] = []

        # 0) home 경유 자세 — 없으면 모션 0 시점에 명시적 실패 (티칭 안내).
        home = await steps.home_waypoint(ctx, so101)

        # 0.5) World 스캔 (옵션) — search 스윕에 편승해 scan 세션을 돌려
        # 3D 배경(World 레이어)을 갱신. best-effort (실패해도 pick 계속).
        world = None
        if build_world:
            world = steps.WorldScan(ctx, so101, voxel_size=world_voxel_size)
            await world.start()

        # 1) 검출 — **한 스윕**에 pick 검출 + place 검출 + world 스캔 전부
        # (2026-07-19 통합 — 옛 구조는 place 가 같은 자세를 다시 돌았다.
        # pose 당 wire 1호출, 후보는 per-candidate prompt 귀속으로 분리).
        prompts = [pick_object]
        if place_object and place_object != pick_object:
            prompts.append(place_object)
        found = await steps.detect(ctx, so101, prompts, world=world)

        # 2) 계획 — 집기(servo 접근 가족)·놓기 도달성을 모두 먼저 검증 (물리
        # 파지 0). 놓을 곳이 도달 불가면 아무것도 집기 전에 실패한다 (물체 쥔 채
        # 멈추는 corrupt 방지). 놓기의 held 기하 = coarse 관측 (steps 주석).
        plan = await steps.plan_pick(
            ctx, so101, pick_object, home, found.get(pick_object, []),
            exclude_xy=self._robot_base_xy,
        )
        markers.append(_grasp_marker(plan.grasp_point0, plan.family))

        drop, drop_pre = None, None
        if place_object:
            drop, drop_pre = await steps.plan_place(
                ctx, so101, place_object,
                held=plan.coarse, lateral=plan.lateral0, home=home,
                spots=found.get(place_object, []),
            )
            markers.append(TaskMarker(
                label="place", position=drop.place,
                # 적치 진입 방향 = pre→place (수직 삽입 축), 자세 = 계획 quat.
                # 조 축은 place 마커엔 생략 (release 는 조 방향이 관심사 아님).
                approach=_unit_dir(drop.pre, drop.place),
                quaternion=drop.quat,
            ))

        # 계획 확정 시점에 마커 표시 (파지·적치 지점을 실행 전 미리 보여줌).
        self._publish_markers(so101, markers)

        # servo 가 파지점을 갱신할 때마다 마커 재발행 — 계획 시점 마커가 실행
        # 내내 고정 표시되던 UI 구멍 (2026-07-17 사용자 리포트. 스트림은
        # latest-wins 라 매 채택 발행이 곧 실시간 표시). fam 도 함께 — refit/
        # 재플랜으로 파지 방향이 바뀌면 화살표·조 축 바가 실시간 따라간다.
        def on_grasp(
            p: tuple[float, float, float], fam: servo.GraspFamily
        ) -> None:
            self._publish_markers(so101, [
                _grasp_marker(p, fam),
                *markers[1:],  # place 마커 유지 (계획값 — 적치는 open-loop)
            ])

        # 3) 실행 — 집기 = closed-loop servo (물체 근처에서 관측→보정 루프,
        # steps/servo.py 정본), 놓기 = open-loop (상자 적치는 오차 관대).
        await steps.servo_pick(ctx, so101, plan, pick_object, home, on_grasp)
        if drop is not None and drop_pre is not None:
            await steps.execute_place(ctx, so101, drop, drop_pre, home)
