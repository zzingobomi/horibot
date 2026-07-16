from __future__ import annotations

import asyncio
import logging
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

from . import steps
from .contract import (
    ListRobotsRequest,
    ListRobotsResponse,
    PickAndPlace,
    RunRequest,
    TaskMarker,
    TaskMarkers,
)

logger = logging.getLogger(__name__)


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
        self, runtime: ModuleRuntime, robots: dict[str, TaskRobotSpec] | None = None
    ) -> None:
        self.runtime = runtime
        self.contexts = TaskContextFactory(runtime, robots)
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
        self, ctx: TaskContext, pick_object: str, place_object: str = ""
    ) -> None:
        so101 = self.TASK_ROBOTS[0]
        markers: list[TaskMarker] = []

        # 0) home 경유 자세 — 없으면 모션 0 시점에 명시적 실패 (티칭 안내).
        home = await steps.home_waypoint(ctx, so101)

        # 1) 계획 — 집기(servo 접근 가족)·놓기 도달성을 모두 먼저 검증 (물리
        # 파지 0). 놓을 곳이 도달 불가면 아무것도 집기 전에 실패한다 (물체 쥔 채
        # 멈추는 corrupt 방지). 놓기의 held 기하 = coarse 관측 (steps 주석).
        plan = await steps.plan_pick(ctx, so101, pick_object, home)
        markers.append(TaskMarker(label="grasp", position=plan.grasp_point0))

        drop, drop_pre = None, None
        if place_object:
            drop, drop_pre = await steps.plan_place(
                ctx, so101, place_object,
                held=plan.coarse, lateral=plan.lateral0, home=home,
            )
            markers.append(TaskMarker(label="place", position=drop.place))

        # 계획 확정 시점에 마커 표시 (파지·적치 지점을 실행 전 미리 보여줌).
        self._publish_markers(so101, markers)

        # 2) 실행 — 집기 = closed-loop servo (물체 근처에서 관측→보정 루프,
        # steps/servo.py 정본), 놓기 = open-loop (상자 적치는 오차 관대).
        await steps.servo_pick(ctx, so101, plan, pick_object, home)
        if drop is not None and drop_pre is not None:
            await steps.execute_place(ctx, so101, drop, drop_pre, home)
