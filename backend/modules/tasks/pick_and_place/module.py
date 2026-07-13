from __future__ import annotations

import logging
import time

from framework.contract.publisher import publishes
from framework.contract.service import service
from framework.runtime.api import ModuleRuntime
from modules.tasks.core.context import TaskContext, TaskContextFactory
from modules.tasks.core.contract import (
    ControlRequest,
    ControlResponse,
    RunResponse,
    RunToRequest,
    TaskState,
    TaskTrace,
    ToggleBreakpointRequest,
    TraceEntry,
)
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

    def __init__(
        self, runtime: ModuleRuntime, robots: dict[str, TaskRobotSpec] | None = None
    ) -> None:
        self.runtime = runtime
        self.contexts = TaskContextFactory(runtime, robots)
        self._seq = {"state": 0, "trace": 0, "markers": 0}
        self.task = TaskRunner(
            on_state=self._publish_state,
            on_trace=self._publish_trace,
        )

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

    # TODO(미결): 실행 전 전체 step 목록 미리보기 서비스 — 디버거 breakpoint/run-to 용.
    # 설계 미확정 (contract.py PickAndPlace.Service 의 TODO 참조): imperative 시나리오라
    # 정적 분석(AST)은 if/loop/동적 호출에서 깨지고, 완전 보장은 선언형 구조가 필요.
    # 방향 확정 후 여기 핸들러 추가. (2026-07-13)

    # ─── Publishing ────

    def _publish_state(self, s: RunState) -> None:
        for robot_id in s.robot_ids:
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

        # 1) 계획 — 집기·놓기 도달성을 모두 먼저 검증 (모션 0). 놓을 곳이 도달
        # 불가면 아무것도 집기 전에 실패한다 (물체 쥔 채 멈추는 corrupt 방지).
        held, grasp = await steps.plan_pick(ctx, so101, pick_object)
        markers.append(TaskMarker(label="grasp", position=grasp.grasp))

        drop = None
        if place_object:
            drop = await steps.plan_place(
                ctx, so101, place_object, held=held, grasp=grasp
            )
            markers.append(TaskMarker(label="place", position=drop.place))

        # 계획 확정 시점에 마커 표시 (파지·적치 지점을 실행 전 미리 보여줌).
        self._publish_markers(so101, markers)

        # 2) 실행 — 계획이 모두 도달 가능일 때만 물리 동작.
        await steps.execute_pick(ctx, so101, grasp)
        if drop is not None:
            await steps.execute_place(ctx, so101, drop)
