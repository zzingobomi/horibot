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

        held, grasp = await steps.pick(ctx, so101, pick_object)
        markers.append(TaskMarker(label="grasp", position=grasp.grasp))
        self._publish_markers(so101, markers)

        if not place_object:
            return

        drop = await steps.place(ctx, so101, place_object, held=held, grasp=grasp)
        markers.append(TaskMarker(label="place", position=drop.place))
        self._publish_markers(so101, markers)
