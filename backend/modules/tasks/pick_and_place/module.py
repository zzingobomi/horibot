from __future__ import annotations

import logging
import time

from framework.contract.publisher import publishes
from framework.contract.service import service
from framework.runtime.api import ModuleRuntime
from modules.detector.contract import (
    Detector,
    DetectOrientedResponse,
    OrientedDetection,
)
from modules.motion.contract import (
    Motion,
    MoveJResponse,
    MoveLResponse,
    ResolveReachableResponse,
)
from modules.motor.contract import Motor, SetGripperResponse
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
from modules.tasks.core.preview import Responder, collect_steps
from modules.tasks.core.runner import RunState, TaskRunner
from modules.tasks.core.spec import TaskRobotSpec

from . import steps
from .contract import (
    ListRobotsRequest,
    ListRobotsResponse,
    PickAndPlace,
    PreviewRequest,
    PreviewResponse,
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
        self._robots = robots or {}
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

    @service(PickAndPlace.Service.PREVIEW)
    async def preview(self, req: PreviewRequest) -> PreviewResponse:
        """전체 step 목록 dry-run 수집 (모션 0) — 실행 전 미리보기. 항상 놓기 포함
        최대 경로. imperative 시나리오라 정적 목록이 없어 한 번 traverse 한다."""
        entries = await collect_steps(
            self.scenario,
            robot_ids=list(self.TASK_ROBOTS),
            specs=self._robots,
            responders=self._preview_responders(),
            pick_object="(미리보기)",
            place_object="(미리보기)",  # 놓기 포함 전체 경로
        )
        return PreviewResponse(steps=entries)

    @staticmethod
    def _preview_responders() -> dict[str, Responder]:
        """dry-run canned 응답 — 시나리오가 부르는 서비스마다 benign 응답 (모션 0).

        검출은 height prior 통과하는 canonical 후보 1개 → geometry 가 파지·적치
        후보를 만들고, resolve 는 index 0 을 돌려줘 happy-path 전 구간을 traverse."""

        def _detect(_req: object) -> DetectOrientedResponse:
            cand = OrientedDetection(
                prompt="preview",
                position=(0.2, 0.0, 0.03),
                score=0.9,
                base_z=0.0,
                height=0.025,  # height prior(0.015~0.15) 통과
                grasp_yaw=0.0,
                footprint=(0.03, 0.03),
            )
            return DetectOrientedResponse(found=True, candidates=[cand])

        return {
            str(Detector.Service.DETECT_ORIENTED): _detect,
            str(Motion.Service.RESOLVE_REACHABLE): (
                lambda _r: ResolveReachableResponse(index=0)
            ),
            str(Motion.Service.MOVE_J): lambda _r: MoveJResponse(),
            str(Motion.Service.MOVE_L): lambda _r: MoveLResponse(),
            str(Motor.Service.SET_GRIPPER): lambda _r: SetGripperResponse(),
        }

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
