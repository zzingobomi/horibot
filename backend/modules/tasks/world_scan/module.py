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
    TaskState,
    TaskTrace,
    TraceEntry,
)
from modules.tasks.core.runner import RunState, TaskRunner
from modules.tasks.core.spec import TaskRobotSpec

from . import steps
from .contract import (
    ListRobotsRequest,
    ListRobotsResponse,
    RunRequest,
    WorldScan,
)

logger = logging.getLogger(__name__)


@publishes(
    (WorldScan.Stream.STATE, TaskState),
    (WorldScan.Stream.TRACE, TaskTrace),
)
class WorldScanModule:
    """World 스캔 task — 자율 스윕으로 3D 배경 메시 생성 (pick_and_place 동형 구조).

    바인딩: rgbd(깊이) capability robot 만 — 스캔은 depth 필수. so101(D405) O,
    omx(웹캠) X. 프론트 스캔 패널이 이 명부(LIST_ROBOTS)로 대상 robot 을 안다."""

    TASK_ROBOTS = ("so101_6dof_0",)

    task = TaskRunner()

    def __init__(
        self,
        runtime: ModuleRuntime,
        robots: dict[str, TaskRobotSpec] | None = None,
    ) -> None:
        self.runtime = runtime
        self.contexts = TaskContextFactory(runtime, robots)
        self._seq = {"state": 0, "trace": 0}

    async def stop(self) -> None:
        self.task.cancel()

    # ─── Services ──

    @service(WorldScan.Service.RUN)
    async def run(self, req: RunRequest) -> RunResponse:
        r = self.task.start(
            self.scenario,
            ctx=self.contexts.create(),
            robot_ids=list(self.TASK_ROBOTS),
            task_name="world_scan",
            voxel_size=req.voxel_size,
        )
        return RunResponse(accepted=r.accepted, message=r.message)

    @service(WorldScan.Service.STOP)
    async def stop_run(self, req: ControlRequest) -> ControlResponse:
        r = self.task.cancel()
        return ControlResponse(ok=r.ok, message=r.message)

    @service(WorldScan.Service.PAUSE)
    async def pause(self, req: ControlRequest) -> ControlResponse:
        r = self.task.pause()
        return ControlResponse(ok=r.ok, message=r.message)

    @service(WorldScan.Service.RESUME)
    async def resume(self, req: ControlRequest) -> ControlResponse:
        r = self.task.resume()
        return ControlResponse(ok=r.ok, message=r.message)

    @service(WorldScan.Service.LIST_ROBOTS)
    async def list_robots(self, req: ListRobotsRequest) -> ListRobotsResponse:
        return ListRobotsResponse(robot_ids=list(self.TASK_ROBOTS))

    # ─── Publishing ────

    @task.on_state
    def _publish_state(self, s: RunState) -> None:
        for robot_id in s.robot_ids or self.TASK_ROBOTS:
            self.runtime.publish(
                WorldScan.Stream.STATE,
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
                WorldScan.Stream.TRACE,
                TaskTrace(
                    robot_id=robot_id,
                    seq=self._next_seq("trace"),
                    timestamp_unix=time.time(),
                    task_name=s.task_name,
                    entries=list(entries),
                ),
            )

    def _next_seq(self, stream: str) -> int:
        seq = self._seq[stream]
        self._seq[stream] = seq + 1
        return seq

    # ─── Scenario ─────────

    async def scenario(
        self,
        ctx: TaskContext,
        voxel_size: float | None = None,
    ) -> None:
        so101 = self.TASK_ROBOTS[0]
        # 0) 관측 전 가동 조 open (§3.4 — 닫힌 조가 근거리 앞 시야 가림).
        await steps.open_gripper(ctx, so101)
        # 1) 새 세션 (이전 world 세션 프루닝).
        sid = await steps.start_session(ctx, so101)
        # 2) 스윕 — pose 마다 캡처→빌드(성장). 실패는 raise (침묵 X).
        await steps.sweep(ctx, so101, sid, voxel_size)
