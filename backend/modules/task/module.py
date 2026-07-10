"""TaskModule — Orchestration layer (§16.1 L3). host-level, robot-agnostic (§2.7).

Day-1 primitive 를 async 함수(task)로 엮어 실행 + 디버거(pause/step/breakpoint/run_to)
= dev 안전장치 (§17.1.4). runner = robot_id 별 1개 (robot 별 독립 실행). TREE 는
여기서 발행 (run/preview 시), STATE/STEP_RESULT 는 runner 가 발행 (같은 runtime).

task 정본은 tasks/ registry — module 은 build_task(name, params) 로 TaskSpec 만 받음
(어떤 task 가 있는지 몰라도 됨, plug-in).
"""

from __future__ import annotations

import logging
import time

from framework.contract.publisher import publishes
from framework.contract.service import service
from framework.contract.subscriber import subscriber
from framework.runtime.api import ModuleRuntime
from modules.motor.contract import JointState, Motor

from .contract import (
    PreviewRequest,
    PreviewResponse,
    RunRequest,
    RunResponse,
    RunToRequest,
    Task,
    TaskControlRequest,
    TaskControlResponse,
    TaskState,
    TaskStepResult,
    TaskTree,
    ToggleBreakpointRequest,
)
from .runner import TaskRunner
from .spec import TaskRobotSpec
from .step import TaskSpec, task_tree
from .tasks import build_task

logger = logging.getLogger(__name__)


@publishes(
    (Task.Stream.STATE, TaskState),
    (Task.Stream.TREE, TaskTree),
    (Task.Stream.STEP_RESULT, TaskStepResult),
)
class TaskModule:
    def __init__(
        self, runtime: ModuleRuntime, robots: dict[str, TaskRobotSpec]
    ) -> None:
        self.runtime = runtime
        self._robots = robots  # robot_id → 물리 config (gripper 등, resolve 주입)
        self._runners: dict[str, TaskRunner] = {}
        self._tree_seq: dict[str, int] = {}
        self._gripper_raw: dict[str, int] = {}  # Motor.RAW_STATE 캐시 (VerifyGrasp)

    async def start(self) -> None:
        logger.info("TaskModule start (host-level)")

    async def stop(self) -> None:
        for r in self._runners.values():
            r.stop()

    @subscriber(Motor.Stream.RAW_STATE)
    def _on_motor_raw(self, state: JointState) -> None:
        """gripper 현재 raw 캐시 — VerifyGrasp 잡힘 판정용 (scan 의 arm 캐시 동형)."""
        spec = self._robots.get(state.robot_id)
        if spec is None or not (0 <= spec.gripper_index < len(state.positions_raw)):
            return
        self._gripper_raw[state.robot_id] = state.positions_raw[spec.gripper_index]

    def _runner(self, robot_id: str) -> TaskRunner:
        r = self._runners.get(robot_id)
        if r is None:
            r = TaskRunner(
                self.runtime,
                robot_id,
                self._robots.get(robot_id),
                lambda rid=robot_id: self._gripper_raw.get(rid),
            )
            self._runners[robot_id] = r
        return r

    @service(Task.Service.RUN)
    async def run(self, req: RunRequest) -> RunResponse:
        try:
            spec = build_task(req.task_name, req.params)
        except KeyError:
            return RunResponse(accepted=False, message=f"미등록 task '{req.task_name}'")
        self._publish_tree(req.robot_id, spec)
        accepted = self._runner(req.robot_id).run(spec)
        return RunResponse(
            accepted=accepted, message="" if accepted else "이미 실행 중"
        )

    @service(Task.Service.PREVIEW)
    async def preview(self, req: PreviewRequest) -> PreviewResponse:
        try:
            spec = build_task(req.task_name, req.params)
        except KeyError:
            return PreviewResponse(ok=False, message=f"미등록 task '{req.task_name}'")
        self._publish_tree(req.robot_id, spec)
        return PreviewResponse(ok=True)

    @service(Task.Service.STOP)
    async def stop_task(self, req: TaskControlRequest) -> TaskControlResponse:
        self._runner(req.robot_id).stop()
        return TaskControlResponse(ok=True)

    @service(Task.Service.PAUSE)
    async def pause(self, req: TaskControlRequest) -> TaskControlResponse:
        return TaskControlResponse(ok=self._runner(req.robot_id).pause())

    @service(Task.Service.RESUME)
    async def resume(self, req: TaskControlRequest) -> TaskControlResponse:
        return TaskControlResponse(ok=self._runner(req.robot_id).resume())

    @service(Task.Service.STEP_ONCE)
    async def step_once(self, req: TaskControlRequest) -> TaskControlResponse:
        return TaskControlResponse(ok=self._runner(req.robot_id).step_once())

    @service(Task.Service.RUN_TO)
    async def run_to(self, req: RunToRequest) -> TaskControlResponse:
        return TaskControlResponse(
            ok=self._runner(req.robot_id).run_to(req.step_id)
        )

    @service(Task.Service.TOGGLE_BREAKPOINT)
    async def toggle_breakpoint(
        self, req: ToggleBreakpointRequest
    ) -> TaskControlResponse:
        return TaskControlResponse(
            ok=self._runner(req.robot_id).toggle_breakpoint(req.step_id)
        )

    def _publish_tree(self, robot_id: str, spec: TaskSpec) -> None:
        tree = task_tree(spec)
        seq = self._tree_seq.get(robot_id, 0)
        self.runtime.publish(
            Task.Stream.TREE,
            TaskTree(
                robot_id=robot_id,
                seq=seq,
                timestamp_unix=time.time(),
                task_name=tree["task_name"],
                description=tree["description"],
                steps=tree["steps"],
            ),
        )
        self._tree_seq[robot_id] = seq + 1
