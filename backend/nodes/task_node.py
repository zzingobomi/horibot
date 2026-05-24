import logging
from typing import Callable

from core.base_node import BaseNode
from core.topic_map import Service, Topic
from core.joint_state_cache import JointStateCache
from core.common import GRIPPER_ID
from core.gripper_setup import GripperSetup
from modules.dynamixel.motor_config import MotorConfig, load_motor_config
from modules.calibration.loader import load_calibration
from modules.task.step_executor import StepExecutor
from modules.task.step_types import Task
from modules.task.task_runner import TaskRunner
from modules.task.tasks.pick_and_place import create_pick_and_place_task
from modules.task.tasks.pick_named_object import create_pick_named_object_task
from modules.task.tasks.self_play_pick import create_self_play_pick_task
from modules.kinematics.solver import Position3

logger = logging.getLogger(__name__)

DEFAULT_PLACE_POSITION = [0.15, 0.0, 0.05]


def _factory_pick_and_place(data: dict) -> Task:
    place = data.get("place_position", DEFAULT_PLACE_POSITION)
    return create_pick_and_place_task(place_position=Position3(place))


def _factory_pick_named_object(data: dict) -> Task:
    prompt = str(data.get("prompt", "")).strip()
    if not prompt:
        raise ValueError("prompt 필요")
    place = data.get("place_position", DEFAULT_PLACE_POSITION)
    return create_pick_named_object_task(
        prompt=prompt,
        place_position=Position3(place),
    )


def _factory_self_play_pick(data: dict) -> Task:
    prompt = str(data.get("prompt", "")).strip()
    if not prompt:
        raise ValueError("prompt 필요")
    max_attempts = int(data.get("max_attempts", 100))
    if max_attempts <= 0:
        raise ValueError(f"max_attempts > 0 필요: {max_attempts}")
    log_dir = data.get("log_dir")  # None → factory default (root robot/logs/)

    # frontend 객체 type preset → nested dict {close_current, ...} → GripperSetup
    gs_raw = data.get("gripper_setup")
    gripper_setup = (
        GripperSetup(**{k: v for k, v in gs_raw.items() if v is not None})
        if isinstance(gs_raw, dict) else None
    )

    return create_self_play_pick_task(
        prompt=prompt,
        max_attempts=max_attempts,
        log_dir=log_dir,
        gripper_setup=gripper_setup,
    )


TASK_REGISTRY: dict[str, Callable[[dict], Task]] = {
    "pick_and_place": _factory_pick_and_place,
    "pick_named_object": _factory_pick_named_object,
    "self_play_pick": _factory_self_play_pick,
}


class TaskNode(BaseNode):
    def __init__(self) -> None:
        super().__init__("task_node")

        _, self._motor_cfgs = load_motor_config()
        self._arm_cfgs: list[MotorConfig] = [
            m for m in self._motor_cfgs if m.id != GRIPPER_ID
        ]
        self._joint_cache = JointStateCache()

        calib = load_calibration()
        if not calib.is_ready():
            logger.warning(
                "TaskNode: 캘리브레이션 미완료 — DetectStep 사용 불가 "
                "(intrinsic=%s, hand_eye=%s)",
                calib.intrinsic is not None,
                calib.hand_eye is not None,
            )

        self._executor = StepExecutor(
            node=self,
            joint_cache=self._joint_cache,
            arm_cfgs=self._arm_cfgs,
            calibration=calib,
        )
        self._runner = TaskRunner(
            executor=self._executor,
            on_state_change=self._on_state_change,
        )

    def start(self) -> None:
        self._joint_cache.subscribe(self)

        self.create_service(Service.TASK_RUN, self._handle_run)
        self.create_service(Service.TASK_STOP, self._handle_stop)
        self.create_service(Service.TASK_PAUSE, self._handle_pause)
        self.create_service(Service.TASK_RESUME, self._handle_resume)
        self.create_service(Service.TASK_STATUS, self._handle_status)

        super().start()
        logger.info("TaskNode 시작")

    # ── Service handlers ──────────────────────────────────────

    def _handle_run(self, req: dict) -> dict:
        data = req.get("data", {})
        task_name = data.get("task", "pick_and_place")

        factory = TASK_REGISTRY.get(task_name)
        if factory is None:
            return {
                "success": False,
                "message": f"알 수 없는 task: {task_name}",
                "data": {},
            }

        if self._runner.is_running():
            return {"success": False, "message": "이미 실행 중인 Task 있음", "data": {}}

        try:
            task = factory(data)
        except (KeyError, ValueError, TypeError) as e:
            return {
                "success": False,
                "message": f"task 인자 오류: {e}",
                "data": {},
            }

        if not self._runner.run(task):
            return {"success": False, "message": "Task 시작 실패", "data": {}}

        logger.info("Task 시작: %s", task_name)
        return {"success": True, "message": "ok", "data": {}}

    def _handle_stop(self, _req: dict) -> dict:
        self._runner.stop()
        return {"success": True, "message": "ok", "data": {}}

    def _handle_pause(self, _req: dict) -> dict:
        ok = self._runner.pause()
        return {
            "success": ok,
            "message": "ok" if ok else "RUNNING 상태 아님",
            "data": {},
        }

    def _handle_resume(self, _req: dict) -> dict:
        ok = self._runner.resume()
        return {
            "success": ok,
            "message": "ok" if ok else "PAUSED 상태 아님",
            "data": {},
        }

    def _handle_status(self, _req: dict) -> dict:
        return {"success": True, "message": "ok", "data": self._runner.state.to_dict()}

    # ── Publishers ────────────────────────────────

    def _on_state_change(self, state) -> None:
        try:
            self.publish(Topic.TASK_STATE, state.to_dict())
        except Exception as exc:
            logger.warning("state 발행 실패: %s", exc)

        logger.info(
            "[%s] %s  step=%d/%d  label=%s  err=%s",
            state.task_name,
            state.status.value,
            state.current_step,
            state.total_steps,
            state.current_label,
            state.error or "-",
        )
