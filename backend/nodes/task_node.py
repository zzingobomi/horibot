import logging

from core.base_node import BaseNode
from core.topic_map import Service, Topic
from core.joint_state_cache import JointStateCache
from core.common import GRIPPER_ID
from modules.dynamixel.motor_config import MotorConfig, load_motor_config
from modules.calibration.loader import load_calibration
from modules.task.step_executor import StepExecutor
from modules.task.task_runner import TaskRunner
from modules.task.tasks.pick_and_place import create_pick_and_place_task
from modules.kinematics.solver import Position3

logger = logging.getLogger(__name__)

TASK_REGISTRY = {
    "pick_and_place": create_pick_and_place_task,
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
        place_pos_raw = data.get("place_position", [0.15, 0.0, 0.05])
        place_pos: Position3 = Position3(place_pos_raw)

        factory = TASK_REGISTRY.get(task_name)
        if factory is None:
            return {
                "success": False,
                "message": f"알 수 없는 task: {task_name}",
                "data": {},
            }

        if self._runner.is_running():
            return {"success": False, "message": "이미 실행 중인 Task 있음", "data": {}}

        task = factory(place_pos)
        if not self._runner.run(task):
            return {"success": False, "message": "Task 시작 실패", "data": {}}

        logger.info("Task 시작: %s  place=%s", task_name, place_pos)
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
