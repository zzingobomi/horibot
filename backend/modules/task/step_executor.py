import logging
import threading
import time
from typing import TYPE_CHECKING

from core.common import GRIPPER_SETTLE
from core.types import TrajStatus
from core.topic_map import Service, Topic
from modules.calibration.loader import CalibrationData
from .step_types import (
    DetectStep,
    GripperStep,
    HomeStep,
    MoveTCPStep,
    Step,
    TaskContext,
    WaitStep,
)

if TYPE_CHECKING:
    from core.base_node import BaseNode
    from core.joint_state_cache import JointStateCache
    from modules.dynamixel.motor_config import MotorConfig


logger = logging.getLogger(__name__)


TRAJ_WAIT_TIMEOUT = 30.0


class StepExecutor:
    def __init__(
        self,
        node: "BaseNode",
        joint_cache: "JointStateCache",
        arm_cfgs: list["MotorConfig"],
        calibration: CalibrationData | None = None,
    ) -> None:
        self._node = node
        self._joint_cache = joint_cache
        self._arm_cfgs = arm_cfgs
        self._calib = calibration

        self._traj_event = threading.Event()
        self._traj_status = TrajStatus.IDLE

        self._node.create_subscriber(
            Topic.MOTION_STATE_TRAJ,
            self._on_traj_state,
        )

    # ─── Execute ─────────────────────────────────────────────────

    def execute(self, step: Step, context: TaskContext) -> bool:
        match step.type:
            case "move_tcp":
                return self._move_tcp(step, context)
            case "gripper":
                return self._gripper(step)
            case "detect":
                return self._detect(step, context)
            case "wait":
                return self._wait(step)
            case "home":
                return self._home(step)
            case _:
                logger.error("알 수 없는 step type: %s", step.type)
                return False

    # ─── Step 구현 ─────────────────────────────────────────────────

    def _move_tcp(self, step: MoveTCPStep, context: TaskContext) -> bool:
        if step.position_key is not None:
            base_pos = context.get(step.position_key)
            if base_pos is None:
                logger.error("MoveTCPStep: context에 '%s' 없음", step.position_key)
                return False
            position = [b + o for b, o in zip(base_pos, step.offset)]
        else:
            position = [b + o for b, o in zip(step.position, step.offset)]

        logger.info("MoveL → %.3f, %.3f, %.3f  [%s]", *position, step.label)

        self._traj_event.clear()
        res = self._node.call_service(
            Service.MOTION_MOVE_L,
            {"position": position},
        )
        if not res.get("success"):
            logger.error("MoveL 서비스 실패: %s", res.get("message"))
            return False

        return self._wait_for_traj()

    def _gripper(self, step: GripperStep) -> bool:
        logger.info(
            "Gripper %s  current=%d  [%s]", step.action, step.current, step.label
        )

        res = self._node.call_service(
            Service.MOTOR_GRIPPER,
            {"action": step.action, "current": step.current},
        )
        if not res.get("success"):
            logger.error("Gripper 서비스 실패: %s", res.get("message"))
            return False

        time.sleep(GRIPPER_SETTLE)
        return True

    def _detect(self, step: DetectStep, context: TaskContext) -> bool:
        logger.info("Detect 시작  [%s]", step.label)

        res = self._node.call_service(Service.DETECT_SERVICE, {})
        if not res.get("success"):
            logger.error("Detect 서비스 실패: %s", res.get("message"))
            return False

        position = res.get("data", {}).get("position")
        if position is None:
            logger.error("Detect: position 없음")
            return False

        logger.info("Detect 성공: base=(%.3f, %.3f, %.3f)", *position)
        context.set(step.output_key, position)
        return True

    def _wait(self, step: WaitStep) -> bool:
        logger.info("Wait %.2fs  [%s]", step.duration_sec, step.label)
        time.sleep(step.duration_sec)
        return True

    def _home(self, step: HomeStep) -> bool:
        logger.info("Home으로 복귀")

        home_joints = [{"id": cfg.id, "degree": 0.0} for cfg in self._arm_cfgs]

        self._traj_event.clear()
        res = self._node.call_service(
            Service.MOTION_MOVE_J,
            {"joints": home_joints},
        )
        if not res.get("success"):
            logger.error("Home MoveJ 실패: %s", res.get("message"))
            return False

        return self._wait_for_traj()

    # ─── 내부 유틸 ─────────────────────────────────────────────────

    def _on_traj_state(self, data: dict) -> None:
        status = data.get("status", "")
        self._traj_status = status
        if status in (TrajStatus.DONE, TrajStatus.FAILED, TrajStatus.STOPPED):
            self._traj_event.set()

    def _wait_for_traj(self, timeout: float = TRAJ_WAIT_TIMEOUT) -> bool:
        triggered = self._traj_event.wait(timeout=timeout)
        if not triggered:
            logger.warning("궤적 대기 timeout (%.0fs)", timeout)
            return False
        return self._traj_status == TrajStatus.DONE
