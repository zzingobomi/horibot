import logging
import threading
from typing import Callable

from core.transport.application_node import ApplicationNode
from core.transport.messages.base import EmptyData, ServiceRequest, ServiceResponse
from core.transport.messages.task import TaskStepIdReq
from core.transport.topic_map import Service, Topic
from core.cache.joint_state_cache import JointStateCache
from modules.motor.motor_config import load_motor_layout
from modules.calibration.loader import load_calibration
from modules.llm import prompt_parser
from modules.llm.prompt_parser import parse_pick_place
from modules.task.step import Task, task_tree
from modules.task.task_runner import TaskRunner
from modules.task.tasks.pick_and_place import create_pick_and_place_task

logger = logging.getLogger(__name__)


def _factory_pick_and_place(data: dict) -> Task:
    prompt = str(data.get("prompt") or "").strip()
    if not prompt:
        raise ValueError("prompt 필요")
    pick_object, place_object = parse_pick_place(prompt)
    return create_pick_and_place_task(
        pick_object=pick_object,
        place_object=place_object,
    )


TASK_REGISTRY: dict[str, Callable[[dict], Task]] = {
    "pick_and_place": _factory_pick_and_place,
}


class TaskNode(ApplicationNode):
    """Application 노드 — multi-robot orchestration.

    Phase 2 의 Step DSL `robot_id` field (multi_robot_walkthrough §8 F) 도입
    전까지는 default robot 의 arm_cfgs / calibration 만 사용 (transition).
    step 안에서 어느 robot 의 motion / detector 호출할지 결정하는 자리는
    Step DSL 변경과 묶임.
    """

    def __init__(self) -> None:
        super().__init__("task_node")

        # transition — Step DSL robot_id field 도입 전까지 default robot 만.
        default_rid = self._registry.default().robot_id
        layout = load_motor_layout(default_rid)
        self._arm_cfgs = layout.arm
        self._gripper_cfg = layout.gripper
        self._joint_cache = JointStateCache()

        calib = load_calibration(default_rid)
        if not calib.is_ready():
            logger.warning(
                "TaskNode: 캘리브레이션 미완료 — DetectStep 사용 불가 "
                "(intrinsic=%s, hand_eye=%s)",
                calib.intrinsic is not None,
                calib.hand_eye is not None,
            )

        self._runner = TaskRunner(
            node=self,
            joint_cache=self._joint_cache,
            arm_cfgs=self._arm_cfgs,
            gripper_cfg=self._gripper_cfg,
            calibration=calib,
            on_state_change=self._on_state_change,
        )

    def start(self) -> None:
        self._joint_cache.subscribe(self)

        # RUN / PREVIEW / STATUS 는 typed 면제 (factory 동적 인자 + tree/state 자유 dict
        # 응답 — typed_messaging.md §마이그레이션 사유). legacy create_service form.
        self.create_service(self.r(Service.TASK_RUN), self._handle_run)
        self.create_service(self.r(Service.TASK_PREVIEW), self._handle_preview)
        self.create_service(self.r(Service.TASK_STATUS), self._handle_status)
        # 단순 명령들은 typed
        self.create_service(
            self.r(Service.TASK_STOP), EmptyData, EmptyData, self._handle_stop
        )
        self.create_service(
            self.r(Service.TASK_PAUSE), EmptyData, EmptyData, self._handle_pause
        )
        self.create_service(
            self.r(Service.TASK_RESUME), EmptyData, EmptyData, self._handle_resume
        )
        self.create_service(
            self.r(Service.TASK_STEP), EmptyData, EmptyData, self._handle_step
        )
        self.create_service(
            self.r(Service.TASK_RUN_TO),
            TaskStepIdReq,
            EmptyData,
            self._handle_run_to,
        )
        self.create_service(
            self.r(Service.TASK_TOGGLE_BREAKPOINT),
            TaskStepIdReq,
            EmptyData,
            self._handle_toggle_breakpoint,
        )

        super().start()

        # LLM prompt parser 백그라운드 preload — 첫 task 호출의 체감 지연 제거.
        # 로드 중 parse_pick_place 호출되면 내부 lock 이 기다림.
        # [detector_node 의 Grounding DINO preload](backend/nodes/detector_node.py)
        # 와 같은 패턴.
        threading.Thread(
            target=self._preload_prompt_parser,
            daemon=True,
            name="prompt-parser-preload",
        ).start()

        logger.info("TaskNode 시작")

    def _preload_prompt_parser(self) -> None:
        try:
            prompt_parser.preload()
        except Exception:
            logger.exception("LLM prompt parser preload 실패")

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

        # tree 를 먼저 publish — frontend 가 첫 state 변경 (RUNNING) 받기 전에
        # 트리 구조를 알고 있어야 step_id 매칭 가능. Zenoh put 은 sync 이므로
        # 순서 보장.
        try:
            self.publish(self.r(Topic.TASK_TREE), task_tree(task))
        except Exception as exc:
            logger.warning("TASK_TREE 발행 실패: %s", exc)

        if not self._runner.run(task):
            return {"success": False, "message": "Task 시작 실패", "data": {}}

        logger.info("Task 시작: %s", task_name)
        return {"success": True, "message": "ok", "data": {}}

    def _handle_stop(
        self, _req: ServiceRequest[EmptyData]
    ) -> ServiceResponse[EmptyData]:
        self._runner.stop()
        return ServiceResponse(success=True, message="ok", data=EmptyData())

    def _handle_pause(
        self, _req: ServiceRequest[EmptyData]
    ) -> ServiceResponse[EmptyData]:
        ok = self._runner.pause()
        return ServiceResponse(
            success=ok,
            message="ok" if ok else "RUNNING 상태 아님",
            data=EmptyData() if ok else None,
        )

    def _handle_resume(
        self, _req: ServiceRequest[EmptyData]
    ) -> ServiceResponse[EmptyData]:
        ok = self._runner.resume()
        return ServiceResponse(
            success=ok,
            message="ok" if ok else "PAUSED 상태 아님",
            data=EmptyData() if ok else None,
        )

    def _handle_status(self, _req: dict) -> dict:
        """legacy — state.to_dict 가 free-form (typed 면제)."""
        return {"success": True, "message": "ok", "data": self._runner.state.to_dict()}

    def _handle_step(
        self, _req: ServiceRequest[EmptyData]
    ) -> ServiceResponse[EmptyData]:
        ok = self._runner.step_once()
        return ServiceResponse(
            success=ok,
            message="ok" if ok else "PAUSED 상태 아님",
            data=EmptyData() if ok else None,
        )

    def _handle_run_to(
        self, req: ServiceRequest[TaskStepIdReq]
    ) -> ServiceResponse[EmptyData]:
        step_id = req.data.step_id.strip()
        if not step_id:
            return ServiceResponse(
                success=False, message="step_id 필요", data=None
            )
        ok = self._runner.run_to(step_id)
        return ServiceResponse(
            success=ok,
            message="ok" if ok else "PAUSED 상태 아님",
            data=EmptyData() if ok else None,
        )

    def _handle_toggle_breakpoint(
        self, req: ServiceRequest[TaskStepIdReq]
    ) -> ServiceResponse[EmptyData]:
        step_id = req.data.step_id.strip()
        if not step_id:
            return ServiceResponse(
                success=False, message="step_id 필요", data=None
            )
        self._runner.toggle_breakpoint(step_id)
        return ServiceResponse(success=True, message="ok", data=EmptyData())

    def _handle_preview(self, req: dict) -> dict:
        """task factory 만 실행해서 tree 만 빌드 (실행 X). Run 전 사전 표시용.

        응답으로 tree 반환 + TASK_TREE 토픽으로도 broadcast — 다른 클라이언트
        / 같은 클라이언트의 다른 패널도 동기화.

        실행 중인 task 가 있으면 preview 가 그 트리를 덮어쓰지 않도록 거절.
        """
        if self._runner.is_running():
            return {
                "success": False,
                "message": "실행 중 — 현재 task 종료 후 preview",
                "data": {},
            }

        data = req.get("data", {})
        task_name = data.get("task", "pick_and_place")
        factory = TASK_REGISTRY.get(task_name)
        if factory is None:
            return {
                "success": False,
                "message": f"알 수 없는 task: {task_name}",
                "data": {},
            }

        try:
            task = factory(data)
        except (KeyError, ValueError, TypeError) as e:
            return {
                "success": False,
                "message": f"task 인자 오류: {e}",
                "data": {},
            }

        tree = task_tree(task)
        try:
            self.publish(self.r(Topic.TASK_TREE), tree)
        except Exception as exc:
            logger.warning("TASK_TREE preview 발행 실패: %s", exc)

        return {"success": True, "message": "ok", "data": tree}

    # ── Publishers ────────────────────────────────

    def _on_state_change(self, state) -> None:
        try:
            self.publish(self.r(Topic.TASK_STATE), state.to_dict())
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
