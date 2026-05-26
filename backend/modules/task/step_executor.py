import logging
import threading
import time
from typing import TYPE_CHECKING

from core.common import GRIPPER_ID, GRIPPER_SETTLE
from core.robot_poses import list_pose_names, load_pose
from core.types import TrajStatus
from core.topic_map import Service, Topic
from modules.calibration.loader import CalibrationData
from .step_types import (
    DetectStep,
    GraspPolicyStep,
    GripperStep,
    GroundedDetectStep,
    HomeStep,
    MoveTCPStep,
    PlacePolicyStep,
    SearchAndDetectStep,
    SelfPlayStep,
    Step,
    TaskContext,
    VerifyGraspStep,
    WaitStep,
)

if TYPE_CHECKING:
    from core.base_node import BaseNode
    from core.joint_state_cache import JointStateCache
    from modules.dynamixel.motor_config import MotorConfig
    from .self_play.runner import SelfPlayRunner


logger = logging.getLogger(__name__)


TRAJ_WAIT_TIMEOUT = 30.0

# close 후 gripper Present_Position 이 이 값 미만이면 빈손으로 판정.
# self-play 의 GRIPPER_HELD_THRESHOLD 와 같은 의미 (1800 까지 끝까지 닫혔으면 빈손).
GRIPPER_HELD_THRESHOLD = 1900

# 단발 샘플링은 큐브 모서리 임시 catch 를 빈손과 못 구분 (pos=1950 에서 stall →
# threshold 통과 → 직후 cube 슬립). 0.3s 추가 settle 후 재측정해서 position 이
# 안정적인지 확인. SLIP_DELTA 만큼 감소 = 아직 close 중 = 실제로는 빈손.
GRIPPER_HELD_RECHECK_DELAY = 0.3
GRIPPER_HELD_SLIP_DELTA = 30


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

        # self-play runner lazy 인스턴스화 (첫 호출 시 생성, 이후 재사용 — Zenoh
        # subscriber 가 누적되지 않게 하기 위함).
        self._self_play_runner: "SelfPlayRunner | None" = None

    # ─── Execute ─────────────────────────────────────────────────

    def execute(
        self,
        step: Step,
        context: TaskContext,
        stop_event: threading.Event | None = None,
    ) -> bool:
        match step.type:
            case "move_tcp":
                return self._move_tcp(step, context)
            case "gripper":
                return self._gripper(step)
            case "detect":
                return self._detect(step, context)
            case "grounded_detect":
                return self._grounded_detect(step, context)
            case "search_and_detect":
                return self._search_and_detect(step, context)
            case "grasp_policy":
                return self._grasp_policy(step, context)
            case "place_policy":
                return self._place_policy(step, context)
            case "verify_grasp":
                return self._verify_grasp(step)
            case "wait":
                return self._wait(step)
            case "home":
                return self._home(step)
            case "self_play":
                return self._self_play(step, stop_event)
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
            "Gripper %s  current=%d  verify=%s  [%s]",
            step.action, step.current, step.verify_grasp, step.label,
        )

        res = self._node.call_service(
            Service.MOTOR_GRIPPER,
            {"action": step.action, "current": step.current},
        )
        if not res.get("success"):
            logger.error("Gripper 서비스 실패: %s", res.get("message"))
            return False

        time.sleep(GRIPPER_SETTLE)

        if step.action == "close" and step.verify_grasp:
            return self._verify_gripper_held(step.label or "close_gripper")

        return True

    def _verify_gripper_held(self, label: str) -> bool:
        """Gripper Present_Position 으로 잡힘 검증.

        모서리 임시 catch → slip 케이스를 잡기 위해 GRIPPER_HELD_RECHECK_DELAY
        간격으로 두 번 측정. 두 번째 측정이 threshold 미만 이거나 첫 측정 대비
        SLIP_DELTA 이상 감소했으면 jaw 가 아직 닫히는 중 = 실제로는 빈손.
        """
        pos1 = self._joint_cache.get_raw(GRIPPER_ID)
        if pos1 is None:
            logger.error("Gripper verify: Present_Position 없음  [%s]", label)
            return False
        if pos1 < GRIPPER_HELD_THRESHOLD:
            logger.error(
                "Gripper verify 실패: 빈손 (pos=%d < %d)  [%s]",
                pos1, GRIPPER_HELD_THRESHOLD, label,
            )
            return False

        time.sleep(GRIPPER_HELD_RECHECK_DELAY)
        pos2 = self._joint_cache.get_raw(GRIPPER_ID)
        if pos2 is None:
            logger.error(
                "Gripper verify (recheck): Present_Position 없음  [%s]", label
            )
            return False
        if pos2 < GRIPPER_HELD_THRESHOLD:
            logger.error(
                "Gripper verify 실패: 재측정 시 빈손 (pos %d → %d < %d)  [%s]",
                pos1, pos2, GRIPPER_HELD_THRESHOLD, label,
            )
            return False
        if pos1 - pos2 > GRIPPER_HELD_SLIP_DELTA:
            logger.error(
                "Gripper verify 실패: slip 중 (pos %d → %d, Δ=%d > %d)  [%s]",
                pos1, pos2, pos1 - pos2, GRIPPER_HELD_SLIP_DELTA, label,
            )
            return False
        logger.info(
            "Gripper verify OK: pos %d → %d  [%s]", pos1, pos2, label
        )
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

    def _grounded_detect(
        self, step: GroundedDetectStep, context: TaskContext
    ) -> bool:
        prompt = step.prompt.strip()
        if not prompt:
            logger.error("GroundedDetectStep: prompt 비어있음")
            return False

        logger.info("GroundedDetect '%s'  [%s]", prompt, step.label)

        res = self._node.call_service(
            Service.PERCEPTION_GROUNDED_DETECT,
            {"prompt": prompt},
            timeout=60.0,  # 첫 호출 시 모델 로드 시간 포함
        )
        if not res.get("success"):
            logger.error("GroundedDetect 서비스 실패: %s", res.get("message"))
            return False

        data = res.get("data", {})
        position = data.get("position")
        if position is None:
            logger.error("GroundedDetect: position 없음")
            return False

        logger.info(
            "GroundedDetect 성공: conf=%.2f base=(%.3f, %.3f, %.3f)",
            data.get("confidence", 0.0),
            *position,
        )
        context.set(step.output_key, position)
        # GraspPolicyStep이 height/base_z를 쓰도록 meta를 별도 키로 저장.
        # detect 결과에 base_z/height가 비어있어도 안전한 default (0.0/0.0).
        context.set(
            f"{step.output_key}_meta",
            {
                "base_z": float(data.get("base_z", 0.0)),
                "height": float(data.get("height", 0.0)),
            },
        )
        return True

    def _search_and_detect(
        self, step: SearchAndDetectStep, context: TaskContext
    ) -> bool:
        prompt = step.prompt.strip()
        if not prompt:
            logger.error("SearchAndDetectStep: prompt 비어있음")
            return False

        search_poses = list_pose_names("search_")
        if not search_poses:
            logger.error(
                "SearchAndDetect: search pose 없음 "
                "(robot_poses.yaml 의 search_* 등록 필요)"
            )
            return False

        SEARCH_SETTLE = 0.5

        for pose_name in search_poses:
            logger.info("Search pose '%s' 로 이동  [%s]", pose_name, step.label)
            try:
                joints = load_pose(pose_name)
            except KeyError as exc:
                logger.warning("자세 로드 실패: %s", exc)
                continue

            self._traj_event.clear()
            res = self._node.call_service(
                Service.MOTION_MOVE_J,
                {"joints": joints},
            )
            if not res.get("success"):
                logger.warning("MoveJ 실패: %s", res.get("message"))
                continue
            if not self._wait_for_traj():
                logger.warning("Search pose '%s' 도달 실패", pose_name)
                continue

            time.sleep(SEARCH_SETTLE)

            logger.info("[%s] '%s' detect 시도", pose_name, prompt)
            res = self._node.call_service(
                Service.PERCEPTION_GROUNDED_DETECT,
                {"prompt": prompt},
                timeout=60.0,
            )
            if not res.get("success"):
                logger.info(
                    "'%s' detect 실패 (%s): %s",
                    prompt, pose_name, res.get("message"),
                )
                continue

            data = res.get("data", {})
            position = data.get("position")
            if position is None:
                continue

            logger.info(
                "'%s' detect 성공 [%s]: conf=%.2f base=(%.3f, %.3f, %.3f)",
                prompt, pose_name, data.get("confidence", 0.0), *position,
            )
            context.set(step.output_key, position)
            context.set(
                f"{step.output_key}_meta",
                {
                    "base_z": float(data.get("base_z", 0.0)),
                    "height": float(data.get("height", 0.0)),
                },
            )
            return True

        logger.error(
            "SearchAndDetect: '%s' 모든 search pose 에서 fail", prompt
        )
        return False

    def _place_policy(self, step: PlacePolicyStep, context: TaskContext) -> bool:
        pos = context.get(step.input_key)
        if not isinstance(pos, (list, tuple)) or len(pos) < 3:
            logger.error(
                "PlacePolicy: context['%s']가 없거나 잘못된 형식 "
                "(SearchAndDetect 가 먼저 실행돼야 함)",
                step.input_key,
            )
            return False
        x, y, top_z = float(pos[0]), float(pos[1]), float(pos[2])
        place_z = top_z + step.drop_clearance
        logger.info(
            "PlacePolicy: top_z=%.3f → place_z=%.3f (clearance=%.3f)  [%s]",
            top_z, place_z, step.drop_clearance, step.label,
        )
        context.set(step.output_key, [x, y, place_z])
        return True

    def _grasp_policy(self, step: GraspPolicyStep, context: TaskContext) -> bool:
        pos = context.get(step.input_key)
        if not isinstance(pos, (list, tuple)) or len(pos) < 3:
            logger.error(
                "GraspPolicy: context['%s']가 없거나 잘못된 형식 "
                "(GroundedDetect가 먼저 실행돼야 함)",
                step.input_key,
            )
            return False

        meta_raw = context.get(f"{step.input_key}_meta")
        meta: dict = meta_raw if isinstance(meta_raw, dict) else {}
        base_z = float(meta.get("base_z", 0.0))
        height = float(meta.get("height", 0.0))

        x, y, _top_z = float(pos[0]), float(pos[1]), float(pos[2])

        grasp_z = base_z + height * step.grasp_ratio

        logger.info(
            "GraspPolicy base_z=%.3f height=%.3f → grasp_z=%.3f  [%s]",
            base_z, height, grasp_z, step.label,
        )
        context.set(step.output_key, [x, y, grasp_z])
        return True

    def _verify_grasp(self, step: VerifyGraspStep) -> bool:
        return self._verify_gripper_held(step.label or "verify_grasp")

    def _wait(self, step: WaitStep) -> bool:
        logger.info("Wait %.2fs  [%s]", step.duration_sec, step.label)
        time.sleep(step.duration_sec)
        return True

    def _self_play(
        self,
        step: SelfPlayStep,
        stop_event: threading.Event | None,
    ) -> bool:
        from datetime import datetime
        from pathlib import Path

        if self._self_play_runner is None:
            # lazy import 로 self_play 모듈의 의존성이 import 트리에 끌려들어가지
            # 않게 함 (motor Pi 같은 분산 노드 보호).
            from .self_play.runner import SelfPlayRunner

            self._self_play_runner = SelfPlayRunner(
                node=self._node,
                joint_cache=self._joint_cache,
                arm_cfgs=self._arm_cfgs,
                calibration=self._calib,
            )

        runner = self._self_play_runner
        # 이전 task 의 stop 상태가 남으면 안 됨
        runner._stop_requested.clear()

        ts = datetime.now().strftime("%Y%m%d-%H%M%S")
        log_path = Path(step.log_dir) / f"self_play_{ts}.jsonl"

        logger.info("self-play step 시작 → %s", log_path)
        return runner.run(
            prompt=step.prompt,
            max_attempts=step.max_attempts,
            log_path=log_path,
            stop_event=stop_event,
            gripper_setup=step.gripper_setup,
        )

    def _home(self, step: HomeStep) -> bool:
        logger.info("Home으로 복귀")

        home_joints = load_pose("home")

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
