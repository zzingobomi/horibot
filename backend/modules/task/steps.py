"""Step primitives — typed Slot DSL 기반.

이전 [step_types.py](step_types.py) 의 dataclass 정의 + [step_executor.py]
(step_executor.py) 의 핸들러를 한 클래스로 흡수. 각 step 이 자기 `execute(ctx)`
를 보유 → polymorphic dispatch (lego test #3).

입력은 `SlotOr[T]` — literal 값 또는 다른 step 의 `out` 둘 다 받음:
    grasp = GraspPolicy(target=pick.out, grasp_ratio=0.5)
    MoveTCP(target=grasp.out, offset=Position3(0, 0, 0.06))

출력은 Step[T_out] 의 T_out — `step.out` 으로 다음 step 에 넘김. 사이드이펙트만
있는 step (MoveTCP/Gripper/...) 은 `Step[None]` — out 은 사용하지 않음.

스코프 — v1 에서는 Position3 기반 (현 동작 1:1 보존). Pose6 / orientation 은
Palletizing 본 작업에서 도입.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass

from core.common import GRIPPER_ID, GRIPPER_SETTLE
from core.robot_poses import list_pose_names, load_pose
from core.topic_map import Service
from modules.task.schema import Detection, Position3, SlotOr
from modules.task.step import Step, StepContext

logger = logging.getLogger(__name__)


# ─── Constants — step_executor 에서 그대로 옮겨옴 ──────────────────────


# close 후 gripper Present_Position 이 이 값 미만이면 빈손으로 판정.
GRIPPER_HELD_THRESHOLD = 1900

# 단발 샘플링은 큐브 모서리 임시 catch 를 빈손과 못 구분. 0.3s settle 후 재측정.
GRIPPER_HELD_RECHECK_DELAY = 0.3
GRIPPER_HELD_SLIP_DELTA = 30


# ─── Wait / Home — 가장 단순한 primitive ──────────────────────────────


@dataclass(kw_only=True)
class Wait(Step[None]):
    duration_sec: float = 0.5

    def execute(self, ctx: StepContext) -> None:
        logger.info("Wait %.2fs  [%s]", self.duration_sec, self.label)
        time.sleep(self.duration_sec)


@dataclass(kw_only=True)
class Home(Step[None]):
    """home 자세 (robot_poses.yaml) 로 MoveJ."""

    def execute(self, ctx: StepContext) -> None:
        logger.info("Home으로 복귀  [%s]", self.label)
        joints = load_pose("home")
        if not ctx.call_motion(Service.MOTION_MOVE_J, {"joints": joints}):
            raise RuntimeError("Home MoveJ 실패")


# ─── MoveTCP — base-frame Position3 로 MoveL ──────────────────────────


@dataclass(kw_only=True)
class MoveTCP(Step[None]):
    """베이스 프레임 (x, y, z) 위치로 MoveL.

    target: Position3 literal 또는 다른 step (Detection/Position3 출력) 의 Slot.
            Detection 슬롯도 받음 — 내부에서 .position 으로 추출.
    offset: target 에 더할 오프셋 (pre-grasp/lift 등).
    """

    target: SlotOr[Position3 | Detection]
    offset: Position3 = Position3(0.0, 0.0, 0.0)

    def execute(self, ctx: StepContext) -> None:
        resolved = ctx.resolve(self.target)
        if isinstance(resolved, Detection):
            base = resolved.position
        elif isinstance(resolved, Position3):
            base = resolved
        else:
            raise TypeError(
                f"MoveTCP.target: Position3/Detection 기대, {type(resolved).__name__} 받음"
            )

        position = base + self.offset
        logger.info(
            "MoveL → %.3f, %.3f, %.3f  [%s]",
            position.x, position.y, position.z, self.label,
        )
        if not ctx.call_motion(
            Service.MOTION_MOVE_L,
            {"position": position.to_list()},
        ):
            raise RuntimeError(f"MoveL 실패 [{self.label}]")


# ─── Gripper — open / close (+ optional verify) ───────────────────────


@dataclass(kw_only=True)
class Gripper(Step[None]):
    """Gripper open/close. close 시 verify_grasp 로 즉시 검증 가능.

    중간 검증 (lift 후 등) 은 별도 `VerifyGrasp` 사용 — lego 정신상 두 개
    primitive 로 분리. close + 즉시 검증은 동작상 한 블록에 묶는 게 단순해서
    flag 로 유지.
    """

    action: str = "open"  # "open" | "close"
    current: int = 200  # mA, 파지력
    verify_grasp: bool = False

    def execute(self, ctx: StepContext) -> None:
        logger.info(
            "Gripper %s  current=%d  verify=%s  [%s]",
            self.action, self.current, self.verify_grasp, self.label,
        )
        res = ctx.call_service(
            Service.MOTOR_GRIPPER,
            {"action": self.action, "current": self.current},
        )
        if not res.get("success"):
            raise RuntimeError(
                f"Gripper 서비스 실패: {res.get('message')} [{self.label}]"
            )

        time.sleep(GRIPPER_SETTLE)

        if self.action == "close" and self.verify_grasp:
            if not _verify_gripper_held(ctx, self.label or "close_gripper"):
                raise RuntimeError(
                    f"Gripper verify 실패 — 빈손 [{self.label}]"
                )


@dataclass(kw_only=True)
class VerifyGrasp(Step[None]):
    """현재 그리퍼 Present_Position 으로 잡힘 검증 (mid-task)."""

    def execute(self, ctx: StepContext) -> None:
        if not _verify_gripper_held(ctx, self.label or "verify_grasp"):
            raise RuntimeError(f"VerifyGrasp 실패 — 빈손 [{self.label}]")


def _verify_gripper_held(ctx: StepContext, label: str) -> bool:
    """두 단계 측정으로 빈손/slip 검출.

    [step_executor.py:_verify_gripper_held](step_executor.py) 와 동일 로직.
    GRIPPER_HELD_RECHECK_DELAY 간격으로 두 번 측정 → threshold 미만 또는
    SLIP_DELTA 이상 감소 시 빈손 판정.
    """
    pos1 = ctx.joint_cache.get_raw(GRIPPER_ID)
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
    pos2 = ctx.joint_cache.get_raw(GRIPPER_ID)
    if pos2 is None:
        logger.error("Gripper verify (recheck): Present_Position 없음  [%s]", label)
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
    logger.info("Gripper verify OK: pos %d → %d  [%s]", pos1, pos2, label)
    return True


# ─── Detection — GroundedDetect / SearchAndDetect ─────────────────────


@dataclass(kw_only=True)
class GroundedDetect(Step[Detection]):
    """현재 자세에서 Grounding DINO 로 prompt 객체 1회 검출.

    출력 Detection 은 {position, height, base_z, confidence, prompt} 한 객체로
    묶여 다음 step 에 넘어감. 이전의 `output_key + "_meta"` 분리 키 추방.
    """

    prompt: SlotOr[str] = ""

    def execute(self, ctx: StepContext) -> Detection:
        prompt = ctx.resolve(self.prompt).strip()
        if not prompt:
            raise ValueError(f"GroundedDetect: prompt 비어있음 [{self.label}]")

        logger.info("GroundedDetect '%s'  [%s]", prompt, self.label)
        res = ctx.call_service(
            Service.PERCEPTION_GROUNDED_DETECT,
            {"prompt": prompt},
            timeout=60.0,
        )
        if not res.get("success"):
            raise RuntimeError(
                f"GroundedDetect 실패: {res.get('message')} [{self.label}]"
            )

        data = res.get("data", {})
        position_raw = data.get("position")
        if position_raw is None:
            raise RuntimeError(f"GroundedDetect: position 없음 [{self.label}]")

        detection = Detection(
            position=Position3.from_iter(position_raw),
            height=float(data.get("height", 0.0)),
            base_z=float(data.get("base_z", 0.0)),
            confidence=float(data.get("confidence", 0.0)),
            prompt=prompt,
        )
        logger.info(
            "GroundedDetect 성공: conf=%.2f base=(%.3f, %.3f, %.3f)",
            detection.confidence,
            detection.position.x, detection.position.y, detection.position.z,
        )
        return detection


@dataclass(kw_only=True)
class SearchAndDetect(Step[Detection]):
    """search pose 들 순회하며 GroundedDetect 시도 — 첫 성공 시 break.

    v1 에서는 매크로 유지 (내부 loop). stage 2 의 ForEach + Break primitive
    도입 후 stage 3 에서 분해 (`ForEach(search_poses) { GroundedDetect; ... }`).
    """

    prompt: SlotOr[str] = ""

    def execute(self, ctx: StepContext) -> Detection:
        prompt = ctx.resolve(self.prompt).strip()
        if not prompt:
            raise ValueError(f"SearchAndDetect: prompt 비어있음 [{self.label}]")

        pose_names = list_pose_names("search_")
        if not pose_names:
            raise RuntimeError(
                "SearchAndDetect: search pose 없음 "
                "(robot_poses.yaml 의 search_* 등록 필요)"
            )

        SEARCH_SETTLE = 0.5

        for pose_name in pose_names:
            if ctx.stop_event.is_set():
                raise RuntimeError("SearchAndDetect: stop 요청")

            logger.info("Search pose '%s' 로 이동  [%s]", pose_name, self.label)
            try:
                joints = load_pose(pose_name)
            except KeyError as exc:
                logger.warning("자세 로드 실패: %s", exc)
                continue

            if not ctx.call_motion(Service.MOTION_MOVE_J, {"joints": joints}):
                logger.warning("Search pose '%s' 도달 실패", pose_name)
                continue

            time.sleep(SEARCH_SETTLE)

            logger.info("[%s] '%s' detect 시도", pose_name, prompt)
            res = ctx.call_service(
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
            position_raw = data.get("position")
            if position_raw is None:
                continue

            detection = Detection(
                position=Position3.from_iter(position_raw),
                height=float(data.get("height", 0.0)),
                base_z=float(data.get("base_z", 0.0)),
                confidence=float(data.get("confidence", 0.0)),
                prompt=prompt,
            )
            logger.info(
                "'%s' detect 성공 [%s]: conf=%.2f base=(%.3f, %.3f, %.3f)",
                prompt, pose_name, detection.confidence,
                detection.position.x, detection.position.y, detection.position.z,
            )
            return detection

        raise RuntimeError(
            f"SearchAndDetect: '{prompt}' 모든 search pose 에서 fail [{self.label}]"
        )


# ─── Policy steps — Detection → Position3 derive ──────────────────────


@dataclass(kw_only=True)
class GraspPolicy(Step[Position3]):
    """객체 height 기반 grasp z 결정 — 옆면 그립.

    grasp_z = base_z + height * grasp_ratio (책상 + 일정 비율).
    출력 Position3 는 MoveTCP.target 으로 그대로 들어감.
    """

    target: SlotOr[Detection]
    grasp_ratio: float = 0.5

    def execute(self, ctx: StepContext) -> Position3:
        det = ctx.resolve(self.target)
        if not isinstance(det, Detection):
            raise TypeError(
                f"GraspPolicy.target: Detection 기대, {type(det).__name__} 받음"
            )

        grasp_z = det.base_z + det.height * self.grasp_ratio
        out = Position3(det.position.x, det.position.y, grasp_z)
        logger.info(
            "GraspPolicy base_z=%.3f height=%.3f → grasp_z=%.3f  [%s]",
            det.base_z, det.height, grasp_z, self.label,
        )
        return out


@dataclass(kw_only=True)
class PlacePolicy(Step[Position3]):
    """place 객체 윗면 + drop_clearance — 공중에서 안 떨구게.

    출력 Position3 = (target.position.x, target.position.y, top_z + clearance).
    """

    target: SlotOr[Detection]
    drop_clearance: float = 0.010

    def execute(self, ctx: StepContext) -> Position3:
        det = ctx.resolve(self.target)
        if not isinstance(det, Detection):
            raise TypeError(
                f"PlacePolicy.target: Detection 기대, {type(det).__name__} 받음"
            )

        # det.position.z 는 객체 윗면 z (Grounding DINO 응답 규약).
        top_z = det.position.z
        place_z = top_z + self.drop_clearance
        out = Position3(det.position.x, det.position.y, place_z)
        logger.info(
            "PlacePolicy: top_z=%.3f → place_z=%.3f (clearance=%.3f)  [%s]",
            top_z, place_z, self.drop_clearance, self.label,
        )
        return out
