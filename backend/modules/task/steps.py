"""Step primitives — typed Slot DSL.

이전 [step_types.py](step_types.py) 의 dataclass 정의 + [step_executor.py]
(step_executor.py) 의 핸들러를 한 클래스로 흡수. 각 step 이 자기 `execute(ctx)`
를 보유 → polymorphic dispatch (lego test #3).

입력은 `SlotOr[T]` — literal 값 또는 다른 step 의 `out` 둘 다 받음:
    grasp = GraspPolicy(target=pick.out, grasp_ratio=0.5)
    MoveTCP(target=grasp.out, offset=Position3(0, 0, 0.06))

출력은 Step[T_out] 의 T_out — `step.out` 으로 다음 step 에 넘김. 사이드이펙트만
있는 step (MoveTCP/Gripper/...) 은 `Step[None]` — out 은 사용 안 함.

Control flow (ForEach / BreakIf / Try):
- 별도 `ControlFlowStep` base 두지 않음. 일반 Step 과 동일하게 `execute(ctx)`
  구현하되 내부에서 `ctx.run_child(child)` 호출 → 디버거 게이트 / status 갱신 /
  step result publish 가 자식 step 에도 자동 적용 (lego test #3).
- BT 의 Selector / Try / Decorator 어휘를 차용하되 BT 의 tick/blackboard 는 X.
"""

from __future__ import annotations

import logging
import time
import uuid
from dataclasses import dataclass, field
from typing import Any, Callable

from core.common import GRIPPER_ID, GRIPPER_SETTLE
from core.robot_poses import load_pose
from core.topic_map import Service
from modules.task.schema import Detection, Position3, Slot, SlotOr
from modules.task.step import Step, StepContext

logger = logging.getLogger(__name__)


# ─── Constants — step_executor 에서 그대로 옮겨옴 ──────────────────────


# close 후 gripper Present_Position 이 이 값 미만이면 빈손으로 판정.
GRIPPER_HELD_THRESHOLD = 1900

# 단발 샘플링은 큐브 모서리 임시 catch 를 빈손과 못 구분. 0.3s settle 후 재측정.
GRIPPER_HELD_RECHECK_DELAY = 0.3
GRIPPER_HELD_SLIP_DELTA = 30


# ─── Control flow internals ─────────────────────────────────────────


class _BreakLoop(Exception):
    """BreakIf 가 raise — 가장 가까운 ForEach 가 catch."""


# ─── Wait / MoveJByName — 단순 primitive ─────────────────────────────


@dataclass(kw_only=True)
class Wait(Step[None]):
    duration_sec: float = 0.5

    def execute(self, ctx: StepContext) -> None:
        logger.info("Wait %.2fs  [%s]", self.duration_sec, self.label)
        time.sleep(self.duration_sec)


@dataclass(kw_only=True)
class MoveJByName(Step[None]):
    """robot_poses.yaml 의 자세를 이름으로 MoveJ.

    pose_name: literal str (예: "home") 또는 Slot[str] (ForEach iteration 변수).
    이전 Home step 의 일반화 — Home 은 recipe 함수 home() 로 대체.
    """

    pose_name: SlotOr[str]

    def execute(self, ctx: StepContext) -> None:
        name = ctx.resolve(self.pose_name)
        if not isinstance(name, str):
            raise TypeError(
                f"MoveJByName.pose_name: str 기대, {type(name).__name__}"
            )
        try:
            joints = load_pose(name)
        except KeyError as exc:
            raise RuntimeError(f"MoveJByName: 자세 '{name}' 없음") from exc

        logger.info("MoveJ '%s'  [%s]", name, self.label)
        if not ctx.call_motion(Service.MOTION_MOVE_J, {"joints": joints}):
            raise RuntimeError(f"MoveJ '{name}' 실패")


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
                f"MoveTCP.target: Position3/Detection 기대, "
                f"{type(resolved).__name__} 받음"
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


# ─── Detection ───────────────────────────────────────────────────────


@dataclass(kw_only=True)
class GroundedDetect(Step[Detection]):
    """현재 자세에서 Grounding DINO 로 prompt 객체 1회 검출.

    출력 Detection 은 {position, height, base_z, confidence, prompt} 한 객체로
    묶여 다음 step 에 넘어감 — 이전의 `output_key + "_meta"` suffix 패턴 추방.

    실패 시 raise (task fail). search_and_detect recipe 는 `Try(GroundedDetect)`
    로 감싸서 한 자세 실패가 다음 자세로 continue 되게 함.
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
    """place 객체 윗면 + drop_clearance — 공중에서 안 떨구게."""

    target: SlotOr[Detection]
    drop_clearance: float = 0.010

    def execute(self, ctx: StepContext) -> Position3:
        det = ctx.resolve(self.target)
        if not isinstance(det, Detection):
            raise TypeError(
                f"PlacePolicy.target: Detection 기대, {type(det).__name__} 받음"
            )

        top_z = det.position.z
        place_z = top_z + self.drop_clearance
        out = Position3(det.position.x, det.position.y, place_z)
        logger.info(
            "PlacePolicy: top_z=%.3f → place_z=%.3f (clearance=%.3f)  [%s]",
            top_z, place_z, self.drop_clearance, self.label,
        )
        return out


# ─── Control flow steps — ForEach / BreakIf / Try ────────────────────


def _new_iter_id() -> str:
    return f"iter-{uuid.uuid4().hex[:8]}"


@dataclass(kw_only=True)
class ForEach(Step[None]):
    """items 의 각 element 에 대해 children 시퀀스 실행.

    iter_step_id: factory 가 만든 IterSlot 의 step_id. children 안에서
    `Slot(iter_step_id)` 로 현재 element 참조. 실행 중 ctx.results 에
    {iter_step_id: current_item} 박았다 iteration 끝나면 제거.

    BreakLoop (BreakIf 가 raise) 가 children 안에서 발생하면 즉시 종료.

    BT 의 Sequence + Selector + 일반 iteration 의 통합 형태:
        - mode 분기 없음 — children 안에 BreakIf 두면 Selector 의미
        - BreakIf 없으면 모든 iteration 실행 (일반 iteration)
    """

    items: SlotOr[list]
    children: list[Step] = field(default_factory=list)
    iter_step_id: str = field(default_factory=_new_iter_id)

    def execute(self, ctx: StepContext) -> None:
        items = ctx.resolve(self.items)
        if not isinstance(items, (list, tuple)):
            raise TypeError(
                f"ForEach.items: list/tuple 기대, {type(items).__name__}"
            )

        for item in items:
            ctx.results[self.iter_step_id] = item
            try:
                for child in self.children:
                    ctx.run_child(child)
            except _BreakLoop:
                logger.debug("ForEach break [%s]", self.label)
                return
        # 정상 종료 시 iter slot cleanup — 다음 task 에 누출 방지
        ctx.results.pop(self.iter_step_id, None)

    @classmethod
    def over(
        cls,
        items: SlotOr[list],
        body: Callable[[Slot[Any]], list[Step]],
        *,
        label: str = "",
    ) -> ForEach:
        """factory — iteration variable Slot 만들고 body lambda 에 넘김.

        사용:
            ForEach.over(poses, lambda pose: [
                MoveJByName(pose_name=pose),
                detect,
                BreakIf(condition=detect.out),
            ])
        """
        iter_id = _new_iter_id()
        iter_slot: Slot[Any] = Slot(step_id=iter_id)
        return cls(
            items=items,
            children=body(iter_slot),
            iter_step_id=iter_id,
            label=label,
        )


@dataclass(kw_only=True)
class BreakIf(Step[None]):
    """condition 이 truthy 면 가장 가까운 ForEach 에서 탈출.

    Python truthy 검사 — None / 빈 list / 0 / False 는 false, 나머지 truthy.
    `BreakIf(condition=detect.out)` 처럼 Slot[Detection] 박으면 detect 가
    성공했을 때 (None 아닐 때) break.
    """

    condition: SlotOr[Any]

    def execute(self, ctx: StepContext) -> None:
        if ctx.resolve(self.condition):
            logger.debug("BreakIf truthy → break [%s]", self.label)
            raise _BreakLoop()


@dataclass(kw_only=True)
class Try(Step[Any]):
    """child step 실행 — raise 면 None 반환, 성공이면 child 결과.

    BT 의 fault-tolerant decorator. search_and_detect recipe 에서 한 자세의
    GroundedDetect 실패가 다음 자세로 continue 되게 하는 핵심 원리.

    출력 type 은 일반화상 child 의 출력 type — pyright 가 narrow 어려우니
    `Any`. 사용자는 `Try(MyStep()).out` 이 None 이 될 수 있다고 가정.
    """

    child: Step[Any]

    def execute(self, ctx: StepContext) -> Any:
        try:
            return ctx.run_child(self.child)
        except _BreakLoop:
            # BreakIf 는 그대로 위로 — Try 의 책임 아님
            raise
        except Exception as exc:
            logger.info(
                "Try: child '%s' 실패 (%s: %s) — None 반환",
                self.child.label or self.child.type_name,
                type(exc).__name__, exc,
            )
            return None
