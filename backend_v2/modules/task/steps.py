"""Task step primitives — Day-1 primitive (§17.1) 만.

옛 backend/modules/task/steps.py 재구성. 각 step 이 `async def execute(ctx)` 보유 →
runner 가 polymorphic await (match/case 없음). 입력 SlotOr[T] (literal / 이전 step.out),
출력 Step[T_out].

포함 = Day-1 (표준 산업 로봇 공통 출하, §17.1 표): MoveJ(waypoint=<ref>) [D8] /
MoveTCP→MOVE_L / Gripper + VerifyGrasp. 이름/매핑 §17.5.

의도적 미포함 (재발 방지 — 문서 대조):
  - Orchestration (ForEach/BreakIf/Try = Loop/Retry) — §17.1 "rule of three (task 2개가
    요구할 때)". task #1(PnP) 하나뿐 → defer. PnP 의 Waypoint Group 순회는 §17.1③
    "거의 DSL 없이 평범한 async 함수" 로 (generic loop primitive X). BreakIf 는 §17.5
    recipe 에서 명시적 제거.
  - 검출 의존 step (GroundedDetect/GraspPolicy/PlacePolicy/SelectTarget) — Detection
    Top-K + 기하 prior 확장(§17.5) 후 milestone ④.
"""

from __future__ import annotations

import asyncio
import logging
from dataclasses import dataclass, field
from typing import Literal

from modules.motion.contract import (
    Motion,
    MoveJRequest,
    MoveJResponse,
    MoveLRequest,
    MoveLResponse,
)
from modules.motor.contract import Motor, SetGripperRequest, SetGripperResponse
from modules.waypoint.contract import (
    ListWaypointsRequest,
    ListWaypointsResponse,
    Waypoint,
)

from .schema import Position3, SlotOr
from .step import Step, StepContext

logger = logging.getLogger(__name__)


# ─── 단순 primitive ──────────────────────────────────────────────────


@dataclass(kw_only=True)
class Wait(Step[None]):
    """지정 시간 대기 (settle 등). async — event loop 안 막음."""

    duration_sec: float = 0.5

    async def execute(self, ctx: StepContext) -> None:
        logger.info("Wait %.2fs  [%s]", self.duration_sec, self.label)
        await asyncio.sleep(self.duration_sec)


@dataclass(kw_only=True)
class NoOp(Step[None]):
    """아무것도 안 함 — 도메인 step 0개 trivial task (runner/디버거 e2e, §17.4)."""

    async def execute(self, ctx: StepContext) -> None:
        return None


# ─── Move — waypoint / base-frame ────────────────────────────────────


@dataclass(kw_only=True)
class MoveJ(Step[None]):
    """waypoint 이름 → 그 joint 자세로 MoveJ (§17.2 D8 "MoveJ(waypoint=<ref>)").

    식별(이름) 은 DSL, resolve 는 runtime — Waypoint 는 robot-agnostic 이라 LIST(
    robot_id) 로 이름 조회 후 joint_values(rad) 로 Motion.MOVE_J. await = 완료(§17.3).
    """

    waypoint: SlotOr[str]

    async def execute(self, ctx: StepContext) -> None:
        name = ctx.resolve(self.waypoint)
        if not isinstance(name, str):
            raise TypeError(f"MoveJ.waypoint: str 기대, {type(name).__name__}")
        # waypoint 는 robot-agnostic (robot_id=req 필드, §2.7) → runtime 직접 호출.
        wps = await ctx.runtime.call(
            Waypoint.Service.LIST,
            ListWaypointsRequest(robot_id=ctx.robot_id),
            ListWaypointsResponse,
        )
        match = next((w for w in wps.waypoints if w.name == name), None)
        if match is None:
            raise RuntimeError(
                f"MoveJ: waypoint '{name}' 없음 (robot={ctx.robot_id})"
            )
        logger.info("MoveJ waypoint '%s'  [%s]", name, self.label)
        res = await ctx.call(
            Motion.Service.MOVE_J,
            MoveJRequest(target_joints=match.joint_values),
            MoveJResponse,
            timeout=60.0,
        )
        if not res.accepted:
            raise RuntimeError(f"MoveJ '{name}' 거부: {res.message}")


@dataclass(kw_only=True)
class MoveTCP(Step[None]):
    """base frame (x,y,z) 로 MoveL (§17.5, position-only v1). target = Position3
    (Detection→Position3 변환은 GraspPolicy/PlacePolicy 담당 → 계층 분리)."""

    target: SlotOr[Position3]
    offset: Position3 = field(default_factory=lambda: Position3(x=0.0, y=0.0, z=0.0))

    async def execute(self, ctx: StepContext) -> None:
        base = ctx.resolve(self.target)
        if not isinstance(base, Position3):
            raise TypeError(f"MoveTCP.target: Position3 기대, {type(base).__name__}")
        pos = base + self.offset
        logger.info(
            "MoveL → %.3f %.3f %.3f  [%s]", pos.x, pos.y, pos.z, self.label
        )
        res = await ctx.call(
            Motion.Service.MOVE_L,
            MoveLRequest(target_position=(pos.x, pos.y, pos.z)),
            MoveLResponse,
            timeout=60.0,
        )
        if not res.accepted:
            raise RuntimeError(f"MoveL 거부: {res.message} [{self.label}]")


# ─── Gripper ─────────────────────────────────────────────────────────


@dataclass(kw_only=True)
class Gripper(Step[None]):
    """open/close — SET_GRIPPER(position_raw). raw = TaskRobotSpec (motors.yaml, 추측 X).

    v2 SET_GRIPPER 은 position 기반 (v1 의 force/current 아님) — STS position servo 가
    물체에 막히면 그 위치서 토크 유지 = 파지. 검증은 별도 VerifyGrasp (lego 분리)."""

    action: Literal["open", "close"] = "open"

    async def execute(self, ctx: StepContext) -> None:
        spec = ctx.require_spec()
        pos = (
            spec.gripper_open_raw
            if self.action == "open"
            else spec.gripper_close_raw
        )
        logger.info("Gripper %s → raw %d  [%s]", self.action, pos, self.label)
        res = await ctx.call(
            Motor.Service.SET_GRIPPER,
            SetGripperRequest(position_raw=pos),
            SetGripperResponse,
        )
        if not res.ok:
            raise RuntimeError(f"Gripper {self.action} 실패 [{self.label}]")


@dataclass(kw_only=True)
class VerifyGrasp(Step[None]):
    """gripper 현재 raw 로 잡힘 검증 — held_threshold 미만이면 빈손 (raise).

    잡힌 물체가 fingers 를 fully-close 위로 벌림 → raw > threshold (so101: open=high).
    threshold = TaskRobotSpec (하드웨어 tuning, §17.5). gripper raw = Motor.RAW_STATE
    캐시 (module subscribe)."""

    async def execute(self, ctx: StepContext) -> None:
        spec = ctx.require_spec()
        raw = ctx.gripper_raw()
        if raw is None:
            raise RuntimeError(f"VerifyGrasp: gripper 상태 미수신 [{self.label}]")
        if raw < spec.gripper_held_threshold_raw:
            raise RuntimeError(
                f"VerifyGrasp: 빈손 (raw {raw} < threshold "
                f"{spec.gripper_held_threshold_raw}) [{self.label}]"
            )
        logger.info("VerifyGrasp OK: raw=%d  [%s]", raw, self.label)
