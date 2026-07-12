"""TaskContext / RobotHandle — 시나리오의 도메인 접근 표면.

책임 분리 (2026-07-12 수렴): TaskRunner 는 실행 생명주기만, **도메인 접근은 전부
여기** — robot spec 보유, primitive (wire 키/timeout/실패→typed 예외 내장), escape
hatch, 그리고 "어느 robot 에 모션을 보냈는지" 추적 → 중단/실패 시 on_abort() 가
실제로 움직인 robot 에만 Motion.STOP.

시나리오가 보는 표면:
    so101 = ctx.robot("so101_6dof_0")      # RobotHandle — robot 에 속한 동작 전부
    await so101.detect_oriented("white cube")
    await so101.move_l(pos, quat, label="descend")
    await so101.call(키, req, res)          # escape hatch (robot-scoped 키 자동 주입)
    await ctx.wait(0.5)                     # robot 무관 task 수준
    ctx.record("grasp", value)              # UI 노출 (STEP_RESULT)
    await ctx.call(키, req, res)            # escape hatch (robot 무관)

primitive 는 진입 시 runner 게이트를 탄다 (pause/step/breakpoint) — 시나리오 작성자
에겐 label 인자 하나뿐 (그마저 선택 — 생략 시 "kind#n" 자동).
"""

from __future__ import annotations

import asyncio
import logging
from typing import Any, Literal, Protocol, TypeVar

from pydantic import BaseModel

from framework.runtime.api import ModuleRuntime
from modules.detector.contract import (
    Detector,
    DetectOrientedResponse,
    DetectRequest,
    OrientedDetection,
)
from modules.motion.contract import (
    Motion,
    MoveJPoseRequest,
    MoveJResponse,
    MoveLRequest,
    MoveLResponse,
    SelectReachableRequest,
    SelectReachableResponse,
    StopRequest,
    StopResponse,
    TcpPose,
)
from modules.motor.contract import Motor, SetGripperRequest, SetGripperResponse

from .contract import TraceEntry
from .errors import GripperFailed, MotionRejected, NoReachableGrasp, TaskError
from .spec import TaskRobotSpec

logger = logging.getLogger(__name__)

TRes = TypeVar("TRes", bound=BaseModel)

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]

# SET_GRIPPER 는 즉시 반환 — 조 이동 대기 (STS 90°/s profile, 2026-07-09 실측).
_GRIPPER_SETTLE_S = 1.2

# primitive timeout — 정책은 primitive 가 소유 (호출부 고민 X).
_DETECT_TIMEOUT_S = 30.0  # GDINO 첫 추론이 느릴 수 있음
_MOVE_TIMEOUT_S = 60.0
_SELECT_TIMEOUT_S = 60.0
_GRIPPER_TIMEOUT_S = 10.0
_STOP_TIMEOUT_S = 5.0


class RunLink(Protocol):
    """runner 가 구현해 bind_run 으로 주입하는 좁은 인터페이스 — 게이트/관측 훅."""

    async def enter(self, kind: str, label: str) -> TraceEntry:
        ...

    def complete(self, entry: TraceEntry, detail: str = "") -> None:
        ...

    def fail(self, entry: TraceEntry, detail: str) -> None:
        ...

    def emit_result(self, label: str, type_name: str, value: Any) -> None:
        ...


def dump_value(value: Any) -> tuple[str, Any]:
    """STEP_RESULT 용 (type, value) 직렬화 — BaseModel/list/None/scalar."""
    if value is None:
        return "None", None
    if isinstance(value, BaseModel):
        return type(value).__name__, value.model_dump()
    if isinstance(value, list):
        return "list", [
            v.model_dump() if isinstance(v, BaseModel) else v for v in value
        ]
    return type(value).__name__, value


class TaskContext:
    def __init__(
        self, runtime: ModuleRuntime, robots: dict[str, TaskRobotSpec] | None = None
    ) -> None:
        self.runtime = runtime  # 최후 escape hatch — 공개
        self._robots = robots or {}
        self._link: RunLink | None = None
        self._allowed: set[str] | None = None
        self._handles: dict[str, RobotHandle] = {}
        self._moved: set[str] = set()  # 모션을 보낸 robot — on_abort 정지 대상

    # ─── runner 가 쓰는 프로토콜 (시나리오 표면 아님) ────────────────

    def bind_run(self, link: RunLink, robot_ids: list[str]) -> None:
        """run 시작 시 runner 가 게이트/관측 훅 주입 + 참여 robot 제한 설정."""
        self._link = link
        self._allowed = set(robot_ids) if robot_ids else None
        self._moved.clear()

    async def on_abort(self) -> None:
        """중단/실패 시 runner 가 호출 — 모션 보낸 robot 에만 Motion.STOP (best-effort)."""
        for robot_id in sorted(self._moved):
            try:
                await self.runtime.call(
                    Motion.Service.STOP,
                    StopRequest(),
                    StopResponse,
                    robot_id=robot_id,
                    timeout=_STOP_TIMEOUT_S,
                )
                logger.info("on_abort: Motion.STOP → %s", robot_id)
            except Exception:
                logger.exception("on_abort: Motion.STOP 실패 (robot=%s)", robot_id)

    # ─── 시나리오 표면 ───────────────────────────────────────────────

    def robot(self, robot_id: str) -> RobotHandle:
        """robot 에 바인딩된 동작 표면. 참여 선언(robot_ids)에 없는 robot 은 즉시 에러."""
        if self._allowed is not None and robot_id not in self._allowed:
            raise TaskError(
                f"robot '{robot_id}' 는 이 task 의 참여 robot 이 아님 "
                f"(선언: {sorted(self._allowed)})"
            )
        handle = self._handles.get(robot_id)
        if handle is None:
            handle = RobotHandle(self, robot_id, self._robots.get(robot_id))
            self._handles[robot_id] = handle
        return handle

    async def wait(self, sec: float, *, label: str = "") -> None:
        """settle 등 대기 — 게이트를 타는 primitive (pause/step 대상)."""
        entry = await self._require_link().enter("wait", label)
        try:
            await asyncio.sleep(sec)
        except BaseException as exc:
            self._require_link().fail(entry, _detail_of(exc))
            raise
        self._require_link().complete(entry)

    def record(self, label: str, value: BaseModel | list | None) -> None:
        """시나리오 중간값을 UI 로 노출 (STEP_RESULT — 씬 오버레이/패널)."""
        type_name, dumped = dump_value(value)
        self._require_link().emit_result(label, type_name, dumped)

    async def call(
        self,
        key: str,
        req: BaseModel,
        res_cls: type[TRes],
        *,
        timeout: float = 5.0,
    ) -> TRes:
        """escape hatch (robot 무관 서비스 — llm 등). 게이트/trace 안 탐."""
        return await self.runtime.call(key, req, res_cls, timeout=timeout)

    # ─── internal ────────────────────────────────────────────────────

    def _require_link(self) -> RunLink:
        if self._link is None:
            raise RuntimeError(
                "TaskContext 가 run 에 바인딩 안 됨 — TaskRunner.start 를 거쳐야 함"
            )
        return self._link

    def _mark_moved(self, robot_id: str) -> None:
        self._moved.add(robot_id)


class RobotHandle:
    """한 robot 에 바인딩된 primitive 묶음 — 시나리오 본문에서 "누가 움직이는지"가
    변수 이름으로 읽히게 하는 표면 (`so101 = ctx.robot(...)`)."""

    def __init__(
        self, ctx: TaskContext, robot_id: str, spec: TaskRobotSpec | None
    ) -> None:
        self._ctx = ctx
        self.robot_id = robot_id
        self._spec = spec

    # ─── perception / planning ──────────────────────────────────────

    async def detect_oriented(
        self, prompt: str, *, top_k: int = 5, label: str = ""
    ) -> list[OrientedDetection]:
        """prompt → base frame OBB 후보 Top-K. 후보 판단(prior/선택)은 시나리오 몫.
        결과는 STEP_RESULT 로 자동 노출 (씬 오버레이)."""
        entry = await self._enter("detect_oriented", label)
        try:
            res = await self._ctx.runtime.call(
                Detector.Service.DETECT_ORIENTED,
                DetectRequest(robot_id=self.robot_id, prompt=prompt, top_k=top_k),
                DetectOrientedResponse,
                timeout=_DETECT_TIMEOUT_S,
            )
        except BaseException as exc:
            self._fail(entry, exc)
            raise
        # detector 의 실패 사유(캘 없음/depth 부족 등)를 trace 에 보존 — 후보 0개가
        # "안 보임"인지 "시스템 문제"인지 UI 에서 구분 가능해야 (침묵 금지).
        detail = f"{len(res.candidates)}개 후보"
        if res.message:
            detail += f" — {res.message}"
        self._complete(entry, detail)
        type_name, dumped = dump_value(list(res.candidates))
        self._link().emit_result(entry.label, type_name, dumped)
        return list(res.candidates)

    async def select_reachable(
        self, groups: list[list[TcpPose]], *, label: str = ""
    ) -> int:
        """후보 pose 그룹 배치 IK 판정 (모션 0) — 첫 가용 그룹 index.
        전멸이면 NoReachableGrasp raise (index -1 반환 없음 — 침묵 금지)."""
        entry = await self._enter("select_reachable", label)
        try:
            res = await self._ctx.runtime.call(
                Motion.Service.SELECT_REACHABLE,
                SelectReachableRequest(groups=groups),
                SelectReachableResponse,
                robot_id=self.robot_id,
                timeout=_SELECT_TIMEOUT_S,
            )
            if res.index < 0:
                raise NoReachableGrasp(res.message)
        except BaseException as exc:
            self._fail(entry, exc)
            raise
        self._complete(entry, f"group {res.index}")
        return res.index

    # ─── motion ──────────────────────────────────────────────────────

    async def move_j_pose(
        self, position: Vec3, quaternion: Quat | None = None, *, label: str = ""
    ) -> None:
        """TCP pose → IK → 관절 공간 이동 (완료까지 await). 거부 = MotionRejected."""
        entry = await self._enter("move_j_pose", label)
        self._ctx._mark_moved(self.robot_id)
        try:
            res = await self._ctx.runtime.call(
                Motion.Service.MOVE_J_POSE,
                MoveJPoseRequest(
                    target_position=position, target_quaternion=quaternion
                ),
                MoveJResponse,
                robot_id=self.robot_id,
                timeout=_MOVE_TIMEOUT_S,
            )
            if not res.accepted:
                raise MotionRejected("move_j_pose", res.message)
        except BaseException as exc:
            self._fail(entry, exc)
            raise
        self._complete(entry)

    async def move_l(
        self, position: Vec3, quaternion: Quat | None = None, *, label: str = ""
    ) -> None:
        """TCP 직선 이동 — quaternion 지정 시 경로 전 구간 자세 고정 (완료까지 await)."""
        entry = await self._enter("move_l", label)
        self._ctx._mark_moved(self.robot_id)
        try:
            res = await self._ctx.runtime.call(
                Motion.Service.MOVE_L,
                MoveLRequest(target_position=position, target_quaternion=quaternion),
                MoveLResponse,
                robot_id=self.robot_id,
                timeout=_MOVE_TIMEOUT_S,
            )
            if not res.accepted:
                raise MotionRejected("move_l", res.message)
        except BaseException as exc:
            self._fail(entry, exc)
            raise
        self._complete(entry)

    async def gripper(
        self, action: Literal["open", "close"], *, label: str = ""
    ) -> None:
        """open/close — raw 값은 motors.yaml 투영 spec (추측 X), settle 대기 내장."""
        if self._spec is None:
            raise TaskError(
                f"robot '{self.robot_id}' 의 gripper spec 없음 (motors.yaml 투영 누락)"
            )
        entry = await self._enter("gripper", label or f"gripper_{action}")
        raw = (
            self._spec.gripper_open_raw
            if action == "open"
            else self._spec.gripper_close_raw
        )
        try:
            res = await self._ctx.runtime.call(
                Motor.Service.SET_GRIPPER,
                SetGripperRequest(position_raw=raw),
                SetGripperResponse,
                robot_id=self.robot_id,
                timeout=_GRIPPER_TIMEOUT_S,
            )
            if not res.ok:
                raise GripperFailed(action)
            await asyncio.sleep(_GRIPPER_SETTLE_S)
        except BaseException as exc:
            self._fail(entry, exc)
            raise
        self._complete(entry, action)

    # ─── escape hatch ────────────────────────────────────────────────

    async def call(
        self,
        key: str,
        req: BaseModel,
        res_cls: type[TRes],
        *,
        timeout: float = 5.0,
    ) -> TRes:
        """robot-scoped escape hatch — `{robot_id}` 키에 이 robot 자동 주입.
        primitive 로 아직 승격 안 된 서비스용 (반복 관찰되면 메서드로 승격)."""
        return await self._ctx.runtime.call(
            key, req, res_cls, robot_id=self.robot_id, timeout=timeout
        )

    # ─── internal ────────────────────────────────────────────────────

    def _link(self) -> RunLink:
        return self._ctx._require_link()

    async def _enter(self, kind: str, label: str) -> TraceEntry:
        return await self._link().enter(kind, label)

    def _complete(self, entry: TraceEntry, detail: str = "") -> None:
        self._link().complete(entry, detail)

    def _fail(self, entry: TraceEntry, exc: BaseException) -> None:
        self._link().fail(entry, _detail_of(exc))


def _detail_of(exc: BaseException) -> str:
    if isinstance(exc, asyncio.CancelledError):
        return "중단됨"
    return str(exc) or type(exc).__name__


class TaskContextFactory:
    """모듈이 보유 — run 마다 새 TaskContext 생성 (moved/handle 상태가 run 단위)."""

    def __init__(
        self, runtime: ModuleRuntime, robots: dict[str, TaskRobotSpec] | None = None
    ) -> None:
        self._runtime = runtime
        self._robots = robots or {}

    def create(self) -> TaskContext:
        return TaskContext(self._runtime, self._robots)
