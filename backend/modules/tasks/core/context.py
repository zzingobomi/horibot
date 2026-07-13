"""TaskContext — 이번 run 에서 시나리오가 세계에 닿는 접점 (2026-07-13 확정).

호출 표면은 **ctx.call 하나** (RobotHandle 삭제 — 표면이 둘이면 "어느 쪽으로
부르지?"라는 질문이 생기고 그 질문이 오용을 낳는다). robot-scoped / agnostic 의
구분은 계약에만 산다:

    # robot-scoped (키에 {robot_id} — 그 robot 에게 명령): robot_id= 인자
    await ctx.call(Motion.Service.MOVE_L, MoveLRequest(...), MoveLResponse,
                   robot_id=so101)
    # robot-agnostic (모듈에 조회 — robot 은 파라미터): req 필드로 (§2.7)
    await ctx.call(Detector.Service.DETECT_ORIENTED,
                   DetectRequest(robot_id=so101, prompt=...), DetectOrientedResponse)

내용물 3개 — 전부 "이번 run 의":
  - call: 서비스 호출. robot_id= 를 주면 **참여 명부 검증** (선언 밖 robot 명령
    즉시 에러 — on_abort STOP 커버리지 보장) 후 robot-scoped 키에 주입.
  - spec(robot_id): motors.yaml 투영 물리값 (gripper raw 등 — 물리값 추측 금지).
  - on_abort: 참여 robot 전원 Motion.STOP — 안전 의무 (core 의 유일한 도메인 지식).

timeout 은 넘기지 않는 게 기본 — 서비스의 contract.py 선언 기본값을 runtime 이
쓴다 (framework.contract.service.declare_service_timeouts).
"""

from __future__ import annotations

import logging
from typing import TypeVar

from pydantic import BaseModel

from framework.runtime.api import ModuleRuntime
from modules.motion.contract import Motion, StopRequest, StopResponse

from .errors import TaskError
from .spec import TaskRobotSpec
from .step import RunLink

logger = logging.getLogger(__name__)

TRes = TypeVar("TRes", bound=BaseModel)

_STOP_TIMEOUT_S = 5.0  # 안전 경로 — 짧게 (계류하면 다음 robot 정지가 밀림)


class TaskContext:
    def __init__(
        self, runtime: ModuleRuntime, robots: dict[str, TaskRobotSpec] | None = None
    ) -> None:
        self.runtime = runtime  # 날것 escape hatch — 검증 없음, 최후 수단
        self._specs = robots or {}
        self._link: RunLink | None = None
        self._allowed: set[str] | None = None
        self._robot_ids: list[str] = []  # 참여 robot — on_abort STOP 대상

    # ─── runner 가 쓰는 프로토콜 (시나리오 표면 아님) ────────────────

    def bind_run(self, link: RunLink, robot_ids: list[str]) -> None:
        """run 시작 시 runner 가 관측 훅 주입 + 참여 robot 설정."""
        self._link = link
        self._allowed = set(robot_ids) if robot_ids else None
        self._robot_ids = list(robot_ids)

    async def on_abort(self) -> None:
        """중단/실패 시 runner 가 호출 — 참여 robot 전원 Motion.STOP (best-effort).

        안 움직인 robot 에 STOP = 무해 (모션 없으면 no-op) — moved 추적보다
        보수적으로 안전한 쪽 (2026-07-13).
        """
        for robot_id in self._robot_ids:
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

    async def call(
        self,
        key: str,
        req: BaseModel,
        res_cls: type[TRes],
        *,
        robot_id: str | None = None,
        timeout: float | None = None,
    ) -> TRes:
        """서비스 호출 — 단일 표면.

        robot_id= 는 robot-scoped 키(`{robot_id}` 포함) 전용: 참여 명부 검증 후
        키에 주입. agnostic 키에 robot_id= 를 주면 fail-fast — 라우팅되는 척
        읽히는 거짓 코드 방지 (대상 robot 은 req 필드로, §2.7). timeout=None →
        contract 선언 기본값.
        """
        key_str = str(key)
        scoped = "{robot_id}" in key_str
        if robot_id is not None:
            if not scoped:
                raise TaskError(
                    f"{key_str!r} 는 robot-agnostic 서비스 — robot_id= 인자가 아니라 "
                    "req 필드로 대상 robot 을 전달 (§2.7)"
                )
            if self._allowed is not None and robot_id not in self._allowed:
                raise TaskError(
                    f"robot '{robot_id}' 는 이 task 의 참여 robot 이 아님 "
                    f"(선언: {sorted(self._allowed)}) — 선언 밖 robot 은 on_abort "
                    "STOP 이 못 덮는다"
                )
        return await self.runtime.call(
            key_str, req, res_cls, robot_id=robot_id, timeout=timeout
        )

    def spec(self, robot_id: str) -> TaskRobotSpec:
        """robot 의 물리 spec (motors.yaml 투영) — 없으면 읽을 수 있는 사유로 raise."""
        spec = self._specs.get(robot_id)
        if spec is None:
            raise TaskError(
                f"robot '{robot_id}' 의 물리 spec 없음 (motors.yaml 투영 누락)"
            )
        return spec


class TaskContextFactory:
    """모듈이 보유 — run 마다 새 TaskContext 생성 (참여 상태가 run 단위)."""

    def __init__(
        self, runtime: ModuleRuntime, robots: dict[str, TaskRobotSpec] | None = None
    ) -> None:
        self._runtime = runtime
        self._robots = robots or {}

    def create(self) -> TaskContext:
        return TaskContext(self._runtime, self._robots)
