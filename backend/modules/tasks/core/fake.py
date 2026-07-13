"""FakeContext — 시나리오/step 로직을 하드웨어/wire 없이 검증하는 공식 테스트 표면.

@step 개편 (2026-07-13) 후 fake 의 단위 = **서비스 호출** (primitive fake 소멸).
step 함수는 run 밖에선 게이트 없이 본문만 실행되므로, 시나리오를 FakeContext 로
그냥 await 하면 된다 — 서비스 응답만 wire 키별로 스크립트한다.

    ctx = FakeContext(
        robots=["so101_6dof_0"],
        specs={"so101_6dof_0": spec},
        service_script={
            Detector.Service.DETECT_ORIENTED: [DetectOrientedResponse(...)],
            Motion.Service.RESOLVE_REACHABLE: [ResolveReachableResponse(index=0)],
            Motion.Service.MOVE_J_POSE: [MoveJResponse()] * 2,
            # 실패 주입 = Exception 인스턴스 (RemoteError("MotionRejected", ...) 등)
        },
    )
    await scenario(ctx, pick_object="white cube")
    assert ctx.keys() == [...]                 # 서비스 호출 순서
    assert ctx.calls(Motion.Service.MOVE_L)[0]["req"].target_position == ...

Motion.STOP 은 스크립트 없어도 성공 — on_abort 안전 경로가 테스트마다 잡음이
되지 않게. 그 외 미등록 키 호출 = AssertionError (명확히 터뜨림).
"""

from __future__ import annotations

from typing import Any, TypeVar

from pydantic import BaseModel

from modules.motion.contract import Motion, StopResponse

from .context import TaskContext
from .spec import TaskRobotSpec

TRes = TypeVar("TRes", bound=BaseModel)


class ScriptedRuntime:
    """wire 키별 응답 스크립트 + 호출/발행 기록 (ModuleRuntime 표면)."""

    def __init__(self, script: dict[str, list[Any]] | None = None) -> None:
        self.script: dict[str, list[Any]] = {
            str(k): list(v) for k, v in (script or {}).items()
        }
        self.call_log: list[dict[str, Any]] = []  # {key, req, robot_id, timeout}
        self.published: list[tuple[str, BaseModel]] = []

    def publish(self, wire_key: str, event: BaseModel) -> None:
        self.published.append((str(wire_key), event))

    async def call(
        self,
        key: str,
        req: BaseModel,
        res_cls: type[TRes],
        *,
        robot_id: str | None = None,
        timeout: float | None = None,
    ) -> TRes:
        key_str = str(key)
        self.call_log.append(
            {"key": key_str, "req": req, "robot_id": robot_id, "timeout": timeout}
        )
        seq = self.script.get(key_str)
        if seq:
            r = seq.pop(0)
            if isinstance(r, Exception):
                raise r
            return r
        if key_str == str(Motion.Service.STOP):
            return StopResponse(ok=True)  # type: ignore[return-value]
        raise AssertionError(
            f"service_script 미등록/소진: {key_str} — 테스트 스크립트 확인"
        )


class FakeContext(TaskContext):
    """TaskContext + ScriptedRuntime — on_abort 관측 추가."""

    def __init__(
        self,
        *,
        robots: list[str] | None = None,
        specs: dict[str, TaskRobotSpec] | None = None,
        service_script: dict[str, list[Any]] | None = None,
    ) -> None:
        super().__init__(ScriptedRuntime(service_script), specs or {})
        self._allowed = set(robots) if robots else None
        self._robot_ids = list(robots or [])
        self.aborted = False

    # ─── 검증 helper ─────────────────────────────────────────────────

    @property
    def wire(self) -> ScriptedRuntime:
        return self.runtime  # type: ignore[return-value]

    def keys(self) -> list[str]:
        """서비스 호출 wire 키 순서 (STOP 포함) — 시나리오 경로 검증용."""
        return [c["key"] for c in self.wire.call_log]

    def calls(self, key: str) -> list[dict[str, Any]]:
        key_str = str(key)
        return [c for c in self.wire.call_log if c["key"] == key_str]

    # ─── TaskContext override (관측) ─────────────────────────────────

    async def on_abort(self) -> None:
        self.aborted = True
        await super().on_abort()  # STOP 경로도 실물대로 (call_log 에 남음)
