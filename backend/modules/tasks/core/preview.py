"""Task 시나리오 dry-run — 실 하드웨어 없이 전체 step 목록 수집 (미리보기).

imperative 시나리오는 실행해 봐야 경로가 정해지므로(사전 정적 목록 없음 —
[docs/task.md]), "전체 step 목록"은 한 번 traverse 해야 안다. 이 모듈은 시나리오를
**canned 응답 + 기록 link** 로 dry-run 해서 진입하는 @step 들을 순서대로 수집한다:
검출/모션 실제 호출 없음(canned), 게이트/발행 없음, 하드웨어 정착 대기 없음(ctx.dry).

게이트/링크는 @step 의 ContextVar(_ACTIVE_LINK)로만 바인딩되고 task-local 이라,
실제 run 이 도는 중에 preview 를 돌려도 서로 간섭하지 않는다 (TaskRunner 상태 무관).

canned 응답은 도메인마다 다르므로(task 가 부르는 서비스가 다름) responders 를
호출자(task 모듈)가 넘긴다 — 시나리오 구조와 호출 수를 커플하지 않도록 키별
responder 가 매 호출 새 응답을 만든다 (list 소진 아님).
"""

from __future__ import annotations

from typing import Any, Awaitable, Callable, TypeVar

from pydantic import BaseModel

from .contract import TRACE_RUNNING, TraceEntry
from .context import TaskContext
from .spec import TaskRobotSpec
from .step import bind_link, reset_link

TRes = TypeVar("TRes", bound=BaseModel)

# dry-run 은 하드웨어를 안 건드리므로 물리 spec 이 없어도 진행 (gripper raw 값은
# 계산되지만 버려짐) — spec 누락으로 preview 가 실패하지 않게 0 dummy 로 채운다.
_DUMMY_SPEC = TaskRobotSpec(
    gripper_open_raw=0,
    gripper_close_raw=0,
    gripper_index=0,
    gripper_held_threshold_raw=0,
)

# 키 → 요청을 받아 canned 응답을 만드는 함수 (매 호출 새로 — 호출 수 무제한).
Responder = Callable[[BaseModel], BaseModel]


class _RecordingLink:
    """RunLink 구현 — 게이트 없이 @step 진입만 순서대로 기록."""

    def __init__(self) -> None:
        self.entries: list[TraceEntry] = []

    async def enter(self, name: str, depth: int, title: str = "") -> TraceEntry:
        entry = TraceEntry(
            name=name,
            title=title,
            depth=depth,
            status=TRACE_RUNNING,
            started_unix=0.0,
        )
        self.entries.append(entry)
        return entry

    def complete(self, entry: TraceEntry, detail: str = "") -> None:
        pass

    def fail(self, entry: TraceEntry, detail: str) -> None:
        pass


class _PreviewRuntime:
    """canned 응답 runtime (ModuleRuntime Protocol duck-type). publish 는 no-op —
    발행(마커 등)은 미리보기에 무의미. call 은 키별 responder 위임 (무제한 호출)."""

    def __init__(self, responders: dict[str, Responder]) -> None:
        self._responders = {str(k): v for k, v in responders.items()}

    def publish(self, wire_key: str, event: BaseModel) -> None:
        pass

    async def call(
        self,
        key: str,
        req: BaseModel,
        res_cls: type[TRes],
        *,
        robot_id: str | None = None,
        timeout: float | None = None,
    ) -> TRes:
        responder = self._responders.get(str(key))
        if responder is None:
            raise AssertionError(
                f"preview responder 미등록: {key} — 시나리오가 부르는 모든 서비스에 "
                "canned 응답 필요"
            )
        return responder(req)  # type: ignore[return-value]


async def collect_steps(
    scenario: Callable[..., Awaitable[None]],
    *,
    robot_ids: list[str],
    specs: dict[str, TaskRobotSpec],
    responders: dict[str, Responder],
    **kwargs: Any,
) -> list[TraceEntry]:
    """시나리오를 dry-run 해서 진입 step 목록을 순서대로 반환 (모션 0).

    specs 에 없는 robot 은 dummy 로 채운다 (dry-run 은 물리값을 안 씀).
    """
    full_specs = {r: specs.get(r, _DUMMY_SPEC) for r in robot_ids}
    link = _RecordingLink()
    ctx = TaskContext(_PreviewRuntime(responders), full_specs)
    ctx.bind_run(link, robot_ids)  # 참여 명부 설정 → ctx.call robot_id 검증 통과
    ctx.dry = True
    token = bind_link(link)
    try:
        await scenario(ctx, **kwargs)
    finally:
        reset_link(token)
    return link.entries
