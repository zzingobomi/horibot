"""@step — 시나리오 실행 단위 선언 (게이트/trace 경계). 2026-07-13 확정 설계.

step = **개발자가 지정한 함수 단위** (프레임워크가 primitive 를 강제하지 않음).
게이트(pause/step_once/run_to/breakpoint)와 trace 는 step 진입점에서만 탄다.

중첩 허용 — "함수 호출은 함수 호출이다": step 안에서 step 을 부르면 trace 에
depth 로 찍히고, 게이트는 모든 진입점에서 탄다 (step_once = step-into 의미론.
step-over 버튼은 없음 — run_to 가 형제 건너뛰기를 커버). runner 는 트리를
관리하지 않고 depth 카운터(ContextVar — asyncio task 상속)만 본다.

run 밖에서 부르면 (단위테스트 등) 게이트/trace 없이 본문만 실행 —
step 함수는 평범한 async 함수로도 테스트 가능하다.

실패 = raise 그대로 전파. 안쪽 step 이 실패하면 그 entry 가 failed 로 찍히고,
예외가 바깥 step 을 타고 오르며 경로(root→leaf)가 전부 failed 로 기록된다.

    @step
    async def descend(ctx: TaskContext, robot_id: str, c: GraspCandidate) -> None:
        await ctx.call(Motion.Service.MOVE_L,
                       MoveLRequest(target_position=c.grasp,
                                    target_quaternion=c.quat),
                       MoveLResponse, robot_id=robot_id)

@step(title="집기") — UI 표시 이름 (한글 등). name(식별자 — breakpoint/run_to
대상, 함수 이름) 과 분리: 표시 문구는 바뀌어도 식별자는 안정.
"""

from __future__ import annotations

import asyncio
import functools
from contextvars import ContextVar, Token
from typing import Any, Awaitable, Callable, Protocol, TypeVar, overload

from .contract import TraceEntry

_T = TypeVar("_T")


class RunLink(Protocol):
    """runner 가 구현 — step 게이트/관측 훅 (시나리오 표면 아님)."""

    async def enter(self, name: str, depth: int, title: str = "") -> TraceEntry: ...

    def complete(self, entry: TraceEntry, detail: str = "") -> None: ...

    def fail(self, entry: TraceEntry, detail: str) -> None: ...


# run 중 활성 링크/깊이 — runner._supervise 가 bind, step wrapper 가 소비.
# ContextVar 라 asyncio task 경계를 따라 상속된다 (미래 병렬 step 안전 토대).
_ACTIVE_LINK: ContextVar[RunLink | None] = ContextVar("task_step_link", default=None)
_DEPTH: ContextVar[int] = ContextVar("task_step_depth", default=0)


def bind_link(link: RunLink) -> Token[RunLink | None]:
    """runner 가 시나리오 실행 직전 호출 (감독 task 안에서) — reset 용 token 반환."""
    return _ACTIVE_LINK.set(link)


def reset_link(token: Token[RunLink | None]) -> None:
    _ACTIVE_LINK.reset(token)


def active_link() -> RunLink | None:
    """run 에 바인딩된 링크 (없으면 None) — ctx.record 등이 사용."""
    return _ACTIVE_LINK.get()


def _detail_of(exc: BaseException) -> str:
    if isinstance(exc, asyncio.CancelledError):
        return "중단됨"
    return str(exc) or type(exc).__name__


@overload
def step(fn: Callable[..., Awaitable[_T]]) -> Callable[..., Awaitable[_T]]: ...


@overload
def step(
    *, title: str = ...
) -> Callable[[Callable[..., Awaitable[_T]]], Callable[..., Awaitable[_T]]]: ...


def step(
    fn: Callable[..., Awaitable[_T]] | None = None,
    *,
    title: str | None = None,
) -> Any:
    """async 함수를 step 으로 선언.

    name(식별자 — breakpoint/run_to 대상) = **함수 이름**. 코드가 곧 안정 식별자라
    별도 override 파라미터를 두지 않는다 (함수 이름이 이미 그 일을 함).
    title = UI 표시 이름 (선택 — "집기" 같은 한글 문구. 식별자와 분리).
    @step / @step(title="집기") 지원.
    """

    def decorate(f: Callable[..., Awaitable[_T]]) -> Callable[..., Awaitable[_T]]:
        if not asyncio.iscoroutinefunction(f):
            raise TypeError(f"@step '{f.__name__}': async 함수여야 함")
        step_name = f.__name__
        step_title = title or ""

        @functools.wraps(f)
        async def wrapper(*args: Any, **kwargs: Any) -> _T:
            link = _ACTIVE_LINK.get()
            if link is None:
                return await f(*args, **kwargs)  # run 밖 — 평범한 함수
            depth = _DEPTH.get()
            entry = await link.enter(step_name, depth, step_title)  # 게이트
            d_token = _DEPTH.set(depth + 1)
            try:
                result = await f(*args, **kwargs)
            except BaseException as exc:
                link.fail(entry, _detail_of(exc))
                raise  # 바깥 step / runner exception filter 로 전파
            finally:
                _DEPTH.reset(d_token)
            link.complete(entry)
            return result

        return wrapper

    if fn is not None:
        return decorate(fn)
    return decorate
