"""Lifecycle Protocol — Module 의 선택적 start / stop (§3.6).

Module 자세 base class 상속 강제 X (§10.6) — Lifecycle 자세 Protocol 박음.
Module 이 `start` / `stop` 박으면 Runtime 자세 호출, 안 박으면 skip (duck typing).

sync / async 둘 다 지원:
- `def start(self) -> None:` — sync
- `async def start(self) -> None:` — async (Mirror snapshot 자세 호출 자세 자연)

부팅 순서 (§3.6):
① 모든 Module instantiate (Runtime.add_module)
② 모든 Module 의 @service / @subscriber register (Runtime.start 의 phase 2)
③ 모든 Module 의 start() 호출 (Runtime.start 의 phase 3) — Mirror snapshot 자세 ② 이후 안전
"""

from __future__ import annotations

from typing import Any, Awaitable, Protocol, runtime_checkable


@runtime_checkable
class Lifecycle(Protocol):
    """Module 의 선택적 start/stop. duck typing 자세 — `hasattr(mod, 'start')` 자세 check."""

    def start(self) -> None | Awaitable[None]:
        """Module 시작 자세. sync 또는 async."""
        ...

    def stop(self) -> None | Awaitable[None]:
        """Module 종료 자세. sync 또는 async."""
        ...


def has_start(mod: Any) -> bool:
    return callable(getattr(mod, "start", None))


def has_stop(mod: Any) -> bool:
    return callable(getattr(mod, "stop", None))
