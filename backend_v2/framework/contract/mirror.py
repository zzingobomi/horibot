from __future__ import annotations

from dataclasses import dataclass
from threading import RLock
from typing import Any, Callable, Generic, TypeVar, overload

from pydantic import BaseModel

T = TypeVar("T", bound=BaseModel)


class NotReady(Exception):
    pass


@dataclass(frozen=True)
class MirrorSpec:
    snapshot_service: str
    snapshot_req: Callable[[Any], BaseModel]
    change_topic: str
    value_cls: type[BaseModel]
    change_event_cls: type[BaseModel]


class MirrorState(Generic[T]):
    def __init__(self, spec: MirrorSpec):
        self._spec = spec
        self._lock = RLock()
        self._cache: T | None = None
        self._initialized = False

    @property
    def spec(self) -> MirrorSpec:
        return self._spec

    @property
    def is_ready(self) -> bool:
        with self._lock:
            return self._initialized

    @property
    def value(self) -> T:
        with self._lock:
            if not self._initialized:
                raise NotReady(
                    f"Mirror[{self._spec.value_cls.__name__}] 아직 snapshot/event 못 받음"
                )
            assert self._cache is not None
            return self._cache

    def _set(self, value: T) -> None:
        with self._lock:
            self._cache = value
            self._initialized = True


class Mirror(Generic[T]):
    def __init__(
        self,
        *,
        snapshot_service: str,
        snapshot_req: Callable[[Any], BaseModel],
        change_topic: str,
        value_cls: type[T],
        change_event_cls: type[BaseModel],
    ):
        self.spec = MirrorSpec(
            snapshot_service=str(snapshot_service),
            snapshot_req=snapshot_req,
            change_topic=str(change_topic),
            value_cls=value_cls,
            change_event_cls=change_event_cls,
        )
        self._attr_name: str | None = None

    def __set_name__(self, owner: type, name: str) -> None:
        self._attr_name = name

    @overload
    def __get__(self, instance: None, owner: type) -> Mirror[T]: ...

    @overload
    def __get__(self, instance: Any, owner: type |
                None = ...) -> MirrorState[T]: ...

    def __get__(
        self, instance: Any, owner: type | None = None
    ) -> MirrorState[T] | Mirror[T]:
        if instance is None:
            return self  # class access — return descriptor for introspection
        if self._attr_name is None:
            raise RuntimeError(
                f"Mirror descriptor missing __set_name__ "
                f"(owner={owner}, instance={type(instance).__name__})"
            )
        # per-instance state stored in instance.__dict__ — auto-create on first access
        state_key = f"_mirror_{self._attr_name}"
        state = instance.__dict__.get(state_key)
        if state is None:
            state = MirrorState[T](self.spec)
            instance.__dict__[state_key] = state
        return state


def discover_mirrors(module: Any) -> list[tuple[str, MirrorState[Any]]]:
    """Walk module class for Mirror descriptors. Returns (attr_name, state)."""
    result: list[tuple[str, MirrorState[Any]]] = []
    cls = type(module)
    for name in dir(cls):
        if name.startswith("_"):
            continue
        attr = getattr(cls, name, None)
        if isinstance(attr, Mirror):
            # triggers __get__ → auto-create state
            state = getattr(module, name)
            result.append((name, state))
    return result
