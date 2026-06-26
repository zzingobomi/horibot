"""Runtime — Module lifecycle + DI + Transport wiring (§3.6 / §3.7 / §11 Step 3).

부팅 순서 (§3.6):
① instantiate — add_module 시 constructor 호출 + DI inject (runtime + 사용자 deps)
② register — start() 의 phase 2: @service queryable + @subscriber callback transport 박음
③ start — start() 의 phase 3: Module 의 start() 호출 (sync / async 둘 다)

`{robot_id}` placeholder 자세 (§3.7):
- service queryable register 자세 = Module instance 의 self.robot_id 자세 substitute
- event publish 자세 = event payload 의 robot_id field 자세 substitute
- service call (caller) 자세 = caller 의 `robot_id=` 인자 자세 substitute
- event subscribe (robot-scoped event) 자세 = Zenoh wildcard `*` 자세 substitute
"""

from __future__ import annotations

import asyncio
import inspect
import time
from typing import Any, Callable, TypeVar, cast

import msgspec
from pydantic import BaseModel

from framework.contract.envelope import ServiceRequest, ServiceResponse
from framework.contract.publisher import decode_event, encode_event
from framework.contract.service import ServiceSpec, get_service_spec
from framework.contract.subscriber import SubscriberSpec
from framework.runtime.api import ModuleRuntime
from framework.runtime.discovery import discover_services, discover_subscribers
from framework.runtime.lifecycle import has_start, has_stop
from framework.transport.protocol import Handle, Transport

TRes = TypeVar("TRes", bound=BaseModel)


# ─── envelope encoding (msgpack, consistent with event encoding) ────


def _encode_request(req: BaseModel) -> bytes:
    envelope = ServiceRequest(timestamp=time.time(), data=req)
    return msgspec.msgpack.encode(envelope.model_dump())


def _decode_request(req_cls: type[BaseModel], payload: bytes) -> BaseModel:
    data = msgspec.msgpack.decode(payload)
    return req_cls.model_validate(data["data"])


def _encode_response(res: BaseModel) -> bytes:
    envelope = ServiceResponse(timestamp=time.time(), data=res)
    return msgspec.msgpack.encode(envelope.model_dump())


def _decode_response(res_cls: type[BaseModel], payload: bytes) -> BaseModel:
    data = msgspec.msgpack.decode(payload)
    return res_cls.model_validate(data["data"])


# ─── _TransportRuntime — ModuleRuntime Protocol impl ───────────


class _TransportRuntime:
    """Transport 자세 wrap 박은 ModuleRuntime adapter. Module 자세 import boundary 자세 X."""

    def __init__(self, transport: Transport):
        self._transport = transport

    def publish(self, wire_key: str, event: BaseModel) -> None:
        topic = str(wire_key)
        if "{robot_id}" in topic:
            robot_id = getattr(event, "robot_id", None)
            if robot_id is None:
                raise ValueError(
                    f"wire_key {topic!r} 자세 {{robot_id}} placeholder 박혀있지만 "
                    f"event {type(event).__name__} payload 자세 robot_id field 없음"
                )
            topic = topic.format(robot_id=robot_id)
        self._transport.publish(topic, encode_event(event))

    async def call(
        self,
        target: Callable[..., TRes],
        req: BaseModel,
        *,
        robot_id: str | None = None,
        timeout: float = 5.0,
    ) -> TRes:
        spec = get_service_spec(target)
        if spec is None:
            raise TypeError(
                f"target 자세 @service 박힌 method reference 박힘 (got {target!r})"
            )
        key = spec.wire_key
        if "{robot_id}" in key:
            if robot_id is None:
                raise ValueError(
                    f"service {key} 자세 robot-scoped — call 시 robot_id= 인자 필요"
                )
            key = key.format(robot_id=robot_id)

        payload = _encode_request(req)
        res_bytes = await self._transport.call(key, payload, timeout)
        return cast(TRes, _decode_response(spec.res_cls, res_bytes))


# ─── Runtime — Module instantiate + register + lifecycle ─────


class Runtime:
    """Module lifecycle 자세 orchestrate."""

    def __init__(self, transport: Transport):
        self._transport = transport
        self._module_runtime: ModuleRuntime = cast(ModuleRuntime, _TransportRuntime(transport))
        self._modules: list[Any] = []
        self._handles: list[Handle] = []
        self._started = False

    def add_module(self, cls: type, **deps: Any) -> Any:
        """Module instantiate + DI inject. constructor 자세 runtime 자세 자동 박힘.

        deps 자세 사용자 deps (robot_id / repo / object_store 등). cls.__init__ 의
        parameter 자세 매칭 inject.
        """
        if self._started:
            raise RuntimeError("Runtime 자세 이미 start — add_module 자세 stop 후 박음")

        sig = inspect.signature(cls.__init__)
        kwargs: dict[str, Any] = {}
        for name, param in sig.parameters.items():
            if name == "self":
                continue
            if name == "runtime":
                kwargs[name] = self._module_runtime
            elif name in deps:
                kwargs[name] = deps[name]
            elif param.default is inspect.Parameter.empty:
                raise TypeError(
                    f"Module {cls.__name__} __init__ parameter {name!r} 자세 박혀있지 X"
                )

        instance = cls(**kwargs)
        self._modules.append(instance)
        return instance

    async def start(self) -> None:
        """Phase 2 (register) + Phase 3 (Module start). §3.6."""
        if self._started:
            raise RuntimeError("Runtime 자세 이미 start 박힘")
        self._started = True

        # Phase 2: register all services + subscribers
        for module in self._modules:
            self._register_module(module)

        # Phase 3: call start() on each Module (sync or async)
        for module in self._modules:
            if has_start(module):
                result = module.start()
                if asyncio.iscoroutine(result):
                    await result

    async def stop(self) -> None:
        """Module stop + transport handle undeclare 자세 reverse order."""
        if not self._started:
            return

        # Phase 3 reverse: stop() on each Module
        for module in reversed(self._modules):
            if has_stop(module):
                result = module.stop()
                if asyncio.iscoroutine(result):
                    await result

        # Phase 2 reverse: undeclare transport handles
        for handle in self._handles:
            try:
                handle.undeclare()
            except Exception:
                pass  # swallow during shutdown
        self._handles.clear()
        self._started = False

    @property
    def module_runtime(self) -> ModuleRuntime:
        """test / external 자세 활용 자세."""
        return self._module_runtime

    # ── internal — register helpers ─────────────────────────

    def _register_module(self, module: Any) -> None:
        for bound_method, spec in discover_services(module):
            self._register_service(module, bound_method, spec)
        for bound_method, spec in discover_subscribers(module):
            self._register_subscriber(module, bound_method, spec)

    def _register_service(
        self,
        module: Any,
        bound_method: Callable[..., Any],
        spec: ServiceSpec,
    ) -> None:
        key = spec.wire_key
        if "{robot_id}" in key:
            robot_id = getattr(module, "robot_id", None)
            if robot_id is None:
                raise ValueError(
                    f"@service {key} 자세 robot-scoped 박힘 — Module "
                    f"{type(module).__name__} 자세 self.robot_id 필요"
                )
            key = key.format(robot_id=robot_id)

        def handler_bytes(req_bytes: bytes) -> bytes:
            req = _decode_request(spec.req_cls, req_bytes)
            result = bound_method(req)
            if not isinstance(result, BaseModel):
                raise TypeError(
                    f"@service {spec.method_name} return 자세 BaseModel 박힘 "
                    f"(got {type(result)})"
                )
            return _encode_response(result)

        handle = self._transport.register_service(key, handler_bytes)
        self._handles.append(handle)

    def _register_subscriber(
        self,
        module: Any,
        bound_method: Callable[..., Any],
        spec: SubscriberSpec,
    ) -> None:
        topic = spec.wire_key
        if "{robot_id}" in topic:
            # robot-scoped event — Zenoh single-chunk wildcard
            topic = topic.replace("{robot_id}", "*")

        def callback_bytes(payload: bytes) -> None:
            event = decode_event(spec.event_cls, payload)
            bound_method(event)

        handle = self._transport.subscribe(topic, callback_bytes)
        self._handles.append(handle)
