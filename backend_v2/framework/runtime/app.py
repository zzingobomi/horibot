from __future__ import annotations

import asyncio
import inspect
import logging
import time
from typing import Any, Callable, TypeVar, cast

import msgspec
from pydantic import BaseModel

from framework.contract.envelope import ServiceRequest, ServiceResponse
from framework.contract.mirror import MirrorState, discover_mirrors
from framework.contract.publisher import decode_event, encode_event
from framework.contract.service import ServiceSpec
from framework.contract.subscriber import SubscriberSpec
from framework.runtime.api import ModuleRuntime
from framework.runtime.discovery import discover_services, discover_subscribers
from framework.runtime.lifecycle import has_start, has_stop
from framework.runtime.snapshot import ContractSnapshot, build_snapshot
from framework.transport.protocol import Handle, RemoteError, Transport

logger = logging.getLogger(__name__)

TRes = TypeVar("TRes", bound=BaseModel)


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


class _TransportRuntime:
    def __init__(self, transport: Transport):
        self._transport = transport

    def publish(self, wire_key: str, event: BaseModel) -> None:
        topic = str(wire_key)
        if "{robot_id}" in topic:
            robot_id = getattr(event, "robot_id", None)
            if robot_id is None:
                raise ValueError(
                    f"wire_key {topic!r}  {{robot_id}} placeholder 있지만 "
                    f"event {type(event).__name__} payload  robot_id field 없음"
                )
            topic = topic.format(robot_id=robot_id)
        self._transport.publish(topic, encode_event(event))

    async def call(
        self,
        key: str,
        req: BaseModel,
        res_cls: type[TRes],
        *,
        robot_id: str | None = None,
        timeout: float = 5.0,
    ) -> TRes:
        key_str = str(key)
        if "{robot_id}" in key_str:
            if robot_id is None:
                raise ValueError(
                    f"service {key_str} robot-scoped — call 시 robot_id= 인자 필요"
                )
            key_str = key_str.format(robot_id=robot_id)

        payload = _encode_request(req)
        res_bytes = await self._transport.call(key_str, payload, timeout)
        return cast(TRes, _decode_response(res_cls, res_bytes))


class Runtime:
    def __init__(self, transport: Transport):
        self._transport = transport
        self._module_runtime: ModuleRuntime = cast(
            ModuleRuntime, _TransportRuntime(transport))
        self._modules: list[Any] = []
        self._handles: list[Handle] = []
        self._started = False
        self._loop: asyncio.AbstractEventLoop | None = None
        # Mirror initial snapshot timeout — Owner 안 떠 있으면 짧게 포기, event 로 fallback
        self.mirror_snapshot_timeout: float = 2.0

    def add_module(self, cls: type, **deps: Any) -> Any:
        if self._started:
            raise RuntimeError("Runtime 이미 start — add_module stop 후 박음")

        sig = inspect.signature(cls.__init__)
        kwargs: dict[str, Any] = {}
        for name, param in sig.parameters.items():
            if name == "self":
                continue
            if name == "runtime":
                kwargs[name] = self._module_runtime
            elif name == "transport":
                # Boundary Module (Bridge) 전용 raw transport 주입.
                # param 타입을 RawTransport 로 좁혀 close/register_service 권한 차단.
                kwargs[name] = self._transport
            elif name in deps:
                kwargs[name] = deps[name]
            elif param.default is inspect.Parameter.empty:
                raise TypeError(
                    f"Module {cls.__name__} __init__ parameter {name!r}  박혀있지 X"
                )

        instance = cls(**kwargs)
        self._modules.append(instance)
        return instance

    async def start(self) -> None:
        if self._started:
            raise RuntimeError("Runtime  이미 start 박힘")
        self._started = True
        self._loop = asyncio.get_running_loop()

        # Phase 2 — register all services + subscribers + Mirror subscribers
        for module in self._modules:
            self._register_module(module)

        # Phase 3a — Mirror initial snapshot fetch (non-blocking, fail OK)
        for module in self._modules:
            await self._initialize_mirrors(module)

        # Phase 3b — Module.start() (sync or async)
        for module in self._modules:
            if has_start(module):
                result = module.start()
                if asyncio.iscoroutine(result):
                    await result

    async def stop(self) -> None:
        if not self._started:
            return

        # 한 모듈의 stop() 실패가 나머지 stop + handle undeclare 를 막지 않게 격리.
        # 실 하드웨어 (driver.close() 가 USB 에서 throw 가능) shutdown 누수 차단.
        for module in reversed(self._modules):
            if has_stop(module):
                try:
                    result = module.stop()
                    if asyncio.iscoroutine(result):
                        await result
                except Exception:
                    logger.exception(
                        "module %s stop() 실패 — 나머지 shutdown 계속",
                        type(module).__name__,
                    )

        for handle in self._handles:
            try:
                handle.undeclare()
            except Exception:
                pass
        self._handles.clear()
        self._started = False

    @property
    def module_runtime(self) -> ModuleRuntime:
        return self._module_runtime

    def contract_snapshot(self) -> ContractSnapshot:
        """로드된 Module 들의 계약(wire_key → payload) 스냅샷.

        gen_contract (frontend TS) 가 module.py import 없이 payload 매핑을 얻는
        source — bridge 의 GET /contract.json 이 serializer 통해 노출
        (frontend_contract_gen.md §6.1). start 여부 무관 (add_module 만 되면 유효).
        """
        return build_snapshot(self._modules)

    # ── internal — register helpers ─────────────────────────

    def _register_module(self, module: Any) -> None:
        for bound_method, spec in discover_services(module):
            self._register_service(module, bound_method, spec)
        for bound_method, spec in discover_subscribers(module):
            self._register_subscriber(module, bound_method, spec)
        for _name, state in discover_mirrors(module):
            self._register_mirror_subscriber(module, state)

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
                    f"@service {key}  robot-scoped 박힘 — Module "
                    f"{type(module).__name__}  self.robot_id 필요"
                )
            key = key.format(robot_id=robot_id)

        def handler_bytes(req_bytes: bytes) -> bytes:
            req = _decode_request(spec.req_cls, req_bytes)
            result = bound_method(req)
            if not isinstance(result, BaseModel):
                raise TypeError(
                    f"@service {spec.method_name} return  BaseModel 박힘 "
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

    # ── Mirror — phase 2 subscribe + phase 3 snapshot ─────────

    def _register_mirror_subscriber(
        self,
        module: Any,
        state: MirrorState[Any],
    ) -> None:
        """Phase 2 — change_topic subscribe. event 받으면 async refetch trigger."""
        topic = state.spec.change_topic
        if "{robot_id}" in topic:
            topic = topic.replace("{robot_id}", "*")

        own_robot_id = getattr(module, "robot_id", None)

        def callback(payload: bytes) -> None:
            try:
                event = decode_event(state.spec.change_event_cls, payload)
            except Exception:
                logger.exception("Mirror callback decode failed: topic=%s", topic)
                return
            # robot_id filter — robot-scoped Reader 면 자기 robot 만
            if own_robot_id is not None:
                event_rid = getattr(event, "robot_id", None)
                if event_rid != own_robot_id:
                    return
            loop = self._loop
            if loop is None or loop.is_closed():
                return
            # refetch — callback thread → asyncio loop
            asyncio.run_coroutine_threadsafe(
                self._refetch_mirror(module, state), loop,
            )

        handle = self._transport.subscribe(topic, callback)
        self._handles.append(handle)

    async def _initialize_mirrors(self, module: Any) -> None:
        """Phase 3a — initial snapshot fetch. fail OK (Owner 안 떠 있으면 event 로 fallback)."""
        for _name, state in discover_mirrors(module):
            try:
                await self._refetch_mirror(
                    module, state, timeout=self.mirror_snapshot_timeout,
                )
            except (TimeoutError, RemoteError):
                logger.info(
                    "Mirror initial snapshot 실패 (Owner 안 떠 있음) — event 받으면 refetch: "
                    "service=%s module=%s",
                    state.spec.snapshot_service, type(module).__name__,
                )

    async def _refetch_mirror(
        self,
        module: Any,
        state: MirrorState[Any],
        *,
        timeout: float = 5.0,
    ) -> None:
        """snapshot_service 호출 → value_cls decode → MirrorState._set."""
        req = state.spec.snapshot_req(module)
        key = state.spec.snapshot_service
        if "{robot_id}" in key:
            rid = getattr(module, "robot_id", None)
            if rid is None:
                raise ValueError(
                    f"Mirror snapshot_service {key} 가 robot-scoped — Module "
                    f"{type(module).__name__} 에 self.robot_id 필요"
                )
            key = key.format(robot_id=rid)

        payload = _encode_request(req)
        res_bytes = await self._transport.call(key, payload, timeout)
        value = _decode_response(state.spec.value_cls, res_bytes)
        state._set(value)
