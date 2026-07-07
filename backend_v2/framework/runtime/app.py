from __future__ import annotations

import asyncio
import inspect
import logging
import time
from concurrent.futures import Future
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
from framework.runtime.snapshot import (
    ContractSnapshot,
    ModuleContract,
    build_module_contracts,
    build_snapshot,
)
from framework.transport.protocol import Handle, RemoteError, Transport

logger = logging.getLogger(__name__)

TRes = TypeVar("TRes", bound=BaseModel)


def _log_async_subscriber_exc(fut: "Future[Any]") -> None:
    """fire-and-forget async subscriber 의 예외를 삼키지 않고 로깅."""
    try:
        exc = fut.exception()
    except Exception:
        return  # cancelled 등 — 무시
    if exc is not None:
        logger.error("async @subscriber callback 실패: %s: %s", type(exc).__name__, exc)


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
            ModuleRuntime, _TransportRuntime(transport)
        )
        self._modules: list[Any] = []
        self._handles: list[Handle] = []
        self._started = False
        self._loop: asyncio.AbstractEventLoop | None = None
        self.mirror_snapshot_timeout: float = 2.0

    def add_module(self, cls: type, **deps: Any) -> Any:
        if self._started:
            raise RuntimeError(
                "Runtime 이 이미 start 됨 — add_module 은 start 전에만 가능 (stop 후 재구성)"
            )

        sig = inspect.signature(cls.__init__)
        unknown = set(deps) - set(sig.parameters)
        if unknown:
            raise TypeError(
                f"Module {cls.__name__} __init__ 에 없는 dep key: {sorted(unknown)} "
                f"— resolve 가 준 이름이 생성자 파라미터와 매칭돼야 함 (오타 의심)"
            )
        kwargs: dict[str, Any] = {}
        for name, param in sig.parameters.items():
            if name == "self":
                continue
            if name == "runtime":
                kwargs[name] = self._module_runtime
            elif name == "transport":
                # Boundary module 은 raw transport 를 직접 사용한다.
                # RawTransport 인터페이스를 주입해 relay 에 필요한 기능만 노출하고,
                # transport lifecycle 및 service 등록 권한은 숨긴다.
                kwargs[name] = self._transport
            elif name in deps:
                kwargs[name] = deps[name]
            elif param.default is inspect.Parameter.empty:
                raise TypeError(
                    f"Module {cls.__name__} 의 __init__ parameter {name!r} 가 주입 안 됨 "
                    f"(resolve deps 또는 runtime/transport/robot_id 에 없음)"
                )

        instance = cls(**kwargs)
        self._modules.append(instance)
        return instance

    async def start(self) -> None:
        if self._started:
            raise RuntimeError("Runtime 이 이미 start 됨 — start 는 한 번만 호출 가능")
        self._started = True
        self._loop = asyncio.get_running_loop()

        # 모든 통신 엔드포인트를 먼저 등록한다.
        # 그래야 모듈의 start()에서 다른 모듈을 안전하게 호출할 수 있다.
        for module in self._modules:
            self._register_module(module)

        # Mirror 초기 상태를 채운다.
        for module in self._modules:
            await self._initialize_mirrors(module)

        # start()는 sync/async 모두 허용한다.
        started: list[Any] = []
        try:
            for module in self._modules:
                if has_start(module):
                    result = module.start()
                    if asyncio.iscoroutine(result):
                        await result
                    started.append(module)
        except BaseException:
            # 부팅 중간 실패 — 이미 start 된 모듈을 역순 stop + endpoint 정리.
            # 방치하면 worker thread/task 가 좀비로 남아 프로세스 종료를 막는다.
            await self._stop_modules(started)
            self._undeclare_handles()
            self._started = False
            raise

    async def stop(self) -> None:
        if not self._started:
            return

        await self._stop_modules(self._modules)
        self._undeclare_handles()
        self._started = False

    async def _stop_modules(self, modules: list[Any]) -> None:
        for module in reversed(modules):
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

    def _undeclare_handles(self) -> None:
        for handle in self._handles:
            try:
                handle.undeclare()
            except Exception:
                pass
        self._handles.clear()

    @property
    def module_runtime(self) -> ModuleRuntime:
        return self._module_runtime

    def contract_snapshot(self) -> ContractSnapshot:
        return build_snapshot(self._modules)

    def module_contracts(self) -> list[ModuleContract]:
        return build_module_contracts(self._modules)

    # ── internal — register helpers ─────────────────────────

    def _register_module(self, module: Any) -> None:
        for bound_method, spec in discover_services(module):
            self._register_service(module, bound_method, spec)
        for bound_method, spec in discover_subscribers(module):
            self._register_subscriber(module, bound_method, spec)
        for _name, state in discover_mirrors(module):
            self._register_mirror_subscriber(module, state)
            self._register_mirror_liveliness(module, state)

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
                    f"@service {key} 가 robot-scoped ({{robot_id}} 포함) — Module "
                    f"{type(module).__name__} 에 self.robot_id 필요"
                )
            key = key.format(robot_id=robot_id)

        def handler_bytes(req_bytes: bytes) -> bytes:
            req = _decode_request(spec.req_cls, req_bytes)
            result = bound_method(req)
            # Zenoh 콜백은 워커 스레드에서 실행된다.
            # async 서비스는 이벤트 루프로 전달한 뒤 완료될 때까지 기다린다.
            if asyncio.iscoroutine(result):
                loop = self._loop
                if loop is None or loop.is_closed():
                    result.close()
                    raise RuntimeError(
                        f"@service {spec.method_name} 가 async 인데 event loop "
                        "미확보 (Runtime.start 전이거나 stop 후 호출)"
                    )
                result = asyncio.run_coroutine_threadsafe(result, loop).result()
            if not isinstance(result, BaseModel):
                raise TypeError(
                    f"@service {spec.method_name} 의 return 은 BaseModel 이어야 함 "
                    f"(got {type(result)})"
                )
            return _encode_response(result)

        handle = self._transport.register_service(key, handler_bytes)
        self._handles.append(handle)
        self._handles.append(self._transport.declare_liveliness(key))

    def _register_subscriber(
        self,
        module: Any,
        bound_method: Callable[..., Any],
        spec: SubscriberSpec,
    ) -> None:
        topic = spec.wire_key
        if "{robot_id}" in topic:
            topic = topic.replace("{robot_id}", "*")

        def callback_bytes(payload: bytes) -> None:
            event = decode_event(spec.event_cls, payload)
            result = bound_method(event)
            # async subscriber는 이벤트 루프로 전달해 실행한다.
            # 호출자는 완료를 기다리지 않는다.
            if asyncio.iscoroutine(result):
                loop = self._loop
                if loop is None or loop.is_closed():
                    result.close()
                    return
                fut = asyncio.run_coroutine_threadsafe(result, loop)
                fut.add_done_callback(_log_async_subscriber_exc)

        handle = self._transport.subscribe(topic, callback_bytes)
        self._handles.append(handle)

    # ── Mirror ─────────

    def _register_mirror_subscriber(
        self,
        module: Any,
        state: MirrorState[Any],
    ) -> None:
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

            if own_robot_id is not None:
                event_rid = getattr(event, "robot_id", None)
                if event_rid != own_robot_id:
                    return

            loop = self._loop
            if loop is None or loop.is_closed():
                return

            asyncio.run_coroutine_threadsafe(
                self._refetch_mirror(module, state),
                loop,
            )

        handle = self._transport.subscribe(topic, callback)
        self._handles.append(handle)

    def _register_mirror_liveliness(
        self,
        module: Any,
        state: MirrorState[Any],
    ) -> None:
        key = self._format_snapshot_key(module, state)

        def callback(_key: str, alive: bool) -> None:
            if not alive:
                return
            loop = self._loop
            if loop is None or loop.is_closed():
                return
            asyncio.run_coroutine_threadsafe(self._refetch_mirror(module, state), loop)

        self._handles.append(self._transport.subscribe_liveliness(key, callback))

    async def _initialize_mirrors(self, module: Any) -> None:
        for _name, state in discover_mirrors(module):
            try:
                await self._refetch_mirror(
                    module,
                    state,
                    timeout=self.mirror_snapshot_timeout,
                )
            except (TimeoutError, RemoteError):
                logger.info(
                    "Mirror initial snapshot 실패 (Owner 안 떠 있음) — liveliness/"
                    "event 로 자동 수렴: service=%s module=%s",
                    state.spec.snapshot_service,
                    type(module).__name__,
                )

    async def _refetch_mirror(
        self,
        module: Any,
        state: MirrorState[Any],
        *,
        timeout: float = 5.0,
    ) -> None:
        key = self._format_snapshot_key(module, state)
        async with state.refetch_lock:
            req = state.spec.snapshot_req(module)
            payload = _encode_request(req)
            res_bytes = await self._transport.call(key, payload, timeout)
            value = _decode_response(state.spec.value_cls, res_bytes)
            old = state.peek()
            state._set(value)
            if old != value and state.on_change_name is not None:
                cb = getattr(module, state.on_change_name, None)
                if cb is None:
                    return
                try:
                    result = cb(old, value)
                    if asyncio.iscoroutine(result):
                        await result
                except Exception:
                    logger.exception(
                        "Mirror on_change 실패 (%s.%s) — cache 는 갱신됨",
                        type(module).__name__,
                        state.on_change_name,
                    )

    @staticmethod
    def _format_snapshot_key(module: Any, state: MirrorState[Any]) -> str:
        key = state.spec.snapshot_service
        if "{robot_id}" in key:
            rid = getattr(module, "robot_id", None)
            if rid is None:
                raise ValueError(
                    f"Mirror snapshot_service {key} 가 robot-scoped — Module "
                    f"{type(module).__name__} 에 self.robot_id 필요"
                )
            key = key.format(robot_id=rid)
        return key
