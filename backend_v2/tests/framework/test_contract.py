"""tests/framework/test_contract.py — Step 2 검증 (§11) + retroactive patch v2 (§3.0).

두 원칙 (§3.0):
- explicit at every use site — service / subscriber / publish / Mirror 모두 wire_key 직접 박음
- typed — StrEnum value (raw str / __wire_topic__ class attribute lookup 자세 X)
- event class 자세 = pure Pydantic data (wire 자세 정보 박지 X)

검증 자세:
1. @service(wire_key) factory + ServiceSpec.wire_key field
2. @subscriber(wire_key) factory + SubscriberSpec.wire_key + event_cls (type hint)
3. @publishes((wire_key, event_cls), ...) pairs 자세 self-doc
4. encode/decode = msgpack (native bytes pass-through)
5. ZenohTransport 위 service + event E2E
"""

from __future__ import annotations

import threading
import time
from enum import StrEnum

import pytest
from pydantic import BaseModel

from framework.contract.envelope import ServiceRequest, ServiceResponse
from framework.contract.publisher import (
    decode_event,
    encode_event,
    get_publishes_spec,
    publishes,
)
from framework.contract.service import (
    get_service_spec,
    is_service,
    service,
)
from framework.contract.subscriber import (
    get_subscriber_spec,
    is_subscriber,
    subscriber,
)
from framework.transport.protocol import RemoteError
from infra.transport.zenoh import ZenohTransport


_LOCAL_CFG = {"mode": "peer", "scouting": {"multicast": {"enabled": False}}}


@pytest.fixture
def transport():
    t = ZenohTransport(_LOCAL_CFG)
    time.sleep(0.05)
    yield t
    t.close()


# ─── Test fixtures — wire keys (StrEnum) + domain class ─────────


class EchoServiceKey(StrEnum):
    ECHO = "srv/test/echo"
    FAIL = "srv/test/fail"


class GreetEventTopic(StrEnum):
    GREETED = "event/test/greeted"


class BlobStreamTopic(StrEnum):
    BLOB = "stream/test/blob"


class EchoRequest(BaseModel):
    message: str


class EchoResponse(BaseModel):
    echoed: str


class GreetEvent(BaseModel):
    """pure Pydantic data — wire 자세 정보 박지 X (§3.0 Option A)."""
    name: str


class BlobFrame(BaseModel):
    """msgpack native bytes pass-through 검증 자세 — bytes field 박음."""
    timestamp: float
    payload: bytes


# ─── @service factory — wire_key + ServiceSpec ───────────────────


def test_service_decorator_extracts_spec_with_wire_key():
    class Mod:
        @service(EchoServiceKey.ECHO)
        def echo(self, req: EchoRequest) -> EchoResponse:
            return EchoResponse(echoed=req.message)

    spec = get_service_spec(Mod.echo)
    assert spec is not None
    assert spec.method_name == "echo"
    assert spec.wire_key == "srv/test/echo"
    assert spec.req_cls is EchoRequest
    assert spec.res_cls is EchoResponse
    assert is_service(Mod.echo)


def test_service_decorator_accepts_raw_string_key():
    """StrEnum 추천이지만 raw str 도 받음 — 단 사용자 코드 자세 StrEnum."""

    class Mod:
        @service("srv/test/raw_key")
        def echo(self, req: EchoRequest) -> EchoResponse:
            return EchoResponse(echoed=req.message)

    spec = get_service_spec(Mod.echo)
    assert spec is not None
    assert spec.wire_key == "srv/test/raw_key"


def test_service_decorator_invalid_req_type_raises():
    with pytest.raises(TypeError, match="req parameter"):

        class Mod:
            @service(EchoServiceKey.ECHO)
            def bad(self, req: int) -> EchoResponse:  # type: ignore[type-var]
                return EchoResponse(echoed="x")


def test_service_decorator_missing_return_type_raises():
    with pytest.raises(TypeError, match="return type hint"):

        class Mod:
            @service(EchoServiceKey.ECHO)
            def bad(self, req: EchoRequest):  # no return annotation
                return EchoResponse(echoed="x")


def test_service_decorator_wrong_arity_raises():
    with pytest.raises(TypeError, match="self \\+ req"):

        class Mod:
            @service(EchoServiceKey.ECHO)
            def bad(self, a: EchoRequest, b: EchoRequest) -> EchoResponse:  # type: ignore[type-arg]
                return EchoResponse(echoed="x")


# ─── @subscriber factory — wire_key + SubscriberSpec ─────────────


def test_subscriber_decorator_extracts_spec_with_wire_key():
    class Mod:
        @subscriber(GreetEventTopic.GREETED)
        def on_greet(self, event: GreetEvent) -> None:
            _ = event

    spec = get_subscriber_spec(Mod.on_greet)
    assert spec is not None
    assert spec.method_name == "on_greet"
    assert spec.wire_key == "event/test/greeted"
    assert spec.event_cls is GreetEvent
    assert is_subscriber(Mod.on_greet)


def test_subscriber_decorator_accepts_raw_string_key():
    class Mod:
        @subscriber("event/test/raw_key")
        def on_greet(self, event: GreetEvent) -> None:
            _ = event

    spec = get_subscriber_spec(Mod.on_greet)
    assert spec is not None
    assert spec.wire_key == "event/test/raw_key"


def test_subscriber_decorator_invalid_event_type_raises():
    with pytest.raises(TypeError, match="event parameter"):

        class Mod:
            @subscriber(GreetEventTopic.GREETED)
            def bad(self, event: str) -> None:  # type: ignore[type-var]
                _ = event


def test_subscriber_decorator_wrong_arity_raises():
    with pytest.raises(TypeError, match="self \\+ event"):

        class Mod:
            @subscriber(GreetEventTopic.GREETED)
            def bad(self, a: GreetEvent, b: GreetEvent) -> None:
                _ = a, b


# ─── @publishes class-level spec — pairs ──────────────────────


def test_publishes_decorator_records_pairs():
    @publishes((GreetEventTopic.GREETED, GreetEvent))
    class Mod:
        pass

    spec = get_publishes_spec(Mod)
    assert spec is not None
    assert spec.pairs == (("event/test/greeted", GreetEvent),)


def test_publishes_decorator_multi_pairs():
    @publishes(
        (GreetEventTopic.GREETED, GreetEvent),
        (BlobStreamTopic.BLOB, BlobFrame),
    )
    class Mod:
        pass

    spec = get_publishes_spec(Mod)
    assert spec is not None
    assert set(spec.pairs) == {
        ("event/test/greeted", GreetEvent),
        ("stream/test/blob", BlobFrame),
    }


def test_publishes_decorator_invalid_pair_raises():
    with pytest.raises(TypeError, match="event_cls"):

        @publishes((GreetEventTopic.GREETED, int))  # type: ignore[arg-type]
        class Mod:
            pass


# ─── encode / decode — msgpack native bytes pass-through ────────


def test_encode_decode_round_trip():
    evt = GreetEvent(name="alice")
    wire = encode_event(evt)
    restored = decode_event(GreetEvent, wire)
    assert isinstance(restored, GreetEvent)
    assert restored.name == "alice"


def test_encode_native_bytes_no_base64_overhead():
    """msgpack 자세 native bytes pass-through — JPEG/depth 자세 base64 overhead 0."""
    payload = b"\x00\x01\x02\xff" * 1024  # 4KB binary
    evt = BlobFrame(timestamp=1.0, payload=payload)
    wire = encode_event(evt)
    assert len(wire) < len(payload) + 200, (
        f"wire size {len(wire)} 자세 base64 overhead 의심 (payload {len(payload)})"
    )
    restored = decode_event(BlobFrame, wire)
    assert restored.payload == payload


# ─── envelope wire round-trip ────────────────────────────


def test_envelope_wrap_unwrap():
    req = ServiceRequest[EchoRequest](timestamp=time.time(), data=EchoRequest(message="hi"))
    wire = req.model_dump_json().encode()
    restored = ServiceRequest[EchoRequest].model_validate_json(wire)
    assert restored.data.message == "hi"


# ─── E2E — Step 2 검증 핵심 ─────────────────────────────


async def test_service_end_to_end_with_transport(transport: ZenohTransport):
    class EchoModule:
        @service(EchoServiceKey.ECHO)
        def echo(self, req: EchoRequest) -> EchoResponse:
            return EchoResponse(echoed=f"got:{req.message}")

    mod = EchoModule()
    spec = get_service_spec(mod.echo)
    assert spec is not None

    # wire key = ServiceSpec.wire_key (explicit + typed).
    key = spec.wire_key

    def handler_bytes(req_bytes: bytes) -> bytes:
        envelope = ServiceRequest[EchoRequest].model_validate_json(req_bytes)
        result = spec.handler(mod, envelope.data)
        return ServiceResponse[EchoResponse](
            timestamp=time.time(),
            data=result,
        ).model_dump_json().encode()

    handle = transport.register_service(key, handler_bytes)
    try:
        time.sleep(0.1)
        req = ServiceRequest[EchoRequest](
            timestamp=time.time(), data=EchoRequest(message="hi")
        )
        res_bytes = await transport.call(
            key, req.model_dump_json().encode(), timeout=2.0
        )
        res = ServiceResponse[EchoResponse].model_validate_json(res_bytes)
        assert res.data.echoed == "got:hi"
    finally:
        handle.undeclare()


async def test_service_handler_exception_propagates_via_transport(
    transport: ZenohTransport,
):
    """@service handler raise → caller 측 RemoteError (transport layer wire)."""

    class NotFound(Exception):
        pass

    class Mod:
        @service(EchoServiceKey.FAIL)
        def fail(self, req: EchoRequest) -> EchoResponse:
            raise NotFound(f"no entry for {req.message}")

    mod = Mod()
    spec = get_service_spec(mod.fail)
    assert spec is not None
    key = spec.wire_key

    def handler_bytes(req_bytes: bytes) -> bytes:
        envelope = ServiceRequest[EchoRequest].model_validate_json(req_bytes)
        result = spec.handler(mod, envelope.data)
        return result.model_dump_json().encode()

    handle = transport.register_service(key, handler_bytes)
    try:
        time.sleep(0.1)
        req = ServiceRequest[EchoRequest](
            timestamp=time.time(), data=EchoRequest(message="x")
        )
        with pytest.raises(RemoteError) as ei:
            await transport.call(
                key, req.model_dump_json().encode(), timeout=2.0
            )
        assert ei.value.type_name == "NotFound"
        assert "no entry for x" in ei.value.message
    finally:
        handle.undeclare()


# ─── event publish/subscribe E2E ────────────────────────


def test_event_publish_subscribe_end_to_end(transport: ZenohTransport):
    """@subscriber(wire_key) + msgpack encode/decode E2E."""
    received: list[GreetEvent] = []
    done = threading.Event()

    class Mod:
        @subscriber(GreetEventTopic.GREETED)
        def on_greet(self, event: GreetEvent) -> None:
            received.append(event)
            done.set()

    mod = Mod()
    spec = get_subscriber_spec(mod.on_greet)
    assert spec is not None
    assert spec.wire_key == "event/test/greeted"

    def callback_bytes(payload: bytes) -> None:
        evt = decode_event(spec.event_cls, payload)
        spec.handler(mod, evt)

    handle = transport.subscribe(spec.wire_key, callback_bytes)
    try:
        time.sleep(0.1)
        evt = GreetEvent(name="alice")
        # publisher 자세 wire_key 직접 박음 (Module 코드 자세 self.runtime.publish(wire_key, evt))
        transport.publish(str(GreetEventTopic.GREETED), encode_event(evt))
        assert done.wait(timeout=2.0)
        assert received == [GreetEvent(name="alice")]
    finally:
        handle.undeclare()


def test_event_publish_subscribe_with_bytes_payload(transport: ZenohTransport):
    """stream/ 자세 bytes field 박힌 event 자세 round-trip — msgpack native pass-through."""
    received: list[BlobFrame] = []
    done = threading.Event()

    class Mod:
        @subscriber(BlobStreamTopic.BLOB)
        def on_blob(self, event: BlobFrame) -> None:
            received.append(event)
            done.set()

    mod = Mod()
    spec = get_subscriber_spec(mod.on_blob)
    assert spec is not None
    assert spec.wire_key == "stream/test/blob"

    def callback_bytes(payload: bytes) -> None:
        evt = decode_event(spec.event_cls, payload)
        spec.handler(mod, evt)

    handle = transport.subscribe(spec.wire_key, callback_bytes)
    try:
        time.sleep(0.1)
        binary = b"\x00\x01\x02\xff" * 256  # 1KB binary
        evt = BlobFrame(timestamp=1.0, payload=binary)
        transport.publish(str(BlobStreamTopic.BLOB), encode_event(evt))
        assert done.wait(timeout=2.0)
        assert len(received) == 1
        assert received[0].payload == binary
    finally:
        handle.undeclare()
