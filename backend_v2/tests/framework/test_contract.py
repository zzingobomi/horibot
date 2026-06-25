"""tests/framework/test_contract.py — Step 2 검증 (§11).

검증 두 case:
1. @service 박은 메소드 inspect → ServiceSpec 추출.
2. ZenohTransport 위에 service register + same-session call round-trip.

추가 surface — @subscriber / @publishes / event_to_topic / envelope wrap+unwrap +
event publish/subscribe E2E.
"""

from __future__ import annotations

import threading
import time

import pytest
from pydantic import BaseModel

from framework.contract.envelope import ServiceRequest, ServiceResponse
from framework.contract.publisher import (
    decode_event,
    encode_event,
    event_to_topic,
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


# ─── Test fixtures — domain class ────────────────────────


class EchoRequest(BaseModel):
    message: str


class EchoResponse(BaseModel):
    echoed: str


class GreetEvent(BaseModel):
    name: str


# ─── @service spec extraction ────────────────────────────


def test_service_decorator_extracts_spec():
    class Mod:
        @service
        def echo(self, req: EchoRequest) -> EchoResponse:
            return EchoResponse(echoed=req.message)

    spec = get_service_spec(Mod.echo)
    assert spec is not None
    assert spec.method_name == "echo"
    assert spec.req_cls is EchoRequest
    assert spec.res_cls is EchoResponse
    assert is_service(Mod.echo)


def test_service_decorator_invalid_req_type_raises():
    with pytest.raises(TypeError, match="req parameter"):

        class Mod:
            @service
            def bad(self, req: int) -> EchoResponse:  # type: ignore[type-var]
                return EchoResponse(echoed="x")


def test_service_decorator_missing_return_type_raises():
    with pytest.raises(TypeError, match="return type hint"):

        class Mod:
            @service
            def bad(self, req: EchoRequest):  # no return annotation
                return EchoResponse(echoed="x")


def test_service_decorator_wrong_arity_raises():
    with pytest.raises(TypeError, match="self \\+ req"):

        class Mod:
            @service
            def bad(self, a: EchoRequest, b: EchoRequest) -> EchoResponse:  # type: ignore[type-arg]
                return EchoResponse(echoed="x")


# ─── @subscriber spec extraction ─────────────────────────


def test_subscriber_decorator_extracts_spec():
    class Mod:
        @subscriber
        def on_greet(self, event: GreetEvent) -> None:
            _ = event

    spec = get_subscriber_spec(Mod.on_greet)
    assert spec is not None
    assert spec.method_name == "on_greet"
    assert spec.event_cls is GreetEvent
    assert is_subscriber(Mod.on_greet)


def test_subscriber_decorator_invalid_event_type_raises():
    with pytest.raises(TypeError, match="event parameter"):

        class Mod:
            @subscriber
            def bad(self, event: str) -> None:  # type: ignore[type-var]
                _ = event


# ─── @publishes class-level spec ─────────────────────────


def test_publishes_decorator_records_event_classes():
    @publishes(GreetEvent)
    class Mod:
        pass

    spec = get_publishes_spec(Mod)
    assert spec is not None
    assert spec.event_classes == (GreetEvent,)


def test_publishes_decorator_multi_events():
    class EventA(BaseModel):
        x: int

    class EventB(BaseModel):
        y: int

    @publishes(EventA, EventB)
    class Mod:
        pass

    spec = get_publishes_spec(Mod)
    assert spec is not None
    assert set(spec.event_classes) == {EventA, EventB}


# ─── event_to_topic ──────────────────────────────────────


def test_event_to_topic_camel_to_snake():
    class CalibrationActivated(BaseModel):
        pass

    assert event_to_topic(CalibrationActivated) == "event/calibration_activated"


def test_event_to_topic_acronym_safe():
    class HTTPResponse(BaseModel):
        pass

    # 연속 대문자 (acronym) 는 한 단어로 묶음 — HTTPResponse → http_response
    assert event_to_topic(HTTPResponse) == "event/http_response"


# ─── envelope wire round-trip ────────────────────────────


def test_envelope_wrap_unwrap():
    req = ServiceRequest[EchoRequest](timestamp=time.time(), data=EchoRequest(message="hi"))
    wire = req.model_dump_json().encode()
    restored = ServiceRequest[EchoRequest].model_validate_json(wire)
    assert restored.data.message == "hi"


# ─── E2E — Step 2 검증 핵심 ─────────────────────────────
# @service 메소드를 ZenohTransport 위에 wire + same-session call round-trip.


async def test_service_end_to_end_with_transport(transport: ZenohTransport):
    class EchoModule:
        @service
        def echo(self, req: EchoRequest) -> EchoResponse:
            return EchoResponse(echoed=f"got:{req.message}")

    mod = EchoModule()
    spec = get_service_spec(mod.echo)
    assert spec is not None

    # Runtime (Step 3) 가 박을 wiring 의 manual 버전.
    # ServiceRequest envelope unwrap → handler 호출 → ServiceResponse envelope wrap.
    # Zenoh key expression — leading slash / empty chunk 금지.
    # 실 시스템 key 는 `horibot/{robot_id}/{module}/{method}` 형식 (spec §4.1).
    key = f"test/svc/{spec.method_name}"

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
    """@service 의 handler 가 raise → caller 측 RemoteError (transport layer 가 wire)."""

    class NotFound(Exception):
        pass

    class Mod:
        @service
        def fail(self, req: EchoRequest) -> EchoResponse:
            raise NotFound(f"no entry for {req.message}")

    mod = Mod()
    spec = get_service_spec(mod.fail)
    assert spec is not None
    # Zenoh key expression — leading slash / empty chunk 금지.
    # 실 시스템 key 는 `horibot/{robot_id}/{module}/{method}` 형식 (spec §4.1).
    key = f"test/svc/{spec.method_name}"

    def handler_bytes(req_bytes: bytes) -> bytes:
        envelope = ServiceRequest[EchoRequest].model_validate_json(req_bytes)
        # spec.handler raise → 그대로 propagate, transport 가 reply_err.
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
    """@subscriber spec + event_to_topic + encode/decode 가 transport 위에서 동작."""
    received: list[GreetEvent] = []
    done = threading.Event()

    class Mod:
        @subscriber
        def on_greet(self, event: GreetEvent) -> None:
            received.append(event)
            done.set()

    mod = Mod()
    spec = get_subscriber_spec(mod.on_greet)
    assert spec is not None

    topic = event_to_topic(spec.event_cls)

    def callback_bytes(payload: bytes) -> None:
        evt = decode_event(spec.event_cls, payload)
        spec.handler(mod, evt)

    handle = transport.subscribe(topic, callback_bytes)
    try:
        time.sleep(0.1)
        evt = GreetEvent(name="alice")
        transport.publish(topic, encode_event(evt))
        assert done.wait(timeout=2.0)
        assert received == [GreetEvent(name="alice")]
    finally:
        handle.undeclare()
