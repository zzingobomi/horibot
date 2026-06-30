"""Bridge WS relay C1b 검증 — subscribe / publish / service / error 4 경로.

브라우저 대역으로 `websockets` 클라이언트 사용. 반대편(backend publisher /
service)은 같은 ZenohTransport 객체로 세운다 (intra-session pub/sub 은
camera→camera_decoded e2e 에서 이미 확인됨).
"""

from __future__ import annotations

import asyncio
import json
import struct
import time

import msgspec
import pytest
from websockets.asyncio.client import connect

from apps.config import DeploymentConfig, DriverMode, ModuleEntry, load_robots
from apps.resolve import resolve_host_deps
from framework.runtime.app import Runtime
from infra.transport.zenoh import ZenohTransport
from modules.bridge.module import BridgeModule
from modules.bridge.ws import (
    FRAME_SERVICE_ERROR,
    FRAME_SERVICE_RESPONSE,
    FRAME_TOPIC_DATA,
)
from pathlib import Path

_CONFIG_DIR = Path(__file__).resolve().parents[2] / "config"
_LOCAL_CFG = {"mode": "peer", "scouting": {"multicast": {"enabled": False}}}
_PORT = 8078


def _decode_frame(frame: bytes | str) -> tuple[int, str, bytes]:
    assert isinstance(frame, bytes)  # Bridge → browser 는 binary 프레임
    ver, ftype, klen = struct.unpack(">BBH", frame[:4])
    assert ver == 1
    key = frame[4 : 4 + klen].decode("utf-8")
    return ftype, key, frame[4 + klen :]


@pytest.fixture
async def bridge():
    transport = ZenohTransport(_LOCAL_CFG)
    time.sleep(0.05)
    runtime = Runtime(transport)
    robots = load_robots()
    deploy = DeploymentConfig(
        driver_mode=DriverMode.MOCK, modules=[ModuleEntry(name="bridge")]
    )
    deps = resolve_host_deps("bridge", robots, deploy)
    runtime.add_module(BridgeModule, port=_PORT, host="127.0.0.1", **deps)
    await runtime.start()
    yield transport, f"ws://127.0.0.1:{_PORT}/ws"
    await runtime.stop()
    transport.close()


async def test_subscribe_forwards_raw_msgpack(bridge):
    transport, uri = bridge
    topic = "stream/test/foo"
    async with connect(uri) as ws:
        await ws.send(json.dumps({"op": "subscribe", "topic": topic}))
        await asyncio.sleep(0.3)  # 구독 declare 까지
        transport.publish(topic, msgspec.msgpack.encode({"value": 42}))
        frame = await asyncio.wait_for(ws.recv(), timeout=2.0)

    ftype, key, payload = _decode_frame(frame)
    assert ftype == FRAME_TOPIC_DATA
    assert key == topic
    assert msgspec.msgpack.decode(payload) == {"value": 42}


async def test_publish_op_reaches_backend(bridge):
    transport, uri = bridge
    topic = "stream/test/bar"
    received: list[dict] = []
    handle = transport.subscribe(
        topic, lambda p: received.append(msgspec.msgpack.decode(p))
    )
    try:
        async with connect(uri) as ws:
            await ws.send(
                json.dumps({"op": "publish", "topic": topic, "data": {"cmd": "go"}})
            )
            await asyncio.sleep(0.3)
    finally:
        handle.undeclare()

    assert {"cmd": "go"} in received


async def test_service_op_relays_response(bridge):
    transport, uri = bridge
    key = "srv/test/echo"

    def handler(req_bytes: bytes) -> bytes:
        req = msgspec.msgpack.decode(req_bytes)  # {timestamp, data}
        return msgspec.msgpack.encode(
            {"timestamp": time.time(), "data": {"echo": req["data"]}}
        )

    svc = transport.register_service(key, handler)
    try:
        async with connect(uri) as ws:
            await ws.send(
                json.dumps(
                    {"op": "service", "key": key, "request_id": "r1", "data": {"x": 1}}
                )
            )
            frame = await asyncio.wait_for(ws.recv(), timeout=2.0)
    finally:
        svc.undeclare()

    ftype, req_id, payload = _decode_frame(frame)
    assert ftype == FRAME_SERVICE_RESPONSE
    assert req_id == "r1"
    assert msgspec.msgpack.decode(payload)["data"] == {"echo": {"x": 1}}


async def test_service_error_relays_error_frame(bridge):
    transport, uri = bridge
    key = "srv/test/boom"

    def handler(req_bytes: bytes) -> bytes:
        raise ValueError("의도된 실패")

    svc = transport.register_service(key, handler)
    try:
        async with connect(uri) as ws:
            await ws.send(
                json.dumps(
                    {"op": "service", "key": key, "request_id": "r2", "data": {}}
                )
            )
            frame = await asyncio.wait_for(ws.recv(), timeout=2.0)
    finally:
        svc.undeclare()

    ftype, req_id, payload = _decode_frame(frame)
    assert ftype == FRAME_SERVICE_ERROR
    assert req_id == "r2"
    info = msgspec.msgpack.decode(payload)
    assert info["type"] == "ValueError"
    assert "의도된 실패" in info["message"]
