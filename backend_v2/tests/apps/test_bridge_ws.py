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
    _EVENT_FIFO_MAX,
    _LATEST_WINS_MAX,
    _SERVICE_CHANNEL,
    _SERVICE_MAX,
    WsConnection,
    _channel_maxlen,
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


# ── send 큐 채널 정책 (backpressure) ────────────────────────────────
# 데이터 중요도 구분: stream(최신만) / event(보존) / service(유실 방지) 를
# 키 prefix taxonomy 로 나눔. 채널별 독립 큐라 고rate 토픽이 남을 안 밀어냄.


def test_channel_maxlen_by_key_prefix():
    # stream/* = telemetry → latest-wins(1)
    assert _channel_maxlen("stream/motor/so101_6dof_0/state") == _LATEST_WINS_MAX
    # event/* = 이산 이벤트 → 보존(FIFO)
    assert _channel_maxlen("event/calibration/so101_6dof_0/activated") == _EVENT_FIFO_MAX
    # service 응답 = 유실 방지
    assert _channel_maxlen(_SERVICE_CHANNEL) == _SERVICE_MAX


class _DummyWs:
    pass


class _DummyTransport:
    pass


async def test_enqueue_channel_isolation_and_retention():
    # 실 WS/네트워크 없이 채널 큐 정책만 (get_running_loop 위해 async).
    conn = WsConnection(_DummyWs(), _DummyTransport())  # type: ignore[arg-type]

    # stream 채널 flood → 최신 1개만 (latest-wins)
    for i in range(50):
        conn._enqueue("stream/motor/r/state", bytes([i]))
    assert list(conn._pending["stream/motor/r/state"]) == [bytes([49])]

    # event 채널 flood → 최근 N개 보존 (drop-oldest, 유실은 하되 backlog 유지)
    for i in range(_EVENT_FIFO_MAX + 30):
        conn._enqueue("event/calibration/r/committed", bytes([i % 256]))
    assert len(conn._pending["event/calibration/r/committed"]) == _EVENT_FIFO_MAX

    # 격리 — event flood 가 stream 채널 프레임을 안 밀어냄
    assert list(conn._pending["stream/motor/r/state"]) == [bytes([49])]

    # service 응답 — stream/event 홍수와 무관하게 자기 채널에 쌓임 (유실 방지)
    conn._enqueue(_SERVICE_CHANNEL, b"resp")
    assert list(conn._pending[_SERVICE_CHANNEL]) == [b"resp"]

    # drain 순서 — service 우선
    assert conn._drain_order()[0] == _SERVICE_CHANNEL


# ── shutdown 경로 회귀 가드 ──────────────────────────────────────────
# 이 둘은 프로세스 종료 시에만 나타나던 버그 — 기존 fixture(clean start/stop,
# 실 SIGINT 없음, 단일 프로세스)는 재현 못 함. 각 가드 자체를 결정적으로 고정.


def test_embedded_server_does_not_capture_signals():
    # 임베딩 uvicorn 이 SIGINT 핸들러를 가로채면(handle_exit + 종료 시 raise_signal)
    # asyncio.run 의 _on_sigint 와 충돌 → shutdown 중 KeyboardInterrupt traceback.
    # _EmbeddedUvicornServer.capture_signals 는 no-op 이어야. 되돌리면(기본 Server)
    # 이 test 가 during != before 로 잡는다.
    import signal
    import threading

    import uvicorn

    from modules.bridge.module import _EmbeddedUvicornServer

    async def _app(scope, receive, send):  # noqa: ANN001
        pass

    cfg = uvicorn.Config(app=_app, log_level="warning")
    before = signal.getsignal(signal.SIGINT)
    with _EmbeddedUvicornServer(cfg).capture_signals():
        during = signal.getsignal(signal.SIGINT)
    after = signal.getsignal(signal.SIGINT)
    assert before is during is after  # 핸들러 안 건드림

    # 대조 — 기본 Server 는 handle_exit 로 바꿈 (regression 이 실제로 잡히는지 증명).
    # capture_signals 는 main thread 에서만 핸들러 설치.
    if threading.current_thread() is threading.main_thread():
        with uvicorn.Server(cfg).capture_signals():
            assert signal.getsignal(signal.SIGINT) is not before


async def test_ws_subscribe_callback_tolerates_closed_loop():
    # zenoh 워커 스레드에서 도는 subscribe 콜백은, shutdown 때 루프가 먼저 닫혀도
    # raise 하면 안 된다 ("Event loop is closed" spam 의 원인). 가드 없으면
    # call_soon_threadsafe 가 RuntimeError → 이 test 가 잡는다.
    captured: dict[str, object] = {}

    class _CapTransport:
        def subscribe(self, topic, cb):  # noqa: ANN001
            captured["cb"] = cb
            return _NullHandle()

    class _NullHandle:
        def undeclare(self) -> None:
            pass

    conn = WsConnection(_DummyWs(), _CapTransport())  # type: ignore[arg-type]
    topic = "stream/camera/so101_6dof_0/jpeg"
    conn._subscribe(topic)

    dead = asyncio.new_event_loop()
    dead.close()
    conn._loop = dead  # 종료 창 재현: 콜백은 살아있고 루프는 닫힘

    cb = captured["cb"]
    assert callable(cb)
    cb(b"\x80")  # raise 없이 조용히 무시돼야
    assert topic not in conn._pending  # 닫힌 루프엔 enqueue 안 함
