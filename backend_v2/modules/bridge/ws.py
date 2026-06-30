"""WebSocket relay — browser ↔ Zenoh raw bytes 중계 (C1b).

backend_v2_modules.md §8.6 (relay only — domain logic 0).

프로토콜 v1:
  browser → Bridge : JSON 텍스트 제어 메시지
    {op:"subscribe", topic}
    {op:"unsubscribe", topic}
    {op:"publish", topic, data}           # data dict → msgpack 감싸 publish
    {op:"service", key, request_id, data, robot_id?}  # ServiceRequest 봉투로 call

  Bridge → browser : binary 프레임
    [u8 ver=1][u8 type][u16 BE key_len][key utf8][payload]
      type=1 topic_data        : key=topic,       payload=Zenoh raw msgpack (그대로)
      type=2 service_response  : key=request_id,  payload=raw 응답 msgpack
      type=3 service_error     : key=request_id,  payload=msgpack {type,message}

Bridge 는 payload schema 를 해석하지 않는다. publish/service 의 data 를 msgpack
으로 감싸고(framework envelope), topic_data 는 raw forward. robot_id 치환은
frontend 책임 — Bridge 는 concrete key 만 받음.
"""

from __future__ import annotations

import asyncio
import json
import logging
import struct
import time
from typing import Any

import msgspec
from fastapi import WebSocket, WebSocketDisconnect

from framework.transport.protocol import Handle, RawTransport, RemoteError

logger = logging.getLogger(__name__)

FRAME_VERSION = 1
FRAME_TOPIC_DATA = 1
FRAME_SERVICE_RESPONSE = 2
FRAME_SERVICE_ERROR = 3

# 느린 클라이언트 메모리 폭증 방지 — bounded queue + latest-wins drop
_SEND_QUEUE_MAX = 256


def encode_frame(ftype: int, key: str, payload: bytes) -> bytes:
    key_b = key.encode("utf-8")
    return struct.pack(">BBH", FRAME_VERSION, ftype, len(key_b)) + key_b + payload


class WsConnection:
    """WS 연결 1개 — 자기 구독 set + send 큐 관리."""

    def __init__(self, ws: WebSocket, transport: RawTransport) -> None:
        self._ws = ws
        self._transport = transport
        self._subs: dict[str, Handle] = {}
        self._queue: asyncio.Queue[bytes] = asyncio.Queue(maxsize=_SEND_QUEUE_MAX)
        self._loop = asyncio.get_running_loop()
        self._closed = False

    async def run(self) -> None:
        await self._ws.accept()
        sender = asyncio.create_task(self._sender())
        try:
            while True:
                text = await self._ws.receive_text()
                await self._handle(text)
        except WebSocketDisconnect:
            pass
        except Exception:
            logger.exception("WsConnection 처리 중 예외")
        finally:
            self._closed = True
            sender.cancel()
            for handle in self._subs.values():
                try:
                    handle.undeclare()
                except Exception:
                    pass
            self._subs.clear()

    # ── send 큐 ────────────────────────────────────────────────

    async def _sender(self) -> None:
        try:
            while True:
                frame = await self._queue.get()
                await self._ws.send_bytes(frame)
        except asyncio.CancelledError:
            pass
        except Exception:
            logger.exception("WS send 실패")

    def _enqueue(self, frame: bytes) -> None:
        # asyncio loop thread 위에서 호출 (call_soon_threadsafe 경유)
        if self._closed:
            return
        if self._queue.full():
            try:
                self._queue.get_nowait()  # latest-wins — 오래된 frame drop
            except asyncio.QueueEmpty:
                pass
        try:
            self._queue.put_nowait(frame)
        except asyncio.QueueFull:
            pass

    # ── op 처리 ────────────────────────────────────────────────

    async def _handle(self, text: str) -> None:
        try:
            msg = json.loads(text)
            op = msg["op"]
        except (json.JSONDecodeError, KeyError, TypeError):
            logger.warning("잘못된 WS 메시지: %.120s", text)
            return

        if op == "subscribe":
            self._subscribe(msg["topic"])
        elif op == "unsubscribe":
            self._unsubscribe(msg["topic"])
        elif op == "publish":
            self._publish(msg["topic"], msg.get("data", {}))
        elif op == "service":
            asyncio.create_task(self._service(msg))
        else:
            logger.warning("알 수 없는 WS op: %r", op)

    def _subscribe(self, topic: str) -> None:
        if topic in self._subs:
            return

        def callback(payload: bytes, _topic: str = topic) -> None:
            frame = encode_frame(FRAME_TOPIC_DATA, _topic, payload)
            self._loop.call_soon_threadsafe(self._enqueue, frame)

        self._subs[topic] = self._transport.subscribe(topic, callback)

    def _unsubscribe(self, topic: str) -> None:
        handle = self._subs.pop(topic, None)
        if handle is not None:
            try:
                handle.undeclare()
            except Exception:
                pass

    def _publish(self, topic: str, data: Any) -> None:
        # data dict → msgpack (framework event wire format). schema 검증 X (relay)
        self._transport.publish(topic, msgspec.msgpack.encode(data))

    async def _service(self, msg: dict) -> None:
        request_id = str(msg.get("request_id", ""))
        key = msg["key"]
        data = msg.get("data", {})
        envelope = {"timestamp": time.time(), "data": data}
        payload = msgspec.msgpack.encode(envelope)
        try:
            res_bytes = await self._transport.call(key, payload)
            frame = encode_frame(FRAME_SERVICE_RESPONSE, request_id, res_bytes)
        except RemoteError as e:
            err = msgspec.msgpack.encode({"type": e.type_name, "message": e.message})
            frame = encode_frame(FRAME_SERVICE_ERROR, request_id, err)
        except TimeoutError as e:
            err = msgspec.msgpack.encode({"type": "TimeoutError", "message": str(e)})
            frame = encode_frame(FRAME_SERVICE_ERROR, request_id, err)
        self._enqueue(frame)
