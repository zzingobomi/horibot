"""WebSocket relay — browser ↔ Zenoh raw bytes 중계.

프로토콜 v1:
  browser → Bridge : JSON 텍스트 제어 메시지
    {op:"subscribe", topic}
    {op:"unsubscribe", topic}
    {op:"publish", topic, data}           # data dict → msgpack으로 인코딩 후 publish
    {op:"service", key, request_id, data} # ServiceRequest 봉투로 call

  Bridge → browser : binary 프레임
    [u8 ver=1][u8 type][u16 BE key_len][key utf8][payload]
      type=1 topic_data        : key=topic,      payload=Zenoh raw msgpack (그대로 전달)
      type=2 service_response  : key=request_id, payload=raw 응답 msgpack
      type=3 service_error     : key=request_id, payload=msgpack {type, message}
"""

from __future__ import annotations

import asyncio
import json
import logging
import struct
import time
from collections import deque
from typing import Any

import msgspec
from fastapi import WebSocket, WebSocketDisconnect

from framework.transport.protocol import Handle, RawTransport, RemoteError

logger = logging.getLogger(__name__)

FRAME_VERSION = 1
FRAME_TOPIC_DATA = 1
FRAME_SERVICE_RESPONSE = 2
FRAME_SERVICE_ERROR = 3

# ── send 큐 정책 (채널별) ──────────────────────────────────────────
#
# 느린 클라이언트로 큐가 밀릴 때 무엇을 버릴지 = 데이터 중요도에 따라 다름.
# 연결당 단일 큐면 두 문제가 생겨 → 채널(토픽/service)별 독립 큐로 분리:
#   - 고rate 토픽(pointcloud 등)이 저rate 토픽 프레임을 밀어냄 (토픽 격리 없음)
#   - service 응답이 스트림 홍수에 유실됨 → 프론트 호출 타임아웃
#
# 정책은 키 taxonomy(계약 SSOT: stream/event/srv prefix)로 결정 — metadata 불필요:
#
#   - stream/*  = telemetry (최신값만 의미 있음)
#                 → maxlen 1 (latest-wins)
#
#   - event/*   = 이산 이벤트 (각 발생이 의미 있음)
#                 → maxlen 128 (bounded FIFO, backlog retention)
#
#   - srv/*     = request/reply (request_id 기반 correlation)
#                 → 큐 정책보다 응답 매칭이 핵심, 유실 시 client timeout 처리
#
# 주의:
# bounded FIFO는 backlog retention이지 delivery guarantee(유실 0 보장)가 아님.
# at-least-once semantics가 필요한 경우 ACK + replay 기반 프로토콜 tier 추가 필요
_LATEST_WINS_MAX = 1
_EVENT_FIFO_MAX = 128
_SERVICE_MAX = 256
_SERVICE_CHANNEL = "\x00service"  # topic 키와 충돌 없는 sentinel


def _channel_maxlen(channel: str) -> int:
    if channel == _SERVICE_CHANNEL:
        return _SERVICE_MAX
    return _EVENT_FIFO_MAX if channel.split("/", 1)[0] == "event" else _LATEST_WINS_MAX


def encode_frame(ftype: int, key: str, payload: bytes) -> bytes:
    key_b = key.encode("utf-8")
    return struct.pack(">BBH", FRAME_VERSION, ftype, len(key_b)) + key_b + payload


class WsConnection:
    def __init__(self, ws: WebSocket, transport: RawTransport) -> None:
        self._ws = ws
        self._transport = transport
        self._subs: dict[str, Handle] = {}
        self._pending: dict[str, deque[bytes]] = {}
        self._wake = asyncio.Event()
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
                await self._wake.wait()
                self._wake.clear()
                # service 응답을 먼저 전송한다 (RPC 특성상 timeout 방지 위해 최우선).
                # 이후 topic(stream/event) 큐를 한 번에 모두 꺼내 전송한다.
                #
                # send 중에도 새로운 데이터가 들어올 수 있으므로 큐를 완전히 비우고 끝내지 않고,
                # drain 도중 신규 입력이 발생하면 _wake 플래그를 다시 설정하여 다음 send cycle에서 재처리한다.
                for channel in self._drain_order():
                    dq = self._pending.get(channel)
                    if not dq:
                        continue
                    frames = list(dq)
                    dq.clear()
                    for frame in frames:
                        await self._ws.send_bytes(frame)
        except asyncio.CancelledError:
            pass
        except Exception:
            logger.exception("WS send 실패")

    def _drain_order(self) -> list[str]:
        others = [c for c in self._pending if c != _SERVICE_CHANNEL]
        return [_SERVICE_CHANNEL, *others]

    def _enqueue(self, channel: str, frame: bytes) -> None:
        # 채널별 독립 큐(stream/event/service) 사용 — overflow 시 해당 채널 내에서만 oldest drop.
        # (stream=latest-wins / event=bounded retention / service=RPC queue)
        if self._closed:
            return
        dq = self._pending.get(channel)
        if dq is None:
            dq = deque(maxlen=_channel_maxlen(channel))
            self._pending[channel] = dq
        dq.append(frame)
        self._wake.set()

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
            # zenoh 워커 스레드에서 실행 — shutdown 시 루프가 먼저 닫히면
            # call_soon_threadsafe 가 RuntimeError. 프레임워크 subscriber 와
            # 동일하게 닫힌 루프 가드 (app.py _register_subscriber 참고).
            if self._closed or self._loop.is_closed():
                return
            frame = encode_frame(FRAME_TOPIC_DATA, _topic, payload)
            try:
                self._loop.call_soon_threadsafe(self._enqueue, _topic, frame)
            except RuntimeError:
                pass  # check 이후 루프가 닫힌 teardown 창 — 무시

        self._subs[topic] = self._transport.subscribe(topic, callback)

    def _unsubscribe(self, topic: str) -> None:
        handle = self._subs.pop(topic, None)
        if handle is not None:
            try:
                handle.undeclare()
            except Exception:
                pass

    def _publish(self, topic: str, data: Any) -> None:
        self._transport.publish(topic, msgspec.msgpack.encode(data))

    async def _service(self, msg: dict) -> None:
        request_id = str(msg.get("request_id", ""))
        key = msg["key"]
        data = msg.get("data", {})
        # 호출자 지정 timeout (초) — 장시간 서비스(TSDF build 등)가 bridge 기본
        # 5s 에 잘리지 않게 wire 로 전파. 비정상 값은 clamp (안전 상한 600s).
        try:
            timeout_s = float(msg.get("timeout_s") or 5.0)
        except (TypeError, ValueError):
            timeout_s = 5.0
        timeout_s = max(0.1, min(600.0, timeout_s))
        envelope = {"timestamp": time.time(), "data": data}
        payload = msgspec.msgpack.encode(envelope)
        try:
            res_bytes = await self._transport.call(key, payload, timeout=timeout_s)
            frame = encode_frame(FRAME_SERVICE_RESPONSE, request_id, res_bytes)
        except RemoteError as e:
            err = msgspec.msgpack.encode({"type": e.type_name, "message": e.message})
            frame = encode_frame(FRAME_SERVICE_ERROR, request_id, err)
        except TimeoutError as e:
            err = msgspec.msgpack.encode({"type": "TimeoutError", "message": str(e)})
            frame = encode_frame(FRAME_SERVICE_ERROR, request_id, err)

        self._enqueue(_SERVICE_CHANNEL, frame)
