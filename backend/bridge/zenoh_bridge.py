import json
import logging
import asyncio
import os
from contextlib import asynccontextmanager
import time
from pathlib import Path

import zenoh
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.openapi.utils import get_openapi
from fastapi.responses import StreamingResponse
from fastapi.staticfiles import StaticFiles
from typing import Any, cast

from pydantic import create_model

from api_contract import (
    PUBLIC_BINARY_TOPICS,
    PUBLIC_TOPICS,
    all_referenced_models,
    to_x_contract,
)
from core.transport.zenoh_session import ZenohSession
from core.transport.topic_map import Topic
from bridge.calibration_router import calibration_router
from bridge.client_stream import ConnectionManager

logger = logging.getLogger(__name__)

# ─── 메시지 타입 ─────────────────────────────────────────


class MsgType:
    SUBSCRIBE = "subscribe"
    UNSUBSCRIBE = "unsubscribe"
    PUBLISH = "publish"
    SERVICE = "service"
    TOPIC_DATA = "topic_data"
    SERVICE_RESPONSE = "service_response"
    ERROR = "error"


# 바이너리 프레임 포맷:
#   [u8 version=1][u8 type=1(topic_data)][u16 BE topic_len][topic UTF-8][payload]
_BIN_VERSION = 1
_BIN_TYPE_TOPIC_DATA = 1


def _encode_binary_topic(topic: str, payload: bytes) -> bytes:
    name = topic.encode("utf-8")
    if len(name) > 0xFFFF:
        raise ValueError(f"topic 이름이 너무 깁니다: {topic}")
    header = bytes([_BIN_VERSION, _BIN_TYPE_TOPIC_DATA])
    header += len(name).to_bytes(2, "big")
    header += name
    return header + payload


# ─── 이벤트 루프 ─────────────────────────────────────────

_loop: asyncio.AbstractEventLoop | None = None
_camera_queues: set[asyncio.Queue] = set()


@asynccontextmanager
async def lifespan(app: FastAPI):
    global _loop
    _loop = asyncio.get_running_loop()
    logger.info("이벤트 루프 설정 완료")
    yield
    for sub in _zenoh_subs:
        sub.undeclare()
    logger.info("Zenoh 구독 정리 완료")


app = FastAPI(title="OMX Bridge", lifespan=lifespan)

ROBOT_DIR = Path(__file__).parents[2] / "robot"
app.mount("/robot", StaticFiles(directory=str(ROBOT_DIR)), name="robot")

app.include_router(calibration_router)


# ─── OpenAPI schema export — auto from api_contract ──────
# api_contract.PUBLIC_TOPICS / PUBLIC_SERVICES 가 참조하는 모든 모델을 한
# class 의 optional 필드로 묶어 `/openapi.json::components/schemas` 에 자동
# 등재. Nested model (예: MotorJoint) 은 referenced 만 해도 FastAPI 가 자동
# 등록 → 본 클래스는 top-level publish/service payload 만 명시.
#
# 새 모델 추가 = `api_contract.py` 에 entry 추가 (`OpenApiSchemaRegistry` 안
# 건드림).

_referenced_models = sorted(all_referenced_models(), key=lambda m: m.__name__)
_registry_fields = {m.__name__: (m | None, None) for m in _referenced_models}
# pydantic v2 `create_model` overload 시그니처가 `**field_definitions` 형태라
# pyright 가 일반 dict unpack 을 reserved kwargs 와 헷갈림 — runtime 정상.
OpenApiSchemaRegistry = cast(Any, create_model)(
    "OpenApiSchemaRegistry", **_registry_fields
)
OpenApiSchemaRegistry.__doc__ = (
    "OpenAPI schema export only — auto-built from api_contract."
)


@app.get(
    "/_schemas",
    response_model=OpenApiSchemaRegistry,
    include_in_schema=True,
    summary="Type registry (OpenAPI export only)",
)
def _schemas():
    """OpenAPI schema export only — clients 가 호출 X.

    Used by `frontend/pnpm gen:types` to emit TS interfaces from Pydantic models.
    """
    return OpenApiSchemaRegistry()


# ─── OpenAPI x-contract extension ────────────────────────
# api_contract 의 (topic_key → schema_name) / (service_key → req,res) 매핑을
# `/openapi.json` 의 vendor extension `x-contract` 키로 인라인. frontend
# `gen-contract.ts` 가 같은 JSON 의 이 키를 읽어 `contract.ts` emit.
#
# OpenAPI spec 상 `x-*` 키는 vendor extension — `openapi-typescript` 는 무시.


def custom_openapi() -> dict:
    if app.openapi_schema:
        return app.openapi_schema
    schema = get_openapi(
        title=app.title,
        version="1.0.0",
        routes=app.routes,
    )
    schema["x-contract"] = to_x_contract()
    app.openapi_schema = schema
    return schema


app.openapi = custom_openapi  # type: ignore[assignment]

_extra_origins = [
    o.strip()
    for o in os.getenv("BRIDGE_CORS_ORIGINS", "").split(",")
    if o.strip()
]

app.add_middleware(
    CORSMiddleware,
    allow_origin_regex=(
        r"^https?://("
        r"localhost|127\.0\.0\.1|"
        r"10\.\d+\.\d+\.\d+|"
        r"192\.168\.\d+\.\d+|"
        r"172\.(1[6-9]|2\d|3[01])\.\d+\.\d+"
        r")(:\d+)?$"
    ),
    allow_origins=_extra_origins,
    allow_methods=["*"],
    allow_headers=["*"],
)


manager = ConnectionManager()


# ─── Zenoh → asyncio 브릿지 ──────────────────────────────────


def _zenoh_callback(topic: str, data: dict) -> None:
    if _loop is None or _loop.is_closed():
        return
    text = json.dumps(
        {
            "type": MsgType.TOPIC_DATA,
            "topic": topic,
            "data": data,
        }
    )
    _loop.call_soon_threadsafe(manager.fanout, topic, text)


def _zenoh_callback_bytes(topic: str, payload: bytes) -> None:
    if _loop is None or _loop.is_closed():
        return
    frame = _encode_binary_topic(topic, payload)
    _loop.call_soon_threadsafe(manager.fanout, topic, frame)


def _put_frame(q: asyncio.Queue, frame: bytes) -> None:
    try:
        q.put_nowait(frame)
    except asyncio.QueueFull:
        pass  # 느린 클라이언트는 프레임 드롭


def _camera_callback(jpeg_bytes: bytes) -> None:
    if _loop and not _loop.is_closed():
        for q in _camera_queues.copy():
            _loop.call_soon_threadsafe(_put_frame, q, jpeg_bytes)


# JSON 토픽: api_contract.PUBLIC_TOPICS 에서 자동 도출.
# Binary 토픽: PUBLIC_BINARY_TOPICS (별도 raw frame 라우팅).
_ALWAYS_SUBSCRIBE = list(PUBLIC_TOPICS)
_ALWAYS_SUBSCRIBE_BINARY = list(PUBLIC_BINARY_TOPICS)

_zenoh_subs: list[zenoh.Subscriber] = []


def setup_zenoh_subscribers() -> None:
    session = ZenohSession.get()

    for topic in _ALWAYS_SUBSCRIBE:

        def make_handler(tp: str):
            def handler(sample: zenoh.Sample):
                try:
                    data = json.loads(sample.payload.to_bytes())
                    _zenoh_callback(tp, data)
                except Exception as e:
                    logger.error(f"bridge subscriber 오류 ({tp}): {e}")

            return handler

        _zenoh_subs.append(session.declare_subscriber(
            topic, make_handler(topic)))

    # 카메라는 raw bytes 로 MJPEG `/camera/stream` HTTP 라우트로 별도 송출.
    # contract 의 PUBLIC_BINARY_TOPICS 에 없는 (별도 라우트) 자리.
    def camera_handler(sample: zenoh.Sample):
        _camera_callback(sample.payload.to_bytes())

    _zenoh_subs.append(
        session.declare_subscriber(Topic.CAMERA_STREAM_RAW, camera_handler)
    )

    # contract 의 binary 토픽은 WS binary frame 으로 직송.
    for topic in _ALWAYS_SUBSCRIBE_BINARY:

        def make_binary_handler(tp: str):
            def handler(sample: zenoh.Sample):
                _zenoh_callback_bytes(tp, sample.payload.to_bytes())

            return handler

        _zenoh_subs.append(
            session.declare_subscriber(topic, make_binary_handler(topic))
        )

    logger.info("Zenoh 구독 설정 완료")


# ─── WebSocket 엔드포인트 ─────────────────────────────────────


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    logger.info(f"WebSocket 연결: {ws.client}")

    try:
        while True:
            raw = await ws.receive_text()
            await _handle_message(ws, json.loads(raw))
    except WebSocketDisconnect:
        logger.info(f"WebSocket 연결 종료: {ws.client}")
    except Exception as e:
        logger.error(f"WebSocket 오류: {e}")
    finally:
        manager.remove_client(ws)


# ─── MJPEG HTTP 스트림 ───────────────────────────────────────


@app.get("/camera/stream")
async def camera_stream():
    q: asyncio.Queue = asyncio.Queue(maxsize=2)
    _camera_queues.add(q)

    async def generate():
        try:
            while True:
                frame = await q.get()
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
        finally:
            _camera_queues.discard(q)

    return StreamingResponse(
        generate(), media_type="multipart/x-mixed-replace; boundary=frame"
    )


# ─── 메시지 라우팅 ───────────────────────────────────────────


async def _handle_message(ws: WebSocket, msg: dict) -> None:
    msg_type = msg.get("type")
    session = ZenohSession.get()

    if msg_type == MsgType.SUBSCRIBE:
        manager.subscribe(ws, msg.get("topic", ""))

    elif msg_type == MsgType.UNSUBSCRIBE:
        manager.unsubscribe(ws, msg.get("topic", ""))

    elif msg_type == MsgType.PUBLISH:
        topic = msg.get("topic", "")
        payload = json.dumps(msg.get("data", {})).encode()
        session.put(topic, payload)

    elif msg_type == MsgType.SERVICE:
        key = msg.get("key", "")
        request_id = msg.get("request_id", "")
        data = msg.get("data", {})
        timeout = float(msg.get("timeout") or 5.0)

        req_payload = json.dumps(
            {
                "timestamp": time.time(),
                "data": data,
            }
        ).encode()

        try:
            replies = session.get(key, payload=req_payload, timeout=timeout)
            res = None
            for reply in replies:
                if reply.ok is not None:
                    res = json.loads(reply.ok.payload.to_bytes())
                else:
                    err = reply.err
                    err_msg = (
                        err.payload.to_string()
                        if err is not None and err.payload is not None
                        else "서비스 err reply"
                    )
                    res = {"success": False, "message": err_msg, "data": {}}
                break

            await ws.send_text(
                json.dumps(
                    {
                        "type": MsgType.SERVICE_RESPONSE,
                        "request_id": request_id,
                        **(
                            res
                            or {"success": False, "message": "응답 없음", "data": {}}
                        ),
                    }
                )
            )
        except Exception as e:
            await ws.send_text(
                json.dumps(
                    {
                        "type": MsgType.SERVICE_RESPONSE,
                        "request_id": request_id,
                        "success": False,
                        "message": str(e),
                        "data": {},
                    }
                )
            )

    else:
        logger.warning(f"알 수 없는 메시지 타입: {msg_type}")
