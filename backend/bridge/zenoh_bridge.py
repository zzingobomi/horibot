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
from fastapi.responses import StreamingResponse
from fastapi.staticfiles import StaticFiles

from core.zenoh_session import ZenohSession
from core.topic_map import Topic
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


_ALWAYS_SUBSCRIBE = [
    Topic.MOTOR_STATE_JOINT,
    Topic.CAMERA_STATE_STATUS,
    Topic.CALIB_HANDEYE_PREVIEW,
    Topic.SYSTEM_HEARTBEAT,
    Topic.SYSTEM_LOG,
    Topic.MOTION_STATE_TRAJ,
    Topic.TASK_STATE,
    Topic.TASK_TREE,
    Topic.TASK_STEP_RESULT,
    Topic.DETECTOR_STATE,
    Topic.PERCEPTION_GROUNDED_STATE,
    Topic.POINTCLOUD_STATE,
]

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

    # 카메라는 raw bytes로 수신
    def camera_handler(sample: zenoh.Sample):
        _camera_callback(sample.payload.to_bytes())

    _zenoh_subs.append(
        session.declare_subscriber(Topic.CAMERA_STREAM_RAW, camera_handler)
    )

    # 포인트클라우드는 바이너리 WS 프레임으로 직송
    def pointcloud_handler(sample: zenoh.Sample):
        _zenoh_callback_bytes(Topic.POINTCLOUD_STREAM,
                              sample.payload.to_bytes())

    _zenoh_subs.append(
        session.declare_subscriber(Topic.POINTCLOUD_STREAM, pointcloud_handler)
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
                "request_id": request_id,
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
