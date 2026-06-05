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
from bridge.schemas import (
    BasePoseSchema,
    RobotInfo,
    RobotsListResponse,
    SystemMetrics,
    TasksResponse,
)

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
# robot 별 카메라 stream 큐 — N>=2 시 robot 마다 별도 MJPEG endpoint.
_camera_queues_by_robot: dict[str, set[asyncio.Queue]] = {}


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


# ─── /robots — robots.yaml SSOT 노출 ────────────────────────
# multi_robot_phase2_frontend.md §2 — frontend 가 fetch 해서 메뉴 / WorldScene
# enumeration 에 사용. robots.yaml 변경 시 backend 재시작 후 자동 반영.


@app.get(
    "/system",
    response_model=SystemMetrics,
    summary="Host system metrics (CPU / Mem / Zenoh peers)",
)
def system_metrics() -> SystemMetrics:
    """Dashboard overview source. psutil 로 CPU / Mem, ZenohSession 으로 peer
    카운트. cpu_percent(interval=0.1) 는 100ms blocking — FastAPI sync handler
    가 thread pool 에서 도니까 event loop 안 막음.
    """
    import psutil

    cpu_pct = psutil.cpu_percent(interval=0.1)
    vm = psutil.virtual_memory()

    routers = 0
    peers = 0
    try:
        info = ZenohSession.get().info
        routers = sum(1 for _ in info.routers_zid())
        peers = sum(1 for _ in info.peers_zid())
    except Exception as e:
        logger.warning("zenoh peer info 조회 실패: %s", e)

    return SystemMetrics(
        cpu_pct=round(cpu_pct, 1),
        mem_used_mb=round(vm.used / (1024 * 1024), 1),
        mem_total_mb=round(vm.total / (1024 * 1024), 1),
        mem_pct=round(vm.percent, 1),
        zenoh_routers=routers,
        zenoh_peers=peers,
    )


@app.get(
    "/tasks",
    response_model=TasksResponse,
    summary="Registered task factories from TASK_REGISTRY",
)
def list_tasks() -> TasksResponse:
    """task_node 의 TASK_REGISTRY enumerate — frontend Sidebar / TasksPage 의
    enumeration source. lazy import 로 task_node 의 무거운 deps (LLM /
    detector chain) 부팅 시 끌고 오지 않음.
    """
    from nodes.task_node import TASK_REGISTRY

    return TasksResponse(tasks=sorted(TASK_REGISTRY.keys()))


@app.get(
    "/robots",
    response_model=RobotsListResponse,
    summary="Registered robots from robots.yaml",
)
def list_robots() -> RobotsListResponse:
    from core.robot.robot_registry import RobotRegistry

    reg = RobotRegistry()
    robots: list[RobotInfo] = []
    for rid in reg.list_robots():
        cfg = reg.get(rid)
        robots.append(
            RobotInfo(
                id=cfg.robot_id,
                type=cfg.robot_type,
                enabled=cfg.enabled,
                capabilities=list(cfg.capabilities),
                base_pose=BasePoseSchema(
                    x=cfg.base_pose.x,
                    y=cfg.base_pose.y,
                    z=cfg.base_pose.z,
                    yaw_deg=cfg.base_pose.yaw_deg,
                ),
                urdf_url=f"/robot/{cfg.robot_type}/urdf/{cfg.robot_type}.urdf",
            )
        )
    # default = enabled 1개일 때만 반환. 0개 / 2개 이상이면 null.
    try:
        default_id: str | None = reg.default_robot_id()
    except RuntimeError:
        default_id = None
    return RobotsListResponse(robots=robots, default=default_id)


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


def _camera_callback(robot_id: str, jpeg_bytes: bytes) -> None:
    if _loop and not _loop.is_closed():
        for q in _camera_queues_by_robot.get(robot_id, set()).copy():
            _loop.call_soon_threadsafe(_put_frame, q, jpeg_bytes)


# JSON / binary 토픽 template — api_contract.PUBLIC_TOPICS / PUBLIC_BINARY_TOPICS.
# robot-scoped template (`horibot/{robot_id}/...`) 는 setup_zenoh_subscribers 가
# robots.yaml enumerate 후 robot 마다 expand 해서 구독.
_ALWAYS_SUBSCRIBE = list(PUBLIC_TOPICS)
_ALWAYS_SUBSCRIBE_BINARY = list(PUBLIC_BINARY_TOPICS)

_zenoh_subs: list[zenoh.Subscriber] = []


def _expand_for_robots(template: str, robot_ids: list[str]) -> list[str]:
    """robot-scoped template 은 robot 마다 expand, global 은 그대로 1개."""
    if "{robot_id}" not in template:
        return [template]
    return [template.format(robot_id=rid) for rid in robot_ids]


def setup_zenoh_subscribers() -> None:
    from core.robot.robot_registry import RobotRegistry

    session = ZenohSession.get()
    robot_ids = RobotRegistry().list_robots()

    for template in _ALWAYS_SUBSCRIBE:
        for topic in _expand_for_robots(template, robot_ids):

            def make_handler(tp: str):
                def handler(sample: zenoh.Sample):
                    try:
                        data = json.loads(sample.payload.to_bytes())
                        _zenoh_callback(tp, data)
                    except Exception as e:
                        logger.error(f"bridge subscriber 오류 ({tp}): {e}")

                return handler

            _zenoh_subs.append(
                session.declare_subscriber(topic, make_handler(topic))
            )

    # 카메라 raw bytes — MJPEG `/robots/<robot_id>/camera/stream` HTTP 라우트
    # 로 별도 송출. contract 의 PUBLIC_BINARY_TOPICS 에 없음 (별도 라우트).
    for rid in robot_ids:
        topic = Topic.CAMERA_STREAM_RAW.format(robot_id=rid)

        def make_camera_handler(_rid: str):
            def handler(sample: zenoh.Sample):
                _camera_callback(_rid, sample.payload.to_bytes())

            return handler

        _zenoh_subs.append(
            session.declare_subscriber(topic, make_camera_handler(rid))
        )

    # contract 의 binary 토픽은 WS binary frame 으로 직송.
    for template in _ALWAYS_SUBSCRIBE_BINARY:
        for topic in _expand_for_robots(template, robot_ids):

            def make_binary_handler(tp: str):
                def handler(sample: zenoh.Sample):
                    _zenoh_callback_bytes(tp, sample.payload.to_bytes())

                return handler

            _zenoh_subs.append(
                session.declare_subscriber(topic, make_binary_handler(topic))
            )

    logger.info("Zenoh 구독 설정 완료 (robots=%s)", robot_ids)


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


@app.get("/robots/{robot_id}/camera/stream")
async def camera_stream(robot_id: str):
    """robot 별 MJPEG stream. URL = frontend `/robots/<id>` 라우팅과 동형.

    multi_robot_phase2_frontend.md §1 결정 — `/camera/stream` → `/robots/<id>/...`
    로 robot-scoped.
    """
    q: asyncio.Queue = asyncio.Queue(maxsize=2)
    _camera_queues_by_robot.setdefault(robot_id, set()).add(q)

    async def generate():
        try:
            while True:
                frame = await q.get()
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
        finally:
            _camera_queues_by_robot.get(robot_id, set()).discard(q)

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
