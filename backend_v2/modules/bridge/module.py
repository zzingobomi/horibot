"""BridgeModule — Boundary Adapter (HTTP/WS ↔ Transport raw relay).

backend_v2_modules.md §1.1 #12 + §8.6 (relay only — domain logic 0).

endpoint: HTTP helper (`/robots` / `/system`) + WS relay (`/ws`, → ws.py) +
MJPEG (`/robots/{id}/camera/stream`, → mjpeg.py).

Bridge 는 raw `RawTransport` 만 받음 (typed ModuleRuntime 아님) — 임의 토픽의
raw bytes 를 그대로 forward 해야 하니 한 계층 아래에서 동작. close /
register_service 권한은 RawTransport 가 차단.
"""

from __future__ import annotations

import asyncio
import logging

import psutil
import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse

from framework.transport.protocol import RawTransport

from .contract import RobotInfo, RobotsResponse, SystemMetrics
from .mjpeg import BOUNDARY, mjpeg_stream
from .ws import WsConnection

logger = logging.getLogger(__name__)


class BridgeModule:
    """host-level (robot-agnostic) Boundary Module. browser ↔ Zenoh gateway."""

    def __init__(
        self,
        transport: RawTransport,
        robots: list[RobotInfo],
        host: str = "0.0.0.0",
        port: int = 8000,
    ) -> None:
        self._transport = transport
        self._robots = robots
        self._host = host
        self._port = port

        self._app = self._build_app()
        self._server: uvicorn.Server | None = None
        self._serve_task: asyncio.Task[None] | None = None

    # ── FastAPI app ───────────────────────────────────────────

    def _build_app(self) -> FastAPI:
        app = FastAPI(title="horibot bridge")
        app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],  # dev — frontend vite :5173 등. 운영 시 좁힐 자리
            allow_methods=["*"],
            allow_headers=["*"],
        )

        @app.get("/robots")
        def get_robots() -> RobotsResponse:
            # §9.1 — application 의 RobotConfig list read-only view relay
            return RobotsResponse(
                robots=self._robots,
                default=self._robots[0].id if self._robots else None,
            )

        @app.get("/system")
        def get_system() -> SystemMetrics:
            # §9 — framework metric helper. domain 무관
            return SystemMetrics(
                cpu_percent=psutil.cpu_percent(interval=None),
                mem_percent=psutil.virtual_memory().percent,
            )

        @app.websocket("/ws")
        async def ws_endpoint(ws: WebSocket) -> None:
            # browser ↔ Zenoh raw relay. uvicorn loop = runtime loop (동일)
            await WsConnection(ws, self._transport).run()

        @app.get("/robots/{robot_id}/camera/stream")
        def camera_stream(robot_id: str) -> StreamingResponse:
            # rgbd capability robot 만 — 없으면 연결 매달리지 않게 404 (§7.5)
            info = next((r for r in self._robots if r.id == robot_id), None)
            if info is None:
                raise HTTPException(404, f"robot {robot_id} 없음")
            if "rgbd" not in info.capabilities:
                raise HTTPException(404, f"robot {robot_id} rgbd capability 없음")
            return StreamingResponse(
                mjpeg_stream(self._transport, robot_id),
                media_type=f"multipart/x-mixed-replace; boundary={BOUNDARY}",
            )

        return app

    @property
    def app(self) -> FastAPI:
        return self._app

    @property
    def port(self) -> int:
        return self._port

    # ── lifecycle ─────────────────────────────────────────────

    async def start(self) -> None:
        config = uvicorn.Config(
            self._app, host=self._host, port=self._port, log_level="warning"
        )
        self._server = uvicorn.Server(config)
        self._serve_task = asyncio.create_task(self._server.serve())

        # uvicorn 이 실제 listen 시작할 때까지 대기 (test 가 곧장 GET 하니 race 방지)
        for _ in range(250):  # ~5s
            if self._server.started:
                break
            await asyncio.sleep(0.02)
        else:
            raise RuntimeError(f"Bridge uvicorn 시작 실패 port={self._port}")
        logger.info("Bridge serving http://%s:%d", self._host, self._port)

    async def stop(self) -> None:
        server = self._server
        if server is not None:
            server.should_exit = True
        task = self._serve_task
        if task is not None:
            try:
                await task
            except asyncio.CancelledError:
                pass
            self._serve_task = None
        self._server = None
