"""BridgeModule — Boundary Adapter (HTTP/WS ↔ Transport raw relay).

backend_v2.md §16.1 #12 + §8.6 (relay only — domain logic 0).

endpoint: HTTP helper (`/robots` / `/system`) + WS relay (`/ws`, → ws.py) +
MJPEG (`/robots/{id}/camera/stream`, → mjpeg.py).

Bridge 는 raw `RawTransport` 만 받음 (typed ModuleRuntime 아님) — 임의 토픽의
raw bytes 를 그대로 forward 해야 하니 한 계층 아래에서 동작. close /
register_service 권한은 RawTransport 가 차단.
"""

from __future__ import annotations

import asyncio
import logging

from pathlib import Path
from typing import Callable

import psutil
import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
from fastapi.staticfiles import StaticFiles

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
        robot_dir: Path | None = None,
        contract_provider: Callable[[], dict] | None = None,
        graph_provider: Callable[[], dict] | None = None,
    ) -> None:
        self._transport = transport
        self._robots = robots
        self._host = host
        self._port = port
        self._robot_dir = robot_dir  # robot_v2/ — /robot 에 static mount (URDF/mesh)
        # frontend TS gen 용 계약 JSON provider (apps 가 runtime 위에 closure 로 주입).
        # bridge 는 apps/serializer 를 모르는 relay — opaque 콜러블만 호출 (§6.3).
        self._contract_provider = contract_provider
        self._contract_cache: dict | None = None
        # contract graph viewer 용 provider (unfiltered 그래프). 같은 relay 패턴.
        self._graph_provider = graph_provider
        self._graph_cache: dict | None = None

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

        @app.get("/contract.json")
        def get_contract() -> dict:
            # frontend `pnpm gen:types` 가 fetch → contract.ts 조립 (§6.3).
            # 계약은 boot 후 불변 → 최초 1회 계산 후 캐시.
            if self._contract_provider is None:
                raise HTTPException(503, "contract provider 미주입 (gen 전용 endpoint)")
            if self._contract_cache is None:
                try:
                    self._contract_cache = self._contract_provider()
                except Exception as e:  # incomplete-host 등 — 명확한 메시지로 500
                    raise HTTPException(500, f"contract 직렬화 실패: {e}") from e
            return self._contract_cache

        @app.get("/contract/graph")
        def get_contract_graph() -> dict:
            # contract graph viewer (frontend /contract 페이지)가 fetch → React Flow.
            # unfiltered module attribution + wiring. 계약은 boot 후 불변 → 캐시.
            if self._graph_provider is None:
                raise HTTPException(503, "graph provider 미주입 (viewer 전용 endpoint)")
            if self._graph_cache is None:
                try:
                    self._graph_cache = self._graph_provider()
                except Exception as e:
                    raise HTTPException(500, f"contract graph 직렬화 실패: {e}") from e
            return self._graph_cache

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

        # robot_v2/ → /robot static mount (frontend urdf-loader 가 URDF/mesh fetch).
        # mount 는 명시 route 뒤 — "/robots" 등과 prefix 안 겹침.
        if self._robot_dir is not None:
            app.mount(
                "/robot",
                StaticFiles(directory=str(self._robot_dir)),
                name="robot",
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
