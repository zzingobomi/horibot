from __future__ import annotations

import asyncio
import contextlib
import logging
import socket

from pathlib import Path
from typing import Callable, Generator

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
    def __init__(
        self,
        transport: RawTransport,
        robots: list[RobotInfo],
        host: str = "0.0.0.0",
        port: int = 8000,
        robot_dir: Path | None = None,
        contract_provider: Callable[[], dict] | None = None,
        graph_provider: Callable[[], dict] | None = None,
        default_robot_id: str | None = None,
    ) -> None:
        self._transport = transport
        self._robots = robots
        self._default_robot_id = default_robot_id
        self._host = host
        self._port = port
        self._robot_dir = robot_dir
        self._contract_provider = contract_provider
        self._contract_cache: dict | None = None
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
            allow_origins=["*"],
            allow_methods=["*"],
            allow_headers=["*"],
        )

        @app.get("/robots")
        def get_robots() -> RobotsResponse:
            return RobotsResponse(
                robots=self._robots,
                default=self._default_robot_id
                or (self._robots[0].id if self._robots else None),
            )

        @app.get("/system")
        def get_system() -> SystemMetrics:
            return SystemMetrics(
                cpu_percent=psutil.cpu_percent(interval=None),
                mem_percent=psutil.virtual_memory().percent,
            )

        # TODO: 추후 모듈별 fragment 빌드 산출물 + merge 방식으로 분산 환경에 대비해야 할듯
        @app.get("/contract.json")
        def get_contract() -> dict:
            if self._contract_provider is None:
                raise HTTPException(503, "contract provider 미주입 (gen 전용 endpoint)")
            if self._contract_cache is None:
                try:
                    self._contract_cache = self._contract_provider()
                except Exception as e:
                    raise HTTPException(500, f"contract 직렬화 실패: {e}") from e
            return self._contract_cache

        @app.get("/contract/graph")
        def get_contract_graph() -> dict:
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
            await WsConnection(ws, self._transport).run()

        @app.get("/robots/{robot_id}/camera/stream")
        def camera_stream(robot_id: str) -> StreamingResponse:
            info = next((r for r in self._robots if r.id == robot_id), None)
            if info is None:
                raise HTTPException(404, f"robot {robot_id} 없음")
            if not info.has_camera:
                raise HTTPException(404, f"robot {robot_id} 카메라 없음")
            return StreamingResponse(
                mjpeg_stream(self._transport, robot_id),
                media_type=f"multipart/x-mixed-replace; boundary={BOUNDARY}",
            )

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
        # 소켓을 uvicorn 보다 먼저 직접 bind — uvicorn 은 bind 실패 시 sys.exit(1)
        # (SystemExit) 로 이벤트 루프째 무너뜨려서, caller 의 예외 처리/rollback 이
        # 전부 건너뛰어진다 (부팅 좀비 → 프로세스 종료 불가). 미리 bind 하면 포트
        # 점유가 평범한 예외로 잡히고, port=0 (ephemeral) 도 지원된다.
        try:
            sock = socket.create_server((self._host, self._port), backlog=128)
        except OSError as e:
            raise RuntimeError(
                f"Bridge port {self._port} bind 실패 — 이미 다른 backend 프로세스가 "
                f"떠 있는지 확인 ({e})"
            ) from e
        self._port = sock.getsockname()[1]  # port=0 이면 실제 할당 포트로 갱신

        config = uvicorn.Config(
            self._app,
            host=self._host,
            port=self._port,
            log_level="warning",
            timeout_graceful_shutdown=2,
        )
        self._server = _EmbeddedUvicornServer(config)
        self._serve_task = asyncio.create_task(self._server.serve(sockets=[sock]))

        # uvicorn 이 실제 listen 시작할 때까지 대기 (최대 5s)
        for _ in range(250):
            if self._server.started:
                break
            await asyncio.sleep(0.02)
        else:
            sock.close()
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


class _EmbeddedUvicornServer(uvicorn.Server):
    """임베딩 uvicorn 시그널 비활성화.

    main.py가 종료 신호와 shutdown 순서를 관리하므로,
    uvicorn 자체 signal handler를 막고 should_exit으로만 종료한다.
    """

    @contextlib.contextmanager
    def capture_signals(self) -> Generator[None, None, None]:
        yield
