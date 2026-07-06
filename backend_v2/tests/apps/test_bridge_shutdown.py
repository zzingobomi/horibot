"""Bridge graceful shutdown 회귀 가드.

실 증상 (2026-07-06, host=pc): 브라우저가 WS(/ws) + MJPEG 를 열어둔 채 Ctrl+C →
"Runtime stopping" 직후 무한 행. root cause = uvicorn `_wait_tasks_to_complete` 가
열린 연결이 있는 한 무한 대기 (`timeout_graceful_shutdown` 기본 None) + 임베딩이라
force_exit 시그널 경로 없음. fix = timeout_graceful_shutdown 상한.

이 테스트는 그 시나리오를 그대로 재현: WS + MJPEG 연결을 열어둔 채 runtime.stop()
이 bounded 시간 안에 완료되는지. fix revert 시 wait_for TimeoutError 로 잡힌다.
"""

from __future__ import annotations

import asyncio
import contextlib
import json
import time

import httpx
import pytest
from websockets.asyncio.client import connect

from apps.config import DeploymentConfig, DriverMode, ModuleEntry, load_robots
from apps.resolve import resolve_host_deps
from framework.runtime.app import Runtime
from infra.transport.zenoh import ZenohTransport
from modules.bridge.module import BridgeModule
from modules.camera.drivers.mock import MockCameraDriver
from modules.camera.module import CameraDriverModule

_LOCAL_CFG = {"mode": "peer", "scouting": {"multicast": {"enabled": False}}}
_PORT = 8081
_SO101 = "so101_6dof_0"


@pytest.fixture
async def booted():
    transport = ZenohTransport(_LOCAL_CFG)
    runtime = Runtime(transport)
    robots = load_robots()
    deploy = DeploymentConfig(
        driver_mode=DriverMode.MOCK, modules=[ModuleEntry(name="bridge")]
    )
    deps = resolve_host_deps("bridge", robots, deploy)
    runtime.add_module(BridgeModule, port=_PORT, host="127.0.0.1", **deps)
    runtime.add_module(
        CameraDriverModule, robot_id=_SO101, driver=MockCameraDriver(has_depth=True)
    )
    await runtime.start()
    yield runtime
    await runtime.stop()  # 테스트가 이미 stop 했으면 no-op (_started guard)
    transport.close()


async def test_stop_completes_with_open_ws_and_mjpeg(booted: Runtime):
    ws_uri = f"ws://127.0.0.1:{_PORT}/ws"
    mjpeg_url = f"http://127.0.0.1:{_PORT}/robots/{_SO101}/camera/stream"

    client = httpx.AsyncClient(timeout=10.0)
    stream_ctx = client.stream("GET", mjpeg_url)
    ws = await connect(ws_uri)
    try:
        # 두 연결 모두 실제 active 상태로 만든다 (브라우저 시나리오 재현).
        await ws.send(
            json.dumps({"op": "subscribe", "topic": f"stream/motor/{_SO101}/state"})
        )
        resp = await stream_ctx.__aenter__()
        assert resp.status_code == 200
        agen = resp.aiter_bytes()
        first = await asyncio.wait_for(agen.__anext__(), timeout=5.0)
        assert first  # MJPEG streaming 시작됨

        # 핵심 assert — 열린 무한 연결에도 stop 이 bounded 완료.
        # (timeout_graceful_shutdown=2 → ~2.x s. fix 없으면 무한 대기.)
        t0 = time.monotonic()
        await asyncio.wait_for(booted.stop(), timeout=15.0)
        elapsed = time.monotonic() - t0
        assert elapsed < 10.0, f"stop 이 너무 오래 걸림: {elapsed:.1f}s"
    finally:
        # server 가 먼저 끊은 연결 정리 — protocol error 는 예상 동작.
        with contextlib.suppress(Exception):
            await stream_ctx.__aexit__(None, None, None)
        with contextlib.suppress(Exception):
            await ws.close()
        await client.aclose()
