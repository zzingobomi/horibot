"""Bridge MJPEG route C1c 검증 — camera(mock) → multipart/x-mixed-replace.

camera Module(mock driver) + bridge 를 한 runtime 에 띄우고, httpx 로 MJPEG
스트림을 읽어 첫 JPEG frame 도착 + rgbd 없는 robot 404 를 확인.
"""

from __future__ import annotations

from pathlib import Path

import httpx
import pytest

from apps.config import DeploymentConfig, DriverMode, ModuleEntry, load_robots
from apps.resolve import resolve_host_deps
from framework.runtime.app import Runtime
from infra.transport.zenoh import ZenohTransport
from modules.bridge.module import BridgeModule
from modules.camera.drivers.mock import MockCameraDriver
from modules.camera.module import CameraDriverModule

_CONFIG_DIR = Path(__file__).resolve().parents[2] / "config"
_LOCAL_CFG = {"mode": "peer", "scouting": {"multicast": {"enabled": False}}}
_PORT = 8079
_SO101 = "so101_6dof_0"


@pytest.fixture
async def base_url():
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
    yield f"http://127.0.0.1:{_PORT}"
    await runtime.stop()
    transport.close()


async def test_mjpeg_stream_serves_jpeg_frames(base_url: str):
    url = f"{base_url}/robots/{_SO101}/camera/stream"
    async with httpx.AsyncClient(timeout=5.0) as client:
        async with client.stream("GET", url) as resp:
            assert resp.status_code == 200
            assert "multipart/x-mixed-replace" in resp.headers["content-type"]
            buf = b""
            async for chunk in resp.aiter_bytes():
                buf += chunk
                if b"\xff\xd8" in buf:  # JPEG SOI marker — frame 도착
                    break
    assert b"image/jpeg" in buf
    assert b"\xff\xd8" in buf


async def test_mjpeg_404_when_no_rgbd(base_url: str):
    # omx_f_0 = capabilities 비어있음 (rgbd 없음)
    async with httpx.AsyncClient() as client:
        res = await client.get(f"{base_url}/robots/omx_f_0/camera/stream")
    assert res.status_code == 404


async def test_mjpeg_404_when_unknown_robot(base_url: str):
    async with httpx.AsyncClient() as client:
        res = await client.get(f"{base_url}/robots/nope/camera/stream")
    assert res.status_code == 404
