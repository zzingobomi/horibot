"""Bridge MJPEG route C1c 검증 — camera(mock) → multipart/x-mixed-replace.

camera Module(mock driver) + bridge 를 한 runtime 에 띄우고, httpx 로 MJPEG
스트림을 읽어 첫 JPEG frame 도착을 확인. + camera/stream 게이팅 — color MJPEG 이라
has_camera(camera 소스 유무) 로 게이트하지 rgbd(depth) 로 하지 않음:
  - camera 있고 rgbd 없는 robot → 200 (depth 없어도 color stream 성립)
  - camera 없는 robot → 404
"""

from __future__ import annotations

from pathlib import Path
from typing import cast

import httpx
import pytest
from fastapi.testclient import TestClient

from framework.runtime.app import Runtime
from framework.transport.protocol import Handle, RawTransport
from infra.transport.zenoh import ZenohTransport
from modules.bridge.contract import RobotInfo
from modules.bridge.module import BridgeModule
from modules.camera.drivers.mock import MockCameraDriver
from modules.camera.module import CameraDriverModule

_CONFIG_DIR = Path(__file__).resolve().parents[2] / "config"
_LOCAL_CFG = {"mode": "peer", "scouting": {"multicast": {"enabled": False}}}
_SO101 = "so101_6dof_0"


@pytest.fixture
async def base_url():
    transport = ZenohTransport(_LOCAL_CFG)
    runtime = Runtime(transport)
    # 게이팅 케이스를 hand-built list 로 명시 — robots.yaml 의 enabled 상태
    # (resolve 가 disabled 제외) 와 무관하게 camera/rgbd 조합별 gate 검증.
    infos = [
        RobotInfo(
            id=_SO101, type="so101_6dof", capabilities=["move", "rgbd"], has_camera=True
        ),
        # camera 있고 rgbd 없음 — color MJPEG 은 성립해야 (옛 rgbd-gate 버그 가드)
        RobotInfo(id="cam_norgbd", type="omx_f", capabilities=["move"], has_camera=True),
    ]
    # port=0 (ephemeral) — 다른 backend/테스트와 포트 충돌 원천 차단
    bridge = runtime.add_module(
        BridgeModule, port=0, host="127.0.0.1", robots=infos
    )
    runtime.add_module(
        CameraDriverModule, robot_id=_SO101, driver=MockCameraDriver(has_depth=True)
    )
    await runtime.start()
    yield f"http://127.0.0.1:{bridge.port}"
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


async def test_mjpeg_camera_without_rgbd_streams(base_url: str):
    # cam_norgbd = camera 있음, rgbd 없음. color MJPEG 은 depth 무관 →
    # 게이트 통과(200). (rgbd 로 게이트하던 옛 버그면 여기서 404 — 회귀 가드.)
    # 해당 camera module 은 이 fixture 에 안 떠 프레임은 안 오지만, StreamingResponse
    # 는 status/헤더를 body 전에 보내므로 status 만 확인하고 빠진다.
    url = f"{base_url}/robots/cam_norgbd/camera/stream"
    async with httpx.AsyncClient(timeout=5.0) as client:
        async with client.stream("GET", url) as resp:
            assert resp.status_code == 200
            assert "multipart/x-mixed-replace" in resp.headers["content-type"]


async def test_mjpeg_404_when_unknown_robot(base_url: str):
    async with httpx.AsyncClient() as client:
        res = await client.get(f"{base_url}/robots/nope/camera/stream")
    assert res.status_code == 404


class _NoopTransport:
    """RawTransport 스텁 — 404 경로는 transport 를 안 건드리므로 미사용."""

    def call(self, key: str, payload: bytes, timeout: float = 5.0) -> bytes:
        raise NotImplementedError

    def publish(self, key: str, payload: bytes) -> None:
        raise NotImplementedError

    def subscribe(self, key: str, callback) -> Handle:  # noqa: ANN001
        raise NotImplementedError


def test_mjpeg_404_when_no_camera():
    # camera_backend 없는 robot (has_camera=False) → color 소스 없어 프레임 영영 안 옴
    # → 매달리지 않게 404. rgbd 와 무관, camera 자체 유무가 기준.
    bridge = BridgeModule(
        transport=cast(RawTransport, _NoopTransport()),
        robots=[RobotInfo(id="nocam", type="x", has_camera=False)],
    )
    client = TestClient(bridge.app)
    res = client.get("/robots/nocam/camera/stream")
    assert res.status_code == 404
