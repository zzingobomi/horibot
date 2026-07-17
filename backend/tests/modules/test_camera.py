"""CameraDriver + CameraDecoded (Step B) test.

검증 자리:
- driver self-declare capability (§7.3) — RGB + DEPTH + POINTCLOUD
- JPEG / DEPTH_RAW stream publish (seq monotonic + timestamp_unix, §8.5)
- derived read model — CameraDecoded 가 JPEG / zstd subscribe → ndarray publish (§3.5 / §4.1)
- decode dedup — N consumer 박혔어도 decode 1 회 (§4 measurement)
- DECODED_SNAPSHOT / DEPTH_DECODED_SNAPSHOT service (point-in-time)
- robot-scoped per-instance — multi-robot 독립
- capability=rgb only driver = depth stream 안 publish
"""

from __future__ import annotations

import asyncio
import time

import numpy as np
import pytest

from framework.contract.subscriber import subscriber
from framework.runtime.api import ModuleRuntime
from framework.runtime.app import Runtime
from infra.transport.zenoh import ZenohTransport
from modules.camera.contract import (
    Camera,
    CameraCapabilities,
    CameraCapability,
    CameraDecodedFrame,
    CameraDepthDecodedFrame,
    CameraDepthRawFrame,
    CameraJpegFrame,
    CapabilitiesRequest,
    DecodedSnapshotRequest,
    DepthDecodedSnapshotRequest,
)
from modules.camera.decoded import CameraDecodedModule
from modules.camera.drivers.mock import MockCameraDriver
from modules.camera.module import CameraDriverModule

_LOCAL_CFG = {"mode": "peer", "scouting": {"multicast": {"enabled": False}}}

# 실 Zenoh 세션 + Runtime 부팅/test + sleep 폴링 — 마커 정의 그대로 sim
pytestmark = pytest.mark.sim


@pytest.fixture
def transport():
    t = ZenohTransport(_LOCAL_CFG)
    time.sleep(0.05)
    yield t
    t.close()


@pytest.fixture
async def runtime(transport: ZenohTransport):
    rt = Runtime(transport)
    yield rt
    await rt.stop()


# ─── 1. driver self-declare capability (§7.3) ────────────


async def test_capabilities_relay_driver_self_declare(runtime: Runtime):
    driver = MockCameraDriver(width=320, height=240, has_depth=True)
    runtime.add_module(CameraDriverModule, robot_id="so101_0", driver=driver)
    await runtime.start()

    res = await runtime.module_runtime.call(
        Camera.Service.CAPABILITIES,
        CapabilitiesRequest(),
        CameraCapabilities,
        robot_id="so101_0",
    )
    assert CameraCapability.RGB in res.flags
    assert CameraCapability.DEPTH in res.flags
    assert CameraCapability.POINTCLOUD in res.flags


async def test_capabilities_rgb_only_when_no_depth(runtime: Runtime):
    driver = MockCameraDriver(width=320, height=240, has_depth=False)
    runtime.add_module(CameraDriverModule, robot_id="usb_0", driver=driver)
    await runtime.start()

    res = await runtime.module_runtime.call(
        Camera.Service.CAPABILITIES,
        CapabilitiesRequest(),
        CameraCapabilities,
        robot_id="usb_0",
    )
    assert res.flags == {CameraCapability.RGB}


# ─── 2. JPEG stream publish — seq + timestamp_unix (§8.5) ─


async def test_jpeg_stream_publishes_with_seq_and_timestamp(
    runtime: Runtime,
):
    received: list[CameraJpegFrame] = []

    class Listener:
        def __init__(self, runtime: ModuleRuntime):
            self.runtime = runtime

        @subscriber(Camera.Stream.JPEG)
        def on_jpeg(self, event: CameraJpegFrame) -> None:
            received.append(event)

    driver = MockCameraDriver(width=320, height=240, has_depth=False)
    runtime.add_module(CameraDriverModule, robot_id="so101_0", driver=driver)
    runtime.add_module(Listener)
    await runtime.start()

    # 30Hz → 3 frame ~100ms. 1s wait 안 자연
    for _ in range(50):
        if len(received) >= 3:
            break
        await asyncio.sleep(0.05)

    assert len(received) >= 3, f"3+ frame 박혀야 — {len(received)}"
    seqs = [e.seq for e in received[:3]]
    assert seqs == sorted(seqs)
    # jpeg_bytes 검증 — 유효 JPEG header (FF D8)
    assert received[0].jpeg_bytes[:2] == b"\xff\xd8"
    assert received[0].width == 320
    assert received[0].height == 240


# ─── 3. DEPTH_RAW stream publish ─────────────────────────


async def test_depth_raw_stream_publishes_zstd(runtime: Runtime):
    received: list[CameraDepthRawFrame] = []

    class Listener:
        def __init__(self, runtime: ModuleRuntime):
            self.runtime = runtime

        @subscriber(Camera.Stream.DEPTH_RAW)
        def on_depth(self, event: CameraDepthRawFrame) -> None:
            received.append(event)

    driver = MockCameraDriver(width=320, height=240, has_depth=True)
    runtime.add_module(CameraDriverModule, robot_id="so101_0", driver=driver)
    runtime.add_module(Listener)
    await runtime.start()

    for _ in range(50):
        if received:
            break
        await asyncio.sleep(0.05)

    assert received, "depth frame 안 받음"
    frame = received[0]
    assert frame.width == 320
    assert frame.height == 240
    assert frame.depth_scale > 0
    # zstd magic 검증 — 0x28 B5 2F FD
    assert frame.depth_zstd[:4] == b"\x28\xb5\x2f\xfd"


async def test_no_depth_when_driver_rgb_only(runtime: Runtime):
    """capability 박지 X 박은 driver = depth stream 안 publish."""
    received: list[CameraDepthRawFrame] = []

    class Listener:
        def __init__(self, runtime: ModuleRuntime):
            self.runtime = runtime

        @subscriber(Camera.Stream.DEPTH_RAW)
        def on_depth(self, event: CameraDepthRawFrame) -> None:
            received.append(event)

    driver = MockCameraDriver(width=320, height=240, has_depth=False)
    runtime.add_module(CameraDriverModule, robot_id="usb_0", driver=driver)
    runtime.add_module(Listener)
    await runtime.start()

    await asyncio.sleep(0.3)  # capture 9 frame 박힐 자리
    assert not received, f"rgb-only driver 인데 depth 받음 — {len(received)}"


# ─── 4. CameraDecoded — derived read model (§3.5 / §4.1) ─


async def test_decoded_module_publishes_ndarray(runtime: Runtime):
    """JPEG → BGR ndarray + zstd depth → uint16 ndarray decode."""
    decoded: list[CameraDecodedFrame] = []
    depth_decoded: list[CameraDepthDecodedFrame] = []

    class Listener:
        def __init__(self, runtime: ModuleRuntime):
            self.runtime = runtime

        @subscriber(Camera.Stream.DECODED)
        def on_decoded(self, event: CameraDecodedFrame) -> None:
            decoded.append(event)

        @subscriber(Camera.Stream.DEPTH_DECODED)
        def on_depth_decoded(self, event: CameraDepthDecodedFrame) -> None:
            depth_decoded.append(event)

    driver = MockCameraDriver(width=320, height=240, has_depth=True)
    runtime.add_module(CameraDriverModule, robot_id="so101_0", driver=driver)
    runtime.add_module(CameraDecodedModule, robot_id="so101_0")
    runtime.add_module(Listener)
    await runtime.start()

    for _ in range(50):
        if decoded and depth_decoded:
            break
        await asyncio.sleep(0.05)

    assert decoded, "DECODED stream 안 받음"
    assert depth_decoded, "DEPTH_DECODED stream 안 받음"

    # ndarray 복원 — H × W × 3 BGR
    color = decoded[0]
    arr = np.frombuffer(color.ndarray_bytes, dtype=np.uint8).reshape(
        color.height, color.width, 3,
    )
    assert arr.shape == (240, 320, 3)

    # uint16 depth 복원
    depth = depth_decoded[0]
    depth_arr = np.frombuffer(depth.depth_bytes, dtype=np.uint16).reshape(
        depth.height, depth.width,
    )
    assert depth_arr.shape == (240, 320)
    # mock 자리 — row 별 1000 + y*10 gradient
    assert depth_arr[0, 0] == 1000
    assert depth_arr[1, 0] == 1010


# ─── 5. decode dedup — N consumer 박혔어도 decode 1 회 (§4) ──


async def test_decode_dedup_one_decode_per_frame(runtime: Runtime):
    """CameraDecoded 안 decode count = 받은 frame 수. N consumer 박혔어도
    decode 가 N 배 되지 X — derived read model 의 핵심 invariant."""
    consumer_a: list[CameraDecodedFrame] = []
    consumer_b: list[CameraDecodedFrame] = []
    consumer_c: list[CameraDecodedFrame] = []

    class ConsumerA:
        def __init__(self, runtime: ModuleRuntime):
            self.runtime = runtime

        @subscriber(Camera.Stream.DECODED)
        def on(self, event: CameraDecodedFrame) -> None:
            consumer_a.append(event)

    class ConsumerB:
        def __init__(self, runtime: ModuleRuntime):
            self.runtime = runtime

        @subscriber(Camera.Stream.DECODED)
        def on(self, event: CameraDecodedFrame) -> None:
            consumer_b.append(event)

    class ConsumerC:
        def __init__(self, runtime: ModuleRuntime):
            self.runtime = runtime

        @subscriber(Camera.Stream.DECODED)
        def on(self, event: CameraDecodedFrame) -> None:
            consumer_c.append(event)

    driver = MockCameraDriver(width=320, height=240, has_depth=False)
    runtime.add_module(CameraDriverModule, robot_id="so101_0", driver=driver)
    decoded_module = runtime.add_module(
        CameraDecodedModule, robot_id="so101_0",
    )
    runtime.add_module(ConsumerA)
    runtime.add_module(ConsumerB)
    runtime.add_module(ConsumerC)
    await runtime.start()

    # 5 frame 받을 자리 (30Hz × ~170ms)
    for _ in range(50):
        if min(len(consumer_a), len(consumer_b), len(consumer_c)) >= 5:
            break
        await asyncio.sleep(0.05)

    # 3 consumer 다 받음
    assert len(consumer_a) >= 5
    assert len(consumer_b) >= 5
    assert len(consumer_c) >= 5

    # decode count — N consumer 박혔어도 frame 수 만 (3× X)
    # ※ frame 수 = consumer_a 길이와 비슷 (decode 1회 / frame 1개)
    assert decoded_module.color_decode_count == len(consumer_a), (
        f"decode dedup 위반 — decoded {decoded_module.color_decode_count} "
        f"vs consumer received {len(consumer_a)}"
    )


# ─── 6. DECODED_SNAPSHOT service — point-in-time ──────────


async def test_decoded_snapshot_service(runtime: Runtime):
    driver = MockCameraDriver(width=320, height=240, has_depth=True)
    runtime.add_module(CameraDriverModule, robot_id="so101_0", driver=driver)
    runtime.add_module(CameraDecodedModule, robot_id="so101_0")
    await runtime.start()

    # frame 받을 자리 기다림
    await asyncio.sleep(0.2)

    res = await runtime.module_runtime.call(
        Camera.Service.DECODED_SNAPSHOT,
        DecodedSnapshotRequest(),
        CameraDecodedFrame,
        robot_id="so101_0",
    )
    assert res.width == 320
    assert res.height == 240
    assert res.robot_id == "so101_0"

    depth_res = await runtime.module_runtime.call(
        Camera.Service.DEPTH_DECODED_SNAPSHOT,
        DepthDecodedSnapshotRequest(),
        CameraDepthDecodedFrame,
        robot_id="so101_0",
    )
    assert depth_res.width == 320
    assert depth_res.height == 240


# ─── 7. multi-robot — per-instance 독립 stream ──────────


async def test_multi_robot_independent_decoded_streams(runtime: Runtime):
    so101_color: list[CameraDecodedFrame] = []
    omx_color: list[CameraDecodedFrame] = []

    class Listener:
        def __init__(self, runtime: ModuleRuntime):
            self.runtime = runtime

        @subscriber(Camera.Stream.DECODED)
        def on(self, event: CameraDecodedFrame) -> None:
            if event.robot_id == "so101_0":
                so101_color.append(event)
            elif event.robot_id == "omx_f_0":
                omx_color.append(event)

    runtime.add_module(
        CameraDriverModule, robot_id="so101_0",
        driver=MockCameraDriver(width=320, height=240, has_depth=False),
    )
    runtime.add_module(
        CameraDriverModule, robot_id="omx_f_0",
        driver=MockCameraDriver(width=160, height=120, has_depth=False),
    )
    runtime.add_module(CameraDecodedModule, robot_id="so101_0")
    runtime.add_module(CameraDecodedModule, robot_id="omx_f_0")
    runtime.add_module(Listener)
    await runtime.start()

    for _ in range(50):
        if so101_color and omx_color:
            break
        await asyncio.sleep(0.05)

    assert so101_color, "so101 DECODED 안 받음"
    assert omx_color, "omx DECODED 안 받음"
    # 각자 자기 robot 의 driver resolution
    assert so101_color[0].width == 320
    assert omx_color[0].width == 160


# ─── 8. JPEG decode round-trip — 합성 frame 일치 검증 ─────


async def test_jpeg_decode_roundtrip_color_consistency(runtime: Runtime):
    """mock 의 첫 frame = counter=0. JPEG encode → decode 후 B channel = 0 확인."""
    decoded: list[CameraDecodedFrame] = []

    class Listener:
        def __init__(self, runtime: ModuleRuntime):
            self.runtime = runtime

        @subscriber(Camera.Stream.DECODED)
        def on(self, event: CameraDecodedFrame) -> None:
            decoded.append(event)

    driver = MockCameraDriver(width=320, height=240, has_depth=False)
    runtime.add_module(CameraDriverModule, robot_id="so101_0", driver=driver)
    runtime.add_module(CameraDecodedModule, robot_id="so101_0")
    runtime.add_module(Listener)
    await runtime.start()

    for _ in range(50):
        if decoded:
            break
        await asyncio.sleep(0.05)

    assert decoded
    first = decoded[0]
    arr = np.frombuffer(first.ndarray_bytes, dtype=np.uint8).reshape(
        first.height, first.width, 3,
    )
    # JPEG 의 lossy 압축 — 0 ± 몇 자리. counter=0 자리 B channel mean ≈ 0
    assert int(arr[:, :, 0].mean()) <= 5, (
        f"B channel mean = {arr[:, :, 0].mean()} (counter=0 시작)"
    )
