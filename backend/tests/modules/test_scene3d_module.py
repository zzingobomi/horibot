"""Scene3DModule (@service wire) 검증 — in-process, hardware 불요.

robot-agnostic (host 당 1) — **multi-robot 눈속임 방지**
(backend.md §2.7.3): rgbd robot 2대의 depth/color/intrinsic/
stream 상태가 한 인스턴스 안에서 격리되는지. e2e wire (mock boot) 는
test_scan_e2e 가 커버 — 여기선 dispatch/격리를 fake runtime 으로 직접.
"""

from __future__ import annotations

import numpy as np
import pytest
import zstandard as zstd
from pydantic import BaseModel

from framework.runtime.discovery import discover_services
from modules.camera.contract import (
    Camera,
    CameraDecodedFrame,
    CameraDepthDecodedFrame,
    FactoryIntrinsic,
)
from modules.scene3d.contract import Scene3d, SetStreamRequest, SnapshotRequest
from modules.scene3d.module import Scene3DModule

_SO101 = "so101_6dof_0"
_OMX = "omx_f_0"
_W, _H = 64, 48


class _FakeRuntime:
    """(key, robot_id) → canned 응답. robot_id 는 kwarg 또는 req 필드에서."""

    def __init__(self, responses: dict) -> None:
        self._responses = responses
        self.events: list[tuple[str, BaseModel]] = []

    def publish(self, wire_key: str, event: BaseModel) -> None:
        self.events.append((str(wire_key), event))

    async def call(self, key, req, res_cls, *, robot_id=None, timeout=None):  # noqa: ANN001,ANN002
        rid = robot_id or getattr(req, "robot_id", None)
        return self._responses[(str(key), rid)]


def _module() -> tuple[Scene3DModule, _FakeRuntime]:
    # calibration bundle 은 canned 미등록 (KeyError) → factory fallback 경로.
    # factory intrinsic fx 로 robot 구분 (600 vs 700).
    rt = _FakeRuntime(
        {
            (str(Camera.Service.GET_FACTORY_INTRINSIC), _SO101): FactoryIntrinsic(
                available=True, width=_W, height=_H, fx=600.0, fy=600.0,
                cx=_W / 2, cy=_H / 2,
            ),
            (str(Camera.Service.GET_FACTORY_INTRINSIC), _OMX): FactoryIntrinsic(
                available=True, width=_W, height=_H, fx=700.0, fy=700.0,
                cx=_W / 2, cy=_H / 2,
            ),
        }
    )
    return Scene3DModule(runtime=rt, robot_ids=[_SO101, _OMX]), rt


def _feed(mod: Scene3DModule, robot_id: str, depth_value: int) -> None:
    depth = np.full((_H, _W), depth_value, dtype=np.uint16)
    color = np.full((_H, _W, 3), 128, dtype=np.uint8)
    mod.on_depth(
        CameraDepthDecodedFrame(
            robot_id=robot_id, seq=0, timestamp_unix=0.0,
            depth_bytes=depth.tobytes(), width=_W, height=_H, depth_scale=0.001,
        )
    )
    mod.on_color(
        CameraDecodedFrame(
            robot_id=robot_id, seq=0, timestamp_unix=0.0,
            ndarray_bytes=color.tobytes(), width=_W, height=_H,
        )
    )


def test_service_wiring_agnostic_keys():
    mod, _ = _module()
    keys = {spec.wire_key for _m, spec in discover_services(mod)}
    assert keys == {Scene3d.Service.SET_STREAM, Scene3d.Service.SNAPSHOT}
    # robot-agnostic — 서비스 키에 {robot_id} placeholder 없음 (§2.7.3 acceptance 1)
    assert all("{robot_id}" not in k for k in keys)
    assert not hasattr(mod, "robot_id")


async def test_single_instance_serves_so101_and_omx_isolated():
    """★ 리트머스 — 한 host-level 인스턴스가 rgbd robot 2대 snapshot 격리.

    각 robot 의 depth(300 vs 500)/intrinsic(fx 600 vs 700) 이 안 섞이는지.
    """
    mod, _ = _module()
    await mod.start()  # intrinsic pull (factory fx 600/700) + live loop
    try:
        _feed(mod, _SO101, depth_value=300)
        _feed(mod, _OMX, depth_value=500)

        snap_so = mod.snapshot(SnapshotRequest(robot_id=_SO101, num_frames=1))
        snap_omx = mod.snapshot(SnapshotRequest(robot_id=_OMX, num_frames=1))

        # intrinsic — 각 robot 의 factory fx
        assert snap_so.intrinsic.fx == 600.0
        assert snap_omx.intrinsic.fx == 700.0
        # depth 무손실 round-trip — 각 robot 의 값 (consensus median = 단일 frame 값)
        dctx = zstd.ZstdDecompressor()
        d_so = np.frombuffer(dctx.decompress(snap_so.depth_zstd), dtype=np.uint16)
        d_omx = np.frombuffer(dctx.decompress(snap_omx.depth_zstd), dtype=np.uint16)
        assert int(np.median(d_so)) == 300
        assert int(np.median(d_omx)) == 500
    finally:
        await mod.stop()


async def test_set_stream_per_robot_and_unknown_rejected():
    mod, rt = _module()
    await mod.start()
    try:
        res = mod.set_stream(SetStreamRequest(robot_id=_SO101, enabled=True))
        assert res.ok and res.enabled
        # omx 는 안 켰음 — 상태 격리 (내부 buf 확인)
        assert mod._buf[_SO101].enabled is True  # noqa: SLF001 — 격리 검증
        assert mod._buf[_OMX].enabled is False  # noqa: SLF001
        # fleet 에 없는 robot → fail-fast
        with pytest.raises(KeyError):
            mod.set_stream(SetStreamRequest(robot_id="ghost", enabled=True))
        with pytest.raises(KeyError):
            mod.snapshot(SnapshotRequest(robot_id="ghost"))
    finally:
        await mod.stop()


async def test_live_loop_publishes_only_enabled_robot():
    """so101 만 enable — 두 robot frame 이 다 있어도 so101 CLOUD 만 발행."""
    import asyncio

    mod, rt = _module()
    await mod.start()
    try:
        _feed(mod, _SO101, depth_value=300)
        _feed(mod, _OMX, depth_value=500)
        mod.set_stream(SetStreamRequest(robot_id=_SO101, enabled=True))
        clouds: list = []
        for _ in range(40):  # ~2s (8Hz)
            await asyncio.sleep(0.05)
            clouds = [e for k, e in rt.events if k == Scene3d.Stream.CLOUD]
            if clouds:
                break
    finally:
        await mod.stop()
    assert clouds, "CLOUD stream 발행 안 됨"
    assert all(c.robot_id == _SO101 for c in clouds)  # omx 발행 X
