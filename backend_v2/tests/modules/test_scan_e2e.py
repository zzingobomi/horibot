"""Scan 파이프라인 e2e — mock.yaml 한 process (real Zenoh peer) 로 boot.

full wire 검증: camera → camera_decoded → scene3d(consensus snapshot) → scan capture
(blob 저장) → scan build (raw→FK→hand_eye→TSDF) → get_mesh. scan CAPTURE/BUILD 의
sync→async bridge (run_coroutine_threadsafe in-process loopback) 포함.

hand_eye 는 mock 에 없어 build 는 offline 이 아니라 여기선 fake activate — 대신
build 가 "hand_eye 없음"을 정확히 reject 하는지 + 강제 seed 후 mesh 생성까지 검증.

`sim` 마크 — Runtime/PyBullet/open3d boot 필요 (집 fast loop 에서 skip).
"""

from __future__ import annotations

import asyncio
import time
from datetime import UTC, datetime
from pathlib import Path

import pytest

from apps.main import build_runtime, load_configs
from framework.runtime.app import Runtime
from infra.transport.zenoh import ZenohTransport
from modules.scan.contract import (
    BuildRequest,
    BuildResponse,
    CaptureRequest,
    CaptureResponse,
    GetMeshRequest,
    GetMeshResponse,
    ListReconstructionsRequest,
    ListReconstructionsResponse,
    ListScansRequest,
    ListScansResponse,
    NewSessionRequest,
    NewSessionResponse,
    Scan,
)
from modules.scene3d.contract import (
    Scene3d,
    SetStreamRequest,
    SetStreamResponse,
)

pytestmark = pytest.mark.sim

_CONFIG_DIR = Path(__file__).resolve().parents[2] / "config"
_LOCAL_CFG = {"mode": "peer", "scouting": {"multicast": {"enabled": False}}}
_SO101 = "so101_6dof_0"


@pytest.fixture
async def booted():
    transport = ZenohTransport(_LOCAL_CFG)
    time.sleep(0.05)
    deploy, robots = load_configs("mock", _CONFIG_DIR)
    runtime: Runtime = build_runtime(deploy, robots, transport)
    await runtime.start()
    # camera → camera_decoded → scene3d depth buffer 채워질 시간
    await asyncio.sleep(1.5)
    yield runtime
    await runtime.stop()
    transport.close()


def _seed_hand_eye(runtime: Runtime) -> None:
    """mock 엔 hand_eye 없음 → build 검증 위해 identity hand_eye 를 DB 에 직접 activate.

    ScanModule.build 가 Calibration.SNAPSHOT_BUNDLE 에서 hand_eye 를 읽으므로,
    calibration repository 에 hand_eye result 를 insert + activate. (offline BA 는
    별도 — 여기선 wire 검증용 최소 seed.)"""
    from modules.calibration.contract import (
        HandEyeResultData,
        HandEyeResultRecord,
    )

    for m in runtime._modules:  # noqa: SLF001 — test 전용 introspection
        if type(m).__name__ == "CalibrationModule" and getattr(m, "robot_id", None) == _SO101:
            repo = m._repo  # noqa: SLF001
            run = repo.create_run(_SO101, "hand_eye", "test_seed")
            assert run.id is not None
            rid = repo.save_result(
                run.id,
                HandEyeResultRecord(
                    run_id=run.id,
                    robot_id=_SO101,
                    created_at=datetime.now(UTC),
                    result_data=HandEyeResultData(
                        R_cam2gripper=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                        t_cam2gripper=[[0.0], [0.0], [0.05]],
                        method="test_seed",
                    ),
                ),
            )
            repo.activate_result(rid)
            return
    raise AssertionError("CalibrationModule 못 찾음")


async def test_scene3d_live_stream_toggle(booted: Runtime):
    # SET_STREAM enable → CLOUD frame 도착 (camera-frame points).
    got: list[bytes] = []
    handle = booted._transport.subscribe(  # noqa: SLF001
        f"stream/scene3d/{_SO101}/cloud", lambda p: got.append(p)
    )
    try:
        res = await booted.module_runtime.call(
            Scene3d.Service.SET_STREAM,
            SetStreamRequest(enabled=True),
            SetStreamResponse,
            robot_id=_SO101,
        )
        assert res.enabled is True
        for _ in range(40):  # ~2s
            await asyncio.sleep(0.05)
            if got:
                break
        assert got, "scene3d CLOUD stream frame 도착 X"
    finally:
        handle.undeclare()


async def test_scan_capture_build_mesh_e2e(booted: Runtime):
    _seed_hand_eye(booted)

    # 1) 세션
    sess = await booted.module_runtime.call(
        Scan.Service.NEW_SESSION,
        NewSessionRequest(label="e2e"),
        NewSessionResponse,
        robot_id=_SO101,
    )
    sid = sess.session.id
    assert sid is not None

    # 2) capture x3 (consensus snapshot + blob 저장)
    for i in range(3):
        cap = await booted.module_runtime.call(
            Scan.Service.CAPTURE,
            CaptureRequest(session_row_id=sid, num_frames=5),
            CaptureResponse,
            robot_id=_SO101,
            timeout=15.0,
        )
        assert cap.accepted, f"capture {i} 실패: {cap.message}"
        assert cap.scan is not None
        assert cap.scan.scan_id == i + 1  # monotonic
        await asyncio.sleep(0.2)

    scans = await booted.module_runtime.call(
        Scan.Service.LIST_SCANS,
        ListScansRequest(session_row_id=sid),
        ListScansResponse,
        robot_id=_SO101,
    )
    assert len(scans.scans) == 3

    # 3) build (TSDF) — raw→FK→hand_eye→mesh
    build = await booted.module_runtime.call(
        Scan.Service.BUILD,
        BuildRequest(session_row_id=sid),
        BuildResponse,
        robot_id=_SO101,
        timeout=90.0,
    )
    assert build.accepted, f"build 실패: {build.message}"
    assert build.reconstruction is not None
    assert build.reconstruction.n_scans == 3
    assert build.reconstruction.vertex_count > 0, "mesh vertex 0 — TSDF integration 문제"
    recon_id = build.reconstruction.id
    assert recon_id is not None

    # 4) get_mesh — .ply blob 반환
    mesh = await booted.module_runtime.call(
        Scan.Service.GET_MESH,
        GetMeshRequest(reconstruction_row_id=recon_id),
        GetMeshResponse,
        robot_id=_SO101,
    )
    assert len(mesh.ply_bytes) > 0
    assert mesh.ply_bytes[:3] == b"ply"  # PLY magic

    # 5) list_reconstructions
    recons = await booted.module_runtime.call(
        Scan.Service.LIST_RECONSTRUCTIONS,
        ListReconstructionsRequest(session_row_id=sid),
        ListReconstructionsResponse,
        robot_id=_SO101,
    )
    assert len(recons.reconstructions) == 1


async def test_scan_build_rejects_without_hand_eye(booted: Runtime):
    # hand_eye seed 안 함 → build 는 명확히 reject (accepted=False).
    sess = await booted.module_runtime.call(
        Scan.Service.NEW_SESSION,
        NewSessionRequest(label="no_he"),
        NewSessionResponse,
        robot_id=_SO101,
    )
    sid = sess.session.id
    assert sid is not None
    for _ in range(2):
        await booted.module_runtime.call(
            Scan.Service.CAPTURE,
            CaptureRequest(session_row_id=sid, num_frames=3),
            CaptureResponse,
            robot_id=_SO101,
            timeout=15.0,
        )
        await asyncio.sleep(0.15)
    build = await booted.module_runtime.call(
        Scan.Service.BUILD,
        BuildRequest(session_row_id=sid),
        BuildResponse,
        robot_id=_SO101,
        timeout=30.0,
    )
    assert build.accepted is False
    assert "hand_eye" in build.message
