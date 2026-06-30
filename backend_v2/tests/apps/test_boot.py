"""apps boot scaffolding 검증 (Step C0).

mock.yaml 을 격리 peer transport 로 한 process 에 띄우고:
- config 파싱 (robots.yaml + deployment yaml)
- resolve_deps → driver impl 선택 (mock)
- runtime.add_module 배선 + start
- motor / camera service 도달 + camera → camera_decoded stream e2e
까지 검증. 실 boot 의 build_runtime path 와 동일 (transport 만 주입).
"""

from __future__ import annotations

import asyncio
import time
from pathlib import Path

import pytest

from apps.config import load_deployment, load_robots
from apps.main import build_runtime, load_configs
from apps.resolve import resolve_deps
from framework.runtime.app import Runtime
from infra.transport.zenoh import ZenohTransport
from modules.camera.contract import (
    Camera,
    CameraCapabilities,
    CameraCapability,
    CameraDecodedFrame,
)
from modules.camera.contract import CapabilitiesRequest as CameraCapsRequest
from modules.camera.contract import DecodedSnapshotRequest
from modules.camera.module import CameraDriverModule
from modules.motor.contract import CapabilitiesRequest as MotorCapsRequest
from modules.motor.contract import Motor, MotorCapabilities, MotorCapability
from modules.motor.module import MotorDriverModule

_CONFIG_DIR = Path(__file__).resolve().parents[2] / "config"
_LOCAL_CFG = {"mode": "peer", "scouting": {"multicast": {"enabled": False}}}
_SO101 = "so101_6dof_0"


# ─── config 파싱 ────────────────────────────────────────────────


def test_load_robots_parses_entries():
    robots = load_robots()
    assert set(robots) == {_SO101, "omx_f_0"}
    assert robots[_SO101].id == _SO101  # key → id 채워짐
    assert robots[_SO101].type == "so101_6dof"
    assert robots[_SO101].camera_backend == "realsense"
    assert robots[_SO101].motor_backend == "feetech"
    assert len(robots[_SO101].motors) == 7  # motors.yaml: 6 joint + gripper
    assert robots["omx_f_0"].camera_backend == "opencv"


def test_load_deployment_mock():
    deploy = load_deployment(_CONFIG_DIR / "deployments" / "mock.yaml")
    assert deploy.driver_mode == "mock"
    assert {m.name for m in deploy.modules} == {"motor", "camera", "camera_decoded"}


# ─── resolve_deps ───────────────────────────────────────────────


def test_resolve_deps_mock_motor_picks_mock_backend():
    deploy = load_deployment(_CONFIG_DIR / "deployments" / "mock.yaml")
    robots = load_robots()
    deps = resolve_deps(MotorDriverModule, robots[_SO101], deploy)
    # mock motor → topology 6 joint + gripper
    topo = deps["driver"].topology()
    assert len(topo.motors) == 7


def test_resolve_deps_real_feetech_constructs():
    # real + feetech → FeetechBackend 생성 (하드웨어 없이 self-declare 만 검증, open X)
    from modules.motor.drivers.feetech import FeetechBackend

    deploy = load_deployment(_CONFIG_DIR / "deployments" / "pi_motor.yaml")  # real
    robots = load_robots()
    driver = resolve_deps(MotorDriverModule, robots[_SO101], deploy)["driver"]
    assert isinstance(driver, FeetechBackend)
    assert len(driver.topology().motors) == 7  # motors.yaml SSOT
    assert MotorCapability.TORQUE_TOGGLE in driver.capabilities().flags


def test_resolve_deps_real_realsense_constructs():
    # real + realsense → RealSenseD405Driver 생성 (하드웨어 없이 self-declare 만, open X)
    from modules.camera.drivers.realsense_d405 import RealSenseD405Driver

    deploy = load_deployment(_CONFIG_DIR / "deployments" / "pi_camera.yaml")  # real
    robots = load_robots()
    driver = resolve_deps(CameraDriverModule, robots[_SO101], deploy)["driver"]
    assert isinstance(driver, RealSenseD405Driver)
    assert CameraCapability.DEPTH in driver.capabilities().flags


def test_resolve_deps_camera_mock_depth_from_rgbd_capability():
    # mock camera 의 depth 여부 = rgbd capability (so101 ✅ / omx ❌)
    deploy = load_deployment(_CONFIG_DIR / "deployments" / "mock.yaml")
    robots = load_robots()
    so101_cam = resolve_deps(CameraDriverModule, robots[_SO101], deploy)["driver"]
    assert CameraCapability.DEPTH in so101_cam.capabilities().flags
    omx_cam = resolve_deps(CameraDriverModule, robots["omx_f_0"], deploy)["driver"]
    assert CameraCapability.DEPTH not in omx_cam.capabilities().flags


# ─── e2e boot — mock.yaml 한 process ────────────────────────────


@pytest.fixture
async def booted():
    transport = ZenohTransport(_LOCAL_CFG)
    time.sleep(0.05)
    deploy, robots = load_configs("mock", _CONFIG_DIR)
    runtime: Runtime = build_runtime(deploy, robots, transport)
    await runtime.start()
    yield runtime
    await runtime.stop()
    transport.close()


async def test_boot_motor_capabilities_reachable(booted: Runtime):
    res = await booted.module_runtime.call(
        Motor.Service.CAPABILITIES,
        MotorCapsRequest(),
        MotorCapabilities,
        robot_id=_SO101,
    )
    assert len(res.flags) > 0


async def test_boot_camera_capabilities_reachable(booted: Runtime):
    res = await booted.module_runtime.call(
        Camera.Service.CAPABILITIES,
        CameraCapsRequest(),
        CameraCapabilities,
        robot_id=_SO101,
    )
    assert CameraCapability.DEPTH in res.flags  # rgbd robot


async def test_boot_camera_decode_stream_e2e(booted: Runtime):
    # camera capture loop (30Hz) → JPEG stream → camera_decoded → DECODED_SNAPSHOT.
    # 한 process 안 두 robot-scoped Module 이 Zenoh peer 로 stream 주고받는지 검증.
    decoded: CameraDecodedFrame | None = None
    for _ in range(50):  # ~2.5s 까지 frame 도착 대기
        await asyncio.sleep(0.05)
        try:
            decoded = await booted.module_runtime.call(
                Camera.Service.DECODED_SNAPSHOT,
                DecodedSnapshotRequest(),
                CameraDecodedFrame,
                robot_id=_SO101,
            )
            break
        except Exception:
            continue
    assert decoded is not None, "camera → camera_decoded stream e2e frame 도착 X"
    assert decoded.width > 0 and decoded.height > 0
