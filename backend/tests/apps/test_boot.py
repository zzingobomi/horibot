"""apps boot scaffolding 검증 (Step C0).

mock.yaml 을 격리 peer transport 로 한 process 에 띄우고:
- config 파싱 (robots.yaml + deployment yaml)
- resolve_robot_deps → driver impl 선택 (mock)
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
from apps.resolve import resolve_robot_deps, resolve_host_deps
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
from modules.motor.contract import CapabilitiesRequest as MotorCapsRequest
from modules.motor.contract import Motor, MotorCapabilities, MotorCapability

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
    assert robots["omx_f_0"].camera_device_index == 0  # instance.yaml camera
    assert len(robots["omx_f_0"].motors) == 6  # 5 joint + gripper
    # motors.yaml `pid` 파싱 — omx J2 shoulder P=1500 (RAM 재적용 대상),
    # so101 은 pid 블록 없음 (STS EEPROM — Wizard 1회).
    assert robots["omx_f_0"].motors[1].pid_p == 1500
    assert robots[_SO101].motors[0].pid_p is None


def test_load_robots_reads_physical_sag_joints():
    # <type>/physical.yaml 의 robot physical model (sag_joint_motor_ids) 로딩.
    robots = load_robots()
    assert robots[_SO101].sag_joint_motor_ids == [2, 3]  # shoulder / elbow


def test_load_deployment_mock():
    # 파싱 동작만 — 모듈 전체 목록은 yaml 미러라 잠그지 않는다 (모듈 추가마다
    # 테스트 수정 유발). yaml↔registry 정합은 booted e2e 가 실부팅으로 잡는다.
    deploy = load_deployment(_CONFIG_DIR / "deployments" / "mock.yaml")
    assert deploy.driver_mode == "mock"
    names = {m.name for m in deploy.modules}
    assert {"motor", "motion", "camera", "bridge"} <= names
    assert deploy.rdb_uri == "sqlite:///:memory:"  # DB owner host


# ─── resolve_robot_deps ───────────────────────────────────────────────


def test_resolve_robot_deps_mock_motor_picks_mock_backend():
    deploy = load_deployment(_CONFIG_DIR / "deployments" / "mock.yaml")
    robots = load_robots()
    deps = resolve_robot_deps("motor", robots[_SO101], deploy)
    # mock motor → topology 6 joint + gripper
    topo = deps["driver"].topology()
    assert len(topo.motors) == 7


@pytest.mark.sim  # clean subprocess 기동 (~5s) — 배포 게이트, fast loop 제외
def test_registry_role_isolated_no_heavy_imports():
    # lazy registry invariant — motor/motion load 가 fastapi(bridge)/pyrealsense2·cv2
    # (camera) 를 끌어오면 안 됨 (pi_hori1 가 그 deps 없이 boot 가능해야).
    # clean subprocess 로 검증 (같은 프로세스는 다른 test 가 이미 import 했을 수 있음).
    import subprocess
    import sys

    code = (
        "import sys\n"
        "from apps.registry import load_module_class\n"
        "load_module_class('motor'); load_module_class('motion')\n"
        "heavy = [m for m in ('fastapi', 'pyrealsense2', 'cv2') if m in sys.modules]\n"
        "assert not heavy, f'pi_hori1 import 에 끌려온 무거운 모듈: {heavy}'\n"
        "print('isolated-ok')\n"
    )
    root = Path(__file__).resolve().parents[2]  # backend
    r = subprocess.run(
        [sys.executable, "-c", code], cwd=root, capture_output=True, text=True
    )
    assert r.returncode == 0, f"stdout={r.stdout} stderr={r.stderr}"
    assert "isolated-ok" in r.stdout


def test_resolve_robot_deps_real_feetech_constructs():
    # real + feetech → FeetechBackend 생성 (하드웨어 없이 self-declare 만 검증, open X)
    from modules.motor.drivers.feetech import FeetechBackend

    deploy = load_deployment(_CONFIG_DIR / "deployments" / "pi_hori1.yaml")  # real
    robots = load_robots()
    driver = resolve_robot_deps("motor", robots[_SO101], deploy)["driver"]
    assert isinstance(driver, FeetechBackend)
    assert len(driver.topology().motors) == 7  # motors.yaml SSOT
    assert MotorCapability.TORQUE_TOGGLE in driver.capabilities().flags


def test_resolve_robot_deps_real_realsense_constructs():
    # real + realsense → RealSenseD405Driver 생성 (하드웨어 없이 self-declare 만, open X)
    from modules.camera.drivers.realsense_d405 import RealSenseD405Driver

    deploy = load_deployment(_CONFIG_DIR / "deployments" / "pi_hori2.yaml")  # real
    robots = load_robots()
    driver = resolve_robot_deps("camera", robots[_SO101], deploy)["driver"]
    assert isinstance(driver, RealSenseD405Driver)
    assert CameraCapability.DEPTH in driver.capabilities().flags


def test_resolve_robot_deps_real_dynamixel_constructs():
    # real + dynamixel → DynamixelBackend 생성 (하드웨어 없이 self-declare 만, open X)
    from modules.motor.drivers.dynamixel import DynamixelBackend

    deploy = load_deployment(_CONFIG_DIR / "deployments" / "pi_hori3.yaml")  # real
    robots = load_robots()
    driver = resolve_robot_deps("motor", robots["omx_f_0"], deploy)["driver"]
    assert isinstance(driver, DynamixelBackend)
    assert len(driver.topology().motors) == 6  # motors.yaml SSOT: 5 joint + gripper
    flags = driver.capabilities().flags
    assert MotorCapability.TORQUE_TOGGLE in flags
    assert MotorCapability.REBOOT in flags  # XL = software reboot (STS 와 차이)


def test_resolve_robot_deps_real_opencv_uvc_constructs():
    # real + opencv → OpenCVUvcDriver 생성 (color-only, open X)
    from modules.camera.drivers.opencv_uvc import OpenCVUvcDriver

    deploy = load_deployment(_CONFIG_DIR / "deployments" / "pi_hori3.yaml")  # real
    robots = load_robots()
    driver = resolve_robot_deps("camera", robots["omx_f_0"], deploy)["driver"]
    assert isinstance(driver, OpenCVUvcDriver)
    flags = driver.capabilities().flags
    assert CameraCapability.RGB in flags
    assert CameraCapability.DEPTH not in flags  # UVC color-only
    assert driver.get_factory_intrinsics() is None  # 사용자 intrinsic 캘 필요


@pytest.mark.sim  # torch/transformers import (~13s) — 배포 게이트, fast loop 제외
def test_resolve_host_deps_real_detector_constructs():
    # real → GroundedSamBackend 생성 (GDINO+SAM2, 모델 로드 X — preload/detect 안 부름).
    from modules.detector.drivers.grounded_sam import GroundedSamBackend

    deploy = load_deployment(_CONFIG_DIR / "deployments" / "pc.yaml")  # real
    robots = load_robots()
    backend = resolve_host_deps("detector", robots, deploy)["backend"]
    assert isinstance(backend, GroundedSamBackend)


def test_resolve_robot_deps_camera_mock_depth_from_rgbd_capability():
    # mock camera 의 depth 여부 = rgbd capability (so101 ✅ / omx ❌)
    deploy = load_deployment(_CONFIG_DIR / "deployments" / "mock.yaml")
    robots = load_robots()
    so101_cam = resolve_robot_deps("camera", robots[_SO101], deploy)["driver"]
    assert CameraCapability.DEPTH in so101_cam.capabilities().flags
    omx_cam = resolve_robot_deps("camera", robots["omx_f_0"], deploy)["driver"]
    assert CameraCapability.DEPTH not in omx_cam.capabilities().flags


# ─── e2e boot — mock.yaml 한 process ────────────────────────────
#
# Runtime 전체 부팅 (~3s/test) = 마커 정의 그대로 sim — 아래 e2e 전부 개별 마킹.

_SIM = pytest.mark.sim


@pytest.fixture
async def booted():
    transport = ZenohTransport(_LOCAL_CFG)
    time.sleep(0.05)
    deploy, robots = load_configs("mock", _CONFIG_DIR)
    deploy.bridge_port = 0  # ephemeral — 실행 중인 실 backend(:8000) 와 공존
    runtime: Runtime = build_runtime(deploy, robots, transport)
    try:
        await runtime.start()
    except BaseException:
        # start 실패 시 teardown(yield 이후) 이 안 돌므로 여기서 정리 —
        # zenoh 세션이 열린 채 남으면 pytest 프로세스가 종료를 못 한다.
        transport.close()
        raise
    yield runtime
    await runtime.stop()
    transport.close()


@_SIM
async def test_boot_motor_capabilities_reachable(booted: Runtime):
    res = await booted.module_runtime.call(
        Motor.Service.CAPABILITIES,
        MotorCapsRequest(),
        MotorCapabilities,
        robot_id=_SO101,
    )
    assert len(res.flags) > 0


@_SIM
async def test_boot_camera_capabilities_reachable(booted: Runtime):
    res = await booted.module_runtime.call(
        Camera.Service.CAPABILITIES,
        CameraCapsRequest(),
        CameraCapabilities,
        robot_id=_SO101,
    )
    assert CameraCapability.DEPTH in res.flags  # rgbd robot


@_SIM
async def test_boot_omx_motor_and_camera_reachable(booted: Runtime):
    # 2-robot mock boot (2026-07-09 omx 재활성화) — omx robot-scoped module 도
    # 같은 process 에 mount, robot_id 라우팅으로 도달.
    res = await booted.module_runtime.call(
        Motor.Service.CAPABILITIES,
        MotorCapsRequest(),
        MotorCapabilities,
        robot_id="omx_f_0",
    )
    assert len(res.flags) > 0
    cam = await booted.module_runtime.call(
        Camera.Service.CAPABILITIES,
        CameraCapsRequest(),
        CameraCapabilities,
        robot_id="omx_f_0",
    )
    assert CameraCapability.DEPTH not in cam.flags  # UVC color-only


@_SIM
async def test_boot_calibration_snapshot_bundle_reachable(booted: Runtime):
    # calibration robot-agnostic Module 이 booted (rdb_uri :memory: + alembic upgrade
    # head) → snapshot_bundle service 가 Zenoh 로 도달 (키에 {robot_id} 없음, 대상은
    # req 필드). §10.1 — Calibration.start() 가 Camera GET_FACTORY_INTRINSIC pull →
    # intrinsic 자동 seed (mock camera 는 synthetic).
    from modules.calibration.contract import (
        Calibration,
        CalibrationBundle,
        SnapshotBundleRequest,
    )

    res = await booted.module_runtime.call(
        Calibration.Service.SNAPSHOT_BUNDLE,
        SnapshotBundleRequest(robot_id=_SO101),
        CalibrationBundle,
    )
    assert res.robot_id == _SO101
    # factory intrinsic over-wire seed 검증 (§10.1)
    assert res.intrinsic is not None
    assert res.intrinsic.result_data.camera_matrix[0][0] > 0  # fx
    # 나머지는 미활성 (hand_eye 등은 offline BA 자리)
    assert res.hand_eye is None
    assert res.signature() == (("intrinsic", res.intrinsic.id),)


@_SIM
async def test_boot_calibration_start_run_and_list_over_wire(booted: Runtime):
    # write 경로 + 마이그레이션 검증 — start_run(INSERT) → list_runs(SELECT) over Zenoh.
    from modules.calibration.contract import (
        Calibration,
        ListRunsRequest,
        ListRunsResponse,
        StartRunRequest,
        StartRunResponse,
    )

    started = await booted.module_runtime.call(
        Calibration.Service.START_RUN,
        StartRunRequest(
            robot_id=_SO101, kind="hand_eye", algorithm="hand_eye_capture_only"
        ),
        StartRunResponse,
    )
    assert started.run_id > 0

    listed = await booted.module_runtime.call(
        Calibration.Service.LIST_RUNS,
        ListRunsRequest(robot_id=_SO101),
        ListRunsResponse,
    )
    assert any(r.id == started.run_id and r.kind == "hand_eye" for r in listed.runs)


@_SIM
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
