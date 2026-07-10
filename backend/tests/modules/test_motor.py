"""MotorDriverModule (Step A) test.

검증 자리:
- service relay (capabilities / topology / set_torque / reboot / set_gripper)
- TorqueChanged event broadcast (set_torque 시)
- state stream 20Hz publish + seq monotonic + timestamp_unix invariant (§8.5)
- robot-scoped — 두 robot 동시 인스턴스 + 독립 stream
- driver self-declare — capabilities / topology cache (boot 1회 read, §7.3)
"""

from __future__ import annotations

import asyncio
import threading
import time

import pytest

from framework.contract.subscriber import subscriber
from framework.runtime.api import ModuleRuntime
from framework.runtime.app import Runtime
from infra.transport.zenoh import ZenohTransport
from modules.motor.contract import (
    CapabilitiesRequest,
    JointCommand,
    JointState,
    Motor,
    MotorCapabilities,
    MotorCapability,
    MotorKind,
    MotorState,
    MotorTopology,
    RebootRequest,
    RebootResponse,
    SetGripperRequest,
    SetGripperResponse,
    SetTorqueRequest,
    SetTorqueResponse,
    TopologyRequest,
    TorqueChanged,
)
from modules.motor.drivers.mock import MockMotorBackend
from modules.motor.layout import MotorSpec
from modules.motor.module import MotorDriverModule

_LOCAL_CFG = {"mode": "peer", "scouting": {"multicast": {"enabled": False}}}


def _layout(n_joints: int, gripper: bool) -> list[MotorSpec]:
    """test 용 합성 모터 레이아웃 (motors.yaml 한 줄 등가)."""
    motors = [
        MotorSpec(
            id=i + 1,
            name=f"j{i + 1}",
            model="MOCK",
            kind=MotorKind.JOINT,
            home=2048,
            limit_min=0,
            limit_max=4095,
            velocity_dps=0.0,
            acceleration_dpss=0.0,
        )
        for i in range(n_joints)
    ]
    if gripper:
        motors.append(
            MotorSpec(
                id=n_joints + 1,
                name="gripper",
                model="MOCK",
                kind=MotorKind.GRIPPER,
                home=2048,
                limit_min=0,
                limit_max=4095,
                velocity_dps=0.0,
                acceleration_dpss=0.0,
            )
        )
    return motors


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


# ─── 1. capability snapshot — driver self-declare (§7.3) ────


async def test_capabilities_service_relays_driver_self_declare(
    runtime: Runtime,
):
    driver = MockMotorBackend(_layout(6, True))
    runtime.add_module(MotorDriverModule, robot_id="so101_0", driver=driver)
    await runtime.start()

    res = await runtime.module_runtime.call(
        Motor.Service.CAPABILITIES,
        CapabilitiesRequest(),
        MotorCapabilities,
        robot_id="so101_0",
    )
    assert MotorCapability.TORQUE_TOGGLE in res.flags
    assert MotorCapability.REBOOT in res.flags


# ─── 2. topology — "무엇이 존재하는가" (§7.2 — consumer-driven) ─────


async def test_topology_service_returns_motor_list(runtime: Runtime):
    driver = MockMotorBackend(_layout(6, True))
    runtime.add_module(MotorDriverModule, robot_id="so101_0", driver=driver)
    await runtime.start()

    res = await runtime.module_runtime.call(
        Motor.Service.GET_TOPOLOGY,
        TopologyRequest(),
        MotorTopology,
        robot_id="so101_0",
    )
    # 6 joint + 1 gripper
    assert len(res.motors) == 7
    assert all(m.kind == MotorKind.JOINT for m in res.motors[:6])
    assert res.motors[-1].kind == MotorKind.GRIPPER
    # has_gripper / joint_count = derived
    assert any(m.kind == MotorKind.GRIPPER for m in res.motors)
    assert sum(1 for m in res.motors if m.kind == MotorKind.JOINT) == 6


async def test_topology_no_gripper_when_driver_says_no(runtime: Runtime):
    driver = MockMotorBackend(_layout(5, False))
    runtime.add_module(MotorDriverModule, robot_id="omx_f_0", driver=driver)
    await runtime.start()

    res = await runtime.module_runtime.call(
        Motor.Service.GET_TOPOLOGY,
        TopologyRequest(),
        MotorTopology,
        robot_id="omx_f_0",
    )
    assert len(res.motors) == 5
    assert all(m.kind == MotorKind.JOINT for m in res.motors)
    assert not any(m.kind == MotorKind.GRIPPER for m in res.motors)


# ─── 3. set_torque + TorqueChanged event broadcast ───────


async def test_set_torque_broadcasts_torque_changed_event(runtime: Runtime):
    received: list[TorqueChanged] = []
    done = threading.Event()

    class Listener:
        def __init__(self, runtime: ModuleRuntime):
            self.runtime = runtime

        @subscriber(Motor.Event.TORQUE_CHANGED)
        def on_changed(self, event: TorqueChanged) -> None:
            received.append(event)
            done.set()

    driver = MockMotorBackend(_layout(6, True))
    runtime.add_module(MotorDriverModule, robot_id="so101_0", driver=driver)
    runtime.add_module(Listener)
    await runtime.start()

    res = await runtime.module_runtime.call(
        Motor.Service.SET_TORQUE,
        SetTorqueRequest(enabled=True),
        SetTorqueResponse,
        robot_id="so101_0",
    )
    assert res.ok is True
    assert done.wait(timeout=2.0)
    assert received == [TorqueChanged(robot_id="so101_0", enabled=True)]


# ─── 3b. Motor.Stream.STATE — driver state 별도 stream (계층 분리) ─────


async def test_driver_state_stream_publishes_torque_enabled(runtime: Runtime):
    """§B 결정 — driver control state 는 JointState 와 분리된 자기 stream 자리.
    mount 직후 self-describing (초기 값 즉시 관찰 가능) — event chicken-and-egg 해소."""
    received: list[MotorState] = []

    class Listener:
        def __init__(self, runtime: ModuleRuntime):
            self.runtime = runtime

        @subscriber(Motor.Stream.STATE)
        def on_state(self, event: MotorState) -> None:
            received.append(event)

    driver = MockMotorBackend(_layout(6, True))
    runtime.add_module(MotorDriverModule, robot_id="so101_0", driver=driver)
    runtime.add_module(Listener)
    await runtime.start()

    # 5Hz publish → 1 frame ~= 200ms. 1s 안 자연 여러 개 박힘.
    for _ in range(30):
        if received:
            break
        await asyncio.sleep(0.05)
    assert received, "MotorState 첫 frame 안 옴 — mount 자기술 실패"
    assert received[0].torque_enabled is False  # 초기 = torque OFF

    # set_torque(True) 이후 다음 frame 은 torque_enabled=True
    await runtime.module_runtime.call(
        Motor.Service.SET_TORQUE,
        SetTorqueRequest(enabled=True),
        SetTorqueResponse,
        robot_id="so101_0",
    )
    baseline = len(received)
    for _ in range(30):
        if len(received) > baseline and received[-1].torque_enabled:
            break
        await asyncio.sleep(0.05)
    assert received[-1].torque_enabled is True, (
        "set_torque 후에도 MotorState.torque_enabled=False 유지 — 재발행 실패"
    )


# ─── 4. reboot + set_gripper service ─────────────────────


async def test_reboot_service(runtime: Runtime):
    driver = MockMotorBackend(_layout(6, True))
    runtime.add_module(MotorDriverModule, robot_id="so101_0", driver=driver)
    await runtime.start()

    res = await runtime.module_runtime.call(
        Motor.Service.REBOOT,
        RebootRequest(),
        RebootResponse,
        robot_id="so101_0",
    )
    assert res.ok is True


async def test_set_gripper_writes_to_driver(runtime: Runtime):
    driver = MockMotorBackend(_layout(6, True))
    runtime.add_module(MotorDriverModule, robot_id="so101_0", driver=driver)
    await runtime.start()

    res = await runtime.module_runtime.call(
        Motor.Service.SET_GRIPPER,
        SetGripperRequest(position_raw=3000),
        SetGripperResponse,
        robot_id="so101_0",
    )
    assert res.ok is True
    # mock driver — gripper position 갱신 검증
    assert driver.read_positions()[-1] == 3000


# ─── 4b. command stream (Motion → Motor, raw 위치) ───────


async def test_command_stream_writes_to_driver(runtime: Runtime):
    driver = MockMotorBackend(_layout(6, True))
    runtime.add_module(MotorDriverModule, robot_id="so101_0", driver=driver)
    await runtime.start()

    target = [1000, 1100, 1200, 1300, 1400, 1500]
    runtime.module_runtime.publish(
        Motor.Stream.COMMAND,
        JointCommand(
            robot_id="so101_0", seq=0, timestamp_unix=time.time(), positions_raw=target
        ),
    )
    for _ in range(50):
        await asyncio.sleep(0.02)
        if driver.read_positions()[:6] == target:
            break
    assert driver.read_positions()[:6] == target  # arm 갱신
    assert driver.read_positions()[6] == 2048  # gripper 미변 (home)


# ─── 5. state stream — 20Hz publish + seq monotonic + timestamp_unix ──


async def test_state_stream_publishes_with_seq_and_timestamp(runtime: Runtime):
    """§8.5 invariant — 모든 stream payload 에 seq + timestamp_unix 박힘."""
    received: list[JointState] = []

    class Listener:
        def __init__(self, runtime: ModuleRuntime):
            self.runtime = runtime

        @subscriber(Motor.Stream.RAW_STATE)
        def on_state(self, event: JointState) -> None:
            received.append(event)

    driver = MockMotorBackend(_layout(6, True))
    runtime.add_module(MotorDriverModule, robot_id="so101_0", driver=driver)
    runtime.add_module(Listener)
    await runtime.start()

    # 20Hz publish → 3 frame 박힘 = ~150ms. async wait 0.5s 안 자연.
    for _ in range(50):
        if len(received) >= 3:
            break
        await asyncio.sleep(0.05)

    assert len(received) >= 3, f"3+ frame 박혀야 — received {len(received)}"

    # seq monotonic (§8.5)
    seqs = [e.seq for e in received[:3]]
    assert seqs == sorted(seqs), f"seq must be monotonic, got {seqs}"
    # timestamp_unix invariant — 양수 + 현재 시각 근처
    now = time.time()
    for e in received[:3]:
        assert 0 < e.timestamp_unix < now + 1.0
    # positions_raw — mock 의 초기 중심 raw
    assert all(p == 2048 for p in received[0].positions_raw)


# ─── 6. multi-robot — per-robot 독립 stream ─────────────


async def test_multi_robot_independent_state_streams(runtime: Runtime):
    """robot-scoped 두 인스턴스 — 각자 자기 robot_id 의 stream publish."""
    so101_events: list[JointState] = []
    omx_events: list[JointState] = []

    class Listener:
        def __init__(self, runtime: ModuleRuntime):
            self.runtime = runtime

        @subscriber(Motor.Stream.RAW_STATE)
        def on_state(self, event: JointState) -> None:
            if event.robot_id == "so101_0":
                so101_events.append(event)
            elif event.robot_id == "omx_f_0":
                omx_events.append(event)

    so101_driver = MockMotorBackend(_layout(6, True))
    omx_driver = MockMotorBackend(_layout(5, True))
    runtime.add_module(
        MotorDriverModule, robot_id="so101_0", driver=so101_driver,
    )
    runtime.add_module(
        MotorDriverModule, robot_id="omx_f_0", driver=omx_driver,
    )
    runtime.add_module(Listener)
    await runtime.start()

    for _ in range(50):
        if so101_events and omx_events:
            break
        await asyncio.sleep(0.05)

    assert so101_events, "so101 stream 안 받음"
    assert omx_events, "omx stream 안 받음"
    # 각 robot 의 joint count 가 자기 driver 의 topology 자연 반영
    assert len(so101_events[0].positions_raw) == 7  # 6 + gripper
    assert len(omx_events[0].positions_raw) == 6    # 5 + gripper


# ─── 7. lifecycle — driver.open() / close() 호출 검증 ────


async def test_driver_open_close_lifecycle(transport: ZenohTransport):
    open_calls: list[None] = []
    close_calls: list[None] = []

    class SpyDriver(MockMotorBackend):
        def open(self) -> None:
            open_calls.append(None)
            super().open()

        def close(self) -> None:
            close_calls.append(None)
            super().close()

    rt = Runtime(transport)
    rt.add_module(MotorDriverModule, robot_id="so101_0", driver=SpyDriver(_layout(6, True)))
    await rt.start()
    assert open_calls == [None]
    await rt.stop()
    assert close_calls == [None]


# ─── 8. state loop cancel — stop() 박힌 후 publish 자리 X ──


async def test_state_loop_stops_after_module_stop(transport: ZenohTransport):
    received: list[JointState] = []

    class Listener:
        def __init__(self, runtime: ModuleRuntime):
            self.runtime = runtime

        @subscriber(Motor.Stream.RAW_STATE)
        def on_state(self, event: JointState) -> None:
            received.append(event)

    rt = Runtime(transport)
    rt.add_module(MotorDriverModule, robot_id="so101_0", driver=MockMotorBackend(_layout(6, True)))
    rt.add_module(Listener)
    await rt.start()

    await asyncio.sleep(0.15)  # 2-3 frame 박힐 자리
    count_at_stop = len(received)
    await rt.stop()

    await asyncio.sleep(0.2)  # stop 후 더 안 박힘
    count_after_stop = len(received)

    # stop 후 잠시 더 박힐 수 있음 (in-flight) — 단 큰 차이 없어야
    assert count_after_stop - count_at_stop < 3, (
        f"stop() 후에도 publish 지속 — {count_at_stop} → {count_after_stop}"
    )
