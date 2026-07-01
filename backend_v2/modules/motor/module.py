"""MotorDriverModule — robot-scoped Hardware Layer Module.

backend_v2_modules.md §1.1 #1 (MotorDriver) + §11 Build order Step A.

책임:
- driver Protocol relay (capabilities / topology / set_torque / reboot / gripper)
- 20Hz raw state publish (Motor.Stream.RAW_STATE)
- TorqueChanged event publish (set_torque 시)

driver 종류 모름 — Dynamixel / Feetech / mock 다 Protocol 위 동작.
"""

from __future__ import annotations

import asyncio
import logging
import time

from framework.contract.publisher import publishes
from framework.contract.service import service
from framework.contract.subscriber import subscriber
from framework.runtime.api import ModuleRuntime

from .contract import (
    CapabilitiesRequest,
    JointCommand,
    JointState,
    Motor,
    MotorCapabilities,
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
from .drivers.protocol import MotorBackend

logger = logging.getLogger(__name__)

# 20Hz kinematic state publish. backend/ 의 motor_node 와 동일 (network bandwidth 자리)
_STATE_PUBLISH_HZ = 20.0
# driver control state — 저빈도 (변화 드묾, mount 직후 self-describing 목적).
_DRIVER_STATE_HZ = 5.0


@publishes(
    (Motor.Stream.RAW_STATE, JointState),
    (Motor.Stream.STATE, MotorState),
    (Motor.Event.TORQUE_CHANGED, TorqueChanged),
)
class MotorDriverModule:
    """robot-scoped Module — robot 의 motor hardware adapter relay."""

    def __init__(
        self,
        runtime: ModuleRuntime,
        robot_id: str,
        driver: MotorBackend,
    ) -> None:
        self.runtime = runtime
        self.robot_id = robot_id
        self._driver = driver

        # boot 1회 cache — capability 가 static (§7.3 invariant)
        self._capabilities: MotorCapabilities | None = None
        self._topology: MotorTopology | None = None

        # state stream seq counter (§8.5 invariant)
        self._seq = 0
        self._driver_state_seq = 0
        self._state_task: asyncio.Task[None] | None = None
        self._driver_state_task: asyncio.Task[None] | None = None
        self._stop_requested = False

    # ── lifecycle ─────────────────────────────────────────────

    async def start(self) -> None:
        self._driver.open()
        # capability / topology cache — driver self-declare 의 boot 1회 read
        self._capabilities = self._driver.capabilities()
        self._topology = self._driver.topology()

        self._stop_requested = False
        self._state_task = asyncio.create_task(self._state_loop())
        self._driver_state_task = asyncio.create_task(self._driver_state_loop())

    async def stop(self) -> None:
        self._stop_requested = True
        for attr in ("_state_task", "_driver_state_task"):
            task = getattr(self, attr)
            if task is not None:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
                setattr(self, attr, None)
        self._driver.close()

    # ── service handlers ──────────────────────────────────────

    @service(Motor.Service.CAPABILITIES)
    def get_capabilities(self, req: CapabilitiesRequest) -> MotorCapabilities:
        assert self._capabilities is not None, "start() 박힌 후 호출"
        return self._capabilities

    @service(Motor.Service.GET_TOPOLOGY)
    def get_topology(self, req: TopologyRequest) -> MotorTopology:
        assert self._topology is not None, "start() 박힌 후 호출"
        return self._topology

    @service(Motor.Service.SET_TORQUE)
    def set_torque(self, req: SetTorqueRequest) -> SetTorqueResponse:
        self._driver.set_torque(req.enabled)
        self.runtime.publish(
            Motor.Event.TORQUE_CHANGED,
            TorqueChanged(robot_id=self.robot_id, enabled=req.enabled),
        )
        return SetTorqueResponse(ok=True)

    @service(Motor.Service.REBOOT)
    def reboot(self, req: RebootRequest) -> RebootResponse:
        self._driver.reboot()
        return RebootResponse(ok=True)

    @service(Motor.Service.SET_GRIPPER)
    def set_gripper(self, req: SetGripperRequest) -> SetGripperResponse:
        self._driver.set_gripper(req.position_raw)
        return SetGripperResponse(ok=True)

    # ── command subscriber (Motion → Motor, raw 위치) ─────────

    @subscriber(Motor.Stream.COMMAND)
    def on_command(self, cmd: JointCommand) -> None:
        # robot-scoped — wildcard subscribe 후 self-filter (CameraDecoded 동형)
        if cmd.robot_id != self.robot_id:
            return
        try:
            self._driver.write_positions(cmd.positions_raw)
        except Exception:
            logger.exception("MotorDriver write_positions 실패 robot_id=%s", self.robot_id)

    # ── driver control state loop (5Hz) — self-describing latch ──

    async def _driver_state_loop(self) -> None:
        interval = 1.0 / _DRIVER_STATE_HZ
        try:
            while not self._stop_requested:
                try:
                    self.runtime.publish(
                        Motor.Stream.STATE,
                        MotorState(
                            robot_id=self.robot_id,
                            seq=self._driver_state_seq,
                            timestamp_unix=time.time(),
                            torque_enabled=self._driver.get_torque_enabled(),
                        ),
                    )
                    self._driver_state_seq += 1
                except Exception:
                    logger.exception(
                        "MotorDriver STATE publish 실패 robot_id=%s", self.robot_id
                    )
                await asyncio.sleep(interval)
        except asyncio.CancelledError:
            pass

    # ── state stream loop (20Hz) ──────────────────────────────

    async def _state_loop(self) -> None:
        interval = 1.0 / _STATE_PUBLISH_HZ
        try:
            while not self._stop_requested:
                try:
                    positions = self._driver.read_positions()
                    velocities = self._driver.read_velocities()
                    loads = self._driver.read_loads()
                    event = JointState(
                        robot_id=self.robot_id,
                        seq=self._seq,
                        timestamp_unix=time.time(),
                        positions_raw=positions,
                        velocities_raw=velocities,
                        loads_raw=loads,
                    )
                    self._seq += 1
                    self.runtime.publish(Motor.Stream.RAW_STATE, event)
                except Exception:
                    logger.exception(
                        "MotorDriver state publish 실패 robot_id=%s",
                        self.robot_id,
                    )
                await asyncio.sleep(interval)
        except asyncio.CancelledError:
            pass
