"""DynamixelBackend — `dynamixel-sdk` 기반 MotorBackend 구현체.

multi_robot_architecture.md §3.3 참조.

기존 [DynamixelDriver](../../dynamixel/driver.py) 의 raw SDK wrap 을 그대로 활용.
이 adapter 는 `MotorBackend` Protocol 메서드 (clean names) + 기존 legacy method
aliases 양쪽 다 제공 → motor_node 같은 caller 가 import 만 바꿔도 동작.

caller migration 은 점진 — 새 코드는 Protocol method (`read_positions` 등) 사용,
기존 코드는 legacy alias (`get_present_positions` 등) 그대로.
"""

from __future__ import annotations

from modules.dynamixel.driver import DynamixelDriver
from modules.dynamixel.motor_config import MotorConfig
from modules.motor.backend import MotorCommError


class DynamixelBackend:
    """`dynamixel-sdk` adapter. MotorBackend Protocol 만족."""

    def __init__(self, port: str, motors: list[MotorConfig]):
        self._driver = DynamixelDriver(port, motors)

    # ─── Lifecycle ─────────────────────────────────────────

    def connect(self) -> None:
        if not self._driver.connect():
            raise MotorCommError(f"DynamixelBackend connect 실패: port={self._driver.port}")

    def disconnect(self) -> None:
        self._driver.disconnect()

    @property
    def motor_ids(self) -> list[int]:
        return self._driver.motor_ids

    # ─── Protocol API (clean names) ────────────────────────

    def read_positions(self) -> dict[int, int]:
        return self._driver.get_present_positions()

    def read_currents(self) -> dict[int, int]:
        # Dynamixel 의 PRESENT_LOAD (signed, ‰ on XL430 / mA on XL330) — current proxy
        return self._driver.get_present_loads()

    def read_velocities(self) -> dict[int, int]:
        # TODO: sync read 추가. 지금은 single-motor get_present_velocity loop.
        return {
            mid: self._driver.get_present_velocity(mid)
            for mid in self._driver.motor_ids
        }

    def write_positions(self, cmd: dict[int, int]) -> None:
        self._driver.set_goal_positions_sync(cmd)

    def set_torque(self, ids: list[int], enable: bool) -> None:
        if not ids:
            return
        for mid in ids:
            if enable:
                self._driver.torque_enable(mid)
            else:
                self._driver.torque_disable(mid)

    def write_profile_velocities(self, vel: dict[int, int]) -> None:
        self._driver.set_profile_velocities_sync(vel)

    def write_profile_accelerations(self, acc: dict[int, int]) -> None:
        self._driver.set_profile_accelerations_sync(acc)

    def configure_pid(
        self,
        motor_id: int,
        p: int | None = None,
        i: int | None = None,
        d: int | None = None,
    ) -> None:
        self._driver.set_position_pid(motor_id, p, i, d)

    def set_goal_current(self, motor_id: int, current: int) -> None:
        self._driver.set_goal_current(motor_id, current)

    def reboot(self, motor_id: int) -> None:
        self._driver.reboot(motor_id)

    # ─── Legacy aliases (기존 motor_node 가 사용) ──────────
    # caller migration 완료 후 제거 가능. 지금은 호환성 위해 유지.

    def torque_enable_all(self) -> None:
        self.set_torque(self.motor_ids, True)

    def torque_disable_all(self) -> None:
        self.set_torque(self.motor_ids, False)

    def get_present_positions(self) -> dict[int, int]:
        return self.read_positions()

    def get_present_loads(self) -> dict[int, int]:
        return self.read_currents()

    def get_present_velocity(self, motor_id: int) -> int:
        return self._driver.get_present_velocity(motor_id)

    def set_goal_positions_sync(self, positions: dict[int, int]) -> None:
        self.write_positions(positions)

    def set_goal_position(self, motor_id: int, position: int) -> None:
        self._driver.set_goal_position(motor_id, position)

    def set_profile_velocity(self, motor_id: int, velocity: int) -> None:
        self._driver.set_profile_velocity(motor_id, velocity)

    def set_profile_acceleration(self, motor_id: int, acceleration: int) -> None:
        self._driver.set_profile_acceleration(motor_id, acceleration)

    def set_profile_velocities_sync(self, velocities: dict[int, int]) -> None:
        self.write_profile_velocities(velocities)

    def set_profile_accelerations_sync(self, accelerations: dict[int, int]) -> None:
        self.write_profile_accelerations(accelerations)

    def set_position_pid(
        self,
        motor_id: int,
        p: int | None = None,
        i: int | None = None,
        d: int | None = None,
    ) -> None:
        self.configure_pid(motor_id, p, i, d)
