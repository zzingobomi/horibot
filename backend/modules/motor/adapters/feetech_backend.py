"""FeetechBackend — `scservo_sdk` 기반 MotorBackend 구현체.

DynamixelBackend (adapters/dynamixel_backend.py) 와 동형 — 같은 Protocol method +
legacy aliases. motor_node 가 import 만 바꿔도 동작 (실제로는 RobotRegistry factory
가 robots.yaml 의 motor_backend 따라 자동 분기).

multi_robot_architecture.md §3.3 + so101_6dof_plan.md §6 참조.
"""

from __future__ import annotations

from modules.motor.adapters.feetech_driver import FeetechDriver
from modules.motor.motor_config import MotorConfig
from modules.motor.backend import MotorCommError


class FeetechBackend:
    """`scservo_sdk` adapter. MotorBackend Protocol 만족."""

    def __init__(self, port: str, motors: list[MotorConfig]):
        self._driver = FeetechDriver(port, motors)

    # ─── Lifecycle ─────────────────────────────────────────

    def connect(self) -> None:
        if not self._driver.connect():
            raise MotorCommError(
                f"FeetechBackend connect 실패: port={self._driver.port}"
            )

    def disconnect(self) -> None:
        self._driver.disconnect()

    @property
    def motor_ids(self) -> list[int]:
        return self._driver.motor_ids

    # ─── Protocol API (clean names) ────────────────────────

    def read_positions(self) -> dict[int, int]:
        return self._driver.get_present_positions()

    def read_currents(self) -> dict[int, int]:
        # STS Present_Load (signed 2byte) — current proxy. STS = current control X.
        return self._driver.get_present_loads()

    def read_velocities(self) -> dict[int, int]:
        # TODO: sync read 추가. 지금은 모터별 single read loop (DynamixelBackend 동일).
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
        """STS PID = EEPROM. read-first-then-write 로 wear 방지.
        so101_6dof_plan §6-6 (c) 패턴.
        """
        if p is None and i is None and d is None:
            return
        cur_p, cur_i, cur_d = self._driver.read_position_pid(motor_id)
        if (
            (p is None or p == cur_p)
            and (i is None or i == cur_i)
            and (d is None or d == cur_d)
        ):
            return  # no change → write 0회 (EEPROM wear 보호)
        self._driver.set_position_pid(motor_id, p, i, d)

    def set_goal_current(self, motor_id: int, current: int) -> None:
        # STS = position loop only. no-op (DynamixelBackend 와 인터페이스 통일).
        self._driver.set_goal_current(motor_id, current)

    def reboot(self, motor_id: int) -> None:
        self._driver.reboot(motor_id)

    # ─── Legacy aliases (DynamixelBackend 와 동일) ─────────
    # motor_node.py 가 직접 호출하는 이름들. caller migration 완료 시 제거 가능.

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
