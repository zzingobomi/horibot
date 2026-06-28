"""Mock MotorBackend — hardware-less 자리. host_mock.yaml + test 자리."""

from __future__ import annotations

from ..contract import (
    MotorCapabilities,
    MotorCapability,
    MotorTopology,
)


class MockMotorBackend:
    """In-process mock — 합성 motor state, hardware 없이 동작."""

    def __init__(self, joint_count: int = 6, has_gripper: bool = True) -> None:
        self._joint_count = joint_count
        self._has_gripper = has_gripper
        # motor_ids = joint_count + (1 if gripper else 0)
        self._motor_ids = list(range(joint_count + (1 if has_gripper else 0)))
        # 중심 raw int (0..4095). Dynamixel/Feetech 컨벤션
        self._positions: list[int] = [2048] * len(self._motor_ids)
        self._torque_enabled = False

    # ── self-declare ──

    def capabilities(self) -> MotorCapabilities:
        flags = {MotorCapability.TORQUE_TOGGLE, MotorCapability.REBOOT}
        if self._has_gripper:
            flags.add(MotorCapability.GRIPPER)
        return MotorCapabilities(flags=flags)

    def topology(self) -> MotorTopology:
        return MotorTopology(
            joint_count=self._joint_count,
            motor_ids=list(self._motor_ids),
            has_gripper=self._has_gripper,
        )

    # ── lifecycle ──

    def open(self) -> None:
        pass

    def close(self) -> None:
        pass

    # ── read ──

    def read_positions(self) -> list[int]:
        return list(self._positions)

    def read_velocities(self) -> list[int] | None:
        return None  # mock — velocity 측정 없음

    def read_loads(self) -> list[int] | None:
        return None  # mock — load 측정 없음

    # ── write ──

    def set_torque(self, enabled: bool) -> None:
        self._torque_enabled = enabled

    def reboot(self) -> None:
        # mock — 실 reboot 없음
        pass

    def set_gripper(self, position_raw: int) -> None:
        if self._has_gripper:
            self._positions[-1] = position_raw

    def write_positions(self, positions_raw: list[int]) -> None:
        # mock — 즉시 갱신 (실 motor latency 없음)
        for i, p in enumerate(positions_raw):
            if i < len(self._positions):
                self._positions[i] = p
