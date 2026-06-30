"""Mock MotorBackend — hardware-less 자리. mock.yaml + test 자리.

레이아웃(MotorSpec list)을 받아 합성 — real 과 동일 layout SSOT (motors.yaml).
"""

from __future__ import annotations

from ..contract import (
    MotorCapabilities,
    MotorCapability,
    MotorInfo,
    MotorKind,
    MotorTopology,
)
from ..layout import MotorSpec


class MockMotorBackend:
    """In-process mock — motors 레이아웃대로 합성 state, hardware 없이 동작."""

    def __init__(self, motors: list[MotorSpec]) -> None:
        self._motors = list(motors)
        # 초기 position = 각 모터 home (real 첫 read 와 동형)
        self._positions: list[int] = [m.home for m in self._motors]
        self._torque_enabled = False

    # ── self-declare ──

    def capabilities(self) -> MotorCapabilities:
        # mock = TORQUE_TOGGLE + REBOOT baseline (vendor 별 추가는 실 driver 자리)
        return MotorCapabilities(
            flags={MotorCapability.TORQUE_TOGGLE, MotorCapability.REBOOT},
        )

    def topology(self) -> MotorTopology:
        return MotorTopology(
            motors=[MotorInfo(id=m.id, kind=m.kind) for m in self._motors]
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
        # 마지막 motor 가 gripper kind 일 때만 (Topology 위 derived)
        if self._motors and self._motors[-1].kind == MotorKind.GRIPPER:
            self._positions[-1] = position_raw

    def write_positions(self, positions_raw: list[int]) -> None:
        # mock — 즉시 갱신 (실 motor latency 없음)
        for i, p in enumerate(positions_raw):
            if i < len(self._positions):
                self._positions[i] = p
