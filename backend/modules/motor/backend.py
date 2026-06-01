"""MotorBackend Protocol — Dynamixel / Feetech SDK 통합 인터페이스.

multi_robot_architecture.md §3.3 참조.

책임:
- motor SDK 의 통합 인터페이스 (Dynamixel/Feetech 의 wire protocol 차이 흡수)
- raw motor unit (int 0..4095, current raw 등) 에서 동작
- rad/m 단위 변환 / joint_offset 적용은 caller (`JointStateCache`, `MotionExecutor`)

Phase 1: DynamixelBackend (XL430/XL330 + OpenRB-150)
Phase 2: FeetechBackend (STS3215/3250 + Waveshare) — SO-101 도착 시
"""

from __future__ import annotations

from typing import Protocol


class MotorBackendError(Exception):
    """모터 backend 관련 예외 base."""


class MotorCommError(MotorBackendError):
    """버스 통신 실패 (timeout, checksum, etc.)."""


class MotorBackend(Protocol):
    # ─── Lifecycle ─────────────────────────────────────────
    def connect(self) -> None:
        """포트 열고 sync read/write 초기화. 실패 시 MotorCommError raise."""
        ...

    def disconnect(self) -> None:
        """torque off + 포트 close. 멱등."""
        ...

    @property
    def motor_ids(self) -> list[int]:
        """이 backend 가 관리하는 모터 ID 목록."""
        ...

    # ─── State read (sync read — 전체 한 번에) ─────────────
    def read_positions(self) -> dict[int, int]:
        """전체 모터 raw position (0..4095). 통신 실패 시 빈 dict."""
        ...

    def read_currents(self) -> dict[int, int]:
        """raw current (signed). 모터 모델별 단위 (XL330=mA, XL430=‰) 는 caller 해석."""
        ...

    def read_velocities(self) -> dict[int, int]:
        """raw velocity (signed)."""
        ...

    # ─── Position command ──────────────────────────────────
    def write_positions(self, cmd: dict[int, int]) -> None:
        """전체 모터 raw goal position sync write. limit clamp + reverse 는 backend 내부."""
        ...

    # ─── Torque ───────────────────────────────────────────
    def set_torque(self, ids: list[int], enable: bool) -> None:
        """ids 의 torque enable/disable. ids=[] no-op."""
        ...

    # ─── Profile (max velocity / acceleration) ────────────
    def write_profile_velocities(self, vel: dict[int, int]) -> None: ...
    def write_profile_accelerations(self, acc: dict[int, int]) -> None: ...

    # ─── PID ──────────────────────────────────────────────
    def configure_pid(
        self,
        motor_id: int,
        p: int | None = None,
        i: int | None = None,
        d: int | None = None,
    ) -> None:
        """raw PID gain. None 인 값은 skip. SDK 미지원 값은 무시."""
        ...

    # ─── Gripper (current-based position) ─────────────────
    def set_goal_current(self, motor_id: int, current: int) -> None:
        """그리퍼 force control 용. backend 미지원 시 NotImplementedError raise."""
        ...

    # ─── Maintenance ──────────────────────────────────────
    def reboot(self, motor_id: int) -> None: ...
