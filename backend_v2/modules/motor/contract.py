"""Motor domain — public contract surface.

backend_v2_modules.md §1.1 #1 (MotorDriver) + §7 (Capability) + §8.5 (Stream
seq/timestamp invariant) 정합. 외부 Module / TS gen / contract viewer 의
read 대상.
"""

from __future__ import annotations

from enum import StrEnum

from pydantic import BaseModel


class Motor:
    """Motor 도메인 — Service / Stream / Event nested StrEnum."""

    class Service(StrEnum):
        # what is possible (§7)
        CAPABILITIES = "srv/motor/{robot_id}/capabilities"

        # robot topology (joint_count 등 — capability 와 분리, §7.6 invariant)
        GET_TOPOLOGY = "srv/motor/{robot_id}/topology"

        # control
        SET_TORQUE = "srv/motor/{robot_id}/set_torque"
        REBOOT = "srv/motor/{robot_id}/reboot"
        SET_GRIPPER = "srv/motor/{robot_id}/set_gripper"

    class Stream(StrEnum):
        # 20Hz raw motor state (rad / fk 변환은 Motion Module 책임)
        RAW_STATE = "stream/motor/{robot_id}/raw_state"

    class Event(StrEnum):
        TORQUE_CHANGED = "event/motor/{robot_id}/torque_changed"


# ─── capability ─────────────────────────────────────────────────────


class MotorCapability(StrEnum):
    """flags only (§7.1 invariant — what is possible, not how configured)."""

    TORQUE_TOGGLE = "torque_toggle"
    REBOOT = "reboot"
    GRIPPER = "gripper"
    POSITION_PID = "position_pid"


class MotorCapabilities(BaseModel):
    """static fact — vendor / 모델 별 다름. driver self-declare (§7.3)."""

    flags: set[MotorCapability]


# ─── topology (capability 와 분리 — int / list 같은 metadata 자리) ─


class MotorTopology(BaseModel):
    """robot 의 motor 구성. joint_count / motor_ids — capability 어휘 아님."""

    joint_count: int
    motor_ids: list[int]
    has_gripper: bool


# ─── request / response ────────────────────────────────────────────


class CapabilitiesRequest(BaseModel):
    pass


class TopologyRequest(BaseModel):
    pass


class SetTorqueRequest(BaseModel):
    enabled: bool


class SetTorqueResponse(BaseModel):
    ok: bool


class RebootRequest(BaseModel):
    pass


class RebootResponse(BaseModel):
    ok: bool


class SetGripperRequest(BaseModel):
    position_raw: int  # Dynamixel/Feetech raw int (0..4095)


class SetGripperResponse(BaseModel):
    ok: bool


# ─── stream payload (seq + timestamp_unix invariant — §8.5) ────────


class JointState(BaseModel):
    """20Hz motor state. raw int — rad 변환은 consumer (Motion) 책임.

    seq / timestamp_unix invariant — frontend reconnect / lag / out-of-order
    detection 자리 (§8.5).
    """

    robot_id: str
    seq: int
    timestamp_unix: float
    positions_raw: list[int]  # 0..4095, motor_ids 순
    velocities_raw: list[int] | None = None  # 모델 / 모터 별
    loads_raw: list[int] | None = None  # torque sensor 있는 모델만


# ─── event payload ─────────────────────────────────────────────────


class TorqueChanged(BaseModel):
    robot_id: str
    enabled: bool
