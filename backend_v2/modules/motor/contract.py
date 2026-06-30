from __future__ import annotations

from enum import StrEnum

from pydantic import BaseModel


class Motor:
    class Service(StrEnum):
        CAPABILITIES = "srv/motor/{robot_id}/capabilities"
        GET_TOPOLOGY = "srv/motor/{robot_id}/topology"

        # control
        SET_TORQUE = "srv/motor/{robot_id}/set_torque"
        REBOOT = "srv/motor/{robot_id}/reboot"
        SET_GRIPPER = "srv/motor/{robot_id}/set_gripper"

    class Stream(StrEnum):
        RAW_STATE = "stream/motor/{robot_id}/raw_state"

    class Event(StrEnum):
        TORQUE_CHANGED = "event/motor/{robot_id}/torque_changed"


# ─── capability ─────────────────────────────────────────────────────


class MotorCapability(StrEnum):
    # GRIPPER 박지 X — Topology 위 `any(m.kind == GRIPPER)` derived.
    # POSITION_PID 박지 X — MotorBackend Protocol baseline.
    TORQUE_TOGGLE = "torque_toggle"
    REBOOT = "reboot"
    VELOCITY_CONTROL = "velocity_control"
    CURRENT_CONTROL = "current_control"
    HOMING = "homing"


class MotorCapabilities(BaseModel):
    flags: set[MotorCapability]


# ─── topology — "무엇이 존재하는가" (Motion 이 wire-level 직접 소비) ──


class MotorKind(StrEnum):
    JOINT = "joint"
    GRIPPER = "gripper"
    RAIL = "rail"
    TOOL = "tool"


class MotorInfo(BaseModel):
    id: int
    kind: MotorKind


class MotorTopology(BaseModel):
    motors: list[MotorInfo]


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
