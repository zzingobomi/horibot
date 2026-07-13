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
        RAW_STATE = "stream/motor/{robot_id}/raw_state"  # 20Hz kinematic (JointState)
        STATE = "stream/motor/{robot_id}/state"  # 5Hz driver control state (MotorState)
        # Motion → Motor 위치 명령 (raw). 100Hz fire-and-forget. Motion 이
        # rad→raw 변환 후 publish, MotorDriver 가 write_positions (§4 — raw↔rad = Motion).
        COMMAND = "stream/motor/{robot_id}/command"

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
    """빈 응답 — 성공 = 반환, driver 실패 = 예외 (RemoteError 로 전파).

    옛 ok 필드는 항상 True 만 반환되던 죽은 필드 (실패는 애초에 raise 경로) —
    2026-07-13 예외/데이터 기준 정리에서 제거."""


# ─── stream payload (seq + timestamp_unix invariant — §8.5) ────────


class JointState(BaseModel):
    """20Hz motor kinematic state. raw int — rad 변환은 consumer (Motion) 책임.

    seq / timestamp_unix invariant — frontend reconnect / lag / out-of-order
    detection 자리 (§8.5).

    도메인 계층 분리 — 이 stream 은 *joint kinematic* (position/velocity/load) 만.
    driver control state (torque_enabled / mode / error) 는 `Motor.Stream.STATE`.
    """

    robot_id: str
    seq: int
    timestamp_unix: float
    positions_raw: list[int]  # 0..4095, motor_ids 순
    velocities_raw: list[int] | None = None  # 모델 / 모터 별
    loads_raw: list[int] | None = None  # torque sensor 있는 모델만


class MotorState(BaseModel):
    """Driver control state — mount 직후 self-describing (초기 latch).

    JointState (kinematic) 와 계층 분리 — driver 가 소유하는 flag/mode/error 는
    여기. 변화 signal 은 `TORQUE_CHANGED` event (state ≠ event 원칙).

    현재 field = torque_enabled 하나뿐이지만 확장 예정 (control_mode / error /
    homed 등) 자리로 미리 stream 을 뽑아둠 — 나중에 JointState 에서 뽑아내면
    frontend contract / test / gen types / subscriber 다 재배선.
    """

    robot_id: str
    seq: int
    timestamp_unix: float
    torque_enabled: bool


class JointCommand(BaseModel):
    """Motion → Motor 위치 명령 (raw). arm joint 만 (gripper = SET_GRIPPER)."""

    robot_id: str
    seq: int
    timestamp_unix: float
    positions_raw: list[int]  # arm joint raw, motors.yaml arm 순


# ─── event payload ─────────────────────────────────────────────────


class TorqueChanged(BaseModel):
    robot_id: str
    enabled: bool
