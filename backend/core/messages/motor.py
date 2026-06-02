"""Motor 노드 토픽 / 서비스 payload schema.

토픽:
- MOTOR_STATE_JOINT (publish) — MotorJointState
- MOTOR_CMD_JOINT   (subscribe) — MotorCmd

서비스 (request data / response data):
- MOTOR_ENABLE           — MotorEnableReq / MotorEnableRes
- MOTOR_REBOOT           — MotorRebootReq / EmptyData
- MOTOR_SET_PROFILE      — MotorSetProfileReq / EmptyData
- MOTOR_SET_PROFILE_ALL  — MotorSetProfileAllReq / EmptyData
- MOTOR_GET_CONFIG       — EmptyData / MotorGetConfigRes
- MOTOR_GRIPPER          — MotorGripperReq / EmptyData
"""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field


# ─── Topic: MOTOR_STATE_JOINT ────────────────────────────────────────


class MotorJoint(BaseModel):
    """state 토픽 한 모터 항목."""

    model_config = ConfigDict(extra="forbid")

    id: int
    name: str
    position: int  # raw 0..4095
    degree: float
    velocity: float = 0.0
    torque: float = 0.0
    load: int = 0


class MotorJointState(BaseModel):
    """MOTOR_STATE_JOINT publish 페이로드. STATE_PUBLISH_HZ 로 발행."""

    model_config = ConfigDict(extra="forbid")

    timestamp: float
    joints: list[MotorJoint]


# ─── Topic: MOTOR_CMD_JOINT ──────────────────────────────────────────


class MotorCmdJoint(BaseModel):
    """cmd 토픽 한 모터 명령. position 만 받음 (raw 0..4095)."""

    model_config = ConfigDict(extra="forbid")

    id: int
    position: int


class MotorCmd(BaseModel):
    """MOTOR_CMD_JOINT subscribe 페이로드. TrajectoryRunner 가 100Hz 로 발행."""

    model_config = ConfigDict(extra="forbid")

    timestamp: float = 0.0
    joints: list[MotorCmdJoint]


# ─── Service: MOTOR_ENABLE ────────────────────────────────────────────


class MotorEnableReq(BaseModel):
    model_config = ConfigDict(extra="forbid")
    enable: bool = True


class MotorEnableRes(BaseModel):
    model_config = ConfigDict(extra="forbid")
    enable: bool


# ─── Service: MOTOR_REBOOT ────────────────────────────────────────────


class MotorRebootReq(BaseModel):
    """id=None 이면 전 모터 reboot."""

    model_config = ConfigDict(extra="forbid")
    id: int | None = None


# ─── Service: MOTOR_SET_PROFILE (single motor) ────────────────────────


class MotorSetProfileReq(BaseModel):
    model_config = ConfigDict(extra="forbid")
    id: int
    velocity: int | None = None
    acceleration: int | None = None


# ─── Service: MOTOR_SET_PROFILE_ALL (multi motor) ─────────────────────


class MotorSetProfileAllReq(BaseModel):
    """ids=None 이면 driver.motor_ids 전체 적용."""

    model_config = ConfigDict(extra="forbid")
    ids: list[int] | None = None
    velocity: int = 0
    acceleration: int = 0


# ─── Service: MOTOR_GET_CONFIG ────────────────────────────────────────


class MotorLimit(BaseModel):
    model_config = ConfigDict(extra="forbid")
    min: int
    max: int


class MotorConfigItem(BaseModel):
    model_config = ConfigDict(extra="forbid")
    id: int
    name: str
    model: str
    mode: str
    home: int
    limit: MotorLimit


class MotorGetConfigRes(BaseModel):
    model_config = ConfigDict(extra="forbid")
    motors: list[MotorConfigItem]
    torque_enabled: bool


# ─── Service: MOTOR_GRIPPER ───────────────────────────────────────────


class MotorGripperReq(BaseModel):
    """객체별 셋업에서 position override 가능 — None 이면 default open/close."""

    model_config = ConfigDict(extra="forbid")
    action: Literal["open", "close"] = "open"
    current: int = Field(default=200, description="목표 전류 [mA] — 파지력")
    position: int | None = None
