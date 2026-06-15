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

from pydantic import Field
from core.transport.messages.base import StrictModel


# ─── Topic: MOTOR_STATE_JOINT ────────────────────────────────────────


class MotorJoint(StrictModel):
    """state 토픽 한 모터 항목."""

    id: int
    name: str
    position: int  # raw 0..4095
    degree: float
    velocity: float = 0.0
    torque: float = 0.0
    load: int = 0


class MotorJointState(StrictModel):
    """MOTOR_STATE_JOINT publish 페이로드. STATE_PUBLISH_HZ 로 발행."""

    timestamp: float
    joints: list[MotorJoint]


# ─── Topic: MOTOR_CMD_JOINT ──────────────────────────────────────────


class MotorCmdJoint(StrictModel):
    """cmd 토픽 한 모터 명령. position 만 받음 (raw 0..4095)."""

    id: int
    position: int


class MotorCmd(StrictModel):
    """MOTOR_CMD_JOINT subscribe 페이로드. TrajectoryRunner 가 100Hz 로 발행."""

    timestamp: float = 0.0
    joints: list[MotorCmdJoint]


# ─── Service: MOTOR_ENABLE ────────────────────────────────────────────


class MotorEnableReq(StrictModel):
    enable: bool = True


class MotorEnableRes(StrictModel):
    enable: bool


# ─── Service: MOTOR_REBOOT ────────────────────────────────────────────


class MotorRebootReq(StrictModel):
    """id=None 이면 전 모터 reboot."""

    id: int | None = None


# ─── Service: MOTOR_SET_PROFILE (single motor) ────────────────────────


class MotorSetProfileReq(StrictModel):
    id: int
    velocity: int | None = None
    acceleration: int | None = None


# ─── Service: MOTOR_SET_PROFILE_ALL (multi motor) ─────────────────────


class MotorSetProfileAllReq(StrictModel):
    """ids=None 이면 driver.motor_ids 전체 적용.

    `restore_defaults=True` 면 velocity/acceleration 무시 + 각 모터의 motors.yaml
    `profile` (dps) 적용. TrajectoryRunner 가 moveJ/L/C/P 종료 시 호출 — release
    (raw 0,0) 의 반대 동작.
    """

    ids: list[int] | None = None
    velocity: int = 0
    acceleration: int = 0
    restore_defaults: bool = False


# ─── Service: MOTOR_GET_CONFIG ────────────────────────────────────────


class MotorLimit(StrictModel):
    min: int
    max: int


class MotorConfigItem(StrictModel):
    id: int
    name: str
    model: str
    mode: str
    kind: Literal["arm", "gripper"]
    home: int
    limit: MotorLimit


class MotorGetConfigRes(StrictModel):
    motors: list[MotorConfigItem]
    torque_enabled: bool


# ─── Service: MOTOR_GRIPPER ───────────────────────────────────────────


class MotorGripperReq(StrictModel):
    """객체별 셋업에서 position override 가능 — None 이면 default open/close."""

    action: Literal["open", "close"] = "open"
    current: int = Field(default=200, description="목표 전류 [mA] — 파지력")
    position: int | None = None
