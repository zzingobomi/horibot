"""Motion 노드 토픽 / 서비스 payload schema.

motion_taxonomy.md 의 3 계층 × 2 입력 공간:
- Trajectory-planned: MOTION_MOVE_J / MOTION_MOVE_L / MOTION_MOVE_C / MOTION_MOVE_P
- Servo (target chase): MOTION_SERVO_TCP
- Velocity (jog, deadman timeout): MOTION_SPEED_TCP / MOTION_SPEED_J

토픽:
- MOTION_STATE_TRAJ (publish) — MotionTrajState
- (MOTOR_CMD_JOINT publish — motor.py 의 MotorCmd 재사용)

서비스 (request data / response data):
- MOTION_GET_TCP    — EmptyData / MotionTcpPose
- MOTION_MOVE_J     — MoveJReq / EmptyData
- MOTION_MOVE_L     — MoveLReq / EmptyData
- MOTION_MOVE_C     — MoveCReq / EmptyData
- MOTION_MOVE_P     — MovePReq / EmptyData
- MOTION_SERVO_TCP  — ServoTcpReq / EmptyData
- MOTION_SPEED_TCP  — SpeedTcpReq / EmptyData
- MOTION_SPEED_J    — SpeedJReq / EmptyData
- MOTION_STOP       — EmptyData / EmptyData
"""

from __future__ import annotations

from enum import Enum
from typing import Literal

from core.transport.messages.base import StrictModel


# ─── TrajStatus ──────────────────────────────────────────────────────


class TrajStatus(str, Enum):
    IDLE = "idle"
    RUNNING = "running"
    DONE = "done"
    STOPPED = "stopped"
    FAILED = "failed"


# ─── Topic: MOTION_STATE_TRAJ ────────────────────────────────────────


class MotionTrajState(StrictModel):
    """trajectory 진행 상태 publish. runner / 핸들러 가 발행."""

    status: TrajStatus
    progress: float = 0.0
    timestamp: float


# ─── Service: MOTION_GET_TCP ─────────────────────────────────────────


class MotionTcpPose(StrictModel):
    """URDF EE pose. position (m) + quaternion [x, y, z, w]."""

    position: list[float]
    quaternion: list[float]


# ─── Service: MOTION_SERVO_TCP ───────────────────────────────────────


class ServoTcpReq(StrictModel):
    """절대 TCP target 직접 IK + publish (planner 우회).

    `quaternion` None → position-only IK (5DOF / 6DOF 무관 — orientation 무시).
    6DOF robot 에서만 quaternion 의미 — 5DOF (OMX-F) 면 orientation 필드 무시.
    """

    position: list[float]
    quaternion: list[float] | None = None


# ─── Service: MOTION_SPEED_TCP ───────────────────────────────────────


class SpeedTcpReq(StrictModel):
    """TCP twist 추종 (linear 3 + angular 3). server 가 timeout 까지 추종.

    `frame`:
      - `"base"` — twist 벡터가 base 좌표계 (world axes)
      - `"tcp"`  — twist 벡터가 현재 EE-local 좌표계

    OMX-F (5DOF) 자리는 angular 무시 (linear-only).
    """

    linear: list[float]  # [vx, vy, vz] m/s
    angular: list[float]  # [wx, wy, wz] rad/s
    frame: Literal["base", "tcp"] = "base"


# ─── Service: MOTION_SPEED_J ─────────────────────────────────────────


class SpeedJReq(StrictModel):
    """joint velocity 벡터 추종. server 가 timeout 까지 추종.

    `velocities` 길이 = robot arm dof (OMX-F=5, SO-101=6). dof 불일치 시 fail.
    """

    velocities: list[float]  # rad/s, arm joint 순서


# ─── Service: MOTION_MOVE_J ──────────────────────────────────────────


class JointDegree(StrictModel):
    """모터 id 와 목표 각도 (degrees)."""

    id: int
    degree: float


class MoveJReq(StrictModel):
    """관절 공간 이동. joints[i] 가 없으면 0도."""

    joints: list[JointDegree]


# ─── Service: MOTION_MOVE_L (linear) ─────────────────────────────────


class MoveLReq(StrictModel):
    """target position (user frame, m). 시작점은 현재 TCP."""

    position: list[float]


# ─── Service: MOTION_MOVE_C (circular via 1 mid + end) ───────────────


class MoveCReq(StrictModel):
    """원호. via / end 둘 다 user frame."""

    via: list[float]
    end: list[float]


# ─── Service: MOTION_MOVE_P (spline) ─────────────────────────────────


class MovePReq(StrictModel):
    """spline waypoints. 최소 2개 — 시작점은 현재 TCP 가 prepend 됨."""

    waypoints: list[list[float]]
