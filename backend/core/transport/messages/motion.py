"""Motion 노드 토픽 / 서비스 payload schema.

토픽:
- MOTION_STATE_TRAJ (publish) — MotionTrajState
- (MOTOR_CMD_JOINT publish — motor.py 의 MotorCmd 재사용)

서비스 (request data / response data):
- MOTION_GET_TCP   — EmptyData / MotionTcpPose
- MOTION_MOVE_TCP  — MoveTcpReq / EmptyData
- MOTION_MOVE_J    — MoveJReq / EmptyData
- MOTION_MOVE_L    — MoveLReq / EmptyData
- MOTION_MOVE_C    — MoveCReq / EmptyData
- MOTION_MOVE_P    — MovePReq / EmptyData
- MOTION_STOP      — EmptyData / EmptyData

`TrajStatus` enum 은 본 모듈에 — 이전 `core/types.py` 의 motion 전용 잔재 흡수
(typed_messaging.md core reorg §결정).
"""

from __future__ import annotations

from core.transport.messages.base import StrictModel

from enum import Enum



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


# ─── Service: MOTION_MOVE_TCP ────────────────────────────────────────


class MoveTcpReq(StrictModel):
    """target_pos (user frame). motion 핸들러가 tool_offset 보정 후 IK."""

    position: list[float]


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
