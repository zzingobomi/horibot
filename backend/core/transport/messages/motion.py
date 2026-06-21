"""Motion 노드 토픽 / 서비스 payload schema.

motion_taxonomy.md 의 4 계층 taxonomy:
- Move*  (one-shot target motion, trajectory-planned)
- Servo* (external absolute target stream — RL / Vision servo)
- Jog*   (human/manual velocity stream — frontend / gamepad)
- Task*  (scripted execution — task_node)

토픽:
- MOTION_STATE_TRAJ (publish) — MotionTrajState
- MOTION_STATE_TCP  (publish) — MotionTcpState (corrected EE pose, sag+link+joint_offset 적용)
- MOTION_JOG_TCP_STREAM (subscribe) — JogTcpReq (frontend/gamepad 50Hz velocity)
- MOTION_JOG_J_STREAM (subscribe) — JogJReq (frontend/gamepad 50Hz velocity)
- (MOTOR_CMD_JOINT publish — motor.py 의 MotorCmd 재사용)

서비스 (request data / response data):
- MOTION_GET_TCP    — EmptyData / MotionTcpPose
- MOTION_MOVE_J     — MoveJReq / EmptyData
- MOTION_MOVE_L     — MoveLReq / EmptyData
- MOTION_MOVE_C     — MoveCReq / EmptyData
- MOTION_MOVE_P     — MovePReq / EmptyData
- MOTION_SERVO_TCP  — ServoTcpReq / EmptyData (절대 pose chase — RL/Vision servo)
- MOTION_SERVO_J    — ServoJReq / EmptyData  (절대 joint chase — RL replay)
- MOTION_JOG_TCP    — JogTcpReq / EmptyData  (velocity 단발 — 자동화 tool 호출)
- MOTION_JOG_J      — JogJReq / EmptyData    (velocity 단발)
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


# ─── Topic: MOTION_STATE_TCP ─────────────────────────────────────────


class MotionTcpState(StrictModel):
    """corrected EE pose stream — joint state 갱신마다 publish.

    Backend `MotionModes.get_tcp_pose()` 의 결과 (sag + link_offset + joint_offset
    가 모두 적용된 corrected FK) 를 wire 로 노출. frontend 의 PointCloud / TCP
    AxisFrame / CameraFrustum 자리 SSOT — frontend 가 자체 URDF FK 로 cameraMatrix
    재계산 X (sag/link_offset 누락 → 사선 PC bug 자리 회귀 차단).

    `MOTION_GET_TCP` service 와 같은 값. 차이는:
      - service = 단발 query (호출자가 fresh 보장 필요한 자리, detector_node)
      - topic   = streaming push (motor state 와 같은 rate, 시각화/뷰어 자리)
    """

    position: list[float]
    quaternion: list[float]
    timestamp: float


# ─── Service: MOTION_SERVO_TCP ───────────────────────────────────────


class ServoTcpReq(StrictModel):
    """Servo (target chase) — 절대 TCP pose stream from external controller.

    Caller (RL policy / Vision servo / 외부 trajectory player) 가 *자기가 계산한
    절대 target* 자리 보냄. server = direct IK + publish (planner 우회).
    UR `servoc` / EGM / RSI Cartesian 자리 정석.

    `quaternion` None → position-only IK (5DOF / 6DOF 무관 — orientation 무시).
    6DOF robot 에서만 quaternion 의미.

    Human jog 자리는 `MOTION_JOG_TCP_STREAM` 자리 사용 — 의미 자리 다름.
    """

    position: list[float]
    quaternion: list[float] | None = None


# ─── Service: MOTION_SERVO_J ─────────────────────────────────────────


class ServoJReq(StrictModel):
    """Servo (target chase) — 절대 joint stream from external controller.

    Caller (RL replay / motion capture remap / 외부 trajectory player) 가 *자기가
    계산한 절대 joint target* 자리 보냄. server = direct publish (IK 불요).
    UR `servoj` / KUKA RSI joint 자리 정석.

    `positions` = arm joint URDF rad (motors.yaml `kind: arm` 순서, gripper 제외).

    Human jog 자리는 `MOTION_JOG_J_STREAM` 자리 사용.
    """

    positions: list[float]


# ─── Service: MOTION_JOG_TCP ─────────────────────────────────────────


class JogTcpReq(StrictModel):
    """Jog (human/manual velocity) — Cartesian twist input.

    Caller (frontend Jog UI / gamepad pendant) 가 *velocity twist 만* 보냄.
    backend JogTcpCommand 가 *실 끝점 pose* fresh latch + 실 측정 dt SE(3)
    적분 → IK → publish_cmd. 모든 caller 가 같은 wire (SE(3) 적분 SSOT = backend).

    `frame`:
      - `"base"` — twist 벡터가 base 좌표계 (world axes)
      - `"tcp"`  — twist 벡터가 현재 EE-local 좌표계

    OMX-F (5DOF) 자리는 angular 무시 (server-side IK 가 position-only fallback).
    `IDLE_RESET_S` 보다 publish 끊긴 자리 → 다음 publish 자리 fresh latch.
    """

    linear: list[float]  # [vx, vy, vz] m/s
    angular: list[float]  # [wx, wy, wz] rad/s
    frame: Literal["base", "tcp"] = "base"


# ─── Service: MOTION_JOG_J ───────────────────────────────────────────


class JogJReq(StrictModel):
    """Jog (human/manual velocity) — joint-space velocity input.

    Caller (frontend Jog UI / gamepad) 는 *velocity 만* 보냄. backend JogJCommand
    가 자기 process joint_cache (joint_offset 적용 URDF rad) 에서 ref latch +
    실 측정 dt 적분 → 절대 URDF rad target publish. cross-process safe
    (joint_offset SSOT = backend).

    `velocities` = arm joint URDF rad/s (motors.yaml `kind: arm` 순서).
    `IDLE_RESET_S` 보다 publish 끊긴 자리 → 다음 publish 자리 fresh latch.
    """

    velocities: list[float]


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
