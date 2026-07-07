"""Motion domain — public contract surface.

backend_v2.md §16.1 #4 (Motion) + §3.3 (TCP stream/snapshot 분리) +
§8.5 (stream seq/timestamp invariant).

D2 = MoveJ + TCP_STATE/SNAPSHOT. MoveL/C/P (D2c) / Jog (D3) 후속.
"""

from __future__ import annotations

from enum import StrEnum

from pydantic import BaseModel


class Motion:
    class Service(StrEnum):
        MOVE_J = "srv/motion/{robot_id}/move_j"  # joint target → trajectory
        MOVE_L = "srv/motion/{robot_id}/move_l"  # TCP 직선 (position-only v1)
        TCP_SNAPSHOT = "srv/motion/{robot_id}/tcp_snapshot"  # point-in-time TCP
        STOP = "srv/motion/{robot_id}/stop"

    class Stream(StrEnum):
        TCP_STATE = "stream/motion/{robot_id}/tcp_state"  # 20Hz fk (output)
        TRAJ_STATE = "stream/motion/{robot_id}/traj_state"  # trajectory 진행 (output)
        # jog 입력 (frontend/gamepad 50Hz fire-and-forget → motion subscribe).
        # output state stream 아니라 seq/timestamp invariant 면제 — dt 는 motion 이
        # 수신 시각으로 측정 (LeRobot delta-pose 패턴).
        JOG_J = "stream/motion/{robot_id}/jog_j"
        JOG_TCP = "stream/motion/{robot_id}/jog_tcp"

    class Event(StrEnum):
        MOTION_COMPLETED = "event/motion/{robot_id}/completed"  # Move 끝


class TrajStatus(StrEnum):
    RUNNING = "running"
    DONE = "done"
    FAILED = "failed"
    STOPPED = "stopped"


# ─── request / response ────────────────────────────────────────────


class MoveJRequest(BaseModel):
    target_joints: list[float]  # arm joint target, rad (dof,)


class MoveJResponse(BaseModel):
    accepted: bool
    message: str = ""


class MoveLRequest(BaseModel):
    """TCP 를 현재 위치에서 target_position 으로 직선(MoveL) 이동.

    orientation:
      - target_quaternion=None → position-only IK (orientation 은 seed 자세에
        딸려감 — v1 제약 그대로).
      - target_quaternion 지정 → 경로 전 구간 그 자세 고정 (constant-orientation
        MoveL, UR 등가). 첫 소비자 = PnP 접근축 진입/자세고정 승강 (2026-07-07 —
        45° 사선 하강이 큐브를 밀던 실패에서 도입). SLERP interpolation 은 후속.
    """

    target_position: tuple[float, float, float]  # base frame, m
    target_quaternion: tuple[float, float, float, float] | None = None  # [x,y,z,w]


class MoveLResponse(BaseModel):
    accepted: bool
    message: str = ""


class TcpSnapshotRequest(BaseModel):
    pass


class JogJInput(BaseModel):
    """joint-space velocity jog 입력 (arm rad/s). motion 이 max_velocity 로 cap +
    joint limit clamp 후 적분."""

    robot_id: str
    velocities: list[float]


class JogTcpInput(BaseModel):
    """cartesian twist jog 입력. linear m/s + angular rad/s + frame(base|tcp).
    motion 이 SE(3) 적분 + IK (unreachable/collision reject)."""

    robot_id: str
    linear: tuple[float, float, float]
    angular: tuple[float, float, float]
    frame: str = "base"  # base | tcp


class StopRequest(BaseModel):
    pass


class StopResponse(BaseModel):
    ok: bool


# ─── stream payload (seq + timestamp_unix invariant — §8.5) ────────


class TcpState(BaseModel):
    """20Hz — fk(current joints). position(m) + quaternion[x,y,z,w] + joints(rad).

    joint_names + joints 는 parallel array (ROS `sensor_msgs/JointState` 패턴).
    joint 순서 SSOT = motors.yaml arm prefix — URDF 파일 순서와 무관하게 consumer 가
    이름 기반으로 URDF joint 를 찾아 매핑하도록 self-describing 계약."""

    robot_id: str
    seq: int
    timestamp_unix: float
    position: tuple[float, float, float]
    quaternion: tuple[float, float, float, float]
    joint_names: list[str]  # arm joint names, motors.yaml 순서
    joints: list[float]  # arm rad, joint_names 와 same index
    # D4 캘 적용 상태 표면화 — "무보정으로 조용히 돈다" 차단 (frontend 배지).
    calibration_applied: bool = False  # joint/link/sag 중 하나라도 적용됨
    calibration_stale: bool = False  # 적용 후 캘 변경 감지 — 재시작 필요


class TrajState(BaseModel):
    robot_id: str
    seq: int
    timestamp_unix: float
    status: TrajStatus
    progress: float  # 0..1


# ─── event payload ─────────────────────────────────────────────────


class MotionCompleted(BaseModel):
    robot_id: str
    status: TrajStatus  # DONE / FAILED / STOPPED


# ─── errors (완료 계약 — backend_v2.md §17.3) ────────────────


class MotionFailed(RuntimeError):
    """trajectory 가 오류(FAILED)/취소(STOPPED)로 끝남.

    `await motion.move_j()` / `move_l()` 의 완료 계약: 목표 정상 종료(DONE) → return,
    IK 실패/충돌/Ruckig 오류(FAILED) → 이 예외, 사용자 STOP(STOPPED) → 이 예외.
    Task DSL 규칙 = await 성공이면 완료, 다음 step 안전.
    """
