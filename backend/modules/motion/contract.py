"""Motion domain — public contract surface.

backend.md §16.1 #4 (Motion) + §3.3 (TCP stream/snapshot 분리) +
§8.5 (stream seq/timestamp invariant).

D2 = MoveJ + TCP_STATE/SNAPSHOT. MoveL/C/P (D2c) / Jog (D3) 후속.
"""

from __future__ import annotations

from enum import StrEnum

from pydantic import BaseModel


class Motion:
    class Service(StrEnum):
        MOVE_J = "srv/motion/{robot_id}/move_j"  # joint target → trajectory
        MOVE_J_POSE = "srv/motion/{robot_id}/move_j_pose"  # TCP pose → IK → joint move
        MOVE_L = "srv/motion/{robot_id}/move_l"  # TCP 직선 (position-only v1)
        TCP_SNAPSHOT = "srv/motion/{robot_id}/tcp_snapshot"  # point-in-time TCP
        STOP = "srv/motion/{robot_id}/stop"
        # 후보 pose 그룹 배치 IK 판정 (모션 0) — MoveIt goal-sampling 패턴의 미니판.
        # task 가 후보마다 move 서비스로 원격 probe 하면 왕복×N + 실패 IK 풀비용이
        # 지배 (2026-07-09 PnP 10s) → in-process 1회 호출 + seed 연쇄 + early-exit.
        SELECT_REACHABLE = "srv/motion/{robot_id}/select_reachable"

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


class MoveJPoseRequest(BaseModel):
    """목표 TCP pose → IK → **관절 공간** MoveJ (Cartesian 직선 아님).

    MoveL 과의 차이 = 경로가 관절 보간이라 자세가 경로 따라 자유롭게 변함 → "특정
    자세 고정한 채 직선" 이 강제하는 높이-의존 도달성 실패가 없음 (SO-101 처럼
    workspace 안에서 자세가 위치마다 바뀌는 팔에 필수). UR `movej(pose)` 등가.
    IK 는 현재 자세를 seed 로 → 목표 config 가 가까워 부드럽게 이동.

    orientation:
      - target_quaternion=None → position-only IK (자세는 IK 가 자유롭게 선택).
      - 지정 → 그 자세로 IK (도달 가능해야 함).

    tool_offset: tcp 가 아니라 **tcp+tool_offset(tool frame) 지점**을 target 에 맞춘다.
      grasp 에서 tcp≠파지점(단일 jaw 그리퍼) 보정용 — 그리퍼 상수(큐브 무관). None=tcp.
      적용: IK(target)→자세 R → target - R·tool_offset 재-IK (자세는 근처라 1회 근사).
    """

    target_position: tuple[float, float, float]  # base frame, m
    target_quaternion: tuple[float, float, float, float] | None = None  # [x,y,z,w]
    tool_offset: tuple[float, float, float] | None = None  # tool frame, m


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


class TcpPose(BaseModel):
    """IK 판정용 TCP pose. quaternion=None → position-only."""

    position: tuple[float, float, float]  # base frame, m
    quaternion: tuple[float, float, float, float] | None = None  # [x,y,z,w]


class SelectReachableRequest(BaseModel):
    """후보 pose 그룹(순서 = 선호도) 중 '그룹 내 전 pose IK 가용'인 첫 그룹 판정.

    그룹 예 = [pre_grasp, grasp] (같은 자세로 접근+파지 둘 다 풀려야 실행 가능).
    IK 만 — 로봇은 안 움직임. 그룹 내 seed 연쇄 (앞 pose 해 → 다음 pose seed,
    가까운 pose 는 1발 수렴) + 첫 가용 그룹에서 early-exit.
    """

    groups: list[list[TcpPose]]


class SelectReachableResponse(BaseModel):
    index: int  # 첫 가용 그룹 index, 가용 없으면 -1
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
    # gripper 관절 — arm(IK/waypoint 벡터)과 분리된 별도 필드. kinematic chain 은
    # 아니지만 로봇 configuration 의 일부라 kinematic-state layer 가 rad 로 report
    # (제어는 여전히 Motor.set_gripper). URDF 시각화가 arm 처럼 이름 기반 매핑.
    # gripper 없는 robot 이거나 아직 raw 미수신이면 None.
    gripper_joint_name: str | None = None
    gripper_rad: float | None = None
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


# ─── errors (완료 계약 — backend.md §17.3) ────────────────


class MotionFailed(RuntimeError):
    """trajectory 가 오류(FAILED)/취소(STOPPED)로 끝남.

    `await motion.move_j()` / `move_l()` 의 완료 계약: 목표 정상 종료(DONE) → return,
    IK 실패/충돌/Ruckig 오류(FAILED) → 이 예외, 사용자 STOP(STOPPED) → 이 예외.
    Task DSL 규칙 = await 성공이면 완료, 다음 step 안전.
    """
