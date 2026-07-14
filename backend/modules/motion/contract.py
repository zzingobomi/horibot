"""Motion domain — public contract surface.

backend.md §16.1 #4 (Motion) + §3.3 (TCP stream/snapshot 분리) +
§8.5 (stream seq/timestamp invariant).

D2 = MoveJ + TCP_STATE/SNAPSHOT. MoveL/C/P (D2c) / Jog (D3) 후속.
"""

from __future__ import annotations

from enum import StrEnum
from typing import Annotated, Literal

from pydantic import BaseModel, Field

from framework.contract.service import declare_service_timeouts


class Motion:
    class Service(StrEnum):
        # 관절 보간 이동 — target 이 JointTarget(관절값 직접) 또는 PoseTarget
        # (TCP pose → IK). "무엇을 관절 보간으로 도달하나" 계약 (UR movej(q|pose)
        # 동형): 목표 표현 차이는 planner 가 같아 한 서비스, target discriminated union.
        MOVE_J = "srv/motion/{robot_id}/move_j"
        MOVE_L = "srv/motion/{robot_id}/move_l"  # TCP 직선 (planner 다름 → 별 서비스)
        TCP_SNAPSHOT = "srv/motion/{robot_id}/tcp_snapshot"  # point-in-time TCP
        STOP = "srv/motion/{robot_id}/stop"
        # 후보 pose 그룹 배치 IK 판정 (모션 0) — MoveIt goal-sampling 패턴의 미니판.
        # task 가 후보마다 move 서비스로 원격 probe 하면 왕복×N + 실패 IK 풀비용이
        # 지배 (2026-07-09 PnP 10s) → in-process 1회 호출 + seed 연쇄 + early-exit.
        # 이름 = Resolve (2026-07-13): 계약이 "순서를 선호 힌트로 존중하며 가용
        # 그룹 하나" 라서 — 엄격한 first 보장이 아님 (deepening 이 속도와 맞바꿈).
        RESOLVE_REACHABLE = "srv/motion/{robot_id}/resolve_reachable"

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


# ─── move target (도달 명세 — MoveJ/MoveL 공용) ─────────────────────
#
# target 표현 두 가지 (discriminated union on `kind`):
#   JointTarget = 관절값 직접 (IK 없음).
#   PoseTarget  = TCP pose → IK. "이 pose 를 어느 제어점(tcp_offset)으로 만족시키나"
#                 = 도달 명세 (Reach Spec) — position/orientation/제어점이 모두 목표의
#                 구성요소. IK seed/redundancy 등 "어떻게 푸나(solver hint)" 는 목표가
#                 아니므로 여기 아님 (생기면 별도 options 로 — 2026-07-13 밤 확정).


class JointTarget(BaseModel):
    kind: Literal["joint"]
    joints: list[float]  # arm joint target, rad (dof,)


class PoseTarget(BaseModel):
    """TCP pose 도달 목표 — IK 로 관절 해소. UR `movej(pose)`/`movel(pose)` 등가.

    orientation:
      - quaternion=None → position-only IK (자세는 IK 가 자유롭게 선택 / MoveL 은
        seed 자세에 딸려감).
      - 지정 → **목표 자세**. MoveJ 는 그 자세로 도달, MoveL 은 현재 자세 → 이
        자세로 경로 s 에 동기해 slerp 보간 (UR/ABB/MoveIt 식 — 자세 고정은
        현재==목표인 특수 케이스). 옛 "MoveL 경로 전 구간 고정" 의미는 폐기.

    tcp_offset: tcp 가 아니라 **tcp+tcp_offset(tool frame) 지점**을 이 pose 에 맞춘다
      (= 제어점/TCP 선택). grasp 에서 tcp≠파지점(단일 jaw 그리퍼) 보정용 — 그리퍼
      상수(큐브 무관). None=tcp. 목표를 바꾸는 값이라(어디 서느냐) 목표 명세의 일부.
      적용: IK(pose)→자세 R → pose - R·tcp_offset 로 목표 보정 후 도달.
    """

    kind: Literal["pose"]
    position: tuple[float, float, float]  # base frame, m
    quaternion: tuple[float, float, float, float] | None = None  # [x,y,z,w]
    tcp_offset: tuple[float, float, float] | None = None  # tool frame, m (None=tcp)


MoveTarget = Annotated[JointTarget | PoseTarget, Field(discriminator="kind")]


class MoveJRequest(BaseModel):
    """관절 보간 이동 — target 이 관절값(JointTarget)이든 pose(PoseTarget)든.

    MoveL 과의 차이 = 경로가 관절 보간이라 자세가 경로 따라 자유롭게 변함 → "특정
    자세 고정한 채 직선" 이 강제하는 높이-의존 도달성 실패가 없음 (SO-101 처럼
    workspace 안에서 자세가 위치마다 바뀌는 팔에 pose 접근/승강 필수). UR
    `movej(q)`/`movej(pose)` 등가 — 인자 타입으로 갈리던 걸 discriminated union 으로.
    """

    target: MoveTarget


class MoveJResponse(BaseModel):
    """빈 응답 — 성공 = 반환, 실패 = raise (MotionRejected/MotionFailed).

    옛 accepted/message in-band 모델 폐기 (2026-07-13): 거부는 기술적 실패라
    예외가 정본. 호출자가 체크를 잊으면 침묵 진행되던 급소 제거."""


class MoveLRequest(BaseModel):
    """TCP 를 현재 위치 → target(pose) 직선(MoveL) 이동. planner 가 MoveJ 와 달라
    (Cartesian 직선) 별 서비스. joint 직선은 무의미하므로 target 은 PoseTarget 전용.

    자세 = PoseTarget.quaternion 이 **목표 자세** — 현재 자세에서 이 자세로 경로 s
    에 동기해 slerp 보간 (UR/ABB/MoveIt base primitive). 자세 고정은 현재==목표인
    특수 케이스 (PnP 접근축 진입/승강이 이 경우 — 2026-07-07 45° 사선 하강이 큐브를
    밀던 실패에서 도입). quaternion=None = position-only. KUKA ORI_TYPE 식 명시
    모드(#JOINT 등)는 실제 필요 시 modifier 로 후속 (2026-07-13 토대 결정).
    """

    target: PoseTarget


class MoveLResponse(BaseModel):
    """빈 응답 — 성공 = 반환, 실패 = raise (MoveJResponse 와 동일 계약)."""


class TcpPose(BaseModel):
    """IK 판정용 TCP pose. quaternion=None → position-only."""

    position: tuple[float, float, float]  # base frame, m
    quaternion: tuple[float, float, float, float] | None = None  # [x,y,z,w]


class ResolveReachableRequest(BaseModel):
    """후보 pose 그룹 중 '그룹 내 전 pose 가용'인 그룹 하나를 resolve.

    계약: **순서 = 선호 힌트 (best-effort)** — 가급적 앞쪽 그룹을 반환하지만
    엄격한 first 보장은 아님 (구현이 cheap→expensive 게이트 파이프라인 —
    싼 게이트를 전 그룹에 먼저 돌리므로, 앞 그룹이 어려운 해면 뒤의 쉬운 가용
    그룹이 먼저 잡힐 수 있다). 선호가 보장이어야 하는 소비자가 나타나면 그때
    별도 질의 (FIRST/ALL/BEST) 신설.

    그룹 예 = [pre_grasp, grasp] (같은 자세로 접근+파지 둘 다 풀려야 실행 가능).
    판정 전용 — 로봇은 안 움직임. 게이트 (grasp_redesign_journey.md §5.5/§10.4,
    cheap→expensive — 뒤 게이트일수록 비싸고, 앞 게이트가 후보를 걸러 비용 절감):
      ① 위치 스크린 (position-only 소예산 IK — workspace 밖 즉시 기각)
      ② 전 pose 자세 IK (예산 점증 deepening — 실패 기각을 싸게)
      ③ floor_z 지정 시 바닥 평면 충돌 (해 자세의 로봇 링크 침투 기각)
      ③b obstacle_points 지정 시 장애물 점군 충돌 — 해 자세에서 로봇(그리퍼
         gripper_open 반영)이 관측 점군(물체/이웃)을 침투하면 기각 (§10.4-3
         그리퍼↔물체 충돌 게이트 — 맹목 파지 차단, fail-safe)
      ④ path_from 지정 시 그 관절 자세 → 첫 pose 해까지 관절 보간 경로의
         self/floor/obstacle 충돌 (§10.4-4 — naive MoveJ 가 물체/바닥을 스치는
         실행 시점 사고를 계획 시점 기각으로. 실행부는 path_from 자세에서 MoveJ
         하는 계약)
      ⑤ linear 지정 시 그룹 내 연속 pose 사이 직선 경로 실현성
         (MoveL 실행 전제 — 샘플 IK + 인접 해 joint jump 검사. 끝점만 풀리고
         중간이 안 풀리는 실행 시점 거부를 계획 시점으로 앞당김)
    """

    groups: list[list[TcpPose]]  # 순서 = 선호도 (힌트)
    # 바닥 평면 z (base frame) — planner 충돌 게이트 (옵션, cm 오차 OK).
    # None = 바닥 게이트 없음 (공중/손 위 물체 등 지지면 무관 시나리오).
    floor_z: float | None = None
    # True = 그룹 내 연속 pose 를 MoveL(직선)로 이을 전제 — 경로 게이트 ⑤ 활성.
    linear: bool = False
    # 장애물 점군 (base frame, m) — 관측 점군(타깃 자신 + 이웃). 게이트 ③b 활성.
    # None/빈 = 게이트 없음.
    obstacle_points: list[tuple[float, float, float]] | None = None
    # ③b/④ 검사 시 그리퍼 조를 벌린 자세(URDF 상한)로 둘지 — 파지 접근은 조를
    # 벌린 채라 그 부피가 실 충돌 형상 (관측 이동 등 비파지 판정은 False).
    gripper_open: bool = False
    # 관절 자세 (rad, dof) — 게이트 ④ 활성: 여기서 첫 pose 해까지 관절 보간
    # 경로가 충돌 없어야 채택. 실행부가 실제로 이 자세에서 MoveJ 한다는 계약
    # (pick_and_place 는 home 경유가 그 자세).
    path_from: list[float] | None = None


class ResolveReachableResponse(BaseModel):
    index: int  # 가용 그룹 index (선호 순서 best-effort), 가용 없으면 -1 (= 데이터)
    # 채택 그룹의 pose 별 IK 해 (rad, index≥0 일 때 그룹 pose 수와 동일) —
    # 실행부가 재계산 없이 이 관절값으로 이동 (판정 해 == 실행 해 보장, §5.5).
    solutions: list[list[float]] = []
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
#
# 예외 vs 데이터 기준 (2026-07-13 확정):
#   예외 = 기술적 실패 — 요청한 행위 자체가 수행 불가 (IK 불능, 진행 중 충돌,
#          motor state 미도달, 이전 motion 점유). wire 로는 RemoteError(type, msg).
#   데이터 = 부정적이지만 유효한 도메인 결과 — RESOLVE_REACHABLE 의 index=-1
#          (후보 전멸이 치명인지는 호출자/시나리오가 판정), DETECT 의 후보 0개.
# 판정은 서비스 계약마다 명시 — "accepted=False 는 전부 예외" 같은 일괄 규칙 없음.


class MotionRejected(RuntimeError):
    """move 요청이 수락 단계에서 거부됨 (모션 0 — 로봇은 안 움직였음).

    IK 실패 / motor state 미도달 / 이전 motion 진행 중 등. 옛 accepted=False
    in-band 응답의 대체 — 호출자는 체크 없이 await 만 하면 되고, 거부는
    RemoteError("MotionRejected", 사유) 로 wire 를 건넌다.
    """


class MotionFailed(RuntimeError):
    """trajectory 가 오류(FAILED)/취소(STOPPED)로 끝남 (모션 도중 종료).

    `await motion.move_j()` / `move_l()` 의 완료 계약: 목표 정상 종료(DONE) → return,
    IK 실패/충돌/Ruckig 오류(FAILED) → 이 예외, 사용자 STOP(STOPPED) → 이 예외.
    task 규칙 = await 성공이면 완료, 다음 step 안전.
    """


# ─── 서비스 기본 timeout (runtime.call 이 timeout 미지정 시 사용) ───

declare_service_timeouts({
    Motion.Service.MOVE_J: 60.0,  # trajectory 완료까지 await (joint/pose 공통)
    Motion.Service.MOVE_L: 60.0,
    # 그룹 다수(파지 260/적치 104) × IK restart 예산 — 실측: 정상 260그룹 ~수십 초,
    # 유령 중복 backend CPU 경합 시 104그룹 74s 로 60s 캡을 스침 (2026-07-14).
    # 경합의 본질 해결은 프로세스 위생이지만 캡은 싸게 넓혀둔다 (전멸 가족은
    # 모든 그룹이 풀예산 IK 를 태워 최악이 성공 케이스보다 느리다).
    Motion.Service.RESOLVE_REACHABLE: 120.0,
})
