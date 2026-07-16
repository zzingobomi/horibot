"""Motion Preview — plan-only 궤적 미리보기 계약 (POC).

목적: TCP pose 목표를 주면 "현재 자세 → 그 pose 로 MoveL/MoveJ(pose) 하면 팔이
어떻게 움직이나"를 **실행 없이** 계산해 관절 프레임 시퀀스로 돌려준다. frontend
고스트가 그 프레임을 재생 (실 로봇은 안 움직임). MoveIt 의 goal-state/trajectory
preview 의 미니판.

역할 경계 (2026-07-16 확정 — Viewer vs Analyzer):
  - 본 모듈 = **Viewer** — motion 과 *같은* TrajectoryRunner + 같은 IK 로 궤적을
    생성해 "실행하면 이렇게 움직인다"를 보여준다. 근사가 아니라 실 경로 그 자체.
  - 관절 급변 / manipulability / singularity 지표 = **Analyzer** (별도 후속 단계).
    여기 안 넣는다 — 뒤집힘(wrist flip)은 애니메이션 + 배속(슬로모)으로 눈에 보이고,
    "왜 뒤집혔나" 설명은 Analyzer 의 몫.

**robot-agnostic** — host 당 1 인스턴스 (backend.md §2.7). 대상 robot 은 req.robot_id.
motion 을 거치지 않는다 (wire/런타임 미접촉) — motion 의 kinematics/TrajectoryRunner
를 *라이브러리로 import* 재사용할 뿐 (scan 이 build_calibrated_kinematics 를 쓰는 것과
같은 공용 패턴). 현재 자세(start_joints)는 frontend 가 live tcp_state 에서 실어 보냄
(본 모듈은 아무 스트림도 구독 안 하는 stateless plan 함수).
"""

from __future__ import annotations

from enum import StrEnum

from pydantic import BaseModel

from framework.contract.service import declare_service_timeouts


class MotionPreview:
    class Service(StrEnum):
        # plan-only — 로봇 안 움직임. req 에 robot_id (robot-agnostic).
        PLAN = "srv/motion_preview/plan"


class PreviewMode(StrEnum):
    MOVE_L = "move_l"  # TCP 직선 (Cartesian) — 경로 샘플마다 IK, TCP 는 직선
    MOVE_J_POSE = "move_j_pose"  # 목표 pose IK 1회 → 관절 보간, TCP 는 호를 그림


class PreviewPoseTarget(BaseModel):
    """미리보기 목표 pose — 사람이 입력. position(m) + RPY(도).

    RPY 규약 = **intrinsic XYZ** (scipy `from_euler("XYZ", ..., degrees=True)`).
    frontend 마커도 three.js Euler order 'XYZ'(intrinsic) 로 *같은* 회전을 그려
    입력↔마커↔IK 가 일치한다 (오일러 관례 mismatch 방지)."""

    position: tuple[float, float, float]  # base frame, m
    rpy_deg: tuple[float, float, float]  # roll(x) / pitch(y) / yaw(z), degrees


class PlanPreviewRequest(BaseModel):
    robot_id: str
    # 현재 arm 관절 (rad) — frontend 가 live Motion.TCP_STATE.joints 에서 실음.
    # 본 모듈은 모터/모션 상태를 구독하지 않으므로 시작 자세를 요청으로 받는다.
    start_joints: list[float]
    target: PreviewPoseTarget
    mode: PreviewMode
    # 자세 축 (motion PoseTarget.quaternion None/set 을 그대로 노출):
    #   True  = target.rpy_deg 를 목표 자세로 (MoveL=현재→목표 slerp, MoveJ=도달)
    #   False = position-only (자세 자유 — IK 가 seed 근처 자세를 알아서, 도달성↑)
    use_orientation: bool = True


class PlanPreviewResponse(BaseModel):
    """plan-only 결과 — 로봇은 안 움직임. frontend 고스트 재생용.

    프레임 = **50Hz 시간등분** (TrajectoryRunner 의 jerk-limited 프로파일 그대로).
    frontend 가 50Hz 로 재생하면 실 로봇 실제 속도·가감속. 배속은 재생 rate 로만
    (프레임 재계산 없음).

    feasible=False 여도 frames/tcp_trace 는 **도달 가능 지점까지** 채워 반환 —
    트레이스가 끊기는 지점이 그림으로 드러난다 (반쯤 가다 얼어붙는 것 방지).
    quality/singularity 분석은 없음 (Analyzer 자리)."""

    feasible: bool
    # arm joint 이름 — frontend URDF 매핑 SSOT (Motion.TcpState.joint_names 동형).
    joint_names: list[str]
    # 관절 프레임 (rad, arm dof), 50Hz 시간등분. feasible=False 면 실패 직전까지.
    frames: list[list[float]]
    # 프레임별 TCP 위치 (base frame, m) — 3D 경로선(MoveL 직선 / MoveJ(pose) 곡선).
    tcp_trace: list[tuple[float, float, float]]
    # feasible=False 시 마지막 성공 프레임 index (트레이스가 끊긴 곳).
    fail_at_sample: int | None = None
    message: str = ""


# plan 은 궤적을 실시간 수집(이동 duration 만큼) + 프레임별 IK — 넉넉히.
declare_service_timeouts({
    MotionPreview.Service.PLAN: 60.0,
})
