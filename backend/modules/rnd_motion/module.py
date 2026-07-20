"""RndMotionModule — 산업 arm(UR 등) sim 전용 모션. 완전 자립.

회사 프리뷰 R&D(host=rnd) 트랙. 집 실물 스택(motor/motion/TrajectoryRunner/틱/
캘리브레이션) 재사용 0. 산업 arm 은 구형 손목이라 기구학이 깨끗 → **seed 하나로 단발
수치 IK** 면 충분. so101/omx 가 비산업 개조라 어쩔 수 없이 쓰던 random restart /
continuous walk / deepening budget 카오스가 여기엔 없다.

궤적 = **Ruckig(저크제한 프로파일) 라이브러리 직접 호출** — 집 TrajectoryRunner(실행
루프) 는 안 쓰고, 계산만 offline 으로. `trajectory.at_time` 으로 50Hz 시간등분 샘플 →
가속→순항→감속이 프레임에 실려 고스트가 현실적으로 움직이고 속도 점 리듬도 산다.

재사용은 **wire 계약 스키마뿐** (TcpState / PlanPreview* — 프론트 공유 SSOT라
재정의하면 안 됨). 구현 코드는 집 모듈에서 한 줄도 안 가져온다.

책임:
  - sim 관절상태 자기소유 (home 자세로 부팅). 라이브 로봇처럼 TCP_STATE 를 발행 →
    씬이 로봇을 그리고 프리뷰 패널이 시작 자세를 얻는다 (모터 모듈 불필요).
  - motion_preview PLAN 서비스: MoveL(직선)/MoveJ(pose) 궤적을 자체 pybullet IK +
    Ruckig 로 생성해 관절 프레임 + TCP 트레이스 반환 (프론트 고스트 재생·경로 휨 관찰).

robot-scoped (robot_id 별 인스턴스). PLAN 은 계약상 agnostic 키라 host 당 산업 arm
1대 전제 (현 rnd = ur5e 1대). 2대 이상 붙일 땐 PLAN 을 robot-scoped 로 승격 필요.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from ruckig import InputParameter, OutputParameter, Result, Ruckig
from scipy.spatial.transform import Rotation, Slerp

from framework.contract.publisher import publishes
from framework.contract.service import service
from framework.runtime.api import ModuleRuntime
from modules.motion.contract import Motion, TcpState
from modules.motion_preview.contract import (
    MotionPreview,
    PlanPreviewRequest,
    PlanPreviewResponse,
    PreviewMode,
    PreviewPoseTarget,
)

from .kinematics import ArmKinematics, Quat, Vec3, quat_angle_rad

logger = logging.getLogger(__name__)

_TCP_STATE_HZ = 20.0
_TRAJ_DT = 1.0 / 50  # 궤적 프레임 = 50Hz 시간등분 (프론트가 이 rate 로 재생 = 실속도)
# 도달 판정 게이트 — 산업 정밀 기준(로봇팀: 1cm 도 큼). IK refine 이 도달 가능한
# 점을 sub-mm/sub-0.1° 로 맞추므로, 이 값을 넘으면 그 점은 실제로 못 맞춘 것
# (workspace 경계/특이) → 도달 불가. 위치·자세 **둘 다** 임계 안이어야 "풀렸다"
# (수치 IK 는 위치 맞추며 자세를 내주거나 그 반대가 가능 — 한쪽만 보면 구멍).
# 트레이스는 어차피 FK(해)라 어긋남은 그림에 남는다 (게이트는 끊는 시점만 결정).
_REACH_TOL_M = 0.001  # 위치 1mm
_REACH_ORI_TOL_DEG = 0.1  # 자세 0.1° (use_orientation 일 때만)


def _residuals(
    kin: ArmKinematics, sol: list[float], target_pos: Vec3, target_quat: Quat | None
) -> tuple[float, float]:
    """해의 실제 도달 오차 (위치 m, 자세 도). target_quat=None 이면 자세 0."""
    rp, rq = kin.fk(sol)
    pos_err = _dist(rp, target_pos)
    ori_err = (
        0.0 if target_quat is None else math.degrees(quat_angle_rad(rq, target_quat))
    )
    return pos_err, ori_err
# 궤적 길이 방어 상한 (초). 한계가 비정상적으로 작아 프로파일이 폭주하는 것 차단.
_MAX_DURATION_S = 120.0


@dataclass(frozen=True)
class RndRobotSpec:
    """sim robot 정적 config (constructor dep, wire contract 아님).

    한계값(vel/acc/jerk)은 <type>/motion.yaml 에서 투영 — Ruckig 프로파일 입력.
    joint 속도는 UR5e 사양 기반, cartesian 속도는 프리뷰 가시성용(실물 ~1m/s 아님).
    """

    urdf_path: Path
    joint_names: list[str]  # arm dof, URDF revolute 이름 순
    home_joints: list[float]  # 부팅 자세 (rad)
    joint_max_velocity: list[float]
    joint_max_acceleration: list[float]
    joint_max_jerk: list[float]
    cartesian_max_velocity: float
    cartesian_max_acceleration: float
    cartesian_max_jerk: float


def _rpy_to_quat(rpy_deg: tuple[float, float, float]) -> Quat:
    """intrinsic XYZ 오일러(도) → quaternion [x,y,z,w]. 프리뷰 계약 규약 SSOT."""
    q = Rotation.from_euler("XYZ", rpy_deg, degrees=True).as_quat()
    return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))


def _dist(a: Vec3, b: Vec3) -> float:
    return math.dist(a, b)


def _profile_1d(
    start: list[float],
    goal: list[float],
    max_vel: list[float],
    max_acc: list[float],
    max_jerk: list[float],
) -> list[list[float]] | None:
    """Ruckig 로 start→goal 저크제한 프로파일 계산 → 50Hz 시간등분 위치 시퀀스.

    offline: update 1회로 전체 궤적 산출 후 `at_time` 으로 샘플 (실행 루프/ sleep
    없음). None = Ruckig 입력 오류(한계 0 등). 다차원 동기(time-sync)는 Ruckig 기본."""
    dof = len(start)
    otg = Ruckig(dof, _TRAJ_DT)
    inp = InputParameter(dof)
    out = OutputParameter(dof)
    inp.current_position = list(start)
    inp.current_velocity = [0.0] * dof
    inp.current_acceleration = [0.0] * dof
    inp.target_position = list(goal)
    inp.target_velocity = [0.0] * dof
    inp.target_acceleration = [0.0] * dof
    inp.max_velocity = list(max_vel)
    inp.max_acceleration = list(max_acc)
    inp.max_jerk = list(max_jerk)

    res = otg.update(inp, out)
    if res not in (Result.Working, Result.Finished):
        return None
    duration = min(out.trajectory.duration, _MAX_DURATION_S)
    n = max(1, math.ceil(duration / _TRAJ_DT))
    samples: list[list[float]] = []
    for i in range(n + 1):
        t = min(i * _TRAJ_DT, duration)
        pos, _vel, _acc = out.trajectory.at_time(t)
        samples.append([float(v) for v in pos])
    return samples


def plan_trajectory(
    kin: ArmKinematics,
    spec: RndRobotSpec,
    start_joints: list[float],
    target: PreviewPoseTarget,
    mode: PreviewMode,
    use_orientation: bool,
) -> PlanPreviewResponse:
    """MoveL/MoveJ 궤적을 pybullet IK + Ruckig 로 생성 (blocking — 호출자 to_thread).

    프레임 = 관절각 시퀀스(50Hz 시간등분), tcp_trace = 각 프레임 FK 위치. MoveL 은
    직선 위 점마다 IK 라 IK 위치오차가 트레이스 휨으로 드러난다 (Viewer — 안 거름)."""
    target_pos: Vec3 = target.position
    target_quat = _rpy_to_quat(target.rpy_deg) if use_orientation else None
    if mode == PreviewMode.MOVE_J_POSE:
        return _plan_move_j(kin, spec, start_joints, target_pos, target_quat)
    return _plan_move_l(kin, spec, start_joints, target_pos, target_quat)


def _infeasible(joint_names: list[str], msg: str) -> PlanPreviewResponse:
    return PlanPreviewResponse(
        feasible=False,
        joint_names=joint_names,
        frames=[],
        tcp_trace=[],
        fail_at_sample=0,
        message=msg,
    )


def _plan_move_j(
    kin: ArmKinematics,
    spec: RndRobotSpec,
    start: list[float],
    target_pos: Vec3,
    target_quat: Quat | None,
) -> PlanPreviewResponse:
    """목표 pose IK 1회 → 관절공간 Ruckig 프로파일. TCP 는 호를 그린다."""
    names = spec.joint_names
    sol = kin.ik(target_pos, target_quat, start)
    pos_err, ori_err = _residuals(kin, sol, target_pos, target_quat)
    if pos_err > _REACH_TOL_M or ori_err > _REACH_ORI_TOL_DEG:
        return _infeasible(
            names,
            f"목표 도달 불가 (위치 {pos_err * 1000:.1f}mm / 자세 {ori_err:.2f}°)",
        )
    frames = _profile_1d(
        start,
        sol,
        spec.joint_max_velocity,
        spec.joint_max_acceleration,
        spec.joint_max_jerk,
    )
    if frames is None:
        return _infeasible(names, "관절 프로파일 계산 실패 (Ruckig 입력 오류)")
    tcp_trace = [kin.fk(f)[0] for f in frames]
    return PlanPreviewResponse(
        feasible=True,
        joint_names=names,
        frames=frames,
        tcp_trace=tcp_trace,  # type: ignore[arg-type]
        message="",
    )


def _plan_move_l(
    kin: ArmKinematics,
    spec: RndRobotSpec,
    start: list[float],
    target_pos: Vec3,
    target_quat: Quat | None,
) -> PlanPreviewResponse:
    """TCP 직선. 경로 길이 s 를 Ruckig(카테시안 한계)로 시간프로파일 → 매 s 에서
    위치 lerp + 자세 slerp → seed 연쇄 IK. 가감속이 s 진행에 실린다."""
    names = spec.joint_names
    start_pos, start_quat = kin.fk(start)
    length = _dist(start_pos, target_pos)
    if length < 1e-6:
        return PlanPreviewResponse(
            feasible=True,
            joint_names=names,
            frames=[list(start)],
            tcp_trace=[start_pos],  # type: ignore[arg-type]
            message="",
        )

    slerp = (
        Slerp([0.0, 1.0], Rotation.from_quat([start_quat, target_quat]))
        if target_quat is not None
        else None
    )

    def quat_at(frac: float) -> Quat | None:
        if slerp is None:
            return None  # position-only
        q = slerp(frac).as_quat()
        return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))

    s_profile = _profile_1d(
        [0.0],
        [length],
        [spec.cartesian_max_velocity],
        [spec.cartesian_max_acceleration],
        [spec.cartesian_max_jerk],
    )
    if s_profile is None:
        return _infeasible(names, "직선 프로파일 계산 실패 (Ruckig 입력 오류)")

    s0 = np.asarray(start_pos)
    delta = np.asarray(target_pos) - s0
    frames: list[list[float]] = [list(start)]
    chain = list(start)
    fail_at: int | None = None
    message = ""
    for row in s_profile[1:]:  # 0 번째 = s=0 = start (이미 넣음)
        frac = min(max(row[0] / length, 0.0), 1.0)
        wp_arr = s0 + delta * frac
        wp: Vec3 = (float(wp_arr[0]), float(wp_arr[1]), float(wp_arr[2]))
        q = quat_at(frac)
        sol = kin.ik(wp, q, chain)
        pos_err, ori_err = _residuals(kin, sol, wp, q)
        if pos_err > _REACH_TOL_M or ori_err > _REACH_ORI_TOL_DEG:
            fail_at = len(frames)
            message = (
                f"직선 {length * frac * 100:.1f}cm 지점 도달 불가 "
                f"(위치 {pos_err * 1000:.1f}mm / 자세 {ori_err:.2f}°)"
            )
            break
        frames.append(sol)
        chain = sol

    tcp_trace = [kin.fk(f)[0] for f in frames]
    return PlanPreviewResponse(
        feasible=fail_at is None,
        joint_names=names,
        frames=frames,
        tcp_trace=tcp_trace,  # type: ignore[arg-type]
        fail_at_sample=fail_at,
        message=message,
    )


@publishes((Motion.Stream.TCP_STATE, TcpState))
class RndMotionModule:
    def __init__(
        self,
        runtime: ModuleRuntime,
        robot_id: str,
        spec: RndRobotSpec,
    ) -> None:
        self.runtime = runtime
        self.robot_id = robot_id
        self._spec = spec
        self._kin: ArmKinematics | None = None
        self._joints = list(spec.home_joints)  # sim 관절상태 (자기소유)
        self._seq = 0
        self._stop = False
        self._tcp_task: asyncio.Task[None] | None = None

    async def start(self) -> None:
        # URDF/pybullet 로드 (blocking) → to_thread (async 계약).
        self._kin = await asyncio.to_thread(
            ArmKinematics, self._spec.urdf_path, self._spec.joint_names
        )
        if len(self._joints) != self._kin.dof:
            raise ValueError(
                f"home_joints dof {len(self._joints)} != URDF dof {self._kin.dof} "
                f"(robot={self.robot_id})"
            )
        self._stop = False
        self._tcp_task = asyncio.create_task(self._tcp_state_loop())
        logger.info(
            "rnd_motion 시작 robot=%s dof=%d home=%s",
            self.robot_id,
            self._kin.dof,
            [round(v, 3) for v in self._joints],
        )

    async def stop(self) -> None:
        self._stop = True
        task = self._tcp_task
        if task is not None:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
        if self._kin is not None:
            self._kin.close()
            self._kin = None

    @service(MotionPreview.Service.PLAN)
    async def plan(self, req: PlanPreviewRequest) -> PlanPreviewResponse:
        """plan-only 미리보기 — 로봇 안 움직임(고스트만). 궤적+트레이스 반환."""
        if req.robot_id != self.robot_id:
            raise RuntimeError(
                f"rnd_motion({self.robot_id}) 가 다른 robot 요청 받음: {req.robot_id}"
            )
        assert self._kin is not None
        if len(req.start_joints) != self._kin.dof:
            raise RuntimeError(
                f"start_joints dof 불일치 ({len(req.start_joints)} != {self._kin.dof})"
            )
        result = await asyncio.to_thread(
            plan_trajectory,
            self._kin,
            self._spec,
            list(req.start_joints),
            req.target,
            req.mode,
            req.use_orientation,
        )
        logger.info(
            "rnd_motion plan robot=%s mode=%s use_ori=%s → %s frames=%d",
            self.robot_id,
            req.mode.value,
            req.use_orientation,
            "OK" if result.feasible else "INFEASIBLE",
            len(result.frames),
        )
        return result

    # ── TCP state loop (sim 상태를 라이브처럼 발행) ──────────────
    async def _tcp_state_loop(self) -> None:
        interval = 1.0 / _TCP_STATE_HZ
        try:
            while not self._stop:
                try:
                    self.runtime.publish(Motion.Stream.TCP_STATE, self._tcp_state())
                except Exception:
                    logger.exception("TCP state publish 실패 %s", self.robot_id)
                await asyncio.sleep(interval)
        except asyncio.CancelledError:
            pass

    def _tcp_state(self) -> TcpState:
        assert self._kin is not None
        pos, quat = self._kin.fk(self._joints)
        state = TcpState(
            robot_id=self.robot_id,
            seq=self._seq,
            timestamp_unix=time.time(),
            position=pos,
            quaternion=quat,
            joint_names=list(self._spec.joint_names),
            joints=list(self._joints),
            gripper_joint_name=None,
            gripper_rad=None,
            calibration_applied=False,
            calibration_stale=False,
        )
        self._seq += 1
        return state
