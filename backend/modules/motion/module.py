from __future__ import annotations

import asyncio
import logging
import math
import time
from pathlib import Path
from typing import Callable

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from framework.contract.mirror import Mirror
from framework.contract.publisher import publishes
from framework.contract.service import service
from framework.contract.subscriber import subscriber
from framework.runtime.api import ModuleRuntime
from modules.calibration.contract import (
    Calibration,
    CalibrationActivated,
    CalibrationBundle,
    SnapshotBundleRequest,
)
from modules.motor.contract import JointCommand, JointState, Motor
from modules.motor.layout import MotorSpec

from . import units
from .contract import (
    JogJInput,
    JogTcpInput,
    MotionCompleted,
    MotionFailed,
    MotionRejected,
    Motion,
    JointTarget,
    MoveJRequest,
    MoveJResponse,
    MoveLRequest,
    MoveLResponse,
    PoseTarget,
    ResolveReachableRequest,
    ResolveReachableResponse,
    StopRequest,
    StopResponse,
    TcpPose,
    TcpSnapshotRequest,
    TcpState,
    TrajState,
    TrajStatus,
)
from .kinematics import Kinematics
from .kinematics_builder import build_calibrated_kinematics
from .trajectory_runner import LinearPath, TrajectoryRunner

logger = logging.getLogger(__name__)

_TCP_STATE_HZ = 20.0

# Jog 입력이 0.2초 이상 끊기면 새 Jog로 간주하고,
# 현재 인코더 위치를 기준(ref)으로 다시 잡아 이전 기준과의 오차 누적을 방지한다.
_JOG_IDLE_RESET_S = 0.2

# ── 직선(MoveL) 경로 실현성 검증 상수 ──────────────────────────
# 샘플 간격 — 끝점은 풀려도 중간 s 에서만 못 풀리는 경우가 실재 (2026-07-09).
_PATH_STEP_M = 0.01
# 인접 샘플 해 사이 관절 도약 상한 — MoveIt jump_threshold 등가. 정상 추종은
# 1cm 당 ≤5° (2026-07-14 시뮬 실측), 구성 플립(특이점 근접)은 수십°~180° 로
# 불연속 — 20° 는 정상의 4배 마진이면서 플립을 확실히 자름.
_PATH_MAX_JUMP_RAD = 0.35  # ≈20°

# ── resolve_reachable 게이트 예산 ─────────────────────────────
# cheap→expensive (grasp_redesign_journey.md §5.5): 실패 기각 비용 = 예산에
# 비례하므로 싼 게이트를 전 그룹에 먼저. 아래 IK 예산 점증은 2026-07-09 벤치
# 근거 (가용 자세 median 8회 수렴 / 단일 풀예산 패스는 실패 케이스 10× 악화).
_SCREEN_IK_BUDGET = 5  # ① position-only 위치 스크린
_IK_BUDGETS = (10, 40, None)  # ② 자세 IK deepening (None = 실행용 풀예산)
_PATH_IK_BUDGET = 10  # ⑤ 경로 샘플 IK — 보수적 소예산 (미수렴 = 후보 기각)
# ④ 관절 보간 경로 샘플 간격 — 최대 관절 이동 기준 (IK 없음, 충돌 검사만이라
# 촘촘해도 싸다. 5° 면 SO-101 링크 끝 이동 ~수 mm 단위 해상도).
_JOINT_PATH_STEP_RAD = math.radians(5.0)


def _linear_path_blocker(
    kin: Kinematics,
    p0: tuple[float, float, float],
    q0: tuple[float, float, float, float] | None,
    p1: tuple[float, float, float],
    q1: tuple[float, float, float, float] | None,
    seed: list[float],
    *,
    restarts: int | None = None,
) -> str | None:
    """직선 p0→p1 (자세 q0→q1 slerp 동기) 을 _PATH_STEP_M 간격 IK 로 검증.

    막히면 사유 문자열 (사용자 표시용), 전 구간 통과면 None. 검증 = 실행과 같은
    보간 (MoveL 의 위치 lerp + 자세 slerp) — 검증 통과 == 실행 가능 이 성립해야
    "계획 시점 기각" 이 의미 있음. 인접 샘플 해 사이 관절 도약(구성 플립)도 기각
    (jump_threshold 등가 — 끝점만 보면 플립 경로가 통과해 실행 중 팔이 튐).
    blocking (IK) — 호출자가 to_thread 책임.
    """
    start = np.asarray(p0, dtype=float)
    delta = np.asarray(p1, dtype=float) - start
    length = float(np.linalg.norm(delta))
    ori_slerp = (
        Slerp([0.0, 1.0], Rotation.from_quat([q0, q1]))
        if q0 is not None and q1 is not None
        else None
    )

    def _quat_at(frac: float) -> tuple[float, float, float, float] | None:
        if ori_slerp is not None:
            q = ori_slerp(frac).as_quat()
            return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
        return q1  # q0 미상(None)이면 목표 자세 고정 / q1=None 은 position-only

    n = max(2, int(length / _PATH_STEP_M))
    chain = list(seed)
    for i in range(1, n + 1):
        frac = i / n
        wp = start + delta * frac
        sol = kin.ik((wp[0], wp[1], wp[2]), _quat_at(frac), chain, restarts)
        if sol is None:
            return f"시작 {length * frac * 100:.1f}cm 지점 IK 불가"
        jump = max(abs(a - b) for a, b in zip(sol, chain))
        if jump > _PATH_MAX_JUMP_RAD:
            return (
                f"시작 {length * frac * 100:.1f}cm 지점 관절 도약 "
                f"{math.degrees(jump):.0f}° — 구성 플립 (특이점 근접)"
            )
        chain = list(sol)
    return None


@publishes(
    (Motor.Stream.COMMAND, JointCommand),
    (Motion.Stream.TCP_STATE, TcpState),
    (Motion.Stream.TRAJ_STATE, TrajState),
    (Motion.Event.MOTION_COMPLETED, MotionCompleted),
)
class MotionModule:
    calibration: Mirror[CalibrationBundle] = Mirror(
        snapshot_service=Calibration.Service.SNAPSHOT_BUNDLE,
        snapshot_req=lambda self: SnapshotBundleRequest(robot_id=self.robot_id),
        change_topic=Calibration.Event.ACTIVATED,
        value_cls=CalibrationBundle,
        change_event_cls=CalibrationActivated,
    )

    def __init__(
        self,
        runtime: ModuleRuntime,
        robot_id: str,
        kinematics_factory: Callable[[Path], Kinematics],
        urdf_path: Path,
        arm_specs: list[MotorSpec],
        joint_max_velocity: list[float],
        joint_max_acceleration: list[float],
        joint_max_jerk: list[float],
        cartesian_max_velocity: float,
        cartesian_max_acceleration: float,
        cartesian_max_jerk: float,
        gripper_spec: MotorSpec | None = None,
        gripper_index: int | None = None,
    ) -> None:
        self.runtime = runtime
        self.robot_id = robot_id
        self._kin_factory = kinematics_factory
        self._urdf_path = Path(urdf_path)
        self._kin: Kinematics | None = None
        self._joint_off: list[float] | None = None
        self._calibration_applied = False
        self._calibration_stale = False
        self._applied_signature: tuple[tuple[str, int], ...] | None = None
        self._apply_lock = asyncio.Lock()
        self._retired_kins: list[Kinematics] = []
        self._arm = arm_specs
        self._dof = len(arm_specs)
        # gripper report (arm 아님) — URDF 시각화용. spec/index 둘 다 있어야 활성.
        self._gripper_spec = gripper_spec
        self._gripper_index = gripper_index
        self._latest_gripper_rad: float | None = None
        self._j_max_vel = joint_max_velocity
        self._j_max_acc = joint_max_acceleration
        self._j_max_jerk = joint_max_jerk
        self._c_max_vel = cartesian_max_velocity
        self._c_max_acc = cartesian_max_acceleration
        self._c_max_jerk = cartesian_max_jerk

        self._latest_arm_rad: list[float] | None = None
        self._runner: TrajectoryRunner | None = None
        self._joint_limits: list[tuple[float, float]] = []
        self._tcp_seq = 0
        self._cmd_seq = 0
        self._traj_seq = 0
        self._tcp_task: asyncio.Task[None] | None = None
        self._stop = False
        self._loop: asyncio.AbstractEventLoop | None = None
        self._move_done: asyncio.Future[TrajStatus] | None = None
        self._jog_j_ref: list[float] | None = None
        self._jog_j_t = 0.0
        self._jog_tcp_pos: np.ndarray | None = None
        self._jog_tcp_quat: np.ndarray | None = None
        self._jog_tcp_t = 0.0
        self._jog_tcp_reject_last_log = 0.0
        self._jog_tcp_reject_count = 0

    # ── lifecycle ─────────────────────────────────────────────

    async def start(self) -> None:
        # move 완료 future 를 traj thread 에서 resolve 하려면 loop 참조 필요.
        self._loop = asyncio.get_running_loop()
        # D4 — calibration Mirror. Runtime 이 start 전에 initial snapshot 을
        # 시도했으므로 (owner 떠 있으면) 이미 도착해 있음. 미도착 = 무보정 부팅,
        # owner 등장 시 on_change 가 live 적용 (수렴 — 재시작 불필요).
        bundle = self.calibration.peek()
        if bundle is None:
            logger.warning(
                "calibration 미도달 robot=%s — 무보정 부팅 "
                "(owner 등장 시 자동 적용, TcpState.calibration_applied 로 관측)",
                self.robot_id,
            )
        # URDF/PyBullet/FkChain 로드 (blocking) → to_thread (async 계약)
        await asyncio.to_thread(self._build_kinematics, bundle)
        assert self._kin is not None
        self._joint_limits = self._kin.joint_limits()  # jog ref clamp 용 (URDF rad)
        self._runner = TrajectoryRunner(
            n_arm=self._dof,
            joint_max_velocity=self._j_max_vel,
            joint_max_acceleration=self._j_max_acc,
            joint_max_jerk=self._j_max_jerk,
            cartesian_max_velocity=self._c_max_vel,
            cartesian_max_acceleration=self._c_max_acc,
            cartesian_max_jerk=self._c_max_jerk,
            release_profile=lambda: True,  # D2 no-op
            restore_profile=lambda: True,
            publish_cmd=self._publish_cmd,
            publish_state=self._publish_traj_state,
            solve_ik=self._solve_ik,
            get_joint_angles=lambda: self._latest_arm_rad,
        )
        self._stop = False
        self._tcp_task = asyncio.create_task(self._tcp_state_loop())
        # build 진행 중 mirror 가 도착한 race 창 회수 — on_change 는 _kin 미빌드
        # 시점(start 전)이라 무시했으므로 여기서 놓친 값을 직접 적용.
        late = self.calibration.peek()
        if bundle is None and late is not None:
            asyncio.create_task(self._apply_bundle(late))

    async def stop(self) -> None:
        self._stop = True
        if self._runner is not None:
            self._runner.stop()
        task = self._tcp_task
        if task is not None:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
            self._tcp_task = None
        if self._kin is not None:
            self._kin.close()
        for retired in self._retired_kins:
            try:
                retired.close()
            except Exception:
                pass
        self._retired_kins.clear()

    # ── motor state 구독 → arm rad cache ──────────────────────

    @subscriber(Motor.Stream.RAW_STATE)
    def on_motor_state(self, state: JointState) -> None:
        if state.robot_id != self.robot_id:
            return
        if len(state.positions_raw) < self._dof:
            return
        self._latest_arm_rad = units.joints_raw_to_rad(
            state.positions_raw[: self._dof], self._arm, self._joint_off
        )
        # gripper rad (units SSOT, 캘 없음) — arm 과 별도. URDF open/close 시각화용.
        if (
            self._gripper_spec is not None
            and self._gripper_index is not None
            and self._gripper_index < len(state.positions_raw)
        ):
            self._latest_gripper_rad = units.raw_to_rad(
                state.positions_raw[self._gripper_index], self._gripper_spec
            )

    # ── jog (50Hz velocity 입력 → 적분 → command) ─────────────

    @subscriber(Motion.Stream.JOG_J)
    def on_jog_j(self, inp: JogJInput) -> None:
        if inp.robot_id != self.robot_id or len(inp.velocities) != self._dof:
            return
        now = time.time()
        if self._runner is not None and self._runner.is_running:
            self._runner.stop()
            self._jog_j_ref = None
        # 속도 cap — motion.yaml max_velocity (안전, 임의 수치 아님)
        vel = [
            max(-self._j_max_vel[i], min(self._j_max_vel[i], inp.velocities[i]))
            for i in range(self._dof)
        ]
        idle = self._jog_j_ref is None or (now - self._jog_j_t) > _JOG_IDLE_RESET_S
        if idle:
            cur = self._latest_arm_rad
            if cur is None:
                return
            self._jog_j_ref = list(cur)
            self._jog_j_t = now
            self._publish_cmd(list(self._jog_j_ref))
            return
        dt = now - self._jog_j_t
        assert self._jog_j_ref is not None  # idle=False → latch 됨
        ref = self._jog_j_ref
        for i in range(self._dof):
            ref[i] += vel[i] * dt
            if i < len(self._joint_limits):  # URDF joint limit clamp (안전)
                lo, hi = self._joint_limits[i]
                ref[i] = max(lo, min(hi, ref[i]))
        self._jog_j_t = now
        self._publish_cmd(list(ref))

    @subscriber(Motion.Stream.JOG_TCP)
    def on_jog_tcp(self, inp: JogTcpInput) -> None:
        if inp.robot_id != self.robot_id:
            return
        kin = self._kin
        if kin is None:  # start() 전 입력 — kinematics 미빌드
            return
        now = time.time()
        if self._runner is not None and self._runner.is_running:
            self._runner.stop()
            self._jog_tcp_pos = None
        cur = self._latest_arm_rad
        if cur is None:
            return
        linear = np.asarray(inp.linear, dtype=float)
        angular = np.asarray(inp.angular, dtype=float)
        # linear cap — motion.yaml max_trans_vel (안전). angular 는 IK reject 에 맡김.
        lin_mag = float(np.linalg.norm(linear))
        if lin_mag > self._c_max_vel:
            linear = linear / lin_mag * self._c_max_vel

        idle = (
            self._jog_tcp_pos is None
            or self._jog_tcp_quat is None
            or (now - self._jog_tcp_t) > _JOG_IDLE_RESET_S
        )
        if idle:
            pos, quat = kin.fk(cur)
            self._jog_tcp_pos = np.asarray(pos, dtype=float)
            self._jog_tcp_quat = np.asarray(quat, dtype=float)
            self._jog_tcp_t = now
            sol = kin.ik(pos, quat, cur)
            if sol is not None:
                self._publish_cmd(sol)
            return

        prev_pos = self._jog_tcp_pos
        prev_quat = self._jog_tcp_quat
        assert prev_pos is not None and prev_quat is not None
        dt = now - self._jog_tcp_t
        new_pos, new_quat = prev_pos, prev_quat
        ang_mag = float(np.linalg.norm(angular))
        if np.any(linear):
            if inp.frame == "base":
                new_pos = prev_pos + linear * dt
            else:  # tcp frame
                new_pos = prev_pos + Rotation.from_quat(prev_quat).apply(linear) * dt
        if ang_mag > 1e-9:
            delta = Rotation.from_rotvec(angular * dt)
            cur_r = Rotation.from_quat(prev_quat)
            new_r = (delta * cur_r) if inp.frame == "base" else (cur_r * delta)
            new_quat = new_r.as_quat()

        target_pos_tuple = (float(new_pos[0]), float(new_pos[1]), float(new_pos[2]))
        # Pure translation jog (angular = 0) → position-only IK.
        # 이유: teach-pendant 표준 (UR/ABB pure X/Y/Z jog 은 orientation drift 허용).
        # 옛 backend cartesian path 도 `servo_tcp(pos, None, angles)` = position-only
        # ([backend/nodes/device/motion_node.py:101]). 6DOF exact solve 는 arm 최대
        # reach 근처 자리 orientation 을 매 프레임 pin 하면 IK 수렴 실패 — 2026-07-01
        # SO-101 Z+ jog IK reject 진단 (reason=orientation-only-fail).
        # angular 도 있으면 사용자 명시 의도라 6DOF exact.
        if ang_mag > 1e-9:
            target_quat_tuple: tuple[float, float, float, float] | None = (
                float(new_quat[0]),
                float(new_quat[1]),
                float(new_quat[2]),
                float(new_quat[3]),
            )
        else:
            target_quat_tuple = None
        sol = kin.ik(target_pos_tuple, target_quat_tuple, cur)
        if sol is None:
            # IK 실패 → ref 적분 전 값 유지 (마지막 valid hold, 누적 X)
            # 원인 분리 진단: position-only IK 도 시도해서 orientation vs reachability 판별.
            #   - pos-only 도 실패 = 진짜 unreachable (target 자체 workspace 밖)
            #   - pos-only 만 성공 = orientation constraint 가 blocker (arm 최대 reach 근처
            #     자리 특정 orientation 유지 불가능 — SO-101 6DOF exact solve 편차)
            self._jog_tcp_reject_count += 1
            if now - self._jog_tcp_reject_last_log > 0.5:
                sol_pos_only = kin.ik(target_pos_tuple, None, cur)
                reason = (
                    "orientation-only-fail (pos-only OK)"
                    if sol_pos_only is not None
                    else "unreachable (both fail)"
                )
                cur_deg = [round(a * 180.0 / 3.14159265, 1) for a in cur]
                logger.warning(
                    "JogTcp IK reject robot=%s frame=%s target_pos=%s linear=%s "
                    "reason=%s current_joints_deg=%s (count=%d in %.1fs)",
                    self.robot_id,
                    inp.frame,
                    target_pos_tuple,
                    inp.linear,
                    reason,
                    cur_deg,
                    self._jog_tcp_reject_count,
                    now - self._jog_tcp_reject_last_log
                    if self._jog_tcp_reject_last_log
                    else 0.0,
                )
                self._jog_tcp_reject_last_log = now
                self._jog_tcp_reject_count = 0
            return
        self._jog_tcp_pos = new_pos
        # position-only IK 성공 시 실 FK(sol) quat 로 sync — angular jog 로 전환 시
        # stale reference 방지. angular 자리는 target 그대로 (사용자 의도).
        if ang_mag > 1e-9:
            self._jog_tcp_quat = new_quat
        else:
            _, actual_quat = kin.fk(sol)
            self._jog_tcp_quat = np.asarray(actual_quat, dtype=float)
        self._jog_tcp_t = now
        self._publish_cmd(sol)

    # ── services ──────────────────────────────────────────────

    def _corrected_target_pos(
        self, pt: PoseTarget, seed: list[float]
    ) -> tuple[float, float, float]:
        """tcp_offset 적용한 목표 위치 (base frame). None 이면 pt.position 그대로.

        tcp+tcp_offset(tool frame) 지점을 pt.position 에 맞추려면 tcp 목표를
        pose-R 만큼 역보정: IK(pose)→자세 R → position - R·tcp_offset. MoveJ/MoveL
        공용 (제어점 보정은 planner 무관 = 목표 정의).
        """
        assert self._kin is not None
        if pt.tcp_offset is None:
            return pt.position
        sol = self._kin.ik(pt.position, pt.quaternion, seed)
        if sol is None:
            raise MotionRejected("IK 실패 — pose 도달 불가 (tcp_offset 보정 전)")
        rot, _ = self._kin.fk_to_matrix(sol)
        base_off = np.asarray(rot) @ np.asarray(pt.tcp_offset, dtype=float)
        corrected = np.asarray(pt.position, dtype=float) - base_off
        return (float(corrected[0]), float(corrected[1]), float(corrected[2]))

    @service(Motion.Service.MOVE_J)
    async def move_j(self, req: MoveJRequest) -> MoveJResponse:
        """관절 보간 이동 — target 이 JointTarget(관절값) 또는 PoseTarget(pose→IK).

        pose 는 관절 보간이라 "자세 고정한 채 직선"(MoveL)이 강제하는 높이-의존 IK
        실패가 없다. pick 접근/승강이 pose 를 씀 (2026-07-07 — MoveL 자세 고정 접근이
        SO-101 workspace 에서 구조적으로 실패해 전환). 거부 = raise (MotionRejected).
        """
        current = self._latest_arm_rad
        if current is None:
            raise MotionRejected("motor state 아직 없음")
        match req.target:
            case JointTarget(joints=joints):
                if len(joints) != self._dof:
                    raise MotionRejected(
                        f"joints dof 불일치 ({len(joints)} != {self._dof})"
                    )
                target = list(joints)
            case PoseTarget() as pt:
                assert self._kin is not None  # start() 이후만 서비스 도달
                corrected = self._corrected_target_pos(pt, current)
                sol = self._kin.ik(corrected, pt.quaternion, current)
                if sol is None:
                    raise MotionRejected("IK 실패 — pose 도달 불가")
                target = sol
        if not self._begin_move():
            raise MotionRejected("이전 motion 진행 중")
        assert self._runner is not None and self._move_done is not None
        fut = self._move_done
        self._runner.run_joint(list(current), target)
        await self._require_done(fut, "MoveJ")
        return MoveJResponse()

    @service(Motion.Service.MOVE_L)
    async def move_l(self, req: MoveLRequest) -> MoveLResponse:
        """TCP 를 현재 위치 → target(pose) 직선 이동. 완료까지 대기.

        자세 = pt.quaternion 이 **목표 자세** — 현재 자세(FK)에서 경로 s 에 동기해
        slerp 보간 (UR/ABB/MoveIt 식, 자세 고정은 현재≈목표인 특수 케이스).
        None = position-only. tcp_offset 은 제어점 보정한 끝점 (MoveJ 와 동일)."""
        pt = req.target
        current = self._latest_arm_rad
        if current is None:
            raise MotionRejected("motor state 아직 없음")
        assert self._kin is not None  # start() 이후만 서비스 도달
        end_pos = self._corrected_target_pos(pt, current)
        start_pos, start_quat = self._kin.fk(current)
        path = LinearPath(
            np.asarray(start_pos, dtype=float),
            np.asarray(end_pos, dtype=float),
        )
        # 경로 사전 검증 (fail-fast) — runner 는 실행 중 step IK 실패 시 그 자리
        # 공중 정지(FAILED). MoveL 은 끝점은 풀려도 중간 s 에서만 못 풀리는 경우가
        # 실재 (2026-07-09 PnP 기울인 approach) → 시작 전 샘플 IK + 구성 플립
        # (jump) 검증, 안 풀리면 모션 0 으로 reject. blocking → to_thread
        # (jog/stream 이벤트 루프 안 굶김 — resolve_reachable 과 동일 이유).
        kin = self._kin
        blocker = await asyncio.to_thread(
            _linear_path_blocker,
            kin,
            (start_pos[0], start_pos[1], start_pos[2]),
            start_quat if pt.quaternion is not None else None,
            end_pos,
            pt.quaternion,
            list(current),
        )
        if blocker is not None:
            raise MotionRejected(f"경로 IK 실패 — {blocker}")
        if not self._begin_move():
            raise MotionRejected("이전 motion 진행 중")
        assert self._runner is not None and self._move_done is not None
        fut = self._move_done
        self._runner.run_cartesian(path, list(current), pt.quaternion, start_quat)
        await self._require_done(fut, "MoveL")
        return MoveLResponse()

    @service(Motion.Service.RESOLVE_REACHABLE)
    async def resolve_reachable(
        self, req: ResolveReachableRequest
    ) -> ResolveReachableResponse:
        """후보 그룹 가용성 판정 (모션 0) — cheap→expensive 게이트 파이프라인.

        게이트 순서 = 계약 docstring (contract.py ResolveReachableRequest):
        ① 위치 스크린 → ② 자세 IK deepening → ③ 바닥 충돌 → ③b 장애물 점군
        충돌 → ④ 관절 보간 경로 → ⑤ 직선 경로. 싼 게이트를 전 그룹에 먼저
        돌려 불가 그룹 기각을 싸게 (deepening 의 "싼 예산 먼저" 정신 계승 —
        2026-07-09 벤치: 단일 풀예산 패스는 실패 케이스 10× 악화). 미래
        cross-robot 충돌 게이트는 ③b 옆에 낀다.

        채택 그룹의 IK 해를 반환 — 실행부가 재계산 없이 그 관절로 이동 (판정
        해 == 실행 해). IK 는 blocking(pybullet) + 그룹 다수라 to_thread.
        """
        current = self._latest_arm_rad
        if current is None:
            return ResolveReachableResponse(index=-1, message="motor state 아직 없음")
        assert self._kin is not None  # start() 이후만 서비스 도달
        kin = self._kin
        groups = req.groups
        has_obstacles = bool(req.obstacle_points)
        if req.path_from is not None and len(req.path_from) != self._dof:
            raise MotionRejected(
                f"path_from dof 불일치 ({len(req.path_from)} != {self._dof})"
            )

        def _screen(group: list[TcpPose]) -> bool:
            # ① position-only 소예산 — 위치 자체가 workspace 밖이면 자세는 볼
            # 것도 없다 (자세 실패의 대다수가 여기서 싸게 걸러지진 않지만,
            # 완전 범위 밖 후보를 풀예산까지 끌고 가는 것을 차단).
            seed = list(current)
            for pose in group:
                sol = kin.ik(pose.position, None, seed, _SCREEN_IK_BUDGET)
                if sol is None:
                    return False
                seed = list(sol)
            return True

        def _solve(
            group: list[TcpPose], budget: int | None
        ) -> list[list[float]] | None:
            # ② 전 pose 자세 IK — seed 연쇄 (앞 해 → 다음 seed, 가까운 pose 는
            # 1발 수렴). 실패 그룹만 restart 예산 풀비용.
            seed = list(current)
            sols: list[list[float]] = []
            for pose in group:
                sol = kin.ik(pose.position, pose.quaternion, seed, budget)
                if sol is None:
                    return None
                sols.append(sol)
                seed = list(sol)
            return sols

        def _floor_blocked(sols: list[list[float]]) -> bool:
            # ③ 바닥 평면 충돌 — 해 자세에서 로봇 링크 침투 기각. 해(구성)
            # 종속이라 엄밀히는 다른 IK 가지가 통과할 수도 있으나, 6DOF 전체
            # 자세 IK 의 가지는 소수 + 후보 가족이 넓어 보수 기각이 싸다.
            if req.floor_z is None:
                return False
            return any(kin.floor_collision(s, req.floor_z) for s in sols)

        def _obstacle_blocked(sols: list[list[float]]) -> bool:
            # ③b 장애물 점군 충돌 — 그리퍼(벌림 반영)가 관측 점군을 침투하는
            # 후보 기각 (§10.4-3). 점군은 _scan 진입 시 1회 로드.
            if not has_obstacles:
                return False
            return any(
                kin.obstacle_collision(s, gripper_open=req.gripper_open)
                for s in sols
            )

        def _joint_path_blocked(sols: list[list[float]]) -> bool:
            # ④ path_from → 첫 pose 해 관절 보간 경로 — naive MoveJ 가 바닥/
            # 물체를 스치는 것을 계획 시점 기각 (§10.4-4). IK 없음 (충돌만).
            if req.path_from is None:
                return False
            qa = np.asarray(req.path_from, dtype=float)
            qb = np.asarray(sols[0], dtype=float)
            n = max(2, int(math.ceil(float(np.max(np.abs(qb - qa)))
                                     / _JOINT_PATH_STEP_RAD)))
            for k in range(1, n + 1):
                q = [float(v) for v in qa + (qb - qa) * (k / n)]
                if kin.self_collision(q):
                    return True
                if req.floor_z is not None and kin.floor_collision(q, req.floor_z):
                    return True
                if has_obstacles and kin.obstacle_collision(
                    q, gripper_open=req.gripper_open
                ):
                    return True
            return False

        def _linear_blocked(group: list[TcpPose], sols: list[list[float]]) -> bool:
            # ⑤ 연속 pose 사이 직선 실현성 — MoveL 실행 전제 (끝점만 보면 중간
            # 실패가 실행 시점에 터짐 — 그 거부를 계획 시점으로 앞당김).
            if not req.linear:
                return False
            for k in range(len(group) - 1):
                blocker = _linear_path_blocker(
                    kin,
                    group[k].position,
                    group[k].quaternion,
                    group[k + 1].position,
                    group[k + 1].quaternion,
                    sols[k],
                    restarts=_PATH_IK_BUDGET,
                )
                if blocker is not None:
                    return True
            return False

        def _scan() -> tuple[int, list[list[float]], str]:
            alive = [gi for gi in range(len(groups)) if _screen(groups[gi])]
            if not alive:
                return -1, [], f"전 후보({len(groups)}) 위치가 workspace 밖"
            rejected: set[int] = set()  # ③~⑤ 확정 기각 (예산과 무관한 불가)
            gate_rejects = {"floor": 0, "obstacle": 0, "joint_path": 0, "path": 0}
            for budget in _IK_BUDGETS:
                for gi in alive:
                    if gi in rejected:
                        continue
                    sols = _solve(groups[gi], budget)
                    if sols is None:
                        continue  # 예산 부족일 수 있음 — 다음 단계 재시도
                    if _floor_blocked(sols):
                        rejected.add(gi)
                        gate_rejects["floor"] += 1
                        continue
                    if _obstacle_blocked(sols):
                        rejected.add(gi)
                        gate_rejects["obstacle"] += 1
                        continue
                    if _joint_path_blocked(sols):
                        rejected.add(gi)
                        gate_rejects["joint_path"] += 1
                        continue
                    if _linear_blocked(groups[gi], sols):
                        rejected.add(gi)
                        gate_rejects["path"] += 1
                        continue
                    return gi, sols, ""
            return -1, [], (
                f"가용 그룹 없음 — 위치 통과 {len(alive)}/{len(groups)}, "
                f"자세 IK 실패 {len(alive) - len(rejected)}, "
                f"바닥 충돌 기각 {gate_rejects['floor']}, "
                f"장애물(그리퍼↔물체) 기각 {gate_rejects['obstacle']}, "
                f"이동 경로 기각 {gate_rejects['joint_path']}, "
                f"직선 경로 기각 {gate_rejects['path']}"
            )

        def _scan_with_obstacles() -> tuple[int, list[list[float]], str]:
            # 장애물 점군 lifecycle — 판정 동안만 scene 에 존재 (잔존 = 이후
            # 판정 오염). 미지정이면 set 호출 자체를 안 함 (fake kin 호환).
            if not has_obstacles:
                return _scan()
            kin.set_obstacle_points(req.obstacle_points)
            try:
                return _scan()
            finally:
                kin.set_obstacle_points(None)

        t0 = time.perf_counter()
        idx, sols, msg = await asyncio.to_thread(_scan_with_obstacles)
        logger.info(
            "resolve_reachable: groups=%d floor_z=%s linear=%s → index=%d (%.2fs)%s",
            len(groups), req.floor_z, req.linear, idx,
            time.perf_counter() - t0, f" [{msg}]" if msg else "",
        )
        return ResolveReachableResponse(index=idx, solutions=sols, message=msg)

    @service(Motion.Service.TCP_SNAPSHOT)
    def tcp_snapshot(self, req: TcpSnapshotRequest) -> TcpState:
        joints = self._latest_arm_rad
        if joints is None:
            raise RuntimeError("motor state 아직 없음 — TCP snapshot 불가")
        return self._tcp_state(joints)

    @service(Motion.Service.STOP)
    def stop_motion(self, req: StopRequest) -> StopResponse:
        if self._runner is not None:
            self._runner.stop()
        return StopResponse(ok=True)

    # ── calibration 변경 감지 ────────────────────────

    @calibration.on_change
    async def _on_calibration_change(
        self, old: CalibrationBundle | None, new: CalibrationBundle
    ) -> None:
        if self._kin is None:
            return
        if old is None:
            logger.info(
                "calibration 도착 (owner 늦은 부팅) — live 적용 robot=%s",
                self.robot_id,
            )
            await self._apply_bundle(new)
        elif new.signature() != old.signature():
            self._calibration_stale = True
            logger.warning(
                "calibration 변경 감지 robot=%s — 적용은 재시작 필요 "
                "(TcpState.calibration_stale=True)",
                self.robot_id,
            )

    async def _apply_bundle(self, bundle: CalibrationBundle) -> None:
        async with self._apply_lock:
            if self._applied_signature == bundle.signature():
                return
            while self._runner is not None and self._runner.is_running:
                await asyncio.sleep(0.2)
            old_kin = self._kin
            await asyncio.to_thread(self._build_kinematics, bundle)
            assert self._kin is not None
            self._joint_limits = self._kin.joint_limits()
            self._calibration_stale = False
            if old_kin is not None:
                self._retired_kins.append(old_kin)
            logger.info("calibration live 적용 완료 robot=%s", self.robot_id)

    def _build_kinematics(self, bundle: CalibrationBundle | None) -> None:
        """bundle 적용 kinematics 빌드 (blocking — start 의 to_thread 안).

        구성 자체는 공유 빌더 (kinematics_builder — scan build 와 같은 의미 SSOT).
        여기선 lifecycle(initialize) + 모듈 상태 필드 반영만. 재빌드 idempotent —
        joint_off 는 빌더가 매번 새로 계산 (이전 빌드 잔존 차단, live 적용 경로).
        """
        built = build_calibrated_kinematics(
            self._urdf_path, self.robot_id, self._arm, bundle, self._kin_factory
        )
        built.kinematics.initialize()
        self._kin = built.kinematics
        self._joint_off = built.joint_offsets
        self._calibration_applied = bool(built.applied)
        self._applied_signature = bundle.signature() if bundle is not None else None
        if built.applied:
            logger.info(
                "calibration 적용 (robot=%s): %s%s",
                self.robot_id,
                "+".join(built.applied),
                f" urdf={built.urdf_path.name}"
                if "link_offset" in built.applied
                else "",
            )
        else:
            logger.info("calibration 없음 (robot=%s) — 무보정 기구학", self.robot_id)

    # ── move 완료 대기 ────────────────────────

    def _begin_move(self) -> bool:
        """새 move 완료 future 준비. 이미 진행 중이면 False — runner 는 단일 trajectory
        라 overlap 거부 (task 는 순차 await 라 안 걸림, 동시 호출만 방어)."""
        if self._move_done is not None and not self._move_done.done():
            return False
        assert self._loop is not None
        self._move_done = self._loop.create_future()
        return True

    async def _require_done(
        self, fut: "asyncio.Future[TrajStatus]", label: str
    ) -> None:
        """trajectory 종료까지 대기. DONE 아니면 MotionFailed (완료 계약)."""
        try:
            status = await fut
        finally:
            if self._move_done is fut:
                self._move_done = None
        if status != TrajStatus.DONE:
            raise MotionFailed(f"{label} {status.value}")

    def _resolve_move(self, status: TrajStatus) -> None:
        """traj thread 의 terminal 상태 → loop.call_soon_threadsafe 로 진입."""
        fut = self._move_done
        if fut is not None and not fut.done():
            fut.set_result(status)

    # ── TrajectoryRunner 콜백 ──────────────────────────────────

    def _publish_cmd(self, angles_rad: list[float]) -> None:
        # traj thread 에서 호출 — zenoh publish 는 thread-safe.
        # joint_offset 차감 (D4) — measured-frame 각 → 모터 명령 rad → raw.
        raw = units.joints_rad_to_raw(angles_rad, self._arm, self._joint_off)
        self.runtime.publish(
            Motor.Stream.COMMAND,
            JointCommand(
                robot_id=self.robot_id,
                seq=self._cmd_seq,
                timestamp_unix=time.time(),
                positions_raw=raw,
            ),
        )
        self._cmd_seq += 1

    def _publish_traj_state(self, status: TrajStatus, progress: float) -> None:
        self.runtime.publish(
            Motion.Stream.TRAJ_STATE,
            TrajState(
                robot_id=self.robot_id,
                seq=self._traj_seq,
                timestamp_unix=time.time(),
                status=status,
                progress=progress,
            ),
        )
        self._traj_seq += 1
        if status in (TrajStatus.DONE, TrajStatus.FAILED, TrajStatus.STOPPED):
            self.runtime.publish(
                Motion.Event.MOTION_COMPLETED,
                MotionCompleted(robot_id=self.robot_id, status=status),
            )
            # await motion.move_* 깨우기 — traj thread → loop 로 넘김 (thread-safe).
            loop = self._loop
            if loop is not None:
                loop.call_soon_threadsafe(self._resolve_move, status)

    def _solve_ik(self, pos, quat, seed: list[float]) -> list[float] | None:
        # cartesian path 추종 — quat=None 이면 position-only. runner 는 start() 이후 생성.
        assert self._kin is not None
        return self._kin.ik(pos, quat, seed)

    # ── TCP state loop (20Hz) ─────────────────────────────────

    async def _tcp_state_loop(self) -> None:
        interval = 1.0 / _TCP_STATE_HZ
        try:
            while not self._stop:
                joints = self._latest_arm_rad
                if joints is not None:
                    try:
                        self.runtime.publish(
                            Motion.Stream.TCP_STATE, self._tcp_state(joints)
                        )
                    except Exception:
                        logger.exception("TCP state publish 실패 %s", self.robot_id)
                await asyncio.sleep(interval)
        except asyncio.CancelledError:
            pass

    def _tcp_state(self, joints: list[float]) -> TcpState:
        assert self._kin is not None  # tcp loop / snapshot 은 start() 이후만
        pos, quat = self._kin.fk(joints)
        state = TcpState(
            robot_id=self.robot_id,
            seq=self._tcp_seq,
            timestamp_unix=time.time(),
            position=pos,
            quaternion=quat,
            joint_names=[s.name for s in self._arm],
            joints=list(joints),
            gripper_joint_name=(
                self._gripper_spec.name if self._gripper_spec is not None else None
            ),
            gripper_rad=self._latest_gripper_rad,
            calibration_applied=self._calibration_applied,
            calibration_stale=self._calibration_stale,
        )
        self._tcp_seq += 1
        return state
