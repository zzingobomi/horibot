from __future__ import annotations

import asyncio
import logging
import time
from pathlib import Path
from typing import Callable

import numpy as np
from scipy.spatial.transform import Rotation

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
    Motion,
    MoveJPoseRequest,
    MoveJRequest,
    MoveJResponse,
    MoveLRequest,
    MoveLResponse,
    StopRequest,
    StopResponse,
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

    @service(Motion.Service.MOVE_J)
    async def move_j(self, req: MoveJRequest) -> MoveJResponse:
        if len(req.target_joints) != self._dof:
            return MoveJResponse(
                accepted=False,
                message=f"target_joints dof 불일치 ({len(req.target_joints)} != {self._dof})",
            )
        current = self._latest_arm_rad
        if current is None:
            return MoveJResponse(accepted=False, message="motor state 아직 없음")
        if not self._begin_move():
            return MoveJResponse(accepted=False, message="이전 motion 진행 중")
        assert self._runner is not None and self._move_done is not None
        fut = self._move_done
        self._runner.run_joint(current, list(req.target_joints))
        await self._require_done(fut, "MoveJ")
        return MoveJResponse(accepted=True)

    @service(Motion.Service.MOVE_J_POSE)
    async def move_j_pose(self, req: MoveJPoseRequest) -> MoveJResponse:
        """목표 TCP pose → IK(현재 자세 seed, multi-restart) → 관절 공간 MoveJ.

        Cartesian 직선(MoveL) 아님 — 경로가 관절 보간이라 "자세 고정한 채 이동" 이
        강제하는 높이-의존 IK 실패가 없다. pick 접근/승강이 이걸 씀 (2026-07-07 —
        MoveL 자세 고정 접근이 SO-101 workspace 에서 구조적으로 실패해 전환).
        """
        current = self._latest_arm_rad
        if current is None:
            return MoveJResponse(accepted=False, message="motor state 아직 없음")
        assert self._kin is not None  # start() 이후만 서비스 도달
        target = self._kin.ik(req.target_position, req.target_quaternion, current)
        if target is None:
            return MoveJResponse(accepted=False, message="IK 실패 — pose 도달 불가")
        if not self._begin_move():
            return MoveJResponse(accepted=False, message="이전 motion 진행 중")
        assert self._runner is not None and self._move_done is not None
        fut = self._move_done
        self._runner.run_joint(list(current), target)
        await self._require_done(fut, "MoveJPose")
        return MoveJResponse(accepted=True)

    @service(Motion.Service.MOVE_L)
    async def move_l(self, req: MoveLRequest) -> MoveLResponse:
        """TCP 를 현재 위치 → target_position 직선 이동 (position-only v1). 완료까지 대기."""
        current = self._latest_arm_rad
        if current is None:
            return MoveLResponse(accepted=False, message="motor state 아직 없음")
        if not self._begin_move():
            return MoveLResponse(accepted=False, message="이전 motion 진행 중")
        assert self._runner is not None and self._move_done is not None
        assert self._kin is not None  # start() 이후만 서비스 도달
        fut = self._move_done
        start_pos, _ = self._kin.fk(current)
        path = LinearPath(
            np.asarray(start_pos, dtype=float),
            np.asarray(req.target_position, dtype=float),
        )
        self._runner.run_cartesian(path, list(current), req.target_quaternion)
        await self._require_done(fut, "MoveL")
        return MoveLResponse(accepted=True)

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
