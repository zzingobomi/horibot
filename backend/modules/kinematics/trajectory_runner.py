import logging
import threading
import time
from abc import ABC, abstractmethod
from typing import Callable

import numpy as np
from ruckig import (
    ControlInterface,
    InputParameter,
    OutputParameter,
    Result,
    Ruckig,
)
from scipy.interpolate import CubicSpline

from core.transport.messages.motion import TrajStatus

from .kinematics import Position3

logger = logging.getLogger(__name__)

TRAJ_DT = 1.0 / 50   # 50 Hz

# SpeedTcp / SpeedJ — caller (gamepad/외부) 가 마지막 명령 보낸 후 timeout 지나면
# target velocity 를 0 으로 잡고 jerk-limited 감속. caller 가 끊기거나 0 vector
# 명시한 경우 둘 다 안전 정지 (deadman 의 2차선).
VELOCITY_INPUT_TIMEOUT = 0.1  # 100ms

# Cartesian / joint Ruckig limit 은 더 이상 하드코드 X — robot/<type>/motion.yaml SSOT.
# motion_node 가 load 해서 TrajectoryRunner ctor 에 주입. Cartesian 저속 chatter
# (J3 P=1500 stick-slip) 회피용 최소 0.10 m/s 는 motion.yaml 코멘트로 박힘.

_MOVEP_MIN_DIST = 1e-4   # 너무 가까운 waypoint 제거

# Cartesian IK 출력 EMA — null-space 미세 노이즈 댐핑 (5DOF position-only IK가
# 매 스텝 미세하게 다른 해를 뽑아 손목이 떨리는 문제). alpha 작을수록 부드러움 ↑
# / lag ↑. 종점은 settle 램프로 raw IK 해에 정확히 수렴시켜 정확도 보존.
_CART_EMA_ALPHA = 0.1
_CART_SETTLE_STEPS = 5
# 모터 PID가 last_raw에 물리적으로 수렴할 dwell. 이게 없으면 DONE 직후 다음 step이
# 실측 encoder(= 아직 수렴 중인 위치)에서 출발해서 모터 momentum과 충돌 → 떨림.
_CART_HOLD_STEPS = 25

# Cartesian-space velocity Ruckig (SpeedTcp primary smoothing layer) 의 rotation
# limits. linear 는 motion.yaml cartesian_limits SSOT.
_CART_ROT_MAX_VEL = 1.0
_CART_ROT_MAX_ACC = 2.5
_CART_ROT_MAX_JERK = 10.0



# ── 콜백 타입 ──────────────────────────────────────────────────
PublishCmdFn = Callable[[list[float]], None]
PublishStateFn = Callable[[TrajStatus, float], None]
# release: motor register profile 을 raw 0,0 (= no cap) 으로 풀어 Ruckig 가
# 직접 명령. restore: 각 모터의 motors.yaml `profile` (dps) 복원.
ReleaseProfileFn = Callable[[], bool]
RestoreProfileFn = Callable[[], bool]
# Cartesian path 추종 시 매 step IK (position-only). servo_tcp 의 5DOF/6DOF 분기는
# caller (motion_node) 가 wrap 해서 주입 — 여기서는 단순 IK 콜백.
SolveIkFn = Callable[[Position3, list[float]], list[float] | None]
# TCP twist → joint velocity 변환 (Jacobian pseudo-inverse). frame 변환 포함.
# 입력: linear (3,) + angular (3,) + 현재 joint angles + frame ('base' | 'tcp')
# 반환: joint velocity (dof,) rad/s. None = 변환 실패.
TcpTwistToJointVelFn = Callable[
    [list[float], list[float], list[float], str], list[float] | None
]

# ═══════════════════════════════════════════════════════════════
# Path 추상화 (Cartesian)
# ═══════════════════════════════════════════════════════════════


class CartesianPath(ABC):
    @property
    @abstractmethod
    def total_length(self) -> float:
        ...

    @abstractmethod
    def position_at(self, s: float) -> list[float]:
        ...

    @property
    def label(self) -> str:
        return self.__class__.__name__


class LinearPath(CartesianPath):
    def __init__(self, start: np.ndarray, end: np.ndarray) -> None:
        self._start = start
        self._end = end
        self._dist = float(np.linalg.norm(end - start))

    @property
    def total_length(self) -> float:
        return self._dist

    def position_at(self, s: float) -> list[float]:
        ratio = s / self._dist if self._dist > 0 else 0.0
        return (self._start + ratio * (self._end - self._start)).tolist()

    @property
    def label(self) -> str:
        return "MoveL"


class ArcPath(CartesianPath):
    def __init__(self, p1: np.ndarray, p2: np.ndarray, p3: np.ndarray) -> None:
        center, radius, u_vec, v_vec, theta_end, arc_len = \
            TrajectoryRunner.arc_from_3_points(p1, p2, p3)
        self._center = center
        self._radius = radius
        self._u_vec = u_vec
        self._v_vec = v_vec
        self._arc_len = arc_len
        self._sign = 1.0 if theta_end >= 0 else -1.0

    @property
    def total_length(self) -> float:
        return self._arc_len

    def position_at(self, s: float) -> list[float]:
        theta = s / self._radius * self._sign
        return (
            self._center
            + self._radius * (np.cos(theta) * self._u_vec +
                              np.sin(theta) * self._v_vec)
        ).tolist()

    @property
    def label(self) -> str:
        return "MoveC"


class SplinePath(CartesianPath):
    def __init__(self, waypoints: np.ndarray) -> None:
        pts = waypoints.copy()
        dists = np.linalg.norm(np.diff(pts, axis=0), axis=1)

        # 너무 가까운 점 제거
        mask = np.concatenate([[True], dists >= _MOVEP_MIN_DIST])
        pts = pts[mask]

        dists = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        cum_dists = np.concatenate([[0.0], np.cumsum(dists)])
        self._total = float(cum_dists[-1])
        self._cs = CubicSpline(cum_dists, pts, bc_type="natural")

    @property
    def total_length(self) -> float:
        return self._total

    def position_at(self, s: float) -> list[float]:
        return self._cs(s).tolist()

    @property
    def label(self) -> str:
        return "MoveP"


class TrajectoryRunner:
    def __init__(
        self,
        n_arm:                      int,
        joint_max_velocity:         list[float],
        joint_max_acceleration:     list[float],
        joint_max_jerk:             list[float],
        cartesian_max_velocity:     float,
        cartesian_max_acceleration: float,
        cartesian_max_jerk:         float,
        release_profile:            ReleaseProfileFn,
        restore_profile:            RestoreProfileFn,
        publish_cmd:                PublishCmdFn,
        publish_state:              PublishStateFn,
        solve_ik:                   SolveIkFn,
        get_joint_angles:           Callable[[], list[float] | None],
        tcp_twist_to_joint_vel:     TcpTwistToJointVelFn,
    ) -> None:
        if not (len(joint_max_velocity) == len(joint_max_acceleration) == len(joint_max_jerk) == n_arm):
            raise ValueError(
                f"TrajectoryRunner: joint limit 배열 길이가 n_arm={n_arm} 와 안 맞음. "
                f"vel={len(joint_max_velocity)} acc={len(joint_max_acceleration)} "
                f"jerk={len(joint_max_jerk)}"
            )
        self._n_arm = n_arm
        self._j_max_vel = list(joint_max_velocity)
        self._j_max_acc = list(joint_max_acceleration)
        self._j_max_jerk = list(joint_max_jerk)
        self._c_max_vel = cartesian_max_velocity
        self._c_max_acc = cartesian_max_acceleration
        self._c_max_jerk = cartesian_max_jerk
        self._release_profile = release_profile
        self._restore_profile = restore_profile
        self._publish_cmd = publish_cmd
        self._publish_state = publish_state
        self._solve_ik = solve_ik
        self._get_joint_angles = get_joint_angles
        self._tcp_twist_to_joint_vel = tcp_twist_to_joint_vel

        self._thread:  threading.Thread | None = None
        self._stop_ev: threading.Event = threading.Event()

        # Velocity streaming 자리 (SpeedJ / SpeedTcp 공통).
        # set_speed_joint() / set_speed_tcp() 가 update 시점 갱신 — streamer 가 추종.
        self._vel_lock = threading.Lock()
        self._vel_target_joint: list[float] = [0.0] * n_arm
        self._vel_last_set: float = 0.0
        # SpeedTcp 가 active 일 때만 채워짐. None = SpeedJ 모드 or idle.
        # streamer 가 매 step 마다 현재 angles 기준으로 Jacobian 풀어 joint_vel 환산.
        self._vel_tcp_twist: tuple[list[float], list[float], str] | None = None

    # ─── Public API ─────────────────────────────────────────────

    @property
    def is_running(self) -> bool:
        return bool(self._thread and self._thread.is_alive())

    def stop(self) -> None:
        """외부 호출 — thread 정지 + velocity state reset (다음 set_speed_* 깨끗이 시작)."""
        self._stop_thread()
        self._reset_velocity_state()

    def _stop_thread(self) -> None:
        """thread 만 정지. velocity state 는 보존 — 내부에서 trajectory↔streamer 교체 시 사용."""
        if self._thread is not None and self._thread.is_alive():
            self._stop_ev.set()
            self._thread.join(timeout=2.0)
        self._thread = None
        self._stop_ev.clear()

    def _reset_velocity_state(self) -> None:
        with self._vel_lock:
            self._vel_target_joint = [0.0] * self._n_arm
            self._vel_tcp_twist = None
            self._vel_last_set = 0.0

    def run_cartesian(self, path: CartesianPath, start_angles: list[float]) -> None:
        self._launch(
            target=self._cartesian_loop,
            args=(path, list(start_angles)),
            name=f"{path.label.lower()}-traj",
        )

    def run_joint(self, start_angles: list[float], target_angles: list[float]) -> None:
        self._launch(
            target=self._joint_loop,
            args=(list(start_angles), list(target_angles)),
            name="movej-traj",
        )

    # ─── Speed (velocity) primitives ────────────────────────────

    def set_speed_joint(self, velocities: list[float]) -> None:
        """SpeedJ — joint velocity 추종 갱신. streamer 없으면 자동 launch.

        caller 가 빠른 rate (~50Hz) 로 호출 → streamer 가 매 step 추종.
        VELOCITY_INPUT_TIMEOUT 동안 갱신 X → 0 velocity 로 jerk-limited 감속.
        """
        if len(velocities) != self._n_arm:
            raise ValueError(
                f"SpeedJ velocities 길이={len(velocities)} != n_arm={self._n_arm}"
            )
        now = time.time()
        with self._vel_lock:
            self._vel_target_joint = list(velocities)
            self._vel_tcp_twist = None
            self._vel_last_set = now
        self._ensure_velocity_streamer()

    def set_speed_tcp(
        self,
        linear: list[float],
        angular: list[float],
        frame: str,
    ) -> None:
        """SpeedTcp — TCP twist 추종 갱신. streamer 가 매 step Jacobian 풀어 joint vel 환산.

        `frame` ∈ {"base", "tcp"}. tcp → 현재 EE-local 좌표계.
        5DOF robot 자리는 caller (motion_node) 가 angular 무시 후 호출 권장 —
        여기서는 받은 twist 그대로 변환 시도 (tcp_twist_to_joint_vel 가 fallback).
        """
        now = time.time()
        with self._vel_lock:
            self._vel_tcp_twist = (list(linear), list(angular), frame)
            self._vel_target_joint = [0.0] * self._n_arm
            self._vel_last_set = now
        self._ensure_velocity_streamer()

    def _ensure_velocity_streamer(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            # 이미 streamer 또는 trajectory thread 가 도는 중.
            # trajectory 가 돌고 있으면 stop_thread + 새 streamer 시작 (velocity 가 가로챔).
            # set_speed_* 가 방금 갱신한 _vel_last_set / _vel_target_joint 는 보존.
            if self._thread.name != "velocity-streamer":
                self._stop_thread()
            else:
                return
        self._launch(
            target=self._velocity_loop,
            args=(),
            name="velocity-streamer",
        )

    # ─── Internal ────────────────────────────────────

    def _launch(self, target, args: tuple, name: str) -> None:
        # thread 만 교체. velocity state 는 보존 — set_speed_* 가 갱신한 직후
        # streamer thread 시작 시 stale reset 으로 timeout 즉시 발동되는 자리 차단.
        self._stop_thread()
        self._thread = threading.Thread(
            target=target, args=args, name=name, daemon=True)
        self._thread.start()

    def _cartesian_loop(self, path: CartesianPath, start_angles: list[float]) -> None:
        label = path.label
        ok = self._release_profile()
        if not ok:
            logger.warning(f"{label}: profile 비활성화 실패 — 계속 진행")

        otg = Ruckig(1, TRAJ_DT)
        inp = InputParameter(1)
        out = OutputParameter(1)
        inp.current_position = [0.0]
        inp.current_velocity = [0.0]
        inp.current_acceleration = [0.0]
        inp.target_position = [path.total_length]
        inp.target_velocity = [0.0]
        inp.target_acceleration = [0.0]
        inp.max_velocity = [self._c_max_vel]
        inp.max_acceleration = [self._c_max_acc]
        inp.max_jerk = [self._c_max_jerk]

        q_filt = list(start_angles)
        last_raw: list[float] = list(start_angles)
        alpha = _CART_EMA_ALPHA

        first_result = otg.update(inp, out)
        est_duration = out.trajectory.duration
        t_start = time.time()

        logger.info(
            f"{label}: dist={path.total_length*100:.1f}cm | 예상 {est_duration:.1f}s")

        def _ik_step(s: float) -> bool:
            nonlocal q_filt, last_raw
            wp_list = path.position_at(s)
            wp: Position3 = (wp_list[0], wp_list[1], wp_list[2])
            raw = self._solve_ik(wp, q_filt)
            if raw is None:
                logger.warning(f"{label} IK 실패 | s={s*100:.1f}cm")
                return False
            last_raw = raw
            q_filt = [(1.0 - alpha) * qf + alpha * qr
                      for qf, qr in zip(q_filt, raw)]
            self._publish_cmd(q_filt)
            return True

        def _settle_to_raw() -> None:
            # 1) EMA lag 제거: q_filt → last_raw로 선형 램프.
            # 2) last_raw에서 dwell — 모터 PID 실제 수렴 보장 (다음 step이
            #    실측 encoder 읽을 때 momentum 충돌 방지).
            nonlocal q_filt
            steps = _CART_SETTLE_STEPS
            for k in range(1, steps + 1):
                if self._stop_ev.is_set():
                    return
                ratio = k / steps
                q_blend = [
                    (1.0 - ratio) * qf + ratio * qr
                    for qf, qr in zip(q_filt, last_raw)
                ]
                self._publish_cmd(q_blend)
                time.sleep(TRAJ_DT)
            q_filt = list(last_raw)
            for _ in range(_CART_HOLD_STEPS):
                if self._stop_ev.is_set():
                    return
                self._publish_cmd(list(last_raw))
                time.sleep(TRAJ_DT)

        try:
            if not _ik_step(out.new_position[0]):
                self._publish_state(TrajStatus.FAILED, 0.0)
                return

            elapsed = time.time() - t_start
            progress = min(elapsed / est_duration,
                           1.0) if est_duration > 0 else 1.0
            self._publish_state(TrajStatus.RUNNING, progress)

            if first_result == Result.Finished:
                _settle_to_raw()
                self._publish_state(TrajStatus.DONE, 1.0)
                return

            inp.current_position = list(out.new_position)
            inp.current_velocity = list(out.new_velocity)
            inp.current_acceleration = list(out.new_acceleration)

            while True:
                if self._stop_ev.is_set():
                    self._publish_state(TrajStatus.STOPPED, progress)
                    return

                next_t = t_start + (time.time() - t_start) + TRAJ_DT
                result = otg.update(inp, out)

                if not _ik_step(out.new_position[0]):
                    self._publish_state(TrajStatus.FAILED, progress)
                    return

                elapsed = time.time() - t_start
                progress = min(elapsed / est_duration,
                               1.0) if est_duration > 0 else 1.0
                self._publish_state(TrajStatus.RUNNING, progress)

                if result == Result.Finished:
                    _settle_to_raw()
                    self._publish_state(TrajStatus.DONE, 1.0)
                    logger.info(f"{label} 완료 ({elapsed*1000:.0f}ms)")
                    return

                if result == Result.Error:
                    logger.error(f"{label} Ruckig 오류")
                    self._publish_state(TrajStatus.FAILED, progress)
                    return

                inp.current_position = list(out.new_position)
                inp.current_velocity = list(out.new_velocity)
                inp.current_acceleration = list(out.new_acceleration)

                sleep_time = next_t - time.time()
                if sleep_time > 0:
                    time.sleep(sleep_time)

        finally:
            ok = self._restore_profile()
            if not ok:
                logger.warning(f"{label}: profile 복원 실패")

    def _joint_loop(self, start_angles: list[float], target_angles: list[float]) -> None:
        ok = self._release_profile()
        if not ok:
            logger.warning("MoveJ: profile 비활성화 실패 — 계속 진행")

        n = self._n_arm
        otg = Ruckig(n, TRAJ_DT)
        inp = InputParameter(n)
        out = OutputParameter(n)
        inp.current_position = start_angles
        inp.current_velocity = [0.0] * n
        inp.current_acceleration = [0.0] * n
        inp.target_position = target_angles
        inp.target_velocity = [0.0] * n
        inp.target_acceleration = [0.0] * n
        inp.max_velocity = list(self._j_max_vel)
        inp.max_acceleration = list(self._j_max_acc)
        inp.max_jerk = list(self._j_max_jerk)

        first_result = otg.update(inp, out)
        est_duration = out.trajectory.duration
        t_start = time.time()

        try:
            self._publish_cmd(list(out.new_position))
            self._publish_state(TrajStatus.RUNNING, 0.0)

            if first_result == Result.Finished:
                self._publish_state(TrajStatus.DONE, 1.0)
                return

            inp.current_position = list(out.new_position)
            inp.current_velocity = list(out.new_velocity)
            inp.current_acceleration = list(out.new_acceleration)

            while True:
                if self._stop_ev.is_set():
                    self._publish_state(TrajStatus.STOPPED, 0.0)
                    return

                next_t = t_start + (time.time() - t_start) + TRAJ_DT
                result = otg.update(inp, out)
                elapsed = time.time() - t_start
                progress = min(elapsed / est_duration,
                               1.0) if est_duration > 0 else 1.0

                self._publish_cmd(list(out.new_position))
                self._publish_state(TrajStatus.RUNNING, progress)

                if result == Result.Finished:
                    self._publish_state(TrajStatus.DONE, 1.0)
                    logger.info(f"MoveJ 완료 ({elapsed*1000:.0f}ms)")
                    return

                if result == Result.Error:
                    logger.error("MoveJ Ruckig 오류")
                    self._publish_state(TrajStatus.FAILED, progress)
                    return

                inp.current_position = list(out.new_position)
                inp.current_velocity = list(out.new_velocity)
                inp.current_acceleration = list(out.new_acceleration)

                sleep_time = next_t - time.time()
                if sleep_time > 0:
                    time.sleep(sleep_time)

        finally:
            ok = self._restore_profile()
            if not ok:
                logger.warning("MoveJ: profile 복원 실패")

    def _velocity_loop(self) -> None:
        """SpeedJ / SpeedTcp 공통 streamer — 50Hz, Ruckig velocity mode.

        매 step:
          1. caller 가 set_speed_joint / set_speed_tcp 로 갱신한 target velocity 읽기.
             VELOCITY_INPUT_TIMEOUT 지나도 갱신 X → target=0 (deadman 2차선).
          2. SpeedTcp 면 현재 joint angles + tcp_twist → joint velocity 변환 (Jacobian).
          3. Ruckig 가 jerk-limited 으로 target_velocity 추종 + new position 산출.
          4. publish_cmd(new_position).
          5. 정지 상태 (target=0 + 실 vel≈0) 가 충분히 지속되면 자연 종료.
        """
        ok = self._release_profile()
        if not ok:
            logger.warning("Speed*: profile 비활성화 실패 — 계속 진행")

        n = self._n_arm
        otg = Ruckig(n, TRAJ_DT)
        inp = InputParameter(n)
        out = OutputParameter(n)
        inp.control_interface = ControlInterface.Velocity

        # ─── Cartesian-space Ruckig (SpeedTcp primary smoothing) ────────
        # log 분석 (2026-06-17) — joint Ruckig 만으론 *각 joint independent
        # jerk-limited ramp* 자리에서 *target_velocity magnitude 다름* (J2=0.30
        # vs J3=0.16) → 같은 max_jerk 에서 *작은 target 먼저 cruise 도달*, 큰
        # target 가속 중. transient out_v ratio ≠ target ratio → cartesian
        # direction 깨짐. cartesian smoothing 이 *twist 자체를 ramp* → 매 cycle
        # Jacobian 환산 시 *joint target_velocity 자체가 *cartesian space 비례
        # 유지*. joint Ruckig 는 *target 변화에 jerk-limited 대응 + saturation 보호*.
        cart_otg = Ruckig(6, TRAJ_DT)
        cart_inp = InputParameter(6)
        cart_out = OutputParameter(6)
        cart_inp.control_interface = ControlInterface.Velocity
        cart_inp.current_position = [0.0] * 6
        cart_inp.current_velocity = [0.0] * 6
        cart_inp.current_acceleration = [0.0] * 6
        cart_inp.target_velocity = [0.0] * 6
        cart_inp.target_acceleration = [0.0] * 6
        cart_inp.max_velocity = (
            [self._c_max_vel] * 3 + [_CART_ROT_MAX_VEL] * 3
        )
        cart_inp.max_acceleration = (
            [self._c_max_acc] * 3 + [_CART_ROT_MAX_ACC] * 3
        )
        cart_inp.max_jerk = (
            [self._c_max_jerk] * 3 + [_CART_ROT_MAX_JERK] * 3
        )

        start = self._get_joint_angles()
        if start is None:
            logger.warning("Speed*: 시작 joint state 없음 — streamer 종료")
            self._restore_profile()
            return
        inp.current_position = list(start)
        inp.current_velocity = [0.0] * n
        inp.current_acceleration = [0.0] * n
        inp.target_velocity = [0.0] * n
        inp.target_acceleration = [0.0] * n
        inp.max_velocity = list(self._j_max_vel)
        inp.max_acceleration = list(self._j_max_acc)
        inp.max_jerk = list(self._j_max_jerk)

        IDLE_GRACE_S = 0.5  # 마지막 input 후 추가 idle 허용 — 짧은 끊김 시 즉시 종료 방지.

        # 진단: jog 시작 transient (target 0→nonzero) 첫 30 cycle (=600ms) log.
        # backend output 이 깨끗한지 (target_velocity 일관 + Ruckig out_velocity
        # jerk-limited ramp) 검증. 깨끗하면 잔존 drift = 모터 PID/gravity 자리.
        transient_log_remaining = 0
        prev_target_zero = True

        try:
            self._publish_state(TrajStatus.RUNNING, 0.0)
            while True:
                if self._stop_ev.is_set():
                    self._publish_state(TrajStatus.STOPPED, 0.0)
                    return

                step_start = time.time()

                with self._vel_lock:
                    target_joint = list(self._vel_target_joint)
                    tcp_twist = self._vel_tcp_twist
                    last_set = self._vel_last_set

                age = step_start - last_set
                timed_out = age > VELOCITY_INPUT_TIMEOUT
                if timed_out:
                    target = [0.0] * n
                    # SpeedTcp 자리 timeout 도 cartesian Ruckig ramp down. caller
                    # 가 명령 멈춤 → twist target = 0 → 자연 deadman ramp.
                    cart_inp.target_velocity = [0.0] * 6
                    cart_otg.update(cart_inp, cart_out)
                    cart_inp.current_velocity = list(cart_out.new_velocity)
                    cart_inp.current_acceleration = list(cart_out.new_acceleration)
                elif tcp_twist is not None:
                    linear, angular, frame = tcp_twist
                    # ── Cartesian smoothing (primary) ─────────────────
                    # twist 자체를 jerk-limited 으로 ramp → 매 cycle smoothed
                    # twist 를 Jacobian 환산. joint target_velocity 가 cartesian
                    # 비례 유지 → transient direction 일관.
                    cart_inp.target_velocity = list(linear) + list(angular)
                    cart_otg.update(cart_inp, cart_out)
                    smoothed = list(cart_out.new_velocity)
                    cart_inp.current_velocity = smoothed
                    cart_inp.current_acceleration = list(cart_out.new_acceleration)

                    # ── Closed-loop Jacobian @ encoder reading ────────
                    encoder = self._get_joint_angles()
                    angles_for_jacobian = (
                        list(encoder) if encoder else list(inp.current_position)
                    )
                    joint_vel = self._tcp_twist_to_joint_vel(
                        smoothed[:3], smoothed[3:], angles_for_jacobian, frame
                    )
                    if joint_vel is None:
                        # Jacobian 못 풂 (singularity 등) → 안전 정지.
                        target = [0.0] * n
                    else:
                        target = list(joint_vel)
                else:
                    # SpeedJ 자리 — cartesian 무관. 다음 SpeedTcp 진입 시 clean
                    # start 위해 cartesian Ruckig 0 으로 ramp.
                    cart_inp.target_velocity = [0.0] * 6
                    cart_otg.update(cart_inp, cart_out)
                    cart_inp.current_velocity = list(cart_out.new_velocity)
                    cart_inp.current_acceleration = list(cart_out.new_acceleration)
                    target = target_joint

                inp.target_velocity = target

                # 진단: target 0→nonzero 전환 시 transient log 시작.
                target_zero_now = all(abs(t) < 1e-6 for t in target)
                if prev_target_zero and not target_zero_now:
                    transient_log_remaining = 30
                prev_target_zero = target_zero_now

                result = otg.update(inp, out)
                if result == Result.Error:
                    logger.error("Speed* Ruckig 오류")
                    self._publish_state(TrajStatus.FAILED, 0.0)
                    return

                if transient_log_remaining > 0:
                    cyc = 30 - transient_log_remaining
                    logger.info(
                        "[transient cyc=%02d] target=%s out_v=%s",
                        cyc,
                        [round(t, 4) for t in target],
                        [round(v, 4) for v in out.new_velocity],
                    )
                    transient_log_remaining -= 1

                self._publish_cmd(list(out.new_position))

                inp.current_position = list(out.new_position)
                inp.current_velocity = list(out.new_velocity)
                inp.current_acceleration = list(out.new_acceleration)

                # 자연 종료: timeout 이고 + 실 velocity 가 0 에 수렴 + idle grace 경과.
                vel_zero = max(abs(v) for v in out.new_velocity) < 1e-4
                if timed_out and vel_zero and age > VELOCITY_INPUT_TIMEOUT + IDLE_GRACE_S:
                    self._publish_state(TrajStatus.DONE, 1.0)
                    return

                sleep_t = TRAJ_DT - (time.time() - step_start)
                if sleep_t > 0:
                    time.sleep(sleep_t)
        finally:
            ok = self._restore_profile()
            if not ok:
                logger.warning("Speed*: profile 복원 실패")

    # ─── 정적 유틸 ────────────────────────────────────────────

    @staticmethod
    def arc_from_3_points(
        p1: np.ndarray, p2: np.ndarray, p3: np.ndarray,
    ) -> tuple[np.ndarray, float, np.ndarray, np.ndarray, float, float]:
        a, b = p2 - p1, p3 - p1
        axb = np.cross(a, b)
        axb_sq = float(np.dot(axb, axb))

        if axb_sq < 1e-10:
            raise ValueError("3점이 일직선입니다 — MoveL을 사용하세요")

        center = p1 + (
            np.dot(b, b) * np.cross(axb, a) +
            np.dot(a, a) * np.cross(b, axb)
        ) / (2.0 * axb_sq)

        radius = float(np.linalg.norm(p1 - center))
        if radius < 1e-4:
            raise ValueError("반지름이 너무 작습니다")

        u_vec = (p1 - center) / radius
        v_vec = np.cross(axb / np.sqrt(axb_sq), u_vec)

        def _angle(p: np.ndarray) -> float:
            rel = p - center
            return float(np.arctan2(np.dot(rel, v_vec), np.dot(rel, u_vec)))

        theta_via = _angle(p2)
        theta_end = _angle(p3)

        if theta_via >= 0:
            if theta_end <= 0:
                theta_end += 2.0 * np.pi
            if theta_via > theta_end:
                theta_end += 2.0 * np.pi
        else:
            if theta_end >= 0:
                theta_end -= 2.0 * np.pi
            if theta_via < theta_end:
                theta_end -= 2.0 * np.pi

        return center, radius, u_vec, v_vec, theta_end, abs(theta_end) * radius
