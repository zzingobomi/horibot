import time
import logging
import threading
from abc import ABC, abstractmethod
from typing import Callable

import numpy as np
from ruckig import Ruckig, InputParameter, OutputParameter, Result
from scipy.interpolate import CubicSpline

from core.transport.messages.motion import TrajStatus
from .kinematics import Position3

logger = logging.getLogger(__name__)

TRAJ_DT = 1.0 / 50   # 50 Hz

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

# ── 콜백 타입 ──────────────────────────────────────────────────
PublishCmdFn = Callable[[list[float]], None]
PublishStateFn = Callable[[TrajStatus, float], None]
# release: motor register profile 을 raw 0,0 (= no cap) 으로 풀어 Ruckig 가
# 직접 명령. restore: 각 모터의 motors.yaml `profile` (dps) 복원.
ReleaseProfileFn = Callable[[], bool]
RestoreProfileFn = Callable[[], bool]
MoveTcpFn = Callable[[Position3, list[float]], list[float] | None]

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
        move_tcp:                   MoveTcpFn,
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
        self._move_tcp = move_tcp

        self._thread:  threading.Thread | None = None
        self._stop_ev: threading.Event = threading.Event()

    # ─── Public API ─────────────────────────────────────────────

    @property
    def is_running(self) -> bool:
        return bool(self._thread and self._thread.is_alive())

    def stop(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            self._stop_ev.set()
            self._thread.join(timeout=2.0)
        self._thread = None
        self._stop_ev.clear()

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

    # ─── Internal ────────────────────────────────────

    def _launch(self, target, args: tuple, name: str) -> None:
        self.stop()
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
            raw = self._move_tcp(wp, q_filt)
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
