"""TrajectoryRunner — Ruckig jerk-limited 보간 (MoveJ/L/C/P).

옛 backend/modules/kinematics/trajectory_runner.py 의 faithful port. callback DI
구조 그대로 (publish_cmd / publish_state / solve_ik / get_joint_angles /
release_profile / restore_profile) — Motion 모듈이 콜백 주입.

D2b = run_joint (MoveJ). cartesian path (run_cartesian) 는 D2c 에서 Motion 이
서비스로 노출. 검증: mock motor 로 회사, 실 모터는 집.
"""

import logging
import threading
import time
from abc import ABC, abstractmethod
from typing import Callable

import numpy as np
from ruckig import (
    InputParameter,
    OutputParameter,
    Result,
    Ruckig,
    Synchronization,
)
from scipy.interpolate import CubicSpline

from .contract import TrajStatus
from .kinematics import Position3

logger = logging.getLogger(__name__)

TRAJ_DT = 1.0 / 50  # 50 Hz
_MOVEP_MIN_DIST = 1e-4
_CART_EMA_ALPHA = 0.1
_CART_SETTLE_STEPS = 5
_CART_HOLD_STEPS = 25

PublishCmdFn = Callable[[list[float]], None]
PublishStateFn = Callable[[TrajStatus, float], None]
ReleaseProfileFn = Callable[[], bool]
RestoreProfileFn = Callable[[], bool]
SolveIkFn = Callable[[Position3, list[float]], list[float] | None]


class CartesianPath(ABC):
    @property
    @abstractmethod
    def total_length(self) -> float: ...

    @abstractmethod
    def position_at(self, s: float) -> list[float]: ...

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
        center, radius, u_vec, v_vec, theta_end, arc_len = (
            TrajectoryRunner.arc_from_3_points(p1, p2, p3)
        )
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
            + self._radius
            * (np.cos(theta) * self._u_vec + np.sin(theta) * self._v_vec)
        ).tolist()

    @property
    def label(self) -> str:
        return "MoveC"


class SplinePath(CartesianPath):
    def __init__(self, waypoints: np.ndarray) -> None:
        pts = waypoints.copy()
        dists = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        mask = np.concatenate([[True], dists >= _MOVEP_MIN_DIST])
        pts = pts[mask]
        dists = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        cum = np.concatenate([[0.0], np.cumsum(dists)])
        self._total = float(cum[-1])
        self._cs = CubicSpline(cum, pts, bc_type="natural")

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
        n_arm: int,
        joint_max_velocity: list[float],
        joint_max_acceleration: list[float],
        joint_max_jerk: list[float],
        cartesian_max_velocity: float,
        cartesian_max_acceleration: float,
        cartesian_max_jerk: float,
        release_profile: ReleaseProfileFn,
        restore_profile: RestoreProfileFn,
        publish_cmd: PublishCmdFn,
        publish_state: PublishStateFn,
        solve_ik: SolveIkFn,
        get_joint_angles: Callable[[], list[float] | None],
    ) -> None:
        if not (
            len(joint_max_velocity)
            == len(joint_max_acceleration)
            == len(joint_max_jerk)
            == n_arm
        ):
            raise ValueError(
                f"TrajectoryRunner: joint limit 배열 길이가 n_arm={n_arm} 와 안 맞음"
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
        self._thread: threading.Thread | None = None
        self._stop_ev = threading.Event()

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
        self._launch(self._cartesian_loop, (path, list(start_angles)),
                     f"{path.label.lower()}-traj")

    def run_joint(self, start_angles: list[float], target_angles: list[float]) -> None:
        self._launch(self._joint_loop, (list(start_angles), list(target_angles)),
                     "movej-traj")

    def _launch(self, target, args: tuple, name: str) -> None:
        if self._thread is not None and self._thread.is_alive():
            self._stop_ev.set()
            self._thread.join(timeout=2.0)
            self._stop_ev.clear()
        self._thread = threading.Thread(
            target=target, args=args, name=name, daemon=True
        )
        self._thread.start()

    def _cartesian_loop(self, path: CartesianPath, start_angles: list[float]) -> None:
        label = path.label
        if not self._release_profile():
            logger.warning("%s: profile 비활성화 실패 — 계속", label)

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

        def _ik_step(s: float) -> bool:
            nonlocal q_filt, last_raw
            wp_list = path.position_at(s)
            wp: Position3 = (wp_list[0], wp_list[1], wp_list[2])
            raw = self._solve_ik(wp, q_filt)
            if raw is None:
                logger.warning("%s IK 실패 | s=%.1fcm", label, s * 100)
                return False
            last_raw = raw
            q_filt = [(1.0 - alpha) * qf + alpha * qr
                      for qf, qr in zip(q_filt, raw)]
            self._publish_cmd(q_filt)
            return True

        def _settle_to_raw() -> None:
            nonlocal q_filt
            for k in range(1, _CART_SETTLE_STEPS + 1):
                if self._stop_ev.is_set():
                    return
                ratio = k / _CART_SETTLE_STEPS
                self._publish_cmd([
                    (1.0 - ratio) * qf + ratio * qr
                    for qf, qr in zip(q_filt, last_raw)
                ])
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
            progress = min(elapsed / est_duration, 1.0) if est_duration > 0 else 1.0
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
                progress = (
                    min(elapsed / est_duration, 1.0) if est_duration > 0 else 1.0
                )
                self._publish_state(TrajStatus.RUNNING, progress)
                if result == Result.Finished:
                    _settle_to_raw()
                    self._publish_state(TrajStatus.DONE, 1.0)
                    return
                if result == Result.Error:
                    logger.error("%s Ruckig 오류", label)
                    self._publish_state(TrajStatus.FAILED, progress)
                    return
                inp.current_position = list(out.new_position)
                inp.current_velocity = list(out.new_velocity)
                inp.current_acceleration = list(out.new_acceleration)
                sleep_time = next_t - time.time()
                if sleep_time > 0:
                    time.sleep(sleep_time)
        finally:
            if not self._restore_profile():
                logger.warning("%s: profile 복원 실패", label)

    def _joint_loop(
        self, start_angles: list[float], target_angles: list[float]
    ) -> None:
        if not self._release_profile():
            logger.warning("MoveJ: profile 비활성화 실패 — 계속")

        n = self._n_arm
        otg = Ruckig(n, TRAJ_DT)
        inp = InputParameter(n)
        out = OutputParameter(n)
        inp.synchronization = Synchronization.Phase
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
                progress = (
                    min(elapsed / est_duration, 1.0) if est_duration > 0 else 1.0
                )
                self._publish_cmd(list(out.new_position))
                self._publish_state(TrajStatus.RUNNING, progress)
                if result == Result.Finished:
                    self._publish_state(TrajStatus.DONE, 1.0)
                    logger.info("MoveJ 완료 (%.0fms)", elapsed * 1000)
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
            if not self._restore_profile():
                logger.warning("MoveJ: profile 복원 실패")

    @staticmethod
    def arc_from_3_points(
        p1: np.ndarray, p2: np.ndarray, p3: np.ndarray
    ) -> tuple[np.ndarray, float, np.ndarray, np.ndarray, float, float]:
        a, b = p2 - p1, p3 - p1
        axb = np.cross(a, b)
        axb_sq = float(np.dot(axb, axb))
        if axb_sq < 1e-10:
            raise ValueError("3점이 일직선 — MoveL 사용")
        center = p1 + (
            np.dot(b, b) * np.cross(axb, a) + np.dot(a, a) * np.cross(b, axb)
        ) / (2.0 * axb_sq)
        radius = float(np.linalg.norm(p1 - center))
        if radius < 1e-4:
            raise ValueError("반지름 너무 작음")
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
