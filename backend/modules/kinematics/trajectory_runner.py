import time
import logging
import threading
from abc import ABC, abstractmethod
from typing import Callable

import numpy as np
from ruckig import Ruckig, InputParameter, OutputParameter, Result
from scipy.interpolate import CubicSpline

from core.types import TrajStatus

logger = logging.getLogger(__name__)

TRAJ_DT = 1.0 / 50   # 50 Hz

# ── Cartesian 경로 제약 ────────────────────────────────────────
_C_MAX_VEL = 0.10    # m/s
_C_MAX_ACC = 0.25    # m/s²
_C_MAX_JERK = 1.00    # m/s³


# ── Joint 제약 (id 순서: 1,2,3,4,5) ────────────────────────────
# XL430(1~3): max ~41 rpm ≈ 4.3 rad/s
# XL330(4~5): max ~61 rpm ≈ 6.4 rad/s
# 안정성 / 정밀도 우선 세팅 (soft motion profile)
# - velocity ~35% 수준 제한
# - acceleration / jerk 추가 감속으로 부드러운 움직임 보장
_J_MAX_VEL = [1.5, 1.5, 1.5, 2.5, 2.5]
_J_MAX_ACC = [3.0, 3.0, 3.0, 5.0, 5.0]
_J_MAX_JERK = [10.0, 10.0, 10.0, 20.0, 20.0]

_MOVEP_MIN_DIST = 1e-4   # 너무 가까운 waypoint 제거

# ── 콜백 타입 ──────────────────────────────────────────────────
PublishCmdFn = Callable[[list[float]], None]
PublishStateFn = Callable[[str, float], None]
SetProfileFn = Callable[[int, int], bool]
MoveTcpFn = Callable[[list[float], list[float]], list[float] | None]

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
        n_arm:               int,
        set_profile:         SetProfileFn,
        publish_cmd:         PublishCmdFn,
        publish_state:       PublishStateFn,
        move_tcp:            MoveTcpFn,
        default_profile_vel: int = 150,
        default_profile_acc: int = 40,
    ) -> None:
        self._n_arm = n_arm
        self._set_profile = set_profile
        self._publish_cmd = publish_cmd
        self._publish_state = publish_state
        self._move_tcp = move_tcp
        self._default_vel = default_profile_vel
        self._default_acc = default_profile_acc

        self._thread:  threading.Thread | None = None
        self._stop_ev: threading.Event = threading.Event()

    # ─── Public API ─────────────────────────────────────────────

    @property
    def is_running(self) -> bool:
        return bool(self._thread and self._thread.is_alive())

    def stop(self) -> None:
        if self.is_running:
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
        ok = self._set_profile(0, 0)
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
        inp.max_velocity = [_C_MAX_VEL]
        inp.max_acceleration = [_C_MAX_ACC]
        inp.max_jerk = [_C_MAX_JERK]

        current_angles = start_angles
        first_result = otg.update(inp, out)
        est_duration = out.trajectory.duration
        t_start = time.time()

        logger.info(
            f"{label}: dist={path.total_length*100:.1f}cm | 예상 {est_duration:.1f}s")

        def _ik_step(s: float) -> bool:
            nonlocal current_angles
            wp = path.position_at(s)
            result = self._move_tcp(wp, current_angles)
            if result is None:
                logger.warning(f"{label} IK 실패 | s={s*100:.1f}cm")
                return False
            current_angles = result
            self._publish_cmd(result)
            return True

        try:
            if not _ik_step(out.new_position[0]):
                self._publish_state(TrajStatus.FAILED, 0.0)
                return

            elapsed = time.time() - t_start
            progress = min(elapsed / est_duration,
                           1.0) if est_duration > 0 else 1.0
            self._publish_state(TrajStatus.RUNNING, progress)

            if first_result == Result.Finished:
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
            ok = self._set_profile(self._default_vel, self._default_acc)
            if not ok:
                logger.warning(f"{label}: profile 복원 실패")

    def _joint_loop(self, start_angles: list[float], target_angles: list[float]) -> None:
        ok = self._set_profile(0, 0)
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
        inp.max_velocity = list(_J_MAX_VEL)
        inp.max_acceleration = list(_J_MAX_ACC)
        inp.max_jerk = list(_J_MAX_JERK)

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
            ok = self._set_profile(self._default_vel, self._default_acc)
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
