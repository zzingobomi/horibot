"""TrajectoryRunner — Ruckig jerk-limited 보간 (MoveJ/L/C/P).

옛 backend/modules/kinematics/trajectory_runner.py 의 faithful port. callback DI
구조 그대로 (publish_cmd / publish_state / solve_ik / get_joint_angles /
release_profile / restore_profile) — Motion 모듈이 콜백 주입.

D2b = run_joint (MoveJ). cartesian path (run_cartesian) 는 D2c 에서 Motion 이
서비스로 노출. 검증: mock motor 로 회사, 실 모터는 집.
"""

import logging
import math
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
from scipy.spatial.transform import Rotation, Slerp

from .contract import TrajStatus
from .kinematics import Position3, Quaternion

logger = logging.getLogger(__name__)

TRAJ_DT = 1.0 / 50  # 50 Hz
_MOVEP_MIN_DIST = 1e-4
_CART_HOLD_STEPS = 25  # 종료 후 목표 자세 hold 틱 수 (servo 정착 여유)

PublishCmdFn = Callable[[list[float]], None]
PublishStateFn = Callable[[TrajStatus, float], None]
ReleaseProfileFn = Callable[[], bool]
RestoreProfileFn = Callable[[], bool]
# (position, orientation | None, seed) — None = position-only IK
SolveIkFn = Callable[[Position3, Quaternion | None, list[float]], list[float] | None]


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

    def run_cartesian(
        self,
        path: CartesianPath,
        start_angles: list[float],
        target_quaternion: Quaternion | None = None,
        start_quaternion: Quaternion | None = None,
        *,
        speed_scale: float = 1.0,
    ) -> None:
        """자세 처리 (UR/ABB/MoveIt 식 MoveL base primitive):
          - target_quaternion=None → position-only IK (자세는 seed 에 딸려감).
          - 지정 + start_quaternion 지정 → 시작 자세 → 목표 자세를 경로 s 에 동기해
            **slerp 보간**. 자세 고정은 start==target 인 자연스러운 특수 케이스.
          - 지정 + start_quaternion=None → 그 자세로 상수(fallback).

        speed_scale: cartesian max vel/acc 배율 (0<s≤1) — 접촉 인접 구간(파지
        최종 접근/후퇴) 감속용. jerk 는 유지 (프로파일 모양 보존).
        """
        self._launch(
            self._cartesian_loop,
            (path, list(start_angles), target_quaternion, start_quaternion,
             speed_scale),
            f"{path.label.lower()}-traj",
        )

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

    def _cartesian_loop(
        self,
        path: CartesianPath,
        start_angles: list[float],
        target_quat: Quaternion | None = None,
        start_quat: Quaternion | None = None,
        speed_scale: float = 1.0,
    ) -> None:
        label = path.label
        if not self._release_profile():
            logger.warning("%s: profile 비활성화 실패 — 계속", label)

        # ── 자세 보간(slerp) 준비 — target 지정 + start 지정이면 경로 s 에 동기해
        # 현재→목표 자세 보간. 하나라도 없으면 상수(target) 또는 position-only(None).
        _slerp = None
        _ori_angle = 0.0
        if target_quat is not None and start_quat is not None:
            _slerp = Slerp([0.0, 1.0], Rotation.from_quat([start_quat, target_quat]))
            _ori_angle = float(
                (
                    Rotation.from_quat(start_quat).inv()
                    * Rotation.from_quat(target_quat)
                ).magnitude()
            )

        def _orientation_at(frac: float) -> Quaternion | None:
            if target_quat is None:
                return None
            if _slerp is None:
                return target_quat
            q = _slerp(min(max(frac, 0.0), 1.0)).as_quat()
            return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))

        otg = Ruckig(1, TRAJ_DT)
        inp = InputParameter(1)
        out = OutputParameter(1)
        inp.current_position = [0.0]
        inp.current_velocity = [0.0]
        inp.current_acceleration = [0.0]
        inp.target_position = [path.total_length]
        inp.target_velocity = [0.0]
        inp.target_acceleration = [0.0]
        # speed_scale — vel/acc 만 배율 (jerk 유지 = 프로파일 모양 보존). 하한은
        # 계약(gt=0)이 보장하지만 로컬 호출 방어로 한 번 더 clamp.
        scale = min(max(speed_scale, 0.05), 1.0)
        inp.max_velocity = [self._c_max_vel * scale]
        inp.max_acceleration = [self._c_max_acc * scale]
        inp.max_jerk = [self._c_max_jerk]
        if scale < 1.0:
            logger.info("%s: speed_scale=%.2f (v≤%.3fm/s)", label, scale,
                        self._c_max_vel * scale)

        # 명령 = seeded IK 결과를 **직접** 발행 (EMA 저역통과 없음). 옛 EMA(alpha=0.1)는
        # Ruckig 의 매끈한 프로파일을 지연시켜 이동 끝에 3~4.5° 잔차 → 램프-snap = 비매끄러움
        # (2026-07-13 진단). 부드러움은 jerk-limited 프로파일 + seed 연쇄(연속 IK)에서 나온다
        # (MoveJ 동형). IK 튐이 실측되면 그때 IK 연속성을 고치지 lag 필터로 덮지 않는다.
        q_cmd = list(start_angles)  # 직전 명령 = 다음 IK seed

        first_result = otg.update(inp, out)
        est_duration = out.trajectory.duration
        t_start = time.time()

        # ── 진단 샘플: 루프 안에서는 I/O 없이 메모리에만 모으고(타이밍 교란 방지)
        # 이동 종료 후 finally 에서 1회 요약 로그. (elapsed, s_cm, vel, dt_ms,
        # per-tick 관절 max step) — MoveL 매끄러움 진단용.
        diag: list[tuple[float, float, float, float, float]] = []
        diag_prev_cmd = list(start_angles)
        diag_prev_t = t_start
        start_pos = path.position_at(0.0)
        end_pos = path.position_at(path.total_length)

        def _diag_sample() -> None:
            nonlocal diag_prev_cmd, diag_prev_t
            now = time.time()
            dt_ms = (now - diag_prev_t) * 1000.0
            diag_prev_t = now
            step_max = max(
                (abs(a - b) for a, b in zip(q_cmd, diag_prev_cmd)), default=0.0
            )
            diag_prev_cmd = list(q_cmd)
            diag.append(
                (now - t_start, out.new_position[0] * 100.0,
                 out.new_velocity[0], dt_ms, step_max)
            )

        def _ik_step(s: float) -> bool:
            nonlocal q_cmd
            wp_list = path.position_at(s)
            wp: Position3 = (wp_list[0], wp_list[1], wp_list[2])
            frac = s / path.total_length if path.total_length > 0 else 1.0
            raw = self._solve_ik(wp, _orientation_at(frac), q_cmd)
            if raw is None:
                logger.warning("%s IK 실패 | s=%.1fcm", label, s * 100)
                return False
            q_cmd = raw
            self._publish_cmd(q_cmd)
            return True

        def _hold() -> None:
            # 프로파일이 목표에서 v=0 로 이미 매끈히 끝났으니 램프-snap 불필요 —
            # 마지막 명령을 몇 틱 유지해 servo 가 정착하게만 한다.
            for _ in range(_CART_HOLD_STEPS):
                if self._stop_ev.is_set():
                    return
                self._publish_cmd(list(q_cmd))
                time.sleep(TRAJ_DT)

        try:
            if not _ik_step(out.new_position[0]):
                self._publish_state(TrajStatus.FAILED, 0.0)
                return
            _diag_sample()
            elapsed = time.time() - t_start
            progress = min(elapsed / est_duration, 1.0) if est_duration > 0 else 1.0
            self._publish_state(TrajStatus.RUNNING, progress)

            if first_result == Result.Finished:
                _hold()
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
                _diag_sample()
                elapsed = time.time() - t_start
                progress = (
                    min(elapsed / est_duration, 1.0) if est_duration > 0 else 1.0
                )
                self._publish_state(TrajStatus.RUNNING, progress)
                if result == Result.Finished:
                    _hold()
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
            self._log_cart_diag(
                label, start_pos, end_pos, est_duration, _ori_angle, diag
            )

    def _log_cart_diag(
        self,
        label: str,
        start_pos: list[float],
        end_pos: list[float],
        est_duration: float,
        ori_angle: float,
        diag: list[tuple[float, float, float, float, float]],
    ) -> None:
        """cartesian 이동 1건 진단 요약 (이동 종료 후 1회 — 루프 타이밍 비교란).

        판별 기준: dt(목표 20ms) 지터 = Windows sleep 분해능 / per-tick 관절 step =
        명령 불연속(튐 = 비매끄러움) / ori = 이동 중 자세 재배향 총각(큰 값 = 자세가
        크게 도는 MoveL — 관절 부담↑). pick&place 의 lift 는 start→end 의 z 가 올라가는
        MoveL 로 식별 (descend 는 내려감)."""
        if not diag:
            logger.info("%s 진단: 샘플 없음 (즉시 종료)", label)
            return
        n = len(diag)
        dur = diag[-1][0]
        dts = [d[3] for d in diag]
        steps = [d[4] for d in diag]
        logger.info(
            "%s 진단: start(%.3f,%.3f,%.3f)->end(%.3f,%.3f,%.3f)m "
            "ori=%.1f° ticks=%d dur=%.2fs(est %.2fs) | dt ms mean=%.1f max=%.1f "
            "min=%.1f (목표 %.0f) | 관절 per-tick step max=%.4frad",
            label,
            start_pos[0], start_pos[1], start_pos[2],
            end_pos[0], end_pos[1], end_pos[2],
            math.degrees(ori_angle),
            n, dur, est_duration,
            sum(dts) / n, max(dts), min(dts), TRAJ_DT * 1000.0,
            max(steps),
        )
        if logger.isEnabledFor(logging.DEBUG):
            for (el, s_cm, vel, dt_ms, step) in diag:
                logger.debug(
                    "%s  t=%.3f s=%.2fcm v=%.3f dt=%.1fms step=%.4f",
                    label, el, s_cm, vel, dt_ms, step,
                )

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
