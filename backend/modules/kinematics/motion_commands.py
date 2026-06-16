from abc import ABC, abstractmethod
from typing import Callable

import numpy as np

from core.units import deg_to_rad
from modules.kinematics.kinematics import Position3, Quaternion
from modules.kinematics.trajectory_runner import (
    ArcPath,
    LinearPath,
    SplinePath,
    TrajectoryRunner,
)

# ServoTcp 는 trajectory planner 우회 — 매 호출 single IK + direct publish.
# 의존성 주입: motion_modes.servo_tcp (6DOF) + cmd publish.
ServoSolveFn = Callable[
    [Position3, Quaternion | None, list[float]], list[float] | None
]
PublishCmdFn = Callable[[list[float]], None]


class MotionCommand(ABC):
    @abstractmethod
    def validate(self, req: dict) -> str | None:
        ...

    @abstractmethod
    def execute(
        self,
        req:     dict,
        angles:  list[float],
        tcp_pos: list[float],
        runner:  TrajectoryRunner,
    ) -> None:
        ...

    @property
    def label(self) -> str:
        return self.__class__.__name__.replace("Command", "")


class MoveJCommand(MotionCommand):
    def __init__(self, arm_cfgs) -> None:
        self._arm_cfgs = arm_cfgs

    def validate(self, req: dict) -> str | None:
        if not req.get("data", {}).get("joints"):
            return "joints 필요"
        return None

    def execute(self, req, angles, tcp_pos, runner) -> None:
        target_by_id = {
            int(j["id"]): float(j["degree"])
            for j in req["data"]["joints"]
        }
        target_angles = [
            deg_to_rad(target_by_id.get(cfg.id, 0.0))
            for cfg in self._arm_cfgs
        ]
        runner.run_joint(angles, target_angles)


class MoveLCommand(MotionCommand):
    def validate(self, req: dict) -> str | None:
        if req.get("data", {}).get("position") is None:
            return "position 필요"
        return None

    def execute(self, req, angles, tcp_pos, runner) -> None:
        start = np.array(tcp_pos, dtype=float)
        end = np.array(req["data"]["position"], dtype=float)
        runner.run_cartesian(LinearPath(start, end), angles)


class MoveCCommand(MotionCommand):
    def validate(self, req: dict) -> str | None:
        data = req.get("data", {})
        if data.get("via") is None or data.get("end") is None:
            return "via, end 모두 필요"
        return None

    def execute(self, req, angles, tcp_pos, runner) -> None:
        data = req["data"]
        p1 = np.array(tcp_pos,       dtype=float)
        p2 = np.array(data["via"],   dtype=float)
        p3 = np.array(data["end"],   dtype=float)
        runner.run_cartesian(ArcPath(p1, p2, p3), angles)


class MovePCommand(MotionCommand):
    def validate(self, req: dict) -> str | None:
        wps = req.get("data", {}).get("waypoints", [])
        if len(wps) < 2:
            return "waypoints 최소 2개 필요"
        return None

    def execute(self, req, angles, tcp_pos, runner) -> None:
        wps = req["data"]["waypoints"]
        all_pts = np.array([tcp_pos] + [list(wp) for wp in wps], dtype=float)
        runner.run_cartesian(SplinePath(all_pts), angles)


class ServoTcpCommand(MotionCommand):
    """절대 TCP target → planner 우회 직접 IK + publish (chase 패턴).

    caller (gamepad/외부 컨트롤러) 가 빠른 rate (50Hz+) 로 갱신 시 자연스러운 추종.
    Ruckig 트래젝토리 없음 — 1 step / 1 publish.
    Trajectory 가 돌고 있으면 stop (servo 가 trajectory 가로채는 의미).

    `quaternion=None` → position-only IK (5DOF / 6DOF orientation 무시).
    """

    def __init__(
        self,
        solve_servo: ServoSolveFn,
        publish_cmd: PublishCmdFn,
    ) -> None:
        self._solve_servo = solve_servo
        self._publish_cmd = publish_cmd

    def validate(self, req: dict) -> str | None:
        data = req.get("data", {})
        if data.get("position") is None:
            return "position 필요"
        return None

    def execute(self, req, angles, tcp_pos, runner) -> None:
        data = req["data"]
        position = tuple(data["position"])  # type: ignore[assignment]
        quat_raw = data.get("quaternion")
        quaternion = tuple(quat_raw) if quat_raw is not None else None  # type: ignore[assignment]

        # Trajectory 가 돌고 있으면 stop — servo 가 새 chase 자리 진입.
        if runner.is_running:
            runner.stop()

        result = self._solve_servo(position, quaternion, angles)  # type: ignore[arg-type]
        if result is None:
            raise ValueError("IK 수렴 실패")
        self._publish_cmd(result)
