from abc import ABC, abstractmethod
import numpy as np

from core.units import deg_to_rad
from modules.kinematics.trajectory_runner import (
    TrajectoryRunner,
    LinearPath,
    ArcPath,
    SplinePath,
)


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
