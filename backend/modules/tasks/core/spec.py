from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class TaskRobotSpec:
    gripper_open_raw: int
    gripper_close_raw: int
    gripper_index: int
    gripper_held_threshold_raw: int
