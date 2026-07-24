from __future__ import annotations

import math
from typing import Callable

from . import servo
from .contract import TaskMarker
from .geometry import PlaceCandidate, Vec3


class MarkerPublisher:
    def __init__(
        self,
        publish: Callable[[str, list[TaskMarker]], None],
        robot_id: str,
    ) -> None:
        self._publish = publish
        self._robot_id = robot_id
        self._place: TaskMarker | None = None

    def set_place(self, drop: PlaceCandidate) -> None:
        self._place = TaskMarker(
            label="place",
            position=drop.place,
            approach=_unit_dir(drop.pre, drop.place),
            quaternion=drop.quat,
        )

    def show_grasp(self, p: Vec3, fam: servo.GraspFamily) -> None:
        markers = [_grasp_marker(p, fam)]
        if self._place is not None:
            markers.append(self._place)
        self._publish(self._robot_id, markers)


def _grasp_marker(p: Vec3, fam: servo.GraspFamily) -> TaskMarker:
    return TaskMarker(
        label="grasp",
        position=p,
        approach=fam.approach,
        jaw_axis=fam.jaw_axis,
        quaternion=fam.quat,
    )


def _unit_dir(a: Vec3, b: Vec3) -> Vec3 | None:
    d = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
    n = math.sqrt(d[0] ** 2 + d[1] ** 2 + d[2] ** 2)
    if n < 1e-9:
        return None
    return (d[0] / n, d[1] / n, d[2] / n)
