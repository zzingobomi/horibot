"""URDF link origin offset 의 런타임 진입점.

[JointCoordinates](backend/core/coords/joint_coordinates.py) 와 같은 dict[robot_id]
패턴. storage 모름 — calibration_node 가 owner, `set_offsets` 로 주입.

PybulletKinematics URDF patch 는 부팅 시 1회 — calibration_node 가 link offsets
주입 후 PybulletKinematics 의 `apply_link_offsets` + `initialize` 호출.
docs/storage_layer.md §7.
"""

from __future__ import annotations

import logging
import threading

import numpy as np

from core.robot.robot_registry import RobotRegistry
from modules.calibration.link_offsets import LinkOffsets

logger = logging.getLogger(__name__)


class LinkCoordinates:
    _instance: "LinkCoordinates | None" = None
    _new_lock = threading.Lock()

    def __new__(cls) -> "LinkCoordinates":
        if cls._instance is None:
            with cls._new_lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
                    cls._instance._initialized = False
        return cls._instance

    def __init__(self) -> None:
        if self._initialized:
            return
        self._initialized = True
        self._cache_lock = threading.Lock()
        self._offsets_by_robot: dict[str, LinkOffsets] = {}

    def _resolve(self, robot_id: str | None) -> str:
        return robot_id if robot_id is not None else RobotRegistry().default_robot_id()

    def _empty(self) -> LinkOffsets:
        return LinkOffsets(trans={}, rot={})

    def set_offsets(self, robot_id: str, offsets: LinkOffsets) -> None:
        """calibration_node 가 storage 에서 load 후 주입."""
        with self._cache_lock:
            self._offsets_by_robot[robot_id] = LinkOffsets(
                trans=dict(offsets.trans), rot=dict(offsets.rot)
            )
        if not offsets.is_empty():
            n = max(len(offsets.trans), len(offsets.rot))
            logger.info(f"link_offsets[{robot_id}] 적용: {n} joints")

    def snapshot(self, robot_id: str | None = None) -> LinkOffsets:
        rid = self._resolve(robot_id)
        with self._cache_lock:
            offsets = self._offsets_by_robot.get(rid, self._empty())
            return LinkOffsets(
                trans=dict(offsets.trans),
                rot=dict(offsets.rot),
            )

    def get_trans(self, jid: int, robot_id: str | None = None) -> np.ndarray:
        rid = self._resolve(robot_id)
        with self._cache_lock:
            return self._offsets_by_robot.get(rid, self._empty()).get_trans(jid)

    def get_rot(self, jid: int, robot_id: str | None = None) -> np.ndarray:
        rid = self._resolve(robot_id)
        with self._cache_lock:
            return self._offsets_by_robot.get(rid, self._empty()).get_rot(jid)
