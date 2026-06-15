"""자세 의존 중력 sag stiffness 의 런타임 진입점.

[JointCoordinates](backend/core/coords/joint_coordinates.py) 와 같은 패턴 —
storage 모름. calibration_node 가 `set_offsets` 로 주입. docs/storage_layer.md §7.
"""

from __future__ import annotations

import logging
import threading

from core.robot.robot_registry import RobotRegistry
from modules.calibration.sag_offsets import SagOffsets

logger = logging.getLogger(__name__)


class SagCoordinates:
    _instance: "SagCoordinates | None" = None
    _new_lock = threading.Lock()

    def __new__(cls) -> "SagCoordinates":
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
        self._offsets_by_robot: dict[str, SagOffsets] = {}

    def _resolve(self, robot_id: str | None) -> str:
        return robot_id if robot_id is not None else RobotRegistry().default_robot_id()

    def _empty(self) -> SagOffsets:
        return SagOffsets(k_rad_per_m={})

    def set_offsets(self, robot_id: str, offsets: SagOffsets) -> None:
        """calibration_node 가 storage 에서 load 후 주입."""
        with self._cache_lock:
            self._offsets_by_robot[robot_id] = SagOffsets(
                k_rad_per_m=dict(offsets.k_rad_per_m)
            )
        if not offsets.is_empty():
            ks = ", ".join(
                f"J{jid}={k:+.4f}"
                for jid, k in sorted(offsets.k_rad_per_m.items())
            )
            logger.info(f"sag_offsets[{robot_id}] 적용: {ks}")

    def snapshot(self, robot_id: str | None = None) -> SagOffsets:
        rid = self._resolve(robot_id)
        with self._cache_lock:
            offsets = self._offsets_by_robot.get(rid, self._empty())
            return SagOffsets(k_rad_per_m=dict(offsets.k_rad_per_m))

    def get_k(self, jid: int, robot_id: str | None = None) -> float:
        rid = self._resolve(robot_id)
        with self._cache_lock:
            return self._offsets_by_robot.get(rid, self._empty()).get_k(jid)
