from __future__ import annotations

import logging
import threading

import numpy as np

from core.robot.robot_registry import RobotRegistry
from modules.calibration.result_models import LinkOffsetResultData

logger = logging.getLogger(__name__)


class LinkCoordinates:
    """URDF link origin offset 의 런타임 진입점.

    Process-wide Memory State (외부 자원 X — in-memory offset 상태).
    """

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
        self._offsets_by_robot: dict[str, LinkOffsetResultData] = {}

    def _resolve(self, robot_id: str | None) -> str:
        return robot_id if robot_id is not None else RobotRegistry().default_robot_id()

    def _empty(self) -> LinkOffsetResultData:
        return LinkOffsetResultData(offsets=[], method="empty")

    def set_offsets(self, robot_id: str, offsets: LinkOffsetResultData) -> None:
        with self._cache_lock:
            self._offsets_by_robot[robot_id] = LinkOffsetResultData(
                offsets=list(offsets.offsets),
                method=offsets.method,
            )
        if not offsets.is_empty():
            logger.info(
                f"link_offsets[{robot_id}] 적용: {len(offsets.offsets)} joints"
            )

    def snapshot(self, robot_id: str | None = None) -> LinkOffsetResultData:
        rid = self._resolve(robot_id)
        with self._cache_lock:
            offsets = self._offsets_by_robot.get(rid, self._empty())
            return LinkOffsetResultData(
                offsets=list(offsets.offsets),
                method=offsets.method,
            )

    def get_trans(self, jid: int, robot_id: str | None = None) -> np.ndarray:
        rid = self._resolve(robot_id)
        with self._cache_lock:
            return self._offsets_by_robot.get(rid, self._empty()).get_trans(jid)

    def get_rot(self, jid: int, robot_id: str | None = None) -> np.ndarray:
        rid = self._resolve(robot_id)
        with self._cache_lock:
            return self._offsets_by_robot.get(rid, self._empty()).get_rot(jid)
