from __future__ import annotations

import logging
import threading

from core.robot.robot_registry import RobotRegistry
from modules.calibration.result_models import SagOffsetResultData

logger = logging.getLogger(__name__)


class SagCoordinates:
    """자세 의존 중력 sag stiffness 의 런타임 진입점.

    Process-wide Memory State (외부 자원 X — in-memory stiffness 상태).
    """

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
        self._offsets_by_robot: dict[str, SagOffsetResultData] = {}

    def _resolve(self, robot_id: str | None) -> str:
        return robot_id if robot_id is not None else RobotRegistry().default_robot_id()

    def _empty(self) -> SagOffsetResultData:
        return SagOffsetResultData(k_rad_per_m={}, method="empty")

    def set_offsets(self, robot_id: str, offsets: SagOffsetResultData) -> None:
        with self._cache_lock:
            self._offsets_by_robot[robot_id] = SagOffsetResultData(
                k_rad_per_m=dict(offsets.k_rad_per_m),
                method=offsets.method,
            )
        if not offsets.is_empty():
            ks = ", ".join(
                f"J{jid}={k:+.4f}" for jid, k in sorted(offsets.k_rad_per_m.items())
            )
            logger.info(f"sag_offsets[{robot_id}] 적용: {ks}")

    def snapshot(self, robot_id: str | None = None) -> SagOffsetResultData:
        rid = self._resolve(robot_id)
        with self._cache_lock:
            offsets = self._offsets_by_robot.get(rid, self._empty())
            return SagOffsetResultData(
                k_rad_per_m=dict(offsets.k_rad_per_m),
                method=offsets.method,
            )

    def get_k(self, jid: int, robot_id: str | None = None) -> float:
        rid = self._resolve(robot_id)
        with self._cache_lock:
            return self._offsets_by_robot.get(rid, self._empty()).get_k(jid)
