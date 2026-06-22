from __future__ import annotations

import logging
import threading

from core.robot.robot_registry import RobotRegistry
from core.units import raw_to_rad, rad_to_raw
from modules.motor.motor_config import MotorConfig

logger = logging.getLogger(__name__)


class JointCoordinates:
    """모터 raw ↔ URDF rad 변환 단일 진입점 (joint_offset 자동 적용).

    Process-wide Memory State (외부 자원 X — in-memory offset 상태).
    """

    _instance: "JointCoordinates | None" = None
    _new_lock = threading.Lock()

    def __new__(cls) -> "JointCoordinates":
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
        self._offsets_by_robot: dict[str, dict[int, float]] = {}

    def _resolve(self, robot_id: str | None) -> str:
        return robot_id if robot_id is not None else RobotRegistry().default_robot_id()

    def set_offsets(self, robot_id: str, offsets: dict[int, float]) -> None:
        with self._cache_lock:
            self._offsets_by_robot[robot_id] = dict(offsets)
        if offsets:
            logger.info(
                "joint_offsets[%s] 적용: %s",
                robot_id,
                {i: round(o, 5) for i, o in offsets.items()},
            )

    def motor_to_urdf(
        self,
        raw: int,
        cfg: MotorConfig,
        robot_id: str | None = None,
    ) -> float:
        """모터 raw → URDF rad. offset 가산 (+)."""
        rid = self._resolve(robot_id)
        rad = raw_to_rad(raw, reverse=cfg.reverse)
        with self._cache_lock:
            return rad + self._offsets_by_robot.get(rid, {}).get(cfg.id, 0.0)

    def urdf_to_motor(
        self,
        rad: float,
        cfg: MotorConfig,
        *,
        min_raw: int = 0,
        max_raw: int = 4095,
        robot_id: str | None = None,
    ) -> int:
        """URDF rad → 모터 raw. offset 차감 (−)."""
        rid = self._resolve(robot_id)
        with self._cache_lock:
            offsets = self._offsets_by_robot.get(rid, {})
            corrected = rad - offsets.get(cfg.id, 0.0)
        return rad_to_raw(
            corrected, reverse=cfg.reverse, min_raw=min_raw, max_raw=max_raw
        )

    def snapshot(self, robot_id: str | None = None) -> dict[int, float]:
        rid = self._resolve(robot_id)
        with self._cache_lock:
            return dict(self._offsets_by_robot.get(rid, {}))
