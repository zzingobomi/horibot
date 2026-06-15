"""모터 raw ↔ URDF rad 변환 단일 진입점. robot_id 차원 (multi_robot §4.5).

싱글톤. **storage 모름** — calibration_node 가 owner, 부팅 시 `set_offsets` 로
주입. docs/storage_layer.md §7 의 layer 분리:

  Storage  ←  CalibrationService (calibration_node)  ─push→  JointCoordinates

state: `dict[robot_id] -> dict[motor_id, offset_rad]`. 주입 전엔 empty
(== offset 0, raw↔rad 변환 그대로).
"""

from __future__ import annotations

import logging
import threading

from core.robot.robot_registry import RobotRegistry
from core.units import raw_to_rad, rad_to_raw
from modules.motor.motor_config import MotorConfig

logger = logging.getLogger(__name__)


class JointCoordinates:
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
        # 주입 전 empty — 부팅 시 storage 접근 X. calibration_node 가 set_offsets 호출.
        self._offsets_by_robot: dict[str, dict[int, float]] = {}

    def _resolve(self, robot_id: str | None) -> str:
        return robot_id if robot_id is not None else RobotRegistry().default_robot_id()

    def set_offsets(
        self, robot_id: str, offsets: dict[int, float]
    ) -> None:
        """calibration_node 가 storage 에서 load 후 호출. in-memory state 만 갱신.

        commit/activate path (disk / storage write) 는 calibration_node 책임.
        """
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
        """현재 메모리 상태 (HTTP 응답 / 진단용)."""
        rid = self._resolve(robot_id)
        with self._cache_lock:
            return dict(self._offsets_by_robot.get(rid, {}))
