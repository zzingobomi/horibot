"""Tool offset 의 런타임 진입점. robot_id 차원 도입 (multi_robot §4.5).

[LinkCoordinates](backend/core/coords/link_coordinates.py) 와 같은 dict[robot_id] 패턴.
state: `dict[robot_id] -> ToolOffset`.

LinkCoordinates 와 다른 점:
    - 단일 값 (trans 3 + rot 3) — joint 별 dict 아님
    - URDF patch 안 함 — motion_node 의 cartesian service handler 가 명령/응답
      변환에만 사용
    - 메모리 자동 갱신 (SagCorrectedKinematics 재시작 불필요) — 단 분산 머신은
      git pull + 재시작
"""

from __future__ import annotations

import logging
import threading

import numpy as np

from core.robot.robot_registry import RobotRegistry
from modules.calibration import tool_offset as tool_offset_io
from modules.calibration.tool_offset import ToolOffset

logger = logging.getLogger(__name__)


def _tool_offset_path(robot_id: str):
    return RobotRegistry().get(robot_id).calibration_dir / "tool_offset.npz"


class ToolCoordinates:
    _instance: "ToolCoordinates | None" = None
    _new_lock = threading.Lock()

    def __new__(cls) -> "ToolCoordinates":
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
        self._offset_by_robot: dict[str, ToolOffset] = {}
        for cfg in RobotRegistry().enabled_robots():
            path = cfg.calibration_dir / "tool_offset.npz"
            offset = tool_offset_io.load(path)
            self._offset_by_robot[cfg.robot_id] = offset
            if not offset.is_empty():
                logger.info(
                    "tool_offset[%s] 적용: trans_mm=%s",
                    cfg.robot_id,
                    (offset.trans_m * 1000).round(2).tolist(),
                )

    def _resolve(self, robot_id: str | None) -> str:
        return robot_id if robot_id is not None else RobotRegistry().default_robot_id()

    def _empty(self) -> ToolOffset:
        return ToolOffset(
            trans_m=np.zeros(3, dtype=np.float64),
            rot_rad=np.zeros(3, dtype=np.float64),
        )

    def snapshot(self, robot_id: str | None = None) -> ToolOffset:
        rid = self._resolve(robot_id)
        with self._cache_lock:
            offset = self._offset_by_robot.get(rid, self._empty())
            return ToolOffset(
                trans_m=offset.trans_m.copy(),
                rot_rad=offset.rot_rad.copy(),
            )

    def trans_m(self, robot_id: str | None = None) -> np.ndarray:
        """EE frame 의 (실제 끝점 - URDF EE) translation. (3,) ndarray."""
        rid = self._resolve(robot_id)
        with self._cache_lock:
            return self._offset_by_robot.get(rid, self._empty()).trans_m.copy()

    def rot_rad(self, robot_id: str | None = None) -> np.ndarray:
        """EE frame 의 (실제 끝점 - URDF EE) rotation rotvec. (3,) ndarray."""
        rid = self._resolve(robot_id)
        with self._cache_lock:
            return self._offset_by_robot.get(rid, self._empty()).rot_rad.copy()

    def commit_absolute(
        self,
        offset: ToolOffset,
        method: str,
        robot_id: str | None = None,
    ) -> ToolOffset:
        """COMMIT 시 atomic 갱신: 디스크 *overwrite* + 메모리 reload.

        URDF patch 안 함 → SagCorrectedKinematics 재시작 불필요. 단 다른 머신은
        git pull + 재시작.
        """
        rid = self._resolve(robot_id)
        tool_offset_io.save(_tool_offset_path(rid), offset, method=method)
        with self._cache_lock:
            self._offset_by_robot[rid] = ToolOffset(
                trans_m=offset.trans_m.copy(),
                rot_rad=offset.rot_rad.copy(),
            )
        return self.snapshot(rid)

    def reload(self, robot_id: str | None = None) -> ToolOffset:
        """디스크에서 다시 로드 → 메모리 갱신 (rollback 후 호출)."""
        rid = self._resolve(robot_id)
        loaded = tool_offset_io.load(_tool_offset_path(rid))
        with self._cache_lock:
            self._offset_by_robot[rid] = ToolOffset(
                trans_m=loaded.trans_m.copy(),
                rot_rad=loaded.rot_rad.copy(),
            )
        return self.snapshot(rid)
