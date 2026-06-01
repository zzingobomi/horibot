"""Tool offset 의 런타임 진입점 (ToolOffset 싱글톤 캐시).

[LinkCoordinates](backend/core/link_coordinates.py) 와 같은 싱글톤 + 디스크 캐시 패턴:
    - 디스크의 robot/calibration/tool_offset.npz 를 부팅 시 1회 load → 메모리 보관
    - snapshot() / commit_offset() — 디스크 save + 메모리 reload
    - 분산 동기화는 git 처리 (.npz 는 git 추적, 같은 commit = 같은 파일)
    - 토픽 publish 없음 — COMMIT 후 다른 머신 적용은 git pull + 재시작

LinkCoordinates 와 다른 점:
    - 단일 값 (trans 3 + rot 3) — joint 별 dict 아님
    - URDF patch 안 함 — motion_node 의 cartesian service handler 가 명령/응답
      변환에만 사용 (자세한 분리 이유는 modules/calibration/tool_offset.py 의
      모듈 docstring 참조)
    - 메모리 자동 갱신 (PybulletSolver 재시작 불필요) — 단 분산 머신은 git pull + 재시작
"""

from __future__ import annotations

import logging
import threading

import numpy as np

from core.robot_registry import RobotRegistry
from modules.calibration import tool_offset as tool_offset_io
from modules.calibration.tool_offset import ToolOffset

logger = logging.getLogger(__name__)


def _tool_offset_path():
    return RobotRegistry().default().calibration_dir / "tool_offset.npz"


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
        self._offset: ToolOffset = tool_offset_io.load(_tool_offset_path())
        if not self._offset.is_empty():
            logger.info(
                "tool_offset 적용: trans_mm=%s",
                (self._offset.trans_m * 1000).round(2).tolist(),
            )

    def snapshot(self) -> ToolOffset:
        with self._cache_lock:
            return ToolOffset(
                trans_m=self._offset.trans_m.copy(),
                rot_rad=self._offset.rot_rad.copy(),
            )

    def trans_m(self) -> np.ndarray:
        """EE frame 의 (실제 끝점 - URDF EE) translation. (3,) ndarray."""
        with self._cache_lock:
            return self._offset.trans_m.copy()

    def rot_rad(self) -> np.ndarray:
        """EE frame 의 (실제 끝점 - URDF EE) rotation rotvec. (3,) ndarray."""
        with self._cache_lock:
            return self._offset.rot_rad.copy()

    def commit_offset(self, offset: ToolOffset, method: str) -> ToolOffset:
        """COMMIT 시 atomic 갱신: 디스크 overwrite + 메모리 reload.

        URDF patch 안 함 → PybulletSolver 재시작 불필요. 단 다른 머신은 git pull + 재시작.
        """
        tool_offset_io.save(_tool_offset_path(), offset, method=method)
        with self._cache_lock:
            self._offset = ToolOffset(
                trans_m=offset.trans_m.copy(),
                rot_rad=offset.rot_rad.copy(),
            )
        return self.snapshot()
