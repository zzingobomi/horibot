"""URDF link origin offset 의 런타임 진입점. robot_id 차원 도입 (multi_robot §4.5).

[JointCoordinates](backend/core/joint_coordinates.py) 와 같은 dict[robot_id] 패턴.
state: `dict[robot_id] -> LinkOffsets`.

joint_offsets 와 다른 점은 동일:
    - 값이 *2종* (link_trans (3,) m, link_rot (3,) rad rotvec) per joint
    - 사용처가 *URDF patch* (PybulletIKSolver 부팅 시 urdf_patcher 호출에 들어감)
    - **commit_offsets semantics: overwrite (절대값 덮어쓰기)**
      (BA 의 link_t 는 absolute total — accuracy_squeeze_plan §1.6)
"""

from __future__ import annotations

import logging
import threading

import numpy as np

from core.robot_registry import RobotRegistry
from modules.calibration import link_offsets as link_offsets_io
from modules.calibration.link_offsets import LinkOffsets

logger = logging.getLogger(__name__)


def _link_offsets_path(robot_id: str):
    return RobotRegistry().get(robot_id).calibration_dir / "link_offsets.npz"


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
        for cfg in RobotRegistry().enabled_robots():
            path = cfg.calibration_dir / "link_offsets.npz"
            offsets = link_offsets_io.load(path)
            self._offsets_by_robot[cfg.robot_id] = offsets
            if not offsets.is_empty():
                n = max(len(offsets.trans), len(offsets.rot))
                logger.info(f"link_offsets[{cfg.robot_id}] 적용: {n} joints")

    def _resolve(self, robot_id: str | None) -> str:
        return robot_id if robot_id is not None else RobotRegistry().default_robot_id()

    def _empty(self) -> LinkOffsets:
        return LinkOffsets(trans={}, rot={})

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

    def commit_offsets(
        self,
        offsets: LinkOffsets,
        method: str,
        robot_id: str | None = None,
    ) -> LinkOffsets:
        """COMMIT 시 atomic 갱신: 디스크 *overwrite* + 메모리 reload (PC 내부 한정).

        Overwrite semantics — `offsets` 는 absolute total 값. cumulative 가산 X.
        다른 머신 전파는 git pull + 재시작.
        """
        rid = self._resolve(robot_id)
        link_offsets_io.save(_link_offsets_path(rid), offsets, method=method)
        with self._cache_lock:
            self._offsets_by_robot[rid] = LinkOffsets(
                trans=dict(offsets.trans),
                rot=dict(offsets.rot),
            )
        return self.snapshot(rid)
