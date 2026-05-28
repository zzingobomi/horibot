"""URDF link origin offset의 런타임 진입점 (LinkOffsets 싱글톤 캐시).

[JointCoordinates](backend/core/joint_coordinates.py)와 같은 패턴:
    - 디스크의 robot/calibration/link_offsets.npz를 부팅 시 1회 load → 메모리 보관
    - snapshot() / commit_offsets() — 디스크 save + 메모리 reload
    - 분산 동기화는 git 처리 (.npz는 git 추적, 같은 commit = 같은 파일)
    - 토픽 publish 없음 — COMMIT 후 다른 머신 적용은 git pull + 재시작

joint_offsets와 다른 점:
    - 값이 *2종* (link_trans (3,) m, link_rot (3,) rad rotvec) per joint
    - 사용처가 *URDF patch* (PybulletSolver 부팅 시 urdf_patcher 호출에 들어감).
      joint_offsets는 raw↔urdf rad 변환에 가산되지만, link_offsets는 URDF의
      <joint><origin xyz rpy/>에 적용. 후자는 FK/IK 둘 다 영향.
"""

from __future__ import annotations

import logging
import threading
from pathlib import Path

import numpy as np

from modules.calibration import link_offsets as link_offsets_io
from modules.calibration.link_offsets import LinkOffsets

LINK_OFFSETS_PATH = (
    Path(__file__).parents[2] / "robot" / "calibration" / "link_offsets.npz"
)

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
        self._offsets: LinkOffsets = link_offsets_io.load(LINK_OFFSETS_PATH)
        if not self._offsets.is_empty():
            n = max(len(self._offsets.trans), len(self._offsets.rot))
            logger.info(f"link_offsets 적용: {n} joints")

    def snapshot(self) -> LinkOffsets:
        with self._cache_lock:
            return LinkOffsets(
                trans=dict(self._offsets.trans),
                rot=dict(self._offsets.rot),
            )

    def get_trans(self, jid: int) -> np.ndarray:
        with self._cache_lock:
            return self._offsets.get_trans(jid)

    def get_rot(self, jid: int) -> np.ndarray:
        with self._cache_lock:
            return self._offsets.get_rot(jid)

    def commit_offsets(
        self,
        delta: LinkOffsets,
        method: str,
    ) -> LinkOffsets:
        """COMMIT 시 atomic 갱신: 디스크 save + 메모리 reload (PC 내부 한정).

        다른 머신 전파는 git pull + 재시작.
        """
        existing = link_offsets_io.load(LINK_OFFSETS_PATH)
        merged = link_offsets_io.merge_delta(existing, delta)
        link_offsets_io.save(LINK_OFFSETS_PATH, merged, method=method)
        with self._cache_lock:
            self._offsets = LinkOffsets(
                trans=dict(merged.trans),
                rot=dict(merged.rot),
            )
        return self.snapshot()
