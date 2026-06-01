"""자세 의존 중력 sag stiffness 의 런타임 진입점. robot_id 차원 도입 (multi_robot §4.5).

[JointCoordinates](backend/core/joint_coordinates.py),
[LinkCoordinates](backend/core/link_coordinates.py) 와 같은 dict[robot_id] 패턴.
state: `dict[robot_id] -> SagOffsets`.

joint_offsets / link_offsets 와 다른 점:
    - 값이 *scalar k* per joint (rad/(m·g_unit), lumped mass 가정)
    - 사용처가 *FK/IK 호출 시 angle 보정* — CorrectedIKSolver 가 fk/ik 메서드에서
      `apply_gravity_sag(angles, k_array)` 로 입력 angle 을 sag 적용 후 inner 호출.
    - PyBullet URDF 재로드 불필요 (link_offsets 와 달리) — commit 후 메모리 캐시만
      갱신하면 다음 FK/IK 호출부터 자동 반영. 단 분산 머신은 git pull + 재시작.
    - **commit_offsets semantics: overwrite** (link_offsets 와 동일 — BA 의 sag_k
      도 absolute total. accuracy_squeeze_plan §1.6)
"""

from __future__ import annotations

import logging
import threading

from core.robot_registry import RobotRegistry
from modules.calibration import sag_offsets as sag_offsets_io
from modules.calibration.sag_offsets import SagOffsets

logger = logging.getLogger(__name__)


def _sag_offsets_path(robot_id: str):
    return RobotRegistry().get(robot_id).calibration_dir / "sag_offsets.npz"


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
        for cfg in RobotRegistry().enabled_robots():
            path = cfg.calibration_dir / "sag_offsets.npz"
            offsets = sag_offsets_io.load(path)
            self._offsets_by_robot[cfg.robot_id] = offsets
            if not offsets.is_empty():
                ks = ", ".join(
                    f"J{jid}={k:+.4f}"
                    for jid, k in sorted(offsets.k_rad_per_m.items())
                )
                logger.info(f"sag_offsets[{cfg.robot_id}] 적용: {ks}")

    def _resolve(self, robot_id: str | None) -> str:
        return robot_id if robot_id is not None else RobotRegistry().default_robot_id()

    def _empty(self) -> SagOffsets:
        return SagOffsets(k_rad_per_m={})

    def snapshot(self, robot_id: str | None = None) -> SagOffsets:
        rid = self._resolve(robot_id)
        with self._cache_lock:
            offsets = self._offsets_by_robot.get(rid, self._empty())
            return SagOffsets(k_rad_per_m=dict(offsets.k_rad_per_m))

    def get_k(self, jid: int, robot_id: str | None = None) -> float:
        rid = self._resolve(robot_id)
        with self._cache_lock:
            return self._offsets_by_robot.get(rid, self._empty()).get_k(jid)

    def commit_offsets(
        self,
        offsets: SagOffsets,
        method: str,
        robot_id: str | None = None,
    ) -> SagOffsets:
        """COMMIT 시 atomic 갱신: 디스크 *overwrite* + 메모리 reload (PC 내부 한정).

        Overwrite semantics. CorrectedIKSolver 재시작 불필요 — 다음 fk/ik 호출이
        snapshot() 으로 읽으면 자동 반영. 단 분산 머신은 git pull + 재시작.
        """
        rid = self._resolve(robot_id)
        sag_offsets_io.save(_sag_offsets_path(rid), offsets, method=method)
        with self._cache_lock:
            self._offsets_by_robot[rid] = SagOffsets(
                k_rad_per_m=dict(offsets.k_rad_per_m)
            )
        return self.snapshot(rid)
