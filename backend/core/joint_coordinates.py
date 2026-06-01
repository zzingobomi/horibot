"""모터 raw ↔ URDF rad 변환 단일 진입점.

기존: units.rad_to_raw가 offset_rad를 받고, JointStateCache가 raw_to_rad에 offset을
가산. 두 함수가 서로 대칭이라는 책임을 묵시적으로 공유 — 새 사용처가 생기면 한 쪽만
적용하기 쉬움. 또한 분산 모드에서 모터 Pi와 PC가 같은 offset을 갖도록 토픽으로
publish/subscribe 인프라가 따라붙음.

여기서 통일:
    - 디스크의 joint_offsets.npz를 1회 load → 메모리 보관
    - motor_to_urdf / urdf_to_motor 두 함수가 유일한 진입점
    - 분산 동기화는 git이 처리 (.npz 3종이 git 추적, 같은 commit = 같은 파일)
    - 토픽 publish 없음 — COMMIT 후 다른 머신 적용은 git pull + 재시작
"""

from __future__ import annotations

import logging
import threading

from core.robot_registry import RobotRegistry
from core.units import raw_to_rad, rad_to_raw
from modules.calibration import joint_offsets as joint_offsets_io
from modules.dynamixel.motor_config import MotorConfig

logger = logging.getLogger(__name__)


def _joint_offsets_path():
    """현재 active robot 의 calibration dir 에서 joint_offsets.npz path 반환.

    multi-robot Phase: 현재는 RobotRegistry().default() 로 single robot. robot_id
    차원 도입 (후속 todo) 시 dict[robot_id] 로 변경.
    """
    return RobotRegistry().default().calibration_dir / "joint_offsets.npz"


class JointCoordinates:
    """싱글톤. 부팅 시 robot/calibration/joint_offsets.npz를 디스크에서 1회 load.

    분산 모드에서는 모든 머신이 같은 git commit을 바라보므로 같은 파일을 가짐.
    Zenoh pub/sub 전파 없음 — COMMIT 후 PC 외 머신 적용은 git pull + 재시작.
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
        self._offsets: dict[int, float] = joint_offsets_io.load(_joint_offsets_path())
        if self._offsets:
            logger.info(
                "joint_offsets 적용: %s",
                {i: round(o, 5) for i, o in self._offsets.items()},
            )

    def motor_to_urdf(self, raw: int, cfg: MotorConfig) -> float:
        """모터 raw → URDF rad. offset 가산 (+)."""
        rad = raw_to_rad(raw, reverse=cfg.reverse)
        with self._cache_lock:
            return rad + self._offsets.get(cfg.id, 0.0)

    def urdf_to_motor(
        self,
        rad: float,
        cfg: MotorConfig,
        *,
        min_raw: int = 0,
        max_raw: int = 4095,
    ) -> int:
        """URDF rad → 모터 raw. offset 차감 (−)."""
        with self._cache_lock:
            corrected = rad - self._offsets.get(cfg.id, 0.0)
        return rad_to_raw(
            corrected, reverse=cfg.reverse, min_raw=min_raw, max_raw=max_raw
        )

    def commit_offsets(
        self, delta_by_id: dict[int, float], method: str
    ) -> dict[int, float]:
        """COMMIT 시 atomic 갱신: 디스크 save + 메모리 reload (PC 내부 한정).

        다른 머신 전파는 git pull + 재시작이 담당.
        """
        existing = joint_offsets_io.load(_joint_offsets_path())
        merged = joint_offsets_io.merge_delta(existing, delta_by_id)
        joint_offsets_io.save(_joint_offsets_path(), merged, method=method)
        with self._cache_lock:
            self._offsets = dict(merged)
        return dict(merged)

    def snapshot(self) -> dict[int, float]:
        """현재 메모리 상태 (HTTP 응답 / 진단용)."""
        with self._cache_lock:
            return dict(self._offsets)
