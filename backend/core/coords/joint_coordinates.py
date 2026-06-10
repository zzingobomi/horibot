"""모터 raw ↔ URDF rad 변환 단일 진입점. robot_id 차원 도입 (multi_robot §4.5).

singleton 유지 — 내부 state 가 `dict[robot_id] → offsets`. 모든 enabled robot 의
joint_offsets.npz 를 부팅 시 load. 메서드는 `robot_id` 인자 받음 (None = default).

기존:
    - 디스크의 joint_offsets.npz를 1회 load → 메모리 보관
    - motor_to_urdf / urdf_to_motor 두 함수가 유일한 진입점
    - 분산 동기화는 git이 처리 (.npz 3종이 git 추적, 같은 commit = 같은 파일)
    - 토픽 publish 없음 — COMMIT 후 다른 머신 적용은 git pull + 재시작
"""

from __future__ import annotations

import logging
import threading

from core.robot.robot_registry import RobotRegistry
from core.units import raw_to_rad, rad_to_raw
from modules.calibration import joint_offsets as joint_offsets_io
from modules.motor.motor_config import MotorConfig

logger = logging.getLogger(__name__)


def _joint_offsets_path(robot_id: str):
    """해당 robot 의 calibration dir 에서 joint_offsets.npz path 반환."""
    return RobotRegistry().get(robot_id).calibration_dir / "joint_offsets.npz"


class JointCoordinates:
    """싱글톤. 부팅 시 enabled robot 들의 joint_offsets.npz 를 1회 load.

    state: `dict[robot_id] -> dict[motor_id, offset_rad]`.
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
        for cfg in RobotRegistry().enabled_robots():
            path = cfg.calibration_dir / "joint_offsets.npz"
            offsets = joint_offsets_io.load(path)
            self._offsets_by_robot[cfg.robot_id] = offsets
            if offsets:
                logger.info(
                    "joint_offsets[%s] 적용: %s",
                    cfg.robot_id,
                    {i: round(o, 5) for i, o in offsets.items()},
                )

    def _resolve(self, robot_id: str | None) -> str:
        return robot_id if robot_id is not None else RobotRegistry().default_robot_id()

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

    def commit_absolute(
        self,
        absolute_by_id: dict[int, float],
        method: str,
        robot_id: str | None = None,
    ) -> dict[int, float]:
        """COMMIT 시 atomic 갱신: 디스크 *overwrite* + 메모리 reload (PC 내부 한정).

        Overwrite semantics — `absolute_by_id` 는 absolute total. cumulative 가산 X.
        link/sag/tool 과 통일된 contract — BA 의 delta 출력은 calibration_node 가
        진입 시점에 `current + delta = absolute` 로 reconcile 한 후 본 함수 호출.

        다른 머신 전파는 git pull + 재시작이 담당.
        """
        rid = self._resolve(robot_id)
        path = _joint_offsets_path(rid)
        joint_offsets_io.save(path, absolute_by_id, method=method)
        with self._cache_lock:
            self._offsets_by_robot[rid] = dict(absolute_by_id)
        return dict(absolute_by_id)

    def reload(self, robot_id: str | None = None) -> dict[int, float]:
        """디스크에서 다시 로드 → 메모리 갱신 (rollback 후 호출)."""
        rid = self._resolve(robot_id)
        loaded = joint_offsets_io.load(_joint_offsets_path(rid))
        with self._cache_lock:
            self._offsets_by_robot[rid] = dict(loaded)
        return dict(loaded)

    def snapshot(self, robot_id: str | None = None) -> dict[int, float]:
        """현재 메모리 상태 (HTTP 응답 / 진단용)."""
        rid = self._resolve(robot_id)
        with self._cache_lock:
            return dict(self._offsets_by_robot.get(rid, {}))
