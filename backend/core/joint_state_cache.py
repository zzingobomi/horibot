import threading
from pathlib import Path
from typing import TYPE_CHECKING

from core.topic_map import Topic
from core.units import raw_to_rad
from modules.calibration import joint_offsets as joint_offsets_io
from modules.dynamixel.motor_config import MotorConfig

if TYPE_CHECKING:
    from core.base_node import BaseNode


JOINT_OFFSETS_PATH = (
    Path(__file__).parents[2] / "robot" / "calibration" / "joint_offsets.npz"
)


class JointStateCache:
    _instance: "JointStateCache | None" = None
    _lock = threading.Lock()

    def __new__(cls) -> "JointStateCache":
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self) -> None:
        if self._initialized:
            return
        self._initialized = True
        self._raw: dict[int, int] = {}
        self._cache_lock = threading.Lock()
        self._subscribed = False
        self._joint_offsets_rad: dict[int, float] = joint_offsets_io.load(
            JOINT_OFFSETS_PATH
        )
        if self._joint_offsets_rad:
            import logging

            logging.getLogger(__name__).info(
                "joint_offsets 적용: %s",
                {i: round(o, 5) for i, o in self._joint_offsets_rad.items()},
            )

    def reload_joint_offsets(self) -> dict[int, float]:
        """COMMIT 직후 등 외부에서 파일을 갱신했을 때 호출."""
        with self._cache_lock:
            self._joint_offsets_rad = joint_offsets_io.load(JOINT_OFFSETS_PATH)
        return dict(self._joint_offsets_rad)

    def get_joint_offsets_rad(self) -> dict[int, float]:
        """현재 적용 중인 offset 사본 (외부 진단/표시용)."""
        with self._cache_lock:
            return dict(self._joint_offsets_rad)

    def subscribe(self, node: "BaseNode") -> None:
        if self._subscribed:
            return
        self._subscribed = True
        node.create_subscriber(Topic.MOTOR_STATE_JOINT, self._on_motor_state)

    def _on_motor_state(self, data: dict) -> None:
        joints = data.get("joints", [])
        with self._cache_lock:
            for j in joints:
                self._raw[j["id"]] = j["position"]

    def get_joint_angles_rad(self, arm_cfgs: list[MotorConfig]) -> list[float] | None:
        """캘리브레이션된 조인트각 반환. joint_offsets.npz가 있으면 자동 보정."""
        with self._cache_lock:
            if not self._raw:
                return None
            result = []
            for cfg in arm_cfgs:
                raw = self._raw.get(cfg.id)
                if raw is None:
                    return None
                rad = raw_to_rad(raw, reverse=cfg.reverse)
                rad += self._joint_offsets_rad.get(cfg.id, 0.0)
                result.append(rad)
            return result

    def get_joint_angles_rad_uncorrected(
        self, arm_cfgs: list[MotorConfig]
    ) -> list[float] | None:
        """offset 적용 전 raw→rad 결과. 캘 진단/디버깅용."""
        with self._cache_lock:
            if not self._raw:
                return None
            result = []
            for cfg in arm_cfgs:
                raw = self._raw.get(cfg.id)
                if raw is None:
                    return None
                result.append(raw_to_rad(raw, reverse=cfg.reverse))
            return result

    def get_raw(self, motor_id: int) -> int | None:
        with self._cache_lock:
            return self._raw.get(motor_id)
