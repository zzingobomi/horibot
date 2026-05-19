import threading
from typing import TYPE_CHECKING

from core.topic_map import Topic
from core.units import raw_to_rad
from core.joint_offsets import get_joint_offset
from modules.dynamixel.motor_config import MotorConfig

if TYPE_CHECKING:
    from core.base_node import BaseNode


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
        with self._cache_lock:
            if not self._raw:
                return None
            result = []
            for idx, cfg in enumerate(arm_cfgs):
                raw = self._raw.get(cfg.id)
                if raw is None:
                    return None
                result.append(
                    raw_to_rad(
                        raw,
                        reverse=cfg.reverse,
                        offset_rad=get_joint_offset(idx),
                    )
                )
            return result

    def get_raw(self, motor_id: int) -> int | None:
        with self._cache_lock:
            return self._raw.get(motor_id)
