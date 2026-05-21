import threading
from typing import TYPE_CHECKING

from core.topic_map import Topic
from core.units import raw_to_rad
from modules.dynamixel.motor_config import MotorConfig

if TYPE_CHECKING:
    from core.base_node import BaseNode


class JointStateCache:
    """MOTOR_STATE_JOINT 토픽 구독 + raw → URDF rad 변환 캐시.

    offset 적용은 JointCoordinates 싱글톤에 위임 (단일 진입점). 이 클래스는
    "최신 raw 보관 + URDF rad 환산" 책임만 가짐.
    """

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
        """캘리브레이션된 조인트각 반환. JointCoordinates로 offset 자동 보정."""
        from core.joint_coordinates import JointCoordinates

        coords = JointCoordinates()
        with self._cache_lock:
            if not self._raw:
                return None
            result = []
            for cfg in arm_cfgs:
                raw = self._raw.get(cfg.id)
                if raw is None:
                    return None
                result.append(coords.motor_to_urdf(raw, cfg))
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

    def get_raw_motor_positions(
        self, arm_cfgs: list[MotorConfig]
    ) -> dict[int, int] | None:
        """arm 모터 raw 묶음. 캘 캡처에서 *시점 독립 ground truth*로 저장하기 위함.

        offset / URDF rad 변환 X — 그건 COMPUTE 시점에서 JointCoordinates가 함.
        """
        with self._cache_lock:
            if not self._raw:
                return None
            result: dict[int, int] = {}
            for cfg in arm_cfgs:
                raw = self._raw.get(cfg.id)
                if raw is None:
                    return None
                result[cfg.id] = int(raw)
            return result
