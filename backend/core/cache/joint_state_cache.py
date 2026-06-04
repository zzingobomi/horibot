"""MOTOR_STATE_JOINT 토픽 구독 + raw → URDF rad 변환 캐시. robot_id 차원 도입.

multi_robot_architecture.md §4.5 참조. state: `dict[robot_id] -> {raw, loads}`.

offset 적용은 [JointCoordinates](joint_coordinates.py) 가 담당 — 본 클래스는 "최신
raw 보관 + URDF rad 환산 dispatch" 책임만.
"""

from __future__ import annotations

import threading
from typing import TYPE_CHECKING

from core.robot.robot_registry import RobotRegistry
from core.transport.topic_map import Topic
from core.units import raw_to_rad
from modules.motor.motor_config import MotorConfig

if TYPE_CHECKING:
    from core.transport.base_node import BaseNode


class JointStateCache:
    """싱글톤 — 내부 state 는 dict[robot_id]. subscribe() 가 robot 별 토픽 구독.

    Phase 1 (토픽 namespace 미정정) 에선 단일 MOTOR_STATE_JOINT 토픽 구독 →
    default robot_id 로 저장. Phase 1.D (topic namespace 정정) 에서 robot 별 토픽
    (`<robot_id>/motor/state/joint`) 으로 분리.
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
        self._raw_by_robot: dict[str, dict[int, int]] = {}
        self._loads_by_robot: dict[str, dict[int, int]] = {}
        self._cache_lock = threading.Lock()
        self._subscribed_robots: set[str] = set()

    def subscribe(self, node: "BaseNode", robot_id: str | None = None) -> None:
        """해당 robot 의 motor state 토픽 구독. None 이면 default robot.

        Topic.MOTOR_STATE_JOINT 가 `horibot/{robot_id}/motor/state/joint` template
        — 명시적 rid 로 expand (node.robot_id 와 다를 수 있어 self.r 사용 X).
        """
        rid = self._resolve(robot_id)
        if rid in self._subscribed_robots:
            return
        self._subscribed_robots.add(rid)
        node.create_subscriber(
            Topic.MOTOR_STATE_JOINT.format(robot_id=rid),
            lambda data, _rid=rid: self._on_motor_state(_rid, data),
        )

    def _on_motor_state(self, robot_id: str, data: dict) -> None:
        joints = data.get("joints", [])
        with self._cache_lock:
            raw = self._raw_by_robot.setdefault(robot_id, {})
            loads = self._loads_by_robot.setdefault(robot_id, {})
            for j in joints:
                raw[j["id"]] = j["position"]
                if "load" in j:
                    loads[j["id"]] = j["load"]

    def _resolve(self, robot_id: str | None) -> str:
        return robot_id if robot_id is not None else RobotRegistry().default_robot_id()

    def get_joint_angles_rad(
        self,
        arm_cfgs: list[MotorConfig],
        robot_id: str | None = None,
    ) -> list[float] | None:
        """캘리브레이션된 조인트각 반환. JointCoordinates 로 offset 자동 보정."""
        from core.coords.joint_coordinates import JointCoordinates

        rid = self._resolve(robot_id)
        coords = JointCoordinates()
        with self._cache_lock:
            raw_dict = self._raw_by_robot.get(rid, {})
            if not raw_dict:
                return None
            result = []
            for cfg in arm_cfgs:
                raw = raw_dict.get(cfg.id)
                if raw is None:
                    return None
                result.append(coords.motor_to_urdf(raw, cfg, robot_id=rid))
            return result

    def get_joint_angles_rad_uncorrected(
        self,
        arm_cfgs: list[MotorConfig],
        robot_id: str | None = None,
    ) -> list[float] | None:
        """offset 적용 전 raw→rad 결과. 캘 진단/디버깅용."""
        rid = self._resolve(robot_id)
        with self._cache_lock:
            raw_dict = self._raw_by_robot.get(rid, {})
            if not raw_dict:
                return None
            result = []
            for cfg in arm_cfgs:
                raw = raw_dict.get(cfg.id)
                if raw is None:
                    return None
                result.append(raw_to_rad(raw, reverse=cfg.reverse))
            return result

    def get_raw(self, motor_id: int, robot_id: str | None = None) -> int | None:
        rid = self._resolve(robot_id)
        with self._cache_lock:
            return self._raw_by_robot.get(rid, {}).get(motor_id)

    def get_raw_motor_positions(
        self,
        arm_cfgs: list[MotorConfig],
        robot_id: str | None = None,
    ) -> dict[int, int] | None:
        """arm 모터 raw 묶음. 캘 캡처에서 *시점 독립 ground truth* 로 저장.

        offset / URDF rad 변환 X — 그건 COMPUTE 시점에서 JointCoordinates 가 함.
        """
        rid = self._resolve(robot_id)
        with self._cache_lock:
            raw_dict = self._raw_by_robot.get(rid, {})
            if not raw_dict:
                return None
            result: dict[int, int] = {}
            for cfg in arm_cfgs:
                raw = raw_dict.get(cfg.id)
                if raw is None:
                    return None
                result[cfg.id] = int(raw)
            return result

    def get_present_loads(
        self,
        arm_cfgs: list[MotorConfig],
        robot_id: str | None = None,
    ) -> dict[int, int] | None:
        """arm 모터 raw Present_Load 묶음. contact spike 감지용.

        XL430 = ‰ (-1000~+1000), XL330 = mA — raw 그대로 (해석은 호출 측).
        """
        rid = self._resolve(robot_id)
        with self._cache_lock:
            loads_dict = self._loads_by_robot.get(rid, {})
            if not loads_dict:
                return None
            result: dict[int, int] = {}
            for cfg in arm_cfgs:
                load = loads_dict.get(cfg.id)
                if load is None:
                    return None
                result[cfg.id] = int(load)
            return result
