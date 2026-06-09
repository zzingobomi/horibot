"""알고리즘이나 시나리오를 다루는 노드 베이스. device 노드 위 레이어.

호스트당 인스턴스 하나 — 여러 robot 다룰 땐 enabled_robot_ids 로 순회.
"""

from __future__ import annotations

from core.robot.robot_registry import RobotRegistry
from core.transport.base_node import BaseNode


class ApplicationNode(BaseNode):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name, robot_id=None)
        self._registry = RobotRegistry()
        self.enabled_robot_ids: list[str] = [
            c.robot_id for c in self._registry.enabled_robots()
        ]
