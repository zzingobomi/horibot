"""ApplicationNode — robot driver 위에 얹는 application/algorithm layer.

calibration / task / detector / pointcloud / gamepad 처럼 robot 무관한
알고리즘/시나리오 레이어 노드 base. DeviceNode (vendor-shipped) 의 contract
(토픽/서비스) 만 보고 동작 — 특정 hardware backend 모름.

상속 의미:
  - `robot_id` 없음 (호스트당 1 인스턴스)
  - multi-robot dispatch 표준 — `self.enabled_robot_ids` 로 활성 robot 순회
  - `issubclass(cls, ApplicationNode)` 가 layer 판정 SSOT
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
