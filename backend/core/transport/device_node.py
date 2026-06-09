"""특정 robot 의 하드웨어를 다루는 노드 베이스.

robot 마다 인스턴스 하나씩 — robot_id 가 그 robot 을 가리킴.
"""

from __future__ import annotations

from core.transport.base_node import BaseNode


class DeviceNode(BaseNode):
    def __init__(self, node_name: str, robot_id: str) -> None:
        super().__init__(node_name, robot_id=robot_id)
