"""DeviceNode — per-robot hardware/controller bundle.

UR Control Box 등가물. motor / camera / motion 처럼 vendor 가 robot 과 같이
들고 오는 layer 의 노드 base. robot 마다 별도 인스턴스 (per-instance).

상속 의미:
  - `robot_id` 필수 (None 허용 X — DeviceNode 는 항상 특정 robot 에 묶임)
  - main.py 가 `device_nodes` × `robots` 데카르트곱으로 인스턴스화
  - `issubclass(cls, DeviceNode)` 가 layer 판정 SSOT
"""

from __future__ import annotations

from core.transport.base_node import BaseNode


class DeviceNode(BaseNode):
    def __init__(self, node_name: str, robot_id: str) -> None:
        super().__init__(node_name, robot_id=robot_id)
