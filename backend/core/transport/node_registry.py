"""노드 lazy-import factory.

특정 호스트에서는 필요 없는 노드가 있을 수 있으므로,
노드 등록과 import 를 분리하여 불필요한 import 방지
"""

from __future__ import annotations

import importlib
import logging
from dataclasses import dataclass

from core.transport.application_node import ApplicationNode
from core.transport.base_node import BaseNode
from core.transport.device_node import DeviceNode

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class NodeSpec:
    module: str
    cls_name: str


_NODE_REGISTRY: dict[str, NodeSpec] = {
    # ─── Device — per-robot ─────────────────────────────────
    "motor": NodeSpec("nodes.device.motor_node", "MotorNode"),
    "motion": NodeSpec("nodes.device.motion_node", "MotionNode"),
    "camera": NodeSpec("nodes.device.camera_node", "CameraNode"),
    "mock_motor": NodeSpec("nodes.device.motor_node_mock", "MockMotorNode"),
    "mock_camera": NodeSpec("nodes.device.camera_node_mock", "MockCameraNode"),
    # ─── Application ────────────────────────────────────────
    "detector": NodeSpec("nodes.application.detector_node", "DetectorNode"),
    "pointcloud": NodeSpec("nodes.application.pointcloud_node", "PointCloudNode"),
    "calibration": NodeSpec("nodes.application.calibration_node", "CalibrationNode"),
    "task": NodeSpec("nodes.application.task_node", "TaskNode"),
    "gamepad": NodeSpec("nodes.application.gamepad_node", "GamepadNode"),
    "storage": NodeSpec("nodes.application.storage_node", "StorageNode"),
}


def known_nodes() -> list[str]:
    return list(_NODE_REGISTRY)


def get_class(name: str) -> type[BaseNode]:
    if name not in _NODE_REGISTRY:
        raise KeyError(f"알 수 없는 노드 '{name}'. 등록된 노드: {known_nodes()}")
    spec = _NODE_REGISTRY[name]
    logger.debug("노드 '%s' lazy import: %s.%s", name, spec.module, spec.cls_name)
    module = importlib.import_module(spec.module)
    return getattr(module, spec.cls_name)


def create_node(name: str, robot_id: str | None = None) -> BaseNode:
    cls = get_class(name)
    if issubclass(cls, DeviceNode):
        if robot_id is None:
            raise ValueError(f"노드 '{name}' 은 DeviceNode — robot_id 필수")
        return cls(robot_id=robot_id)
    if issubclass(cls, ApplicationNode):
        return cls()
    raise TypeError(
        f"노드 '{name}' ({cls.__name__}) 는 DeviceNode/ApplicationNode 둘 다 상속 안 함"
    )
