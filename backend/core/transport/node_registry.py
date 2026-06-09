"""노드 lazy-import factory.

NodeSpec 은 **순수 lazy-import 메커니즘** — `(module, cls_name)` 두 string 만
들고 있어서 `node_registry` import 가 PC 전용 dep (ultralytics / open3d /
pyrealsense2) 를 끌어오지 않음. 모터 Pi 가 detector / pointcloud 등록만
보고도 import 트리 깨끗 유지.

architecture 정보 (이 노드가 어느 layer 냐) 는 NodeSpec 에 없음 —
`DeviceNode` / `ApplicationNode` 클래스 계층 자체가 SSOT. lazy import 후
`issubclass(cls, DeviceNode)` 로 판정.
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
    # ─── Device — vendor-shipped, per-robot ──────────────────
    "motor":       NodeSpec("nodes.device.motor_node",       "MotorNode"),
    "motion":      NodeSpec("nodes.device.motion_node",      "MotionNode"),
    "camera":      NodeSpec("nodes.device.camera_node",      "CameraNode"),
    "mock_motor":  NodeSpec("nodes.device.motor_node_mock",  "MockMotorNode"),
    "mock_camera": NodeSpec("nodes.device.camera_node_mock", "MockCameraNode"),
    # ─── Application — robot driver 위 algorithm layer ──────
    "detector":    NodeSpec("nodes.application.detector_node",    "DetectorNode"),
    "pointcloud":  NodeSpec("nodes.application.pointcloud_node",  "PointCloudNode"),
    "calibration": NodeSpec("nodes.application.calibration_node", "CalibrationNode"),
    "task":        NodeSpec("nodes.application.task_node",        "TaskNode"),
    "gamepad":     NodeSpec("nodes.application.gamepad_node",     "GamepadNode"),
}


def known_nodes() -> list[str]:
    return list(_NODE_REGISTRY)


def get_class(name: str) -> type[BaseNode]:
    """Lazy import 후 클래스 반환. 호출 시점에만 모듈 import — node_registry
    import 자체는 PC 전용 dep 끌어오지 않음.
    """
    if name not in _NODE_REGISTRY:
        raise KeyError(f"알 수 없는 노드 '{name}'. 등록된 노드: {known_nodes()}")
    spec = _NODE_REGISTRY[name]
    logger.debug("노드 '%s' lazy import: %s.%s", name, spec.module, spec.cls_name)
    module = importlib.import_module(spec.module)
    return getattr(module, spec.cls_name)


def create_node(name: str, robot_id: str | None = None) -> BaseNode:
    """노드 인스턴스 생성. 타입 계층 기반 dispatch.

    `DeviceNode` 상속 → robot_id 필수 (per-robot 인스턴스).
    `ApplicationNode` 상속 → robot_id 안 받음 (호스트당 1).
    """
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
