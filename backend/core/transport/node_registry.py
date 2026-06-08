"""노드 메타데이터 declaration + lazy import factory.

각 노드의 scope (ROBOT / SYSTEM) 가 declaration 으로 SSOT. main.py 가 host
config 의 robot_nodes / system_nodes 위치 검증 + 인스턴스화 분기에 사용.

scope 정의:
  - ROBOT  — hardware 직결 (motor / camera / motion). robot 마다 별도 인스턴스.
             create_node(name, robot_id=...) 필수. main.py 가 robots × robot_nodes
             데카르트곱으로 인스턴스화.
  - SYSTEM — robot 무관 (bridge 와 별개로 task / detector / pointcloud /
             calibration / gamepad). 한 인스턴스. 내부에서 dict[robot_id]
             dispatch. create_node(name) — robot_id 안 받음.
"""

from __future__ import annotations

import importlib
import logging
from dataclasses import dataclass
from enum import Enum

logger = logging.getLogger(__name__)


class NodeScope(Enum):
    ROBOT = "robot"
    SYSTEM = "system"


@dataclass(frozen=True)
class NodeSpec:
    module: str
    cls_name: str
    scope: NodeScope


_NODE_REGISTRY: dict[str, NodeSpec] = {
    # ─── ROBOT scope — hardware 직결 ──────────────────────
    "motor":       NodeSpec("nodes.motor_node",       "MotorNode",       NodeScope.ROBOT),
    "motion":      NodeSpec("nodes.motion_node",      "MotionNode",      NodeScope.ROBOT),
    "camera":      NodeSpec("nodes.camera_node",      "CameraNode",      NodeScope.ROBOT),
    "mock_motor":  NodeSpec("nodes.motor_node_mock",  "MockMotorNode",   NodeScope.ROBOT),
    "mock_camera": NodeSpec("nodes.camera_node_mock", "MockCameraNode",  NodeScope.ROBOT),
    # ─── SYSTEM scope — 한 인스턴스, multi-robot dispatch ───
    "detector":    NodeSpec("nodes.detector_node",    "DetectorNode",    NodeScope.SYSTEM),
    "pointcloud":  NodeSpec("nodes.pointcloud_node",  "PointCloudNode",  NodeScope.SYSTEM),
    "calibration": NodeSpec("nodes.calibration_node", "CalibrationNode", NodeScope.SYSTEM),
    "task":        NodeSpec("nodes.task_node",        "TaskNode",        NodeScope.SYSTEM),
    "gamepad":     NodeSpec("nodes.gamepad_node",     "GamepadNode",     NodeScope.SYSTEM),
}


def known_nodes() -> list[str]:
    return list(_NODE_REGISTRY)


def get_spec(name: str) -> NodeSpec:
    if name not in _NODE_REGISTRY:
        raise KeyError(f"알 수 없는 노드 '{name}'. 등록된 노드: {known_nodes()}")
    return _NODE_REGISTRY[name]


def create_node(name: str, robot_id: str | None = None):
    """noded factory — lazy import 로 의존성 트리 격리.

    ROBOT scope: robot_id 필수. caller 가 명시.
    SYSTEM scope: robot_id 안 받음 — 내부에서 RobotRegistry.enabled_robots() 사용.
    """
    spec = get_spec(name)
    logger.debug("노드 '%s' lazy import: %s.%s (scope=%s)",
                 name, spec.module, spec.cls_name, spec.scope.value)
    module = importlib.import_module(spec.module)
    cls = getattr(module, spec.cls_name)
    if spec.scope == NodeScope.ROBOT:
        if robot_id is None:
            raise ValueError(
                f"노드 '{name}' 은 ROBOT scope — robot_id 필수"
            )
        return cls(robot_id=robot_id)
    # SYSTEM scope — 인자 없음
    return cls()
