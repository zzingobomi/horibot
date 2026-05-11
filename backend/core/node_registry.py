import importlib
import logging

logger = logging.getLogger(__name__)

_NODE_REGISTRY: dict[str, tuple[str, str]] = {
    "motor": ("nodes.motor_node", "MotorNode"),
    "camera": ("nodes.camera_node", "CameraNode"),
    "motion": ("nodes.motion_node", "MotionNode"),
    "calibration": ("nodes.calibration_node", "CalibrationNode"),
    "task": ("nodes.task_node", "TaskNode"),
    "detector": ("nodes.detector_node", "DetectorNode"),
    "pointcloud": ("nodes.pointcloud_node", "PointCloudNode"),
    "gamepad": ("nodes.gamepad_node", "GamepadNode"),
}


def known_nodes() -> list[str]:
    return list(_NODE_REGISTRY)


def create_node(name: str):
    if name not in _NODE_REGISTRY:
        raise KeyError(f"알 수 없는 노드 '{name}'. 등록된 노드: {known_nodes()}")

    module_path, class_name = _NODE_REGISTRY[name]
    logger.debug("노드 '%s' lazy import: %s.%s", name, module_path, class_name)
    module = importlib.import_module(module_path)
    cls = getattr(module, class_name)
    return cls()
