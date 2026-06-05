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
    # ─── Mock variants (frontend UX 개발용, host_mock.yaml 에서 사용) ──────
    # 실 motor/camera 대신 자리 채움 — topic/service contract 만 충족.
    "mock_motor": ("nodes.motor_node_mock", "MockMotorNode"),
    "mock_camera": ("nodes.camera_node_mock", "MockCameraNode"),
}


def known_nodes() -> list[str]:
    return list(_NODE_REGISTRY)


def create_node(name: str, robot_id: str | None = None):
    """robot_id — robot-scoped 노드 (motor / motion / camera / calibration /
    detector / pointcloud) 가 담당할 robot id. task / gamepad 는 global.

    모든 노드 __init__ 이 robot_id 받음 (BaseNode signature 일관). global
    노드는 받아도 안 씀 — robot-scoped service 호출 시 BaseNode.r() 의
    default fallback 으로 처리.
    """
    if name not in _NODE_REGISTRY:
        raise KeyError(f"알 수 없는 노드 '{name}'. 등록된 노드: {known_nodes()}")

    module_path, class_name = _NODE_REGISTRY[name]
    logger.debug("노드 '%s' lazy import: %s.%s", name, module_path, class_name)
    module = importlib.import_module(module_path)
    cls = getattr(module, class_name)
    return cls(robot_id=robot_id)
