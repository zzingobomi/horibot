from __future__ import annotations

from typing import Any

from modules.bridge.contract import BasePoseInfo, RobotInfo
from modules.bridge.module import BridgeModule
from modules.camera.decoded import CameraDecodedModule
from modules.camera.module import CameraDriverModule
from modules.motor.module import MotorDriverModule

from .config import DeploymentConfig, DriverMode, RobotConfig


def resolve_deps(
    mod_cls: type,
    robot: RobotConfig,
    deploy: DeploymentConfig,
) -> dict[str, Any]:
    if mod_cls is MotorDriverModule:
        return {"driver": _motor_driver(robot, deploy)}
    if mod_cls is CameraDriverModule:
        return {"driver": _camera_driver(robot, deploy)}
    if mod_cls is CameraDecodedModule:
        return {}
    raise NotImplementedError(
        f"resolve_deps 미지원 Module: {mod_cls.__name__} "
        f"(robot={robot.id}). registry 추가 시 여기도 분기 박을 것."
    )


def resolve_host_deps(
    mod_cls: type,
    robots: dict[str, RobotConfig],
    deploy: DeploymentConfig,
) -> dict[str, Any]:
    """host-level (robot-agnostic) Module 의 constructor deps."""
    if mod_cls is BridgeModule:
        # 내부 config 모델 (RobotConfig) → frontend wire 모델 (RobotInfo) 변환.
        # 레이어링 — modules/bridge 는 apps 모름, 변환은 apps 책임 (§8.6 / §9.1).
        return {"robots": [_to_robot_info(r) for r in robots.values()]}
    raise NotImplementedError(f"host-level Module 미구현: {mod_cls.__name__} (Step C+)")


def _to_robot_info(robot: RobotConfig) -> RobotInfo:
    return RobotInfo(
        id=robot.id,
        type=robot.type,
        base_pose=BasePoseInfo(
            x=robot.base_pose.x,
            y=robot.base_pose.y,
            z=robot.base_pose.z,
            yaw_deg=robot.base_pose.yaw_deg,
        ),
        capabilities=list(robot.capabilities),
    )


# ─── driver 선택 ────────────────────────────────────────────────


def _motor_driver(robot: RobotConfig, deploy: DeploymentConfig) -> Any:
    if deploy.driver_mode == DriverMode.MOCK:
        from modules.motor.drivers.mock import MockMotorBackend

        # 레이아웃 SSOT = motors.yaml (robot.motors). mock·real 공통.
        return MockMotorBackend(motors=robot.motors)
    # real — vendor 별 분기
    if robot.motor_backend == "feetech":
        from modules.motor.drivers.feetech import FeetechBackend

        if robot.motor_port is None:
            raise ValueError(
                f"robot {robot.id} 에 motor_port 없음 (instance.yaml 의 platform port 확인)"
            )
        return FeetechBackend(
            motors=robot.motors,
            port=robot.motor_port,
            baudrate=robot.motor_baudrate or 1_000_000,
        )
    raise NotImplementedError(
        f"real motor driver {robot.motor_backend!r} 미구현 (dynamixel 등 후속)."
    )


def _camera_driver(robot: RobotConfig, deploy: DeploymentConfig) -> Any:
    if robot.camera_backend is None:
        raise ValueError(
            f"robot {robot.id} 에 camera_backend 없음 — camera Module 배치 불가."
        )
    if deploy.driver_mode == DriverMode.MOCK:
        from modules.camera.drivers.mock import MockCameraDriver

        has_depth = "rgbd" in robot.capabilities
        return MockCameraDriver(has_depth=has_depth)
    # real — vendor 별 분기
    if robot.camera_backend == "realsense":
        from modules.camera.drivers.realsense_d405 import RealSenseD405Driver

        return RealSenseD405Driver()
    raise NotImplementedError(
        f"real camera driver {robot.camera_backend!r} 미구현 (opencv 등 후속)."
    )
