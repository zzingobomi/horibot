from __future__ import annotations

from typing import TYPE_CHECKING, Any

from .config import _ROBOT_DIR, DeploymentConfig, DriverMode, RobotConfig

if TYPE_CHECKING:
    from sqlalchemy.orm import Session, sessionmaker

    from framework.storage.protocol import ObjectStore
    from modules.camera.drivers.protocol import CameraDriver
    from modules.detector.drivers.protocol import DetectorBackend
    from modules.llm.drivers.protocol import LlmBackend
    from modules.motor.drivers.protocol import MotorBackend


def resolve_robot_deps(
    name: str,
    robot: RobotConfig,
    deploy: DeploymentConfig,
    session_factory: sessionmaker[Session] | None = None,
) -> dict[str, Any]:
    if name == "motor":
        return {"driver": _motor_driver(robot, deploy)}
    if name == "camera":
        return {"driver": _camera_driver(robot, deploy)}
    if name == "camera_decoded":
        return {}
    if name == "motion":
        return _motion_deps(robot)
    raise NotImplementedError(
        f"robot-scoped resolve 미지원 module: {name!r} (robot={robot.id})"
    )


def resolve_host_deps(
    name: str,
    robots: dict[str, RobotConfig],
    deploy: DeploymentConfig,
    runtime: Any | None = None,
    session_factory: sessionmaker[Session] | None = None,
) -> dict[str, Any]:
    if name == "detector":
        return {"backend": _detector_backend(deploy)}
    if name == "llm":
        return {"backend": _llm_backend(deploy)}
    if name == "calibration":
        from modules.calibration.module import CalibrationRobotSpec
        from modules.calibration.persistence.repository import CalibrationRepository
        from modules.motor.contract import MotorKind

        specs = {
            r.id: CalibrationRobotSpec(
                motor_ids=[m.id for m in r.motors if m.kind != MotorKind.GRIPPER],
                has_camera=r.camera_backend is not None,
            )
            for r in robots.values()
            if r.enabled
        }
        return {
            "repository": CalibrationRepository(
                _require_session_factory("calibration", session_factory)
            ),
            "object_store": _object_store(_require_object_uri("calibration", deploy)),
            "robots": specs,
        }
    if name == "scene3d":
        return {
            "robot_ids": [
                r.id for r in robots.values() if r.enabled and "rgbd" in r.capabilities
            ]
        }
    if name == "scan":
        from modules.motion.adapters.pybullet import PybulletKinematics
        from modules.motor.contract import MotorKind
        from modules.scan.module import ScanRobotSpec
        from modules.scan.persistence.repository import ScanRepository

        scan_specs = {
            r.id: ScanRobotSpec(
                # factory 주입 — build 시점 fresh bundle 의 link_offset 이 로드할
                # URDF 를 결정 (motion D4 동일 이유). 보정 kin 은 build 마다 구성.
                kinematics_factory=PybulletKinematics,
                urdf_path=_ROBOT_DIR / r.type / "urdf" / f"{r.type}.urdf",
                arm_specs=[m for m in r.motors if m.kind != MotorKind.GRIPPER],
            )
            for r in robots.values()
            if r.enabled and "rgbd" in r.capabilities
        }
        return {
            "repository": ScanRepository(
                _require_session_factory("scan", session_factory)
            ),
            "object_store": _object_store(_require_object_uri("scan", deploy)),
            "robots": scan_specs,
        }
    if name == "waypoint":
        from modules.waypoint.persistence.repository import WaypointRepository

        return {
            "repository": WaypointRepository(
                _require_session_factory("waypoint", session_factory)
            )
        }
    if name == "task":
        from modules.motor.contract import MotorKind
        from modules.task.spec import TaskRobotSpec

        task_specs: dict[str, TaskRobotSpec] = {}
        for r in robots.values():
            if not r.enabled:
                continue
            grip = next((m for m in r.motors if m.kind == MotorKind.GRIPPER), None)
            if grip is None:
                continue
            open_raw, close_raw = grip.limit_max, grip.limit_min
            held = close_raw + round((open_raw - close_raw) * 0.15)
            task_specs[r.id] = TaskRobotSpec(
                gripper_open_raw=open_raw,
                gripper_close_raw=close_raw,
                gripper_index=r.motors.index(grip),
                gripper_held_threshold_raw=held,
            )
        return {"robots": task_specs}
    if name == "pick_and_place":
        # 현재는 추가 dep 없음.
        return {}
    if name == "bridge":
        from modules.bridge.contract import BasePoseInfo, RobotInfo, TaskInfo
        from modules.task.tasks import task_infos

        # robots.yaml spec — enabled=false robot 은 런타임이 무시 (frontend 에
        # 노출 X). "기본 로봇" 개념 없음 — robot 은 라우트/task 바인딩에서 명시.
        enabled_robots = [r for r in robots.values() if r.enabled]
        infos = [
            RobotInfo(
                id=r.id,
                type=r.type,
                base_pose=BasePoseInfo(
                    x=r.base_pose.x,
                    y=r.base_pose.y,
                    z=r.base_pose.z,
                    yaw_deg=r.base_pose.yaw_deg,
                ),
                capabilities=list(r.capabilities),
                has_camera=r.camera_backend is not None,
            )
            for r in enabled_robots
        ]
        deps: dict[str, Any] = {
            "robots": infos,
            "robot_dir": _ROBOT_DIR,
            "port": deploy.bridge_port,
            "dev_console": deploy.dev_console,
            # task 가 참여 robot 을 선언 (§2.7) — bridge 는 GET /tasks 로 relay 만.
            "tasks": [
                TaskInfo(name=name, robot_ids=robot_ids)
                for name, robot_ids in task_infos()
            ],
        }
        if runtime is not None:
            # contract는 요청 시점에 생성한다.
            # 이때는 모든 모듈 등록이 끝난 뒤이므로 최신 상태를 반환할 수 있다.
            def _contract_provider() -> dict:
                from apps.contract_export import build_contract_json

                return build_contract_json(runtime.contract_snapshot())

            # runtime에는 현재 프로세스의 모듈만 있으므로,
            # 그래프는 MODULE_REGISTRY를 기준으로 전체 모듈을 표시한다.
            def _graph_provider() -> dict:
                from apps.contract_export import build_static_contract_graph

                return build_static_contract_graph()

            deps["contract_provider"] = _contract_provider
            deps["graph_provider"] = _graph_provider
        return deps
    raise NotImplementedError(f"host-level resolve 미지원 module: {name!r}")


# ─── 공통 dep 전제 (DB / blob) ──────────────────────────────────


def _require_session_factory(
    name: str, session_factory: sessionmaker[Session] | None
) -> sessionmaker[Session]:
    if session_factory is None:
        raise ValueError(
            f"{name} 배치엔 deployment 의 rdb_uri 필요 (session_factory 미주입)"
        )
    return session_factory


def _require_object_uri(name: str, deploy: DeploymentConfig) -> str:
    if deploy.object_uri is None:
        raise ValueError(f"{name} 배치엔 deployment 의 object_uri 필요 (blob)")
    return deploy.object_uri


# ─── object store (blob) ────────────────────────────────────────


def _object_store(object_uri: str) -> ObjectStore:
    from infra.object_store.filesystem import FilesystemObjectStore

    if object_uri.startswith("file:///"):
        base = object_uri[len("file://") :]  # /path 유지
    elif object_uri.startswith("file://"):
        base = object_uri[len("file://") :]
    else:
        base = object_uri
    return FilesystemObjectStore(base)


# ─── driver / kinematics ─────────────────────


def _motor_driver(robot: RobotConfig, deploy: DeploymentConfig) -> MotorBackend:
    if deploy.driver_mode == DriverMode.MOCK:
        from modules.motor.drivers.mock import MockMotorBackend

        return MockMotorBackend(motors=robot.motors)
    if robot.motor_backend == "feetech":
        from modules.motor.drivers.feetech import FeetechBackend

        if robot.motor_port is None:
            raise ValueError(
                f"robot {robot.id} 에 motor_port 없음 (instance.yaml platform port)"
            )
        return FeetechBackend(
            motors=robot.motors,
            port=robot.motor_port,
            baudrate=robot.motor_baudrate or 1_000_000,
        )
    raise NotImplementedError(
        f"real motor driver {robot.motor_backend!r} 미구현 (dynamixel 등 후속)."
    )


def _detector_backend(deploy: DeploymentConfig) -> DetectorBackend:
    if deploy.driver_mode == DriverMode.MOCK:
        from modules.detector.drivers.mock import MockDetectorBackend

        return MockDetectorBackend()

    from modules.detector.drivers.grounded_sam import GroundedSamBackend

    return GroundedSamBackend()


def _llm_backend(deploy: DeploymentConfig) -> LlmBackend:
    if deploy.driver_mode == DriverMode.MOCK:
        from modules.llm.drivers.mock import MockLlmBackend

        return MockLlmBackend()

    from modules.llm.drivers.qwen import QwenBackend

    return QwenBackend()


def _camera_driver(robot: RobotConfig, deploy: DeploymentConfig) -> CameraDriver:
    if robot.camera_backend is None:
        raise ValueError(f"robot {robot.id} 에 camera_backend 없음 — camera 배치 불가.")
    if deploy.driver_mode == DriverMode.MOCK:
        from modules.camera.drivers.mock import MockCameraDriver

        return MockCameraDriver(has_depth="rgbd" in robot.capabilities)
    if robot.camera_backend == "realsense":
        from modules.camera.drivers.realsense_d405 import RealSenseD405Driver

        return RealSenseD405Driver()
    raise NotImplementedError(
        f"real camera driver {robot.camera_backend!r} 미구현 (opencv 등 후속)."
    )


def _motion_deps(robot: RobotConfig) -> dict[str, Any]:
    from modules.motion.adapters.pybullet import PybulletKinematics
    from modules.motor.contract import MotorKind

    arm = [s for s in robot.motors if s.kind != MotorKind.GRIPPER]
    if robot.motors[: len(arm)] != arm:
        raise ValueError(
            f"robot {robot.id}: arm joint가 motors.yaml의 앞쪽에 연속으로 배치되어 있지 않습니다. "
            f"Motion은 motors.yaml의 앞쪽 DOF개를 arm joint로 사용하므로, "
            f"gripper/rail은 arm joint 뒤에 배치해야 합니다."
        )
    limits = []
    for s in arm:
        lim = robot.motion_joint_limits.get(s.name)
        if lim is None:
            raise ValueError(
                f"robot {robot.id} motion.yaml 에 joint '{s.name}' limit 없음"
            )
        limits.append(lim)
    urdf = _ROBOT_DIR / robot.type / "urdf" / f"{robot.type}.urdf"
    # gripper 관절 report 용 (arm 아님) — URDF 시각화가 open/close 를 보이게.
    gripper = next((s for s in robot.motors if s.kind == MotorKind.GRIPPER), None)
    return {
        # D4 — link_offset(calibration) 이 patched URDF 경로를 결정하므로 factory 주입.
        # PybulletKinematics 클래스 자체가 Path → Kinematics factory.
        "kinematics_factory": PybulletKinematics,
        "urdf_path": urdf,
        "arm_specs": arm,
        "gripper_spec": gripper,
        "gripper_index": robot.motors.index(gripper) if gripper else None,
        "joint_max_velocity": [x.max_velocity for x in limits],
        "joint_max_acceleration": [x.max_acceleration for x in limits],
        "joint_max_jerk": [x.max_jerk for x in limits],
        "cartesian_max_velocity": robot.cartesian_limits.max_trans_vel,
        "cartesian_max_acceleration": robot.cartesian_limits.max_trans_acc,
        "cartesian_max_jerk": robot.cartesian_limits.max_trans_jerk,
    }
