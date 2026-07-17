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
    *,
    host: str = "",
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
    if name == "motion_preview":
        # robot-agnostic plan-only 미리보기 — motion 과 같은 값을 preview 전용으로
        # 투영 (motion 인스턴스 미참조). motion 한계/URDF 가 완비된 arm robot 만.
        from modules.motion.adapters.pybullet import PybulletKinematics
        from modules.motion_preview.module import PreviewRobotSpec
        from modules.motor.contract import MotorKind

        preview_specs: dict[str, PreviewRobotSpec] = {}
        for r in robots.values():
            if not r.enabled:
                continue
            arm = [m for m in r.motors if m.kind != MotorKind.GRIPPER]
            if not arm or r.motors[: len(arm)] != arm:
                continue
            try:
                lims = [r.motion_joint_limits[m.name] for m in arm]
            except KeyError:
                continue  # motion.yaml 한계 미완 — 미리보기 불가 robot 은 skip
            preview_specs[r.id] = PreviewRobotSpec(
                kinematics_factory=PybulletKinematics,
                urdf_path=_ROBOT_DIR / r.type / "urdf" / f"{r.type}.urdf",
                arm_specs=arm,
                joint_max_velocity=[x.max_velocity for x in lims],
                joint_max_acceleration=[x.max_acceleration for x in lims],
                joint_max_jerk=[x.max_jerk for x in lims],
                cartesian_max_velocity=r.cartesian_limits.max_trans_vel,
                cartesian_max_acceleration=r.cartesian_limits.max_trans_acc,
                cartesian_max_jerk=r.cartesian_limits.max_trans_jerk,
            )
        return {"robots": preview_specs}
    if name == "waypoint":
        from modules.waypoint.persistence.repository import WaypointRepository

        return {
            "repository": WaypointRepository(
                _require_session_factory("waypoint", session_factory)
            )
        }
    if name == "pick_and_place":
        # task 모듈 공통 물리 config — gripper raw 를 motors.yaml 에서 투영
        # (TaskContext.gripper 가 사용. 물리값 추측 X — motors.yaml SSOT).
        from modules.motor.contract import MotorKind
        from modules.tasks.core.spec import TaskRobotSpec

        task_specs: dict[str, TaskRobotSpec] = {}
        for r in robots.values():
            if not r.enabled:
                continue
            grip = next((m for m in r.motors if m.kind == MotorKind.GRIPPER), None)
            if grip is None:
                continue
            open_raw, close_raw = grip.limit_max, grip.limit_min
            # held 문턱 = close + 5% range. 15% → 5% (2026-07-17 실물 실측):
            # 조는 피벗 회전이라 25mm 큐브를 조 끝으로 물면 gap 이 117~181 raw
            # 뿐 (so101 range 1251 의 15%=188 이 물림 분포 전체 위 → 물고도
            # EMPTY 오판 6연속, 영상 확인). 진짜 빈손 실측 gap=6 — 5%(63) 가
            # 빈손×10 / 물림 최소치의 절반 자리.
            held = close_raw + round((open_raw - close_raw) * 0.05)
            task_specs[r.id] = TaskRobotSpec(
                gripper_open_raw=open_raw,
                gripper_close_raw=close_raw,
                gripper_index=r.motors.index(grip),
                gripper_held_threshold_raw=held,
            )
        return {"robots": task_specs}
    if name == "handover":
        # pick_and_place 와 같은 gripper spec 투영 (위 분기 복제 — 공유 helper
        # 추출은 pick_and_place 실물 검증 완료 후: 지금은 그 분기를 안 건드리는
        # 게 우선, 2026-07-17) + 크로스캘 base_pose / cross-robot 충돌 체커.
        import math as _math

        from modules.motor.contract import MotorKind
        from modules.tasks.core.spec import TaskRobotSpec
        from modules.tasks.handover.collision import BasePose, CrossRobotChecker

        ho_specs: dict[str, TaskRobotSpec] = {}
        for r in robots.values():
            if not r.enabled:
                continue
            grip = next((m for m in r.motors if m.kind == MotorKind.GRIPPER), None)
            if grip is None:
                continue
            open_raw, close_raw = grip.limit_max, grip.limit_min
            held = close_raw + round((open_raw - close_raw) * 0.05)
            ho_specs[r.id] = TaskRobotSpec(
                gripper_open_raw=open_raw,
                gripper_close_raw=close_raw,
                gripper_index=r.motors.index(grip),
                gripper_held_threshold_raw=held,
            )
        so = robots.get("so101_6dof_0")
        omx = robots.get("omx_f_0")
        base = None
        checker = None
        if omx is not None:
            base = BasePose(
                x=omx.base_pose.x,
                y=omx.base_pose.y,
                z=omx.base_pose.z,
                yaw_rad=_math.radians(omx.base_pose.yaw_deg),
            )
            if so is not None:
                checker = CrossRobotChecker(
                    _ROBOT_DIR / so.type / "urdf" / f"{so.type}.urdf",
                    _ROBOT_DIR / omx.type / "urdf" / f"{omx.type}.urdf",
                    base,
                )
        return {"robots": ho_specs, "omx_base_pose": base, "checker": checker}
    if name == "bridge":
        from modules.bridge.contract import BasePoseInfo, RobotInfo

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
    if name == "logcollector":
        # dep 없음 — raw transport 만 필요하고 그건 add_module 이 파라미터 이름
        # `transport` 로 자동 주입 (bridge 와 동일 경로). deployment yaml 로 배치.
        return {}
    if name == "host_monitor":
        # host = deployment `--host` (payload 에 각인 → bridge fan-in demux, §3.4.1).
        return {"host": host}
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

# mock 배치 전용 ready 자세 (arm raw, motors.yaml 순) — motors.yaml home 영점
# ([0]*rad)이 IK-특이(seed 가 정확해도 PyBullet 수치 IK 가 1cm 내 수렴 실패)라, sim
# 미리보기/데모가 그 자세에서 안 도는 것을 피하려고 자연스러운 자세에서 부팅한다.
# **rad 영점(home)은 불변** — 이건 "sim 로봇이 어디서 쉬나"일 뿐 캘/변환 기준이 아니다.
# 실 driver 는 실물 위치에서 시작하므로 무관 (mock 편의). J6=3083(≈91°)=D405
# top-down. 값은 사용자 실측 자세(Robot State 패널)를 raw 로 그대로 (round-trip 오차
# 없이 재현). robot type 키 (없으면 home).
_MOCK_READY_POSE_RAW: dict[str, list[int]] = {
    # J1~J6 raw — 사용자 실측 자세 (Robot State: 2.5/32.8/-51.6/62.3/0.0/91.0°).
    # J5=2048(=0.0°=home), J6=3083(≈91°=D405 top-down).
    "so101_6dof": [2077, 2421, 1461, 2757, 2048, 3083],
}


def _mock_initial_raw(robot: RobotConfig) -> list[int] | None:
    from modules.motor.contract import MotorKind

    ready = _MOCK_READY_POSE_RAW.get(robot.type)
    if ready is None:
        return None
    arm = [m for m in robot.motors if m.kind != MotorKind.GRIPPER]
    if len(ready) != len(arm):
        return None  # 레이아웃 불일치 — 안전하게 기본(home)
    rest_raw = [m.initial_raw for m in robot.motors[len(arm) :]]  # gripper 등
    return list(ready) + rest_raw


def _motor_driver(robot: RobotConfig, deploy: DeploymentConfig) -> MotorBackend:
    if deploy.driver_mode == DriverMode.MOCK:
        from modules.motor.drivers.mock import MockMotorBackend

        return MockMotorBackend(
            motors=robot.motors, initial_positions_raw=_mock_initial_raw(robot)
        )
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
    if robot.motor_backend == "dynamixel":
        from modules.motor.drivers.dynamixel import DynamixelBackend

        if robot.motor_port is None:
            raise ValueError(
                f"robot {robot.id} 에 motor_port 없음 (instance.yaml platform port)"
            )
        return DynamixelBackend(
            motors=robot.motors,
            port=robot.motor_port,
            baudrate=robot.motor_baudrate or 1_000_000,
        )
    raise NotImplementedError(f"real motor driver {robot.motor_backend!r} 미구현.")


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
    if robot.camera_backend == "opencv":
        from modules.camera.drivers.opencv_uvc import OpenCVUvcDriver

        return OpenCVUvcDriver(device_index=robot.camera_device_index or 0)
    raise NotImplementedError(f"real camera driver {robot.camera_backend!r} 미구현.")


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
