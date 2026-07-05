"""resolve_deps — application logic: robot config → Module constructor deps (§5.2).

**모듈 클래스를 top-level import 안 함** — module NAME(string) 으로 dispatch + 필요한
것만 branch 안에서 lazy import. registry.py 와 같은 role 격리 이유 (pi_camera 가
resolve import 만으로 pybullet/fastapi 끌어오면 안 됨).

`runtime`/`robot_id` 은 Runtime/main 이 주입 — 여기선 그 외 dep (driver / kinematics 등).
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from .config import _ROBOT_DIR, DeploymentConfig, DriverMode, RobotConfig

if TYPE_CHECKING:
    from sqlalchemy.orm import Session, sessionmaker


def resolve_deps(
    name: str,
    robot: RobotConfig,
    deploy: DeploymentConfig,
    session_factory: sessionmaker[Session] | None = None,
) -> dict[str, Any]:
    """robot-scoped Module 의 constructor deps (runtime / robot_id 제외).

    session_factory = boot 가 만든 process-shared DB factory (DB 모듈만 사용).
    """
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
    """host-level (robot-agnostic) Module 의 constructor deps.

    runtime = build_runtime 이 넘기는 Runtime (bridge 의 contract provider closure 용).
    None 이면 provider 미주입 (GET /contract.json → 503) — contract gen 안 쓰는
    배선/test 경로. session_factory = boot 가 만든 process-shared DB factory
    (DB owner host 의 robot-agnostic DB 모듈만 사용)."""
    if name == "detector":
        # robot-agnostic (host 당 1) — 무거운 모델 1회 로드, req.robot_id 로 dispatch.
        return {"backend": _detector_backend(deploy)}
    if name == "llm":
        # robot-agnostic (host 당 1) — Qwen 1회 로드, 파싱은 robot 무관 (§2.7).
        return {"backend": _llm_backend(deploy)}
    if name == "calibration":
        # robot-agnostic (host 당 1) — DB robot_id 멀티테넌트, req.robot_id dispatch.
        if session_factory is None:
            raise ValueError(
                "calibration 배치엔 deployment 의 rdb_uri 필요 (session_factory 미주입)"
            )
        if deploy.object_uri is None:
            raise ValueError("calibration 배치엔 deployment 의 object_uri 필요 (blob)")
        from modules.calibration.module import CalibrationRobotSpec
        from modules.calibration.persistence.repository import CalibrationRepository
        from modules.motor.contract import MotorKind

        # robots.yaml → lean 투영 (enabled 만). 모듈은 RobotConfig 원본을 안 봄 —
        # bridge 의 RobotInfo 변환과 동형 (내부 config → module dep 는 apps 책임).
        specs = {
            r.id: CalibrationRobotSpec(
                motor_ids=[m.id for m in r.motors if m.kind != MotorKind.GRIPPER],
                has_camera=r.camera_backend is not None,
            )
            for r in robots.values()
            if r.enabled
        }
        return {
            "repository": CalibrationRepository(session_factory),
            "object_store": _object_store(deploy.object_uri),
            "robots": specs,
        }
    if name == "scene3d":
        # robot-agnostic (host 당 1) — 멤버십만 (rgbd capability + enabled).
        return {
            "robot_ids": [
                r.id
                for r in robots.values()
                if r.enabled and "rgbd" in r.capabilities
            ]
        }
    if name == "scan":
        # robot-agnostic (host 당 1) — rgbd robot 별 kinematics/arm 투영 + DB/blob.
        if session_factory is None:
            raise ValueError("scan 배치엔 deployment 의 rdb_uri 필요 (session_factory 미주입)")
        if deploy.object_uri is None:
            raise ValueError("scan 배치엔 deployment 의 object_uri 필요 (blob)")
        from modules.motion.adapters.pybullet import PybulletKinematics
        from modules.motor.contract import MotorKind
        from modules.scan.module import ScanRobotSpec
        from modules.scan.persistence.repository import ScanRepository

        scan_specs = {
            r.id: ScanRobotSpec(
                kinematics=PybulletKinematics(
                    _ROBOT_DIR / r.type / "urdf" / f"{r.type}.urdf"
                ),
                arm_specs=[m for m in r.motors if m.kind != MotorKind.GRIPPER],
            )
            for r in robots.values()
            if r.enabled and "rgbd" in r.capabilities
        }
        return {
            "repository": ScanRepository(session_factory),
            "object_store": _object_store(deploy.object_uri),
            "robots": scan_specs,
        }
    if name == "waypoint":
        # robot-agnostic (host 당 1) — per-robot config 0 (joints 캐시는 payload 키,
        # 이름 유일성은 DB (robot_id, name)). Motion 계약만 소비.
        if session_factory is None:
            raise ValueError(
                "waypoint 배치엔 deployment 의 rdb_uri 필요 (session_factory 미주입)"
            )
        from modules.waypoint.persistence.repository import WaypointRepository

        return {"repository": WaypointRepository(session_factory)}
    if name == "task":
        # host-level, robot-agnostic (§2.7) — task 정본은 tasks/ registry. 단 gripper
        # open/close raw 등 per-robot 물리값은 TaskRobotSpec 로 주입 (calibration/scan
        # robot spec 동형, motors.yaml SSOT — 추측 X, CLAUDE.md 안전수치 규칙).
        from modules.motor.contract import MotorKind
        from modules.task.spec import TaskRobotSpec

        task_specs: dict[str, TaskRobotSpec] = {}
        for r in robots.values():
            if not r.enabled:
                continue
            grip = next((m for m in r.motors if m.kind == MotorKind.GRIPPER), None)
            if grip is None:
                continue  # gripper 없는 robot 은 PnP 대상 X — spec 생략
            open_raw, close_raw = grip.limit_max, grip.limit_min
            # 잡힘 threshold = close 에서 range 15% 위 (보수 default) — 하드웨어 tuning
            # (§17.5 "정확도 = 집 하드웨어"). 이 값 미만 raw = 빈손 (VerifyGrasp).
            held = close_raw + round((open_raw - close_raw) * 0.15)
            task_specs[r.id] = TaskRobotSpec(
                gripper_open_raw=open_raw,
                gripper_close_raw=close_raw,
                gripper_index=r.motors.index(grip),
                gripper_held_threshold_raw=held,
            )
        return {"robots": task_specs}
    if name == "bridge":
        from modules.bridge.contract import BasePoseInfo, RobotInfo

        # 내부 config(RobotConfig) → frontend wire(RobotInfo). 변환은 apps 책임 (§9.1).
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
            )
            for r in robots.values()
        ]
        # robot_v2/ 경로 주입 — /robot static mount (URDF/mesh). 레이어링: 경로는
        # apps 가 앎, modules/bridge 는 받기만.
        deps: dict[str, Any] = {"robots": infos, "robot_dir": _ROBOT_DIR}
        if runtime is not None:
            # contract provider — closure 가 runtime 을 capture. request 시점엔
            # 이미 전 module add 됨 (add 순서상 bridge 가 먼저여도 안전). import 는
            # request 시점에 (bridge host 만 apps.contract_export 끌어옴, role 격리).
            def _contract_provider() -> dict:
                from apps.contract_export import build_contract_json

                return build_contract_json(runtime.contract_snapshot())

            def _graph_provider() -> dict:
                # contract graph viewer (contract_graph_viewer.md §1 — 개발자 도구,
                # §4 — unfiltered 전 module 의 전 계약). runtime.module_contracts()
                # 는 자기 프로세스에 로드된 module 만 봄 → 분산 배치 (PC 는
                # camera_decoded + bridge 만) 자리 다른 host 의 module (motor,
                # motion, camera) 이 그래프에 안 나옴. MODULE_REGISTRY 전체를
                # lazy introspect 해서 declared universe 를 그림.
                from apps.contract_export import build_static_contract_graph

                return build_static_contract_graph()

            deps["contract_provider"] = _contract_provider
            deps["graph_provider"] = _graph_provider
        return deps
    raise NotImplementedError(f"host-level resolve 미지원 module: {name!r}")


# ─── object store (blob) ────────────────────────────────────────


def _object_store(object_uri: str) -> Any:
    from infra.object_store.filesystem import FilesystemObjectStore

    if object_uri.startswith("file:///"):
        base = object_uri[len("file://") :]  # /path 유지
    elif object_uri.startswith("file://"):
        base = object_uri[len("file://") :]
    else:
        base = object_uri
    return FilesystemObjectStore(base)


# ─── driver / kinematics 선택 (lazy import) ─────────────────────


def _motor_driver(robot: RobotConfig, deploy: DeploymentConfig) -> Any:
    if deploy.driver_mode == DriverMode.MOCK:
        from modules.motor.drivers.mock import MockMotorBackend

        return MockMotorBackend(motors=robot.motors)  # layout SSOT = motors.yaml
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


def _detector_backend(deploy: DeploymentConfig) -> Any:
    if deploy.driver_mode == DriverMode.MOCK:
        from modules.detector.drivers.mock import MockDetectorBackend

        return MockDetectorBackend()
    # real — Grounding DINO. torch/transformers 는 이 branch 에서만 lazy import
    # (role 격리). 공유 load-lock + device_map="auto" 는 drivers/gdino.py.
    from modules.detector.drivers.gdino import GroundingDinoBackend

    return GroundingDinoBackend()


def _llm_backend(deploy: DeploymentConfig) -> Any:
    if deploy.driver_mode == DriverMode.MOCK:
        from modules.llm.drivers.mock import MockLlmBackend

        return MockLlmBackend()
    # real — Qwen2.5. torch/transformers 는 이 branch 에서만 lazy import (role 격리).
    # GDINO 와 공유 load-lock + device_map="auto" 는 drivers/qwen.py.
    from modules.llm.drivers.qwen import QwenBackend

    return QwenBackend()


def _camera_driver(robot: RobotConfig, deploy: DeploymentConfig) -> Any:
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
    # 순서 계약: arm 은 motors.yaml 의 prefix (gripper 등은 뒤). Motion 이
    # positions_raw[:dof] 로 arm 추출 + write_positions(arm raw) 하므로, gripper 가
    # 중간에 끼면 엉뚱한 모터 구동. boot 시 fail-fast 로 silent 오구동 차단.
    if robot.motors[: len(arm)] != arm:
        raise ValueError(
            f"robot {robot.id}: arm 모터가 motors.yaml 의 prefix 가 아님 "
            f"(gripper/rail 이 arm joint 앞/사이에 있음). Motion 의 positional "
            f"raw 매핑이 깨짐 — motors.yaml 순서 (arm joints 먼저, gripper 뒤) 확인."
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
    return {
        "kinematics": PybulletKinematics(urdf),
        "arm_specs": arm,
        "joint_max_velocity": [x.max_velocity for x in limits],
        "joint_max_acceleration": [x.max_acceleration for x in limits],
        "joint_max_jerk": [x.max_jerk for x in limits],
        "cartesian_max_velocity": robot.cartesian_limits.max_trans_vel,
        "cartesian_max_acceleration": robot.cartesian_limits.max_trans_acc,
        "cartesian_max_jerk": robot.cartesian_limits.max_trans_jerk,
    }
