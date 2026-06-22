from __future__ import annotations

import logging
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal, cast, get_args

import yaml

logger = logging.getLogger(__name__)

ROBOT_ROOT = Path(__file__).parents[3] / "robot"
ROBOTS_YAML_PATH = ROBOT_ROOT / "robots.yaml"


# ──────────────────────────────────────────────────────────────────────
# Schema / Constants — 허용 backend / capability literal
# ──────────────────────────────────────────────────────────────────────

MotorBackendName = Literal["dynamixel", "feetech"]
KinematicsBackendName = Literal["pybullet"]
CameraBackendName = Literal["realsense", "opencv"]
RobotCapability = Literal["move", "calibrate", "rgbd", "gamepad"]
PoseRecommendStrategyName = Literal["geometry", "joint_perturbation"]

_VALID_MOTOR_BACKENDS = frozenset(get_args(MotorBackendName))
_VALID_KINEMATICS_BACKENDS = frozenset(get_args(KinematicsBackendName))
_VALID_CAMERA_BACKENDS = frozenset(get_args(CameraBackendName))
_VALID_CAPABILITIES = frozenset(get_args(RobotCapability))
_VALID_POSE_RECOMMEND_STRATEGIES = frozenset(get_args(PoseRecommendStrategyName))


# ──────────────────────────────────────────────────────────────────────
# Config dataclasses — yaml → frozen RobotConfig
# ──────────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class BasePose:
    """World frame 기준 robot base 위치."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw_deg: float = 0.0


@dataclass(frozen=True)
class RobotConfig:
    """robot instance 1개의 모든 path / 설정."""

    robot_id: str
    robot_type: str
    enabled: bool
    is_default: bool
    base_pose: BasePose
    motor_backend: MotorBackendName
    kinematics_backend: KinematicsBackendName
    camera_backend: CameraBackendName
    capabilities: tuple[RobotCapability, ...]

    # type-level paths — robot/<robot_type>/
    type_dir: Path
    urdf_path: Path
    type_motors_yaml: Path
    type_motion_yaml: Path

    # instance-level paths — robot/instances/<robot_id>/
    instance_dir: Path
    instance_yaml: Path
    robot_poses_yaml: Path
    calibration_dir: Path
    scans_dir: Path
    meshes_dir: Path

    pose_recommend_strategy: PoseRecommendStrategyName = "geometry"
    wrist_roll_motor_id: int = 0
    sag_joint_motor_ids: tuple[int, ...] = (2, 3)


class RobotRegistry:
    """Robot 정의(robots.yaml)와 per-robot runtime 객체를 관리한다.

    Responsibilities:
      - robots.yaml 로드 및 RobotConfig 검증
      - robot 별 runtime singleton 객체 캐시
      - cache miss 시 backend 객체 lazy 생성

    Notes:
      - Process-wide singleton (`RobotRegistry()` 는 get-or-create)
    """

    # ─── Singleton lifecycle + yaml load ──────────────────────

    _instance: "RobotRegistry | None" = None
    _new_lock = threading.Lock()

    def __new__(cls) -> "RobotRegistry":
        if cls._instance is None:
            with cls._new_lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
                    cls._instance._initialized = False
        return cls._instance

    def __init__(self) -> None:
        if self._initialized:
            return
        self._initialized = True
        self._robots: dict[str, RobotConfig] = {}
        self._kinematics: dict[str, object] = {}
        self._motor_backends: dict[str, object] = {}
        self._camera_captures: dict[str, object] = {}
        self._motion_configs: dict[str, object] = {}
        self._fk_chains: dict[str, object] = {}
        # RLock 로 factory 내에서 다른 factory 호출 가능.
        self._factory_lock = threading.RLock()
        self._load()

    def _load(self) -> None:
        if not ROBOTS_YAML_PATH.exists():
            raise FileNotFoundError(f"robots.yaml 없음: {ROBOTS_YAML_PATH}. ")

        with open(ROBOTS_YAML_PATH, "r", encoding="utf-8") as f:
            raw = yaml.safe_load(f)

        if not isinstance(raw, dict):
            raise ValueError(f"robots.yaml: top-level 이 dict 아님 ({type(raw)})")

        robots_section = raw.get("robots", {})
        if not isinstance(robots_section, dict) or not robots_section:
            raise ValueError(
                "robots.yaml: 'robots' 가 비어있거나 dict 아님 — "
                "최소 1개 robot entry 필요"
            )

        for robot_id, entry in robots_section.items():
            cfg = self._build_config(str(robot_id), entry)
            self._robots[str(robot_id)] = cfg

        explicit_defaults = [c.robot_id for c in self._robots.values() if c.is_default]
        if len(explicit_defaults) > 1:
            raise ValueError(
                f"robots.yaml: 'default: true' 가 두 개 이상 — {explicit_defaults}. "
                "정확히 한 robot 에만 명시 (또는 안 적으면 첫 enabled robot 자동)."
            )

        logger.info(
            "RobotRegistry load 완료: %d robot — %s",
            len(self._robots),
            list(self._robots.keys()),
        )

    @staticmethod
    def _build_config(robot_id: str, entry: dict) -> RobotConfig:
        robot_type = str(entry["type"])
        type_dir = ROBOT_ROOT / robot_type
        instance_dir = ROBOT_ROOT / "instances" / robot_id

        motor_backend = str(entry.get("motor_backend", "dynamixel"))
        if motor_backend not in _VALID_MOTOR_BACKENDS:
            raise ValueError(
                f"robot '{robot_id}' motor_backend={motor_backend!r} 미지원. 가능: {sorted(_VALID_MOTOR_BACKENDS)}"
            )
        kinematics_backend = str(entry.get("kinematics_backend", "pybullet"))
        if kinematics_backend not in _VALID_KINEMATICS_BACKENDS:
            raise ValueError(
                f"robot '{robot_id}' kinematics_backend={kinematics_backend!r} 미지원. 가능: {sorted(_VALID_KINEMATICS_BACKENDS)}"
            )
        camera_backend = str(entry.get("camera_backend", "realsense"))
        if camera_backend not in _VALID_CAMERA_BACKENDS:
            raise ValueError(
                f"robot '{robot_id}' camera_backend={camera_backend!r} 미지원. 가능: {sorted(_VALID_CAMERA_BACKENDS)}"
            )

        pose_recommend_strategy = str(entry.get("pose_recommend_strategy", "geometry"))
        if pose_recommend_strategy not in _VALID_POSE_RECOMMEND_STRATEGIES:
            raise ValueError(
                f"robot '{robot_id}' pose_recommend_strategy="
                f"{pose_recommend_strategy!r} 미지원. "
                f"가능: {sorted(_VALID_POSE_RECOMMEND_STRATEGIES)}"
            )

        wrist_roll_raw = entry.get("wrist_roll_motor_id")
        if wrist_roll_raw is None:
            raise ValueError(
                f"robot '{robot_id}' wrist_roll_motor_id 필수 "
                f"(1-based motor ID, motor 라벨과 일치). OMX-F=5, SO-101=6."
            )
        if not isinstance(wrist_roll_raw, int) or wrist_roll_raw < 1:
            raise ValueError(
                f"robot '{robot_id}' wrist_roll_motor_id="
                f"{wrist_roll_raw!r} 는 1 이상 int 이어야 함 (1-based)."
            )
        wrist_roll_motor_id = int(wrist_roll_raw)

        sag_raw = entry.get("sag_joint_motor_ids", [2, 3])
        if not isinstance(sag_raw, list) or not all(
            isinstance(m, int) and m >= 1 for m in sag_raw
        ):
            raise ValueError(
                f"robot '{robot_id}' sag_joint_motor_ids={sag_raw!r} 는 "
                f"1-based int list 이어야 함 (예: [2, 3])."
            )
        sag_joint_motor_ids = tuple(int(m) for m in sag_raw)

        caps_raw = entry.get("capabilities", []) or []
        if not isinstance(caps_raw, list):
            raise ValueError(
                f"robot '{robot_id}' capabilities 가 list 아님 "
                f"({type(caps_raw).__name__}). 예: capabilities: [move, calibrate, rgbd]"
            )
        caps: list[RobotCapability] = []
        for c in caps_raw:
            cs = str(c)
            if cs not in _VALID_CAPABILITIES:
                raise ValueError(
                    f"robot '{robot_id}' 알 수 없는 capability={cs!r}. "
                    f"가능: {sorted(_VALID_CAPABILITIES)}"
                )
            caps.append(cast(RobotCapability, cs))

        pose_raw = entry.get("base_pose", {}) or {}
        if not isinstance(pose_raw, dict):
            raise ValueError(
                f"robot '{robot_id}' base_pose 가 dict 아님 ({type(pose_raw).__name__})"
            )
        base_pose = BasePose(
            x=float(pose_raw.get("x", 0.0)),
            y=float(pose_raw.get("y", 0.0)),
            z=float(pose_raw.get("z", 0.0)),
            yaw_deg=float(pose_raw.get("yaw_deg", 0.0)),
        )

        return RobotConfig(
            robot_id=robot_id,
            robot_type=robot_type,
            enabled=bool(entry.get("enabled", True)),
            is_default=bool(entry.get("default", False)),
            base_pose=base_pose,
            motor_backend=cast(MotorBackendName, motor_backend),
            kinematics_backend=cast(KinematicsBackendName, kinematics_backend),
            camera_backend=cast(CameraBackendName, camera_backend),
            capabilities=tuple(caps),
            pose_recommend_strategy=cast(
                PoseRecommendStrategyName, pose_recommend_strategy
            ),
            wrist_roll_motor_id=wrist_roll_motor_id,
            sag_joint_motor_ids=sag_joint_motor_ids,
            type_dir=type_dir,
            urdf_path=type_dir / "urdf" / f"{robot_type}.urdf",
            type_motors_yaml=type_dir / "motors.yaml",
            type_motion_yaml=type_dir / "motion.yaml",
            instance_dir=instance_dir,
            instance_yaml=instance_dir / "instance.yaml",
            robot_poses_yaml=instance_dir / "robot_poses.yaml",
            calibration_dir=instance_dir / "calibration",
            scans_dir=instance_dir / "scans",
            meshes_dir=instance_dir / "meshes",
        )

    # ─── Config Registry — robot_id → RobotConfig lookup ──────

    def get(self, robot_id: str) -> RobotConfig:
        try:
            return self._robots[robot_id]
        except KeyError:
            raise KeyError(
                f"robot_id '{robot_id}' 없음. 등록된 robot: {list(self._robots.keys())}"
            ) from None

    def list_robots(self) -> list[str]:
        return list(self._robots.keys())

    def enabled_robots(self) -> list[RobotConfig]:
        return [cfg for cfg in self._robots.values() if cfg.enabled]

    def default_robot_id(self) -> str:
        return self.default().robot_id

    def default(self) -> RobotConfig:
        explicit = [c for c in self._robots.values() if c.is_default]
        if len(explicit) == 1:
            return explicit[0]
        enabled = self.enabled_robots()
        if not enabled:
            raise RuntimeError(
                "default robot 없음 — robots.yaml 에 enabled=true 인 robot 이 0개."
            )

        # default robot 지정 안 하면 첫 enabled robot 자동 선택
        return enabled[0]

    # ─── Runtime Factories — per-robot 인스턴스 생성 + cache lookup ──────

    def _resolve(self, robot_id: str | None) -> str:
        return robot_id if robot_id is not None else self.default_robot_id()

    def _get_or_build(self, cache: dict, robot_id: str | None, builder) -> object:
        rid = self._resolve(robot_id)
        with self._factory_lock:
            if rid not in cache:
                cache[rid] = builder(rid)
            return cache[rid]

    def get_kinematics(self, robot_id: str | None = None):
        return self._get_or_build(self._kinematics, robot_id, self._build_kinematics)

    def _build_kinematics(self, robot_id: str):
        from core.coords.link_coordinates import LinkCoordinates
        from core.coords.sag_coordinates import SagCoordinates
        from modules.kinematics.adapters.pybullet_kinematics import PybulletKinematics
        from modules.kinematics.adapters.sag_corrected import SagCorrectedKinematics

        # pybullet 의 경우 SagCorrectedKinematics wrapper 씌운 형태로 반환.
        cfg = self.get(robot_id)
        if cfg.kinematics_backend == "pybullet":
            inner = PybulletKinematics(cfg.urdf_path)
            return SagCorrectedKinematics(
                inner,
                LinkCoordinates(),
                SagCoordinates(),
                self.get_fk_chain(robot_id),
                sag_joint_motor_ids=cfg.sag_joint_motor_ids,
            )
        raise ValueError(
            f"unknown kinematics_backend: {cfg.kinematics_backend!r} (robot_id={robot_id})"
        )

    def get_fk_chain(self, robot_id: str | None = None):
        return self._get_or_build(self._fk_chains, robot_id, self._build_fk_chain)

    def _build_fk_chain(self, robot_id: str):
        from modules.kinematics.fk_chain import FkChain
        from modules.motor.motor_config import load_motor_layout

        cfg = self.get(robot_id)
        layout = load_motor_layout(robot_id)
        arm_joint_names = [m.name for m in layout.arm]
        return FkChain(cfg.urdf_path, arm_joint_names)

    def get_motor_backend(self, robot_id: str | None = None):
        return self._get_or_build(
            self._motor_backends, robot_id, self._build_motor_backend
        )

    def _build_motor_backend(self, robot_id: str):
        from modules.motor.motor_config import load_motor_layout

        cfg = self.get(robot_id)
        layout = load_motor_layout(robot_id)
        if cfg.motor_backend == "dynamixel":
            from modules.motor.adapters.dynamixel_backend import DynamixelBackend

            return DynamixelBackend(layout.port.get(), layout.motors)
        if cfg.motor_backend == "feetech":
            from modules.motor.adapters.feetech_backend import FeetechBackend

            return FeetechBackend(layout.port.get(), layout.motors)
        raise ValueError(
            f"unknown motor_backend: {cfg.motor_backend!r} (robot_id={robot_id})"
        )

    def get_motion_config(self, robot_id: str | None = None):
        return self._get_or_build(
            self._motion_configs, robot_id, self._build_motion_config
        )

    def _build_motion_config(self, robot_id: str):
        from modules.kinematics.motion_config import load_motion_config

        cfg = self.get(robot_id)
        return load_motion_config(cfg.type_motion_yaml)

    # TODO: camera backend 설계 이후 return Any 수정 필요
    # return 이 Any 인 이유: camera_node 는 현재 RealsenseCapture 의 legacy method
    # (`read` / `read_aligned` / `width` / ...) 를 사용
    def get_camera_capture(self, robot_id: str | None = None) -> Any:
        return self._get_or_build(
            self._camera_captures, robot_id, self._build_camera_capture
        )

    def _build_camera_capture(self, robot_id: str):
        cfg = self.get(robot_id)
        if cfg.camera_backend == "realsense":
            from modules.camera.adapters.realsense_capture import RealsenseCapture

            return RealsenseCapture()
        if cfg.camera_backend == "opencv":
            raise NotImplementedError(
                f"opencv CameraCapture — NotImplementedError (robot_id={robot_id})"
            )
        raise ValueError(
            f"unknown camera_backend: {cfg.camera_backend!r} (robot_id={robot_id})"
        )
