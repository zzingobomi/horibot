"""robots.yaml 의 single source of truth + per-robot path resolution.

multi_robot_architecture.md §4 (Robot identity 모델) / §5 (디렉토리 구조) 참조.

핵심 책임:
- robot/robots.yaml 을 부팅 시 1회 load → 메모리 캐시
- robot_id → RobotConfig (모든 path / 설정) 매핑
- robot_id validation (reserved name 충돌 차단)

Phase 1 에서는 모든 caller 가 `RobotRegistry().default()` 로 single robot 가져옴 —
robot_id 차원 도입은 후속 todo (`JointStateCache` / `Coordinates` 등의 dict[robot_id]
화) 에서.
"""

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

# 예약 top-level 이름 (§5.1) — robot_type / robot_id 로 사용 금지
RESERVED_TOP_LEVEL = frozenset(
    {"instances", "robots.yaml", "extrinsics", "workspace"}
)

# 예약 topic domain (§6.3) — robot_id 로 사용 금지
RESERVED_TOPIC_DOMAINS = frozenset(
    {"system", "task", "coord", "viz", "cameras"}
)

# Valid backend / kinematics 이름 — yaml typo 부팅 시 fail-fast.
# 새 backend 추가 시 여기에 + factory 분기 추가 (pyright 가 두 곳 동기화 검사).
MotorBackendName = Literal["dynamixel", "feetech"]
KinematicsBackendName = Literal["pybullet", "mujoco"]
CameraBackendName = Literal["realsense", "opencv", "mujoco"]

# Robot mode sub-route 의 sidebar / route enablement 결정 (frontend Phase 2 UX —
# multi_robot_phase2_frontend.md). camera 가 depth 인지 RGB 인지에 따라 scan 가능
# 여부가 달라짐 — robots.yaml capabilities 가 SSOT.
RobotCapability = Literal["move", "calibrate", "scan"]

_VALID_MOTOR_BACKENDS = frozenset(get_args(MotorBackendName))
_VALID_KINEMATICS_BACKENDS = frozenset(get_args(KinematicsBackendName))
_VALID_CAMERA_BACKENDS = frozenset(get_args(CameraBackendName))
_VALID_CAPABILITIES = frozenset(get_args(RobotCapability))


@dataclass(frozen=True)
class BasePose:
    """World frame 기준 robot base 위치 (multi_robot_phase2_frontend.md §2).

    frontend WorldScene 이 두 URDF 동시 마운트 시 겹치지 않게 분리하는 자리.
    실 hardware 도착 시 robot-to-robot extrinsic 캘리브레이션 결과로 update.
    """

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw_deg: float = 0.0


@dataclass(frozen=True)
class RobotConfig:
    """robot instance 1개의 모든 path / 설정.

    paths 는 `RobotRegistry._build_config()` 가 robot_type / robot_id 로 일관성 있게
    조립 — robots.yaml 에서 path 를 매 entry 마다 적지 않아도 됨.
    """

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

    # Hand-Eye 캘 추천 자세 전략. 5DOF = "joint_perturbation", 6DOF = "geometry".
    # robots.yaml 의 `pose_recommend_strategy` SSOT. None 이면 default = "geometry".
    pose_recommend_strategy: str = "geometry"

    # Hand-Eye observability metric 의 wrist roll 모터 ID (1-based, motor 라벨과 일치).
    # robots.yaml SSOT — robot 별 wrist roll 위치가 달라서 (OMX-F=5, SO-101=6).
    # observability.analyze_pose_data 는 array index (0-based) 를 받으니 caller 가
    # `motor_id - 1` 변환해서 주입 ([calibration_node._publish_observability_state]).
    wrist_roll_motor_id: int = 0


class RobotRegistry:
    """robots.yaml 싱글톤. 부팅 시 1회 load + validation.

    분산 환경에서 모든 머신이 같은 git commit 의 robots.yaml 을 봄 — Zenoh
    pub/sub 전파 없음.
    """

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
        self._kinematics: dict[str, object] = {}  # Kinematics — lazy import 회피
        self._motor_backends: dict[str, object] = {}  # MotorBackend
        self._camera_captures: dict[str, object] = {}  # CameraCapture
        self._motion_configs: dict[str, object] = {}  # MotionConfig
        self._fk_chains: dict[str, object] = {}  # FkChain (BA + sag hot path)
        # RLock — `_build_kinematics` 가 안에서 `get_fk_chain` 호출 같이 factory
        # 끼리 의존하는 자리에서 reentrant 필요 (non-reentrant Lock 이면 deadlock).
        self._factory_lock = threading.RLock()
        self._load()

    def _load(self) -> None:
        if not ROBOTS_YAML_PATH.exists():
            raise FileNotFoundError(
                f"robots.yaml 없음: {ROBOTS_YAML_PATH}. "
                "multi_robot_architecture.md §4.3 참조."
            )

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
            self._validate_robot_id(robot_id)
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
    def _validate_robot_id(robot_id: str) -> None:
        if robot_id in RESERVED_TOP_LEVEL:
            raise ValueError(
                f"robot_id '{robot_id}' 는 reserved top-level name 과 충돌. "
                f"금지 목록: {sorted(RESERVED_TOP_LEVEL)}"
            )
        if robot_id in RESERVED_TOPIC_DOMAINS:
            raise ValueError(
                f"robot_id '{robot_id}' 는 reserved topic domain 과 충돌. "
                f"금지 목록: {sorted(RESERVED_TOPIC_DOMAINS)}"
            )

    @staticmethod
    def _build_config(robot_id: str, entry: dict) -> RobotConfig:
        robot_type = str(entry["type"])
        type_dir = ROBOT_ROOT / robot_type
        instance_dir = ROBOT_ROOT / "instances" / robot_id

        motor_backend = str(entry.get("motor_backend", "dynamixel"))
        if motor_backend not in _VALID_MOTOR_BACKENDS:
            raise ValueError(f"robot '{robot_id}' motor_backend={motor_backend!r} 미지원. 가능: {sorted(_VALID_MOTOR_BACKENDS)}")
        kinematics_backend = str(entry.get("kinematics_backend", "pybullet"))
        if kinematics_backend not in _VALID_KINEMATICS_BACKENDS:
            raise ValueError(f"robot '{robot_id}' kinematics_backend={kinematics_backend!r} 미지원. 가능: {sorted(_VALID_KINEMATICS_BACKENDS)}")
        camera_backend = str(entry.get("camera_backend", "realsense"))
        if camera_backend not in _VALID_CAMERA_BACKENDS:
            raise ValueError(f"robot '{robot_id}' camera_backend={camera_backend!r} 미지원. 가능: {sorted(_VALID_CAMERA_BACKENDS)}")

        pose_recommend_strategy = str(entry.get("pose_recommend_strategy", "geometry"))
        if pose_recommend_strategy not in ("geometry", "joint_perturbation"):
            raise ValueError(
                f"robot '{robot_id}' pose_recommend_strategy="
                f"{pose_recommend_strategy!r} 미지원. "
                f"가능: 'geometry' | 'joint_perturbation'"
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

        caps_raw = entry.get("capabilities", []) or []
        if not isinstance(caps_raw, list):
            raise ValueError(
                f"robot '{robot_id}' capabilities 가 list 아님 "
                f"({type(caps_raw).__name__}). 예: capabilities: [move, calibrate, scan]"
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
            pose_recommend_strategy=pose_recommend_strategy,
            wrist_roll_motor_id=wrist_roll_motor_id,
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

    def get(self, robot_id: str) -> RobotConfig:
        try:
            return self._robots[robot_id]
        except KeyError:
            raise KeyError(
                f"robot_id '{robot_id}' 없음. 등록된 robot: "
                f"{list(self._robots.keys())}"
            ) from None

    def list_robots(self) -> list[str]:
        return list(self._robots.keys())

    def enabled_robots(self) -> list[RobotConfig]:
        """`enabled: true` 인 robot 만 — Coordinates / Cache 가 load 대상 결정 시."""
        return [cfg for cfg in self._robots.values() if cfg.enabled]

    def default_robot_id(self) -> str:
        """N=1 편의 — default() 의 robot_id 만 반환."""
        return self.default().robot_id

    def default(self) -> RobotConfig:
        """string-entry fallback 용 default robot.

        정책:
          - `default: true` 명시된 robot 이 정확히 1 → 그것
          - 명시 0 → 첫 enabled robot (robots.yaml entry 순서)
          - 명시 2 이상 → ValueError (이미 _load 가 부팅 시 잡음)

        호출 시점에 enabled 0 이면 RuntimeError. host config 의 string entry
        가 의미를 가지려면 적어도 한 robot 이 enabled 여야 함.
        """
        explicit = [c for c in self._robots.values() if c.is_default]
        if len(explicit) == 1:
            return explicit[0]
        enabled = self.enabled_robots()
        if not enabled:
            raise RuntimeError(
                "default robot 없음 — robots.yaml 에 enabled=true 인 robot 이 0개."
            )
        return enabled[0]

    # ─── Factory methods (per-robot 인스턴스 캐시) ───────────────

    def _resolve(self, robot_id: str | None) -> str:
        return robot_id if robot_id is not None else self.default_robot_id()

    def _get_or_build(self, cache: dict, robot_id: str | None, builder) -> object:
        """per-robot 인스턴스 캐시 lookup + miss 시 builder 호출. lock 보호."""
        rid = self._resolve(robot_id)
        with self._factory_lock:
            if rid not in cache:
                cache[rid] = builder(rid)
            return cache[rid]

    def get_kinematics(self, robot_id: str | None = None):
        """cfg.kinematics_backend = "pybullet" → SagCorrectedKinematics(PybulletKinematics(urdf), ...) / "mujoco" 미구현."""
        return self._get_or_build(self._kinematics, robot_id, self._build_kinematics)

    def _build_kinematics(self, robot_id: str):
        # Lazy import — RobotRegistry 가 kinematics 모듈에 의존 X
        from core.coords.link_coordinates import LinkCoordinates
        from core.coords.sag_coordinates import SagCoordinates
        from modules.kinematics.adapters.pybullet_kinematics import PybulletKinematics
        from modules.kinematics.adapters.sag_corrected import SagCorrectedKinematics

        cfg = self.get(robot_id)
        if cfg.kinematics_backend == "pybullet":
            inner = PybulletKinematics(cfg.urdf_path)
            return SagCorrectedKinematics(
                inner,
                LinkCoordinates(),
                SagCoordinates(),
                self.get_fk_chain(robot_id),
            )
        if cfg.kinematics_backend == "mujoco":
            raise NotImplementedError(
                f"mujoco Kinematics — Phase 2+ (robot_id={robot_id})"
            )
        raise ValueError(f"unknown kinematics_backend: {cfg.kinematics_backend!r} (robot_id={robot_id})")

    def get_fk_chain(self, robot_id: str | None = None):
        """FkChain — BA / sag 의 numpy FK chain (link_offset variable 자리).

        PybulletKinematics 와 별개 — BA hot path 가 PyBullet 의 정적 limit 우회.
        per-robot 캐시. arm motor names = `MotorLayout.arm()` 의 `.name`.
        """
        return self._get_or_build(self._fk_chains, robot_id, self._build_fk_chain)

    def _build_fk_chain(self, robot_id: str):
        from modules.kinematics.fk_chain import FkChain
        from modules.motor.motor_config import load_motor_layout

        cfg = self.get(robot_id)
        layout = load_motor_layout(robot_id)
        arm_joint_names = [m.name for m in layout.arm]
        return FkChain(cfg.urdf_path, arm_joint_names)

    def get_motor_backend(self, robot_id: str | None = None):
        """cfg.motor_backend = "dynamixel" → DynamixelBackend / "feetech" → FeetechBackend."""
        return self._get_or_build(self._motor_backends, robot_id, self._build_motor_backend)

    def _build_motor_backend(self, robot_id: str):
        # Lazy import — Pi 별 SDK 그룹 (pi-motor) 의 dynamixel-sdk / feetech-servo-sdk
        # 가 동작 시점에만 import. host config 에 robot 하나만 enabled 면 다른 SDK 미설치 OK.
        from modules.motor.motor_config import load_motor_layout

        cfg = self.get(robot_id)
        layout = load_motor_layout(robot_id)
        if cfg.motor_backend == "dynamixel":
            from modules.motor.adapters.dynamixel_backend import DynamixelBackend
            return DynamixelBackend(layout.port.get(), layout.motors)
        if cfg.motor_backend == "feetech":
            # so101_6dof (STS3215/3250 + scservo_sdk). so101_6dof_plan §6-1 / §6-6.
            from modules.motor.adapters.feetech_backend import FeetechBackend
            return FeetechBackend(layout.port.get(), layout.motors)
        raise ValueError(
            f"unknown motor_backend: {cfg.motor_backend!r} (robot_id={robot_id})"
        )

    def get_motion_config(self, robot_id: str | None = None):
        """robot/<type>/motion.yaml 의 MotionConfig — Ruckig 한계 SSOT."""
        return self._get_or_build(self._motion_configs, robot_id, self._build_motion_config)

    def _build_motion_config(self, robot_id: str):
        from modules.kinematics.motion_config import load_motion_config

        cfg = self.get(robot_id)
        return load_motion_config(cfg.type_motion_yaml)

    def get_camera_capture(self, robot_id: str | None = None) -> Any:
        """cfg.camera_backend = "realsense" → RealsenseCapture() / "opencv" / "mujoco" 미구현.

        return 이 Any 인 이유: camera_node 는 현재 RealsenseCapture 의 legacy method
        (`read` / `read_aligned` / `width` / ...) 를 사용 — CameraCapture Protocol 에
        없음. camera_node Protocol 마이그레이션 완료 후 `-> CameraCapture` 로 좁힘.
        """
        return self._get_or_build(self._camera_captures, robot_id, self._build_camera_capture)

    def _build_camera_capture(self, robot_id: str):
        cfg = self.get(robot_id)
        if cfg.camera_backend == "realsense":
            # Lazy import — pyrealsense2 는 camera host 에서만 설치됨
            from modules.camera.adapters.realsense_capture import RealsenseCapture

            return RealsenseCapture()
        if cfg.camera_backend == "opencv":
            raise NotImplementedError(
                f"opencv CameraCapture — Phase 2 (omx_f UVC, "
                f"distributed_topology.md §1) (robot_id={robot_id})"
            )
        if cfg.camera_backend == "mujoco":
            raise NotImplementedError(
                f"mujoco CameraCapture — Track C sim (random_palletizing.md) "
                f"(robot_id={robot_id})"
            )
        raise ValueError(
            f"unknown camera_backend: {cfg.camera_backend!r} (robot_id={robot_id})"
        )
