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

import yaml

logger = logging.getLogger(__name__)

ROBOT_ROOT = Path(__file__).parents[2] / "robot"
ROBOTS_YAML_PATH = ROBOT_ROOT / "robots.yaml"

# 예약 top-level 이름 (§5.1) — robot_type / robot_id 로 사용 금지
RESERVED_TOP_LEVEL = frozenset(
    {"instances", "robots.yaml", "extrinsics", "workspace"}
)

# 예약 topic domain (§6.3) — robot_id 로 사용 금지
RESERVED_TOPIC_DOMAINS = frozenset(
    {"system", "task", "coord", "viz", "cameras"}
)


@dataclass(frozen=True)
class RobotConfig:
    """robot instance 1개의 모든 path / 설정.

    paths 는 `RobotRegistry._build_config()` 가 robot_type / robot_id 로 일관성 있게
    조립 — robots.yaml 에서 path 를 매 entry 마다 적지 않아도 됨.
    """

    robot_id: str
    robot_type: str
    enabled: bool
    host: str
    motor_backend: str  # "dynamixel" | "feetech"
    iksolver: str  # "pybullet" | "mujoco"

    # type-level paths — robot/<robot_type>/
    type_dir: Path
    urdf_path: Path
    type_motors_yaml: Path

    # instance-level paths — robot/instances/<robot_id>/
    instance_dir: Path
    instance_yaml: Path
    robot_poses_yaml: Path
    calibration_dir: Path
    scans_dir: Path
    meshes_dir: Path


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

        return RobotConfig(
            robot_id=robot_id,
            robot_type=robot_type,
            enabled=bool(entry.get("enabled", True)),
            host=str(entry.get("host", "dev")),
            motor_backend=str(entry.get("motor_backend", "dynamixel")),
            iksolver=str(entry.get("iksolver", "pybullet")),
            type_dir=type_dir,
            urdf_path=type_dir / "urdf" / f"{robot_type}.urdf",
            type_motors_yaml=type_dir / "motors.yaml",
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
        """N=1 single-robot 환경 편의 — robot 1개만 있을 때 그것 반환.

        N>=2 이면 RuntimeError. 명시적 robot_id 사용 강제.
        """
        if len(self._robots) != 1:
            raise RuntimeError(
                f"default() 는 N=1 일 때만 — 현재 {len(self._robots)} robot 등록. "
                "명시적 robot_id 로 get() 사용."
            )
        return next(iter(self._robots.values()))
