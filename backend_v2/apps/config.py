"""Config 모델 + 로더.

두 출처:
- **robot/ 트리** (top-level, v2 가 canonical 소유) — robot 데이터 SSOT.
  `robots.yaml`(registry) + `<type>/motors.yaml`(모터 레이아웃) +
  `instances/<id>/instance.yaml`(port/baud). 옛 RobotRegistry 코드는 안 씀 —
  v2 자체 loader (깨끗한 데이터만 재사용, 옛 아키텍처 X).
- **backend_v2/config/deployments/** — v2 배포 토폴로지 (host→module, driver_mode).

robots.yaml 의 calibration 도메인 파라미터(pose_recommend_strategy /
wrist_roll_motor_id / sag_joint_motor_ids)는 lean read 에서 무시 — Calibration
Module config 자리 (Step E 재배치).
"""

from __future__ import annotations

import sys
from enum import StrEnum
from pathlib import Path

import yaml
from pydantic import BaseModel, Field

from modules.motor.contract import MotorKind
from modules.motor.layout import MotorSpec

# v2 소유 robot 데이터 — top-level robot_v2/ (apps → backend_v2 → repo root).
# top-level 인 이유: robot 데이터(URDF/mesh)는 backend_v2 + frontend 가 공유하는
# project-domain 자산. 옛 top-level robot/ 은 폐기될 옛 backend 용 — v2 는 robot_v2/
# 를 소유하고 자유롭게 v2 아키텍처로 reshape (옛 backend 안 깨짐).
_ROBOT_DIR = Path(__file__).resolve().parents[2] / "robot_v2"


# ─── robot/ 트리 — robot 데이터 SSOT ────────────────────────────


class BasePose(BaseModel):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw_deg: float = 0.0


class RobotConfig(BaseModel):
    """robot 1개 — registry(robots.yaml) + 모터 레이아웃(motors.yaml) +
    instance(port/baud) 합본. v2 의 lean view (calib 파라미터 제외)."""

    id: str
    type: str
    enabled: bool = True
    default: bool = False
    capabilities: list[str] = Field(default_factory=list)
    motor_backend: str  # vendor — feetech / dynamixel
    camera_backend: str | None = None  # realsense / opencv
    base_pose: BasePose = Field(default_factory=BasePose)
    # instance-level (instances/<id>/instance.yaml, platform 별 port 해소)
    motor_port: str | None = None
    motor_baudrate: int | None = None
    # type-level (<type>/motors.yaml)
    motors: list[MotorSpec] = Field(default_factory=list)


def _resolve_port(instance_raw: dict) -> tuple[str | None, int | None]:
    motor = instance_raw.get("motor") or {}
    port = motor.get("port") or {}
    key = "windows" if sys.platform.startswith("win") else "linux"
    return port.get(key), motor.get("baudrate")


def _load_motors(robot_type: str, robot_dir: Path) -> list[MotorSpec]:
    raw = yaml.safe_load(
        (robot_dir / robot_type / "motors.yaml").read_text(encoding="utf-8")
    )
    specs: list[MotorSpec] = []
    for m in raw.get("motors") or []:
        limit = m.get("limit") or {}
        profile = m.get("profile") or {}
        kind = MotorKind(m["kind"]) if m.get("kind") else MotorKind.JOINT
        specs.append(
            MotorSpec(
                id=m["id"],
                name=m["name"],
                model=m["model"],
                kind=kind,
                home=m["home"],
                limit_min=limit["min"],
                limit_max=limit["max"],
                reverse=m.get("reverse", False),
                velocity_dps=profile.get("velocity_dps", 0.0),
                acceleration_dpss=profile.get("acceleration_dpss", 0.0),
            )
        )
    return specs


def load_robots(robot_dir: Path = _ROBOT_DIR) -> dict[str, RobotConfig]:
    raw = yaml.safe_load((robot_dir / "robots.yaml").read_text(encoding="utf-8"))
    robots: dict[str, RobotConfig] = {}
    for rid, body in (raw.get("robots") or {}).items():
        rtype = body["type"]
        port, baud = None, None
        inst_path = robot_dir / "instances" / rid / "instance.yaml"
        if inst_path.exists():
            inst_raw = yaml.safe_load(inst_path.read_text(encoding="utf-8"))
            port, baud = _resolve_port(inst_raw)
        bp = body.get("base_pose") or {}
        robots[rid] = RobotConfig(
            id=rid,
            type=rtype,
            enabled=body.get("enabled", True),
            default=body.get("default", False),
            capabilities=list(body.get("capabilities") or []),
            motor_backend=body["motor_backend"],
            camera_backend=body.get("camera_backend"),
            base_pose=BasePose(**bp),
            motor_port=port,
            motor_baudrate=baud,
            motors=_load_motors(rtype, robot_dir),
        )
    return robots


# ─── deployment yaml (backend_v2/config/deployments/) ───────────


class DriverMode(StrEnum):
    """driver 구현 선택 — vendor(robots.yaml) 와 분리. §2.4 driver subdir swap."""

    REAL = "real"
    MOCK = "mock"


class ModuleEntry(BaseModel):
    name: str
    # robots 비면 host-level singleton, 있으면 per-robot 인스턴스
    robots: list[str] = Field(default_factory=list)


class DeploymentConfig(BaseModel):
    driver_mode: DriverMode = DriverMode.REAL
    zenoh: dict = Field(default_factory=dict)
    modules: list[ModuleEntry] = Field(default_factory=list)


def load_deployment(path: Path | str) -> DeploymentConfig:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    return DeploymentConfig.model_validate(raw)
