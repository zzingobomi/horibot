from __future__ import annotations

from enum import StrEnum
from pathlib import Path

import yaml
from pydantic import BaseModel, Field


# ─── robots.yaml ────────────────────────────────────────────────


class BasePose(BaseModel):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw_deg: float = 0.0


class CameraSpec(BaseModel):
    id: str
    driver: str


class RobotConfig(BaseModel):
    id: str
    type: str
    base_pose: BasePose = Field(default_factory=BasePose)
    motor_driver: str
    motor_port: str | None = None
    camera: CameraSpec | None = None
    capabilities: list[str] = Field(default_factory=list)


def load_robots(path: Path | str) -> dict[str, RobotConfig]:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    robots: dict[str, RobotConfig] = {}
    for rid, body in (raw.get("robots") or {}).items():
        robots[rid] = RobotConfig(id=rid, **body)
    return robots


# ─── deployment yaml ────────────────────────────────────────────


class DriverMode(StrEnum):
    REAL = "real"
    MOCK = "mock"


class ModuleEntry(BaseModel):
    name: str
    robots: list[str] = Field(default_factory=list)


class DeploymentConfig(BaseModel):
    driver_mode: DriverMode = DriverMode.REAL
    zenoh: dict = Field(default_factory=dict)
    modules: list[ModuleEntry] = Field(default_factory=list)


def load_deployment(path: Path | str) -> DeploymentConfig:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    return DeploymentConfig.model_validate(raw)
