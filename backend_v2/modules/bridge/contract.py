from __future__ import annotations

from pydantic import BaseModel, Field


class BasePoseInfo(BaseModel):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw_deg: float = 0.0


class RobotInfo(BaseModel):
    id: str
    type: str
    base_pose: BasePoseInfo = Field(default_factory=BasePoseInfo)
    capabilities: list[str] = Field(default_factory=list)
    has_camera: bool = False


class RobotsResponse(BaseModel):
    robots: list[RobotInfo]
    default: str | None = None


class SystemMetrics(BaseModel):
    cpu_percent: float
    mem_percent: float
