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


class SystemMetrics(BaseModel):
    cpu_percent: float
    mem_percent: float


# GET /tasks 는 2026-07-13 삭제 — task 의 정보 채널은 계약이 유일 (frontend 는
# gen:types 로 task 의 서비스/스트림 키를 정적으로 알고, robot 바인딩/표시 문구는
# task 전용 페이지가 소유 — "robot 은 패널이 소유" 원칙).
