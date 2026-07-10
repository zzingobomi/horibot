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


class TaskInfo(BaseModel):
    """task registry 원소 — 참여 robot 을 task 가 선언 (frontend 는 이 목록으로
    통신 robot 을 정함. ambient default 로봇 없음). 단팔=1개, 협동=여러 개,
    빈 리스트=robot 무관 task."""

    name: str
    robot_ids: list[str] = Field(default_factory=list)


class TasksResponse(BaseModel):
    tasks: list[TaskInfo]
