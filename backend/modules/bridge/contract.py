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


class TaskParamInfo(BaseModel):
    """task 실행 param 1개 — 각 task 모듈 RunRequest 모델에서 자동 파생
    (tasks/core/metadata.py). frontend 실행 폼이 이걸로 입력 UI 를 구성한다."""

    name: str
    type: str  # "str" | "int" | "float" | "bool"
    required: bool
    default: str = ""


class TaskInfo(BaseModel):
    """task registry 원소 — 참여 robot 을 task 가 선언 (frontend 는 이 목록으로
    통신 robot 을 정함. ambient default 로봇 없음). 단팔=1개, 협동=여러 개.

    run = 그 task 의 RUN 서비스 wire 키 (task 모듈마다 자기 네임스페이스)."""

    name: str
    robot_ids: list[str] = Field(default_factory=list)
    description: str = ""
    run: str = ""
    params: list[TaskParamInfo] = Field(default_factory=list)


class TasksResponse(BaseModel):
    tasks: list[TaskInfo]
