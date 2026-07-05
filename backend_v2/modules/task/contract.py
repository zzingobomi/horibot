"""Task domain — public contract surface (task-first PnP, backend_v2.md §17).

Task = Orchestration layer (§16.1 Layer 3) — Day-1 primitive 를 async 함수로 엮는다.
robot-agnostic (host당 1, §2.7) — task 가 대상 robot 을 포함 (run req.robot_id).
디버거(pause/step/breakpoint/run_to) = dev 안전장치 (§17.1.4 "지금 짓는다") — 실
하드웨어 burn 절감. 스트림 3종 = 옛 TASK_STATE/TREE/STEP_RESULT 포팅 (§17.4).
"""

from __future__ import annotations

from enum import StrEnum
from typing import Any

from pydantic import BaseModel


class TaskStatus(StrEnum):
    IDLE = "idle"
    RUNNING = "running"
    PAUSED = "paused"
    SUCCESS = "success"
    FAILED = "failed"
    STOPPED = "stopped"


# step 별 실행 상태 (TaskState.step_statuses 값). frontend 트리 색상용.
STEP_PENDING = "pending"
STEP_RUNNING = "running"
STEP_COMPLETED = "completed"
STEP_FAILED = "failed"


class Task:
    class Service(StrEnum):
        # robot-agnostic (host당 1) — 대상 robot 은 req.robot_id (§2.7).
        RUN = "srv/task/run"
        PREVIEW = "srv/task/preview"  # 실행 없이 tree 만 publish
        STOP = "srv/task/stop"
        PAUSE = "srv/task/pause"
        RESUME = "srv/task/resume"
        STEP_ONCE = "srv/task/step_once"
        RUN_TO = "srv/task/run_to"
        TOGGLE_BREAKPOINT = "srv/task/toggle_breakpoint"

    class Stream(StrEnum):
        # robot-scoped 키 — payload robot_id 로 framework 라우팅 (host-level 발행,
        # scan BUILD_PROGRESS 동형).
        STATE = "stream/task/{robot_id}/state"
        TREE = "stream/task/{robot_id}/tree"
        STEP_RESULT = "stream/task/{robot_id}/step_result"


# ─── request / response ─────────────────────────────────────────────


class RunRequest(BaseModel):
    robot_id: str
    task_name: str
    params: dict[str, str] = {}  # task factory 인자 (예: pick_object/place_object)


class RunResponse(BaseModel):
    accepted: bool
    message: str = ""


class PreviewRequest(BaseModel):
    robot_id: str
    task_name: str
    params: dict[str, str] = {}


class PreviewResponse(BaseModel):
    ok: bool
    message: str = ""


class TaskControlRequest(BaseModel):
    """stop / pause / resume / step_once 공통 — 대상 robot 만."""

    robot_id: str


class TaskControlResponse(BaseModel):
    ok: bool


class RunToRequest(BaseModel):
    robot_id: str
    step_id: str  # 이 step 직전까지 진행 후 pause (VSCode 'run to cursor')


class ToggleBreakpointRequest(BaseModel):
    robot_id: str
    step_id: str


# ─── stream payload (robot_id + seq + timestamp_unix, §16.6) ─────────


class TaskState(BaseModel):
    robot_id: str
    seq: int
    timestamp_unix: float
    status: TaskStatus
    task_name: str = ""
    current_step: int = 0
    total_steps: int = 0
    current_label: str = ""
    current_step_id: str = ""
    error: str | None = None
    step_statuses: dict[str, str] = {}
    breakpoints: list[str] = []


class TaskTree(BaseModel):
    robot_id: str
    seq: int
    timestamp_unix: float
    task_name: str = ""
    description: str = ""
    steps: list[dict] = []  # step_to_dict 재귀 트리 (frontend 가 동적 walk)


class TaskStepResult(BaseModel):
    robot_id: str
    seq: int
    timestamp_unix: float
    step_id: str
    type: str  # "Detection" / "Position3" / "None" — TaskResultLayer dispatch
    value: Any = None  # BaseModel model_dump(dict) / None / scalar (type 로 해석)
