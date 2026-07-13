from __future__ import annotations

from enum import StrEnum

from framework.contract.model import StrictModel
from modules.tasks.core.contract import TraceEntry


class PickAndPlace:
    class Service(StrEnum):
        RUN = "srv/pick_and_place/run"
        STOP = "srv/pick_and_place/stop"
        PAUSE = "srv/pick_and_place/pause"
        RESUME = "srv/pick_and_place/resume"
        STEP_ONCE = "srv/pick_and_place/step_once"
        RUN_TO = "srv/pick_and_place/run_to"
        TOGGLE_BREAKPOINT = "srv/pick_and_place/toggle_breakpoint"
        LIST_ROBOTS = "srv/pick_and_place/list_robots"
        PREVIEW = "srv/pick_and_place/preview"

    class Stream(StrEnum):
        STATE = "stream/pick_and_place/{robot_id}/state"
        TRACE = "stream/pick_and_place/{robot_id}/trace"
        MARKERS = "stream/pick_and_place/{robot_id}/markers"


class RunRequest(StrictModel):
    pick_object: str
    place_object: str = ""


class ListRobotsRequest(StrictModel):
    pass


class ListRobotsResponse(StrictModel):
    """task 참여 robot 명부 — 바인딩 SSOT(모듈 TASK_ROBOTS)를 계약으로 노출.

    frontend 는 이 목록으로 robot-scoped 스트림 키의 `{robot_id}` 를 채운다
    (task 패널은 robot 을 *고르지* 않고, task 가 *알려주는* 사실을 쓴다)."""

    robot_ids: list[str]


class PreviewRequest(StrictModel):
    """미리보기 요청 — 실행 전 전체 step 목록만 dry-run 으로 수집 (모션 0).

    항상 놓기 포함 최대 경로를 보여준다 (전체 단계 미리보기) — 실제 파라미터는
    canned dry-run 이라 무의미."""


class PreviewResponse(StrictModel):
    """dry-run 으로 수집한 전체 step 목록 (진입 순서 + depth). status 는 아직 실행
    안 됨을 뜻하는 running placeholder — 프론트가 회색(미실행)으로 렌더."""

    steps: list[TraceEntry]


class TaskMarker(StrictModel):
    label: str
    position: tuple[float, float, float]


class TaskMarkers(StrictModel):
    robot_id: str
    seq: int
    timestamp_unix: float
    markers: list[TaskMarker] = []
