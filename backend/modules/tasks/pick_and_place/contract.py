from __future__ import annotations

from enum import StrEnum

from framework.contract.model import StrictModel


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


class TaskMarker(StrictModel):
    label: str
    position: tuple[float, float, float]


class TaskMarkers(StrictModel):
    robot_id: str
    seq: int
    timestamp_unix: float
    markers: list[TaskMarker] = []
