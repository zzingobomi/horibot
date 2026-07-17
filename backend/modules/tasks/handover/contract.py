"""handover task 계약 — omx 가 집어 든 물체를 so101 이 받아 상자에 적치.

pick_and_place 와 같은 표준형 (task.md §3 — 새 task = 복제): 트리거/조작판/
진행 스트림 전부 손 선언, 등록 의식 0 (task 정보 채널 = 계약뿐). frontend
노출은 아직 안 함 (FRONTEND_EXPOSED 미등록 — 전용 페이지는 TODO, 터미널
실행은 scripts/run_task.py 로 가능).

⚠ 2026-07-17 신설 — **실물 미검증** (사용자 지시: 코드만, 테스트는 mock 레벨).
"""

from __future__ import annotations

from enum import StrEnum

from framework.contract.model import StrictModel


class Handover:
    class Service(StrEnum):
        RUN = "srv/handover/run"
        STOP = "srv/handover/stop"
        PAUSE = "srv/handover/pause"
        RESUME = "srv/handover/resume"
        STEP_ONCE = "srv/handover/step_once"
        RUN_TO = "srv/handover/run_to"
        TOGGLE_BREAKPOINT = "srv/handover/toggle_breakpoint"
        LIST_ROBOTS = "srv/handover/list_robots"
        PREVIEW = "srv/handover/preview"

    class Stream(StrEnum):
        STATE = "stream/handover/{robot_id}/state"
        TRACE = "stream/handover/{robot_id}/trace"
        MARKERS = "stream/handover/{robot_id}/markers"


class RunRequest(StrictModel):
    pick_object: str
    place_object: str = ""  # 비우면 수취 후 적치 생략 (so101 이 든 채 종료 방지 — home 에 내려놓기 없음, §시나리오 주석)


class ListRobotsRequest(StrictModel):
    pass


class ListRobotsResponse(StrictModel):
    """참여 robot 명부 — giver(omx) / receiver(so101) 순서 고정 아님, 모듈
    TASK_ROBOTS 가 SSOT (frontend 는 이 목록으로 스트림 키를 채운다)."""

    robot_ids: list[str]


class TaskMarker(StrictModel):
    label: str
    position: tuple[float, float, float]


class TaskMarkers(StrictModel):
    robot_id: str
    seq: int
    timestamp_unix: float
    markers: list[TaskMarker] = []
