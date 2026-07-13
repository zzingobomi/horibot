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
        # TODO(미결): 실행 전 전체 step 목록 미리보기 (디버거 breakpoint/run-to 는
        # 실행 전에 대상 목록이 필요 — @step 을 지정하는 이유). 설계 미확정:
        #   - imperative 시나리오라 실행 없이 목록을 뽑으려면 정적 분석(AST) 필요한데,
        #     if/loop/동적·변수 함수 호출에서 순수 AST 는 깨진다 (실행 구조를 안 돌리고
        #     정확히 아는 건 근본적으로 불가 — 정지문제급).
        #   - 완전 보장하려면 구조를 **선언형 데이터**로 (Airflow DAG / BehaviorTree /
        #     MoveIt Task Constructor / 옛 DSL 방식) 표현해야 함 = @step 을 선언형 구조로
        #     전환하는 방향.
        #   - 절충: best-effort AST(정적, 비동적 부분) + 런타임 trace(동적 부분 보완).
        # 방향 확정 후 서비스/요청·응답 추가. (2026-07-13 논의)

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
