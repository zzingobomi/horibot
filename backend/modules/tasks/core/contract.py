"""task wire 규약 payload — 모든 task 가 같은 모양 (공용 UI/실행기 조작판의 전제).

키는 각 task 모듈의 contract.py 가 소유 (`srv/<task>/...` +
`stream/<task>/{robot_id}/...` — 전부 저자 손코드) — 여기는 **payload 모양만**
정의한다. 스트림 payload (TaskState/TaskTrace — 모듈의 runner 콜백
이 조립·발행) 와 실행기 조작판의 req/res (RunResponse/Control*/RunTo*/
ToggleBreakpoint*) — TaskRunner API 의 wire 노출에 쓰는 공용 모양이라 task 마다
재선언하지 않는다 (2026-07-13. task 고유 wire 는 RunRequest — 각 task 소유).

파일 이름이 contract.py 인 이유: contract_export 의 모델 탐색이 contract.py 에서
**정의된** 클래스만 카탈로그에 넣는다 (재노출 import 제외) — task 규약 payload 의
정의 자리가 곧 contract 표면이어야 frontend 타입이 생성된다. core 는 Service/
Stream 키가 없으니 (outer class 없음) 키 카탈로그엔 안 잡히고 모델만 실린다.

stream invariant (§8.5): robot_id + seq + timestamp_unix.
"""

from __future__ import annotations

from enum import StrEnum

from pydantic import BaseModel

from framework.contract.model import StrictModel


class TaskStatus(StrEnum):
    IDLE = "idle"
    RUNNING = "running"
    PAUSED = "paused"
    SUCCESS = "success"
    FAILED = "failed"
    STOPPED = "stopped"


# TraceEntry.status 값 — frontend 진행 표시 색상용.
TRACE_RUNNING = "running"
TRACE_COMPLETED = "completed"
TRACE_FAILED = "failed"


class TraceEntry(BaseModel):
    """step 진입 1건 — TRACE 의 원소.

    name = @step 함수 이름 — breakpoint/run_to 의 대상 (식별자, run 간 안정).
    title = UI 표시 이름 (@step(title="집기") — 선택, 빈 값 = name 그대로 표시).
    depth = 중첩 깊이 (0 = 시나리오 최상위). wire 는 flat 리스트 + depth 하나 —
    리스트 순서 = 실행 순서, 트리 표현은 UI 렌더링 몫 (들여쓰기/접기).
    안쪽 step 이 실패하면 그 entry 와 바깥 step entry 가 전부 failed 로 찍혀
    실패 경로(root→leaf)가 보인다.
    """

    name: str
    title: str = ""  # 표시 이름 — name(식별자)과 분리 (한글 등)
    depth: int = 0
    status: str  # TRACE_RUNNING / COMPLETED / FAILED
    detail: str = ""  # 실패 사유 (FAILED 일 때만 채워짐)
    started_unix: float
    ended_unix: float | None = None


class TaskState(BaseModel):
    """실행 상태 — 상태 전이마다 발행. error = FAILED 사유 (사용자 표시용 문장)."""

    robot_id: str
    seq: int
    timestamp_unix: float
    status: TaskStatus
    task_name: str = ""
    current_name: str = ""  # 지금 실행/정지 중인 step name (식별자)
    current_title: str = ""  # 그 step 의 표시 이름 (빈 값 = name 그대로)
    error: str | None = None
    breakpoints: list[str] = []


class TaskTrace(BaseModel):
    """step 진입 누적 — entry 추가/상태 변경마다 전체 리스트 재발행
    (latest-wins 스트림에서 frontend 가 항상 전체를 재구성)."""

    robot_id: str
    seq: int
    timestamp_unix: float
    task_name: str = ""
    entries: list[TraceEntry] = []


# ─── 실행기 조작판 req/res (표준 표면 — @task 가 핸들러 합성에 사용) ──


class RunResponse(StrictModel):
    accepted: bool
    message: str = ""  # 거부 사유 ("이미 실행 중" 등)


class ControlRequest(StrictModel):
    """stop/pause/resume/step_once 공용 — 대상 run 은 task 당 1개라 필드 없음."""


class ControlResponse(StrictModel):
    ok: bool
    message: str = ""  # 실패 사유 ("실행 중인 run 없음" 등 — 침묵 금지)


class RunToRequest(StrictModel):
    name: str  # TRACE 의 step name — 이 step 직전까지 진행 후 pause


class ToggleBreakpointRequest(StrictModel):
    name: str  # breakpoint 대상 step name
