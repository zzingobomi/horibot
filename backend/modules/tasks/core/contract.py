"""task 스트림 payload 규약 — 모든 task 모듈이 같은 모양으로 발행 (공용 UI 의 전제).

키는 각 task 모듈의 contract.py 가 소유 (`stream/<task>/{robot_id}/state|trace|
step_result`) — 여기는 **payload 모양만** 정의한다. 이 모양을 따르면 frontend 의
공용 task 부품 (진행 패널/씬 오버레이) 이 그대로 붙는다.

파일 이름이 contract.py 인 이유: contract_export 의 모델 탐색이 contract.py 에서
**정의된** 클래스만 카탈로그에 넣는다 (재노출 import 제외) — task 규약 payload 의
정의 자리가 곧 contract 표면이어야 frontend 타입이 생성된다. core 는 Service/
Stream 키가 없으니 (outer class 없음) 키 카탈로그엔 안 잡히고 모델만 실린다.

stream invariant (§8.5): robot_id + seq + timestamp_unix.
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


# TraceEntry.status 값 — frontend 진행 표시 색상용.
TRACE_RUNNING = "running"
TRACE_COMPLETED = "completed"
TRACE_FAILED = "failed"


class TraceEntry(BaseModel):
    """primitive 호출 1건 — TRACE 의 원소 (옛 DSL step tree 의 대체).

    label = 시나리오가 붙인 이름 (없으면 자동 "kind#n") — breakpoint/run_to 의 대상.
    kind = primitive 종류 ("move_l"/"detect_oriented"/...).
    """

    label: str
    kind: str
    status: str  # TRACE_RUNNING / COMPLETED / FAILED
    detail: str = ""  # 실패 사유 등
    started_unix: float
    ended_unix: float | None = None


class TaskState(BaseModel):
    """실행 상태 — 상태 전이마다 발행. error = FAILED 사유 (사용자 표시용 문장)."""

    robot_id: str
    seq: int
    timestamp_unix: float
    status: TaskStatus
    task_name: str = ""
    current_label: str = ""  # 지금 실행/정지 중인 primitive label
    error: str | None = None
    breakpoints: list[str] = []


class TaskTrace(BaseModel):
    """primitive 호출 누적 — entry 추가/상태 변경마다 전체 리스트 재발행
    (latest-wins 스트림에서 frontend 가 항상 전체를 재구성)."""

    robot_id: str
    seq: int
    timestamp_unix: float
    task_name: str = ""
    entries: list[TraceEntry] = []


class TaskStepResult(BaseModel):
    """시나리오 중간값 노출 — ctx.record + 값 primitive 자동 발행.

    type = 값 클래스 이름 ("OrientedDetection"/"list"/"None"...) — 씬 오버레이가
    type 별 마커로 dispatch. value = model_dump dict / list / scalar / None.
    """

    robot_id: str
    seq: int
    timestamp_unix: float
    label: str
    type: str
    value: Any = None
