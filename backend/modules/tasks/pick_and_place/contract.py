"""Pick & Place task 모듈 wire 계약 — 이 모듈 wire 의 SSOT.

표준 task 표면 규약 (tasks/core): `srv/<task>/run|stop|...` 키 이름 + core/wire.py
payload 모양을 따르면 frontend 공용 task 부품이 그대로 붙는다. 디버거 계열
(pause/step_once/run_to/toggle_breakpoint) 은 선언한 task 만 노출 — 이 모듈은
디버거 레퍼런스 구현이라 전부 선언. STOP 은 안전 의무 (움직이는 로봇을 세울 통로).
"""

from __future__ import annotations

from enum import StrEnum

from framework.contract.model import StrictModel

# 스트림 payload 모양의 정의(SSOT) = modules/tasks/core/contract.py
# (TaskState/TaskTrace/TaskStepResult — task 공통 규약, 모든 task 모듈이 공유).


class PickAndPlace:
    class Service(StrEnum):
        # robot-agnostic (host 당 1, §2.7) — 대상 robot 은 task 정의가 소유
        # (TASK_INFO.robots, 시나리오의 ctx.robot(...) 리터럴).
        RUN = "srv/pick_and_place/run"
        STOP = "srv/pick_and_place/stop"
        PAUSE = "srv/pick_and_place/pause"
        RESUME = "srv/pick_and_place/resume"
        STEP_ONCE = "srv/pick_and_place/step_once"
        RUN_TO = "srv/pick_and_place/run_to"
        TOGGLE_BREAKPOINT = "srv/pick_and_place/toggle_breakpoint"

    class Stream(StrEnum):
        # robot-scoped 키 — payload robot_id 라우팅 (host-level 발행, scan 동형).
        # payload 모양 = tasks/core/wire.py (TaskState/TaskTrace/TaskStepResult).
        STATE = "stream/pick_and_place/{robot_id}/state"
        TRACE = "stream/pick_and_place/{robot_id}/trace"
        STEP_RESULT = "stream/pick_and_place/{robot_id}/step_result"


class RunRequest(StrictModel):
    """실행 param — typed (GET /tasks param 스펙이 이 모델에서 자동 파생)."""

    pick_object: str  # 검출 prompt (영어 — GDINO), 예: "white cube"
    place_object: str = ""  # 빈 값 = pick+lift 만 (놓지 않고 든 채 종료)


class RunResponse(StrictModel):
    accepted: bool
    message: str = ""  # 거부 사유 ("이미 실행 중" 등)


class ControlRequest(StrictModel):
    """stop/pause/resume/step_once 공용 — 이 task 는 대상 robot 고정이라 필드 없음."""


class ControlResponse(StrictModel):
    ok: bool
    message: str = ""  # 실패 사유 ("실행 중인 run 없음" 등 — 침묵 금지)


class RunToRequest(StrictModel):
    label: str  # TRACE 의 단계 이름 — 이 primitive 직전까지 진행 후 pause


class ToggleBreakpointRequest(StrictModel):
    label: str
