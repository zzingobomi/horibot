"""Task 노드 토픽 / 서비스 payload schema.

토픽 (모두 typed 면제 — frontend 가 step type union / state.to_dict 직렬화 처리):
- TASK_TREE         — step tree (재귀 union)
- TASK_STATE        — state.to_dict (free-form)
- TASK_STEP_RESULT  — typed value class union (Detection/Position3/Pose6/None)

서비스 (request data / response data):
- TASK_RUN              — typed 면제 (factory 동적 인자: task name + extras)
- TASK_STOP             — EmptyData / EmptyData
- TASK_PAUSE            — EmptyData / EmptyData
- TASK_RESUME           — EmptyData / EmptyData
- TASK_STATUS           — typed 면제 (state.to_dict free-form 응답)
- TASK_STEP             — EmptyData / EmptyData
- TASK_RUN_TO           — TaskStepIdReq / EmptyData
- TASK_TOGGLE_BREAKPOINT — TaskStepIdReq / EmptyData
- TASK_PREVIEW          — typed 면제 (factory 동적 인자 + tree dict 응답)
"""

from __future__ import annotations

from core.transport.messages.base import StrictModel



# ─── Service: TASK_RUN_TO / TASK_TOGGLE_BREAKPOINT ───────────────────


class TaskStepIdReq(StrictModel):
    """step_id 만 받는 디버거 명령 공통 입력."""

    step_id: str
