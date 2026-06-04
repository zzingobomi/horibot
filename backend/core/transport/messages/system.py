"""System (heartbeat / log) 토픽 payload schema.

토픽:
- SYSTEM_HEARTBEAT (publish) — Heartbeat
- SYSTEM_LOG       (publish) — LogMessage

heartbeat 은 BaseNode._heartbeat_loop 가 1Hz 발행.
log 는 BaseNode.log() 가 호출 시점 발행.
"""

from __future__ import annotations

from core.transport.messages.base import StrictModel

from typing import Literal



class Heartbeat(StrictModel):
    """SYSTEM_HEARTBEAT 페이로드. 노드별 1Hz.

    robot_id 는 robot-scoped 노드 (motor / motion / camera / ...) 만 채움 —
    global 노드 (task / gamepad / bridge) 는 None. Dashboard 가 robot_id 별
    온라인 상태 구분.
    """

    node: str
    timestamp: float
    status: str = "ok"
    robot_id: str | None = None


class LogMessage(StrictModel):
    """SYSTEM_LOG 페이로드. BaseNode.log("info", "...") 호출 시 발행."""

    node: str
    timestamp: float
    level: Literal["debug", "info", "warning", "error"]
    message: str
    robot_id: str | None = None
