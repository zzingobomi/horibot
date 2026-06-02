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
    """SYSTEM_HEARTBEAT 페이로드. 노드별 1Hz."""

    node: str
    timestamp: float
    status: str = "ok"


class LogMessage(StrictModel):
    """SYSTEM_LOG 페이로드. BaseNode.log("info", "...") 호출 시 발행."""

    node: str
    timestamp: float
    level: Literal["debug", "info", "warning", "error"]
    message: str
