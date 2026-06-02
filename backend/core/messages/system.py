"""System (heartbeat / log) 토픽 payload schema.

토픽:
- SYSTEM_HEARTBEAT (publish) — Heartbeat
- SYSTEM_LOG       (publish) — LogMessage

heartbeat 은 BaseNode._heartbeat_loop 가 1Hz 발행.
log 는 BaseNode.log() 가 호출 시점 발행.
"""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, ConfigDict


class Heartbeat(BaseModel):
    """SYSTEM_HEARTBEAT 페이로드. 노드별 1Hz."""

    model_config = ConfigDict(extra="forbid")

    node: str
    timestamp: float
    status: str = "ok"


class LogMessage(BaseModel):
    """SYSTEM_LOG 페이로드. BaseNode.log("info", "...") 호출 시 발행."""

    model_config = ConfigDict(extra="forbid")

    node: str
    timestamp: float
    level: Literal["debug", "info", "warning", "error"]
    message: str
