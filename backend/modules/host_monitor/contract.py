from __future__ import annotations

from enum import StrEnum

from pydantic import BaseModel


class HostMonitor:
    class Stream(StrEnum):
        METRICS = "stream/host_monitor/metrics"


class HostMetrics(BaseModel):
    host: str
    seq: int
    timestamp_unix: float
    cpu_percent: float
    mem_percent: float
