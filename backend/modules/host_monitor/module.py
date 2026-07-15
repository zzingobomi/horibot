from __future__ import annotations

import asyncio
import logging
import time

import psutil

from framework.contract.publisher import publishes
from framework.runtime.api import ModuleRuntime

from .contract import HostMetrics, HostMonitor

logger = logging.getLogger(__name__)

_PUBLISH_HZ = 1.0


@publishes((HostMonitor.Stream.METRICS, HostMetrics))
class HostMonitorModule:
    def __init__(
        self, runtime: ModuleRuntime, host: str, publish_hz: float = _PUBLISH_HZ
    ) -> None:
        self.runtime = runtime
        self._host = host
        self._interval = 1.0 / publish_hz
        self._seq = 0
        self._task: asyncio.Task[None] | None = None
        self._stop = False

    async def start(self) -> None:
        psutil.cpu_percent(interval=None)
        self._stop = False
        self._task = asyncio.create_task(self._loop())

    async def stop(self) -> None:
        self._stop = True
        if self._task is not None:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
            self._task = None

    async def _loop(self) -> None:
        try:
            while not self._stop:
                try:
                    metrics = HostMetrics(
                        host=self._host,
                        seq=self._seq,
                        timestamp_unix=time.time(),
                        cpu_percent=psutil.cpu_percent(interval=None),
                        mem_percent=psutil.virtual_memory().percent,
                    )
                    self._seq += 1
                    self.runtime.publish(HostMonitor.Stream.METRICS, metrics)
                except Exception:
                    logger.exception("host_monitor publish 실패 host=%s", self._host)
                await asyncio.sleep(self._interval)
        except asyncio.CancelledError:
            pass
