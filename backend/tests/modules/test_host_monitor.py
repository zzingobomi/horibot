"""HostMonitorModule — 각 host 가 자기 CPU/mem 을 주기 publish 검증.

host 는 payload 에 자기완결로 담긴다 (§3.4.1 — 여러 host 가 한 키로 발행, bridge 가
payload.host 로 fan-in). 여기선 발행 자체(host 각인 + 필드)를 stub runtime 으로 잠근다.
"""

from __future__ import annotations

import asyncio

from pydantic import BaseModel

from modules.host_monitor.contract import HostMetrics, HostMonitor
from modules.host_monitor.module import HostMonitorModule


class _StubRuntime:
    """ModuleRuntime.publish 만 잡는 stub — 발행 키/이벤트 기록."""

    def __init__(self) -> None:
        self.published: list[tuple[str, BaseModel]] = []

    def publish(self, wire_key: str, event: BaseModel) -> None:
        self.published.append((str(wire_key), event))


async def test_publishes_own_host_metrics():
    rt = _StubRuntime()
    mod = HostMonitorModule(rt, host="pi_hori1", publish_hz=50.0)  # type: ignore[arg-type]
    await mod.start()
    try:
        for _ in range(50):
            if rt.published:
                break
            await asyncio.sleep(0.02)
    finally:
        await mod.stop()

    assert rt.published, "host_monitor 가 아무것도 발행 안 함"
    key, event = rt.published[0]
    assert key == str(HostMonitor.Stream.METRICS)  # {host} 치환 없는 단일 키
    assert isinstance(event, HostMetrics)
    assert event.host == "pi_hori1"  # payload 에 host 각인 (fan-in demux 근거)
    assert 0.0 <= event.cpu_percent <= 100.0
    assert 0.0 <= event.mem_percent <= 100.0
    # seq monotonic (§8.5)
    seqs = [e.seq for _, e in rt.published if isinstance(e, HostMetrics)]
    assert seqs == sorted(seqs)


async def test_stop_halts_publishing():
    rt = _StubRuntime()
    mod = HostMonitorModule(rt, host="pc", publish_hz=50.0)  # type: ignore[arg-type]
    await mod.start()
    await asyncio.sleep(0.1)
    await mod.stop()
    count = len(rt.published)
    await asyncio.sleep(0.1)
    # stop 후 더 안 쌓임 (in-flight 1개 여유)
    assert len(rt.published) - count <= 1
