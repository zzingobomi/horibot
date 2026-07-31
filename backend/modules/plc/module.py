"""PlcModule — 산업용 PLC 태그 read model owner + write 표면.

설계 정본 = docs/plc_conveyor.md §9 (§10 잠금). 책임 경계 (§9.1):
Module 은 "무엇을" 폴링할지 정하고 (태그 DB / 스캔 정책 / cache / 재연결),
Driver 는 "어떻게" 가져올지 정한다 (주소 파싱 / coalescing / 인코딩).

스캔 루프 (PLC 당 1 task):
    미연결 → open() backoff 재시도 (1s → 5s cap, 로그는 전이 시 1회)
    연결   → read(전 태그) → cache diff → 변경분만 TAGS_CHANGED 발행
    read 실패(전송) → 연결 끊김 처리: 태그 전부 **STALE 강등** (값 유지 —
        "PLC 죽음" 과 "값 false" 구분, 침묵 fallback 금지) + connected=False
        발행 → open 재시도로 복귀. 복귀 스캔에서 GOOD 재승격이 diff 로 발행됨.

드라이버 호출은 전부 blocking sync → asyncio.to_thread (async 계약 규약).
write 반영은 다음 스캔부터 (contract.py WriteTagResponse docstring — PLC 자체
스캔 주기 실측 교훈, plc/probe.py).
"""

from __future__ import annotations

import asyncio
import logging
import time
from dataclasses import dataclass, field

from framework.contract.publisher import publishes
from framework.contract.service import service
from framework.runtime.api import ModuleRuntime

from .contract import (
    Plc,
    PlcBundle,
    PlcSnapshot,
    Quality,
    SnapshotTagsRequest,
    TagReading,
    TagsChanged,
    WriteTagRequest,
    WriteTagResponse,
)
from .drivers.protocol import PlcBackend, PointSpec

logger = logging.getLogger(__name__)

_RECONNECT_BACKOFF_START_S = 1.0
_RECONNECT_BACKOFF_CAP_S = 5.0


@dataclass
class PlcUnitSpec:
    """PLC 1대의 배선 명세 — resolve 가 plcs.yaml 에서 구성해 주입."""

    backend: PlcBackend
    points: dict[str, PointSpec]  # 태그 이름 → 주소 명세
    scan_interval_s: float


@dataclass
class _PlcState:
    connected: bool = False
    seq: int = 0
    tags: dict[str, TagReading] = field(default_factory=dict)


@publishes((Plc.Event.TAGS_CHANGED, TagsChanged))
class PlcModule:
    def __init__(self, runtime: ModuleRuntime, plcs: dict[str, PlcUnitSpec]) -> None:
        self.runtime = runtime
        self._plcs = plcs
        self._states: dict[str, _PlcState] = {pid: _PlcState() for pid in plcs}
        self._tasks: list[asyncio.Task[None]] = []
        self._stop = False

    async def start(self) -> None:
        # 주소 오타 fail-fast — 부팅 시점에 전 태그 검증 (§9.3 validate 훅).
        for pid, unit in self._plcs.items():
            unit.backend.validate(unit.points)
        self._stop = False
        self._tasks = [
            asyncio.create_task(self._scan_loop(pid), name=f"plc-scan-{pid}")
            for pid in self._plcs
        ]
        logger.info(
            "PlcModule start — %d PLC: %s",
            len(self._plcs),
            {pid: sorted(u.points) for pid, u in self._plcs.items()},
        )

    async def stop(self) -> None:
        self._stop = True
        for task in self._tasks:
            task.cancel()
        for task in self._tasks:
            try:
                await task
            except asyncio.CancelledError:
                pass
        self._tasks = []
        for pid, unit in self._plcs.items():
            try:
                await asyncio.to_thread(unit.backend.close)
            except Exception:
                logger.exception("PLC %s close 실패 — shutdown 계속", pid)

    # ── 서비스 ──────────────────────────────────────────────

    @service(Plc.Service.SNAPSHOT_TAGS)
    async def snapshot_tags(self, req: SnapshotTagsRequest) -> PlcBundle:
        return PlcBundle(
            plcs={
                pid: PlcSnapshot(connected=st.connected, tags=dict(st.tags))
                for pid, st in self._states.items()
            }
        )

    @service(Plc.Service.WRITE_TAG)
    async def write_tag(self, req: WriteTagRequest) -> WriteTagResponse:
        unit = self._plcs.get(req.plc_id)
        if unit is None:
            raise ValueError(
                f"PLC {req.plc_id!r} 없음 — 구성된 PLC: {sorted(self._plcs)} "
                f"(plc/plcs.yaml)"
            )
        point = unit.points.get(req.tag)
        if point is None:
            raise ValueError(
                f"PLC {req.plc_id} 에 태그 {req.tag!r} 없음 — 구성된 태그: "
                f"{sorted(unit.points)} (plc/plcs.yaml tags)"
            )
        state = self._states[req.plc_id]
        if not state.connected:
            raise RuntimeError(
                f"PLC {req.plc_id} 연결 안 됨 (백그라운드 재연결 중) — "
                f"연결 복구 후 재시도하세요"
            )
        # 실패는 드라이버 raise 그대로 전파 (RemoteError 로 wire 를 건넘).
        await asyncio.to_thread(unit.backend.write, point, req.value)
        return WriteTagResponse(tag=req.tag, value=req.value)

    # ── 스캔 루프 ────────────────────────────────────────────

    async def _scan_loop(self, pid: str) -> None:
        unit = self._plcs[pid]
        state = self._states[pid]
        backoff = _RECONNECT_BACKOFF_START_S
        connect_failed_logged = False
        try:
            while not self._stop:
                if not state.connected:
                    try:
                        await asyncio.to_thread(unit.backend.open)
                    except Exception as exc:
                        if not connect_failed_logged:
                            logger.warning(
                                "PLC %s 연결 실패 — %.0fs 간격 재시도: %s",
                                pid, backoff, exc,
                            )
                            connect_failed_logged = True
                        await asyncio.sleep(backoff)
                        backoff = min(backoff * 2, _RECONNECT_BACKOFF_CAP_S)
                        continue
                    state.connected = True
                    backoff = _RECONNECT_BACKOFF_START_S
                    connect_failed_logged = False
                    logger.info("PLC %s 연결됨", pid)
                    # 연결 전이는 changed 없이도 발행 (아래 스캔이 곧바로
                    # diff 를 실어 나르지만, 태그 0개 구성도 전이는 보여야 함)
                    self._publish(pid, changed={})

                try:
                    values = await asyncio.to_thread(unit.backend.read, unit.points)
                except Exception:
                    logger.exception(
                        "PLC %s read 실패 — 연결 끊김 처리, 재연결 진입", pid
                    )
                    await self._demote_disconnected(pid)
                    continue

                changed: dict[str, TagReading] = {}
                for name, tv in values.items():
                    reading = TagReading(value=tv.value, quality=tv.quality, ts=tv.ts)
                    prev = state.tags.get(name)
                    # ts 는 매 스캔 갱신되므로 비교에서 제외 — 값/품질 변화만 이벤트
                    if (
                        prev is None
                        or prev.value != reading.value
                        or prev.quality != reading.quality
                    ):
                        state.tags[name] = reading
                        changed[name] = reading
                if changed:
                    self._publish(pid, changed=changed)
                await asyncio.sleep(unit.scan_interval_s)
        except asyncio.CancelledError:
            pass

    async def _demote_disconnected(self, pid: str) -> None:
        """연결 단절 — 태그 값 유지 + STALE 강등 (마지막 GOOD ts 보존)."""
        state = self._states[pid]
        state.connected = False
        unit = self._plcs[pid]
        try:
            await asyncio.to_thread(unit.backend.close)
        except Exception:
            logger.exception("PLC %s close 실패 — 재연결 계속", pid)
        demoted = {
            name: r.model_copy(update={"quality": Quality.STALE})
            for name, r in state.tags.items()
            if r.quality != Quality.STALE
        }
        state.tags.update(demoted)
        self._publish(pid, changed=demoted)

    def _publish(self, pid: str, changed: dict[str, TagReading]) -> None:
        state = self._states[pid]
        state.seq += 1
        self.runtime.publish(
            Plc.Event.TAGS_CHANGED,
            TagsChanged(
                plc_id=pid,
                seq=state.seq,
                timestamp_unix=time.time(),
                connected=state.connected,
                changed=changed,
            ),
        )
