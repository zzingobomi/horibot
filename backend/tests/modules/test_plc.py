"""plc 모듈 검증 — 드라이버 코덱/주소 파싱(순수 함수) + 모듈 시나리오.

시나리오는 UX 워크스루 그대로: 시작(첫 스캔 발행) → 진행(변경분만 발행)
→ write → **단절(STALE 강등 + connected=False — "PLC 죽음" ≠ "값 false")**
→ 복구(GOOD 재승격). 실물 없이 mock/스크립트 backend 로 결정적 재현.
"""

from __future__ import annotations

import asyncio

import pytest
from pydantic import BaseModel

from modules.plc.contract import (
    Plc,
    Quality,
    SnapshotTagsRequest,
    TagsChanged,
    WriteTagRequest,
)
from modules.plc.drivers.mock import MockPlcBackend
from modules.plc.drivers.modbus_tcp import (
    ParsedAddress,
    coalesce,
    decode_words,
    encode_words,
    parse_address,
)
from modules.plc.drivers.protocol import PointSpec, coerce_dtype
from modules.plc.module import PlcModule, PlcUnitSpec

# ─── 드라이버 순수 함수: 주소 파싱 ────────────────────────────────


def test_parse_address_ok():
    p = parse_address("coil:2", "bool", tag="pick_done")
    assert (p.kind, p.offset, p.words) == ("coil", 2, 1)
    p = parse_address("hr:10", "float32_be", tag="temp")
    assert (p.kind, p.offset, p.words) == ("hr", 10, 2)
    p = parse_address("ir:0", "int16", tag="raw")
    assert (p.kind, p.offset, p.words) == ("ir", 0, 1)


@pytest.mark.parametrize(
    ("address", "dtype", "fragment"),
    [
        ("holding:0", "int16", "문법 오류"),  # 미지 종류
        ("coil:x", "bool", "정수가 아님"),
        ("coil:-1", "bool", "0 이상"),
        ("coil:0", None, "dtype 필수"),  # Modbus 무타입 — dtype 강제
        ("di:0", "int16", "bool 이어야"),  # 비트 종류 ↔ 숫자 dtype 궁합
        ("hr:0", "bool", "bool 불가"),  # 레지스터 ↔ bool 궁합
    ],
)
def test_parse_address_rejects(address, dtype, fragment):
    with pytest.raises(ValueError, match=fragment):
        parse_address(address, dtype, tag="t")


def test_coerce_dtype_rejects_unknown():
    with pytest.raises(ValueError, match="알 수 없는 dtype"):
        coerce_dtype("float64", tag="t")
    assert coerce_dtype(None, tag="t") is None
    assert coerce_dtype("bool", tag="t") == "bool"


# ─── 드라이버 순수 함수: 워드 코덱 (엔디안 footgun 잠금, §10) ──────


def test_word_codec_roundtrip():
    for dtype, value in [
        ("int16", -123),
        ("uint16", 65535),
        ("int32", -70000),
        ("float32_be", 12.5),
        ("float32_le_swap", -0.25),
    ]:
        words = encode_words(value, dtype)  # type: ignore[arg-type]
        assert decode_words(words, dtype) == pytest.approx(value)  # type: ignore[arg-type]


def test_float32_word_order_differs():
    """be 와 le_swap 은 같은 값의 워드 순서가 정확히 뒤집힘 (CDAB)."""
    be = encode_words(12.5, "float32_be")
    swap = encode_words(12.5, "float32_le_swap")
    assert be == list(reversed(swap))
    # 12.5 = 0x41480000 → BE 워드 = [0x4148, 0x0000]
    assert be == [0x4148, 0x0000]


def test_encode_rejects_non_numeric():
    with pytest.raises(TypeError):
        encode_words(True, "int16")
    with pytest.raises(ValueError, match="범위 밖"):
        encode_words(70000, "uint16")


# ─── 드라이버 순수 함수: coalescing ───────────────────────────────


def _pa(kind: str, offset: int, words: int = 1) -> ParsedAddress:
    return ParsedAddress(kind=kind, offset=offset, words=words)


def test_coalesce_merges_neighbors_and_splits_far():
    members = [
        ("a", _pa("coil", 0)),
        ("b", _pa("coil", 1)),
        ("c", _pa("coil", 2)),
        ("far", _pa("coil", 100)),
    ]
    blocks = coalesce(members, kind="coil")
    assert len(blocks) == 2
    assert (blocks[0].start, blocks[0].count) == (0, 3)
    assert [n for n, _ in blocks[0].members] == ["a", "b", "c"]
    assert (blocks[1].start, blocks[1].count) == (100, 1)


def test_coalesce_counts_multiword_span():
    members = [("f", _pa("hr", 0, words=2)), ("g", _pa("hr", 2, words=2))]
    blocks = coalesce(members, kind="hr")
    assert len(blocks) == 1
    assert (blocks[0].start, blocks[0].count) == (0, 4)


# ─── mock 드라이버 ───────────────────────────────────────────────

_POINTS = {
    "conveyor_run": PointSpec(address="coil:0", dtype="bool"),
    "object_arrived": PointSpec(address="coil:1", dtype="bool"),
    "pick_done": PointSpec(address="coil:2", dtype="bool"),
    "sensor": PointSpec(address="di:0", dtype="bool"),
}


def test_mock_backend_read_write_cycle():
    be = MockPlcBackend()
    be.validate(_POINTS)
    with pytest.raises(ConnectionError):
        be.read(_POINTS)  # open 전 read = 연결 에러 (module 재연결 경로)
    be.open()
    values = be.read(_POINTS)
    assert set(values) == set(_POINTS)
    assert all(v.quality is Quality.GOOD for v in values.values())
    assert values["conveyor_run"].value is False  # dtype 기본값
    be.write(_POINTS["pick_done"], True)
    assert be.read(_POINTS)["pick_done"].value is True
    be.force("di:0", True)  # 읽기전용 입력 주입 (Editor force 대응)
    assert be.read(_POINTS)["sensor"].value is True


# ─── 모듈 시나리오 ───────────────────────────────────────────────


class _StubRuntime:
    def __init__(self) -> None:
        self.published: list[tuple[str, BaseModel]] = []

    def publish(self, wire_key: str, event: BaseModel) -> None:
        self.published.append((str(wire_key), event))

    def events(self) -> list[TagsChanged]:
        return [e for _, e in self.published if isinstance(e, TagsChanged)]


class _ScriptedBackend(MockPlcBackend):
    """단절 주입 가능한 mock — broken=True 면 open/read 가 전송 실패처럼 raise."""

    def __init__(self) -> None:
        super().__init__()
        self.broken = False

    def open(self) -> None:
        if self.broken:
            raise ConnectionError("주입된 연결 실패")
        super().open()

    def read(self, points):
        if self.broken:
            raise ConnectionError("주입된 read 실패")
        return super().read(points)


async def _wait_for(predicate, *, timeout_s: float = 2.0) -> None:
    deadline = asyncio.get_running_loop().time() + timeout_s
    while not predicate():
        if asyncio.get_running_loop().time() > deadline:
            raise AssertionError("조건 대기 timeout")
        await asyncio.sleep(0.01)


def _module(backend) -> tuple[PlcModule, _StubRuntime]:
    rt = _StubRuntime()
    mod = PlcModule(
        rt,  # type: ignore[arg-type]
        plcs={
            "openplc_0": PlcUnitSpec(
                backend=backend, points=dict(_POINTS), scan_interval_s=0.01
            )
        },
    )
    return mod, rt


async def test_first_scan_publishes_all_tags_then_quiesces():
    mod, rt = _module(MockPlcBackend())
    await mod.start()
    try:
        await _wait_for(lambda: any(e.changed for e in rt.events()))
        first = next(e for e in rt.events() if e.changed)
        assert set(first.changed) == set(_POINTS)  # 첫 스캔 = 전 태그가 "변경"
        assert first.connected is True
        # snapshot(Mirror) 과 이벤트가 같은 상태를 말함
        snap = await mod.snapshot_tags(SnapshotTagsRequest())
        assert snap.plcs["openplc_0"].connected is True
        assert snap.plcs["openplc_0"].tags["conveyor_run"].value is False
        # 변화 없으면 침묵 (idle 동안 changed 있는 이벤트 추가 발행 X)
        count = len([e for e in rt.events() if e.changed])
        await asyncio.sleep(0.1)
        assert len([e for e in rt.events() if e.changed]) == count
    finally:
        await mod.stop()


async def test_write_tag_reflected_on_next_scan_and_published_as_diff():
    backend = MockPlcBackend()
    mod, rt = _module(backend)
    await mod.start()
    try:
        await _wait_for(lambda: any(e.changed for e in rt.events()))
        base = len(rt.events())
        res = await mod.write_tag(
            WriteTagRequest(plc_id="openplc_0", tag="pick_done", value=True)
        )
        assert res.value is True
        await _wait_for(
            lambda: any("pick_done" in e.changed for e in rt.events()[base:])
        )
        diff = next(e for e in rt.events()[base:] if e.changed)
        assert set(diff.changed) == {"pick_done"}  # 변경분만 — 전체 재발행 X
        assert diff.changed["pick_done"].value is True
    finally:
        await mod.stop()


async def test_write_tag_rejects_unknown_targets():
    mod, _rt = _module(MockPlcBackend())
    await mod.start()
    try:
        with pytest.raises(ValueError, match="PLC 'nope' 없음"):
            await mod.write_tag(WriteTagRequest(plc_id="nope", tag="x", value=1))
        with pytest.raises(ValueError, match="태그 'nope' 없음"):
            await mod.write_tag(
                WriteTagRequest(plc_id="openplc_0", tag="nope", value=1)
            )
    finally:
        await mod.stop()


async def test_disconnect_demotes_stale_then_recovers(monkeypatch):
    # 재연결 backoff 을 테스트 속도로 (상수는 실물용 1s→5s)
    monkeypatch.setattr("modules.plc.module._RECONNECT_BACKOFF_START_S", 0.02)
    monkeypatch.setattr("modules.plc.module._RECONNECT_BACKOFF_CAP_S", 0.05)
    backend = _ScriptedBackend()
    mod, rt = _module(backend)
    await mod.start()
    try:
        await _wait_for(lambda: any(e.changed for e in rt.events()))

        # ── 단절 주입 ──
        backend.broken = True
        await _wait_for(lambda: any(not e.connected for e in rt.events()))
        down = next(e for e in rt.events() if not e.connected)
        # 값은 유지 + STALE 강등 — "PLC 죽음" 과 "값 false" 구분 (침묵 fallback 금지)
        assert down.changed and all(
            r.quality is Quality.STALE for r in down.changed.values()
        )
        snap = await mod.snapshot_tags(SnapshotTagsRequest())
        assert snap.plcs["openplc_0"].connected is False
        assert (
            snap.plcs["openplc_0"].tags["conveyor_run"].quality is Quality.STALE
        )
        # 단절 중 write = 사유 있는 거부 (다음 행동 안내)
        with pytest.raises(RuntimeError, match="연결 안 됨"):
            await mod.write_tag(
                WriteTagRequest(plc_id="openplc_0", tag="pick_done", value=True)
            )

        # ── 복구 ── (단절 이후 이벤트만 관찰 — 단절 전 GOOD 이벤트 오매칭 방지)
        after_down = len(rt.events())
        backend.broken = False
        await _wait_for(
            lambda: any(
                e.connected
                and any(r.quality is Quality.GOOD for r in e.changed.values())
                for e in rt.events()[after_down:]
            )
        )
        snap = await mod.snapshot_tags(SnapshotTagsRequest())
        assert snap.plcs["openplc_0"].connected is True
        assert all(
            r.quality is Quality.GOOD
            for r in snap.plcs["openplc_0"].tags.values()
        )
    finally:
        await mod.stop()


async def test_start_fails_fast_on_bad_address():
    """부팅 validate 훅 — 주소 오타는 스캔 시작 전에 죽는다 (§9.3)."""
    from modules.plc.drivers.modbus_tcp import ModbusTcpBackend

    mod, _rt = _module(ModbusTcpBackend(host="127.0.0.1"))
    mod._plcs["openplc_0"].points["typo"] = PointSpec(address="coli:0", dtype="bool")
    with pytest.raises(ValueError, match="문법 오류"):
        await mod.start()


def test_tags_changed_key_is_event_scoped():
    """계약 키 규약 — event/plc/* (host-scoped, {robot_id} placeholder 없음)."""
    assert str(Plc.Event.TAGS_CHANGED) == "event/plc/tags_changed"
    assert "{robot_id}" not in str(Plc.Service.SNAPSHOT_TAGS)
