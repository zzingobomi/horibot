"""LogCollector + 발행 핸들러 왕복 검증 (docs/logging.md).

실 Zenoh 없이 in-memory 버스로 publish→subscribe 를 잇고, `logger.info` 한 줄이
중앙 파일에 host 태그와 함께 verbatim 으로 떨어지는지, 그리고 발행/수집 경로가
무한 루프를 안 만드는지(가드)를 결정적으로 고정한다.
"""

from __future__ import annotations

import logging
from pathlib import Path

from infra.logging.publisher import (
    ZenohLogPublisher,
    attach_log_publisher,
    detach_log_publisher,
)
from modules.logcollector.module import LogCollectorModule


class _FakeBus:
    """log/** 만 매칭하는 최소 in-memory transport (publish→subscribe 연결)."""

    def __init__(self) -> None:
        self._subs: list[tuple[str, object]] = []

    def publish(self, key: str, payload: bytes) -> None:
        for pattern, cb in self._subs:
            if _matches(pattern, key):
                cb(payload)  # type: ignore[operator]

    def subscribe(self, key: str, callback: object) -> "_FakeHandle":
        entry = (key, callback)
        self._subs.append(entry)
        return _FakeHandle(self._subs, entry)

    async def call(self, key, payload, timeout=5.0):  # noqa: ANN001 — 미사용, protocol 충족
        raise NotImplementedError


class _FakeHandle:
    def __init__(self, subs: list, entry: tuple) -> None:
        self._subs = subs
        self._entry = entry

    def undeclare(self) -> None:
        if self._entry in self._subs:
            self._subs.remove(self._entry)


def _matches(pattern: str, key: str) -> bool:
    # 우리 용도(log/**)만 지원하는 축약 매처.
    if pattern.endswith("/**"):
        return key.startswith(pattern[:-2])
    return pattern == key


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8") if path.exists() else ""


def test_publisher_line_is_self_contained_with_host(tmp_path: Path) -> None:
    bus = _FakeBus()
    captured: list[tuple[str, bytes]] = []
    bus.publish = lambda k, p: captured.append((k, p))  # type: ignore[method-assign]

    handler = ZenohLogPublisher(bus, host="pi_hori1")  # type: ignore[arg-type]
    record = logging.LogRecord(
        name="modules.detector",
        level=logging.INFO,
        pathname=__file__,
        lineno=1,
        msg="집기 실패: 도달 불가",
        args=(),
        exc_info=None,
    )
    handler.emit(record)

    assert len(captured) == 1
    key, payload = captured[0]
    assert key == "log/pi_hori1"  # wire 키 = 라우팅 전용
    line = payload.decode("utf-8")
    # 줄이 자기완결 — host / level / logger / message 가 줄 안에 있다 (§2).
    assert "[pi_hori1]" in line
    assert "[INFO]" in line
    assert "modules.detector" in line
    assert "집기 실패: 도달 불가" in line


def test_collector_writes_received_line_verbatim(tmp_path: Path) -> None:
    bus = _FakeBus()
    collector = LogCollectorModule(bus, log_dir=tmp_path)  # type: ignore[arg-type]
    collector.start()
    try:
        raw = "2026-07-15 10:00:00.000 [pc] [INFO] modules.motor (pid=1 MainThread): hi"
        bus.publish("log/pc", raw.encode("utf-8"))
        collector._file_handler.flush()  # type: ignore[union-attr]

        content = _read(tmp_path / "horibot.log")
        assert content.strip() == raw  # 접두 조립 없이 그대로 (verbatim)
    finally:
        collector.stop()


def test_percent_signs_in_line_are_not_formatted(tmp_path: Path) -> None:
    # 수신 줄에 % 가 있어도 logging format 문자열로 오해하면 안 됨.
    bus = _FakeBus()
    collector = LogCollectorModule(bus, log_dir=tmp_path)  # type: ignore[arg-type]
    collector.start()
    try:
        raw = "progress 50% done — value=%s not-substituted"
        bus.publish("log/pc", raw.encode("utf-8"))
        collector._file_handler.flush()  # type: ignore[union-attr]
        assert raw in _read(tmp_path / "horibot.log")
    finally:
        collector.stop()


def test_end_to_end_logger_call_lands_in_central_file(tmp_path: Path) -> None:
    # 진짜 왕복: logger.info → 발행 핸들러 → 버스 → collector → 파일.
    bus = _FakeBus()
    collector = LogCollectorModule(bus, log_dir=tmp_path)  # type: ignore[arg-type]
    collector.start()
    publisher = attach_log_publisher(bus, host="pc")  # type: ignore[arg-type]
    test_logger = logging.getLogger("modules.example")
    prev_level = test_logger.level
    test_logger.setLevel(logging.INFO)
    try:
        test_logger.info("스캔 완료 — 프레임 12")
        collector._file_handler.flush()  # type: ignore[union-attr]
        content = _read(tmp_path / "horibot.log")
        assert "[pc]" in content
        assert "스캔 완료 — 프레임 12" in content
        assert "modules.example" in content
    finally:
        detach_log_publisher(publisher)
        test_logger.setLevel(prev_level)
        collector.stop()


def test_collector_sink_does_not_loop_back_into_publisher(tmp_path: Path) -> None:
    # collector 가 파일 쓰며 내는 로그가 다시 발행→수신→기록 되면 무한 루프.
    # sink 로거가 propagate=False 라 root(발행 핸들러)로 안 새는지 고정.
    bus = _FakeBus()
    collector = LogCollectorModule(bus, log_dir=tmp_path)  # type: ignore[arg-type]
    collector.start()
    try:
        assert collector._sink.propagate is False
        # publish 횟수 카운트 — 한 줄 수신이 추가 발행을 유발하지 않아야.
        published: list[str] = []
        orig = bus.publish

        def _counting(key: str, payload: bytes) -> None:
            published.append(key)
            orig(key, payload)

        bus.publish = _counting  # type: ignore[method-assign]
        bus.publish("log/pc", b"one line")
        assert published == ["log/pc"]  # 재발행 없음
    finally:
        collector.stop()
