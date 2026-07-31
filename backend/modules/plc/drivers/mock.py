"""MockPlcBackend — 인메모리 태그 딕셔너리 (라이브 PLC 없이 개발/pytest).

driver_mode=mock 배포 + fast-loop 테스트용. 주소 문법 없음 — 임의 문자열
key 로 값을 저장한다 (validate 는 빈 주소만 거부). 미기록 주소 read 는
dtype 기본값(bool=False / int=0 / float=0.0)으로 시작 — 실 PLC 의
"부팅 직후 코일 전부 0" 과 동형.

테스트/데모 제어 표면 (driver-private — Protocol 밖):
- force(address, value): 마스터가 못 쓰는 읽기전용 입력(센서 %IX 류) 주입.
"""

from __future__ import annotations

from datetime import UTC, datetime

from ..contract import Quality, TagScalar
from .protocol import PointSpec, TagValue, default_for_dtype


class MockPlcBackend:
    def __init__(self, initial: dict[str, TagScalar] | None = None) -> None:
        self._values: dict[str, TagScalar] = dict(initial or {})
        self._open = False

    # ── lifecycle ──
    def open(self) -> None:
        self._open = True

    def close(self) -> None:
        self._open = False

    def is_connected(self) -> bool:
        return self._open

    # ── config 검증 ──
    def validate(self, points: dict[str, PointSpec]) -> None:
        for name, point in points.items():
            if not point.address:
                raise ValueError(f"plc 태그 {name!r}: address 가 비어 있음")

    # ── I/O ──
    def read(self, points: dict[str, PointSpec]) -> dict[str, TagValue]:
        if not self._open:
            raise ConnectionError("mock PLC 가 open 되지 않음")
        now = datetime.now(UTC)
        out: dict[str, TagValue] = {}
        for name, point in points.items():
            if point.address not in self._values:
                self._values[point.address] = default_for_dtype(point.dtype)
            out[name] = TagValue(
                value=self._values[point.address], quality=Quality.GOOD, ts=now
            )
        return out

    def write(self, point: PointSpec, value: TagScalar) -> None:
        if not self._open:
            raise ConnectionError("mock PLC 가 open 되지 않음")
        self._values[point.address] = value

    # ── 테스트/데모 제어 ──
    def force(self, address: str, value: TagScalar) -> None:
        """읽기전용 입력 주입 (Editor 디버그 force 대응)."""
        self._values[address] = value
