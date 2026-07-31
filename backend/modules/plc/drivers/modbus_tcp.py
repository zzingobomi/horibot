"""ModbusTcpBackend — pymodbus 동기 클라이언트 기반 Modbus TCP 드라이버.

검증 대역 = OpenPLC Runtime v4 (:502, docs/plc_conveyor.md §9.7). Modbus 는
무타입 워드 프로토콜 — dtype 필수 (§9.6), 멀티워드 엔디안은 dtype 문자열에
인코딩 (§10 잠금).

주소 문법 (이 드라이버 소유 — validate 가 부팅 시 fail-fast):
    coil:N   코일 (bool, read/write)          — OpenPLC %QX0.k = coil k
    di:N     discrete input (bool, 읽기전용)  — OpenPLC %IX0.k = di k
    hr:N     holding register (read/write)    — dtype: int16/uint16/int32/float32_*
    ir:N     input register (읽기전용)        — 위와 동일 dtype

coalescing (§10 — read 배치 최적화는 드라이버 몫): 종류별로 offset 정렬 후
gap ≤ _COALESCE_GAP 인 이웃을 한 요청으로 병합. 왕복 수 = 태그 수가 아니라
병합 블록 수.

에러 두 층위 (Module 의 처리가 갈림):
- 전송 실패 (연결 끊김 등) → **raise** — Module 이 disconnected 처리 + 재연결.
- 프로토콜 에러 응답 (주소 없음 등) → 해당 블록 태그만 **quality=BAD** —
  PLC 는 살아 있으므로 나머지 태그는 계속 GOOD.
"""

from __future__ import annotations

import logging
import struct
from dataclasses import dataclass
from datetime import UTC, datetime

from ..contract import Quality, TagScalar
from .protocol import DType, PointSpec, TagValue, default_for_dtype

logger = logging.getLogger(__name__)

_BIT_KINDS = ("coil", "di")  # bool 1비트
_WORD_KINDS = ("hr", "ir")  # 16bit 워드
_READONLY_KINDS = ("di", "ir")

# dtype → 점유 워드 수 (비트 종류는 dtype=bool 강제라 미사용)
_WORDS_PER_DTYPE: dict[str, int] = {
    "int16": 1,
    "uint16": 1,
    "int32": 2,
    "float32_be": 2,
    "float32_le_swap": 2,
}

# 병합 허용 gap (이 이하로 떨어진 이웃 주소는 한 요청으로 — 사이 값은 버림)
_COALESCE_GAP = 8
# 요청당 최대 span (Modbus 프로토콜 한계 내 보수값: 코일 2000비트/레지스터 125워드)
_MAX_SPAN = {"coil": 1968, "di": 1968, "hr": 120, "ir": 120}


@dataclass(frozen=True)
class ParsedAddress:
    kind: str  # coil / di / hr / ir
    offset: int
    words: int  # 점유 크기 (비트 종류 = 1비트라 1)


def parse_address(address: str, dtype: DType | None, *, tag: str) -> ParsedAddress:
    """주소 문자열 검증/파싱 — 오류는 태그 이름을 담아 raise (부팅 fail-fast)."""
    kind, sep, rest = address.partition(":")
    if not sep or kind not in (*_BIT_KINDS, *_WORD_KINDS):
        raise ValueError(
            f"plc 태그 {tag!r}: 주소 {address!r} 문법 오류 — "
            f"coil:N / di:N / hr:N / ir:N (modbus_tcp 드라이버 문법)"
        )
    try:
        offset = int(rest)
    except ValueError:
        raise ValueError(
            f"plc 태그 {tag!r}: 주소 {address!r} 의 offset 이 정수가 아님"
        ) from None
    if offset < 0:
        raise ValueError(f"plc 태그 {tag!r}: 주소 {address!r} offset 은 0 이상")

    # Modbus 는 무타입 → dtype 필수 (§9.6). 종류별 궁합도 여기서 잠근다.
    if dtype is None:
        raise ValueError(
            f"plc 태그 {tag!r}: Modbus 는 무타입 프로토콜 — dtype 필수 "
            f"(coil/di 는 bool, hr/ir 는 int16/uint16/int32/float32_*)"
        )
    if kind in _BIT_KINDS:
        if dtype != "bool":
            raise ValueError(
                f"plc 태그 {tag!r}: {kind} 주소는 dtype: bool 이어야 함 (got {dtype})"
            )
        return ParsedAddress(kind=kind, offset=offset, words=1)
    if dtype == "bool":
        raise ValueError(
            f"plc 태그 {tag!r}: {kind} 주소에 dtype: bool 불가 — 레지스터는 "
            f"int16/uint16/int32/float32_be/float32_le_swap"
        )
    return ParsedAddress(kind=kind, offset=offset, words=_WORDS_PER_DTYPE[dtype])


def decode_words(words: list[int], dtype: DType) -> TagScalar:
    """레지스터 워드열 → 값. 워드 내부 byte 는 Modbus 표준(BE), 워드 순서는
    dtype 이 지정 (be = 상위 워드 먼저 / le_swap = 하위 워드 먼저 = CDAB)."""
    if dtype == "int16":
        w = words[0]
        return w - 0x10000 if w >= 0x8000 else w
    if dtype == "uint16":
        return words[0]
    if dtype == "int32":
        raw = (words[0] << 16) | words[1]
        return raw - 0x1_0000_0000 if raw >= 0x8000_0000 else raw
    if dtype == "float32_be":
        return struct.unpack(">f", struct.pack(">HH", words[0], words[1]))[0]
    if dtype == "float32_le_swap":
        return struct.unpack(">f", struct.pack(">HH", words[1], words[0]))[0]
    raise ValueError(f"레지스터 디코드 불가 dtype: {dtype}")


def encode_words(value: TagScalar, dtype: DType) -> list[int]:
    """값 → 레지스터 워드열 (decode_words 역방향)."""
    if isinstance(value, bool) or isinstance(value, str):
        raise TypeError(f"레지스터 write 값은 숫자여야 함 (got {value!r})")
    if dtype == "int16":
        iv = int(value)
        if not -0x8000 <= iv <= 0x7FFF:
            raise ValueError(f"int16 범위 밖: {iv}")
        return [iv & 0xFFFF]
    if dtype == "uint16":
        iv = int(value)
        if not 0 <= iv <= 0xFFFF:
            raise ValueError(f"uint16 범위 밖: {iv}")
        return [iv]
    if dtype == "int32":
        iv = int(value)
        if not -0x8000_0000 <= iv <= 0x7FFF_FFFF:
            raise ValueError(f"int32 범위 밖: {iv}")
        raw = iv & 0xFFFF_FFFF
        return [(raw >> 16) & 0xFFFF, raw & 0xFFFF]
    if dtype in ("float32_be", "float32_le_swap"):
        hi, lo = struct.unpack(">HH", struct.pack(">f", float(value)))
        return [hi, lo] if dtype == "float32_be" else [lo, hi]
    raise ValueError(f"레지스터 인코드 불가 dtype: {dtype}")


@dataclass(frozen=True)
class _Block:
    """병합된 read 블록 — 한 Modbus 요청."""

    start: int
    count: int
    members: tuple[tuple[str, ParsedAddress], ...]  # (태그 이름, 파싱 주소)


def coalesce(members: list[tuple[str, ParsedAddress]], *, kind: str) -> list[_Block]:
    """같은 종류의 주소를 offset 정렬 → gap ≤ _COALESCE_GAP 이웃 병합."""
    ordered = sorted(members, key=lambda m: m[1].offset)
    blocks: list[_Block] = []
    cur: list[tuple[str, ParsedAddress]] = []
    start = end = 0
    for name, parsed in ordered:
        p_end = parsed.offset + parsed.words
        if cur and (
            parsed.offset - end > _COALESCE_GAP or p_end - start > _MAX_SPAN[kind]
        ):
            blocks.append(_Block(start=start, count=end - start, members=tuple(cur)))
            cur = []
        if not cur:
            start = parsed.offset
            end = p_end
        else:
            end = max(end, p_end)
        cur.append((name, parsed))
    if cur:
        blocks.append(_Block(start=start, count=end - start, members=tuple(cur)))
    return blocks


class ModbusTcpBackend:
    def __init__(self, host: str, port: int = 502, timeout_s: float = 3.0) -> None:
        self._host = host
        self._port = port
        self._timeout_s = timeout_s
        self._client = None  # pymodbus lazy import — mock 배포가 dep 안 끌게

    # ── lifecycle ──
    def open(self) -> None:
        from pymodbus.client import ModbusTcpClient

        client = ModbusTcpClient(self._host, port=self._port, timeout=self._timeout_s)
        if not client.connect():
            client.close()
            raise ConnectionError(
                f"Modbus TCP 연결 실패: {self._host}:{self._port} — "
                f"PLC(OpenPLC Runtime) 가 RUNNING 인지 확인"
            )
        self._client = client

    def close(self) -> None:
        if self._client is not None:
            try:
                self._client.close()
            finally:
                self._client = None

    def is_connected(self) -> bool:
        return self._client is not None and bool(self._client.connected)

    # ── config 검증 ──
    def validate(self, points: dict[str, PointSpec]) -> None:
        for name, point in points.items():
            parse_address(point.address, point.dtype, tag=name)

    # ── I/O ──
    def read(self, points: dict[str, PointSpec]) -> dict[str, TagValue]:
        client = self._require_client()
        now = datetime.now(UTC)
        by_kind: dict[str, list[tuple[str, ParsedAddress]]] = {}
        dtypes: dict[str, DType] = {}
        for name, point in points.items():
            parsed = parse_address(point.address, point.dtype, tag=name)
            by_kind.setdefault(parsed.kind, []).append((name, parsed))
            assert point.dtype is not None  # parse_address 가 보장
            dtypes[name] = point.dtype

        out: dict[str, TagValue] = {}
        for kind, members in by_kind.items():
            for block in coalesce(members, kind=kind):
                # 전송 실패는 pymodbus 예외로 그대로 전파 → Module 재연결 경로
                rr = self._read_block(client, kind, block.start, block.count)
                if rr.isError():
                    # 프로토콜 에러 응답 — PLC 는 살아있음. 이 블록만 BAD.
                    logger.warning(
                        "Modbus %s read 에러 응답 (start=%d count=%d): %s",
                        kind, block.start, block.count, rr,
                    )
                    for name, parsed in block.members:
                        out[name] = TagValue(
                            value=default_for_dtype(dtypes[name]),
                            quality=Quality.BAD,
                            ts=now,
                        )
                    continue
                for name, parsed in block.members:
                    rel = parsed.offset - block.start
                    if kind in _BIT_KINDS:
                        value: TagScalar = bool(rr.bits[rel])
                    else:
                        words = list(rr.registers[rel : rel + parsed.words])
                        value = decode_words(words, dtypes[name])
                    out[name] = TagValue(value=value, quality=Quality.GOOD, ts=now)
        return out

    def write(self, point: PointSpec, value: TagScalar) -> None:
        client = self._require_client()
        parsed = parse_address(point.address, point.dtype, tag=point.address)
        if parsed.kind in _READONLY_KINDS:
            raise ValueError(
                f"주소 {point.address!r} 는 읽기전용 ({parsed.kind}) — "
                f"마스터가 쓸 수 없음 (PLC 입력은 센서/래더 소유)"
            )
        if parsed.kind == "coil":
            rr = client.write_coil(parsed.offset, bool(value))
        else:  # hr
            assert point.dtype is not None
            words = encode_words(value, point.dtype)
            rr = client.write_registers(parsed.offset, words)
        if rr.isError():
            raise RuntimeError(f"Modbus write 에러 응답 ({point.address}): {rr}")

    # ── internal ──
    def _require_client(self):
        if self._client is None:
            raise ConnectionError("Modbus TCP 미연결 — open() 선행 필요")
        return self._client

    @staticmethod
    def _read_block(client, kind: str, start: int, count: int):
        if kind == "coil":
            return client.read_coils(start, count=count)
        if kind == "di":
            return client.read_discrete_inputs(start, count=count)
        if kind == "hr":
            return client.read_holding_registers(start, count=count)
        return client.read_input_registers(start, count=count)
