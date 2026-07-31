"""PlcBackend Protocol — PLC 도메인의 wire adapter 계약.

docs/plc_conveyor.md §9.3 정본. motor/drivers/protocol.py 와 동형 —
Module SDK internal, 외부 import 박지 X (TS gen / catalog viewer 대상 X).

§10 잠금 사항 (재논의 X):
- 주소 = **드라이버가 파싱하는 문자열** ("coil:1" / "DB1.DBX0.1" / "ns=2;i=5").
  드라이버별 Address 객체를 Protocol 에 두면 read() 시그니처가 갈라져
  치환 불가 → 교체성 붕괴. 검증은 validate() 부팅 훅으로 fail-fast.
- read = **배치가 primitive** — 연속 주소 병합(coalescing)은 프로토콜
  지식이라 드라이버 몫. 단건은 read({name: one}).
- write = **단건 + 실패 raise** (부작용 명령의 원자성. 배치 write 는 YAGNI).
- Modbus 용어(coil 등)를 Protocol 에 노출 금지 — 지멘스엔 coil 없음.

spec §9.3 대비 시그니처 보정 1건: points 를 list 가 아니라 **dict[태그이름 →
PointSpec]** 로 받는다 — 반환 dict 의 key 가 태그 이름이라고 spec 이 요구하는데
list 입력으로는 이름을 알 수 없음 (+ 에러 메시지에 태그 이름 필요).
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from typing import Literal, Protocol, get_args

from ..contract import Quality, TagScalar

# Modbus/S7 = 무타입 워드라 디코드에 필수. 멀티워드 엔디안은 dtype 문자열에
# 인코딩 (float32_be / float32_le_swap) — Modbus 워드순서 무표준 footgun 을
# config 에서 명시하게 강제 (§10).
DType = Literal["bool", "int16", "uint16", "int32", "float32_be", "float32_le_swap"]

_DTYPES: tuple[str, ...] = get_args(DType)


def coerce_dtype(raw: str | None, *, tag: str) -> DType | None:
    """config 의 dtype 문자열 → DType (미지 값 fail-fast — 부팅 오타 차단)."""
    if raw is None:
        return None
    if raw not in _DTYPES:
        raise ValueError(
            f"plc 태그 {tag!r}: 알 수 없는 dtype {raw!r} — 지원: {', '.join(_DTYPES)}"
        )
    return raw  # type: ignore[return-value]  # 위 멤버십 체크로 좁혀짐


def default_for_dtype(dtype: DType | None) -> TagScalar:
    """dtype 의 영값 — mock 초기값 / BAD reading 의 placeholder 값."""
    if dtype in ("float32_be", "float32_le_swap"):
        return 0.0
    if dtype in ("int16", "uint16", "int32"):
        return 0
    return False  # bool / 자기기술 프로토콜(None) 은 bool 기본


@dataclass(frozen=True)
class PointSpec:
    """태그 1개의 주소 명세 — 문법은 드라이버 소유 (validate 로 검증)."""

    address: str
    # Modbus/S7 = 필수 (무타입 워드), AB/OPC-UA = 생략 (장비가 타입 자기기술)
    dtype: DType | None = None


@dataclass
class TagValue:
    """드라이버가 읽은 값 — 값+품질+시각 (§9.2). ts = read time (UTC)."""

    value: TagScalar
    quality: Quality
    ts: datetime


class PlcBackend(Protocol):
    """PLC wire adapter — Modbus / (향후 S7 / AB / OPC-UA) / mock 의 공통 계약.

    open/close/read/write 는 **blocking sync** — Module 이 asyncio.to_thread 로
    분리한다 (async 계약 규약). 연결 단절은 raise 로 표면화 (Module 재연결)."""

    # ── lifecycle ──
    def open(self) -> None: ...  # 실패 raise — Module 이 backoff 재시도
    def close(self) -> None: ...

    # ── 관측 ──
    def is_connected(self) -> bool: ...  # 연결상태 = per-tag quality 와 별개 층위

    # ── config 검증 (부팅 훅 — 주소 오타 fail-fast) ──
    def validate(self, points: dict[str, PointSpec]) -> None: ...

    # ── I/O ──
    def read(self, points: dict[str, PointSpec]) -> dict[str, TagValue]: ...
    def write(self, point: PointSpec, value: TagScalar) -> None: ...
