"""LlmBackend Protocol — 자연어 파서 구현체 adapter 계약.

LlmModule 은 이 Protocol 만 안다 (§17.1 "인터페이스 ≠ 구현"). 실 구현(Qwen2.5 등)은
뒤에 숨는다 — 모델 교체가 module/DSL 을 안 건드림. detector.drivers.protocol 동형.

Module SDK internal — 외부 import 박지 X (TS gen / catalog viewer read 대상 X).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True)
class ParsedCommand:
    """파서 출력 — pick(영어 객체명) + place(영어, destination 없으면 None)."""

    pick: str
    place: str | None = None


class LlmBackend(Protocol):
    def parse(self, text: str) -> ParsedCommand | None:
        """자연어 명령 → ParsedCommand. 파싱 실패(pick 추출 불가)면 None."""
        ...

    def preload(self) -> None:
        """모델 미리 로드 — 첫 parse 지연 제거. 로드할 게 없는 구현(mock)은 no-op."""
        ...
