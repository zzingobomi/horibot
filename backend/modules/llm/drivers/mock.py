"""MockLlmBackend — 고정 파싱 stub (실 모델 없이 PARSE_COMMAND wiring / UX 검증).

detector.drivers.mock 동형 — Protocol 만 충족, 모델 0. 실 한국어→영어 번역은 Qwen.
기본이 pick+place 둘 다 반환 → mock e2e 가 full PnP(집기+놓기) flow 를 타게.
"""

from __future__ import annotations

from .protocol import ParsedCommand


class MockLlmBackend:
    """고정 ParsedCommand 반환 (실 파싱 아님 — 모델 부재). 생성자로 canned 값 조절."""

    def __init__(
        self, pick: str = "white cube", place: str | None = "blue box"
    ) -> None:
        self._parsed = ParsedCommand(pick=pick, place=place)

    def parse(self, text: str) -> ParsedCommand | None:
        return self._parsed if text.strip() else None

    def preload(self) -> None:
        """mock 은 로드할 모델 없음 — no-op (Protocol 충족)."""
