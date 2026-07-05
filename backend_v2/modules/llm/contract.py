"""LLM domain — public contract surface.

자연어(한국어/영어) pick-and-place 명령 → (pick_object, place_object) 구조화.
GroundingDINO 는 영어 prompt 만 잘 먹으므로 LLM 이 **영어 객체명으로 번역** + 의도
추출 (backend_v2.md §17 NL PnP). 구현체(Qwen 등)는 adapter 뒤 (§17.1 "인터페이스 ≠
구현"). robot-agnostic (host 당 1 — 파싱은 robot 무관, §2.7).
"""

from __future__ import annotations

from enum import StrEnum

from pydantic import BaseModel


class ParsedPickPlace(BaseModel):
    """파싱된 pick-and-place 의도 — detector prompt 로 바로 쓰이는 **영어** 객체명.

    Task.RUN 의 params (pick_object/place_object) 로 그대로 전달 (frontend 가 중계).
    """

    pick_object: str
    place_object: str | None = None


class Llm:
    class Service(StrEnum):
        # robot-agnostic (host 당 1) — 파싱은 robot 무관. 무거운 모델 1회 로드.
        PARSE_COMMAND = "srv/llm/parse_command"  # 자연어 → pick/place


class ParseCommandRequest(BaseModel):
    text: str  # 사용자 자연어 명령 (한국어/영어)


class ParseCommandResponse(BaseModel):
    """ok=False 면 parsed=None + message (빈 명령 / 파싱 실패). frontend 가 사용자 안내."""

    ok: bool
    parsed: ParsedPickPlace | None = None
    message: str = ""
