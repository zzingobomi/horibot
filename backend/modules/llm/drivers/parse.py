"""모델 응답 텍스트 파싱 — 순수 함수 (torch/transformers 무관).

qwen.py 에서 분리한 이유: "모델 없이 단위테스트" 가 원 설계 의도인데 함수가
heavy driver 모듈 안에 있어 테스트 import 만으로 transformers/torch 로드(~20s)
를 물었다 (2026-07-17 테스트 정리에서 발견). 소비자 = QwenBackend.parse + 테스트.
"""

from __future__ import annotations

import json
import logging
import re

from .protocol import ParsedCommand

logger = logging.getLogger(__name__)


def parse_json_response(response: str) -> ParsedCommand | None:
    """모델 응답 텍스트 → ParsedCommand. JSON 못 찾거나 pick 없으면 None.

    모델이 markdown fence 로 감싸도 첫 {...} 블록 추출. pick 필수, place 없으면 None.
    """
    match = re.search(r"\{.*\}", response, re.DOTALL)
    if not match:
        logger.warning("Qwen 응답에서 JSON 못 찾음: %r", response)
        return None
    try:
        data = json.loads(match.group())
    except json.JSONDecodeError as exc:
        logger.warning("Qwen JSON parse 실패: %s (%r)", exc, response)
        return None
    if not isinstance(data, dict):
        return None
    pick = data.get("pick")
    if not isinstance(pick, str) or not pick.strip():
        logger.warning("Qwen 응답에 pick 없음: %r", data)
        return None
    place_raw = data.get("place")
    place = (
        place_raw.strip()
        if isinstance(place_raw, str) and place_raw.strip()
        else None
    )
    return ParsedCommand(pick=pick.strip(), place=place)
