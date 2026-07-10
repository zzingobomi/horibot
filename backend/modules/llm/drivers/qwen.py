"""Qwen2.5 자연어 파서 backend — LlmBackend 구현.

옛 backend/modules/llm/prompt_parser.py 를 v2 어댑터로 재구성 (무지성 복붙 X):
  - 계약 = LlmBackend Protocol(parse / preload)만 만족 (모델 교체 seam).
  - transformers/torch 는 module-top import → resolve.py real branch 에서만 lazy import
    (role 격리, gdino 동형). mock/pi 배치엔 안 끌려온다.
  - 로드는 **공유 transformers_load_lock**(infra/ml/loader)으로 GDINO 와 직렬화 — 두
    preload thread 가 동시에 weight 로드 시 meta-tensor race 차단 (CLAUDE.md
    llm_preload_race_debug). 옛 파일의 private lock 대체 = GDINO 와 같은 lock.
  - v5: from_pretrained(device_map="auto") — dtype=auto 기본의 meta-tensor ".to(device)"
    깨짐 우회 (accelerate 가 배치, gdino 동형). 옛 low_cpu_mem_usage=False + .to 대체.

JSON 파싱(_parse_json_response)은 순수 함수 분리 — 모델 없이 단위테스트 (옛 inline 은
테스트 불가였음).
"""

from __future__ import annotations

import json
import logging
import re
from typing import Any

import torch
from transformers import AutoModelForCausalLM, AutoTokenizer

from infra.ml.loader import transformers_load_lock

from .protocol import ParsedCommand

logger = logging.getLogger(__name__)

# Qwen2.5-1.5B-Instruct — 한국어/영어 다국어, instruction-tuned, 작음(~3GB).
# 부족하면 "Qwen/Qwen2.5-3B-Instruct" 로 교체 (생성자 인자, resolve 조립 자리).
_DEFAULT_MODEL_ID = "Qwen/Qwen2.5-1.5B-Instruct"

_SYSTEM = """You parse pick-and-place robot commands from natural language (Korean or English).

Output a single JSON object on one line with exactly these keys:
  - "pick":  English object name to pick up (short noun phrase)
  - "place": English object name to place onto, or null if the command has no destination

Rules:
- Always translate object names to English (downstream detector only handles English).
- Use lowercase, concise descriptions (e.g. "white cube", "blue cabinet", "red ball").
- If only a pick is requested (no destination), set "place" to null.
- Output JSON only — no explanation, no markdown fences."""


class QwenBackend:
    """Qwen2.5 단일 인스턴스. parse() 는 thread-safe 하지 않음 — module 이 host당 1 +
    단일 서비스 핸들러로 직렬화. model_id 는 구현 detail (Protocol 이 숨김)."""

    def __init__(self, model_id: str = _DEFAULT_MODEL_ID) -> None:
        self._model_id = model_id
        # transformers v5 스텁이 .generate / tokenizer __call__ 을 정밀 타입 못 줌 →
        # adapter 뒤 opaque handle 이라 Any (모델 교체 seam, 타입 friction 회피).
        self._tokenizer: Any = None
        self._model: Any = None

    def preload(self) -> None:
        """모델 미리 로드 — module.start() 가 백그라운드 thread 에서 호출."""
        self._ensure_loaded()

    def _ensure_loaded(self) -> None:
        if self._model is not None:  # double-checked: 핫패스는 lock 없이 통과.
            return
        with transformers_load_lock:  # GDINO 와 공유 — 동시 weight 로드 race 차단.
            if self._model is not None:
                return
            logger.info(
                "Qwen 로드 중: %s (device_map=auto, cuda=%s) — 다운로드/초기화 "
                "수십 초~수 분",
                self._model_id,
                torch.cuda.is_available(),
            )
            self._tokenizer = AutoTokenizer.from_pretrained(self._model_id)
            self._model = AutoModelForCausalLM.from_pretrained(
                self._model_id, device_map="auto"
            )
            logger.info("Qwen 로드 완료 (device=%s)", self._model.device)

    def parse(self, text: str) -> ParsedCommand | None:
        cmd = text.strip()
        if not cmd:
            return None
        self._ensure_loaded()
        assert self._tokenizer is not None and self._model is not None

        messages = [
            {"role": "system", "content": _SYSTEM},
            {"role": "user", "content": cmd},
        ]
        prompt = self._tokenizer.apply_chat_template(
            messages, tokenize=False, add_generation_prompt=True
        )
        inputs = self._tokenizer(prompt, return_tensors="pt").to(self._model.device)
        with torch.no_grad():
            out = self._model.generate(
                **inputs,
                max_new_tokens=128,
                do_sample=False,
                pad_token_id=self._tokenizer.eos_token_id,
            )
        generated = out[0][inputs["input_ids"].shape[1] :]
        response = self._tokenizer.decode(
            generated, skip_special_tokens=True
        ).strip()
        return _parse_json_response(response)


def _parse_json_response(response: str) -> ParsedCommand | None:
    """모델 응답 텍스트 → ParsedCommand. JSON 못 찾거나 pick 없으면 None (순수·테스트가능).

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
