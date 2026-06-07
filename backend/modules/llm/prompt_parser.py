"""task prompt 의 자연어 → (pick_object, place_object) 분리.

HuggingFace transformers + Qwen2.5-1.5B-Instruct 로컬 로드. GroundingDINO 와
같은 패턴 — 첫 호출 시 자동 다운로드 + 모델 캐시 (~3GB), 이후 메모리 상주.

GPU 우선 (RTX 3060 등), 없으면 CPU. inference 1-2초/호출 (GPU).

사용자는 한국어/영어 자유롭게 박을 수 있고, GroundingDINO 가 한국어 약하므로
모델한테 *영어로 객체 이름 추출* 시킴.

예시:
    "흰 큐브 들어서 파란 박스에 놔" → ("white cube", "blue box")
    "pick the red ball and put it on the green tray" → ("red ball", "green tray")
    "그냥 큐브 들어"                  → ("cube", None)
"""

from __future__ import annotations

import json
import logging
import re
import threading

# transformers / torch 는 module-top import — detector_node 의 GroundedDetector
# preload 스레드와 동시에 lazy `from transformers import X` 가 돌면
# `_LazyModule.__getattr__` race 로 "cannot import name" 발생.
# 모듈 import 는 메인 스레드 직렬 실행이므로 여기서 미리 resolve.
# weight 로드 (수 GB) 는 여전히 `_ensure_loaded` 시점에 lazy.
import torch
from transformers import AutoModelForCausalLM, AutoTokenizer

logger = logging.getLogger(__name__)

# Qwen2.5-1.5B-Instruct — 한국어/영어 다국어, instruction-tuned, 작음(~3GB).
# 부족하면 "Qwen/Qwen2.5-3B-Instruct" 또는 "microsoft/Phi-3.5-mini-instruct" 로 교체.
_MODEL_ID = "Qwen/Qwen2.5-1.5B-Instruct"

_SYSTEM = """You parse pick-and-place robot commands from natural language (Korean or English).

Output a single JSON object on one line with exactly these keys:
  - "pick":  English object name to pick up (short noun phrase)
  - "place": English object name to place onto, or null if the command has no destination

Rules:
- Always translate object names to English (downstream detector only handles English).
- Use lowercase, concise descriptions (e.g. "white cube", "blue cabinet", "red ball").
- If only a pick is requested (no destination), set "place" to null.
- Output JSON only — no explanation, no markdown fences."""


# 모델 / tokenizer 싱글톤 — 첫 호출에서 로드, 이후 재사용.
_lock = threading.Lock()
_model = None
_tokenizer = None
_device: str | None = None


def _ensure_loaded() -> bool:
    """모델 / tokenizer 를 lazy load. 실패 시 False."""
    global _model, _tokenizer, _device
    if _model is not None:
        return True
    with _lock:
        if _model is not None:
            return True
        try:
            _device = "cuda" if torch.cuda.is_available() else "cpu"
            logger.info(
                "LLM 모델 로드 시작: %s (device=%s)", _MODEL_ID, _device
            )
            _tokenizer = AutoTokenizer.from_pretrained(_MODEL_ID)
            # low_cpu_mem_usage=False 강제 — transformers 4.56 default(True) 의
            # meta-init path 에서 Qwen tied weight (embed_tokens ↔ lm_head) 가
            # 가끔 meta 인 채 남아 .to(device) / dispatch_model 단계에서
            # "Cannot copy out of meta tensor" 터짐. 1.5B 면 메모리 부담 없음.
            _model = AutoModelForCausalLM.from_pretrained(
                _MODEL_ID,
                dtype=torch.float16 if _device == "cuda" else torch.float32,
                low_cpu_mem_usage=False,
            ).to(_device)  # type: ignore[arg-type]
            logger.info("LLM 모델 로드 완료")
            return True
        except Exception as exc:
            logger.exception("LLM 모델 로드 실패: %s", exc)
            _model = None
            _tokenizer = None
            return False


def preload() -> None:
    """task_node.start() 에서 백그라운드 thread 로 호출 — 첫 호출 지연 제거.

    [detector_node 의 GroundedDetector.preload()](backend/nodes/detector_node.py) 와
    같은 패턴. parse_pick_place 의 _ensure_loaded 와 lock 공유.
    """
    _ensure_loaded()


def parse_pick_place(command: str) -> tuple[str, str | None]:
    """prompt → (pick_object, place_object). 실패 시 (command, None) fallback."""
    cmd = command.strip()
    if not cmd:
        return cmd, None

    if not _ensure_loaded():
        logger.warning("LLM 미로드 — prompt 를 그대로 pick 으로 사용")
        return cmd, None

    assert _model is not None and _tokenizer is not None

    try:
        messages = [
            {"role": "system", "content": _SYSTEM},
            {"role": "user", "content": cmd},
        ]
        text = _tokenizer.apply_chat_template(
            messages, tokenize=False, add_generation_prompt=True
        )
        inputs = _tokenizer(text, return_tensors="pt").to(_device)
        out = _model.generate(
            **inputs,
            max_new_tokens=128,
            do_sample=False,
            pad_token_id=_tokenizer.eos_token_id,
        )
        # generated portion 만 디코드
        generated = out[0][inputs["input_ids"].shape[1]:]
        response = _tokenizer.decode(generated, skip_special_tokens=True).strip()
    except Exception as exc:
        logger.warning("LLM inference 실패: %s — fallback", exc)
        return cmd, None

    # JSON 본문 추출 (모델이 markdown 으로 감싸도 대응)
    match = re.search(r"\{.*\}", response, re.DOTALL)
    if not match:
        logger.warning("LLM 응답에서 JSON 못 찾음: %r — fallback", response)
        return cmd, None

    try:
        data = json.loads(match.group())
    except json.JSONDecodeError as exc:
        logger.warning("LLM JSON parse 실패: %s — fallback", exc)
        return cmd, None

    pick_raw = data.get("pick")
    if not isinstance(pick_raw, str) or not pick_raw.strip():
        logger.warning("LLM 응답에 pick 없음: %r — fallback", data)
        return cmd, None
    pick = pick_raw.strip()

    place_raw = data.get("place")
    if isinstance(place_raw, str) and place_raw.strip():
        place: str | None = place_raw.strip()
    else:
        place = None

    logger.info("LLM parse: '%s' → pick='%s' place=%s", cmd, pick, place)
    return pick, place
