"""Grounding DINO 기반 open-vocabulary detection 래퍼.

transformers / torch 의존. 첫 detect() 호출 시 모델 lazy 로드 → DetectorNode
시작 시점엔 import 비용 0. pi-motor / pi-camera 머신에선 이 모듈 자체를
import 안 함 (CLAUDE.md lazy import 패턴).
"""

from __future__ import annotations

import logging
import threading
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    import torch  # noqa: F401

logger = logging.getLogger(__name__)


DEFAULT_MODEL_ID = "IDEA-Research/grounding-dino-base"  # Swin-B
DEFAULT_BOX_THRESHOLD = 0.3
DEFAULT_TEXT_THRESHOLD = 0.25


class GroundedDetector:
    """Grounding DINO Swin-B 단일 인스턴스. detect()는 thread-safe하지 않음
    — DetectorNode 의 service 핸들러가 직렬화하므로 외부에서 동시 호출 금지.
    """

    def __init__(
        self,
        model_id: str = DEFAULT_MODEL_ID,
        box_threshold: float = DEFAULT_BOX_THRESHOLD,
        text_threshold: float = DEFAULT_TEXT_THRESHOLD,
    ) -> None:
        self._model_id = model_id
        self._box_threshold = box_threshold
        self._text_threshold = text_threshold

        self._processor = None
        self._model = None
        self._device: str | None = None
        self._load_lock = threading.Lock()

    def is_loaded(self) -> bool:
        return self._model is not None

    def preload(self) -> None:
        """노드 시작 시점에 호출해 모델을 미리 로드. 사용자 첫 detect 호출의
        체감 지연을 없앰. 백그라운드 thread 에서 호출 권장 — 모델 다운로드/로드
        가 수십 초~수 분 걸릴 수 있음."""
        self._ensure_loaded()

    def _ensure_loaded(self) -> None:
        # double-checked locking: 핫패스에서 lock acquire 없이 통과.
        if self._model is not None:
            return

        with self._load_lock:
            if self._model is not None:
                return

            import torch
            from transformers import (
                AutoModelForZeroShotObjectDetection,
                AutoProcessor,
            )

            self._device = "cuda" if torch.cuda.is_available() else "cpu"
            logger.info(
                "Grounding DINO 로드 중: %s (device=%s) — 모델 다운로드/초기화 "
                "에 수십 초 걸릴 수 있음",
                self._model_id,
                self._device,
            )
            self._processor = AutoProcessor.from_pretrained(self._model_id)
            self._model = AutoModelForZeroShotObjectDetection.from_pretrained(
                self._model_id
            ).to(self._device)
            logger.info("Grounding DINO 로드 완료")

    def detect(
        self,
        image_bgr: np.ndarray,
        prompt: str,
    ) -> tuple[tuple[float, float, float, float], float] | None:
        """이미지 + prompt → 최고 score bbox (x1, y1, x2, y2)와 confidence.

        image_bgr: HxWx3 uint8 BGR (OpenCV 컨벤션).
        prompt: 영어 자연어. Grounding DINO 는 마침표로 phrase 분리하므로
                내부에서 자동으로 마침표를 보장.
        """
        self._ensure_loaded()
        import torch
        from PIL import Image

        rgb = image_bgr[..., ::-1]
        pil_image = Image.fromarray(np.ascontiguousarray(rgb))

        text = prompt.strip().rstrip(".") + "."

        assert self._processor is not None and self._model is not None
        inputs = self._processor(
            images=pil_image,
            text=text,
            return_tensors="pt",
        ).to(self._device)

        with torch.no_grad():
            outputs = self._model(**inputs)

        # transformers >=4.51: box_threshold → threshold 로 인자명 변경.
        # 이전 버전 호환은 안 함 (pyproject.toml에서 >=4.45.0 강제 + 보통 최신).
        results = self._processor.post_process_grounded_object_detection(
            outputs,
            inputs.input_ids,
            threshold=self._box_threshold,
            text_threshold=self._text_threshold,
            target_sizes=[pil_image.size[::-1]],  # (H, W)
        )[0]

        boxes = results["boxes"].detach().cpu().numpy()
        scores = results["scores"].detach().cpu().numpy()

        if len(boxes) == 0:
            return None

        best_idx = int(scores.argmax())
        x1, y1, x2, y2 = boxes[best_idx].tolist()
        score = float(scores[best_idx])
        return (float(x1), float(y1), float(x2), float(y2)), score
