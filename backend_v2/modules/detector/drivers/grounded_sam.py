"""Grounded-SAM = GDINO(text→box) + SAM2(box→mask) — DetectorBackend 실 구현.

seam 뒤 유일한 실 어댑터 (protocol.py §0). module/DSL 은 `detect()` 만 안다 — 두 모델
합성은 여기 숨는다. RawDetection(bbox + mask + score) 반환 → module 이 projection +
geometry 로 base frame OBB 산출.

**SAM3 교체 지점**: SAM3(text→mask 단일 모델)는 gated(manual 승인). 승인되면 이 파일을
sam3 어댑터로 교체하면 GDINO 가 빠지고 나머지(projection/geometry/module)는 무변경.
transformers/torch 는 gdino/sam2 module-top import → resolve.py real branch 에서만 lazy.
"""

from __future__ import annotations

import numpy as np

from .gdino import GroundingDino
from .protocol import RawDetection
from .sam2 import Sam2Segmenter


class GroundedSamBackend:
    """GDINO box → SAM2 mask 2-stage. host당 1 + 단일 핸들러 직렬화 (동시 호출 금지)."""

    def __init__(
        self,
        gdino: GroundingDino | None = None,
        sam: Sam2Segmenter | None = None,
    ) -> None:
        self._gdino = gdino or GroundingDino()
        self._sam = sam or Sam2Segmenter()

    def preload(self) -> None:
        """두 모델 미리 로드 — 첫 detect 지연 제거. 공유 lock 으로 직렬화됨."""
        self._gdino.preload()
        self._sam.preload()

    def detect(
        self, image_bgr: np.ndarray, prompt: str, top_k: int
    ) -> list[RawDetection]:
        boxes = self._gdino.detect_boxes(image_bgr, prompt, top_k)
        if not boxes:
            return []
        masks = self._sam.segment(image_bgr, [bbox for bbox, _ in boxes])
        return [
            RawDetection(bbox=bbox, mask=mask, score=float(score))
            for (bbox, score), mask in zip(boxes, masks, strict=True)
        ]
