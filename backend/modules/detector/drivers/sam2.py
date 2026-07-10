"""SAM2 (Segment Anything 2) — box → mask (2-stage Grounded-SAM 의 2단계).

GDINO 가 준 box 를 prompt 로 물체를 픽셀 단위 mask 로 분리한다. mask 는
projection→geometry 로 base frame OBB(grasp yaw / footprint)의 소스. **box 전용 내부
헬퍼** — DetectorBackend 구현체는 grounded_sam.GroundedSamBackend (이 파일은 2단계).

로드 규약은 gdino.py 와 동일: transformers/torch module-top import (lazy `from` race
회피) + 무거운 weight 로드만 공유 transformers_load_lock 으로 직렬화 (GDINO·미래 Qwen
과 동시 from_pretrained 차단, docs/perception.md). device_map="auto".

facebook/sam2.1-hiera-large = 오픈(ungated). SAM3 는 gated(manual 승인) 이라 이 seam
뒤 교체 대상 — 승인 후 sam3.py 로 갈아끼움 (module/DSL 무변경).
"""

from __future__ import annotations

import logging

import numpy as np
import torch
from PIL import Image
from transformers import Sam2Model, Sam2Processor

from infra.ml.loader import transformers_load_lock

from .protocol import Bbox

logger = logging.getLogger(__name__)

# ungated (HF gated=False). 경량 후보 = "facebook/sam2.1-hiera-tiny".
_DEFAULT_MODEL_ID = "facebook/sam2.1-hiera-large"


class Sam2Segmenter:
    """SAM2 단일 인스턴스. segment() 는 thread-safe 하지 않음 — module 이 host당 1 +
    단일 서비스 핸들러로 직렬화하므로 외부 동시 호출 금지.

    model_id 는 SAM2 구현 detail — 배치별 override 는 생성자 인자 (resolve/grounded_sam
    조립 자리). 현재 SSOT = 여기 default.
    """

    def __init__(self, model_id: str = _DEFAULT_MODEL_ID) -> None:
        self._model_id = model_id
        self._processor = None
        self._model = None

    def preload(self) -> None:
        """모델을 미리 로드 — 첫 segment 지연 제거 (grounded_sam.preload 가 호출)."""
        self._ensure_loaded()

    def _ensure_loaded(self) -> None:
        if self._model is not None:
            return
        with transformers_load_lock:
            if self._model is not None:
                return
            logger.info(
                "SAM2 로드 중: %s (device_map=auto, cuda=%s)",
                self._model_id,
                torch.cuda.is_available(),
            )
            self._processor = Sam2Processor.from_pretrained(self._model_id)
            self._model = Sam2Model.from_pretrained(
                self._model_id, device_map="auto"
            )
            logger.info("SAM2 로드 완료 (device=%s)", self._model.device)

    def segment(
        self, image_bgr: np.ndarray, boxes: list[Bbox]
    ) -> list[np.ndarray]:
        """BGR 이미지 + box prompt 목록 → box 별 bool mask (H, W). box 순서 유지.

        boxes: [(x1,y1,x2,y2), ...] px. 반환 mask[i] 는 boxes[i] 안 물체의 픽셀.
        빈 boxes → []. multimask_output=False (box 가 잘 국소화 → 단일 mask).
        """
        if not boxes:
            return []
        self._ensure_loaded()
        assert self._processor is not None and self._model is not None

        rgb = image_bgr[..., ::-1]
        pil_image = Image.fromarray(np.ascontiguousarray(rgb))
        # processor input_boxes 규약: [batch][boxes][4].
        input_boxes = [[[float(v) for v in box] for box in boxes]]

        inputs = self._processor(
            images=pil_image, input_boxes=input_boxes, return_tensors="pt"
        ).to(self._model.device)

        with torch.no_grad():
            outputs = self._model(**inputs, multimask_output=False)

        # post_process_masks: 저해상 pred_masks → 원본 크기 + binarize(bool).
        # 반환 = 이미지별 list, 각 (num_boxes, num_masks=1, H, W).
        masks = self._processor.post_process_masks(
            outputs.pred_masks, inputs["original_sizes"]
        )[0]
        arr = np.asarray(masks.cpu())
        if arr.ndim == 4:  # (num_boxes, 1, H, W) → mask-per-box 차원 squeeze
            arr = arr[:, 0, :, :]
        return [arr[i].astype(bool) for i in range(arr.shape[0])]
