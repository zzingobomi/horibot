"""Grounding DINO open-vocabulary 검출 — text → box (2-stage Grounded-SAM 의 1단계).

옛 backend/modules/detector/grounded_detector.py 를 v2 로 **재구성** (무지성 복붙 아님):
  - **box 전용 내부 헬퍼** — DetectorBackend 구현체는 grounded_sam.GroundedSamBackend
    (GDINO box + SAM2 mask). 이 파일은 그 1단계. module/DSL 은 둘 다 모른다 (seam).
  - transformers/torch 는 module-top import → grounded_sam 이 resolve.py real branch 에서만
    lazy import 된다. mock/pi 배치엔 안 끌려온다 (role 격리, motor/camera 드라이버 동형).
  - 로드는 공유 transformers_load_lock(infra/ml/loader)으로 직렬화 — NL PnP 로 Qwen
    LLM 이 두 번째 transformers 소비자로 재등장해도 preload race 구조적 차단.
  - v5: from_pretrained(device_map="auto") — dtype=auto 기본의 meta-tensor ".to(device)"
    깨짐 우회 (v5 공식 권장). 옛 low_cpu_mem_usage=False + .to(device) 대체.

torch/transformers 를 함수 안이 아니라 module-top 에 두는 이유 (옛 파일과 동일): 두
preload thread 가 동시에 lazy `from transformers import X` 하면 `_LazyModule.__getattr__`
race 로 "cannot import name". import 는 메인 스레드 직렬, 무거운 weight 로드만
_ensure_loaded (lock 안) 로 미룬다.
"""

from __future__ import annotations

import logging

import numpy as np
import torch
from PIL import Image
from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor

from infra.ml.loader import transformers_load_lock

from .protocol import Bbox

logger = logging.getLogger(__name__)

# Swin-B. 더 빠르고 작은 후보 = "IDEA-Research/grounding-dino-tiny" (Swin-T).
_DEFAULT_MODEL_ID = "IDEA-Research/grounding-dino-base"
_DEFAULT_BOX_THRESHOLD = 0.3
_DEFAULT_TEXT_THRESHOLD = 0.25


class GroundingDino:
    """Grounding DINO 단일 인스턴스. detect_boxes() 는 thread-safe 하지 않음 — module 이
    host당 1 + 단일 서비스 핸들러로 직렬화하므로 외부 동시 호출 금지.

    model_id / threshold 는 GDINO 구현 detail (Protocol 이 숨김) — 어댑터가 소유한다.
    배치별 override 필요 시 생성자 인자 (resolve.py 조립 자리). 현재 SSOT = 여기 default.
    """

    def __init__(
        self,
        model_id: str = _DEFAULT_MODEL_ID,
        box_threshold: float = _DEFAULT_BOX_THRESHOLD,
        text_threshold: float = _DEFAULT_TEXT_THRESHOLD,
    ) -> None:
        self._model_id = model_id
        self._box_threshold = box_threshold
        self._text_threshold = text_threshold
        self._processor = None
        self._model = None

    def preload(self) -> None:
        """모델을 미리 로드 — 첫 detect 지연 제거. module.start() 가 백그라운드
        thread 에서 호출 (다운로드+초기화 수십 초~수 분)."""
        self._ensure_loaded()

    def _ensure_loaded(self) -> None:
        # double-checked: 로드 완료 후 핫패스는 lock 없이 통과.
        if self._model is not None:
            return
        # 공유 lock — 미래 Qwen 등 다른 transformers 소비자와 로드 직렬화 (race 차단).
        with transformers_load_lock:
            if self._model is not None:
                return
            logger.info(
                "Grounding DINO 로드 중: %s (device_map=auto, cuda=%s) — "
                "다운로드/초기화 수십 초~수 분",
                self._model_id,
                torch.cuda.is_available(),
            )
            self._processor = AutoProcessor.from_pretrained(self._model_id)
            # device_map="auto": 단일 GPU 면 통째로 GPU, 없으면 CPU. v5 dtype=auto 의
            # meta-tensor ".to(device)" 깨짐을 우회 (accelerate 가 device 배치 담당).
            self._model = AutoModelForZeroShotObjectDetection.from_pretrained(
                self._model_id, device_map="auto"
            )
            logger.info("Grounding DINO 로드 완료 (device=%s)", self._model.device)

    def detect_boxes(
        self, image_bgr: np.ndarray, prompt: str, top_k: int
    ) -> list[tuple[Bbox, float]]:
        """BGR 이미지 + 영어 prompt → score 내림차순 Top-K [(bbox, score)]. 미검출 [].

        image_bgr: HxWx3 uint8 BGR (OpenCV). Grounding DINO 는 마침표로 phrase 분리
        → 내부에서 마침표 보장. Top-K (§17.5) — 최종 선택은 소비자(task SelectTarget).
        SAM2 는 이 box 를 prompt 로 mask 를 뽑는다 (grounded_sam).
        """
        self._ensure_loaded()
        assert self._processor is not None and self._model is not None

        rgb = image_bgr[..., ::-1]
        pil_image = Image.fromarray(np.ascontiguousarray(rgb))
        text = prompt.strip().rstrip(".") + "."

        inputs = self._processor(
            images=pil_image, text=text, return_tensors="pt"
        ).to(self._model.device)

        with torch.no_grad():
            outputs = self._model(**inputs)

        # v5: input_ids 는 optional (없으면 outputs 에서 취함) 이나 명시 전달 (옛 동형).
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
            return []

        # score 내림차순 Top-K — 진짜 물체가 2등이어도 누락 안 되게 상위 후보 모두 반환.
        order = np.argsort(scores)[::-1][: max(1, top_k)]
        return [
            (
                (
                    float(boxes[i][0]),
                    float(boxes[i][1]),
                    float(boxes[i][2]),
                    float(boxes[i][3]),
                ),
                float(scores[i]),
            )
            for i in order
        ]
