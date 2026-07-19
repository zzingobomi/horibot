"""Grounded-SAM = GDINO(text→box) + SAM2(box→mask) — DetectorBackend 실 구현.

seam 뒤 유일한 실 어댑터 (protocol.py §0). module/DSL 은 `detect()` 만 안다 — 두 모델
합성은 여기 숨는다. RawDetection(bbox + mask + score) 반환 → module 이 projection +
geometry 로 base frame OBB 산출.

**SAM3 교체 지점**: SAM3(text→mask 단일 모델)는 gated(manual 승인). 승인되면 이 파일을
sam3 어댑터로 교체하면 GDINO 가 빠지고 나머지(projection/geometry/module)는 무변경.
transformers/torch 는 gdino/sam2 module-top import → resolve.py real branch 에서만 lazy.
"""

from __future__ import annotations

import logging
from collections.abc import Sequence

import numpy as np

from .gdino import GroundingDino
from .protocol import Bbox, RawDetection
from .sam2 import Sam2Segmenter

logger = logging.getLogger(__name__)


class GroundedSamBackend:
    """GDINO box → SAM2 mask 2-stage. host당 1 + 단일 핸들러 직렬화 (동시 호출 금지).

    멀티 프롬프트 추론 전략 (2026-07-19 — wire 는 "무엇을 찾을지"만 안다):
    - 기본 = prompt 별 **단독 추론 N-loop** — 단일 prompt 시절과 score 가 바이트
      단위로 동일 (실측 튜닝된 score 컷이 흔들리지 않는다).
    - joint_inference=True (deployment detector_joint_inference) = GDINO 1-forward
      합동 쿼리 — 추론 N회 → 1회. 합동 score 는 단독과 다를 수 있어 **기본 off**,
      켜기 전 scripts/compare_joint_prompt_scores.py 로 실물 덤프 분포 확인.
    어느 경로든 SAM2 mask 는 모든 box 를 한 번에 (box 수는 동일 — SAM 비용 불변).
    """

    def __init__(
        self,
        gdino: GroundingDino | None = None,
        sam: Sam2Segmenter | None = None,
        joint_inference: bool = False,
    ) -> None:
        self._gdino = gdino or GroundingDino()
        self._sam = sam or Sam2Segmenter()
        self._joint_inference = joint_inference

    def preload(self) -> None:
        """두 모델 미리 로드 — 첫 detect 지연 제거. 공유 lock 으로 직렬화됨."""
        self._gdino.preload()
        self._sam.preload()

    def detect(
        self, image_bgr: np.ndarray, prompts: Sequence[str], top_k: int
    ) -> list[RawDetection]:
        plist = [p for p in (s.strip() for s in prompts) if p]
        if not plist:
            return []
        if self._joint_inference and len(plist) > 1:
            try:
                triples = self._gdino.detect_boxes_joint(image_bgr, plist, top_k)
            except Exception:
                # 합동 경로는 opt-in 실험 경로 — 죽으면 run 을 살리는 게 우선.
                # 침묵 아님 (exception 로그) + 폴백은 의미 동일한 단독 N-loop.
                logger.exception(
                    "GDINO 합동 추론 실패 — 단독 N-loop 폴백 (이번 호출만)"
                )
                triples = self._loop_singles(image_bgr, plist, top_k)
        else:
            triples = self._loop_singles(image_bgr, plist, top_k)
        if not triples:
            return []
        masks = self._sam.segment(image_bgr, [bbox for bbox, _, _ in triples])
        return [
            RawDetection(bbox=bbox, mask=mask, score=float(score), prompt=prompt)
            for (bbox, score, prompt), mask in zip(triples, masks, strict=True)
        ]

    def _loop_singles(
        self, image_bgr: np.ndarray, prompts: list[str], top_k: int
    ) -> list[tuple[Bbox, float, str]]:
        """기본 경로 — prompt 별 단독 추론 (기존 단일 prompt 와 score 동일)."""
        out: list[tuple[Bbox, float, str]] = []
        for prompt in prompts:
            out.extend(
                (bbox, score, prompt)
                for bbox, score in self._gdino.detect_boxes(
                    image_bgr, prompt, top_k
                )
            )
        return out
