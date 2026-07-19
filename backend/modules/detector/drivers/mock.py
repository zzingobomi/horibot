"""MockDetectorBackend — 합성 검출 (실 모델 없이 DETECT wiring/투영 e2e 검증).

motor.drivers.mock / camera.drivers.mock 과 동형 — Protocol 만 충족, 하드웨어/모델 0.
"""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np

from .protocol import Bbox, RawDetection


class MockDetectorBackend:
    """합성 검출 — prompt 당 중앙(high score) + 우상단(low score) 후보 2개 반환.

    실 모델 없이 DETECT/DETECT_ORIENTED 파이프라인(camera→adapter→Top-K→후보별
    projection→base→geometry)을 회사에서 검증. 후보 2개 = Top-K wiring + 후보 누적 +
    score desc 정렬 검증용. mask = bbox 를 채운 사각형(bool) — geometry OBB 소스.
    멀티 프롬프트: prompt 마다 같은 두 후보를 그 prompt 로 귀속해 반환 — per-prompt
    Top-K + 귀속 배선 검증용 (N-loop 기본 구현과 동형).
    """

    def __init__(self, box_frac: float = 0.2) -> None:
        self._frac = box_frac

    def detect(
        self, image_bgr: np.ndarray, prompts: Sequence[str], top_k: int
    ) -> list[RawDetection]:
        h, w = image_bgr.shape[:2]
        bw, bh = w * self._frac, h * self._frac
        cx, cy = w / 2.0, h / 2.0  # 중앙 (high)
        ox, oy = w * 0.7, h * 0.3  # 우상단 (low)
        boxes: list[tuple[Bbox, float]] = [
            ((cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2), 0.95),
            ((ox - bw / 2, oy - bh / 2, ox + bw / 2, oy + bh / 2), 0.60),
        ]
        out: list[RawDetection] = []
        for prompt in prompts:
            for bbox, score in boxes[: max(1, top_k)]:
                mask = np.zeros((h, w), dtype=bool)
                x1, y1, x2, y2 = (int(round(v)) for v in bbox)
                mask[y1:y2, x1:x2] = True  # bbox 채운 사각형
                out.append(
                    RawDetection(bbox=bbox, mask=mask, score=score, prompt=prompt)
                )
        return out

    def preload(self) -> None:
        """mock 은 로드할 모델 없음 — no-op (Protocol 충족)."""
