"""DetectorBackend — 검출 구현체 adapter (§0 "인터페이스 ≠ 구현").

DetectorModule 은 이 Protocol 만 안다. 실 구현(Grounding DINO open-vocab / YOLO /
FoundationPose)은 뒤에 숨는다 — 모델 교체가 module/DSL 을 안 건드림.
motor backend / camera capture adapter 와 동형.
"""

from __future__ import annotations

from typing import Protocol

import numpy as np

# (x1, y1, x2, y2) px — image 좌표계 bbox.
Bbox = tuple[float, float, float, float]


class DetectorBackend(Protocol):
    def detect(
        self, image_bgr: np.ndarray, prompt: str
    ) -> tuple[Bbox, float] | None:
        """color image + prompt → (bbox, score). 미검출이면 None.

        Day-1 = 단일 최선 후보. Top-K (§5.2) 는 실제 task 요구 시 확장.
        """
        ...


class MockDetectorBackend:
    """합성 검출 — image 중앙 고정 bbox 반환 (prompt 무관). wiring/투영 e2e 검증용.

    실 모델 없이 DETECT 파이프라인(camera→adapter→projection→base)을 회사에서 검증.
    """

    def __init__(self, box_frac: float = 0.2) -> None:
        self._frac = box_frac

    def detect(self, image_bgr: np.ndarray, prompt: str) -> tuple[Bbox, float] | None:
        h, w = image_bgr.shape[:2]
        bw, bh = w * self._frac, h * self._frac
        cx, cy = w / 2.0, h / 2.0
        return (cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2), 0.99
