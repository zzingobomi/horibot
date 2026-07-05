"""DetectorBackend Protocol — Detector 도메인의 검출 구현체 adapter 계약.

DetectorModule 은 이 Protocol 만 안다 (§0 "인터페이스 ≠ 구현"). 실 구현(Grounding
DINO open-vocab / YOLO / FoundationPose)은 뒤에 숨는다 — 모델 교체가 module/DSL 을
안 건드림. motor.drivers.protocol / camera.drivers.protocol 과 동형.

Module SDK internal — 외부 import 박지 X (TS gen / catalog viewer read 대상 X, §8.2).
"""

from __future__ import annotations

from typing import Protocol

import numpy as np

# (x1, y1, x2, y2) px — image 좌표계 bbox.
Bbox = tuple[float, float, float, float]


class DetectorBackend(Protocol):
    def detect(
        self, image_bgr: np.ndarray, prompt: str, top_k: int
    ) -> list[tuple[Bbox, float]]:
        """color image + prompt → score 내림차순 Top-K [(bbox, score)]. 미검출 = [].

        Top-K (§17.5 ①) — 진짜 물체가 2등이면 top-1 만으로 영원히 누락되므로 상위
        후보를 모두 올리고, prompt 매칭 + 기하 prior 최종 선택은 소비자(task
        SelectTarget) 가 한다. 최대 top_k 개, score desc 정렬.
        """
        ...

    def preload(self) -> None:
        """모델을 미리 로드 — 첫 detect 지연 제거. 로드할 게 없는 구현(mock)은 no-op."""
        ...
