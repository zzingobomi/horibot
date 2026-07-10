"""DetectorBackend Protocol — Detector 도메인의 검출 구현체 adapter 계약.

DetectorModule 은 이 Protocol 만 안다 (§0 "인터페이스 ≠ 구현"). 실 구현(Grounding
DINO open-vocab / YOLO / FoundationPose)은 뒤에 숨는다 — 모델 교체가 module/DSL 을
안 건드림. motor.drivers.protocol / camera.drivers.protocol 과 동형.

Module SDK internal — 외부 import 박지 X (TS gen / catalog viewer read 대상 X, §8.2).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np

# (x1, y1, x2, y2) px — image 좌표계 bbox.
Bbox = tuple[float, float, float, float]


@dataclass(frozen=True, slots=True, eq=False)
class RawDetection:
    """검출 구현체의 원출력 — projection/geometry 전, SDK internal (wire 아님).

    tuple `(bbox, mask, score)` 대신 구조체 — 필드(label/logits 등)가 자라도 호출부
    positional 이 안 깨진다. `mask` 는 np.ndarray 라 pydantic 부적합 + wire 로 안 나감
    (무거움) → 모듈 내부에서만 흐른다. wire 계약은 base frame 최종값(Detection).

    eq=False: ndarray 필드의 __eq__/__hash__ 모호성(elementwise) 회피 — 동일성 비교만.
    """

    bbox: Bbox  # (x1,y1,x2,y2) px — frontend 오버레이 / ROI
    mask: np.ndarray  # bool (H, W) — 물체 픽셀 (geometry OBB 소스)
    score: float


class DetectorBackend(Protocol):
    def detect(
        self, image_bgr: np.ndarray, prompt: str, top_k: int
    ) -> list[RawDetection]:
        """color image + prompt → score 내림차순 Top-K [RawDetection]. 미검출 = [].

        각 후보는 bbox(어디) + mask(픽셀 단위 형상) + score. mask 는 base frame OBB
        (grasp yaw / footprint) 계산의 소스 — 픽셀 각도가 아니라 depth 로 base 3D 를
        구하므로 원근 왜곡 없음 (geometry.py).

        Top-K (§17.5 ①) — 진짜 물체가 2등이면 top-1 만으로 영원히 누락되므로 상위
        후보를 모두 올리고, prompt 매칭 + 기하 prior 최종 선택은 소비자(task
        SelectTarget) 가 한다. 최대 top_k 개, score desc 정렬.
        """
        ...

    def preload(self) -> None:
        """모델을 미리 로드 — 첫 detect 지연 제거. 로드할 게 없는 구현(mock)은 no-op."""
        ...
