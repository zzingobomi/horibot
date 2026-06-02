"""Detector 노드 토픽 / 서비스 payload schema.

토픽:
- DETECTOR_STATE              (publish) — DetectorState (YOLO raw 5fps)
- PERCEPTION_GROUNDED_STATE   (publish) — GroundedDetectionResult (호출 broadcast)

서비스 (request data / response data):
- DETECT_SERVICE              — EmptyData / DetectResult
- PERCEPTION_GROUNDED_DETECT  — GroundedDetectReq / GroundedDetectionResult
"""

from __future__ import annotations

from pydantic import ConfigDict, Field
from core.transport.messages.base import StrictModel


# ─── Topic: DETECTOR_STATE (YOLO raw 5fps) ───────────────────────────


class YoloDetection(StrictModel):
    """YOLO raw_detect 결과 한 항목. frontend 가 그대로 받음.

    `class` 는 python keyword 라 attribute 명 `cls_` + alias `"class"` 로 매핑.
    """

    model_config = ConfigDict(extra="forbid", populate_by_name=True)

    cls_: str = Field(alias="class")
    bbox: list[float]  # [x1, y1, x2, y2]
    conf: float


class DetectorState(StrictModel):
    """DETECTOR_STATE publish — YOLO raw 5fps."""

    timestamp: float
    detections: list[YoloDetection]


# ─── Service: DETECT_SERVICE (YOLO + plane Z=0) ──────────────────────


class DetectResult(StrictModel):
    """obj 의 base frame 좌표 (1개)."""

    position: list[float]  # [x, y, z] m


# ─── Service: PERCEPTION_GROUNDED_DETECT + topic broadcast ───────────


class Bbox2D(StrictModel):
    x1: float
    y1: float
    x2: float
    y2: float


class GroundedDetectReq(StrictModel):
    """open-vocabulary prompt (예: "cube", "red mug")."""

    prompt: str


class GroundedDetectionResult(StrictModel):
    """Grounding DINO 결과 + depth median + base 좌표 + height.

    PERCEPTION_GROUNDED_STATE topic 과 서비스 응답 data 양쪽에 동일 모양.
    """

    prompt: str
    position: list[float]  # 객체 윗면 base xyz (m)
    bbox2d: Bbox2D
    confidence: float
    base_z: float  # 객체 아래 책상 z (m)
    height: float  # base_z 부터 윗면까지
    timestamp: float  # ms (frontend Date.now() 호환)
