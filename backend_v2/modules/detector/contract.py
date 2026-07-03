"""Detector domain — public contract surface.

`Detect Object` = Day-1 primitive (모든 매니퓰레이션 stack 공통, 하드웨어 무관 의미).
구현체(Grounding DINO / YOLO / FoundationPose)는 adapter 뒤 — DSL·Runtime 은
"Detect Object" 만 안다 (backend_v2.md §17.1).

Day-1 = prompt → base frame 3D 위치. Top-K / height / geometric prior (§5.2) 는
실제 task 가 요구할 때 확장 (rule of three).
"""

from __future__ import annotations

from enum import StrEnum

from pydantic import BaseModel


class Detection(BaseModel):
    """검출 결과 — base frame 3D 위치 + 신뢰도.

    position: base frame (m). score: 검출 신뢰도 0..1. Day-1 = 단일 최선 후보.
    """

    prompt: str
    position: tuple[float, float, float]
    score: float


class Detector:
    class Service(StrEnum):
        # robot-agnostic (host 당 1, backend_v2.md §2.7) — robot_id 는 req field.
        # 무거운 모델(GDINO)은 1회 로드, 매 요청이 robot_id 로 그 로봇의 camera/캘/TCP 조회.
        DETECT = "srv/detector/detect"  # prompt + robot_id → base 3D 위치


class DetectRequest(BaseModel):
    robot_id: str  # 어느 로봇의 camera/캘/base frame 으로 검출할지 (host당 1 dispatch)
    prompt: str


class DetectResponse(BaseModel):
    found: bool
    detection: Detection | None = None
    message: str = ""
