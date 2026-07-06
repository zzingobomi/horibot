"""Detector domain — public contract surface.

`Detect Object` = Day-1 primitive (모든 매니퓰레이션 stack 공통, 하드웨어 무관 의미).
구현체(Grounding DINO / YOLO / FoundationPose)는 adapter 뒤 — DSL·Runtime 은
"Detect Object" 만 안다 (backend_v2.md §17.1).

prompt → base frame 3D 후보 **Top-K** (§17.5 ①). 후보별 기하 속성(base_z=position[2],
size_m) 을 제공하고, prior 적용/최종 선택은 소비자(task `SelectTarget(candidates,
prompt, priors)`, §17.5) 책임 — detector 는 "예상 범위" 를 모른다 (계층 분리). multi-view
3D 합의는 후속 (후보 누적 구조만 먼저, §17.5 ③).
"""

from __future__ import annotations

from enum import StrEnum

from pydantic import BaseModel


class Detection(BaseModel):
    """검출 후보 — base frame 3D 위치 + 기하 prior 속성 (§17.5, v1 GroundedDetection 포팅).

    position: 물체 **윗면 중심** base frame (m). base_z: 물체 **주변 책상/바닥** 의 base-z
      (bbox 외곽 ring depth percentile). height: 물체 높이 = position[2] - base_z. §17.5
      ② "height/base_z" 로 예상 범위 밖 후보 reject (confidence 무관 — 예: 테이블 큐브 vs
      바닥 천). GraspPolicy 가 base_z + height 로 옆면 grasp z 계산 (§17.5 순수 계산).
      예상 범위 임계는 실물 tuning (§17.5 "스코어링 = 집 하드웨어"). score: 신뢰도 0..1.
    bbox_2d: 검출 시점 color 이미지의 픽셀 bbox (x1,y1,x2,y2) — frontend 카메라
      오버레이용 (DetectionsUpdate.image_width/height 기준 좌표).
    """

    prompt: str
    position: tuple[float, float, float]
    score: float
    base_z: float
    height: float
    bbox_2d: tuple[float, float, float, float] | None = None


class DetectionsUpdate(BaseModel):
    """DETECT 1회의 결과 스냅샷 — frontend 카메라 오버레이 (v1 DETECTOR_STATE 계승).

    v1 은 5fps 연속 검출 loop 였으나 v2 detector 는 on-demand — **DETECT 호출
    시마다** publish (task 의 search 자세에서 팔이 멈춰 검출하는 순간 = 오버레이가
    유효한 순간, GPU 추가 비용 0). 팔이 움직이면 stale — frontend 가 timestamp 로
    fade. found=False 여도 빈 candidates 로 publish (오버레이 clear).
    """

    robot_id: str
    seq: int
    timestamp_unix: float
    prompt: str
    image_width: int
    image_height: int
    candidates: list[Detection] = []


class Detector:
    class Service(StrEnum):
        # robot-agnostic (host 당 1, backend_v2.md §2.7) — robot_id 는 req field.
        # 무거운 모델(GDINO)은 1회 로드, 매 요청이 robot_id 로 그 로봇의 camera/캘/TCP 조회.
        DETECT = "srv/detector/detect"  # prompt + robot_id → base 3D 후보 Top-K

    class Stream(StrEnum):
        # robot-scoped 키 — payload robot_id 로 framework 라우팅 (host-level 발행,
        # scan BUILD_PROGRESS 동형).
        DETECTIONS = "stream/detector/{robot_id}/detections"


class DetectRequest(BaseModel):
    robot_id: str  # 어느 로봇의 camera/캘/base frame 으로 검출할지 (host당 1 dispatch)
    prompt: str
    top_k: int = 5  # 상위 몇 후보 반환 (§17.5 Top-K). 소비자(task)가 조절.


class DetectResponse(BaseModel):
    """Top-K 후보 (score desc). found = 후보 ≥1. 최종 선택은 소비자(task SelectTarget)."""

    found: bool
    candidates: list[Detection] = []
    message: str = ""
