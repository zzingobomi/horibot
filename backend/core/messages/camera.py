"""Camera 노드 토픽 / 서비스 payload schema.

토픽:
- CAMERA_STATE_STATUS (publish) — CameraStatus
- CAMERA_STREAM_RAW   (publish, raw bytes JPEG) — typed 안 함 (binary 트랙, typed_messaging.md §미해결 #1)
- CAMERA_DEPTH_FRAME  (publish, raw bytes header+JPEG+zstd) — typed 안 함 (binary 트랙, §미해결 #2)

서비스 (request data / response data):
- CAMERA_SET_DEPTH_STREAM — CameraSetDepthStreamReq / CameraSetDepthStreamRes
"""

from __future__ import annotations

from pydantic import BaseModel, ConfigDict


# ─── Topic: CAMERA_STATE_STATUS ──────────────────────────────────────


class CameraStatus(BaseModel):
    """CAMERA_STATE_STATUS publish 페이로드. 연결/해제 시점 + 카메라 메타.

    width/height/fps 는 `is_opened` 가 False 일 때 0 — 그래도 필드 유지 (consumer 가
    optional 체크 안 하도록).
    """

    model_config = ConfigDict(extra="forbid")

    timestamp: float
    connected: bool
    width: int
    height: int
    fps: float
    depth_scale: float


# ─── Service: CAMERA_SET_DEPTH_STREAM ────────────────────────────────


class CameraSetDepthStreamReq(BaseModel):
    """depth_frame 토픽 스트림 enable/disable."""

    model_config = ConfigDict(extra="forbid")

    enabled: bool


class CameraSetDepthStreamRes(BaseModel):
    """전환 후 상태 echo."""

    model_config = ConfigDict(extra="forbid")

    enabled: bool
