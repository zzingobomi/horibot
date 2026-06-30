from __future__ import annotations

from enum import StrEnum

from pydantic import BaseModel


class Camera:
    class Service(StrEnum):
        CAPABILITIES = "srv/camera/{robot_id}/capabilities"

        # decoded frame point-in-time (CameraDecoded snapshot)
        DECODED_SNAPSHOT = "srv/camera/{robot_id}/decoded_snapshot"
        DEPTH_DECODED_SNAPSHOT = "srv/camera/{robot_id}/depth_decoded_snapshot"

    class Stream(StrEnum):
        # CameraDriver publish — raw wire (JPEG + zstd depth)
        JPEG = "stream/camera/{robot_id}/jpeg"
        DEPTH_RAW = "stream/camera/{robot_id}/depth_raw"

        # CameraDecoded publish — decoded ndarray
        DECODED = "stream/camera/{robot_id}/decoded"
        DEPTH_DECODED = "stream/camera/{robot_id}/depth_decoded"


# ─── capability ─────────────────────────────────────────────────────


class CameraCapability(StrEnum):
    """flags only (§7.1 invariant — what is possible, not how configured)."""

    RGB = "rgb"
    DEPTH = "depth"
    POINTCLOUD = "pointcloud"


class CameraCapabilities(BaseModel):
    """static fact — D405 / USB / Basler 마다 다름. driver self-declare (§7.3)."""

    flags: set[CameraCapability]


# ─── request ────────────────────────────────────────────────────────


class CapabilitiesRequest(BaseModel):
    pass


class DecodedSnapshotRequest(BaseModel):
    pass


class DepthDecodedSnapshotRequest(BaseModel):
    pass


# ─── stream payload — raw wire (CameraDriver) ──────────────────────


class CameraJpegFrame(BaseModel):
    """JPEG-encoded color frame. CameraDriver → CameraDecoded / Bridge."""

    robot_id: str
    seq: int
    timestamp_unix: float
    jpeg_bytes: bytes
    width: int
    height: int


class CameraDepthRawFrame(BaseModel):
    """zstd-compressed uint16 depth. CameraDriver → CameraDecoded."""

    robot_id: str
    seq: int
    timestamp_unix: float
    depth_zstd: bytes
    width: int
    height: int
    depth_scale: float  # uint16 → meter 변환 (D405 = 0.0001 자리)


# ─── stream payload — decoded (CameraDecoded) ──────────────────────


class CameraDecodedFrame(BaseModel):
    """BGR ndarray decoded color. CameraDecoded → Detector / Scene3D / ..."""

    robot_id: str
    seq: int
    timestamp_unix: float
    ndarray_bytes: bytes  # H × W × 3 uint8 BGR raw
    width: int
    height: int


class CameraDepthDecodedFrame(BaseModel):
    """uint16 depth ndarray decoded. CameraDecoded → Scene3D."""

    robot_id: str
    seq: int
    timestamp_unix: float
    depth_bytes: bytes  # H × W uint16 raw
    width: int
    height: int
    depth_scale: float
