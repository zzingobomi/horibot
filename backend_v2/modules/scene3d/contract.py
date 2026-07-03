"""Scene3D domain — public contract surface.

RGBD **primitive** (backend_v2 scene3d 분리 원칙): 라이브 pointcloud stream +
N-frame consensus snapshot 만. scan/mesh/session/storage 는 scan 모듈 책임.

옛 backend/nodes/application/scene3d_node.py 의 primitive 부분만 이월 —
refcount(camera depth on/off) 는 v2 camera 가 depth 상시 stream 이라 제거.
"""

from __future__ import annotations

from enum import StrEnum

from pydantic import BaseModel


class Scene3d:
    class Service(StrEnum):
        SET_STREAM = "srv/scene3d/{robot_id}/set_stream"  # 라이브 PC on/off + voxel
        SNAPSHOT = "srv/scene3d/{robot_id}/snapshot"  # N-frame consensus RGBD

    class Stream(StrEnum):
        # 라이브 pointcloud (camera-frame xyz+rgb). frontend 가 tcp·hand_eye 부모
        # transform 적용 (옛 backend Scene3DLayer 패턴). seq/timestamp invariant §8.5.
        CLOUD = "stream/scene3d/{robot_id}/cloud"


# ─── intrinsic (pinhole) ────────────────────────────────────────────


class Scene3dIntrinsic(BaseModel):
    """depth frame 해상도 기준 pinhole intrinsic. build 시 RGBD 생성에 사용."""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    depth_scale: float  # uint16 → meter (D405 = 0.0001 자리)


# ─── set_stream ─────────────────────────────────────────────────────


class SetStreamRequest(BaseModel):
    enabled: bool
    voxel_size: float | None = None  # None = 현재 유지


class SetStreamResponse(BaseModel):
    ok: bool
    enabled: bool
    voxel_size: float


# ─── snapshot (N-frame consensus) ───────────────────────────────────


class SnapshotRequest(BaseModel):
    num_frames: int = 10


class SnapshotResponse(BaseModel):
    """consensus RGBD — scan 모듈이 blob 인코딩 + 저장. raw(무손실 depth) 보존."""

    color_jpeg: bytes  # cv2.imencode(".jpg", consensus color)
    depth_zstd: bytes  # zstd(consensus depth uint16, 무손실)
    intrinsic: Scene3dIntrinsic
    num_frames: int  # 실제 consensus 에 쓴 frame 수
    timestamp_unix: float


# ─── live cloud (binary stream) ─────────────────────────────────────


class Scene3dCloud(BaseModel):
    """라이브 point cloud — camera frame. xyz_bytes = N*3 float32, rgb_bytes = N*3 uint8.

    bytes field 는 msgpack bin 으로 wire → frontend 가 Float32Array/Uint8Array 로 view.
    """

    robot_id: str
    seq: int
    timestamp_unix: float
    point_count: int
    xyz_bytes: bytes
    rgb_bytes: bytes
