"""Scene3D — RGBD primitive sensor schema.

Topic:
- SCENE3D_STREAM (binary, raw point cloud) — typed 안 함
- SCENE3D_STATE  (publish) — Scene3DState

Service:
- SCENE3D_SNAPSHOT   — Scene3DSnapshotReq / Scene3DSnapshotRes  (단발, raw + intrinsic + motor)
- SCENE3D_SET_STREAM — Scene3DSetStreamReq / Scene3DSetStreamRes  (continuous toggle)
"""

from __future__ import annotations

from pydantic import Base64Bytes

from core.transport.messages.base import StrictModel


# ─── Topic: SCENE3D_STATE — primitive sensor 상태 ─────────────────


class Scene3DState(StrictModel):
    timestamp: float
    enabled: bool
    voxel_size: float


# ─── Service: SCENE3D_SNAPSHOT — primitive 단발 capture ─────────────
# RGBD sensor primitive 자리. caller (ScanTask 의 CaptureScan / 캘 verification
# / 미래 3D detection 등) 가 받은 raw frame 으로 자체 처리 (storage put / point
# cloud 변환 / fuse 등). depth_z16 은 zstd, color 는 JPEG 으로 압축된 채 반환
# — Pydantic bytes field 가 wire 시 base64. depth 614KB raw → ~300KB zstd →
# ~400KB base64. ScanTask 의 10-pose ForEach 자리 ~4.7 MB wire.


class Scene3DIntrinsic(StrictModel):
    """RGBD 카메라 intrinsic — pinhole + depth scale."""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    depth_scale: float


class Scene3DSnapshotReq(StrictModel):
    """N frame consensus median.

    num_frames=1 이면 단순 latest. 보통 10 (consensus 안정성).
    """

    num_frames: int = 10
    timeout_s: float = 5.0


class Scene3DSnapshotRes(StrictModel):
    # Pydantic bytes field 는 model_dump_json 시 utf-8 string 으로 인코딩 시도
    # → binary blob 자체 자리 자체 자리 자체 자리 fail. Base64Bytes 자체 자리 자체 자리
    # 자체 자리 자체 자리 자체 자리 wire base64 string ↔ Python raw bytes.
    color_bgr_jpeg: Base64Bytes  # cv2.imencode("jpg") 결과
    depth_z16_zstd: Base64Bytes  # zstd.compress(depth_z16.tobytes())
    intrinsic: Scene3DIntrinsic
    motor_positions: list[int]
    arm_motor_ids: list[int]
    timestamp: float
    num_frames: int


# ─── Service: SCENE3D_SET_STREAM — continuous toggle ───────────────


class Scene3DSetStreamReq(StrictModel):
    enabled: bool


class Scene3DSetStreamRes(StrictModel):
    enabled: bool
