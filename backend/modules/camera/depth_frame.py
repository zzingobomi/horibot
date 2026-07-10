"""RGBD "primary" blob codec — combined color JPEG + zstd depth + intrinsic header.

Calibration capture / Scan 이 한 자세의 RGBD 를 한 blob 으로 묶는 포맷 (ObjectStore
`primary` artifact). offline BA 의 Stage E (depth 3D residual) 가 이 blob 을 decode.

포맷 (little-endian):
  [u32 header_len][JSON header][u32 jpeg_len][color JPEG][zstd Z16 depth]
  header: timestamp / width / height / depth_scale / fx fy cx cy
          / depth_uncompressed_bytes (검증용)

옛 backend/modules/camera/depth_frame.py 이월 (self-contained codec, repo 의존 0).
v2 CameraDepthRawFrame(스트리밍 wire) 와는 다른 concern — 이건 영속 blob codec.
"""

from __future__ import annotations

import json
import struct
from dataclasses import dataclass

import cv2
import numpy as np
import zstandard as zstd

_JPEG_QUALITY = 85


@dataclass
class DepthFrame:
    timestamp: float
    width: int
    height: int
    depth_scale: float
    fx: float
    fy: float
    cx: float
    cy: float
    color_bgr: np.ndarray  # H x W x 3 uint8
    depth_z16: np.ndarray  # H x W uint16


def encode(
    timestamp: float,
    color_bgr: np.ndarray,
    depth_z16: np.ndarray,
    depth_scale: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    jpeg_quality: int = _JPEG_QUALITY,
    zstd_level: int = 3,
) -> bytes:
    if depth_z16.dtype != np.uint16:
        raise ValueError(f"depth_z16 dtype은 uint16이어야 함, 받은: {depth_z16.dtype}")
    if color_bgr.dtype != np.uint8 or color_bgr.ndim != 3:
        raise ValueError("color_bgr는 HxWx3 uint8이어야 함")

    h, w = depth_z16.shape
    header = {
        "timestamp": float(timestamp),
        "width": int(w),
        "height": int(h),
        "depth_scale": float(depth_scale),
        "fx": float(fx),
        "fy": float(fy),
        "cx": float(cx),
        "cy": float(cy),
        "depth_uncompressed_bytes": int(depth_z16.nbytes),
    }
    header_bytes = json.dumps(header, ensure_ascii=False).encode("utf-8")

    ok, jpeg_buf = cv2.imencode(
        ".jpg",
        color_bgr,
        [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)],
    )
    if not ok:
        raise RuntimeError("color JPEG 인코딩 실패")
    jpeg_bytes = jpeg_buf.tobytes()

    cctx = zstd.ZstdCompressor(level=zstd_level)
    depth_compressed = cctx.compress(depth_z16.tobytes())

    return (
        struct.pack("<I", len(header_bytes))
        + header_bytes
        + struct.pack("<I", len(jpeg_bytes))
        + jpeg_bytes
        + depth_compressed
    )


def decode(payload: bytes) -> DepthFrame:
    offset = 0

    (header_len,) = struct.unpack_from("<I", payload, offset)
    offset += 4
    header_bytes = payload[offset : offset + header_len]
    offset += header_len
    header = json.loads(header_bytes.decode("utf-8"))

    (jpeg_len,) = struct.unpack_from("<I", payload, offset)
    offset += 4
    jpeg_bytes = payload[offset : offset + jpeg_len]
    offset += jpeg_len

    depth_compressed = payload[offset:]

    color_bgr = cv2.imdecode(
        np.frombuffer(jpeg_bytes, dtype=np.uint8),
        cv2.IMREAD_COLOR,
    )
    if color_bgr is None:
        raise RuntimeError("color JPEG 디코드 실패")

    dctx = zstd.ZstdDecompressor()
    depth_raw = dctx.decompress(depth_compressed)
    expected_bytes = header["depth_uncompressed_bytes"]
    if len(depth_raw) != expected_bytes:
        raise RuntimeError(
            f"depth 압축 해제 크기 불일치: "
            f"expected={expected_bytes} actual={len(depth_raw)}"
        )

    depth_z16 = np.frombuffer(depth_raw, dtype=np.uint16).reshape(
        header["height"], header["width"]
    )

    return DepthFrame(
        timestamp=header["timestamp"],
        width=header["width"],
        height=header["height"],
        depth_scale=header["depth_scale"],
        fx=header["fx"],
        fy=header["fy"],
        cx=header["cx"],
        cy=header["cy"],
        color_bgr=color_bgr,
        depth_z16=depth_z16,
    )
