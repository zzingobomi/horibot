"""Scan blob — ObjectStore 자리 wire format.

Snapshot (color_bgr JPEG + depth_z16 zstd) 자리 그대로 concat 한 단순 binary.
metadata (width/height/intrinsic/motor) 자리는 RDB row 자리만 (SSOT).
Reconstruction 자리 load 시 RDB record 의 width/height 자리 사용 자리 reshape.

format:
    [u32 jpeg_len LE][JPEG color][zstd depth]
"""

from __future__ import annotations

import struct

import cv2
import numpy as np
import zstandard as zstd


def encode(color_bgr_jpeg: bytes, depth_z16_zstd: bytes) -> bytes:
    """Scene3DSnapshotRes 의 color_bgr_jpeg / depth_z16_zstd 자리 그대로 concat."""
    return struct.pack("<I", len(color_bgr_jpeg)) + color_bgr_jpeg + depth_z16_zstd


def split(blob_bytes: bytes) -> tuple[bytes, bytes]:
    """raw bytes 자리만 split. return (color_jpeg_bytes, depth_zstd_bytes)."""
    if len(blob_bytes) < 4:
        raise ValueError("scan blob 자리 header 자리 자리 부족")
    (jpeg_len,) = struct.unpack_from("<I", blob_bytes, 0)
    if len(blob_bytes) < 4 + jpeg_len:
        raise ValueError(
            f"scan blob 자리 자리 부족 — header_jpeg_len={jpeg_len}, "
            f"actual={len(blob_bytes) - 4}"
        )
    return blob_bytes[4 : 4 + jpeg_len], blob_bytes[4 + jpeg_len :]


def decode(
    blob_bytes: bytes, width: int, height: int
) -> tuple[np.ndarray, np.ndarray]:
    """ReconstructionNode 자리 자리. width/height 자리 RDB record 자리 사용.

    return (color_bgr [H x W x 3 uint8], depth_z16 [H x W uint16]).
    """
    color_jpeg, depth_zstd = split(blob_bytes)
    color_bgr = cv2.imdecode(
        np.frombuffer(color_jpeg, dtype=np.uint8), cv2.IMREAD_COLOR
    )
    if color_bgr is None:
        raise RuntimeError("scan blob 자리 color JPEG decode 실패")
    depth_raw = zstd.ZstdDecompressor().decompress(depth_zstd)
    depth_z16 = np.frombuffer(depth_raw, dtype=np.uint16).reshape(height, width)
    return color_bgr, depth_z16
