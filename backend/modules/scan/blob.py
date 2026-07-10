"""Scan blob 인코딩 — 옛 backend/modules/scan_workflow/blob.py 이월.

wire format: [u32 jpeg_len LE][color JPEG][zstd depth uint16].
depth 는 무손실(zstd) — ICP/TSDF 정밀도 보존. color 만 JPEG 손실.
"""

from __future__ import annotations

import struct

import cv2
import numpy as np
import zstandard as zstd


def encode(color_jpeg: bytes, depth_zstd: bytes) -> bytes:
    return struct.pack("<I", len(color_jpeg)) + color_jpeg + depth_zstd


def decode(blob: bytes, width: int, height: int) -> tuple[np.ndarray, np.ndarray]:
    """blob → (color_bgr HxWx3 uint8, depth_z16 HxW uint16)."""
    (jpeg_len,) = struct.unpack_from("<I", blob, 0)
    jpeg = blob[4 : 4 + jpeg_len]
    depth_z = blob[4 + jpeg_len :]
    color = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
    if color is None:
        raise ValueError("scan blob: color JPEG decode 실패")
    depth_raw = zstd.ZstdDecompressor().decompress(depth_z)
    depth = np.frombuffer(depth_raw, dtype=np.uint16).reshape(height, width)
    return color, depth
