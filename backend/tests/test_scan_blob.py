"""scan_workflow.blob encode/decode round-trip 자체 자리 자체 자리.

Scene3DNode 의 SCENE3D_SNAPSHOT 자체 자리 자체 자리 자체 자리 결과 (color JPEG + depth
zstd) 자체 자리 자체 자리 자체 자리 자체 자리 ScanTask 의 CaptureScan step 자체 자리 자체 자리
encode → storage put. ReconstructionNode 자체 자리 자체 자리 자체 자리 storage get →
decode → ndarray. round-trip 자체 자리 자체 자리 자체 자리 검증.
"""

from __future__ import annotations

import cv2
import numpy as np
import zstandard as zstd

from modules.scan_workflow import blob as scan_blob


def _make_snapshot_bytes(
    width: int = 320, height: int = 240
) -> tuple[bytes, bytes, np.ndarray, np.ndarray]:
    """Snapshot 자체 자리 자체 자리 자체 자리 합성 — color JPEG + depth zstd 자체 자리."""
    # color: gradient BGR
    color_bgr = np.zeros((height, width, 3), dtype=np.uint8)
    color_bgr[:, :, 0] = np.linspace(0, 255, width, dtype=np.uint8)[None, :]
    color_bgr[:, :, 1] = np.linspace(0, 255, height, dtype=np.uint8)[:, None]
    color_bgr[:, :, 2] = 128

    # depth: 1mm step gradient (uint16)
    depth_z16 = (
        np.linspace(500, 1500, width, dtype=np.uint16)[None, :]
        .repeat(height, axis=0)
        .astype(np.uint16)
    )

    ok, jpeg_buf = cv2.imencode(".jpg", color_bgr)
    assert ok
    color_jpeg = bytes(jpeg_buf)
    depth_zstd = zstd.ZstdCompressor().compress(depth_z16.tobytes())
    return color_jpeg, depth_zstd, color_bgr, depth_z16


def test_blob_encode_split_round_trip():
    color_jpeg, depth_zstd, _, _ = _make_snapshot_bytes()

    payload = scan_blob.encode(color_jpeg, depth_zstd)
    out_jpeg, out_zstd = scan_blob.split(payload)

    assert out_jpeg == color_jpeg
    assert out_zstd == depth_zstd


def test_blob_decode_to_ndarrays():
    color_jpeg, depth_zstd, _, depth_orig = _make_snapshot_bytes(
        width=320, height=240
    )

    payload = scan_blob.encode(color_jpeg, depth_zstd)
    color_out, depth_out = scan_blob.decode(payload, width=320, height=240)

    assert color_out.shape == (240, 320, 3)
    assert color_out.dtype == np.uint8
    assert depth_out.shape == (240, 320)
    assert depth_out.dtype == np.uint16
    # depth 는 lossless (zstd) — bit-exact
    np.testing.assert_array_equal(depth_out, depth_orig)


def test_blob_split_header_too_short():
    import pytest
    with pytest.raises(ValueError):
        scan_blob.split(b"\x00")  # 4 bytes 미만


def test_blob_split_payload_too_short():
    import struct
    import pytest
    # header 자체 자리 1000 byte jpeg 자체 자리 자체 자리 자체 자리 — 실제 자체 자리 자체 자리 자체 자리 100 byte
    bad = struct.pack("<I", 1000) + b"x" * 100
    with pytest.raises(ValueError):
        scan_blob.split(bad)
