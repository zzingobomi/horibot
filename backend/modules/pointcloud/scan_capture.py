"""capture 시점 frame 모으기 + median consensus.

PointCloudNode가 이미 CAMERA_DEPTH_FRAME(8 FPS)을 raw subscriber로 받고 있어
`_latest_frame`이 항상 최신. 여기서는 그 캐시를 timestamp 기준 폴링해서 *서로 다른*
frame N개를 모은다 (8 FPS → N=10이면 1.25s).
"""

from __future__ import annotations

import time
from typing import Callable

import numpy as np

from modules.camera.depth_frame import DepthFrame

N_FRAMES_DEFAULT = 10
FRAME_GATHER_TIMEOUT = 5.0  # depth 8 FPS 기준 10장이 1.25s — 여유 4배


def gather_frames(
    get_latest_frame: Callable[[], DepthFrame | None],
    n: int = N_FRAMES_DEFAULT,
    timeout: float = FRAME_GATHER_TIMEOUT,
) -> list[DepthFrame]:
    """`_latest_frame`을 폴링해 서로 다른 timestamp의 frame n개 수집.

    수동 캡처라 사용자가 이미 정지해 있다고 가정.
    """
    out: list[DepthFrame] = []
    last_ts = -1.0
    deadline = time.time() + timeout
    while len(out) < n and time.time() < deadline:
        f = get_latest_frame()
        if f is not None and f.timestamp > last_ts + 1e-6:
            out.append(f)
            last_ts = f.timestamp
        else:
            time.sleep(0.02)
    if len(out) < n:
        raise TimeoutError(
            f"depth_frame {n}장 수집 실패: {len(out)}장만 들어옴 (timeout {timeout}s)"
        )
    return out


def consensus_depth(frames: list[DepthFrame]) -> np.ndarray:
    """N장의 depth_z16을 픽셀별 median으로 합침. invalid(0) 픽셀 robust.

    - 과반(>=ceil(N/2)) valid 픽셀: nonzero만으로 median
    - 그 외: 0 유지 (invalid)
    """
    stack = np.stack([f.depth_z16 for f in frames], axis=0)  # (N, H, W) uint16
    h, w = stack.shape[1:]
    out = np.zeros((h, w), dtype=np.uint16)

    valid_mask = stack > 0
    valid_count = valid_mask.sum(axis=0)
    threshold = (len(frames) + 1) // 2

    # 빠른 path: 전부 valid
    all_valid = valid_count == len(frames)
    if all_valid.any():
        out[all_valid] = np.median(stack[:, all_valid], axis=0).astype(np.uint16)

    # 일부 valid (>= threshold): nonzero만 median
    partial = (valid_count >= threshold) & ~all_valid
    if partial.any():
        masked = np.where(valid_mask, stack, np.nan).astype(np.float32)
        with np.errstate(all="ignore"):
            med = np.nanmedian(masked, axis=0)
        out[partial] = np.nan_to_num(med[partial], nan=0).astype(np.uint16)

    return out


def consensus_color(frames: list[DepthFrame]) -> np.ndarray:
    """color는 마지막 frame 사용 — JPEG 압축이라 median 의미 작음."""
    return frames[-1].color_bgr.copy()
