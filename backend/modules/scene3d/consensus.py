"""Snapshot consensus 자리 — N frame median (depth + color).

Scene3DNode 의 _srv_snapshot 자리 자리. 8 FPS depth 자리 N=10 자리 ~1.25s.
"""

from __future__ import annotations

import time
from typing import Callable

import numpy as np

from modules.camera.depth_frame import DepthFrame

N_FRAMES_DEFAULT = 10
FRAME_GATHER_TIMEOUT = 5.0


def gather_frames(
    get_latest_frame: Callable[[], DepthFrame | None],
    n: int = N_FRAMES_DEFAULT,
    timeout: float = FRAME_GATHER_TIMEOUT,
) -> list[DepthFrame]:
    """latest_frame 자리 polling 자리 서로 다른 timestamp 자리 frame n개 자리 모음.

    사용자가 캡처 시점 자리 정지 가정. timeout 자리 분산 자리 LAN latency 자리 포함.
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
    """N장 depth_z16 자리 픽셀별 median 자리. invalid(0) 픽셀 robust.

    - 과반(>=ceil(N/2)) valid 픽셀: nonzero 만으로 median
    - 그 외: 0 유지 (invalid)
    """
    stack = np.stack([f.depth_z16 for f in frames], axis=0)  # (N, H, W) uint16
    h, w = stack.shape[1:]
    out = np.zeros((h, w), dtype=np.uint16)

    valid_mask = stack > 0
    valid_count = valid_mask.sum(axis=0)
    threshold = (len(frames) + 1) // 2

    all_valid = valid_count == len(frames)
    if all_valid.any():
        out[all_valid] = np.median(stack[:, all_valid], axis=0).astype(np.uint16)

    partial = (valid_count >= threshold) & ~all_valid
    if partial.any():
        masked = np.where(valid_mask, stack, np.nan).astype(np.float32)
        with np.errstate(all="ignore"):
            med = np.nanmedian(masked, axis=0)
        out[partial] = np.nan_to_num(med[partial], nan=0).astype(np.uint16)

    return out


def consensus_color(frames: list[DepthFrame]) -> np.ndarray:
    """color 자리 마지막 frame 자리 사용 — JPEG 압축 자리 median 의미 작음."""
    return frames[-1].color_bgr.copy()
