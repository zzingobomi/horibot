"""N-frame consensus median depth — 옛 backend/modules/scene3d/consensus.py 이월.

라이브 stream 은 latest frame 만, snapshot 은 여러 frame 의 pixel-wise median 으로
sensor noise 억제. 무효 pixel(0) 은 robust 하게 masking (nanmedian).
"""

from __future__ import annotations

import numpy as np


def consensus_depth(depths: list[np.ndarray]) -> np.ndarray:
    """pixel-wise median depth (uint16). 무효(0) pixel robust 처리.

    - 전 frame 유효 pixel → 단순 median
    - 과반 이상 유효 → 무효 masking 후 nanmedian
    - 과반 미만 유효 → 0 (신뢰 불가)
    """
    if not depths:
        raise ValueError("consensus_depth: 빈 frame 리스트")
    if len(depths) == 1:
        return depths[0].astype(np.uint16, copy=True)

    stack = np.stack(depths, axis=0)  # (N, H, W) uint16
    h, w = stack.shape[1:]
    out = np.zeros((h, w), dtype=np.uint16)

    valid_mask = stack > 0  # (N, H, W)
    valid_count = valid_mask.sum(axis=0)  # (H, W)
    threshold = (len(depths) + 1) // 2  # ceil(N/2)

    all_valid = valid_count == len(depths)
    if all_valid.any():
        out[all_valid] = np.median(stack[:, all_valid], axis=0).astype(np.uint16)

    partial = (valid_count >= threshold) & ~all_valid
    if partial.any():
        masked = np.where(valid_mask, stack, np.nan).astype(np.float32)
        with np.errstate(all="ignore"):
            med = np.nanmedian(masked, axis=0)
        out[partial] = np.nan_to_num(med[partial], nan=0.0).astype(np.uint16)

    return out
