"""캘리브레이션용 동차 변환 헬퍼."""

from __future__ import annotations

import numpy as np


def make_T(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """3x3 R + 3-vec t → 4x4 homogeneous."""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t).reshape(3)
    return T
