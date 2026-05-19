"""Bundle Adjustment으로 추정된 joint zero offset 로더 (싱글톤).

robot/calibration/joint_offsets.npz가 있으면 시작 시 로드. 없으면 모두 0
(하위 호환 — 기존 캘리브 동작 그대로).

offset 의미: Dynamixel horn 조립 시 한 톱니 어긋남 같은 systematic FK 오차.
raw_to_rad/rad_to_raw가 이 값을 가산/감산해서 motor raw ↔ URDF rad 변환을 보정.

joint_offsets.npz 포맷:
  - joint_offsets_rad: shape (5,) float64 — joint 1..5 순서 (gripper 제외)
  - method: 'BUNDLE' 또는 유사 식별자
"""

from __future__ import annotations
import logging
import threading
from pathlib import Path
import numpy as np

logger = logging.getLogger(__name__)

_OFFSETS_PATH = (
    Path(__file__).parents[2] / "robot" / "calibration" / "joint_offsets.npz"
)

_offsets: list[float] | None = None
_loaded: bool = False
_lock = threading.Lock()


def get_joint_offset(joint_index: int) -> float:
    """0-based joint index. 파일 없거나 인덱스 초과면 0.0."""
    if not _loaded:
        _load()
    if _offsets is None or joint_index >= len(_offsets) or joint_index < 0:
        return 0.0
    return _offsets[joint_index]


def reload() -> None:
    """캘리브 직후 핫리로드용 (재시작 없이 새 offset 반영)."""
    global _loaded
    with _lock:
        _loaded = False
    _load()


def _load() -> None:
    global _offsets, _loaded
    with _lock:
        if _loaded:
            return
        _loaded = True
        if not _OFFSETS_PATH.exists():
            logger.info(f"joint_offsets 파일 없음 (offsets=0): {_OFFSETS_PATH}")
            _offsets = None
            return
        try:
            data = np.load(str(_OFFSETS_PATH))
            arr = np.asarray(data["joint_offsets_rad"], dtype=np.float64)
            _offsets = arr.tolist()
            deg = np.degrees(arr).tolist()
            method = str(data.get("method", "?"))
            logger.info(
                "joint_offsets 로드 (method=%s): %s",
                method,
                ", ".join(f"{d:+.3f}°" for d in deg),
            )
        except Exception as e:
            logger.exception("joint_offsets 로드 실패: %s", e)
            _offsets = None
