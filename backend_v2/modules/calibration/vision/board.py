"""ChArUco 보드 spec SSOT + 검출 헬퍼.

보드 spec (calib.io, 2026-06-10):
    - 5×7 squares (rows × cols) → OpenCV size=(squaresX=cols=7, squaresY=rows=5)
    - checker 25mm / marker 18mm / DICT_4X4_50 / Start Id 0
    - 내부 ChArUco 코너 = (7-1)*(5-1) = 24

intrinsic / handeye / preview 가 같은 spec 을 다른 자리에 hard-code 하지 않게 SSOT.
plain chessboard 와 달리 ChArUco 는 marker 단위 검출이라 일부 가려져도 살아남음.
"""

from __future__ import annotations

import logging
from typing import Any

import cv2
import numpy as np

logger = logging.getLogger(__name__)

# 보드 spec — calib.io ChArUco. Rows=5, Columns=7.
# OpenCV CharucoBoard.size = (squaresX=cols, squaresY=rows) 컨벤션.
SQUARES_X: int = 7  # Columns
SQUARES_Y: int = 5  # Rows
SQUARE_LENGTH_M: float = 0.025
MARKER_LENGTH_M: float = 0.018
ARUCO_DICT_ID: int = cv2.aruco.DICT_4X4_50

# (cols-1)*(rows-1) = 24
TOTAL_CHARUCO_CORNERS: int = (SQUARES_X - 1) * (SQUARES_Y - 1)

# solvePnP 는 4 점부터 풀리지만 BA seed 안정성 위해 12 점 (전체 절반).
MIN_CORNERS: int = 12


_BOARD: cv2.aruco.CharucoBoard | None = None
_DETECTOR: cv2.aruco.CharucoDetector | None = None


def _make_board() -> cv2.aruco.CharucoBoard:
    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)
    b = cv2.aruco.CharucoBoard(
        size=(SQUARES_X, SQUARES_Y),
        squareLength=SQUARE_LENGTH_M,
        markerLength=MARKER_LENGTH_M,
        dictionary=dictionary,
    )
    # calib.io 는 modern (non-legacy) ChArUco 패턴.
    b.setLegacyPattern(False)
    return b


def board() -> cv2.aruco.CharucoBoard:
    global _BOARD
    if _BOARD is None:
        _BOARD = _make_board()
    return _BOARD


def _detector() -> cv2.aruco.CharucoDetector:
    global _DETECTOR
    if _DETECTOR is None:
        _DETECTOR = cv2.aruco.CharucoDetector(board())
    return _DETECTOR


def detect(
    gray: np.ndarray,
) -> tuple[bool, np.ndarray | None, np.ndarray | None]:
    """ChArUco 검출. MIN_CORNERS 미만이면 ok=False.

    Returns:
        (ok, charuco_corners (N,1,2) float32, charuco_ids (N,1) int32)
    """
    ch_corners, ch_ids, _marker_corners, _marker_ids = _detector().detectBoard(gray)
    if ch_corners is None or ch_ids is None or len(ch_ids) < MIN_CORNERS:
        return False, None, None
    return True, ch_corners, ch_ids


def detect_full(gray: np.ndarray) -> tuple[Any, Any, Any, Any]:
    """ChArUco corner + marker 둘 다 반환 (preview overlay 용).

    cv2 type stub 이 marker_corners 를 Sequence[MatLike] 로 표기해 return 은 Any —
    caller 가 None / len 을 check 후 사용.

    Returns:
        (charuco_corners (N,1,2), charuco_ids (N,1),
         marker_corners list[(1,4,2)], marker_ids (M,1))  검출 안 되면 None.
    """
    return _detector().detectBoard(gray)


def match_object_points(
    ch_corners: np.ndarray, ch_ids: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """검출된 ChArUco 코너 → cv2.calibrateCamera / solvePnP 입력 pair.

    Returns:
        (obj_pts (N,1,3) float32, img_pts (N,1,2) float32)
    """
    obj_pts, img_pts = board().matchImagePoints(ch_corners, ch_ids)  # type: ignore[arg-type]
    return obj_pts, img_pts


def draw(image: np.ndarray, ch_corners: np.ndarray, ch_ids: np.ndarray) -> None:
    """검출 overlay (in-place)."""
    cv2.aruco.drawDetectedCornersCharuco(image, ch_corners, ch_ids)


def spec_as_dict() -> dict:
    """보드 spec snapshot — capture run.algorithm_params 에 freeze.

    offline 분석 스크립트가 이 snapshot 으로 BA 입력 (보드 차원 / dictionary) 재현.
    """
    return {
        "squares_x": SQUARES_X,
        "squares_y": SQUARES_Y,
        "square_length_m": SQUARE_LENGTH_M,
        "marker_length_m": MARKER_LENGTH_M,
        "aruco_dict_id": ARUCO_DICT_ID,
        "min_corners": MIN_CORNERS,
    }


def board_corner_points_3d() -> np.ndarray:
    """보드 4 외곽 코너의 보드 frame 3D 좌표 (m). 시각화 / visibility gate 용.

    보드 frame 원점 = (0,0,0) 첫 square 좌측-상단, +X right, +Y down, Z=0 (평면).
    """
    w = SQUARES_X * SQUARE_LENGTH_M
    h = SQUARES_Y * SQUARE_LENGTH_M
    return np.array(
        [[0.0, 0.0, 0.0], [w, 0.0, 0.0], [w, h, 0.0], [0.0, h, 0.0]],
        dtype=np.float64,
    )
