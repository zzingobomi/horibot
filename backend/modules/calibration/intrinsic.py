"""Camera intrinsic 캘리브레이션.

ChArUco 검출 via board.py — plain chessboard 시절 (`findChessboardCornersSB` +
8×5 CHECKERBOARD 상수) 에선 보드 일부 가림 시 전체 fail → 사용자가 자세 매번
원위치 잡아야 했음. ChArUco 는 marker 단위 검출 + sub-set 통과라 사용자 자세
자유도 ↑ (success criteria #1: 재캘 거부감 0).

obj_pts/img_pts 는 frame 마다 길이 다름 — 검출된 ChArUco 코너 수가 가변. board
의 `matchImagePoints(charuco_corners, charuco_ids)` 가 두 list 를 같은 길이로
맞춰 반환. cv2.calibrateCamera 가 그 가변 length list 그대로 받음.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from . import board as board_module

logger = logging.getLogger(__name__)


@dataclass
class IntrinsicResult:
    camera_matrix: np.ndarray
    dist_coeffs: np.ndarray
    rms_error: float
    image_size: tuple[int, int]
    captured_count: int


class IntrinsicCalibration:
    def __init__(self):
        self.captured_frames: list[np.ndarray] = []
        # ChArUco 검출 → matchImagePoints 결과 누적. 각 frame 의 길이 가변.
        self.obj_points: list[np.ndarray] = []  # (N_i, 1, 3) float32
        self.img_points: list[np.ndarray] = []  # (N_i, 1, 2) float32
        self.result: IntrinsicResult | None = None

    def capture(self, frame: np.ndarray) -> tuple[bool, np.ndarray]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ok, ch_corners, ch_ids = board_module.detect(gray)

        vis = frame.copy()
        if ok and ch_corners is not None and ch_ids is not None:
            board_module.draw(vis, ch_corners, ch_ids)
            obj_pts, img_pts = board_module.match_object_points(
                ch_corners, ch_ids
            )
            self.obj_points.append(obj_pts)
            self.img_points.append(img_pts)
            self.captured_frames.append(frame.copy())
            logger.info(
                "ChArUco 캡처 성공 (%d장, 코너 %d개)",
                len(self.captured_frames),
                len(ch_ids),
            )

        return ok, vis

    def calibrate(self, image_size: tuple[int, int]) -> IntrinsicResult | None:
        if len(self.obj_points) < 5:
            logger.warning(
                "캡처 이미지 부족: %d장 (최소 5장 필요)", len(self.obj_points)
            )
            return None

        # cv2.calibrateCamera 는 가변 길이 obj/img list 그대로 받음.
        # opencv-python stub 은 MatLike 강제라 type: ignore.
        rms, camera_matrix, dist_coeffs, _rvecs, _tvecs = cv2.calibrateCamera(
            self.obj_points, self.img_points, image_size, None, None  # type: ignore[arg-type,call-overload]
        )

        self.result = IntrinsicResult(
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            rms_error=rms,
            image_size=image_size,
            captured_count=len(self.obj_points),
        )
        logger.info("intrinsic 캘리브 완료: RMS=%.4f", rms)
        return self.result

    def save(self, path: str | Path) -> bool:
        if self.result is None:
            logger.warning("저장할 intrinsic 결과 없음")
            return False

        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez(
            str(path),
            camera_matrix=self.result.camera_matrix,
            dist_coeffs=self.result.dist_coeffs,
            rms_error=self.result.rms_error,
            image_size=self.result.image_size,
        )
        logger.info("intrinsic 저장: %s", path)
        return True

    def load(self, path: str | Path) -> IntrinsicResult | None:
        path = Path(path)
        if not path.exists():
            return None

        data = np.load(str(path))
        self.result = IntrinsicResult(
            camera_matrix=data["camera_matrix"],
            dist_coeffs=data["dist_coeffs"],
            rms_error=float(data["rms_error"]),
            image_size=tuple(data["image_size"]),
            captured_count=0,
        )
        return self.result

    def reset(self) -> None:
        self.captured_frames.clear()
        self.obj_points.clear()
        self.img_points.clear()
        self.result = None
