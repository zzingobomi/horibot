import cv2
import logging
from pathlib import Path
import numpy as np
from dataclasses import dataclass

logger = logging.getLogger(__name__)

CHECKERBOARD = (8, 5)  # 내부 코너 수 (가로, 세로)
SQUARE_SIZE = 0.025  # 체커보드 한 칸 크기 (미터)


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
        self.obj_points: list[np.ndarray] = []
        self.img_points: list[np.ndarray] = []
        self.result: IntrinsicResult | None = None

        # 3D 체커보드 포인트 준비
        objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0 : CHECKERBOARD[0], 0 : CHECKERBOARD[1]].T.reshape(
            -1, 2
        )
        objp *= SQUARE_SIZE
        self._objp_template = objp

    def capture(self, frame: np.ndarray) -> tuple[bool, np.ndarray]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # SB(sector-based)는 조명/블러에 강하고 sub-pixel 정확도까지 내장 → cornerSubPix 불필요.
        # 캡처 경로는 정확도 우선이라 ACCURACY + EXHAUSTIVE까지 켬.
        flags = (
            cv2.CALIB_CB_NORMALIZE_IMAGE
            | cv2.CALIB_CB_EXHAUSTIVE
            | cv2.CALIB_CB_ACCURACY
        )
        ret, corners = cv2.findChessboardCornersSB(gray, CHECKERBOARD, flags=flags)

        vis = frame.copy()
        if ret:
            cv2.drawChessboardCorners(vis, CHECKERBOARD, corners, ret)

            self.obj_points.append(self._objp_template.copy())
            self.img_points.append(corners)
            self.captured_frames.append(frame.copy())
            logger.info(f"체커보드 캡처 성공 ({len(self.captured_frames)}장)")

        return ret, vis

    def calibrate(self, image_size: tuple[int, int]) -> IntrinsicResult | None:
        if len(self.obj_points) < 5:
            logger.warning(
                f"캡처 이미지 부족: {len(self.obj_points)}장 (최소 5장 필요)"
            )
            return None

        # cv2.calibrateCamera 는 cameraMatrix/distCoeffs 가 None 일 때 내부 할당
        # — opencv-python stub 은 MatLike 강제라 type: ignore 필요.
        rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            self.obj_points, self.img_points, image_size, None, None  # type: ignore[arg-type,call-overload]
        )

        self.result = IntrinsicResult(
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            rms_error=rms,
            image_size=image_size,
            captured_count=len(self.obj_points),
        )
        logger.info(f"캘리브레이션 완료: RMS={rms:.4f}")
        return self.result

    def save(self, path: str | Path) -> bool:
        if self.result is None:
            logger.warning("저장할 캘리브레이션 결과가 없습니다")
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
        logger.info(f"캘리브레이션 결과 저장: {path}")
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
