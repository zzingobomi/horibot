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
from dataclasses import dataclass, field
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
    # 3×3 grid 어느 cell 에 보드 중심이 떨어졌나 (cumulative). USB UVC distortion
    # 모델이 image plane 전 영역에서 generalize 하려면 9 cell 다 cover 가 정공법.
    # cell 좌표 (gx, gy), gx/gy ∈ {0, 1, 2}.
    coverage_cells: list[tuple[int, int]] = field(default_factory=list)


class IntrinsicCalibration:
    def __init__(self):
        self.captured_frames: list[np.ndarray] = []
        # ChArUco 검출 → matchImagePoints 결과 누적. 각 frame 의 길이 가변.
        self.obj_points: list[np.ndarray] = []  # (N_i, 1, 3) float32
        self.img_points: list[np.ndarray] = []  # (N_i, 1, 2) float32
        # 3×3 grid 의 어느 cell 에 보드 중심이 떨어졌나 누적. Set 이라 같은 cell
        # 중복 카운트 X — coverage 의미가 "cell 채움 여부" 라서.
        self.coverage_cells: set[tuple[int, int]] = set()
        self.result: IntrinsicResult | None = None

    def capture(
        self, frame: np.ndarray, image_size: tuple[int, int] | None = None
    ) -> tuple[bool, np.ndarray, str]:
        """ChArUco 캡처.

        Returns:
            (ok, vis, hint):
                ok=True 면 캡처 성공.
                hint 는 사용자 안내 — 성공이면 "캡처 성공 (코너 N개)",
                실패면 *왜 실패했는지* 분기별 ("마커 미검출 / 마커 잡혔는데 corner 부족 / ...").
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ch_corners, ch_ids, _marker_corners, marker_ids = board_module.detect_full(
            gray
        )

        vis = frame.copy()
        n_markers = len(marker_ids) if marker_ids is not None else 0
        n_corners = len(ch_ids) if ch_ids is not None else 0
        ok = (
            ch_corners is not None
            and ch_ids is not None
            and n_corners >= board_module.MIN_CORNERS
        )

        if ok and ch_corners is not None and ch_ids is not None:
            board_module.draw(vis, ch_corners, ch_ids)
            obj_pts, img_pts = board_module.match_object_points(
                ch_corners, ch_ids
            )
            self.obj_points.append(obj_pts)
            self.img_points.append(img_pts)
            self.captured_frames.append(frame.copy())

            # 3×3 grid coverage — 보드 중심 위치 기준.
            if image_size is not None:
                w, h = image_size
                cx = float(ch_corners[:, 0, 0].mean())
                cy = float(ch_corners[:, 0, 1].mean())
                gx = min(max(int(cx / max(w, 1) * 3), 0), 2)
                gy = min(max(int(cy / max(h, 1) * 3), 0), 2)
                self.coverage_cells.add((gx, gy))

            logger.info(
                "ChArUco 캡처 성공 (%d장, 코너 %d개, 마커 %d개)",
                len(self.captured_frames),
                n_corners,
                n_markers,
            )
            hint = f"성공 — 코너 {n_corners}개 / 마커 {n_markers}개"
            return True, vis, hint

        # 실패 분기 — *왜* 인지 구체적으로
        if n_markers == 0:
            hint = "마커 0개 — 보드 시야 안 / 조명 / 거리 점검"
        elif n_corners < board_module.MIN_CORNERS:
            hint = (
                f"마커 {n_markers}개 잡힘, ChArUco 코너 {n_corners}개 부족 "
                f"(최소 {board_module.MIN_CORNERS}). 보드 정면도 / 일부 가림 점검"
            )
        else:
            hint = "검출 실패 (원인 미상)"
        return False, vis, hint

    def calibrate(self, image_size: tuple[int, int]) -> IntrinsicResult | None:
        from . import thresholds as T

        if len(self.obj_points) < T.INTRINSIC_MIN_CAPTURES:
            logger.warning(
                "캡처 이미지 부족: %d장 (최소 %d장 필요)",
                len(self.obj_points),
                T.INTRINSIC_MIN_CAPTURES,
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
            coverage_cells=sorted(self.coverage_cells),
        )
        logger.info(
            "intrinsic 캘리브 완료: RMS=%.4f, coverage=%d/9 cells",
            rms,
            len(self.coverage_cells),
        )
        return self.result

    def save(self, path: str | Path) -> bool:
        if self.result is None:
            logger.warning("저장할 intrinsic 결과 없음")
            return False

        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        cells_arr = (
            np.asarray(self.result.coverage_cells, dtype=np.int32)
            if self.result.coverage_cells
            else np.empty((0, 2), dtype=np.int32)
        )
        np.savez(
            str(path),
            camera_matrix=self.result.camera_matrix,
            dist_coeffs=self.result.dist_coeffs,
            rms_error=self.result.rms_error,
            image_size=self.result.image_size,
            coverage_cells=cells_arr,
        )
        logger.info("intrinsic 저장: %s", path)
        return True

    def load(self, path: str | Path) -> IntrinsicResult | None:
        path = Path(path)
        if not path.exists():
            return None

        data = np.load(str(path))
        cells: list[tuple[int, int]] = []
        if "coverage_cells" in data.files:
            cells_arr = data["coverage_cells"]
            cells = [(int(gx), int(gy)) for gx, gy in cells_arr]
        self.result = IntrinsicResult(
            camera_matrix=data["camera_matrix"],
            dist_coeffs=data["dist_coeffs"],
            rms_error=float(data["rms_error"]),
            image_size=tuple(data["image_size"]),
            captured_count=0,
            coverage_cells=cells,
        )
        return self.result

    def reset(self) -> None:
        self.captured_frames.clear()
        self.obj_points.clear()
        self.img_points.clear()
        self.coverage_cells.clear()
        self.result = None
