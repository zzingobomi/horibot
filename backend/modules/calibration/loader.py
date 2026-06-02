import logging
import numpy as np
from dataclasses import dataclass
from pathlib import Path

from core.robot.robot_registry import RobotRegistry

logger = logging.getLogger(__name__)


def _calib_dir() -> Path:
    """active robot 의 calibration dir.

    Phase 1 single-robot: RobotRegistry().default(). robot_id 차원 도입 시
    `load_calibration(robot_id)` signature 변경.
    """
    return RobotRegistry().default().calibration_dir


@dataclass
class IntrinsicData:
    camera_matrix: np.ndarray  # shape (3, 3)
    dist_coeffs: np.ndarray  # shape (1, N)
    image_size: tuple[int, int] | None = None


@dataclass
class HandEyeData:
    R: np.ndarray  # shape (3, 3)
    t: np.ndarray  # shape (3, 1)


@dataclass
class CalibrationData:
    intrinsic: IntrinsicData | None = None
    hand_eye: HandEyeData | None = None

    def is_ready(self) -> bool:
        return self.intrinsic is not None and self.hand_eye is not None


def load_calibration() -> CalibrationData:
    return CalibrationData(
        intrinsic=_load_intrinsic(_calib_dir() / "intrinsic.npz"),
        hand_eye=_load_hand_eye(_calib_dir() / "hand_eye.npz"),
    )


def _load_intrinsic(path: Path) -> IntrinsicData | None:
    if not path.exists():
        logger.warning("intrinsic.npz 없음: %s", path)
        return None
    try:
        data = np.load(path)
        camera_matrix = data["camera_matrix"]  # (3, 3)
        dist_coeffs = data["dist_coeffs"]  # (1, N)
        image_size: tuple[int, int] | None = None
        if "image_size" in data:
            w, h = data["image_size"].tolist()
            image_size = (int(w), int(h))
        logger.info("intrinsic 로드 완료: %s", path)
        return IntrinsicData(
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            image_size=image_size,
        )
    except Exception as e:
        logger.error("intrinsic.npz 로드 실패 (%s): %s", path, e)
        return None


def _load_hand_eye(path: Path) -> HandEyeData | None:
    if not path.exists():
        logger.warning("hand_eye.npz 없음: %s", path)
        return None
    try:
        data = np.load(path)
        r_key = next((k for k in data.files if k.upper().startswith("R")), None)
        t_key = next((k for k in data.files if k.upper().startswith("T")), None)
        if r_key is None or t_key is None:
            logger.error("hand_eye.npz 키 없음 (files=%s)", data.files)
            return None
        logger.info("hand_eye 로드 완료: %s", path)
        return HandEyeData(R=data[r_key], t=data[t_key])
    except Exception as e:
        logger.error("hand_eye.npz 로드 실패 (%s): %s", path, e)
        return None


def to_json(data: CalibrationData) -> dict:
    result: dict = {}

    if data.intrinsic is not None:
        intrinsic: dict = {
            "camera_matrix": data.intrinsic.camera_matrix.tolist(),
            "dist_coeffs": data.intrinsic.dist_coeffs.tolist(),
        }
        if data.intrinsic.image_size is not None:
            intrinsic["image_size"] = list(data.intrinsic.image_size)
        result["intrinsic"] = intrinsic

    if data.hand_eye is not None:
        result["hand_eye"] = {
            "R": data.hand_eye.R.tolist(),
            "t": data.hand_eye.t.tolist(),
        }

    return result
