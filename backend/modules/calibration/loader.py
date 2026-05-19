import logging
import numpy as np
from dataclasses import dataclass
from pathlib import Path

logger = logging.getLogger(__name__)

CALIB_DIR = Path(__file__).parents[3] / "robot" / "calibration"


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
    joint_offsets_rad: list[float] | None = None  # 길이 5, BA 결과

    def is_ready(self) -> bool:
        return self.intrinsic is not None and self.hand_eye is not None


def load_calibration() -> CalibrationData:
    return CalibrationData(
        intrinsic=_load_intrinsic(CALIB_DIR / "intrinsic.npz"),
        hand_eye=_load_hand_eye(CALIB_DIR / "hand_eye.npz"),
        joint_offsets_rad=_load_joint_offsets(CALIB_DIR / "joint_offsets.npz"),
    )


def _load_joint_offsets(path: Path) -> list[float] | None:
    if not path.exists():
        return None
    try:
        data = np.load(path)
        arr = np.asarray(data["joint_offsets_rad"], dtype=np.float64)
        return arr.tolist()
    except Exception as e:
        logger.error("joint_offsets.npz 로드 실패 (%s): %s", path, e)
        return None


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

    if data.joint_offsets_rad is not None:
        result["joint_offsets_rad"] = data.joint_offsets_rad

    return result
