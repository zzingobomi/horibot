"""Calibration 의 runtime 데이터 모델 — Detector / PointCloudLayer 가 사용.

storage 모름 — calibration_node 가 부팅 시 storage 에서 fetch 후 본 dataclass
객체를 만들어 소비자 (DetectorNode 등) 에 set_calibration 으로 push.

`load_calibration_from_npz(robot_id)` 함수는 Stage 3 마이그레이션 스크립트의
도움이로 남겨둠 — 옛 npz → storage import 시 사용. 런타임 부팅 path 에선 호출 X.
"""

import logging
import numpy as np
from dataclasses import dataclass
from pathlib import Path

from core.robot.robot_registry import RobotRegistry

logger = logging.getLogger(__name__)


def _calib_dir(robot_id: str) -> Path:
    return RobotRegistry().get(robot_id).calibration_dir


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


def load_calibration_from_npz(robot_id: str) -> CalibrationData:
    """옛 npz 직접 load — Stage 3 마이그레이션 스크립트 전용. runtime 부팅 path X."""
    calib_dir = _calib_dir(robot_id)
    return CalibrationData(
        intrinsic=_load_intrinsic(calib_dir / "intrinsic.npz"),
        hand_eye=_load_hand_eye(calib_dir / "hand_eye.npz"),
    )


def _load_intrinsic(path: Path) -> IntrinsicData | None:
    if not path.exists():
        return None
    try:
        data = np.load(path)
        camera_matrix = data["camera_matrix"]  # (3, 3)
        dist_coeffs = data["dist_coeffs"]  # (1, N)
        image_size: tuple[int, int] | None = None
        if "image_size" in data:
            w, h = data["image_size"].tolist()
            image_size = (int(w), int(h))
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
        return None
    try:
        data = np.load(path)
        r_key = next((k for k in data.files if k.upper().startswith("R")), None)
        t_key = next((k for k in data.files if k.upper().startswith("T")), None)
        if r_key is None or t_key is None:
            return None
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
