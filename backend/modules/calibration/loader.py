"""Calibration 의 runtime 데이터 모델 — Detector / PointCloudLayer 가 사용.

storage 모름 — calibration_node 가 부팅 시 storage 에서 fetch 후 본 dataclass
객체를 만들어 소비자 (DetectorNode 등) 에 set_calibration 으로 push.
"""

import numpy as np
from dataclasses import dataclass


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
