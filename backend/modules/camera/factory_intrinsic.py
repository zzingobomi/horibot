import logging
from pathlib import Path

import numpy as np
import pyrealsense2 as rs

logger = logging.getLogger(__name__)


def seed_d405_intrinsic_if_missing(
    npz_path: Path,
    width: int = 1280,
    height: int = 720,
) -> bool:
    if npz_path.exists():
        return False

    try:
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, width,
                             height, rs.format.bgr8, 30)
        profile = pipeline.start(config)
    except RuntimeError as e:
        logger.warning(f"D405 공장 intrinsic 시드 실패 (장치 미연결?): {e}")
        return False

    try:
        color_profile = profile.get_stream(
            rs.stream.color).as_video_stream_profile()
        intr = color_profile.get_intrinsics()
        camera_matrix = np.array(
            [
                [intr.fx, 0.0, intr.ppx],
                [0.0, intr.fy, intr.ppy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        dist_coeffs = np.array([intr.coeffs], dtype=np.float64)
        image_size = np.array([intr.width, intr.height], dtype=np.int64)
    finally:
        pipeline.stop()

    npz_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        str(npz_path),
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
        rms_error=0.0,
        image_size=image_size,
    )
    logger.info(f"D405 공장 intrinsic 저장: {npz_path}")
    return True
