"""D405 factory intrinsic seed → storage.

D405 는 펌웨어 자체에 calibrated intrinsic 을 가지고 있으므로 사용자가 별도
ChArUco 캘을 돌릴 필요 없음. 부팅 시 storage 에 active intrinsic 이 없으면
RealSense SDK 의 factory intrinsic 을 그대로 commit → activate.

flow:
    storage.get_active(rid, INTRINSIC) 있음   → skip (사용자 캘 덮어쓰지 않음)
    없음                                       → D405 SDK fetch → storage commit + activate

idempotent — Pi N회 부팅 = commit 최대 1회 (첫 부팅에만). 사용자가 chessboard
캘을 별도로 돌리면 그게 active 가 되고 factory seed 는 history row 로 남음.

storage 미연결 / D405 미연결 자리는 warn + skip (Pi 부팅 안 막음). 다음 부팅
또는 storage 가 살아난 후 retry. docs/storage_layer.md §7.
"""

from __future__ import annotations

import logging
import time
from typing import Any

import pyrealsense2  # type: ignore[import-not-found]

from modules.calibration.persistence_models import (
    CalibrationRunRecord,
    IntrinsicResultRecord,
)
from modules.calibration.result_models import IntrinsicResultData
from modules.calibration.storage_client import CalibrationStorageClient
from modules.storage.transport import StorageUnavailable

# stub 미흡 — pipeline / config / stream / format 동적 attribute 허용용 Any rebind.
rs: Any = pyrealsense2

logger = logging.getLogger(__name__)


def seed_d405_intrinsic_to_storage(
    robot_id: str,
    width: int = 1280,
    height: int = 720,
) -> bool:
    """D405 factory intrinsic 을 storage 에 idempotent seed.

    Returns True 면 새로 commit + activate 함, False 면 skip (이미 있음 / 실패).
    """
    client = CalibrationStorageClient()

    # 이미 storage 에 active intrinsic 있나 — 사용자 캘 결과 덮어쓰지 않음.
    try:
        existing = client.get_active(robot_id, "intrinsic")
    except StorageUnavailable as e:
        logger.warning(
            "factory intrinsic seed skip — storage 미연결 (robot=%s): %s",
            robot_id, e,
        )
        return False

    if existing is not None:
        logger.info(
            "factory intrinsic seed skip — storage 에 이미 있음 (robot=%s, run_id=%d)",
            robot_id, existing.run_id,
        )
        return False

    # D405 SDK 에서 factory intrinsic 자체 자체 — 짧은 pipeline open 으로 추출.
    try:
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(
            rs.stream.color, width, height, rs.format.bgr8, 30
        )
        profile = pipeline.start(config)
    except RuntimeError as e:
        logger.warning(
            "factory intrinsic seed skip — D405 미연결 (robot=%s): %s",
            robot_id, e,
        )
        return False

    try:
        color_profile = (
            profile.get_stream(rs.stream.color).as_video_stream_profile()
        )
        intr = color_profile.get_intrinsics()
        camera_matrix = [
            [intr.fx, 0.0, intr.ppx],
            [0.0, intr.fy, intr.ppy],
            [0.0, 0.0, 1.0],
        ]
        dist_coeffs = [list(intr.coeffs)]
        image_size = [int(intr.width), int(intr.height)]
    finally:
        pipeline.stop()

    # storage commit + activate — chessboard 캘과 동일 path.
    now = time.time()
    run = CalibrationRunRecord(
        robot_id=robot_id,
        started_at=now,
        ended_at=now,
        algorithm="d405_factory",
        algorithm_params={"image_size": image_size},
    )
    record = IntrinsicResultRecord(  # type: ignore[arg-type]
        run_id=0,
        robot_id=robot_id,
        created_at=now,
        sigma_rot=None,
        sigma_t=None,
        result_data=IntrinsicResultData(
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            image_size=image_size,
        ),
    )
    try:
        run_id, result_ids = client.commit(run, [record], [])
        client.activate(result_ids[0])
    except StorageUnavailable as e:
        logger.warning(
            "factory intrinsic commit 실패 — storage 미연결 (robot=%s): %s",
            robot_id, e,
        )
        return False

    logger.info(
        "factory intrinsic seed 완료 (robot=%s, run_id=%d)", robot_id, run_id
    )
    return True
