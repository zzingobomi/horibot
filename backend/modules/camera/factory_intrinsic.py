"""D405 factory intrinsic seed → storage.

D405 는 펌웨어 자체에 calibrated intrinsic 을 가지고 있으므로 사용자가 별도
ChArUco 캘을 돌릴 필요 없음. 부팅 시 storage 에 active intrinsic 이 없으면
RealSense SDK 의 factory intrinsic 을 그대로 commit → activate.

flow:
    storage.get_active(rid, INTRINSIC) 있음   → skip (사용자 캘 덮어쓰지 않음)
    없음                                       → D405 SDK fetch → storage commit + activate

idempotent — Pi N회 부팅 = commit 최대 1회 (첫 부팅에만). 사용자가 chessboard
캘을 별도로 돌리면 그게 active 가 되고 factory seed 는 history row 로 남음.

storage 미연결 자리는 `load_active_blocking` 가 무한 retry — motor Pi 의
motion_node 패턴과 통일. PC 가 늦게 떠도 살아나면 자동 seed. caller
(camera_node.start) 가 blocking. storage 는 본 프로젝트의 essential 인프라
(5종 캘 + scan + metadata SSOT) — 없으면 시스템 자체가 의미 없으므로 graceful
camera-only fallback 안 둠. D405 미연결 자리는 warn + skip (hardware 부재 자리
retry 의미 없음). docs/storage_layer.md §7.
"""

from __future__ import annotations

import logging
import time
from datetime import UTC, datetime
from typing import Any

import pyrealsense2  # type: ignore[import-not-found]

from modules.calibration.persistence_models import (
    CalibrationRunRecord,
    IntrinsicResultRecord,
)
from modules.calibration.result_models import IntrinsicResultData
from modules.calibration.storage_client import (
    CalibrationStorageClient,
    load_active_blocking,
)
from modules.storage.transport import StorageUnavailable

# stub 미흡 — pipeline / config / stream / format 동적 attribute 허용용 Any rebind.
rs: Any = pyrealsense2

logger = logging.getLogger(__name__)


def seed_d405_intrinsic_to_storage(
    robot_id: str,
    width: int = 1280,
    height: int = 720,
) -> bool:
    """D405 factory intrinsic 을 storage 에 idempotent seed (blocking).

    storage 미연결 자리는 `load_active_blocking` 가 무한 retry — caller 자리
    storage 살아날 때까지 막힘. motor 의 motion_node.start() 패턴과 통일.

    Returns True 면 새로 commit + activate 함, False 면 skip (이미 있음 /
    D405 미연결).
    """
    client = CalibrationStorageClient()

    # 이미 storage 에 active intrinsic 있나 — 사용자 캘 결과 덮어쓰지 않음.
    # storage 미연결 자리는 무한 retry (motor 의 fetch_active 패턴과 통일).
    existing = load_active_blocking(robot_id, "intrinsic")

    if existing is not None:
        logger.info(
            "factory intrinsic seed skip — storage 에 이미 있음 (robot=%s, run_id=%d)",
            robot_id, existing.run_id,
        )
        return False

    # D405 SDK 에서 factory intrinsic 추출 — 짧은 pipeline open 으로.
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
    now = datetime.now(UTC)
    run = CalibrationRunRecord(
        robot_id=robot_id,
        started_at=now,
        ended_at=now,
        algorithm="d405_factory",
        algorithm_params={"image_size": image_size},
        status="success",
        kind="intrinsic",
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
    # commit 자리 transient race (load_active_blocking 직후 storage 가 잠시
    # 끊김) 자리 retry. motor 패턴과 같은 1초 간격.
    while True:
        try:
            run_id, result_ids = client.commit(run, [record], [])
            client.activate(result_ids[0])
            break
        except StorageUnavailable as e:
            logger.info(
                "factory intrinsic commit 대기 중 (robot=%s): %s", robot_id, e,
            )
            time.sleep(1.0)

    logger.info(
        "factory intrinsic seed 완료 (robot=%s, run_id=%d)", robot_id, run_id
    )
    return True
