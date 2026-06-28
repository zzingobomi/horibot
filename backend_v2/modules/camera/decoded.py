"""CameraDecodedModule — robot-scoped Derived read model.

backend_v2_modules.md §1.1 #3 (CameraDecoded) + §3 (derived read model 패턴)
+ §4 (decode dedup, Camera 만) + §4.1 (한 Module 두 stream).

책임:
- Camera.Stream.JPEG subscribe → cv2.imdecode → Camera.Stream.DECODED publish
- Camera.Stream.DEPTH_RAW subscribe → zstd decompress → Camera.Stream.DEPTH_DECODED publish
- DECODED_SNAPSHOT / DEPTH_DECODED_SNAPSHOT service — point-in-time

decode 1 회 / N consumer → wire ndarray (39% → 21% CPU, §4 측정).
"""

from __future__ import annotations

import logging

import cv2
import numpy as np
import zstandard as zstd

from framework.contract.publisher import publishes
from framework.contract.service import service
from framework.contract.subscriber import subscriber
from framework.runtime.api import ModuleRuntime

from .contract import (
    Camera,
    CameraDecodedFrame,
    CameraDepthDecodedFrame,
    CameraDepthRawFrame,
    CameraJpegFrame,
    DecodedSnapshotRequest,
    DepthDecodedSnapshotRequest,
)

logger = logging.getLogger(__name__)


@publishes(
    (Camera.Stream.DECODED, CameraDecodedFrame),
    (Camera.Stream.DEPTH_DECODED, CameraDepthDecodedFrame),
)
class CameraDecodedModule:
    """robot-scoped Module — JPEG / zstd depth decode 의 자리. derived (§3.5)."""

    def __init__(
        self,
        runtime: ModuleRuntime,
        robot_id: str,
    ) -> None:
        self.runtime = runtime
        self.robot_id = robot_id

        # latest cache — snapshot service 의 source
        self._latest_color: CameraDecodedFrame | None = None
        self._latest_depth: CameraDepthDecodedFrame | None = None

        # zstd decompressor — boot 1회
        self._zstd_decompressor = zstd.ZstdDecompressor()

        # decode counter — test 자리 의 dedup 검증 (instrumentation)
        self.color_decode_count = 0
        self.depth_decode_count = 0

    # ── subscriber — JPEG → BGR ndarray ──────────────────────

    @subscriber(Camera.Stream.JPEG)
    def on_jpeg(self, event: CameraJpegFrame) -> None:
        # 자기 robot 만 (robot-scoped Module — wildcard subscribe 후 self-filter)
        if event.robot_id != self.robot_id:
            return
        try:
            jpeg_arr = np.frombuffer(event.jpeg_bytes, dtype=np.uint8)
            decoded = cv2.imdecode(jpeg_arr, cv2.IMREAD_COLOR)
            if decoded is None:
                logger.warning(
                    "JPEG decode failed robot_id=%s seq=%d",
                    self.robot_id, event.seq,
                )
                return
            self.color_decode_count += 1
            frame = CameraDecodedFrame(
                robot_id=event.robot_id,
                seq=event.seq,
                timestamp_unix=event.timestamp_unix,
                ndarray_bytes=decoded.tobytes(),
                width=int(decoded.shape[1]),
                height=int(decoded.shape[0]),
            )
            self._latest_color = frame
            self.runtime.publish(Camera.Stream.DECODED, frame)
        except Exception:
            logger.exception(
                "CameraDecoded color decode 실패 robot_id=%s seq=%d",
                self.robot_id, event.seq,
            )

    # ── subscriber — zstd depth → uint16 ndarray ─────────────

    @subscriber(Camera.Stream.DEPTH_RAW)
    def on_depth_raw(self, event: CameraDepthRawFrame) -> None:
        if event.robot_id != self.robot_id:
            return
        try:
            depth_bytes = self._zstd_decompressor.decompress(event.depth_zstd)
            self.depth_decode_count += 1
            frame = CameraDepthDecodedFrame(
                robot_id=event.robot_id,
                seq=event.seq,
                timestamp_unix=event.timestamp_unix,
                depth_bytes=depth_bytes,
                width=event.width,
                height=event.height,
                depth_scale=event.depth_scale,
            )
            self._latest_depth = frame
            self.runtime.publish(Camera.Stream.DEPTH_DECODED, frame)
        except Exception:
            logger.exception(
                "CameraDecoded depth decode 실패 robot_id=%s seq=%d",
                self.robot_id, event.seq,
            )

    # ── service — point-in-time snapshot ─────────────────────

    @service(Camera.Service.DECODED_SNAPSHOT)
    def decoded_snapshot(
        self, req: DecodedSnapshotRequest,
    ) -> CameraDecodedFrame:
        if self._latest_color is None:
            raise RuntimeError(
                f"DECODED_SNAPSHOT 아직 frame 안 받음 robot_id={self.robot_id}",
            )
        return self._latest_color

    @service(Camera.Service.DEPTH_DECODED_SNAPSHOT)
    def depth_decoded_snapshot(
        self, req: DepthDecodedSnapshotRequest,
    ) -> CameraDepthDecodedFrame:
        if self._latest_depth is None:
            raise RuntimeError(
                f"DEPTH_DECODED_SNAPSHOT 아직 frame 안 받음 robot_id={self.robot_id}",
            )
        return self._latest_depth
