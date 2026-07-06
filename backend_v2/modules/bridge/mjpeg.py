from __future__ import annotations

import asyncio
import logging
from typing import AsyncIterator

from framework.contract.publisher import decode_event
from framework.transport.protocol import RawTransport
from modules.camera.contract import Camera, CameraJpegFrame

logger = logging.getLogger(__name__)

BOUNDARY = "frame"


async def mjpeg_stream(transport: RawTransport, robot_id: str) -> AsyncIterator[bytes]:
    """robot 의 JPEG stream 을 구독해 multipart chunk 로 yield.

    latest-wins (maxsize=1) — 느린 브라우저가 frame 쌓이게 두지 않는다.
    client disconnect 시 generator GC → finally 에서 unsubscribe.
    """
    topic = Camera.Stream.JPEG.format(robot_id=robot_id)
    queue: asyncio.Queue[bytes] = asyncio.Queue(maxsize=1)
    loop = asyncio.get_running_loop()

    def on_frame(payload: bytes) -> None:
        try:
            frame = decode_event(CameraJpegFrame, payload)
        except Exception:
            logger.exception("MJPEG frame decode 실패 robot_id=%s", robot_id)
            return

        def put() -> None:
            if queue.full():
                try:
                    queue.get_nowait()
                except asyncio.QueueEmpty:
                    pass
            queue.put_nowait(frame.jpeg_bytes)

        loop.call_soon_threadsafe(put)

    handle = transport.subscribe(topic, on_frame)
    try:
        while True:
            jpeg = await queue.get()
            yield (
                (
                    f"--{BOUNDARY}\r\n"
                    f"Content-Type: image/jpeg\r\n"
                    f"Content-Length: {len(jpeg)}\r\n\r\n"
                ).encode("ascii")
                + jpeg
                + b"\r\n"
            )
    finally:
        try:
            handle.undeclare()
        except Exception:
            pass
