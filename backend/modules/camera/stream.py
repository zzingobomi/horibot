import base64
import cv2
import numpy as np


def frame_to_jpeg_bytes(frame: np.ndarray, quality: int = 80) -> bytes:
    ret, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ret:
        raise RuntimeError("JPEG 인코딩 실패")
    return buf.tobytes()


def frame_to_base64(frame: np.ndarray, quality: int = 80) -> str:
    encode_params = [cv2.IMWRITE_JPEG_QUALITY, quality]
    _, buffer = cv2.imencode(".jpg", frame, encode_params)
    return base64.b64encode(buffer.tobytes()).decode("utf-8")
