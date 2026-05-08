import numpy as np

from core.realsense_capture import RealsenseCapture


class CameraCapture:
    def __init__(self):
        self._rs = RealsenseCapture()

    def open(self) -> bool:
        return self._rs.open()

    def close(self) -> None:
        self._rs.close()

    def read(self) -> tuple[bool, np.ndarray | None]:
        return self._rs.read_color()

    @property
    def is_opened(self) -> bool:
        return self._rs.is_opened

    @property
    def width(self) -> int:
        return self._rs.width

    @property
    def height(self) -> int:
        return self._rs.height

    @property
    def fps(self) -> float:
        return self._rs.fps
