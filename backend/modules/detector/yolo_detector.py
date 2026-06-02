import numpy as np

from .base_detector import BaseDetector

# 감지할 COCO 클래스명 (yolov8 기준)
TARGET_CLASSES = {"cup", "bottle", "fork",
                  "knife", "spoon", "remote", "cell phone"}


class YoloDetector(BaseDetector):
    def __init__(self, model_path: str = "yolov8n.pt") -> None:
        from ultralytics import YOLO  # type: ignore[reportPrivateImportUsage]

        self._model = YOLO(model_path)

    def detect(self, frame: np.ndarray) -> tuple[float, float] | None:
        results = self._model(frame, verbose=False)
        boxes = results[0].boxes

        best_conf = 0.0
        best_center = None

        for box in boxes:
            cls_name = self._model.names[int(box.cls)]
            if cls_name not in TARGET_CLASSES:
                continue
            conf = float(box.conf)
            if conf > best_conf:
                best_conf = conf
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                best_center = ((x1 + x2) / 2, (y1 + y2) / 2)

        return best_center

    def raw_detect(self, frame: np.ndarray) -> list[dict]:
        results = self._model(frame, verbose=False)
        boxes = results[0].boxes

        detections = []
        for box in boxes:
            cls_name = self._model.names[int(box.cls)]
            detections.append({
                "class": cls_name,
                "bbox":  [round(v, 1) for v in box.xyxy[0].tolist()],
                "conf":  round(float(box.conf), 3),
            })
        return detections
