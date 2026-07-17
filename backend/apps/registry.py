"""MODULE_REGISTRY — module name → "path:ClassName" 매핑.

Module 클래스를 직접 참조하지 않고 import 경로만 저장한다.
필요한 시점에만 import하여 현재 host가 실행하는 Module만 로드하고,
다른 Module의 의존성은 가져오지 않는다.
"""

from __future__ import annotations

import importlib

MODULE_REGISTRY: dict[str, str] = {
    "motor": "modules.motor.module:MotorDriverModule",
    "camera": "modules.camera.module:CameraDriverModule",
    "camera_decoded": "modules.camera.decoded:CameraDecodedModule",
    "motion": "modules.motion.module:MotionModule",
    "motion_preview": "modules.motion_preview.module:MotionPreviewModule",
    "calibration": "modules.calibration.module:CalibrationModule",
    "scene3d": "modules.scene3d.module:Scene3DModule",
    "scan": "modules.scan.module:ScanModule",
    "waypoint": "modules.waypoint.module:WaypointModule",
    "detector": "modules.detector.module:DetectorModule",
    "llm": "modules.llm.module:LlmModule",
    "pick_and_place": "modules.tasks.pick_and_place.module:PickAndPlaceModule",
    "handover": "modules.tasks.handover.module:HandoverModule",
    "bridge": "modules.bridge.module:BridgeModule",
    "logcollector": "modules.logcollector.module:LogCollectorModule",
    "host_monitor": "modules.host_monitor.module:HostMonitorModule",
}


def load_module_class(name: str) -> type:
    spec = MODULE_REGISTRY.get(name)
    if spec is None:
        raise KeyError(f"MODULE_REGISTRY 에 module {name!r} 없음 — registry.py 확인")
    module_path, cls_name = spec.split(":")
    module = importlib.import_module(module_path)
    return getattr(module, cls_name)
