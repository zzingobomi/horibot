from __future__ import annotations

from modules.bridge.module import BridgeModule
from modules.camera.decoded import CameraDecodedModule
from modules.camera.module import CameraDriverModule
from modules.motor.module import MotorDriverModule

MODULE_REGISTRY: dict[str, type] = {
    "motor": MotorDriverModule,
    "camera": CameraDriverModule,
    "camera_decoded": CameraDecodedModule,
    "bridge": BridgeModule,
}
