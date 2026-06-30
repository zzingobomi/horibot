"""MODULE_REGISTRY — module name → "path:ClassName" (lazy import).

**lazy 가 핵심** — host 는 자기 deployment 의 모듈만 import 해야 한다. eager import
면 pi_camera 가 registry import 만으로 MotionModule(pybullet) / BridgeModule(fastapi)
까지 끌어와 boot 실패 (role-split deps 무력화). importlib 로 instantiate 시점에만
import → pi_camera 는 camera 모듈만, pi_motor 는 motor/motion 만 import.

옛 backend node_registry 의 NodeSpec lazy-import 패턴과 동형.
"""

from __future__ import annotations

import importlib

MODULE_REGISTRY: dict[str, str] = {
    "motor": "modules.motor.module:MotorDriverModule",
    "camera": "modules.camera.module:CameraDriverModule",
    "camera_decoded": "modules.camera.decoded:CameraDecodedModule",
    "motion": "modules.motion.module:MotionModule",
    "bridge": "modules.bridge.module:BridgeModule",
}


def load_module_class(name: str) -> type:
    """name → 실 class (호출 시점에만 import). 미등록 시 KeyError."""
    spec = MODULE_REGISTRY.get(name)
    if spec is None:
        raise KeyError(f"MODULE_REGISTRY 에 module {name!r} 없음 — registry.py 확인")
    module_path, cls_name = spec.split(":")
    module = importlib.import_module(module_path)
    return getattr(module, cls_name)
