# Camera Capability Protocol 재설계 + robot_registry typing

다음 세션 anchor (2026-06-22 작성).

이 세션은 `robot_registry.py` 의 `dict[str, object]` / `-> Any` 자리 질문에서 시작 → Camera Protocol 설계 자체를 다시 보는 자리로 확장 → 결론 내리지 않고 다음 세션 진입 anchor 로 정리.

## 1. 시작점

[robot_registry.py:148-151](../backend/core/robot/robot_registry.py#L148-L151):
```python
self._kinematics: dict[str, object] = {}  # Kinematics — lazy import 회피
self._motor_backends: dict[str, object] = {}
self._camera_captures: dict[str, object] = {}
self._motion_configs: dict[str, object] = {}
self._fk_chains: dict[str, object] = {}
```

[robot_registry.py:355-361](../backend/core/robot/robot_registry.py#L355-L361):
```python
def _get_or_build(self, cache: dict, robot_id: str | None, builder) -> object:
    ...
```

→ 5개 getter (`get_kinematics` 등) 가 무 annotation + 본문이 `_get_or_build` 호출 → caller 가 type `object` 받음. `get_camera_capture` 만 명시 `-> Any` (docstring 에 별개 사유).

**의문 1**: Protocol 이 return 시 object 로 강등되나? → **아니다**. Protocol 도 평범한 타입. `dict[str, Kinematics]` / `-> Kinematics` 그대로 인식. 위 코드의 `object` 는 *lazy import 회피용 일부러 안 한 것* (주석에 명시).

## 2. robot_registry.py 자리 — generic + TYPE_CHECKING (4 cache, camera 제외)

확인된 사실 (실제 코드 grep):

| target | 정의 위치 | 종류 | runtime dep |
|---|---|---|---|
| `Kinematics` | [modules/kinematics/kinematics.py:38](../backend/modules/kinematics/kinematics.py#L38) | Protocol | typing 만 |
| `MotorBackend` | [modules/motor/backend.py:27](../backend/modules/motor/backend.py#L27) | Protocol | typing 만 |
| `CameraCapture` | [modules/camera/capture.py:51](../backend/modules/camera/capture.py#L51) | Protocol | numpy |
| `MotionConfig` | [modules/kinematics/motion_config.py:49](../backend/modules/kinematics/motion_config.py#L49) | `@dataclass(frozen=True)` | yaml |
| `FkChain` | [modules/kinematics/fk_chain.py:78](../backend/modules/kinematics/fk_chain.py#L78) | 일반 class | numpy + **yourdfpy** |

`from __future__ import annotations` 이미 [robot_registry.py:15](../backend/core/robot/robot_registry.py#L15) 에 있음 → PEP 563 lazy annotation 보장. `_get_or_build` 가 5개 getter 자리 (`get_kinematics` 363 / `get_fk_chain` 390 / `get_motor_backend` 407 / `get_motion_config` 429 / `get_camera_capture` 439) 다 호출.

**정석 패턴**:

```python
from __future__ import annotations
from typing import TYPE_CHECKING, Any, Callable, TypeVar

if TYPE_CHECKING:
    from modules.camera.capture import CameraCapture  # camera 는 아직 못 좁힘
    from modules.kinematics.fk_chain import FkChain
    from modules.kinematics.kinematics import Kinematics
    from modules.kinematics.motion_config import MotionConfig
    from modules.motor.backend import MotorBackend

T = TypeVar("T")

class RobotRegistry:
    def __init__(self) -> None:
        ...
        self._kinematics: dict[str, Kinematics] = {}
        self._fk_chains: dict[str, FkChain] = {}
        self._motor_backends: dict[str, MotorBackend] = {}
        self._motion_configs: dict[str, MotionConfig] = {}
        self._camera_captures: dict[str, Any] = {}  # ↓ §4 자리 trauma

    def _get_or_build(
        self,
        cache: dict[str, T],
        robot_id: str | None,
        builder: Callable[[str], T],
    ) -> T: ...

    def get_kinematics(self, robot_id: str | None = None) -> Kinematics: ...
    def _build_kinematics(self, robot_id: str) -> Kinematics: ...
    # ... 5쌍 동일 (camera 제외 4쌍 narrow, camera 자리 `-> Any` 유지)
```

**런타임 import tree 변화 0** (TYPE_CHECKING block 은 runtime False + annotation 은 PEP 563 lazy). 4 cache narrow 의 hw risk **0** — 다른 caller scan 도 같이 필요하지만 변경 자리는 robot_registry 한 파일.

## 3. Camera 자리 — `-> Any` 의 root cause

camera 자리는 lazy import 가 아닌 **별개 trauma**.

[camera_node.py](../backend/nodes/device/camera_node.py) 가 `self.camera` 호출하는 자리 8개:

| 호출 | Protocol method? |
|---|---|
| `camera.open()` | ✅ Protocol |
| `camera.is_opened` | ✅ Protocol |
| `camera.close()` | ✅ Protocol |
| `camera.read()` | ❌ legacy (Protocol = `read_color()`) |
| `camera.read_aligned()` | ❌ legacy (Protocol = `read_depth_frame()`) |
| `camera.set_cloud_enabled()` | ❌ legacy (Protocol = `set_depth_enabled()`) |
| `camera.depth_scale` / `.width` / `.height` / `.fps` | ❌ legacy (Protocol 없음) |

→ `CameraCapture` Protocol 의 새 method (`read_color` / `read_depth_frame` / `set_depth_enabled`) 가 *정의는 됐는데 사용처 0* — **dead Protocol**. [RealsenseCapture](../backend/modules/camera/adapters/realsense_capture.py) adapter 가 양쪽 다 노출하지만 [camera_node](../backend/nodes/device/camera_node.py) 가 legacy 만 호출.

**카메라 추상 호출자 = `camera_node` 단 1곳** (확인 grep: `get_camera_capture` 도 camera_node line 28 만). 외부 노드 (Scene3D / Detector / Calibration / Reconstruction / Task) 는 카메라 추상 직접 호출 X — 다 토픽 통신:
- `FrameCache.get_frame()` → JPEG 토픽 디코드 → BGR ndarray
- `Scene3DNode` ← `CAMERA_DEPTH_FRAME` raw subscribe
- `CalibrationNode` / `DetectorNode` ← FrameCache

즉 `-> CameraCapture` 좁힘 = camera_node 의 legacy 호출 자리 *마이그레이션 prerequisite*.

## 4. Capability-based Protocol 재설계 논의

세션 중 capability-based Protocol split 아이디어 제시됨 (ISP + Composition over Inheritance):

```python
class ColorCapture(Protocol):
    def open(self) -> bool: ...
    def close(self) -> None: ...
    @property
    def is_opened(self) -> bool: ...
    def read_color(self) -> ColorFrame | None: ...

class DepthCapture(Protocol):
    def set_depth_enabled(self, enabled: bool) -> None: ...
    def read_depth_frame(self) -> DepthFrame | None: ...

class IntrinsicsCapture(Protocol):  # ?
    def get_intrinsic(self) -> CameraIntrinsic: ...

class RgbdCamera(ColorCapture, DepthCapture, Protocol):
    """ColorCapture + DepthCapture 둘 다 만족."""
    ...
```

**일반론 = 동의**. Python 자리 정석. 단:

- Python 은 PEP 484 intersection type 미지원 (`Camera & DepthProvider` 문법 X). 정석은 **named combined Protocol** (`class RgbdCamera(ColorCapture, DepthCapture, Protocol): ...`).
- Kinematics/MotorBackend 가 이미 Protocol 철학으로 가니까 카메라도 같은 일관성.
- ISP — UsbCamera 는 ColorCapture 만 만족, D405/D435/ZED 는 RgbdCamera. 함수 시그니처 narrowing 자연.

**우리 use case 정직 평가**:
- 함수 시그니처 narrowing 의 *실 호출 자리* = camera_node 자체 + `RobotRegistry.get_camera_capture()` — 외부 노드는 토픽 통신이라 type narrowing 효과 작음
- N=1 D405. UsbCamera/OpenCV/Mujoco 는 `NotImplementedError`. capability split 의 *실 ROI* 는 미래 추가 시점
- 단 [memory feedback_no_phase_deferral] — "UsbCamera 도착 시" 로 미루는 reflex X. legacy migration 끝나면 split 같이.

## 5. 미해결 자리 (다음 세션 진입점)

### 5.1 Capability split 의 정확한 단위

- ColorCapture / DepthCapture / IntrinsicsCapture / PointCloudCapture 중 우리 use case 에 어디까지?
- **PointCloudCapture 자리** — 우리 코드 자리 point cloud 는 `Scene3DNode` 가 *외부* Open3D 로 생성 ([scene3d_node.py](../backend/nodes/application/scene3d_node.py)). RealsenseCapture 자체 점군 만드는 자리 X → PointCloudCapture 자리 카메라 자체에 없음. 외부 sensor 가 자체 점군 publish 하는 ZED-like case 도착할 때 추가.
- **IntrinsicsCapture 별도 Protocol vs DepthCapture 포함** — 우리 자리 intrinsic 은 세 자리:
  - frame-by-frame: `DepthFrame.intrinsic` (이미 dataclass 안)
  - device-static: SDK query (factory_intrinsic 자리, D405 firmware)
  - calibration result: `intrinsic.npz` (storage)
  - → 어디서 Protocol 화 자연한지 case-by-case (특히 `CameraStatus` publish 자리 §5.2)

### 5.2 `CameraStatus` publish 자리 source

[camera_node.py:176-179](../backend/nodes/device/camera_node.py#L176-L179) 의 `camera.width/height/fps/depth_scale` 호출 — Protocol method 자리 옵션:

- **(a) Protocol 에 `get_intrinsic() -> CameraIntrinsic` 추가** → RealsenseCapture 가 SDK query. IntrinsicsCapture Protocol 자리 자연. **권장 후보**.
- (b) `read_color` 의 ColorFrame 에 size 자리 들고 가서 캐시. fps 자리 별도 source 필요.
- (c) `read_depth_frame` 의 `DepthFrame.intrinsic` 사용. 단 depth off 일 때 안 옴 → 부적합.

`CameraStatus` schema ([messages/camera.py:21-33](../backend/core/transport/messages/camera.py#L21-L33)) 의 `width/height/fps/depth_scale` 필드 *유지* — source 만 바꾸면 frontend/contract 영향 0.

### 5.3 mock_camera_node 영향

- [mock_camera_node](../backend/nodes/device/camera_node_mock.py) 가 Protocol method (`read_color` / `read_depth_frame` / `set_depth_enabled` / `open` / `close` / `is_opened`) 다 노출하는지 확인 필요
- legacy method (`read` / `read_aligned` / `set_cloud_enabled` / `width` / `height` / `fps` / `depth_scale`) 도 노출 중일 확률 높음 (camera_node 가 legacy 호출하니까 mock 도 동일 contract 맞춤)
- legacy migration 자리 mock 도 같이 손봐야

### 5.4 Stepping 전략

두 갈래 PR 후보 (다음 세션에서 §5.1 + §5.2 결정 후 fix):

**PR1 (지금, hw risk 0)** — Protocol 자리 / Registry generic / capture.py reshape:
- `robot_registry.py` generic + TYPE_CHECKING (4 cache narrow, camera 제외 `-> Any` 유지)
- `capture.py` Protocol split (capability-based — §5.1 결정 단위)
- `RealsenseCapture` 가 새 Protocol 만족 (구조적 자동, 코드 변경 0)
- `get_camera_capture` 자리는 `-> Any` 유지 docstring update

**PR2 (집, hw 검증 필요)** — Camera legacy migration:
- `camera_node` legacy 5 호출 → Protocol method 마이그레이션 (§5.2 의 `get_intrinsic` 자리 결정 결과 적용)
- `mock_camera_node` 동일 migration
- `realsense_capture.py` legacy 5 method + 4 property 삭제
- `get_camera_capture -> RgbdCamera` (또는 결정한 combined Protocol name) 좁힘

→ 사용자 [memory project_hardware_only_at_home] / [memory feedback_hw_untestable_rigor] 자리. PR2 는 회사 자리 X.

## 6. 관련 memory / 컨텍스트

- [memory project_camera_swap_state] — SO-101 도착·조립·캘 진행 중. OMX OFF. D405 swap 은 camera 후속 session. PR2 자리 의존
- [memory project_hardware_only_at_home] — 카메라 코드 실 검증은 집 자리만 → PR2 자리 한정
- [memory feedback_hw_untestable_rigor] — hw 검증 못 하는 자리 reactive fix 금지 → PR2 진행 시 hand-simulate + edge case 사전 질문 단단히
- [memory feedback_no_cargo_cult] — capability split = 일반론 정석 맞지만 우리 use case 정당화 (다중 카메라 type 예상 — UsbCamera/D435/ZED 자리) 있어야 진행
- [memory feedback_discuss_before_implement] — Protocol 설계 자체 다음 세션 첫 결정 자리, *코드 점프 X*

## 7. 다음 세션 첫 자리 todo

1. §5.1 결정 — Protocol split 단위 (ColorCapture/DepthCapture/IntrinsicsCapture, PointCloudCapture 자리 보류)
2. §5.2 결정 — `CameraStatus` publish 자리 source 옵션 (a/b/c)
3. §5.3 확인 — mock_camera_node 코드 읽고 Protocol 만족 + legacy 노출 자리 grep
4. PR1 진행 — robot_registry generic + TYPE_CHECKING (4 cache) + capture.py Protocol split
5. PR2 자리 — 집 자리 hw 검증 진입 시점에 진행 (camera_node + mock + adapter migration)
