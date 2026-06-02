# distributed_topology.md

Phase 2 (SO-101 도착 후) 분산 토폴로지 + 카메라 배치 design 결정사항.

> **Status**: 잠정 — SO-101 실물 + Pi 셋업 / USB hub 배선 후 미세조정 예상. 본 문서는 코드 작성 전 design intent 를 박아두는 anchor. 본 문서의 결정이 [multi_robot_architecture.md](multi_robot_architecture.md) §13 open question #10 ("분산 토폴로지") 에 대응.

## 0. 작업 일정 — 지금 가능 vs SO-101 도착 후

본 문서의 결정사항을 **하드웨어 의존 / 무관** 으로 분리. 다음 세션 진입점.

### 0.1 지금 가능 — SO-101 / Pi 추가 도착 전에 코드 작업 가능 (5개)

영향 작은 순:

| 순 | 항목 | 절 | 영향 범위 | 상태 |
|---|---|---|---|---|
| 1 | Literal 타입 도입 | [§5](#5-타입-안전성--study-todo) | 0 (순수 타입 강화) | ✅ done |
| 2 | `robots.yaml` `hosts: {motor, camera}` 스키마 분리 | [§4](#4-robotsyaml-host-필드-표현-한계-미해결) | 0 (값은 `dev` 그대로, 어차피 안 읽힘) | ✅ done |
| 3 | `FrameCache` `dict[robot_id]` 화 | [§8](#8-cache-류-robot_id-차원-도입-마무리--framecache) | 0 (default 인자, 사용처 무영향) | ✅ done |
| 4 | `dynamixel/` 폴더 leftover 정리 | [§7](#7-protocoladapter-패턴-적용-범위) | S (import 처 갱신) | ✅ done |
| 5 | Protocol 네이밍 통일 + camera 폴더 분리 | [§6](#6-protocol-네이밍-통일--mini-refactor) | M (rename + 파일 이동 + import 처) | ✅ done |

### 0.2 SO-101 도착 후 가능 (3개)

하드웨어 / Pi 셋업 의존:

| 항목 | 절 | 의존 |
|---|---|---|
| 카메라 배치 변경 (OMX UVC, SO-101 D405) | [§1](#1-카메라-배치-변경--d405-양도) | SO-101 실물 + UVC 카메라 마운트 + 캘 |
| 호스트 네이밍 `hori1/2/3` | [§2](#2-호스트-네이밍--역할-bind-폐기) | Pi 1대 추가 셋업 |
| 노드 분배 (PC + 3 Pi) | [§3](#3-노드-분배안-잠정) | 위 둘 다 |

위 3개 완료 후 §4 의 `hosts:` 값을 `hori*` 로 갱신.

---

## 1. 카메라 배치 변경 — D405 양도

[so101_6dof_plan.md](so101_6dof_plan.md) §5 의 LeRobot 원안 (omx D405 / so101 UVC) 과 **반대**로 결정:

| 로봇 | 카메라 | 드라이버 | 변경 사유 |
|---|---|---|---|
| omx_f_0 | **USB UVC 카메라** | OS 표준 (`cv2.VideoCapture`) | D405 양도 — so101 의 6DOF 정밀 manipulation 에 RGBD 가 더 가치 |
| so101_6dof_0 | **D405 (RealSense)** | `pyrealsense2` | wrist 마운트, eye-in-hand. TSDF / pointcloud 파이프라인 so101 전용 |

### 1.1 함의

**omx_f URDF / 캘리브레이션 갱신 필요** ([robot/omx_f/urdf/](../robot/omx_f/urdf/)):
- `camera_body` mesh: `follower_06_pan_Revised_d405.stl` → UVC 카메라 mesh (또는 [LeRobot 원본](https://github.com/TheRobotStudio/SO-ARM100) 의 `camera_uvc_25x32.stl` 패턴)
- `camera_body` mass / `camera_mount_joint` origin 갱신
- `intrinsic.npz` 재캘 (UVC 다른 fx/fy/cx/cy)
- `hand_eye.npz` 재캘 (마운트 위치 변경)

**파이프라인 영향:**
- omx_f 의 detector: `Z=0` 평면 제약 기반 → 그대로 동작 (depth 불필요)
- omx_f 의 pointcloud / TSDF: **불가** (depth 없음). [pointcloud_node.py](../backend/nodes/pointcloud_node.py) 는 so101 D405 만 구독
- `CameraCapture` Protocol ([multi_robot_architecture.md §3.4](multi_robot_architecture.md#34-cameracapture-protocol)) 의 `OpenCVCapture` adapter Phase 2 실 도입

## 2. 호스트 네이밍 — 역할-bind 폐기

### 2.1 변경

| 현재 (Phase 1) | → Phase 2 |
|---|---|
| `pc` / `pi_motor` / `pi_camera` (역할 == 머신 1:1) | `hori1` / `hori2` / `hori3` (머신 식별만, 역할-agnostic) |
| PC 는 그대로 (`pc` 유지 or `hori0` 식 통일 — TBD) | — |

### 2.2 이유

Phase 2 에선 머신 4대 + 로봇 2대 + 카메라 2종 + 모터 backend 2종 → 한 머신이 단일 역할로 매핑 안 됨. 호스트 이름이 의미를 가지면 자원 분배 변경마다 rename 필요. 머신 식별과 역할 분리:

| 결정 | 누가 함 |
|---|---|
| 머신 식별 | `host_name` (host config 의 첫 줄) |
| 그 머신이 무엇을 띄울지 | `nodes:` 리스트 (host config) — **single source of truth** |
| 어느 로봇의 자원인지 | (미정 — §4 참조) |

## 3. 노드 분배안 (잠정)

### 3.1 제약

- **pyrealsense2 wheel = hori2 에만 source build 됨** ([pyrealsense2-build-guide.md](pyrealsense2-build-guide.md)) → D405 카메라 노드는 hori2 강제
- **MotorNode + MotionNode 같은 머신** — [CLAUDE.md](../CLAUDE.md) "100Hz 제어 명령 네트워크 회피" 원칙
- **USB host = 카메라/모터 노드 host** — USB serial 은 같은 머신에서만 열림

### 3.2 분배

| 머신 | 노드 | 부담 |
|---|---|---|
| PC (hori0?) | detector / task / pointcloud / calibration / bridge (+ gamepad) | YOLO inference / Open3D / TSDF — heavy compute |
| **hori1** | omx_f motor + motion + omx_f camera (UVC) | dynamixel + IK 100Hz + UVC JPEG (가벼움) |
| **hori2** | so101 D405 카메라 | D405 capture + zstd depth 무손실 압축 (heavy I/O) |
| **hori3** | so101 motor + motion | feetech + IK 100Hz |

### 3.3 근거

1. **so101 자원 분산** (hori2 + hori3) → 한 머신 부담 분산. 현재 omx_f 가 pi_motor + pi_camera 에 흩어진 패턴과 일관
2. **D405 zstd 압축 전담** (hori2) — depth 무손실 압축이 cpu heavy. 다른 부담과 묶지 않음
3. **omx 풀스택** (hori1) — UVC 가 가벼우니 dynamixel + IK 와 한 머신에 묶어 자원 균형
4. **architecture 원칙 보존** — motor+motion 같은 머신, 카메라 USB 같은 머신, PC heavy compute 모두 만족

### 3.4 검증해야 할 것 (실측)

- hori2 의 D405 zstd 압축 + Zenoh publish 가 Pi 4 단독 부담으로 OK 인지
- hori3 의 so101 6DOF IK 100Hz + feetech 모터 통신 동시 부담
- so101 자원이 hori2/hori3 에 흩어진 상태에서 ICP / TSDF (PC) 의 motor state 동기화 latency

## 4. robots.yaml `host` 필드 표현 한계 (미해결)

### 4.1 문제

현재 [robot_registry.py:51](../backend/core/robot_registry.py#L51) 의 `RobotConfig.host: str` 1개 필드는 motor/motion 만 가리킴 ([multi_robot_architecture.md:680](multi_robot_architecture.md#L680)). 본 문서의 3.2 분배는 **한 로봇이 두 머신에 흩어짐** — so101 motor=hori3, so101 camera=hori2. 1 필드로 표현 불가.

### 4.2 후보

**옵션 A — robots.yaml 확장 (필드 분리):**

```yaml
robots:
  omx_f_0:
    hosts:
      motor: hori1
      camera: hori1
  so101_6dof_0:
    hosts:
      motor: hori3
      camera: hori2
```

**옵션 B — host config 에 역방향 매핑:**

```yaml
# host_hori2.yaml
host_name: hori2
nodes: [camera]
robots:
  camera: [so101_6dof_0]   # 이 host 의 camera 노드가 담당할 로봇
```

### 4.3 추천

**옵션 A**. robots.yaml 이 robot 의 single source of truth ([multi_robot_architecture.md §4.3](multi_robot_architecture.md#43-robotsyaml-top-level-registry)) 인 일관성. host config 는 "이 머신이 어떤 종류의 노드 켤지" 만 적고, "어느 로봇 자원인지" 는 robots.yaml 한 곳. Phase 2 진입 시 확정.

## 5. 타입 안전성 — study TODO

현재 [robot_registry.py:52-53](../backend/core/robot_registry.py#L52-L53):

```python
motor_backend: str  # "dynamixel" | "feetech"
iksolver: str       # "pybullet" | "mujoco"
```

valid 값이 yaml 주석으로만 표시 → typo 부팅 시까지 잡힘 + factory exhaustiveness pyright 미보장.

Phase 2 진입 시 적용:

```python
from typing import Literal

MotorBackendName = Literal["dynamixel", "feetech"]
IKSolverName = Literal["pybullet", "mujoco"]

@dataclass(frozen=True)
class RobotConfig:
    motor_backend: MotorBackendName
    iksolver: IKSolverName
```

+ `_build_config` 에서 yaml load 시 set 멤버십 체크 (fail-fast). enum 대비 yaml 친화 + Phase 2 새 backend 추가가 가벼움.

## 6. Protocol 네이밍 통일 — mini refactor ✅ 완료

본 절 6.1~6.5 는 *refactor 전* 의 design plan. 아래 변경 적용됨:
- `CameraCaptureProtocol` → `CameraCapture` (Protocol 이름 회수)
- `CameraCapture` (RealSense impl) → `RealsenseCapture` ([modules/camera/adapters/realsense_capture.py](../backend/modules/camera/adapters/realsense_capture.py))
- [capture.py](../backend/modules/camera/capture.py) 는 Protocol + data classes 만
- **`camera_backend` selector layout 추가** (motor/ik 와 동일 패턴) — `CameraBackendName = Literal["realsense", "opencv", "mujoco"]` + `RobotRegistry.get_camera_capture(robot_id)` factory + `robots.yaml` 의 `camera_backend:` 필드. opencv / mujoco impl 은 placeholder (`NotImplementedError`) — SO-101 도착 (§1) / Track C 진입 시 작성
- **후속 통일 (Jun 3)**: raw SDK wrap 을 별도 파일로 분리해 motor 도메인의 `*Driver` / `*Backend` 어휘와 정합 — `realsense_capture.py` (raw SDK 가 들어있던 자리) → [`realsense_driver.py::RealsenseDriver`](../backend/modules/camera/adapters/realsense_driver.py), Protocol impl 자리 `realsense.py::RealSenseCapture` → [`realsense_capture.py::RealsenseCapture`](../backend/modules/camera/adapters/realsense_capture.py). 이로써 (Protocol `CameraCapture` ← impl `RealsenseCapture` ← raw `RealsenseDriver`) 가 motor (Protocol `MotorBackend` ← impl `DynamixelBackend` ← raw `DynamixelDriver`) 와 동형.

### 6.1 변경 전 현황 (일관성 깨짐)

| Protocol | 구현체 | 접미사 |
|---|---|---|
| [`IKSolver`](../backend/modules/kinematics/iksolver.py) | `PybulletIKSolver`, `MujocoIKSolver` | ❌ 없음 |
| [`MotorBackend`](../backend/modules/motor/backend.py) | `DynamixelBackend`, `FeetechBackend` | ❌ 없음 |
| `CameraCaptureProtocol` (구) | `CameraCapture` (구, RealSense wrap) | ✅ **`Protocol` 붙음** |

### 6.2 원인

`CameraCapture` 만 구현체 이름이 도메인 그 자체. Protocol 도 같은 이름이면 한 모듈에서 이름 충돌 → Protocol 쪽이 `Protocol` 접미사로 양보.

### 6.3 카메라 모듈의 폴더 구조도 불일치

kinematics / motor 는 `<module>/adapters/<impl>.py` 패턴을 따르는데 camera 만 Protocol + 구현체가 한 파일에 박혀있음:

```
backend/modules/
├── kinematics/
│   ├── iksolver.py              ← Protocol
│   ├── corrected.py             ← Decorator
│   └── adapters/pybullet_solver.py
│
├── motor/
│   ├── backend.py               ← Protocol
│   └── adapters/dynamixel_backend.py
│
└── camera/
    └── capture.py               ← Protocol + 구현체 같은 파일 ⚠️
```

RealSense 1 구현체뿐이라 분리 안 했었음. `OpenCVCapture` 추가 시점이 분리 트리거.

### 6.4 Phase 2 진입 시 통일 (`OpenCVCapture` adapter 추가 직전 묶어서)

Python 관습 = **Protocol 에 접미사 안 붙이고 구현체 이름으로 차별화** (표준 라이브러리 `Iterable` / `Sized` / `Hashable` 패턴).

**클래스 이름:**

```python
# 변경 전
class CameraCaptureProtocol(Protocol): ...
class CameraCapture: ...               # RealSense wrap

# 변경 후
class CameraCapture(Protocol): ...     # Protocol 이름 회수
class RealSenseCapture: ...            # 구현체 prefix 차별화
class OpenCVCapture: ...               # Phase 2 신규 (§1 omx_f UVC)
```

**폴더 구조:**

```
camera/
├── capture.py              ← Protocol 만 (CameraCapture)
└── adapters/
    ├── realsense.py        ← RealSenseCapture (현 CameraCapture)
    └── opencv.py           ← OpenCVCapture (신규)
```

다른 두 Protocol (IKSolver / MotorBackend) 의 `Pybullet*` / `Dynamixel*` prefix + `adapters/<impl>.py` 패턴과 일치.

### 6.5 영향 범위

- `CameraCaptureProtocol` import 처: type hint 위치만. grep 으로 일괄 치환
- `CameraCapture` 구현체 → `RealSenseCapture` rename + 파일 이동: factory (Phase 2 의 `RobotRegistry.get_camera_capture()` 신설 시) 에서 분기
- 외부 행동 변화 0 — 순수 rename + 파일 이동

## 7. Protocol/Adapter 패턴 적용 범위

### 7.1 원칙

> **외부 SDK 가 갈아끼움 가능성 있을 때만 Protocol + `adapters/` 폴더. 알고리즘 / framework 는 평탄 폴더로 충분.**

다 분리하면 over-engineering. 본 시스템에서 Protocol 패턴 도입 기준은 "같은 일을 다른 SDK 로 swap 가능성이 실제로 있는가".

### 7.2 모듈별 분류

| 모듈 | 카테고리 | 이유 |
|---|---|---|
| [`kinematics/`](../backend/modules/kinematics/) | ✅ 이미 adapter | PyBullet ↔ MuJoCo swap |
| [`motor/`](../backend/modules/motor/) | ✅ 이미 adapter | Dynamixel ↔ Feetech swap |
| [`camera/`](../backend/modules/camera/) | ✅ adapter 분리 완료 (§6) | RealSense ↔ OpenCV ↔ sim |
| [`detector/`](../backend/modules/detector/) | 🤔 검토 가치 | YOLO / Grounded / Color — `base_detector.py` 가 사실상 base. Protocol 명시화 가능 |
| [`llm/`](../backend/modules/llm/) | 🤔 필요 시 도입 | Anthropic ↔ OpenAI swap 가능성 |
| [`gamepad/`](../backend/modules/gamepad/) | ❌ 불필요 | USB HID 표준 — pygame 1개로 충분 |
| [`calibration/`](../backend/modules/calibration/) | ❌ 불필요 | 순수 알고리즘 (BA, hand_eye, sag 등) |
| [`pointcloud/`](../backend/modules/pointcloud/) | ❌ 불필요 | Open3D 알고리즘 — swap SDK 없음 |
| [`task/`](../backend/modules/task/) | ❌ 불필요 | Step DSL framework — 자체 패턴 |
| ~~`dynamixel/`~~ | ✅ 정리 완료 | `driver.py` → `motor/adapters/dynamixel_driver.py`, `motor_config.py` → `motor/motor_config.py` (Dynamixel-비종속) |

### 7.3 카테고리별 후속 action

- **✅ 이미 패턴 적용 (3)** — 현재 코드 그대로 유지
- **✅ camera adapter 분리 완료 (1)** — `CameraCapture` Protocol + `RealSenseCapture` adapter (§6)
- **🤔 검토 가치 (2)** — detector / llm. 실제 두 번째 구현체 추가 시점에 자연스러운 분기점. 미리 박으면 over-engineering
- **❌ 분리 불필요 (4)** — 그대로
- **✅ 정리 완료 (1)** — ~~`dynamixel/`~~ 폴더 삭제. `motor_config.py` 는 `motor/` 으로 (Dynamixel 비종속 = generic motor metadata), `driver.py` 는 `motor/adapters/dynamixel_driver.py` 로 (Dynamixel-specific 저수준은 backend adapter 옆)

## 8. Cache 류 robot_id 차원 도입 마무리 — `FrameCache`

### 8.1 현황

| 클래스 | robot_id 차원 |
|---|---|
| [`JointStateCache`](../backend/core/joint_state_cache.py) | ✅ done (Phase 1 sub-A, commit `e8f75ea`) |
| [`JointCoordinates`](../backend/core/joint_coordinates.py) | ✅ done |
| [`LinkCoordinates`](../backend/core/link_coordinates.py) | ✅ done |
| [`SagCoordinates`](../backend/core/sag_coordinates.py) | ✅ done |
| [`ToolCoordinates`](../backend/core/tool_coordinates.py) | ✅ done |
| [`RobotRegistry`](../backend/core/robot_registry.py) | ✅ done (factory per-robot 캐시) |
| **[`FrameCache`](../backend/core/frame_cache.py)** | ⚠️ **Phase 2 todo** — 한 카메라 1대 가정 |

### 8.2 FrameCache 변경 안

현재:
```python
self._latest_jpeg: bytes | None = None       # 단일 state
self._latest_status: dict = {}

def subscribe(self, node): ...
def get_frame(self) -> tuple[bool, np.ndarray | None]: ...
```

Phase 2:
```python
self._latest_jpeg_by_robot: dict[str, bytes] = {}
self._latest_status_by_robot: dict[str, dict] = {}

def subscribe(self, node, robot_id: str | None = None): ...    # robot 별 토픽 구독
def get_frame(self, robot_id: str | None = None) -> tuple[bool, np.ndarray | None]: ...
```

`JointStateCache` 와 같은 패턴 — `dict[robot_id]` 화 + 모든 method 에 `robot_id` 인자 (None 이면 default).

### 8.3 트리거

토픽 namespace 정정 (`omx/camera/stream/raw` → `<robot_id>/camera/stream/raw`) 과 묶어서 처리. multi_robot_architecture.md §12 Phase 2 의 #7 (토픽 namespace 정정) 과 자연스러운 동일 commit.

## 9. 관련 문서

- [multi_robot_architecture.md](multi_robot_architecture.md) — 핵심 abstraction (§3 Protocols, §4 robot identity, §5 디렉토리)
- [so101_6dof_plan.md](so101_6dof_plan.md) — SO-101 하드웨어 (§5 카메라 마운트는 본 문서 §1 결정으로 변경됨)
- [operations.md](operations.md) — Pi/IP/OS 셋업
- [pyrealsense2-build-guide.md](pyrealsense2-build-guide.md) — pyrealsense2 source build (hori2 전용 제약 origin)
- [multi_robot_walkthrough.md](multi_robot_walkthrough.md) — Phase 1 산출물 학습 anchor
