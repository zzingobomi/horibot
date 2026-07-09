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
| 2 | ~~`robots.yaml` `hosts: {motor, camera}` 스키마 분리~~ → **rollback: hosts 필드 제거** | [§4](#4-robotsyaml-host-필드--제거-결정) | 0 (어차피 안 읽히던 자리) | ✅ done (rollback) |
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

위 3개 완료 후 §3 의 분배안을 **host config (`host_hori*.yaml`) 의 `nodes:` 리스트** 로 박는다. robots.yaml 은 robot 정체성만 보유 — deployment 정보 안 들어감 (§4 결정).

---

## 1. 카메라 배치 변경 — D405 양도

> **현재 상태 (2026-06)**: D405 는 OMX 에 부착, SO-101 미도착. 아래 표는 **SO-101 수령 후** 적용될 swap plan. 카메라 spec 정식 정의는 [hardware.md § 카메라](hardware.md).

[so101_6dof_plan.md](so101_6dof_plan.md) §5 의 LeRobot 원안 (omx D405 / so101 UVC) 과 **반대**로 결정:

| 로봇 | 카메라 | 드라이버 | 변경 사유 |
|---|---|---|---|
| omx_f_0 | **720P USB UVC (DFOV 120°)** | OS 표준 (`cv2.VideoCapture`) | D405 양도 — so101 의 6DOF 정밀 manipulation 에 RGBD 가 더 가치 |
| so101_6dof_0 | **D405 (RealSense)** | `pyrealsense2` | wrist 마운트, eye-in-hand. TSDF / pointcloud 파이프라인 so101 전용 |

### 1.1 함의

**omx_f URDF / 캘리브레이션 갱신 필요** ([robot/omx_f/urdf/](../robot/omx_f/urdf/)):
- `camera_body` mesh: `follower_06_pan_Revised_d405.stl` → UVC 카메라 mesh (또는 [LeRobot 원본](https://github.com/TheRobotStudio/SO-ARM100) 의 `camera_uvc_25x32.stl` 패턴)
- `camera_body` mass / `camera_mount_joint` origin 갱신
- `intrinsic.npz` 재캘 (UVC 다른 fx/fy/cx/cy)
- `hand_eye.npz` 재캘 (마운트 위치 변경)

**파이프라인 영향:**
- omx_f 의 detector: `Z=0` 평면 제약 기반 → 그대로 동작 (depth 불필요)
- omx_f 의 pointcloud / TSDF: **불가** (depth 없음). [pointcloud_node.py](../backend/nodes/application/pointcloud_node.py) 는 so101 D405 만 구독
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
| 어느 로봇의 자원인지 | host config 의 `nodes:` entry 가 `robot_id` 까지 명시 (§4 결정) |

## 3. 노드 분배안 (잠정)

### 3.1 제약

- **pyrealsense2 wheel = hori2 에만 source build 됨** ([pyrealsense2-build-guide.md](pyrealsense2-build-guide.md)) → D405 카메라 노드는 hori2 강제
- **MotorNode + MotionNode 같은 머신** — [CLAUDE.md](../CLAUDE.md) "100Hz 제어 명령 네트워크 회피" 원칙
- **USB host = 카메라/모터 노드 host** — USB serial 은 같은 머신에서만 열림

### 3.2 분배

| 머신 | 노드 | 부담 |
|---|---|---|
| PC (hori0?) | detector / task / pointcloud / calibration / bridge (+ gamepad) | YOLO inference / Open3D / TSDF — heavy compute |
| **hori1** | so101 motor + motion | feetech + IK 100Hz |
| **hori2** | so101 D405 카메라 | D405 capture + zstd depth 무손실 압축 (heavy I/O) |
| **hori3** | omx_f motor + motion + omx_f camera (UVC) | dynamixel + IK 100Hz + UVC JPEG (가벼움) |

### 3.3 근거

1. **so101 자원 분산** (hori1 + hori2) → 한 머신 부담 분산. 현재 omx_f 가 pi_motor + pi_camera 에 흩어진 패턴과 일관
2. **D405 zstd 압축 전담** (hori2) — depth 무손실 압축이 cpu heavy. 다른 부담과 묶지 않음
3. **omx 풀스택** (hori3) — UVC 가 가벼우니 dynamixel + IK 와 한 머신에 묶어 자원 균형
4. **architecture 원칙 보존** — motor+motion 같은 머신, 카메라 USB 같은 머신, PC heavy compute 모두 만족

### 3.4 검증해야 할 것 (실측)

- hori2 의 D405 zstd 압축 + Zenoh publish 가 Pi 4 단독 부담으로 OK 인지
- hori1 의 so101 6DOF IK 100Hz + feetech 모터 통신 동시 부담
- so101 자원이 hori1/hori2 에 흩어진 상태에서 ICP / TSDF (PC) 의 motor state 동기화 latency

## 4. robots.yaml `host` 필드 — 제거 결정

### 4.1 문제 (원래 자리)

Phase 1 에 깎아둔 `RobotConfig.hosts: HostMap{motor, camera}` 필드는 **한 robot 이 어느 머신에서 도는가** 를 robots.yaml entry 안에 박으려는 자리였음. 본 문서의 §3.2 분배 (so101 motor=hori3, camera=hori2 처럼 한 robot 이 두 머신에 흩어짐) 를 표현하려고 motor/camera 별로 분리해놨다.

### 4.2 검토 끝에 hosts 자리 자체를 제거

검토 결론: **robots.yaml 에 deployment 정보가 들어가지 않는 게 맞다**. 이유:

**(a) robots.yaml 의 책임 = robot 정체성, deployment 는 다른 차원**

| 항목 | 종류 |
|---|---|
| type / capabilities / motor_backend / kinematics_backend / camera_backend | 정체성 (robot 그 자체) |
| base_pose | 정체성 (world frame 의 물리적 사실) |
| ~~hosts~~ | deployment (옮길 수 있는 배치 결정) |

robot 의 motor 노드를 hori3 → hori1 으로 옮겨도 같은 robot 이고 calibration 그대로 따라옴. "어디 사느냐" 는 정체성 아님.

**(b) "어느 robot 이 어디서 도냐?" 의 두 차원**

- **Intent** ("어디서 떠야 하나?") — config 가 답
- **Actual** ("지금 어디서 떠 있나?") — **runtime state 차원**. Zenoh peer list / heartbeat / 모니터링 대시보드가 답

(a) (hosts SSOT) 의 장점이라고 들이밀어진 "한 곳에서 보임" 시나리오는 사실 actual 질문 — 그건 모니터링 영역이지 config 가 답할 자리 아님. 결국 (a) 의 정당화는 "정체성에 host 포함" 하나로 줄어드는데, 그건 (b) 의 정체성/deployment 분리 원칙에 어긋남.

**(c) host config 가 이미 SSOT — 두 자리 SSOT 만들지 않음**

robot-agnostic 노드 (bridge / task / detector / pointcloud / calibration / gamepad) 는 어차피 robots.yaml 에 못 들어감 → host config 가 책임. robot-scoped 노드 (motor / camera) 만 robots.yaml hosts 가 책임지면 SSOT 가 두 군데로 갈라짐. 같은 종류 정보 (어느 머신에서 뭐 띄울지) 가 두 SSOT 에 흩어지는 게 더 비일관.

**(d) 변경 빈도 차원**

robot 정체성 (calibration / base_pose) 은 새 robot 추가 시에만 바뀜 — 드뭄. deployment (USB hub 배선, hori2 pyrealsense build 깨짐, 부하 분산) 는 더 자주 바뀜. 한 파일에 묶으면 git history 도 섞임.

### 4.3 결론

**host config 의 `nodes:` 리스트가 deployment SSOT**. Phase 2 multi-robot 분산 시 host config 의 nodes entry 가 robot_id 까지 명시:

```yaml
# host_hori1.yaml
host_name: hori1
nodes:
  - {type: motor, robot_id: so101_6dof_0}
  - {type: motion, robot_id: so101_6dof_0}

# host_hori3.yaml
host_name: hori3
nodes:
  - {type: motor, robot_id: omx_f_0}
  - {type: motion, robot_id: omx_f_0}
  - {type: camera, robot_id: omx_f_0}

# host_pc.yaml — robot-agnostic 노드와 같은 SSOT
host_name: pc
nodes:
  - bridge
  - task
  - detector
  - pointcloud
  - calibration
```

robots.yaml entry 는 type / base_pose / capabilities / *_backend 만 들고 deployment 정보 안 가짐.

### 4.4 적용 (이미 완료)

- [robots.yaml](../robot/robots.yaml) — 두 entry 의 `hosts:` 블록 제거
- [robot_registry.py](../backend/core/robot/robot_registry.py) — `HostMap` dataclass + `RobotConfig.hosts` 필드 + 파싱 로직 제거
- [main.py](../backend/main.py) + host config 5개 — `robots:` / `device_nodes:` / `application_nodes:` schema 로 전환. Device (motor/motion/camera) 는 `robots × device_nodes` 데카르트곱 인스턴스, Application (detector/pointcloud/calibration/task/gamepad) 는 한 인스턴스 + 내부 dispatch
- [node_registry.py](../backend/core/transport/node_registry.py) — `(module, cls_name)` 두 string 만 들고 있는 lazy-import 컨테이너. layer 판정은 `issubclass(cls, DeviceNode/ApplicationNode)` 가 SSOT

### 4.5 노드 ownership taxonomy

`hosts` 제거 후속 작업으로 노드 분류 명확화:

| Node | Layer | 이유 |
|---|---|---|
| motor / motion / camera | **Device** | vendor-shipped (UR Control Box 등가물). hardware 직결 (USB). 한 프로세스가 두 머신 USB 못 잡음 → robot 마다 인스턴스 필연 |
| detector / pointcloud / calibration / task / gamepad | **Application** | robot driver 위 algorithm / orchestration / UI. 한 인스턴스 + `dict[robot_id]` dispatch. YOLO / Open3D 모델 메모리 1번 |

이전 *"모든 노드 robot-scoped 인스턴스"* 가정은 outdated. 자세한 진행 상황은 [multi_robot_walkthrough.md §8](multi_robot_walkthrough.md) 표.

## 5. 타입 안전성 — study TODO

현재 [robot_registry.py:52-53](../backend/core/robot/robot_registry.py#L52-L53):

```python
motor_backend: str  # "dynamixel" | "feetech"
kinematics_backend: str       # "pybullet" | "mujoco"
```

valid 값이 yaml 주석으로만 표시 → typo 부팅 시까지 잡힘 + factory exhaustiveness pyright 미보장.

Phase 2 진입 시 적용:

```python
from typing import Literal

MotorBackendName = Literal["dynamixel", "feetech"]
KinematicsBackendName = Literal["pybullet"]   # mujoco 는 Track C 진입 시 추가

@dataclass(frozen=True)
class RobotConfig:
    motor_backend: MotorBackendName
    kinematics_backend: KinematicsBackendName
```

+ `_build_config` 에서 yaml load 시 set 멤버십 체크 (fail-fast). enum 대비 yaml 친화 + Phase 2 새 backend 추가가 가벼움.

## 6. Protocol 네이밍 통일 — mini refactor ✅ 완료

본 절 6.1~6.5 는 *refactor 전* 의 design plan. 아래 변경 적용됨:
- `CameraCaptureProtocol` → `CameraCapture` (Protocol 이름 회수)
- `CameraCapture` (RealSense impl) → `RealsenseCapture` ([modules/camera/adapters/realsense_capture.py](../backend/modules/camera/adapters/realsense_capture.py))
- [capture.py](../backend/modules/camera/capture.py) 는 Protocol + data classes 만
- **`camera_backend` selector layout 추가** (motor/ik 와 동일 패턴) — `CameraBackendName = Literal["realsense", "opencv"]` + `RobotRegistry.get_camera_capture(robot_id)` factory + `robots.yaml` 의 `camera_backend:` 필드. opencv impl 은 placeholder (`NotImplementedError`) — SO-101 도착 (§1) 시 작성. mujoco 는 Track C 진입 시 추가 (현재 Literal 에서 제외 — 코드 없는데 placeholder 만 두는 거 정리)
- **후속 통일 (Jun 3)**: raw SDK wrap 을 별도 파일로 분리해 motor 도메인의 `*Driver` / `*Backend` 어휘와 정합 — `realsense_capture.py` (raw SDK 가 들어있던 자리) → [`realsense_driver.py::RealsenseDriver`](../backend/modules/camera/adapters/realsense_driver.py), Protocol impl 자리 `realsense.py::RealSenseCapture` → [`realsense_capture.py::RealsenseCapture`](../backend/modules/camera/adapters/realsense_capture.py). 이로써 (Protocol `CameraCapture` ← impl `RealsenseCapture` ← raw `RealsenseDriver`) 가 motor (Protocol `MotorBackend` ← impl `DynamixelBackend` ← raw `DynamixelDriver`) 와 동형.

### 6.1 변경 전 현황 (일관성 깨짐)

| Protocol | 구현체 | 접미사 |
|---|---|---|
| [`Kinematics`](../backend/modules/kinematics/kinematics.py) | `PybulletKinematics`, `MujocoIKSolver` | ❌ 없음 |
| [`MotorBackend`](../backend/modules/motor/backend.py) | `DynamixelBackend`, `FeetechBackend` | ❌ 없음 |
| `CameraCaptureProtocol` (구) | `CameraCapture` (구, RealSense wrap) | ✅ **`Protocol` 붙음** |

### 6.2 원인

`CameraCapture` 만 구현체 이름이 도메인 그 자체. Protocol 도 같은 이름이면 한 모듈에서 이름 충돌 → Protocol 쪽이 `Protocol` 접미사로 양보.

### 6.3 카메라 모듈의 폴더 구조도 불일치

kinematics / motor 는 `<module>/adapters/<impl>.py` 패턴을 따르는데 camera 만 Protocol + 구현체가 한 파일에 박혀있음:

```
backend/modules/
├── kinematics/
│   ├── kinematics.py              ← Protocol
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

다른 두 Protocol (Kinematics / MotorBackend) 의 `Pybullet*` / `Dynamixel*` prefix + `adapters/<impl>.py` 패턴과 일치.

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

## 8. Cache 류 robot_id 차원 도입 — 완료

### 8.1 현황

| 클래스 | robot_id 차원 |
|---|---|
| [`JointStateCache`](../backend/core/cache/joint_state_cache.py) | ✅ done (Phase 1 sub-A, commit `e8f75ea`) |
| [`JointCoordinates`](../backend/core/coords/joint_coordinates.py) | ✅ done |
| [`LinkCoordinates`](../backend/core/coords/link_coordinates.py) | ✅ done |
| [`SagCoordinates`](../backend/core/coords/sag_coordinates.py) | ✅ done |
| [`RobotRegistry`](../backend/core/robot/robot_registry.py) | ✅ done (factory per-robot 캐시) |
| [`FrameCache`](../backend/core/cache/frame_cache.py) | ✅ **done** (commit `2270eba` 와 묶어서) — 토픽 namespace 정정 시 같이 마이그레이션 |

### 8.2 FrameCache 모양 (참고)

```python
self._latest_jpeg_by_robot: dict[str, bytes] = {}
self._latest_status_by_robot: dict[str, CameraStatus] = {}
self._subscribed_robots: set[str] = set()

def subscribe(self, node, robot_id: str | None = None): ...    # robot 별 토픽 구독
def get_frame(self, robot_id: str | None = None) -> tuple[bool, np.ndarray | None]: ...
```

`JointStateCache` 와 같은 패턴 — `dict[robot_id]` 화 + 모든 method 에 `robot_id` 인자 (None 이면 default).

### 8.3 트리거 (완료된 히스토리)

토픽 namespace 정정 (`omx/camera/stream/raw` → `horibot/{robot_id}/camera/stream/raw`) 과 묶어서 commit `2270eba` 에서 처리. `JointStateCache` 와 동형 `dict[robot_id]` 패턴 적용.

## 9. 관련 문서

- [multi_robot_architecture.md](multi_robot_architecture.md) — 핵심 abstraction (§3 Protocols, §4 robot identity, §5 디렉토리)
- [so101_6dof_plan.md](so101_6dof_plan.md) — SO-101 하드웨어 (§5 카메라 마운트는 본 문서 §1 결정으로 변경됨)
- [operations.md](operations.md) — Pi/IP/OS 셋업
- [pyrealsense2-build-guide.md](pyrealsense2-build-guide.md) — pyrealsense2 source build (hori2 전용 제약 origin)
- [multi_robot_walkthrough.md](multi_robot_walkthrough.md) — Phase 1 산출물 학습 anchor
