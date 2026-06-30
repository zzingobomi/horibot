# backend_v2 — Module catalog

> [backend_v2.md](backend_v2.md) framework spec 위에 박힐 **Module catalog** — 12 Module / 4 layer / 3 host 배치. framework anchor (§14 18 결정) 의 자연 연장.
>
> 본 문서가 다루는 것: Module 이름 / 책임 / scope / host / cross-module 의존 / Module SDK + capability 패턴.
> 다루지 않는 것: ORM schema (Step 6+), business logic 옮겨심 (Step 9), framework spec 자체 (backend_v2.md SSOT).

## 1. 4 layer + 12 Module

§2 catalog (46 책임) 를 한 발 떨어져 보면 **4 layer 의 자연 분리** — 강제 layer architecture 아닌, 책임의 본질이 다른 묶음. framework 는 layer 모름 (duck typing).

```
┌──────────────────────────────────────────────────────────────┐
│ Layer 4 — Boundary    : Bridge, Gamepad                      │
│ Layer 3 — Orchestration: Task                                │
│ Layer 2 — Domain      : Motion, Calibration, Detector,        │
│                         Scene3D, Scan, Reconstruction        │
│ Layer 1 — Hardware    : MotorDriver, CameraDriver,           │
│           + Derived     CameraDecoded                        │
└──────────────────────────────────────────────────────────────┘
```

### 1.1 12 Module catalog

| # | Module | Layer | Scope | Host | 한 줄 책임 | 영속성 |
|---|---|---|---|---|---|---|
| 1 | **MotorDriver** | Hardware | robot-scoped | pi_motor | Dynamixel / Feetech raw 통신 (state 20Hz + command 100Hz + torque/reboot/gripper) | X |
| 2 | **CameraDriver** | Hardware | robot-scoped | pi_camera | RealSense capture + JPEG encode + depth zstd | X |
| 3 | **CameraDecoded** | Derived | robot-scoped | pc | JPEG → BGR ndarray + zstd → uint16 depth (decode dedup, 두 stream) | X |
| 4 | **Motion** | Domain | robot-scoped | pi_motor | kinematics (PyBullet + SagCorrected) + motion primitive (Move/Servo/Jog) + TcpState publish | X |
| 5 | **Calibration** | Domain | robot-agnostic | pc | 5종 산출물 Bundle owner (intrinsic / hand_eye / joint / link / sag) + capture loop + offline BA result | DB + ObjectStore |
| 6 | **Detector** | Domain | robot-agnostic | pc | YOLO + Grounding DINO + LLM. detect (project to base) | X |
| 7 | **Scene3D** | Domain | robot-agnostic | pc | RGBD → pointcloud primitive (라이브 stream + snapshot N-frame consensus) | X |
| 8 | **Scan** | Domain | robot-agnostic | pc | scan capture (Scene3D snapshot → blob) + session 관리 (CASCADE) | DB + ObjectStore |
| 9 | **Reconstruction** | Domain | robot-agnostic | pc | scan session N blob → ICP + PoseGraph + TSDF + mesh extract | DB + ObjectStore |
| 10 | **Task** | Orchestration | robot-agnostic | pc | Step DSL 실행 + Slot + debugger + recipe (pick_and_place / scan) | DB (history) |
| 11 | **Gamepad** | Orchestration | robot-agnostic | pc | 8BitDo Ultimate 2C polling + Jog dispatch (capabilities='gamepad' robot) | X |
| 12 | **Bridge** | Boundary | robot-agnostic | pc | WebSocket gateway + MJPEG stream + HTTP endpoint (`/robots`, `/tasks`, `/system` — framework helper relay) + static `/robot` | X |

### 1.2 합치지 / 더 잘라지 않은 자리 — motivation

**합치지 않은 자리**:

| 합칠 가능성 | 분리 유지 이유 |
|---|---|
| MotorDriver + Motion | vendor SDK swap (Dynamixel → Feetech) 시 Motion 의 kinematics / Ruckig logic 변경 0 |
| CameraDriver + CameraDecoded | pi_camera 강제 (pyrealsense2 USB) vs pc (decode CPU + consumer 모임) — host 횡단 |
| Scene3D + Scan + Reconstruction | primitive / workflow / heavy compute — trigger 와 cost profile 모두 다름 |
| Calibration + Detector | 영속성 owner vs perception. Detector 는 Calibration Mirror 의 Reader |
| Scan + Reconstruction | write-heavy (매 capture) vs read-heavy + CPU (1 회 build). 별 entity |
| Detector + Scene3D | 2D box → base 좌표 vs depth → 3D geometry. 자산 / 출력 다름 |

**더 잘라지 않은 자리**:

| 잘랐을 가능성 | 합쳐 유지 이유 |
|---|---|
| TcpState / JointRad 별도 Module | small payload (수십 bytes × 20Hz), dedup < 1%. Motion 안 fk + publish SSOT |
| ChArUco detect 별도 Module | Calibration capture 안에서만 쓰임. 외부 consumer 0 |
| LLM 별도 Module | model load 1회 + preload race ([llm_preload_race_debug.md](llm_preload_race_debug.md)) Detector 안 자연 |
| Trajectory / IK / Jog 분리 | 같은 kinematics object 공유. 분리 = duplicate |
| MotorState / MotorCommand 분리 | 같은 hardware handle. 한 Module 안 양방향 자연 |

**근거 한 줄** — 한 Module 이 한 정직한 책임 묶음. 부풀어도 흩뿌려도 안 됨.

## 2. Scope + Host placement

### 2.1 Scope — [backend_v2.md §2.7](backend_v2.md) 기준

| Scope | Module | 판단 근거 |
|---|---|---|
| **robot-scoped (4)** | MotorDriver, CameraDriver, Motion, CameraDecoded | 한 robot 의 물리 자원 / 카메라 frame cache owner |
| **robot-agnostic (8)** | Calibration, Detector, Scene3D, Scan, Reconstruction, Task, Gamepad, Bridge | host 당 1 인스턴스, 매 req.robot_id 로 dispatch (DB 의 `robot_id` column 으로 multi-tenant) |

**Gamepad** — gamepad 자체가 *host* 자원 (USB). dispatch 대상은 `capabilities='gamepad'` 박힌 robot 1개 (N>1 fail-fast).

### 2.2 Hardware 토폴로지 (활성: so101 + D405)

[CLAUDE.md](../CLAUDE.md) 분산 토폴로지 표 + [[project-active-robot-so101-d405]] (OMX detach).

| 머신 | IP | hardware | 핵심 보장 |
|---|---|---|---|
| **pc** | 개발 머신 | gamepad USB / no robot USB | 무거운 연산 (YOLO / Open3D / TSDF) + browser bridge + DB owner |
| **pi_motor** | 192.168.0.101 | Feetech STS3215 (so101) | 100Hz `MOTOR_CMD_JOINT` network 안 넘게 |
| **pi_camera** | 192.168.0.102 | Intel RealSense D405 | USB 대역폭 경합 회피 (motor controller 와 다른 USB 컨트롤러) |

### 2.3 12 Module 의 host 배치

| Host | Module | 이유 |
|---|---|---|
| **pi_motor** | MotorDriver, Motion | 100Hz 명령 network 안 넘는 강제. kinematics + IK 도 pi_motor (RTT 0) |
| **pi_camera** | CameraDriver | pyrealsense2 USB 강제 |
| **pc** | CameraDecoded, Calibration, Detector, Scene3D, Scan, Reconstruction, Task, Gamepad, Bridge | decode CPU + 무거운 연산 + DB owner + browser + gamepad USB |

### 2.4 deployment yaml — operational / mock / dev / sim

| yaml | 의미 | Module 배치 | driver |
|---|---|---|---|
| `pc.yaml` / `pi_motor.yaml` / `pi_camera.yaml` | 운영 분산 | 위 §2.3 | real (realsense / feetech) |
| `mock.yaml` | hardware 없이 UX 검증 | 12 Module 모두 한 process | `mock` impl (driver subdir) |
| `dev.yaml` | 단일 머신 풀스택 | 12 Module 모두 한 process | real |
| `pc_sim.yaml` / `pi_motor_sim.yaml` / `pi_camera_sim.yaml` | localhost 3 process distributed 검증 | 운영과 동일 분배 | mock |

mock 은 별도 Module 박지 않음 — `modules/<domain>/drivers/mock.py` 로 driver subdir swap.

## 3. Cross-module 의존

### 3.1 Mirror / service call

| Reader | 사용 패턴 | Source |
|---|---|---|
| Motion | **Mirror[CalibrationBundle]** | Calibration (link_offset 변경 시 kinematics rebuild) |
| Calibration | call (capture) + subscribe (5Hz preview) | CameraDecoded (preview stream) / Motion (capture 시 TCP_SNAPSHOT) |
| Detector | call (매 detect, consumer request dependent) | Calibration / CameraDecoded / Motion (TCP_SNAPSHOT) |
| Scene3D | call (매 snapshot req) | CameraDecoded |
| Scan | call (매 capture) | Scene3D / Motion (TCP_SNAPSHOT) |
| Reconstruction | call (1 회 build) + local kinematics 인스턴스 | Scan / Calibration (snapshot 매 build, Mirror X) |
| Task | call (step 별) | 모두 |
| Gamepad | publish stream | Motion (one-way) |
| Bridge | subscribe + relay | 모두 |

**Mirror 사용 자리 = Motion 하나** (control correctness state). 다른 Reader 는 매 호출 service call (per-request fresh). [backend_v2.md §4.4](backend_v2.md) robot-agnostic Reader 패턴.

### 3.2 Mirror invariant — control correctness state only

> **Mirror = control correctness state synchronization.**
> Continuously changing runtime telemetry / observation streams (TCP pose / joint state / motor state / force / temperature) **must NOT** use Mirror.
> point-in-time state 필요 → snapshot service. continuous flow → stream subscription.

판단 표:

| 종류 | 예시 | 자리 |
|---|---|---|
| **Control correctness state** (변경 시 consumer 가 *재계산* 필요) | CalibrationBundle (link_offset 변경 → kinematics rebuild) | **Mirror** ✅ |
| **Runtime telemetry** (지속 변화, observation) | TCP pose 20Hz, joint state 20Hz, motor state, force, temperature | **snapshot service** (point-in-time) 또는 **stream subscribe** (continuous) — Mirror ❌ |
| **Static fact** (boot 1회, 변경 X) | Capabilities | **snapshot service** 1회 cache — Mirror ❌ ([§7](#7-capability-layer)) |

본 invariant 가 미래 MotorState / ForceState / Temperature 도 자연 적용.

### 3.3 Motion stream catalog

§4.1 의 Camera stream 처럼 Motion 의 stream key 명시:

```python
class Motion:
    class Stream(StrEnum):
        TCP_STATE   = "stream/motion/{robot_id}/tcp_state"     # 20Hz fk + sag corrected
        JOINT_STATE = "stream/motion/{robot_id}/joint_state"   # 20Hz raw → rad

    class Service(StrEnum):
        TCP_SNAPSHOT = "srv/motion/{robot_id}/tcp_snapshot"    # point-in-time (Detector / Scan / Calibration)
        FK           = "srv/motion/{robot_id}/fk"              # joints → TCP pose (compute, 자세 추정 시)
        # ... Move/Servo/Jog services
```

**stream vs service 분리**:
- `Motion.Stream.TCP_STATE` = 20Hz continuous (Bridge 의 frontend 시각화 / scope 등 살아있는 표시)
- `Motion.Service.TCP_SNAPSHOT` = consumer 의 point-in-time (Detector / Scan / Calibration capture 시 1 회)

Jog command stream (gamepad / frontend 50Hz publish → motion subscribe) 의 자리는 motion command 영역 — 본 catalog 박지 X (Step 6+ 자리).

### 3.4 도메인 event

| Owner | event | payload |
|---|---|---|
| Calibration | **CalibrationActivated** (bundle 변경 — Mirror trigger) | `{robot_id, bundle_id, version}` |
| Calibration | CalibrationCommitted (run finalize) | `{robot_id, run_id}` |
| Scan | ScanCaptured / ScanSessionFinalized | — |
| Reconstruction | ReconstructionBuilt + ReconstructionProgress (5-stage) | — |
| Motion | MotionCompleted (Move 끝) — Servo/Jog 는 stream, event X | — |
| Task | TaskTree / TaskState (RUNNING/PAUSED/...) / StepResult | — |

**CalibrationActivated 의 versioning** — `bundle_id` (DB row id) + `version` (monotonic). Mirror[CalibrationBundle] 의 consumer (Motion) 가 어떤 bundle 위에서 kinematics rebuild 박았는지 추적. 미래 audit / debug 자연.

## 4. Derived read model — CameraDecoded

[backend_v2.md §3.5](backend_v2.md) 의 decode dedup 패턴. **적용 자리 = Camera 만**.

**측정** — JPEG 1280×720 decode = 4.34ms × 30Hz:
- 각 consumer 별 decode (N=3): 39% CPU
- decode 1회 + ndarray transport (N=3): **21% CPU** ← 본 design

**미적용 자리** — 작은 payload 의 derived 는 SSOT 분산만:

| 자리 | derived? | 이유 |
|---|---|---|
| Motor raw → joint rad | ❌ Motion 안 | 수십 bytes × 20Hz, dedup < 1%. `Motion.Stream.JOINT_STATE` publish |
| Joint → TCP pose (fk) | ❌ Motion 안 | 단순 PyBullet fk, kinematics SSOT. `Motion.Stream.TCP_STATE` (20Hz) + `Motion.Service.TCP_SNAPSHOT` (point-in-time) |
| Scan blob / Reconstruction mesh | ❌ | 1 회 build / serve, fanout X |

### 4.1 CameraDecoded — 한 Module 두 stream

```python
class Camera:
    class Stream(StrEnum):
        JPEG          = "stream/camera/{robot_id}/jpeg"             # CameraDriver publish (raw color)
        DEPTH_RAW     = "stream/camera/{robot_id}/depth_raw"        # CameraDriver publish (zstd depth)
        DECODED       = "stream/camera/{robot_id}/decoded"          # CameraDecoded publish (BGR ndarray)
        DEPTH_DECODED = "stream/camera/{robot_id}/depth_decoded"    # CameraDecoded publish (uint16 ndarray)
```

**왜 두 stream**:
- 같은 source (RealSense 동시) + 같은 host + 같은 robot → 한 Module 책임 자연
- consumer profile 다름 — Detector / Bridge 는 color 만, Scene3D 는 둘 다. 합치면 wire 낭비
- Bridge 는 `Camera.Stream.JPEG` 직접 subscribe (decode 안 거침, raw forward)

## 5. Framework 어휘 vs Application 어휘

[backend_v2.md §2.1](backend_v2.md) "Distribution is runtime concern" 의 자연 연장 — robot detail 도 *application concern, framework primitive X*.

### 5.1 Framework 가 아는 어휘

| 어휘 | 의미 |
|---|---|
| Module class + lifecycle | `@service` / `@subscriber` / `@publishes` / `Mirror` 박힌 plain class |
| service / event / stream key | StrEnum value (raw string X) |
| payload class | Pydantic BaseModel |
| `{robot_id}` placeholder | key 안 substitute |
| `robots: [...]` yaml 박힘 | per-robot 인스턴스화 |
| constructor parameter | DI inject 대상 |

이 어휘 **밖** 의 robot detail (driver type / capability / 마운트 / urdf / capacity) 은 모름.

### 5.2 Application SSOT — robots.yaml + apps/main.py

```yaml
# robot/robots.yaml — schema 예 (detail 은 Step 6+)
robots:
  so101_6dof_0:
    type: so101
    base_pose: {x: 0, y: 0, z: 0, yaw_deg: 0}
    motor_driver: feetech                    # MotorDriver 의 drivers/ 어느 impl
    motor_port: /dev/ttyACM0
    camera: {id: wrist, driver: realsense_d405}   # 단수 — 1 robot 1 camera (multi-camera 는 §10 후속)
    capabilities: [gamepad, rgbd]                 # robot-level high-level summary

  omx_f_0:
    type: omx_f
    motor_driver: dynamixel
    camera: null                                  # 카메라 없음 → CameraDriver 인스턴스화 X
    capabilities: []
```

```python
# apps/main.py — boot logic
def main(host: str):
    deploy_cfg = load_yaml(f"deployments/{host}.yaml")
    robots_cfg = load_yaml("robot/robots.yaml")
    runtime = Runtime(transport=ZenohTransport(...))

    for mod_cfg in deploy_cfg.modules:
        mod_cls = MODULE_REGISTRY[mod_cfg.name]
        if mod_cfg.robots:
            for rid in mod_cfg.robots:
                deps = resolve_deps(mod_cls, robots_cfg.get(rid))   # ★ application logic
                runtime.add_module(mod_cls, robot_id=rid, **deps)
        else:
            runtime.add_module(mod_cls, **resolve_host_deps(mod_cls, deploy_cfg))
    runtime.start()
```

`resolve_deps` 가 application 책임 — driver impl 선택 / robot config inject. framework 의 `add_module` 은 그저 kwargs → constructor.

### 5.3 Module 의 constructor — driver 종류 모름

```python
class MotorDriverModule:
    def __init__(self, runtime: ModuleRuntime, robot_id: str, driver: MotorBackend):
        self._driver = driver        # ★ Protocol — Dynamixel / Feetech / mock 모름
```

`MotorBackend` Protocol = `modules/motor/drivers/protocol.py` (framework 안 박지 X).

### 5.4 Capability 기반 dispatch — Module 안 application logic

robot-agnostic Module 이 *rgbd capability 박힌 robot 만* 처리. framework 어휘에 capability 박지 X — Module 내부:

```python
class Scene3DModule:
    def __init__(self, runtime, robots: list[RobotConfig]):
        self._rgbd_robots = {r.id for r in robots if "rgbd" in r.capabilities}

    @service(Scene3D.Service.SNAPSHOT)
    def snapshot(self, req):
        if req.robot_id not in self._rgbd_robots:
            raise InvalidRequest(f"robot {req.robot_id} rgbd capability 없음")
```

## 6. Module SDK — bounded context

각 Module 이 자기 도메인의 SDK. driver 공통 abstraction = Module SDK 안 (`drivers/protocol.py`), framework X — 안 그러면 Gripper / Lidar / PLC 추가될 때마다 framework 가 부풀음 = 모든 산업 장비 SDK.

### 6.1 폴더 구조

```
horibot/
├── framework/                        # 도메인 모름 (runtime / transport / contract / persistence / storage)
├── modules/
│   ├── camera/
│   │   ├── contract.py               # Public Surface — Service/Stream key + payload + capability
│   │   ├── module.py                 # framework entry (@service / @publishes)
│   │   └── drivers/
│   │       ├── protocol.py           # CameraDriver Protocol
│   │       ├── realsense_d405.py
│   │       ├── usb_uvc.py
│   │       └── mock.py
│   ├── motor/  ⤴ drivers/{dynamixel,feetech,mock}.py
│   ├── motion/                       # drivers X — kinematics 자체 logic
│   ├── calibration/  detector/  scene3d/  scan/  reconstruction/  task/  gamepad/  bridge/
├── apps/main.py                      # robots.yaml + deployment yaml → resolve_deps
└── infra/                            # framework Protocol 의 실 impl
    ├── transport/  database/  object_store/
```

**`drivers/` 박을 자리** = hardware adapter swap 책임 (MotorDriver / CameraDriver / 미래 Gripper / Lidar).
**`drivers/` 안 박는 자리** = logic 자체가 Module 책임 (Motion / Calibration / Detector / Scene3D / Scan / Reconstruction / Task / Bridge).

### 6.2 driver Protocol — 도메인의 공통 계약

```python
# modules/camera/drivers/protocol.py
class CameraDriver(Protocol):
    def capabilities(self) -> CameraCapabilities: ...
    def capture(self) -> ColorFrame: ...
    def capture_depth(self) -> DepthFrame | None: ...

# modules/camera/module.py
class CameraDriverModule:
    def __init__(self, runtime, robot_id, driver: CameraDriver):
        self._driver = driver                        # Protocol 만 — D405 / USB 모름
```

### 6.3 3 계층 분리

1. **framework** — service/event/stream + Module instantiate. 도메인 모름.
2. **Module SDK** (`modules/<domain>/`) — 도메인 contract + driver Protocol + driver impl.
3. **consumer Module** — framework 어휘로 호출. driver Protocol 도 모름.

### 6.4 미래 확장 — 새 hardware / 새 도메인

| 추가 | 박힐 자리 | framework 변경 | 다른 Module 변경 |
|---|---|---|---|
| 새 Camera vendor (Basler / FLIR) | `modules/camera/drivers/<vendor>.py` | 0 | 0 |
| 새 도메인 (Gripper / Lidar / Force / PLC) | `modules/<domain>/` + `drivers/` | 0 | 0 |

## 7. Capability layer

UI / consumer 가 hardware 차이 (D405 = rgbd / USB = rgb only / 5축 vs 6축) 를 알아야 dispatch / 기능 노출 결정. capability = "이 device 가 무엇을 지원하는가" 의 선언.

### 7.1 invariant

> **Topology = "무엇이 존재하는가" (구조 존재).**
> **Capability = "무엇을 할 수 있는가 / 어떤 결과를 제공하는가" (외부 노출 기능 + 지원 max metadata).**
> **Config = "현재 어떻게 설정되어 있는가" (현재 값).**

세 어휘의 시간축 / 의미축 분리:

| 종류 | 의미 | 예시 | 변경 빈도 | 자리 |
|---|---|---|---|---|
| **Topology** | 무엇이 존재하는가 (structure) | Motor: `[id=1 kind=joint, ..., id=7 kind=gripper]` | 부팅 시 1회, 변경 X | `@service topology` (consumer 가 구조 알아야 할 때만) |
| **Capability** | 무엇을 할 수 있는가 / 어떤 결과 제공 (feature + supported max) | Camera: `RGB/DEPTH/POINTCLOUD` flag / `max_resolution=(1280,720)` / `supported_fps=[30,60]` | 부팅 시 1회, 변경 X | `@service capabilities`, flags set + metadata |
| **Config** | 현재 설정 값 (지원 max 와 별) | Camera: 현재 `resolution=(640,480)` / `fps=30` | 사용자 설정 | 별 service (`get_config` / `set_config`) |
| **Runtime state** | 현재 status | streaming on / torque enabled | 자주 변경 | event + Mirror |

**핵심 — Capability vs Config 의 시간축 분리**:
- "지원 가능한 max resolution" = Capability ("가능한 것")
- "현재 선택된 resolution" = Config ("현재 값")
같은 어휘 (resolution) 라도 시간축이 다르면 별 자리.

### 7.2 Topology 박는 기준 — consumer-driven, 도메인-driven X

> **Topology 를 만드는 기준 = consumer 가 구조 자체를 알아야 하는가.**

도메인 별 일률 적용 X. consumer 의 contract surface 가 기준:

| 도메인 | Topology 필요? | 이유 |
|---|---|---|
| **Motor** | ✅ | Motion Module 이 `driver.send_positions(motor_ids, ...)` 처럼 wire-level 구조 직접 소비. for loop / IK 매핑 / dispatch 자리 다 motor 단위 |
| **Camera** | ❌ | consumer 는 "RGB / DEPTH / pointcloud 얻을 수 있나" 만 봄. IR sensor 갯수 / depth ASIC / stereo baseline 자리 안 봄. RGB/DEPTH 자체가 capability 어휘 (외부 feature) 자연 |

Motor 의 `motor_ids` 자리가 *wire-level 외부 노출 정보*. Camera 의 IR sensor 자리가 *내부 구조* (consumer 안 봄). 두 도메인의 어휘 비대칭은 자연 — 도메인 평행 적용 무리.

### 7.3 schema — contract.py 안

**Motor — Topology + Capability 분리**

```python
# modules/motor/contract.py
class Motor:
    class Service(StrEnum):
        CAPABILITIES = "srv/motor/{robot_id}/capabilities"
        GET_TOPOLOGY = "srv/motor/{robot_id}/topology"
        SET_TORQUE   = "srv/motor/{robot_id}/set_torque"
        # ...

class MotorKind(StrEnum):
    JOINT = "joint"
    GRIPPER = "gripper"
    RAIL = "rail"        # 미래 linear axis
    TOOL = "tool"        # 미래 spindle / vacuum

class MotorInfo(BaseModel):
    id: int
    kind: MotorKind

class MotorTopology(BaseModel):
    motors: list[MotorInfo]
    # joint_count / has_gripper = derived (sum / any). 중복 박지 X

class MotorCapability(StrEnum):
    TORQUE_TOGGLE = "torque_toggle"
    REBOOT = "reboot"
    VELOCITY_CONTROL = "velocity_control"
    CURRENT_CONTROL = "current_control"
    HOMING = "homing"
    # POSITION_PID 박지 X — 모든 servo 의 baseline (MotorBackend Protocol 기본 계약)
    # GRIPPER 박지 X — Topology 위 `any(m.kind == GRIPPER)` 로 derived

class MotorCapabilities(BaseModel):
    flags: set[MotorCapability]
```

**Camera — Capability 만 (Topology X)**

```python
# modules/camera/contract.py
class Camera:
    class Service(StrEnum):
        CAPABILITIES = "srv/camera/{robot_id}/capabilities"
        GET_CONFIG   = "srv/camera/{robot_id}/config"          # 현재 설정 값 — 별 service
        DECODED_SNAPSHOT = "srv/camera/{robot_id}/decoded_snapshot"
        # GET_TOPOLOGY 박지 X — consumer 안 봄
        # GET_INTRINSICS 박지 X — intrinsic SSOT = Calibration Bundle (§7.6)

class CameraCapability(StrEnum):
    RGB = "rgb"
    DEPTH = "depth"
    POINTCLOUD = "pointcloud"
    HDR = "hdr"
    AUTO_EXPOSURE = "auto_exposure"

class CameraCapabilities(BaseModel):
    flags: set[CameraCapability]
    # 미래 metadata — supported max 자리 (Capability 어휘 안 흡수)
    max_resolution: tuple[int, int] | None = None
    supported_fps: list[int] = Field(default_factory=list)
    # 현재 resolution / fps 자리 = CameraConfig (별 service)
```

contract.py 안 inline 첫 박을 때. 커지면 `capability.py` / `topology.py` 분리 + contract.py 재export.

**핵심 차이**: Motor 의 `motors[id, kind]` 가 *consumer 가 wire 시 직접 사용* 하는 정보. Camera 의 RGB/DEPTH 가 *consumer 가 외부 feature 로 보는* 정보. 같은 "structure" 어휘여도 contract surface 위 자리가 달라서 별 schema.

### 7.4 값의 SSOT = driver self-declare

```python
class RealSenseD405:
    def capabilities(self) -> CameraCapabilities:
        return CameraCapabilities(
            flags={CameraCapability.RGB, CameraCapability.DEPTH, CameraCapability.POINTCLOUD},
            max_resolution=(1280, 720),
            supported_fps=[15, 30, 60, 90],
        )

class UsbUvc:
    def capabilities(self) -> CameraCapabilities:
        return CameraCapabilities(
            flags={CameraCapability.RGB},
            max_resolution=(1920, 1080),
            supported_fps=[30, 60],
        )

class FeetechSO101:
    def topology(self) -> MotorTopology:
        return MotorTopology(motors=[
            MotorInfo(id=1, kind=MotorKind.JOINT),
            # ... id=2..6 joint
            MotorInfo(id=7, kind=MotorKind.GRIPPER),
        ])
    def capabilities(self) -> MotorCapabilities:
        return MotorCapabilities(flags={
            MotorCapability.TORQUE_TOGGLE,
            MotorCapability.REBOOT,
        })
```

```python
# modules/<domain>/module.py
class CameraDriverModule:
    def __init__(self, runtime, robot_id, driver: CameraDriver):
        self._driver = driver
        self._capabilities = driver.capabilities()       # boot 1회 read + cache

    @service(Camera.Service.CAPABILITIES)
    def get_capabilities(self, req) -> CameraCapabilities:
        return self._capabilities                        # static, snapshot 1회로 충분 (Mirror X)
```

Motor 자리 Module 도 동일 패턴 — `topology()` + `capabilities()` 둘 다 boot 1회 cache + 두 service relay.

**왜 driver self-declare 이지 robots.yaml X**:
- yaml 박으면 duplication (driver impl + yaml 둘 다 안다)
- yaml 잘못 박으면 inconsistency (D405 박혔는데 depth flag 빠뜨림)
- yaml 의 자리 = "어느 driver 박혀있나" 만. flag / topology 는 driver SSOT

### 7.5 UI access pattern

```typescript
// boot 시 1회 snapshot fetch + local cache
const caps = await Promise.all(robots.map(r => ({
  robot_id: r.id,
  camera: await bridge.call(Camera.Service.CAPABILITIES, { robot_id: r.id }),
  motion: await bridge.call(Motion.Service.CAPABILITIES, { robot_id: r.id }),
})));

if (caps[id].camera.flags.has("depth"))      showPointCloudPanel();
if (caps[id].motion.flags.has("cartesian_move")) showMoveLButton();
```

UI 는 D405 / UR / Doosan / Basler 모름 — capability flag 만 봄. 새 hardware 추가 시 UI 변경 0.

### 7.6 책임 분리 — owner Module 별

| capability 자리 | owner | example |
|---|---|---|
| Camera | `modules/camera` | rgb / depth / pointcloud / hdr / auto_exposure |
| Motion / Motor | `modules/motion` | joint_move / cartesian_move / jog / force_control |
| Motor (low-level) | `modules/motor` | torque_toggle / reboot / velocity_control / current_control / homing |
| Force sensor (미래) | `modules/force` | calibrated / units / axes |
| Gripper (미래) | `modules/gripper` | adaptive / suction / parallel |
| High-level composition (pick_and_place) | UI 또는 별 Module | camera.depth ∧ motion.cartesian ∧ motor.gripper |

high-level composition 첫 박을 때 명시 X — module-level capability 만 노출, UI 가 AND. Gripper 존재 자리는 Motor Module 의 Topology (motors 안 `kind=GRIPPER`) 자리 derived — 별 capability flag 박지 X.

### 7.7 Intrinsic SSOT — Camera factory vs Calibration calibrated

intrinsic 어휘의 **두 자리 분리**:

| 자리 | owner | 어휘 | 노출 |
|---|---|---|---|
| **Factory intrinsic** | CameraDriver SDK internal | `driver.get_factory_intrinsics()` | driver method only (Calibration capture 의 seed 용도) |
| **Calibrated intrinsic** | Calibration | Calibration Bundle 의 `intrinsic` field | public service (`Calibration.Service.SNAPSHOT_BUNDLE`) |

**Camera Module 의 public contract 에 `GET_INTRINSICS` 박지 X** — consumer 입장에서 "intrinsic" 어휘 한 단어가 두 의미 되면 ambiguity. 모든 consumer (Detector / Scene3D / Scan / Reconstruction) 는 Calibration Bundle 의 intrinsic 만 봄 = SSOT.

factory intrinsic 자리 = Calibration capture 의 첫 BA seed (intrinsic 캘 시작 시 `driver.get_factory_intrinsics()` 호출 → seed → BA 산출 → Calibration Bundle 의 active calibrated).

## 8. Public contract surface — 두 generator 의 SSOT

`contract.py` 가 **두 별 생성기의 source**. Module / Bridge / runtime 와 무관한 자리.

### 8.1 두 소비자, 한 SSOT

| 소비자 | 명령 | 산출 | 목적 |
|---|---|---|---|
| **Frontend (runtime client)** | `pnpm gen:types` | TypeScript interface (request / response / event / stream payload) | frontend 컴파일 안정성, IDE 자동완성 |
| **Developer (contract viewer)** | `python -m horibot.contract_viewer` (또는 비슷한 별 command) | API catalog viewer (Swagger-like) | "어떤 Module / service / event / stream 있나" introspection |

**둘 다 source = `modules/<domain>/contract.py`** — SSOT. 단 *생성기 두 갈래* / *소비자 두 갈래* / *런타임 무관*.

backend/ 의 backend_v2.md §3.0 의 `custom_openapi()` 자리는 **Frontend 연동의 build-time generator** 인지, **runtime contract viewer** 인지 — 본 design 에선 분리. Bridge 는 둘 다 자리 X.

### 8.2 contract.py 의 노출 자리

Module 의 파일 별 — 두 generator 의 read 대상:

| 파일 | TS type gen | catalog viewer | 이유 |
|---|---|---|---|
| `contract.py` | ✅ | ✅ | Public Surface — Service / Event / Stream key + payload + Capability |
| `module.py` | ❌ | ❌ | framework entry, internal |
| `drivers/protocol.py` | ❌ | ❌ | Module SDK internal (D405 / USB / Feetech impl detail) |
| `drivers/<vendor>.py` | ❌ | ❌ | internal |
| `models.py` (ORM) | ❌ | ❌ | DB internal |
| `service.py` (business logic) | ❌ | ❌ | internal |

§6.3 의 3 계층 분리 (framework / Module SDK / consumer) 와 정합 — Module SDK 의 internal (drivers / models / service) 은 두 generator 의 read 대상 X.

### 8.3 Service / Event / Stream 의 자연 분리

contract.py 의 세 어휘 — 세 protocol 의 의미. 두 generator 가 각각 적절히 매핑:

| 어휘 | 의미 | TS gen 매핑 | catalog viewer 매핑 |
|---|---|---|---|
| Service | request/response RPC | `callService<Req, Res>(key, req): Promise<Res>` | "Services" section |
| Event | state notification (one-way broadcast) | `subscribe<Event>(key): Observable<Event>` | "Events" section |
| Stream | continuous flow (high-frequency) | `subscribeStream<Frame>(key): Observable<Frame>` | "Streams" section |

backend/ 의 [naming_conventions.md](naming_conventions.md) 의 wire schema 어휘 (verb-first + sub-domain prefix) 자연 흡수.

### 8.4 Service metadata — contract introspection 의 자리

backend_v2.md §3.1 의 `@service` decorator 가 metadata 받음 — catalog viewer 위 human-readable description 박힘:

```python
@service(
    Camera.Service.CAPABILITIES,
    description="Hardware capability snapshot — UI 가 boot 시 1회 read",
    tags=["camera", "capability", "static"],
)
def get_capabilities(self, req: SnapshotRequest) -> CameraCapabilities: ...
```

**중요한 자리**:
- TS gen 은 metadata 무시 (type 추출만)
- catalog viewer 가 description / tags 위 표시
- framework 어휘 확장 — backend_v2.md §3.1 의 `@service(key)` → `@service(key, *, description=, tags=)` 자연 (anchor 의 frontend / catalog 자리)

### 8.5 Stream payload — seq / timestamp invariant

frontend 의 runtime 자리 — WebSocket reconnect / lag / out-of-order detection 의 기본 어휘. 모든 state stream payload 박힘:

```python
class TcpState(BaseModel):
    robot_id: str
    seq: int                  # monotonic sequence (per-stream)
    timestamp_unix: float     # publish 시 UTC epoch
    tcp_pose: Transform4x4
    joint_state: list[float]

class CameraJpegFrame(BaseModel):
    robot_id: str
    seq: int
    timestamp_unix: float
    jpeg_bytes: bytes
```

> **invariant**: 모든 stream payload 에 `seq: int` + `timestamp_unix: float` 박힘.

event payload 는 보통 1회 broadcast — seq 자리 X (단 미래 audit / replay 자리 박힐 수 있음, 후속).

### 8.6 Bridge ≠ contract viewer ≠ TS gen

본 § 의 자리 명확화:

| 자리 | 책임 | 박힌 자리 |
|---|---|---|
| **Bridge** | runtime traffic relay (WS / MJPEG / HTTP) — frontend ↔ Module Zenoh wire | §1.1 catalog #12 |
| **TS type gen** (frontend 연동) | build-time, `pnpm gen:types` 명령 | §8.1 |
| **Contract viewer** (개발자 introspection) | build-time, 별 command | §8.1 |

**Bridge invariant**:
> Bridge = runtime relay only. **domain Module logic** 박지 X. framework infrastructure helper relay (`/robots` = RobotConfig list / `/tasks` = MODULE_REGISTRY / `/system` = metric helper) 는 OK.

위반 예 — Bridge 안 `/api/calibrations` endpoint 박힌 자리에 calibration_results table direct read (Calibration Module 의 책임 우회) 박힘. 본 자리 = *domain Module 의 service 우회* → Bridge 책임 X. 대신 `Calibration.Service.LIST_RESULTS` 같은 service 를 Calibration Module 박고, Bridge 가 그 service relay.

허용 자리 — framework 의 자체 helper (robot_id list / module registry / system metric) 의 read-only relay. domain Module 의 영역 안 들이지 않음.

## 9. Framework infrastructure 자리 (Module X)

framework 가 자동 흡수 — Module 코드 0.

| 자리 | 흡수 자리 |
|---|---|
| Heartbeat (1Hz per-Module) | Runtime 이 자동 publish |
| System monitoring (CPU/Mem/Zenoh peers) | framework metric helper. Bridge 의 `/system` endpoint 가 relay |
| Logging | `self.runtime.log()` 가 동일 topic 박음 |

backend/ 의 BaseNode 15+ method (publish / log / heartbeat / lifecycle) 가 모두 framework 자동 주입.

### 9.1 Robot registry — application 파싱, framework 어휘 없음

robots.yaml 파싱 책임 = **application** (apps/main.py — §5.2). framework 는 robot 의 detail (driver / capability / 마운트) 모름, 그저 `robot_id` placeholder substitute 어휘만 (backend_v2.md §3.7 + §2.7).

흐름:
```
robots.yaml
    │
    ▼  application 파싱 (apps/main.py)
RobotConfig 인스턴스 list
    │
    ▼  resolve_deps → driver impl 선택 + Module dep inject
runtime.add_module(MotorDriver, robot_id=..., driver=FeetechBackend(...))
    │
    ▼  framework 는 robot_id 만 봄
ZenohTransport: srv/motor/so101_6dof_0/set_torque (placeholder substituted)
```

Bridge 의 `/robots` endpoint = application 의 RobotConfig list 의 read-only view relay (framework helper). 본 자리 = "domain Module logic 박지 X" invariant 위반 X — robot list 어휘는 application 의 결과, framework 가 relay.

## 10. 박힌 결정 — anchor 표

| # | 결정 | 위치 |
|---|---|---|
| 1 | 4 layer 자연 분리 (Hardware/Derived / Domain / Orchestration / Boundary) | §1 |
| 2 | 12 Module (합치지 / 더 잘라지 않은 motivation) | §1.1 / §1.2 |
| 3 | derived = Camera 만 (small payload derived X) | §4 |
| 4 | CameraDecoded = 한 Module 두 stream (color + depth) | §4.1 |
| 5 | Mock = driver subdir swap (Mock Module X) | §2.4 |
| 6 | Framework 어휘 ≠ application 어휘 (driver / 마운트 = robots.yaml SSOT) | §5 |
| 7 | 공통 abstraction = Module SDK (`drivers/protocol.py`), framework X | §6 |
| 8 | **Topology / Capability / Config 의 어휘 분리**. Topology = "무엇이 존재하는가" (consumer 가 구조 알아야 할 때만 — Motor ✅ / Camera ❌). Capability = "무엇을 할 수 있는가 / 어떤 결과 제공" (flags + supported max metadata). Config = "현재 설정 값". snapshot only, Mirror X. 값 SSOT = driver `topology()` / `capabilities()`. **Motor 의 GRIPPER / POSITION_PID 박지 X** (Topology derived / baseline) | §7.1 / §7.2 / §7.3 |
| 9 | **Mirror invariant — control correctness state synchronization only**. Runtime telemetry / continuously changing observation 자리 = snapshot service or stream subscribe | §3.2 |
| 10 | TcpState access = `Motion.Service.TCP_SNAPSHOT` (point-in-time) + `Motion.Stream.TCP_STATE` (continuous). Mirror X (telemetry) | §3.3 / §4 |
| 11 | **Offline computation 의 local kinematics OK**. Reconstruction 안 PyBullet 인스턴스 자연. control authority = Motion Module SSOT, Reconstruction = read-only compute. duplicate 위험 = Calibration Bundle SSOT 가 흡수 (URDF / joint / sag 한 자리) | §3.1 / §12 |
| 12 | **multi-camera per robot = 후속** (1 robot 1 camera 강제, robots.yaml `camera:` 단수). Module scope 정의 (`robot-scoped` = robot 의 physical resource owner) 유지 | §5.2 |
| 13 | **Intrinsic SSOT 분리** — CameraDriver SDK internal `get_factory_intrinsics()` (calibration seed only). Calibration Bundle 의 `intrinsic` = public calibrated SSOT. Camera Module 의 public service 에 `GET_INTRINSICS` 박지 X | §7.6 |
| 14 | **CalibrationActivated event 의 versioning** — `{robot_id, bundle_id, version}`. Mirror[CalibrationBundle] consumer 가 어떤 bundle 위 rebuild 박았는지 추적 | §3.4 |
| 15 | **contract.py = 두 generator 의 SSOT** — frontend `pnpm gen:types` (TS interface) + backend contract viewer (Swagger-like). 같은 source, 두 별 명령 / 두 별 소비자 / 런타임 무관 | §8.1 / §8.2 |
| 16 | **Service / Event / Stream = 세 protocol 의 자연 분리** — TS gen / catalog viewer 가 각각 RPC / event broadcast / continuous flow 매핑 | §8.3 |
| 17 | **Service metadata** (description / tags) — `@service` decorator 어휘 확장. catalog viewer 위 표시 (TS gen 무관) | §8.4 |
| 18 | **Stream payload invariant** — 모든 stream payload 에 `seq: int` + `timestamp_unix: float`. frontend reconnect / lag / out-of-order 차단 | §8.5 |
| 19 | **Bridge ≠ contract viewer ≠ TS gen** — Bridge = runtime relay only. domain API logic 박지 X | §8.6 |

## 11. Build order — Module 박을 순서

본 build order = **의존성 DAG (§1 / §3.1) + framework 의 진짜 검증 + 검증 가능성** 위. [backend_v2.md §15.6](backend_v2.md) 의 "Step 6 = Calibration Module" 박혀있지만, 본 module catalog 의 자리 (12 Module 의 의존성 + bottom-up) 위에서 다시 박힌 자리.

### 11.1 의존성 DAG (재정리)

```
                MotorDriver           CameraDriver      (leaf, dep 0)
                    │                      │
                    │                CameraDecoded
                    │                      │
                    ▼                      ▼
                 Motion ◄──── Mirror[Bundle] ──── Calibration
                    │                                 │
            ┌───────┼──────────┬──────────────┬──────┘
            ▼       ▼          ▼              ▼
        Gamepad  Detector   Scene3D ─── Scan ─── Reconstruction
                    │          │         │           │
                    └──────────┴─────────┴───────────┴────► Task ────► Bridge
```

### 11.2 5 step (+ 후속)

| Step | Module | 의존 | 검증 자리 | 산출물 |
|---|---|---|---|---|
| **A** | **MotorDriver** | 0 (leaf) | robot-scoped 첫 검증 + `{robot_id}` substitute + Pi deployment + driver Protocol swap (feetech/dynamixel/mock) + Capability service + 3 framework 어휘 (service + event + stream) 다 검증 | mock motor 의 raw state publish + torque/reboot service. Zenoh subscriber verify |
| **B** | **CameraDriver + CameraDecoded** | 0 (A 와 병렬 가능) | derived read model 패턴 첫 검증 + host 횡단 stream (pi → pc) + decode dedup CPU 측정 + Stream payload seq/timestamp invariant | mock JPEG publish + decode dedup 측정 |
| **C** | **Bridge** | A + B | runtime relay 첫 검증 + WebSocket + MJPEG + framework helper (`/robots` / `/system`) + `pnpm gen:types` 의 첫 실 검증 | browser 의 라이브 카메라 view + multi-Module e2e |
| **D** | **Motion** | A + Mirror[Calibration] 의 mock owner | kinematics + IK + Ruckig + jog + TCP_SNAPSHOT service + Mirror[Bundle] consumer 검증 | mock motor 의 trajectory publish (move_l / jog) |
| **E** | **Calibration** | B + D | DB-per-Module 첫 검증 + ObjectStore 첫 검증 + capture loop + Bundle atomic + Mirror[Bundle] event broadcast 의 진짜 e2e (Motion 의 kinematics rebuild 자리) | capture + Bundle commit + 진짜 Mirror[Bundle] consumer 검증 |

이후 (의존성 DAG 따라 순차):
- **F** Detector (B + E + D 위)
- **G** Scene3D (B 위)
- **H** Scan (B + D + E + Scene3D 위)
- **I** Reconstruction (Scan + E 위)
- **J** Task (A-I 위)
- **K** Gamepad (D 위, 단순)

### 11.3 왜 MotorDriver 가 첫째 (Calibration 아닌)

| 비교 | MotorDriver | Calibration |
|---|---|---|
| 의존성 | 0 (leaf) | CameraDecoded + Motion 필요 — 둘 없으면 capture loop 동작 X |
| framework 어휘 검증 | service + event + stream **3 어휘 다** | 영속성 + service + event (stream X) |
| scope 검증 | robot-scoped 첫 검증 (`{robot_id}` substitute / Pi deployment / driver Protocol swap) | robot-agnostic 의 첫 검증 (한 자리만) |
| hardware 검증 | mock 또는 실 Feetech 둘 다 자연 | dep 다 박혀야 실 검증 |
| 다음 step 의 자연 dep | Motion 의 자연 dep | dep 별 mock 박는 자리 박혀야 |

### 11.4 step 별 mock 박을 자리

| Step | mock 박는 dep | 자리 |
|---|---|---|
| A | hardware = `drivers/mock.py` (motor SDK 안 박는 driver) | `modules/motor/drivers/mock.py` |
| B | hardware = `drivers/mock.py` (합성 JPEG / 합성 depth) | `modules/camera/drivers/mock.py` |
| C | (mock dep 없음 — A + B 의 실 Module 위) | — |
| D | Mirror[CalibrationBundle] 의 **mock owner** | `tests/fixtures/mock_calibration_owner.py` (ACTIVATED event publish + SNAPSHOT_BUNDLE service) |
| E | (mock dep 없음 — B + D 의 실 Module 위. D 의 mock owner 제거) | — |

D 의 mock owner = framework 의 Mirror invariant 검증 자리. 실 Calibration 박힌 자리 (E) 에선 mock owner 제거 + 실 e2e 검증.

### 11.5 backend_v2.md §15 와의 관계

backend_v2.md §15.6 = "Step 6 = Calibration Module" 박힌 자리는 *framework spec 진입 시점* 의 자리. 본 build order 는 *Module catalog 의 의존성 + 검증 가능성* 위. backend_v2.md §15 update 자연 (후속).

## 12. 후속 자리

- 각 Module 의 contract.py 박을 자리 (Service / Event / Stream key 의 string 값 catalog) — Step A+
- 각 Module 의 DB schema / Repository method — Step E+
- ORM table name convention (`calibration_*` / `scan_*` / `task_*` 접두) — Step E+
- gamepad N>1 fail-fast framework 흡수 / Module 책임 — Step K+
- **multi-camera per robot** — robots.yaml `cameras: [list]` + Module scope 확장 (`device-scoped` = `(robot_id, camera_id)`). framework anchor 변경 필요. wrist + workspace 다중 자리 박힐 때.
- **pyproject deps role-split** — 현재 backend_v2 는 `[project].dependencies` 단일 리스트 (framework bring-up 편의). §2.3 host 배치 + 옛 backend 선례대로 real driver + heavy 모듈 port (Step 9) 시 PEP 735 group 으로 분리 (`pi-motor`: dynamixel/feetech/ruckig/pybullet/scipy, `pi-camera`: pyrealsense2/opencv/zstandard, `pc`: fastapi/uvicorn/open3d/ultralytics/...). pyrealsense2 소스빌드 / open3d 무게가 split 을 load-bearing 하게 만드는 시점.
- **Effective capability = Hardware + Runtime** (미래) — driver self-declare 위에 runtime condition (pipeline disabled / CPU 부족 / driver crashed) AND. 첫 박을 때 hardware capability 만
- High-level capability composition (pick_and_place = camera.depth ∧ motion.cartesian ∧ gripper.parallel) — UI 또는 별 Module 책임
- **`pnpm gen:types` 구현 detail** — backend/ 의 [bridge/zenoh_bridge.py](../backend/bridge/zenoh_bridge.py) 의 `custom_openapi() / x-contract` 자리 reference. backend_v2 의 contract.py 위 별 generator 명령 — build-time, 런타임 0
- **Backend contract viewer 구현 detail** — `python -m horibot.contract_viewer` 같은 별 명령. `MODULE_REGISTRY` + `contract.py` introspection → Swagger-like viewer
- **Event payload 의 audit / replay seq** — 본 design 박지 X (event = 1 회 broadcast 강제). 미래 audit replay 필요 시 박힐 자리
- **`@service` metadata 의 permission / state** — RBAC / state-machine 어휘 (현 design 없음, 미래)
- **backend_v2.md §15 update** — "Step 6 = Calibration" 자리 → 본 build order (Step A = MotorDriver) 정합으로 update
- **§11 DAG 어휘** — "의존성 DAG" → "Module relationship graph" 더 정확 (나중에 자연 수정)
- **Step C Bridge 의 의미** — "Bridge Module 개발" 아닌 *contract → TS gen → runtime relay → frontend smoke test* 의 검증 단계. Bridge Module 자체 박혀있고, Step C 는 framework 의 외부 노출 path 의 첫 e2e
- **Step D Motion 의 mock Calibration** — 실 Module X. `tests/fixtures/mock_calibration_owner.py` 같은 fixture / mock contract owner. Mirror invariant 의 검증 자리
