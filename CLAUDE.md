# CLAUDE.md

이 파일은 Claude Code(claude.ai/code)가 이 저장소에서 작업할 때 참고하는 가이드입니다.

## 프로젝트 개요

OMX Control — **OMX_F**(OpenMANIPULATOR-X 커스텀 변형) 6DOF 로봇팔 제어 스택. 백엔드는 Dynamixel 서보를 구동하고, 카메라 + YOLO 디텍션 + Hand-Eye 캘리브레이션을 실행하며, Ruckig으로 trajectory를 계획하고, PyBullet으로 [robot/urdf/omx_f/](robot/urdf/omx_f/) URDF에 대한 FK/IK를 푼다. 프론트엔드는 teleop / 캘리브레이션 / 3D 디지털 트윈 워크스페이스를 제공하는 React 앱.

D405 RGBD로 전환 진행 중 — RGB 스트림은 끝났고, depth + aligned color + intrinsics가 한 메시지로 묶여 LAN에 흐르며, PC가 구독해 Open3D 포인트클라우드로 발행한다 (자세히는 마지막 섹션).

## 자주 쓰는 명령어

### Backend (Python 3.11, uv 관리, [backend/](backend/)에서 실행)

```powershell
cd backend
uv sync                  # PC 개발 환경 (default-groups = dev + pc)
uv run python main.py    # hostname 자동 감지 → 매칭 실패 시 host_dev.yaml
uv run ruff check .      # 린트
uv run pyright           # 타입 체크
```

실행 모드:

```powershell
# 단일 머신 풀스택 (PC 한 대에 다 연결, 개발/회귀)
uv sync
uv run python main.py                    # --host 미지정 → host_dev.yaml fallback
# 또는 명시적으로
uv run python main.py --host dev

# 분산: PC 역할 (heavy 연산 + 브릿지)
uv run python main.py --host pc

# 분산: 모터 Pi (192.168.0.101)
uv sync --only-group pi-motor
uv run python main.py --host pi_motor

# 분산: 카메라 Pi (192.168.0.102)
# pyrealsense2는 사전 소스 빌드 후 별도 install
uv sync --only-group pi-camera --no-install-package pyrealsense2
uv run python main.py --host pi_camera
```

호스트 config 파일들 ([backend/config/](backend/config/)):

- `host_dev.yaml` — 단일 머신 풀스택 (motor/camera/motion/calibration/task/detector/pointcloud + 브릿지; gamepad는 미포함)
- `host_pc.yaml` — 분산 PC (calibration/task/detector/pointcloud + 브릿지; motor/motion/camera 없음)
- `host_pi_motor.yaml` — 분산 모터 Pi (motor/motion)
- `host_pi_camera.yaml` — 분산 카메라 Pi (camera)

기본 모터 포트는 Windows `COM6` / Linux `/dev/ttyACM0` ([robot/config/motors.yaml](robot/config/motors.yaml)). 모터 인터페이스는 U2D2가 아닌 **OpenRB-150** — U2D2 호환 스케치가 올라가 있어 동작은 같지만 Linux에선 FTDI(`ttyUSB*`)가 아니라 CDC-ACM(`ttyACM*`)로 잡힌다.

### Frontend (pnpm, [frontend/](frontend/)에서 실행)

```powershell
cd frontend
pnpm install
pnpm dev        # vite 개발 서버 :5173 (브릿지에서 CORS 허용)
pnpm build      # tsc -b && vite build
pnpm lint       # eslint
```

브릿지 URL은 `VITE_WS_URL` / `VITE_BASE_URL`에서 읽고, 기본값은 `ws://localhost:8000/ws`와 `http://localhost:8000` ([src/constants/index.ts](frontend/src/constants/index.ts)).

## 아키텍처

### 분산 토폴로지 — PC + 모터 Pi + 카메라 Pi

세 머신이 같은 코드베이스를 공유하고, 각자의 host config로 어떤 노드를 띄울지만 다름. **단일 머신 모드와 분산 모드의 코드 경로가 동일** — Zenoh가 토픽/서비스 라우팅을 투명하게 처리하므로 노드는 자기가 어디서 도는지 알 필요 없음.

| 머신            | 노드                                                       | 책임                                       |
| --------------- | ---------------------------------------------------------- | ------------------------------------------ |
| PC              | detector, task, pointcloud, calibration, bridge, (gamepad) | YOLO, 포인트클라우드 가공, 브릿지, 프론트  |
| 모터 Pi (101)   | motor, motion                                              | Dynamixel + Ruckig + IK (제어 루프 로컬화) |
| 카메라 Pi (102) | camera                                                     | D405 캡처 + JPEG + 압축 depth 발행         |

이 분배의 핵심 이유:

- (a) USB 대역폭 경합 해소 — D405와 OpenRB-150이 한 USB 컨트롤러를 공유하지 않음
- (b) 100Hz 제어 명령(`MOTOR_CMD_JOINT`)을 네트워크로 안 보냄 — TrajectoryRunner와 MotorNode 같은 머신
- (c) 무거운 연산(YOLO, Open3D, PyBullet PC 측)은 PC

### 2-layer 트랜스포트: Zenoh (백엔드 내부) + WebSocket (브라우저)

백엔드 노드는 모두 **Zenoh** pub/sub + queryable로 통신. 프로세스당 하나의 `ZenohSession` 싱글톤 ([backend/core/zenoh_session.py](backend/core/zenoh_session.py))을 모든 노드가 `ZenohSession.get()`으로 공유. 토픽 페이로드는 보통 JSON, 카메라 JPEG / depth_frame / 포인트클라우드는 raw 바이너리. 서비스는 `{success, message, data}` 응답 봉투.

분산 시 각 머신이 독립 peer로 동작 — 같은 LAN이면 멀티캐스트 scout으로 자동 발견. 명시적 endpoint가 필요하면 host config의 `zenoh.connect`에 적음. `ZenohSession.init(cfg_dict)`이 host config의 `zenoh` 섹션(mode/connect/listen)을 받아 `zenoh.Config`로 변환.

브라우저는 Zenoh를 못 하니까 [backend/bridge/zenoh_bridge.py](backend/bridge/zenoh_bridge.py)의 FastAPI가:

- `/ws` — WebSocket: `subscribe` / `unsubscribe` / `publish` / `service` 메시지 ↔ Zenoh 양방향 변환.
- `/camera/stream` — MJPEG `multipart/x-mixed-replace` 스트림, `omx/camera/stream/raw` Zenoh 토픽 소스.
- [robot/](robot/)를 `/robot`에 정적 마운트 (URDF, mesh, calibration .npz).
- "코어" 토픽을 미리 구독해 WS 클라이언트들에 재방송 (`_ALWAYS_SUBSCRIBE` — motor state, camera status, system heartbeat/log, motion trajectory, task state, detector state, pointcloud state).

JSON 토픽은 `{type:"topic_data", topic, data}`로 텍스트 송신. **바이너리 토픽**(카메라 raw 제외 — 그건 MJPEG로 별도 라우트)은 다음 프레이밍으로 binary WS 송신:

```
[u8 version=1][u8 type=1 (topic_data)][u16 BE topic_len][UTF-8 topic][payload]
```

현재 이 바이너리 프레이밍을 쓰는 토픽은 `omx/pointcloud/stream`. 프론트에서 동일 디코더로 풀어 React-Three-Fiber에 먹임.

#### 클라이언트별 송신 큐 + 백프레셔 ([backend/bridge/client_stream.py](backend/bridge/client_stream.py))

각 (WS 클라이언트, 구독 토픽) 짝마다 별도 `ClientStream` (bounded `asyncio.Queue` + sender task) — 느린 브라우저 한 명 때문에 메모리가 폭증하지 않게. Zenoh 콜백 스레드는 `call_soon_threadsafe(manager.fanout, topic, payload)`만 하고 실제 송신은 이벤트 루프의 sender task가 담당.

토픽별 정책 ([client_stream.py](backend/bridge/client_stream.py)):

- 기본: `LATEST_WINS` — 큐 크기 1, 새 값이 옛 값을 덮어씀 (실시간 스트림 기본 동작).
- `SYSTEM_LOG`: `BOUNDED_FIFO`(128) — 로그는 누락되면 디버깅이 어려우므로 일정량 보존.

새 토픽이 들어와도 자동 안 잡힘 → 정책 바꾸려면 `_TOPIC_POLICIES`에 추가.

프론트엔드는 `BridgeClient` 싱글톤 ([frontend/src/api/bridge.ts](frontend/src/api/bridge.ts))으로 감싸고 `ReconnectingWebSocket`, 토픽별 멀티플렉싱, `request_id` 기반 service promise를 처리. `useBridge` ([frontend/src/hooks/useBridge.ts](frontend/src/hooks/useBridge.ts))가 `App.tsx`에서 한 번 마운트되어 들어오는 데이터를 Zustand store로 라우팅.

### 토픽/서비스 레지스트리 — 두 곳에서 동기화

토픽/서비스 키는 **두 군데**에 선언됨: [backend/core/topic_map.py](backend/core/topic_map.py)의 `Topic`/`Service` 클래스와 [frontend/src/constants/topics.ts](frontend/src/constants/topics.ts)의 `Topic`/`ServiceKey`. 추가/변경 시 두 파일 같이 수정, 문자열 정확히 일치 (예: `omx/motion/srv/move_l`). 프론트에서 사용하지 않는 내부 토픽/서비스(예: `CAMERA_DEPTH_FRAME`, `POINTCLOUD_SNAPSHOT`, `CAMERA_SET_DEPTH_STREAM`)는 프론트 쪽 미러링 생략 가능.

### 노드 패턴 + 노드 레지스트리 (lazy import)

모든 노드는 `BaseNode` ([backend/core/base_node.py](backend/core/base_node.py))를 상속:

- `create_subscriber(topic, callback)` — JSON 디코드된 pub/sub
- `create_raw_subscriber(topic, callback)` — JSON 디코드 없는 binary 페이로드 (JPEG, depth_frame, 포인트클라우드)
- `create_service(key, handler)` — Zenoh queryable 등록, 핸들러는 `{success, message, data}` 반환
- `call_service(key, data, timeout=5.0)` — 기본 5초 타임아웃 동기 호출; 응답 없으면 `{success: False, ...}` 반환
- `publish(topic, data)`, `log(level, msg)`, 1Hz heartbeat `omx/system/heartbeat`

라이프사이클: `start()` → heartbeat 스레드 + 노드별 워커 스레드 → `stop()`이 모든 subscriber/queryable undeclare. 노드별 hw 루프는 자기 `start()` override에서 등록 (예: `MotorNode._state_loop` 20Hz, `CameraNode._stream_loop` 30 FPS color + `_depth_loop` 8 FPS depth, `DetectorNode._detection_loop` 5 FPS, `PointCloudNode._stream_loop` 8 FPS).

[backend/core/node_registry.py](backend/core/node_registry.py)가 노드 이름 → `(모듈, 클래스)` 매핑을 **문자열로** 유지. `importlib`로 호출 시점에만 import → 모터 Pi가 `node_registry`를 import해도 open3d/pyrealsense2/ultralytics 등 PC 전용 의존성이 import 트리에 안 끌려옴. 등록된 노드: `motor / camera / motion / calibration / task / detector / pointcloud / gamepad`. 새 노드 추가하려면 `_NODE_REGISTRY`에 한 줄 + 해당 의존성을 `pyproject.toml`의 적절한 그룹에 추가.

### 호스트 config (config-driven main.py)

[backend/main.py](backend/main.py)는 `--host` 인자(미지정 시 hostname을 lowercase/`-`→`_` 정규화 후 `host_<hostname>.yaml` 매칭 → 매칭 실패 시 `host_dev.yaml` fallback)로 [backend/config/](backend/config/)의 YAML을 로드:

```yaml
host_name: pi_motor
zenoh:
  mode: peer
  connect: [] # 비우면 멀티캐스트 scout
  # listen:            # 방화벽 룰 고정용 (선택)
  #   - "tcp/0.0.0.0:7447"
nodes: [motor, motion]
bridge:
  enabled: false # 브릿지는 PC에서만
```

main.py는 `nodes`에 명시된 것만 lazy-import + 시작. `bridge.enabled`로 FastAPI/uvicorn 켜기. `camera` 노드가 활성일 때만 D405 factory intrinsic seed가 실행됨 (`seed_d405_intrinsic_if_missing`).

### 의존성 그룹 (`pyproject.toml`)

PEP 735 `[dependency-groups]`로 역할별 분리:

- 공통(`[project]`): `eclipse-zenoh`, `numpy`, `pyyaml`, `python-dotenv`
- `dev`: `ruff`, `pyright`
- `pi-motor`: `dynamixel-sdk`, `pybullet`, `ruckig`, `scipy`
- `pi-camera`: `pyrealsense2`, `opencv-python`, `zstandard`
- `pc`: `opencv-python`, fastapi 스택(`fastapi`/`uvicorn`/`websockets`), `pybullet`, `scipy`, `ultralytics`, `open3d`, `pygame`, `zstandard` (pyrealsense2 없음 — 카메라 Pi가 다 처리)
- `all`: 위 세 역할 그룹 include (개발용 한 머신 풀스택)

`tool.uv.default-groups = ["dev", "pc"]` — PC 개발 환경에선 `uv sync`만으로 충분, Pi에서는 `--only-group <role>` 명시.

Pi 빌드 노트: `pyrealsense2`는 aarch64 PyPI wheel이 없어 카메라 Pi에서 소스 빌드 필요 (mathklk/realsense_raspberry_pi4 레시피). 빌드한 wheel은 `uv pip install ./pyrealsense2-*.whl`로 별도 설치 + `--no-install-package pyrealsense2`로 그룹 sync.

### 노드 간 공유 싱글톤들

- `ZenohSession` — 프로세스당 하나의 Zenoh 세션.
- `JointStateCache` ([backend/core/joint_state_cache.py](backend/core/joint_state_cache.py)) — `MOTOR_STATE_JOINT`를 한 번만 구독, `get_joint_angles_rad(arm_cfgs)`로 라디안 단위 최신 조인트각 노출. motion/task/detector/calibration이 `_cache.subscribe(self)`로 공유.
- `FrameCache` ([backend/core/frame_cache.py](backend/core/frame_cache.py)) — `CAMERA_STREAM_RAW`(JPEG) + `CAMERA_STATE_STATUS` 구독, `get_frame()`이 BGR ndarray 반환. **detector/calibration이 `CameraCapture`를 직접 안 들고 토픽 기반으로 동작** → 카메라가 다른 머신에 있어도 동일 코드. (`JointStateCache`와 동일 패턴)
- `RealsenseCapture` ([backend/core/realsense_capture.py](backend/core/realsense_capture.py)) — pyrealsense2 파이프라인 1개를 공유. **카메라 호스트에서만 살아 있음** (PC 분산 모드에선 instantiate 안 됨). `CameraCapture`([backend/modules/camera/capture.py](backend/modules/camera/capture.py))는 이걸 감싸는 얇은 래퍼로 CameraNode 전용.
- `PybulletSolver` ([backend/modules/kinematics/solver.py](backend/modules/kinematics/solver.py)) — DIRECT 모드 PyBullet으로 URDF 로드, thread-safe `fk()` / `ik()` / `fk_to_matrix()`. EE 링크 이름 `end_effector_link`. 모터 Pi 또는 PC에 위치 (MotionNode와 같은 머신, CalibrationNode는 PC 측 자체 인스턴스).

### Motion 파이프라인

`MotionNode` ([backend/nodes/motion_node.py](backend/nodes/motion_node.py))가 `move_j` / `move_l` / `move_c` / `move_p` / `move_tcp` 서비스 수신. 검증/실행은 `MotionCommand` 서브클래스 ([backend/modules/kinematics/motion_commands.py](backend/modules/kinematics/motion_commands.py))로 분리. 실제 보간은 `TrajectoryRunner` ([backend/modules/kinematics/trajectory_runner.py](backend/modules/kinematics/trajectory_runner.py))가 **Ruckig** jerk-limited 프로파일로 처리하고 `omx/motion/state/trajectory`에 진행 발행. 조인트 명령은 `publish_cmd` 콜백 → `MOTOR_CMD_JOINT` 토픽.

**MotionNode와 MotorNode는 같은 머신(모터 Pi)에 배치** — TrajectoryRunner가 100Hz로 publish하는 명령이 네트워크를 넘지 않게 해야 trajectory 끊김/지터를 막을 수 있음. PyBullet IK도 같은 머신.

아암은 운동학적으로 **5DOF** (모터 ID 1–5), ID 6은 그리퍼로 `core.common.GRIPPER_ID`로 필터링. 단위 변환은 [backend/core/units.py](backend/core/units.py) — Dynamixel raw는 `0..4095`, 중심 `2048`(=0°). [robot/config/motors.yaml](robot/config/motors.yaml)의 각 모터에 `reverse` 플래그와 `limit.min/max` raw 클램프 (rad_to_raw이 강제).

### Task 시스템

Task는 선언형 step 리스트 ([backend/modules/task/step_types.py](backend/modules/task/step_types.py): `MoveTCPStep`, `DetectStep`, `GripperStep`, `HomeStep`, `WaitStep`). `TASK_REGISTRY`는 [backend/nodes/task_node.py](backend/nodes/task_node.py)에. `StepExecutor`가 각 step 실행(motion/detector/motor 서비스 호출), `TaskRunner`가 상태 머신(`run/pause/resume/stop`), 진행은 `omx/task/state`로 발행. `DetectStep`은 결과를 context dict(`output_key`)에 쓰고, 이후 `MoveTCPStep`이 `position_key` + `offset`으로 소비 — [backend/modules/task/tasks/pick_and_place.py](backend/modules/task/tasks/pick_and_place.py)가 정규 예시.

TaskNode/StepExecutor는 더 이상 카메라 객체를 직접 들지 않음 (분산 호환을 위해 detector에 위임).

### Detector → 월드 좌표 파이프라인

`DetectorNode._handle_detect` ([backend/nodes/detector_node.py](backend/nodes/detector_node.py)) 체인: `FrameCache.get_frame()` → YOLO centroid → `cv2.undistortPoints`로 intrinsic 보정 → `MOTION_GET_TCP`로 현재 EE 포즈 → base-frame 평면 `Z=0` 제약으로 `Z_cam` 역산 → 베이스 프레임의 물체 위치 반환. [robot/calibration/intrinsic.npz](robot/calibration/intrinsic.npz)와 [robot/calibration/hand_eye.npz](robot/calibration/hand_eye.npz) **둘 다** 필요 — `load_calibration().is_ready()`가 서비스 가드. 별도 5fps `_detection_loop`가 `DETECTOR_STATE`에 raw detection을 stream으로 발행.

### D405 카메라 → 깊이 → 포인트클라우드 파이프라인

```
CameraNode (카메라 Pi)
  ├─ 30 FPS color JPEG → omx/camera/stream/raw           (항상 켜짐)
  └─ 8 FPS depth_frame → omx/camera/stream/depth_frame   (CAMERA_SET_DEPTH_STREAM enable 시)
       페이로드 (modules/camera/depth_frame.py, little-endian):
         [u32 header_len][JSON header][u32 jpeg_len][aligned color JPEG][zstd Z16 depth]
       header: timestamp / width / height / depth_scale / fx fy cx cy
               / depth_uncompressed_bytes (검증용)

PointCloudNode (PC)
  ← CAMERA_DEPTH_FRAME 구독 (raw subscriber), depth_frame.decode()로 복원
  → Open3D RGBDImage → PointCloud → voxel_down_sample
  → omx/pointcloud/stream (raw binary: [u32 n][n*3 float32 xyz][n*3 uint8 rgb])

프론트엔드 (브릿지가 binary WS 프레이밍으로 중계)
  → POINTCLOUD_CONFIGURE {enabled?, voxel_size?}
       ├─ voxel_size: PointCloudNode 로컬 상태로만 저장
       └─ enabled:    PointCloudNode가 CAMERA_SET_DEPTH_STREAM 호출 → 카메라 Pi
                      (또한 비활성 시 _latest_frame 캐시 비움 — 오래된 프레임으로
                       cloud 만드는 사고 방지)
  ← POINTCLOUD_STATE {enabled, voxel_size} — configure마다 브로드캐스트
```

설계 포인트:

- **Depth는 무손실(zstd) — ICP/TSDF 정밀도 보존.** Color만 JPEG 손실. 무손실 + zstd 5–10x 압축으로 8fps에서도 LAN 트래픽 24–32Mbps 정도.
- 단일 메시지로 묶어 color/depth/intrinsics 시간 정합성 보장 (지터 무).
- `voxel_size`는 PC 가공 파라미터, `enabled`는 카메라 측 device 게이트 — 한 service(`POINTCLOUD_CONFIGURE`)가 둘을 자기/forward로 나눠 처리해서 프론트엔드는 단일 진입점.
- PointCloudNode는 마지막 처리한 frame timestamp를 기억해 같은 프레임을 중복 처리하지 않음.
- 미래 ICP / TSDF / PLY 스냅샷 — 같은 `depth_frame` 페이로드를 디스크에 저장하면 끝. 별도 토픽 신설 불필요 (`POINTCLOUD_SNAPSHOT` 자리는 예약만 되어 있음).

### Frontend stores & 3D 워크스페이스

상태는 [frontend/src/store/](frontend/src/store/)의 Zustand store로 분리 (`robotStore`, `cameraStore`, `motionStore`, `taskStore`, `detectorStore`, `systemStore`, `sceneStore`). `Workspace3D` 페이지 ([frontend/src/pages/Workspace3D.tsx](frontend/src/pages/Workspace3D.tsx))는 `dockview` 플로팅 패널 위에 `react-three-fiber` 씬 + `urdf-loader` — 패널은 [frontend/src/components/workspace3d/dockview/panelComponents.ts](frontend/src/components/workspace3d/dockview/panelComponents.ts)에 등록. URDF에 들어가는 조인트각은 `MOTOR_STATE_JOINT`에서 `(position - 2048) / 4095 * 2π` 형태로 도출 (`units.raw_to_rad`와 일치).

## 규약

- 로그 메시지, 주석, docstring은 한국어 자유롭게 — 주변 코드의 스타일을 유지.
- Backend는 **ruff**(line-length 88, target py311) + **pyright**, Frontend는 ESLint + Prettier + `editor.formatOnSave` (VS Code).
- 프론트엔드 import는 `@/`alias = [frontend/src/](frontend/src/) ([frontend/vite.config.ts](frontend/vite.config.ts)).
- 서비스 핸들러는 반드시 `{"success": bool, "message": str, "data": dict}` 반환 — 브릿지와 `BridgeClient.callService`가 이 모양에 의존.
- 모델 가중치(`*.pt`, `*.pth`), `.venv/`, `node_modules/`, `frontend/dist/`, `uv.lock`은 gitignore.

## D405 마이그레이션 진행상황

OpenCV USB 카메라 → **Intel RealSense D405** 전환. 단순 pick-and-place 센서가 아니라 범용 RGBD 디바이스로 취급 — 라이브 포인트클라우드, PLY 스냅샷, ICP 등록, detector의 Z=0 가정을 depth 직접 lookup으로 대체까지 단계적 진행.

### Phase 진행 상태

1. ✅ **RGB-only swap** — `CameraCapture` 내부를 pyrealsense2로 교체. `pyrealsense2` 의존성 추가.
2. ✅ **Binary WebSocket 페이로드 프로토콜** — 브릿지 + `BridgeClient`가 binary WS 프레임 지원 (`[u8 ver][u8 type][u16 topic_len][topic][payload]`).
3. ✅ **Live 포인트클라우드 프리뷰** — `PointCloudNode` 도입. 단, 아래 "분산 배치 작업"으로 RealsenseCapture 직접 사용은 제거하고 `CAMERA_DEPTH_FRAME` 토픽 구독 기반으로 전환됨.
4. ⏳ **PLY 스냅샷 캡처/로드** — 미구현. `depth_frame` 페이로드 그대로 저장 + Open3D PLY 변환으로 구현 예정 (`POINTCLOUD_SNAPSHOT` 토픽 자리만 예약).
5. ⏳ **ICP 등록** — 미구현. PointCloudNode 서비스로.
6. ⏳ **Detector depth lookup 전환** — 미구현. 평면-Z=0 역산 자리에 depth 직접 lookup.

캘리브레이션은 D405 마운트가 바뀌었으니 **hand-eye 재캘리브 필요** (`hand_eye.npz` 무효).

### 분산 배치 작업 (PC + Pi×2)

원래 후속 단계로 분리하려던 작업을 USB 경합 문제 때문에 앞당겨 진행. 코드 변경 요약:

- **의존성 그룹 분리** ([backend/pyproject.toml](backend/pyproject.toml)) — `pi-motor` / `pi-camera` / `pc` / `all` PEP 735 그룹 + `default-groups=["dev","pc"]`.
- **노드 레지스트리** ([backend/core/node_registry.py](backend/core/node_registry.py)) — 문자열 매핑 → `importlib` lazy import.
- **호스트 config** ([backend/config/](backend/config/)) — `host_dev.yaml` / `host_pc.yaml` / `host_pi_motor.yaml` / `host_pi_camera.yaml`. main.py가 `--host` 또는 hostname으로 선택, 실패 시 `host_dev.yaml` fallback.
- **`ZenohSession.init(cfg_dict)`** — host config의 `zenoh` 섹션을 zenoh.Config로 변환 (mode/connect/listen).
- **FrameCache** ([backend/core/frame_cache.py](backend/core/frame_cache.py)) — detector/calibration이 토픽 기반으로 카메라 프레임을 받게 → 분산 호환. TaskNode/StepExecutor의 카메라 인자는 미사용이라 제거.
- **CAMERA_DEPTH_FRAME 파이프라인** ([backend/modules/camera/depth_frame.py](backend/modules/camera/depth_frame.py), [backend/nodes/camera_node.py](backend/nodes/camera_node.py), [backend/nodes/pointcloud_node.py](backend/nodes/pointcloud_node.py)) — 카메라 Pi에서 무손실 압축 depth + JPEG color + intrinsics 한 메시지로 발행, PC PointCloudNode가 구독해 cloud 생성. `POINTCLOUD_CONFIGURE`가 카메라 측 enable을 forward.
- **브릿지 백프레셔 + 바이너리 프레이밍** ([backend/bridge/zenoh_bridge.py](backend/bridge/zenoh_bridge.py), [backend/bridge/client_stream.py](backend/bridge/client_stream.py)) — 클라이언트별/토픽별 bounded queue, LATEST_WINS 기본 / SYSTEM_LOG는 BOUNDED_FIFO. 바이너리 토픽은 `[u8 ver][u8 type][u16 topic_len][topic][payload]`로 인코딩해 binary WS로 송신.

이 변경은 **단일 머신(PC만)과 분산(PC+Pi×2) 양쪽 모두에서 동일하게 동작** — 같은 토픽이 Zenoh에 흐를 뿐.

### 미해결 / 다음 단계

- HW 회귀 테스트 (현재 코드 변경만, 미검증)
- 분산 시 Zenoh 멀티캐스트 scout 검증 — 안 되면 host config의 `zenoh.connect`에 endpoint 명시
- `pyrealsense2` 카메라 Pi 빌드 wheel 절차 정형화
- PLY 스냅샷 (phase 4) — `depth_frame` 캐시 → 디스크 (PLY + npz with TCP pose)
- ICP, depth lookup (phase 5/6)
- 단일 머신 회귀 후 점진적 분산 시도 (모터 Pi → 카메라 Pi 순서 권장)

### 운영 메모

- 카메라 Pi / 모터 Pi: Ubuntu 22.04, uv 설치 완료
- IP: 192.168.0.101 (모터), 192.168.0.102 (카메라)
- 일반 의존성은 Pi 둘 다 `uv sync --only-group <role>` 가능
- `pyrealsense2`와 `open3d`는 aarch64 wheel 이슈 있음 — `pyrealsense2`는 카메라 Pi 소스 빌드, `open3d`는 PC만 필요하니 무관

