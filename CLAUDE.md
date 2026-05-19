# CLAUDE.md

이 파일은 Claude Code(claude.ai/code)가 이 저장소에서 작업할 때 참고하는 가이드입니다.

## 프로젝트 개요

OMX Control — **OMX_F**(OpenMANIPULATOR-X 커스텀 변형) 6DOF 로봇팔 제어 스택. 백엔드는 Dynamixel 서보를 구동하고, 카메라 + YOLO 디텍션 + Hand-Eye 캘리브레이션을 실행하며, Ruckig으로 trajectory를 계획하고, PyBullet으로 [robot/urdf/omx_f/](robot/urdf/omx_f/) URDF에 대한 FK/IK를 푼다. 프론트엔드는 teleop / 캘리브레이션 / 3D 디지털 트윈 워크스페이스를 제공하는 React 앱.

D405 RGBD로 전환 진행 중 — color/depth/intrinsics가 한 메시지로 묶여 LAN에 흐르고, PC가 구독해 Open3D 포인트클라우드로 발행한다 (자세히는 아키텍처 § D405 파이프라인).

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

## 하드웨어 구성

### 모터 컨트롤러

- **OpenRB-150** (SAMD21 기반, Arduino MKR Zero 호환). U2D2 정품(FTDI)이 아님.
- ROBOTIS의 "USB to Dynamixel" 예제 스케치를 올려 **U2D2 호환 모드**로 사용 (소프트 USB↔TTL 패킷 릴레이).
- USB **CDC-ACM** 클래스 → Pi에서 `/dev/ttyACM0`로 enumerate.

### 모터 (5DOF arm + 그리퍼)

| Joint             | Model      | 정격 전압 (operating) | 비고                     |
| ----------------- | ---------- | --------------------- | ------------------------ |
| 1 (base rotation) | XL430-W250 | 10.0~14.8V (12V 권장) |                          |
| 2 (shoulder)      | XL430-W250 | "                     | 중력 부하 큼             |
| 3 (elbow)         | XL430-W250 | "                     | 중력 부하 큼             |
| 4 (wrist pitch)   | XL330-M288 | 3.7~6.0V (5V 권장)    |                          |
| 5 (wrist roll)    | XL330-M288 | "                     |                          |
| 6 (gripper)       | XL330-M288 | "                     | `core.common.GRIPPER_ID` |

운동학상으로는 **5DOF** (ID 1~5), ID 6은 그리퍼라 IK/FK 대상 아님.

### 전원 토폴로지

```
[메인 PSU 11V] ──── OpenRB-150 (배럴잭, 또한 통신/스케치)
                      │
                      ├─ 직접 분기 ──── XL430 체인 (joint 1, 2, 3) @ 11V
                      │                    (정격 10~14.8V — 하한 가까움, 마진 작음)
                      │
                      └─ XL4015 DC-DC 강압 모듈 (CV/CC 5A) ──── XL330 체인 (joint 4, 5, 6) @ 5V
                                                                  (정격 3.7~6V — 정중앙)
```

- 데이지 체인은 두 그룹으로 분리되어 있으나 **TTL 데이터 라인은 같은 버스를 공유** (Dynamixel half-duplex). 즉 ID 1~6이 모두 한 패킷 버스 위에 있음.
- Wizard에서 확인된 실측: XL430 그룹 10~11V, XL330 그룹 ~5V — 둘 다 정격 안.
- XL430 그룹이 정격 하한 근처라 토크 마진이 작을 수 있음 (필요 시 12V로 올리는 것 검토).

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

### D405 → 포인트클라우드 파이프라인

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

라이브 포인트클라우드는 백엔드에서 camera-frame xyz를 그대로 publish하고, 프론트 [PointCloudLayer.tsx](frontend/src/components/workspace3d/3d/PointCloudLayer.tsx)가 `<group position quaternion>` 부모 transform으로 `cameraMatrix = tcpMatrix · handEyeMatrix`를 적용 — three.js가 GPU에서 vertex별로 `parent.matrixWorld · local`을 곱하므로 base 프레임 렌더링과 수학적으로 동일.

### Frontend stores & 3D 워크스페이스

상태는 [frontend/src/store/](frontend/src/store/)의 Zustand store로 분리 (`robotStore`, `cameraStore`, `motionStore`, `taskStore`, `detectorStore`, `systemStore`, `sceneStore`). `Workspace3D` 페이지 ([frontend/src/pages/Workspace3D.tsx](frontend/src/pages/Workspace3D.tsx))는 `dockview` 플로팅 패널 위에 `react-three-fiber` 씬 + `urdf-loader` — 패널은 [frontend/src/components/workspace3d/dockview/panelComponents.ts](frontend/src/components/workspace3d/dockview/panelComponents.ts)에 등록. URDF에 들어가는 조인트각은 `MOTOR_STATE_JOINT`에서 `(position - 2048) / 4095 * 2π` 형태로 도출 (`units.raw_to_rad`와 일치).

## 규약

- 로그 메시지, 주석, docstring은 한국어 자유롭게 — 주변 코드의 스타일을 유지.
- Backend는 **ruff**(line-length 88, target py311) + **pyright**, Frontend는 ESLint + Prettier + `editor.formatOnSave` (VS Code).
- 프론트엔드 import는 `@/`alias = [frontend/src/](frontend/src/) ([frontend/vite.config.ts](frontend/vite.config.ts)).
- 서비스 핸들러는 반드시 `{"success": bool, "message": str, "data": dict}` 반환 — 브릿지와 `BridgeClient.callService`가 이 모양에 의존.
- 모델 가중치(`*.pt`, `*.pth`), `.venv/`, `node_modules/`, `frontend/dist/`, `uv.lock`은 gitignore.

## 캘리브레이션

### 현재 결과

[robot/calibration/intrinsic.npz](robot/calibration/intrinsic.npz) — D405 color 1280x720, **factory seed 기반** (`seed_d405_intrinsic_if_missing`이 카메라 노드 기동 시 채움)

- camera_matrix: fx=649.75, fy=648.10, cx=632.67, cy=359.60
- dist_coeffs: [-0.0525, 0.0596, -0.000246, 0.000545, -0.0198]
- rms_error=0.0 — 재캘리브 잔차가 아니라 factory seed라서 0. D405의 color stream 공장 캘리브는 일반적으로 정확하므로 별도 재캘리브는 보류.

[robot/calibration/hand_eye.npz](robot/calibration/hand_eye.npz) — D405 마운트 기준 재캘리브, method=TSAI (2026-05-14, 25자세)

- R_cam2gripper:
  ```
  [[-0.0243, -0.4855,  0.8739],
   [-0.9993,  0.0373, -0.0070],
   [-0.0292, -0.8735, -0.4860]]
  ```
- t_cam2gripper = (-46.7mm, 7.4mm, 45.3mm)
- method 비교: PARK Δrot=1.461°, DANIILIDIS Δrot=0.883° → self-consistency 경계선.
- 시각 검증: 라이브 클라우드 바닥이 Z=0 그리드 대비 살짝 사선 + ~2cm 들림 (1.5° 회전 오차와 일치). 이전 캘 대비 명확히 개선됨.
- TSDF 적용 시 캘 정확도가 결과 품질의 가장 큰 결정 요소 — 1° 미만으로 끌어내려야 함. 현 다포즈 + cv2.calibrateHandEye 방식은 자세 더 추가해도 정체될 가능성 큼 (아래 § 정확도 향상 로드맵).

### Hand-Eye 재캘 절차

프론트 캘리브레이션 페이지의 Hand-Eye 탭에서 진행. 좌측 카메라 피드 위에 라이브 체커보드 코너 오버레이가 자동 표시되어 자세 평가가 실시간으로 됨.

1. (필요 시) **Capture 카드 [리셋]** — 누적 포즈 비움 (백엔드 재시작 불필요).
2. 자세 잡기 (Move TCP / 토크 OFF 후 수동). 라이브 오버레이가 초록색이면 검출 OK.
3. **[캡처]** — 프레임 캡처 + 체커보드 검출 + PnP + 포즈 추가. 검출 실패면 사유 표시되고 포즈 미추가.
4. 8~10자세 반복 (자세 다양성 가이드 ↓).
5. **Compute 카드 [COMPUTE]** — `cv2.calibrateHandEye` 실행 + method 비교 + per-pose 잔차 표시. **파일 저장 X** (미리보기만).
6. 결과 해석 (§ 결과 해석 가이드). per-pose 표에서 평균에서 도드라지게 벗어난 빨강 행만 삭제 → 자세 추가 캡처 → 다시 COMPUTE. σ가 충분히 작아질 때까지 반복.
7. 만족스러우면 **Commit 카드 [COMMIT]** — `hand_eye.npz`에 저장.

자세 다양성이 핵심 (5DOF 한계 안에서 최대한):

- joint 1 base yaw — 좌우 회전 (월드 yaw)
- joint 4 wrist pitch — 위아래 끄덕임
- joint 5 wrist roll — 비틀기
- 셋을 골고루 섞기. 한 축만 위주로 돌리면 TSAI 회전 추정이 부정확해짐.
- 체커보드는 화면 중앙 가깝게, 너무 비스듬한 각도(<30°)는 PnP 정확도 떨어짐.
- 매 자세 캡처 직전 로봇 완전 정지 (모터 명령 전송 후 ~0.5s 대기).

### 결과 해석 가이드

COMPUTE 결과를 보고 어떤 조치를 취할지 판단하는 룰. 색 임계값은 [HandEyeResults.tsx](frontend/src/components/calibration/HandEyeResults.tsx)에 박혀 있음.

> **주의** — 현재 코드의 per-pose drot/dt + σ_rot/σ_t는 **첫 포즈 기준 + std-of-deviations** 라 통상 의미와 어긋남. § TODO의 "outlier 계산 식 수정"으로 평균 기준 + RMS로 고쳐야 아래 임계값이 진짜로 유효. 그 전에는 다양한 자세를 outlier로 잘못 비춰주므로 워크플로우대로 자세 추가→삭제 반복 시 정확도가 오히려 떨어질 수 있음.

#### 색 임계값 (식 수정 후 기준)

| 항목                       | 의미                                                                                                                 | 초록 (좋음)  | 노랑 (경계)   | 빨강 (나쁨)   |
| -------------------------- | -------------------------------------------------------------------------------------------------------------------- | ------------ | ------------- | ------------- |
| **σ_rot**                  | T_target←base 회전의 **평균 대비 RMS** 편차. 체커보드는 안 움직였으니 이상적이면 0. FK + X 오차의 직접 측정치        | <0.5°        | <1.5°         | ≥1.5°         |
| **σ_t**                    | 위 위치 버전 (mm)                                                                                                    | <5           | <15           | ≥15           |
| **PARK / DANIILIDIS Δrot** | TSAI 대비 다른 알고리즘 결과의 차이. 같은 입력을 세 가지 다른 수학으로 풀어서 합의 정도 → 입력 self-consistency 척도 | <1°          | <3°           | ≥3°           |
| **per-pose drot / dt**     | 각 포즈가 **평균** 대비 벗어난 양 (식 수정 후). 한 포즈만 평균에서 떨어진 경우 진짜 outlier 후보                     | <0.5° / <5mm | <1.5° / <15mm | ≥1.5° / ≥15mm |

#### 진단 룰

읽는 순서: **PARK Δrot → per-pose → σ**.

1. **per-pose 표에 한두 행만 빨강 (나머진 깨끗)** → 그 포즈가 평균에서 도드라지게 벗어남 → 진짜 outlier. 삭제 후 COMPUTE 재실행 → σ 줄어들어야 함.
2. **per-pose 표 전체가 비슷한 정도로 색깔 있음** → 특정 자세 outlier 아니라 **시스템 전반 오차** (자세 다양성 부족 or FK floor). 다양성 더 추가하거나 → 정체되면 BA.
3. **PARK Δrot 노랑/빨강** (≥1°) → 입력 데이터에 outlier 섞여 있을 가능성 (PARK이 TSAI보다 outlier에 민감). per-pose 빨강 행 식별 → 삭제 → 재 COMPUTE.
4. **σ_rot 초록 (<0.5°) + σ_t 초록 (<5mm)** → 캘 품질 충분. COMMIT.

#### 액션 플레이북

| 상황                                | 조치                                                                  |
| ----------------------------------- | --------------------------------------------------------------------- |
| per-pose에 빨강 1~3개 (나머진 깨끗) | 빨강 포즈 삭제 → COMPUTE 재실행 → σ 감소 확인                         |
| per-pose 전체가 노랑/빨강이 비슷    | 특정 outlier 아님. 자세 다양성 늘려 추가 캡처. 정체되면 FK floor → BA |
| PARK 노랑, σ_rot 경계               | 자세 다양성 부족 가능 → joint 1/4/5 분포 점검 후 추가 캡처            |
| 모든 게 깨끗한데 σ_rot ~ 1° 정체    | FK floor 확정 → Bundle Adjustment 필요                                |

> **TSDF 목표치**: σ_rot < 0.5° / σ_t < 5mm. 그 위면 라이브 클라우드 바닥이 Z=0 그리드 대비 눈에 띄게 사선/들림.

### 정확도 향상 로드맵

#### 문제 진단

σ_rot ~1.5° 정체는 `cv2.calibrateHandEye` 알고리즘이 아니라 **FK 입력의 systematic 오차**가 원인일 가능성이 높음. 자세 추가해도 안 줄어드는 건 랜덤 노이즈가 아니라 자세마다 일관된 방향의 오차가 있다는 뜻.

FK 오차의 출처 (DIY 5축에서 큼):

- **모터 zero offset** — `joint_rad = (raw - 2048)/4095 * 2π`는 "raw 2048 = URDF zero"를 가정. Dynamixel 혼(horn) 조립 시 한 톱니 어긋나면 ~1°. `motors.yaml`에 `zero_offset` 보정 없음 → 각 모터 ±1~2° 오차 상존.
- **링크 길이** — OMX_F는 Robotis OMX의 커스텀 변형. URDF의 link geometry가 물리 조립과 정확히 일치한다는 보장 없음. 3D프린트 파트면 ±0.3mm.
- **중력 처짐** — XL430 그룹이 11V (정격 10~14.8V 하한). joint 2/3에서 자세 의존적 sag.
- **체커보드 PnP** — D405 factory intrinsic + 체커보드면 보통 0.1~0.3° 수준. 보통 FK보다 정확.

#### 다음 작업 1 — Pose 안정 ID 도입 (즉시)

배경: 현재 `HandEyeCalibration.remove_pose(index)`가 `del self.poses[index]`라 삭제 시 뒤 포즈들의 인덱스가 시프트됨. Compute 결과의 per-pose 행은 옛 인덱스, 캡처 리스트는 새 인덱스 → 매핑 깨짐. outlier 여러 개 삭제하거나 삭제 후 추가 캡처하면 어느 행이 어느 포즈인지 추적 불가.

**Backend** ([backend/modules/calibration/hand_eye.py](backend/modules/calibration/hand_eye.py)):

- `Pose` dataclass에 `id: int` 필드 추가.
- `HandEyeCalibration._next_id` 카운터 (삭제해도 재사용 X — 한 세션 내 monotonic).
- `add_pose`에서 id 자동 부여 (또는 Pose 생성 시 None이면 부여).
- `remove_pose(index)` → `remove_pose_by_id(id)` 또는 인자 의미를 id로 변경.
- `_compute_residuals` / `list_poses_meta`에서 `enumerate` 대신 `pose.id` 사용 → per-pose dict의 `index` 키를 `id`로 교체.
- `reset()` 시 `_next_id`도 0으로 리셋.

**Frontend** ([frontend/src/components/calibration/types.ts](frontend/src/components/calibration/types.ts) + 컴포넌트):

- `PoseMeta`에 `id: number` 추가. 기존 `index`는 표시용 순서로 유지 가능.
- `PerPoseResidual.index` → `id`로 키 이름 변경 (백엔드 응답과 동기화).
- `CALIB_HANDEYE_REMOVE_POSE` 호출 시 `{id}` 전달.
- 캡처 리스트와 Compute 결과 per-pose 행 모두 `#<id>` 표시. 같은 ID면 동일 포즈로 즉시 식별.
- 각 per-pose 행에 휴지통 아이콘 직접 삽입 (체크박스+일괄삭제 불필요 — id 기반이라 한 번에 하나씩 해도 안 꼬임).
- 캡처 후 새 포즈 추가되어도 기존 Compute 결과의 id들은 그대로 유효 (단 `computeStale` 표시).

검증: outlier 삭제 → 추가 캡처 → 다시 outlier 삭제를 연속으로 했을 때 표시되는 id와 실제 삭제되는 포즈가 일치하는지.

#### 다음 작업 2 — Bundle Adjustment (정확도 향상)

> 안정 ID 도입 + 재캘 후 outlier 제거하고 σ_rot 측정 → **0.5° 못 깨면** BA 진입. 그 위면 알고리즘이 아니라 FK 입력이 floor이므로 자세 더 추가해도 무의미.

배경: 모든 자세의 관측을 동시에 최적화해 FK 오차도 흡수.

**원리**:

```
변수: joint_zero_offset[5], R_cam2gripper(3 = rodrigues), t_cam2gripper(3)
       총 11개. 선택적으로 링크 길이 보정 추가.

목적함수: Σ_pose Σ_corner ||proj(corner_world_estimated) - corner_pixel_observed||²
  - corner_world: 체커보드의 알려진 3D 격자 위치 + 추정된 board pose
  - proj: K · [R|t]_cam_world · corner_world
  - 또는 더 단순: Σ_pose ||T_base←board(pose) - mean_T_base←board||² (σ_rot 직접 최소화)

해결: scipy.optimize.least_squares(method='lm') — Levenberg-Marquardt
초기값: 현재 cv2.calibrateHandEye 결과를 seed (joint_offset은 0으로 시작)
```

**파일 배치**:

- `backend/modules/calibration/bundle_adjust.py` 신규.
- `HandEyeCalibration.calibrate(method=...)` API에 `"bundle"` 모드 추가하거나 별도 클래스.
- joint_offset 결과는 `robot/calibration/joint_offsets.npz` (신규)에 저장.
- `motor_node`가 raw→rad 변환 시 이 파일을 로드해 자동 반영. 없으면 0으로 동작 (하위 호환).
- [backend/core/units.py](backend/core/units.py)의 `raw_to_rad` 시그니처에 offset 받도록 확장 검토.

**Frontend**:

- Compute 카드의 알고리즘 드롭다운에 `bundle-adjust` 추가.
- BA 결과에 joint_offset 5개도 같이 표시.
- COMMIT 시 "joint_offset도 함께 적용하시겠습니까?" 확인 다이얼로그.

**성공 기준**: BA COMPUTE σ_rot < 0.5° / σ_t < 5mm. PARK/DANIILIDIS Δrot도 같이 떨어져야 진짜 개선 (안 그러면 BA가 다른 오차로 흡수해버린 거).

#### 미사용 옵션 (참고만)

이전에 검토했던 안들. 위 경로가 더 직접적인 진단/개선이라 후순위:

- **바닥 평면 RANSAC fit** — PointCloudNode가 plane fit → Z=0 대비 각도/거리 발행. 간접 검증이고 평평한 바닥이 카메라에 항상 잡혀야 함. COMPUTE σ가 더 직접적.
- **멀티 자세 누적 클라우드** — 시각적으로 강력하지만 정량 수치 없음. 이미 RobotScene의 `<Grid>`와 라이브 클라우드로 정성 비교는 됨.

## 운영 메모

- 카메라 Pi / 모터 Pi: Ubuntu 22.04, uv 설치 완료
- IP: 192.168.0.101 (모터), 192.168.0.102 (카메라)
- 일반 의존성은 Pi 둘 다 `uv sync --only-group <role>` 가능
- `pyrealsense2`와 `open3d`는 aarch64 wheel 이슈 있음 — `pyrealsense2`는 카메라 Pi 소스 빌드, `open3d`는 PC만 필요하니 무관

## TODO / 다음 단계

### 캘리브레이션

> 이미 구현: 라이브 체커보드 프리뷰, 캡처/COMPUTE/COMMIT 분리, Pose 안정 ID + 휴지통 삭제, per-pose 잔차, method 비교, FK 흩어짐 검증, 2-컬럼 Hand-Eye 탭 레이아웃. 결과 판독은 § 결과 해석 가이드 참조.

**목표 워크플로우** (캘 정확도 향상용):

1. 10장 정도 캡처
2. **COMPUTE** → 결과 + per-pose outlier 표 확인
3. 결과 안 좋으면 outlier로 표시된 자세 삭제 + 다양한 자세로 몇 장 더 캡처
4. 다시 COMPUTE
5. σ가 충분히 작아질 때까지 3–4 반복

---

#### 다음 작업 — Outlier 식 수정 + 진단 트리 + UI 배너 (즉시. BA 전 정확도 짜내기)

##### Step 1 — Outlier 계산 식 수정 (정확도 향상의 핵심)

**문제**: 현재 [hand_eye.py `_compute_residuals`](backend/modules/calibration/hand_eye.py)의 outlier 검출이 **holistic이 아님**:

```python
ref_R = T_target2base_list[0][:3, :3]    # ← pose[0]을 기준으로 고정
drot = _rotation_diff_deg(ref_R, T[:3, :3])    # 모든 포즈의 회전 편차를 "첫 포즈 대비"로 측정
sigma_rot = np.std(rot_devs)              # 편차들의 std (RMS 아님)
sigma_t   = np.std(pos_devs)              # 거리들의 std (RMS 아님)
```

이 식의 결과:

- **다양성을 outlier로 오인**: 새로 다양한 자세를 추가하면 "첫 포즈와 다르다" → 빨강 → 사용자가 지움 → X가 클러스터에 과적합 → 실제 정확도 ↓.
- **σ_rot / σ_t 값이 통상 의미와 어긋남**: 이미 "편차 스칼라"인 값들의 std라 진척도 측정 신뢰 불가.

사용자가 워크플로우대로 "캡처→compute→outlier 삭제→재캡처" 반복해도 정확도가 오히려 떨어지는 원인이 이것.

**수정 방향**: 기준을 **첫 포즈 → 평균(holistic)** 로 옮기고, σ를 표준 RMS로.

```python
# 1) 평균 회전: T_target2base 회전들의 quaternion 평균.
#    각 R → quaternion q_i → M = Σ q_i q_i^T → numpy.linalg.eigh로 최대 고유벡터 = mean q.
#    (scipy 의존 없이 numpy로 OK. Markley/Crassidis 표준 방법.)
# 2) drot[i] = angle(R_i, R_mean)    # 모든 포즈에 대칭. degrees.
#    dt[i]   = ||t_i - t_mean||      # mm
# 3) sigma_rot_deg = sqrt(mean(drot_i^2))   # RMS, np.std 아님
#    sigma_t_mm    = sqrt(mean(dt_i^2))     # RMS
```

수정 후 효과:

- 진짜 outlier (한 포즈만 평균에서 멀리 떨어짐) → drot 큼 → 빨강 → 삭제하면 σ 줄어듦.
- 다양한 자세 추가 (분포가 넓어짐) → 모든 자세의 drot이 비슷한 정도로 커짐 → 어느 하나만 빨강이 아니라 σ_rot 전체가 커짐 → 사용자가 "이건 floor 문제구나" 판단 가능.

##### Step 2 — 진단 결정 트리 (Step 1 위에 얹음)

목적: 사용자가 σ / per-pose 색을 보고 직접 추론하지 않아도, 코드가 다음 행동을 명시적으로 안내.

[backend/modules/calibration/hand_eye.py](backend/modules/calibration/hand_eye.py)에 `_diagnose()` 함수 추가. `compute_with_diagnostics()` 응답에 `diagnosis: dict` 필드로 묶어 반환.

판단 순서와 로직:

```python
def _diagnose(per_pose, sigma_rot_deg, sigma_t_mm, method_compare, poses) -> dict:
    # 1) 단일 outlier 검출 — MAD 기반 robust
    drots = np.array([p['drot_deg'] for p in per_pose])
    median = np.median(drots)
    mad = np.median(np.abs(drots - median))
    # MAD가 너무 작으면 모든 자세가 비슷하다는 뜻 → outlier 없음 (분모 0 방지)
    if mad > 1e-6:
        threshold = median + 3.0 * mad
        outlier_ids = [p['id'] for p in per_pose if p['drot_deg'] > threshold]
        if outlier_ids:
            return {
                'status': 'outlier_present',
                'severity': 'action_required',
                'message': f"포즈 #{outlier_ids}이(가) 평균에서 도드라짐 — 삭제 후 재 COMPUTE",
                'outlier_ids': outlier_ids,
            }

    # 2) 자세 다양성 부족 검출 — joint 1/4/5 회전 범위
    #    (5DOF arm: joint 1=base yaw, joint 4=wrist pitch, joint 5=wrist roll)
    joints = np.array([p.joint_angles_rad for p in poses])  # (N, 5)
    ranges_deg = np.degrees(joints.max(0) - joints.min(0))
    DIVERSITY_THRESHOLD = {0: 60.0, 3: 40.0, 4: 40.0}  # joint index → 최소 범위
    insufficient = [
        (idx + 1, ranges_deg[idx]) for idx, thr in DIVERSITY_THRESHOLD.items()
        if ranges_deg[idx] < thr
    ]
    if insufficient and sigma_rot_deg >= 0.5:
        names = {1: 'base yaw', 4: 'wrist pitch', 5: 'wrist roll'}
        details = ', '.join(f"joint {i}({names[i]}) {r:.0f}°" for i, r in insufficient)
        return {
            'status': 'insufficient_diversity',
            'severity': 'action_required',
            'message': f"다양성 부족: {details} — 부족한 축의 자세 추가 캡처",
            'low_diversity_joints': [i for i, _ in insufficient],
        }

    # 3) 캘 품질 충분 — COMMIT 권장
    if sigma_rot_deg < 0.5 and sigma_t_mm < 5.0:
        return {
            'status': 'good',
            'severity': 'success',
            'message': f"품질 충분 (σ_rot {sigma_rot_deg:.2f}°, σ_t {sigma_t_mm:.1f}mm) — COMMIT 권장",
        }

    # 4) FK floor 도달 — BA 필요
    #    여기 도달 = outlier 없음 + 다양성 OK + σ가 목표 미달
    park_drot = next((c['drot_deg'] for c in method_compare if c['method'] == 'PARK'), None)
    park_ok = park_drot is not None and park_drot < 1.0
    return {
        'status': 'fk_floor_reached',
        'severity': 'action_required',
        'message': (
            f"σ_rot {sigma_rot_deg:.2f}° 정체 (outlier 없음 + 다양성 충분"
            f"{' + PARK 합의' if park_ok else ''}) — cv2 한계. Bundle Adjustment 필요."
        ),
        'sigma_rot_deg': sigma_rot_deg,
        'park_drot_deg': park_drot,
    }
```

호출부 ([hand_eye.py `compute_with_diagnostics`](backend/modules/calibration/hand_eye.py)) 마지막에:

```python
return {
    ...,
    'diagnosis': self._diagnose(per_pose, sigma_rot_deg, sigma_t_mm, compare, self.poses),
}
```

**한계 (의도적)**: "모든 자세에 약간씩 모터 흔들림" 케이스는 σ 전반 증가로만 나타나서 위 트리는 `fk_floor_reached`로 진단함. 실제론 데이터 품질 문제일 수 있는데 구분 불가. 이걸 분리하려면 캡처 시 `reproj_err_px`/`joint_drift_deg` 측정이 필요(향후 작업). 식 수정 + 진단 트리만으로도 정확도는 floor까지 충분히 끌어올림.

##### Step 3 — Frontend: 진단 배너 표시

[frontend/src/components/calibration/types.ts](frontend/src/components/calibration/types.ts):

```ts
export type DiagnosisStatus =
  | "outlier_present"
  | "insufficient_diversity"
  | "fk_floor_reached"
  | "good";

export type Diagnosis = {
  status: DiagnosisStatus;
  severity: "action_required" | "success";
  message: string;
  outlier_ids?: number[];
  low_diversity_joints?: number[];
  sigma_rot_deg?: number;
  park_drot_deg?: number;
};

// ComputeData에 추가:
//   diagnosis: Diagnosis;
```

[frontend/src/components/calibration/HandEyeResults.tsx](frontend/src/components/calibration/HandEyeResults.tsx) `ComputePreview` 상단에 배너 컴포넌트 추가:

- `status === 'good'` → 초록 배너 + COMMIT 강조.
- `status === 'outlier_present'` → 빨강 배너 + `outlier_ids` 강조 표시 (per-pose 표에서 그 행 하이라이트).
- `status === 'insufficient_diversity'` → 주황 배너 + 어느 joint 부족한지 명시.
- `status === 'fk_floor_reached'` → 보라/파랑 배너 + "Bundle Adjustment 필요" 메시지 (BA UI는 아직 없지만 안내만).

##### 검증 시나리오

1. 일부러 한 자세를 흔들면서 캡처 → 다른 자세는 정상 → 진단: `outlier_present` + 그 자세 id.
2. joint 4(wrist pitch)만 거의 안 움직이고 10개 캡처 → 진단: `insufficient_diversity` + joint 4 명시.
3. 정상 캡처 + 다양성 OK + 정확한 캘 → 진단: `good`.
4. 정상 캡처 + 다양성 OK인데 σ_rot가 1° 부근 정체 → 진단: `fk_floor_reached` → BA로 가야 함.
5. 같은 데이터로 식 수정 전/후 σ_rot 비교 — 수정 후 값이 통상 RMS 의미와 일치하는지.

---

#### 향후 작업 (이번엔 안 함)

- **데이터 품질 척도** (`reproj_err_px`, `joint_drift_deg`, `board_tilt_deg`) — 진단 트리의 "모터 흔들림" 케이스 분리에 필요. 식 수정 + 진단 트리만으로 정확도가 부족할 때만 추가.
- **Bundle Adjustment** — 식 수정 + 진단 트리 끝낸 다음 워크플로우로 σ_rot < 0.5° 끝까지 못 깨면 그때 진입. 자세 설계는 § 정확도 향상 로드맵의 "다음 작업 2" 참조.

### 문서

- **CLAUDE.md 전체 재정비** — 코드와 안 맞는 부분 점검 + 갱신. 구조 재배치는 1차 완료(2026-05-14)했으나, 본문 내용이 현재 코드와 실제로 일치하는지 항목별 검증 필요. 특히 토픽/서비스 키, 노드 책임, 캘리브 서비스 시그니처, 호스트 config 항목.
- cluade.md 부분에서 하드웨어, 운영 등등 바뀌지 않는것들이나 지금 당장 필요하지 않는것들은 docs 에 md 로 분리하느것에 대해 논의하기

### TODO

- BA 결과는 어느정도 되었음 (TSDF 도전해도 될만한 수준)
- 캘과정에서 BA 후 overflow (scroll) 안되는 문제 있음
- 캘 과정에서 각 pose 가 메모리에만 저장되어 있어서 코드 수정후 재캘 어려움
- 코드 여기저기 남발 main 에서 offset load 등 정리하기
