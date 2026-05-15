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
- 시각 검증: 라이브 클라우드 바닥이 Z=0 그리드 대비 살짝 사선 + ~2cm 들림 (1.5° 회전 오차와 일치). 이전 캘 대비 명확히 개선됨. 더 줄이려면 워크플로우 개선 후 재캘 (TODO 참조).

### Hand-Eye 재캘 절차

⚠️ **현재 워크플로우 한계** — `CALIB_HANDEYE_START`는 이름과 달리 reset이 아니라 **"한 프레임 캡처 + 포즈 추가"** 동작 ([calibration_node.py:124-183](backend/nodes/calibration_node.py#L124-L183)). 명시적 reset 서비스가 없어 새로 시작하려면 **백엔드 프로세스 재시작**이 유일한 방법. (`HandEyeCalibration.reset()` 메서드는 존재하지만 노출 안 됨.) 개선안은 TODO § 캘 워크플로우 개선 참조.

1. 백엔드 프로세스 재시작 (이전 세션의 포즈 잔재 제거)
2. 프론트 띄우고 캘리브 패널 열기 (intrinsic은 이미 로드돼야 함)
3. 매 자세마다 **CALIB_HANDEYE_START** 호출 — 호출 시점에 프레임 캡처+체커보드 검출+PnP+포즈 추가가 일어남
4. 8~10자세 반복 (자세 다양성 가이드 ↓)
5. **CALIB_HANDEYE_SAVE** — `calibrate()` 실행 + `hand_eye.npz` 저장 + method 비교 로그 자동 출력 (현재는 결과 확인과 저장이 한 번에 이뤄짐)

자세 다양성이 핵심 (5DOF 한계 안에서 최대한):

- joint 1 base yaw — 좌우 회전 (월드 yaw)
- joint 4 wrist pitch — 위아래 끄덕임
- joint 5 wrist roll — 비틀기
- 셋을 골고루 섞기. 한 축만 위주로 돌리면 TSAI 회전 추정이 부정확해짐.
- 체커보드는 화면 중앙 가깝게, 너무 비스듬한 각도(<30°)는 PnP 정확도 떨어짐.
- 매 자세 캡처 직전 로봇 완전 정지 (모터 명령 전송 후 ~0.5s 대기).

품질 판정 — save 직후 백엔드 콘솔의 method 비교 로그:

```
─── method 비교 (기준: TSAI) ───
  TSAI          Δrot=  0.000°  Δt=  0.0mm  (기준)
  PARK          Δrot= X.XXX°  Δt= XX.Xmm
  DANIILIDIS    Δrot= X.XXX°  Δt= XX.Xmm
```

- 셋 다 Δrot < 1° → 데이터 self-consistent. 캘 자체 품질 OK. 그래도 바닥이 어긋나면 자세 추가 / 체커보드 인쇄 정확도 / D405 factory intrinsic 오차 의심.
- Δrot 1~3° → 자세 다양성 부족. 회전축 분포 재점검 후 추가 캡처.
- Δrot > 3° → 자세 품질 문제 (흔들림, 체커보드 부분 가림, PnP 실패에 가까운 자세). reset 후 다시.

### 정확도 시각 검증 (검토 중)

캘 직후 "잘된 캘인지"를 화면에서 즉시 확인하는 방법. 현재는 콘솔 로그(method 비교)와 사용자 눈으로 바닥 기울기 보는 게 전부. 후보:

**A. 바닥 평면 fit 라이브 표시** (낮은 구현 비용)
PointCloudNode가 RANSAC plane fit으로 바닥 평면을 추출 → 평면 normal과 (0,0,1) 사이 각도(°), Z=0 평면으로부터 평균 거리(mm)를 토픽으로 발행 → 패널 한쪽에 라이브 수치 표시. 좋은 캘이면 < 0.5°, < 5mm 수준. 단점: 평평한 바닥이 카메라에 항상 잡혀야 의미 있음.

**B. 멀티 자세 누적 클라우드** (가장 강한 시각 증거)
"검증 모드" 토글로 PointCloudNode가 최근 N개 프레임을 base 프레임에 그대로 누적(replace 안 함). 로봇을 다양한 자세로 천천히 움직이면서 동일 정적 장면을 비추면 — 좋은 캘이면 한 점으로 모이고, 나쁜 캘이면 ghosting/double-wall로 흐려짐. 단점: 별도 누적 버퍼 필요, 메모리/렌더 비용.

**C. 그리드 vs 클라우드 시각 비교** (이미 거의 됨)
[RobotScene.tsx](frontend/src/components/workspace3d/3d/RobotScene.tsx)에 `<Grid>`로 Z=0 평면이 이미 그려져 있음. 바닥 클라우드가 그 그리드와 평행하고 거의 일치하면 OK. 무료지만 정량 수치 없음 — 1° 미만의 미세한 어긋남은 눈으로 못 잡음.

**D. 알려진 마커 위치 비교**
체커보드를 베이스 좌표 (X, Y, 0) 같이 ground-truth 위치에 두고, 디텍터로 위치 추정 → ground-truth와의 차이를 표시. 단 PnP 자체 정확도가 섞여 들어와서 hand-eye만 따로 분리 안 됨.

추천 (구현 순): **A를 1순위** (낮은 비용으로 즉시 수치 피드백), **B를 2순위**. C는 무료라 일단 눈으로 항상 확인.

## 운영 메모

- 카메라 Pi / 모터 Pi: Ubuntu 22.04, uv 설치 완료
- IP: 192.168.0.101 (모터), 192.168.0.102 (카메라)
- 일반 의존성은 Pi 둘 다 `uv sync --only-group <role>` 가능
- `pyrealsense2`와 `open3d`는 aarch64 wheel 이슈 있음 — `pyrealsense2`는 카메라 Pi 소스 빌드, `open3d`는 PC만 필요하니 무관

## TODO / 다음 단계

### 캘리브레이션

- **캘 워크플로우 개선** — 현재 한 번 시작하면 reset 수단이 없고(백엔드 재시작 필요), `CALIB_HANDEYE_SAVE`가 calibrate + 파일 저장을 한 번에 수행해서 "결과 확인 후 채택/기각" 흐름이 없음. 개선 방향:
  - `CALIB_HANDEYE_RESET` 서비스 신설 (poses 비우기)
  - SAVE를 **CALIB_HANDEYE_COMPUTE** (calibrate만 수행 + 결과 + method 비교 + 잔차 반환)와 **CALIB_HANDEYE_COMMIT** (마지막 compute 결과를 `hand_eye.npz`로 저장) 두 단계로 분리
  - `_srv_handeye_save`에 AX=XB 잔차 출력 추가
  - 잔차 기반 outlier 포즈 제거 (한 포즈가 큰 잔차 → 그 포즈만 빼고 재계산 옵션)
  - 자세 자동 추천 (현재 누적된 포즈들의 회전축 분포를 보고 부족한 축 가이드)
  - 프론트 캘리브 패널에 포즈별 잔차 표 + 개별 삭제 UI
- **캘 정확도 시각 검증 구현** — 위 § 정확도 시각 검증 옵션 A(바닥 평면 RANSAC 라이브 수치) 1순위.
- 워크플로우 개선 후 자세 다양성 신경 써서 hand-eye **재캘 2차** — PARK Δrot 1° 미만 목표.

### D405 마이그레이션 잔여 phase

- **Phase 4 — 멀티-뷰 캡처 → PLY/npz 저장 → TSDF → 메시.** 아래 § Phase 4 구현 가이드 참조.
- **Phase 5 — ICP 등록.** PointCloudNode 서비스로. (TSDF 자체로 hand-eye 작은 오차는 어느 정도 흡수되지만, 부족하면 자세별 ICP refinement 추가.)
- **Phase 6 — Detector depth lookup 전환.** 현재의 평면-Z=0 역산 자리에 depth 직접 lookup.

### Phase 4 구현 가이드 (2026-05-15 아키텍처 합의)

**목적:** 로봇이 여러 자세를 순회하며 정지 캡처 → 자세별 npz/PLY 저장 → TSDF 적분 → 메시 export. 단일 거대 PLY/메시 1개 생성이 최종 산출물.

#### 핵심 결정

1. **캡처는 스트림 frame 재사용 X, 전용 서비스 신설.** 스트림(8 FPS, voxel down, 라이브 프리뷰)과 캡처(정밀도 최우선, multi-frame averaging 가능)는 성격이 다름. 한 frame을 양쪽 용도로 쓰면 한쪽 튜닝이 다른 쪽을 망침.
2. **카메라 Pi는 raw 데이터 획득만, PC가 가공/판단.** Pi는 N장 raw frame을 묶어 service 응답으로 반환. averaging/filter/cam→base 변환/저장/TSDF는 전부 PC. 알고리즘 튜닝 시 Pi 안 건드림.
3. **정지 후 캡처가 전제.** D405는 active stereo라 움직이면서 캡처 불가능 + 정지가 TCP 포즈 시간 정합 문제도 해결. 캡처 직전 motion stop + ~0.5s wait.
4. **TSDF/메시 빌드는 PC PointCloudNode 안 서비스.** open3d가 PC 그룹에만 있고 (aarch64 wheel 부재) Pi는 어차피 못 함.
5. **저장은 로컬 파일시스템 — MinIO/FTP 도입 X.** 단일 사용자 + PC 한 대 처리 + 세션당 ~수십 MB라 over-engineering. 미래 산업급 카메라(Photoneo 25MB/Zivid 10MB)까지는 동일 패턴 유지 가능. 그 위 metrology급으로 가면 어차피 transport/compute 레이어 통째 재설계라 미리 추상화해도 못 맞춤. **storage 추상화 인터페이스 만들지 않음.** `pathlib.Path` 직접 사용.
6. **Service-response envelope으로 N frame 묶음 통째 전송.** 1MB 수준이라 Zenoh queryable 응답으로 충분. 별도 토픽 + ack 패턴은 race condition 위험만 추가.

#### 데이터 흐름

```
[로봇 자세 i로 이동 → motion stop → 0.5s wait]

PC PointCloudNode
  ──CAMERA_CAPTURE_DEPTH_FRAMES { num_frames: 5 }──>  카메라 Pi CameraNode
                                                       wait_for_frames N장,
                                                       align(color↔depth),
                                                       각각 zstd(depth) + JPEG(color),
                                                       N개 depth_frame을 envelope으로 묶음
  <──{ frames: [depth_frame×N], intrinsics }─────────  ~500KB~1MB
  ──MOTION_GET_TCP──> 모터 Pi
  <──{ position, quaternion }─

PC PointCloudNode
  - depth N장 median (averaging)
  - color 중 1장 채택 (또는 평균)
  - cam→base 변환 (T_cam_to_base = T_world←ee · T_gripper←cam, [[d405-handeye-convention]])
  - PLY 빌드 (base frame)
  - npz 저장 (재가공/TSDF용 원본)
  - PLY 저장 (시각화/라이브러리용)
  - POINTCLOUD_SNAPSHOT 토픽 발행
```

#### 저장 레이아웃

```
robot/scans/{session_id}/
  scan_001.npz   ← 원본 (TSDF 입력)
  scan_001.ply   ← base-frame, 시각화용
  scan_002.npz
  scan_002.ply
  ...
robot/models/
  mesh_{session_id}.ply   ← TSDF 결과 메시
```

`session_id`: `session_YYYYMMDD_HHMMSS` 같은 timestamp slug. 사용자가 별도 이름 줄 수 있게 옵션.

npz 내용:
```python
{
  "depth_z16":   uint16 (H, W),       # averaging 후 1장
  "color_bgr":   uint8 (H, W, 3),     # 또는 color_jpeg: bytes
  "fx fy cx cy width height": float,
  "depth_scale": float,
  "tcp_position": float (3,),
  "tcp_quaternion": float (4,),       # xyzw
  "hand_eye_R":  float (3, 3),
  "hand_eye_t":  float (3,),
  "timestamp":   float,
  "depth_trunc": float,               # 캡처 시 사용 값
  "num_frames":  int,                 # averaging에 쓴 frame 수
}
```

#### 토픽/서비스 추가

[backend/core/topic_map.py](backend/core/topic_map.py) + [frontend/src/constants/topics.ts](frontend/src/constants/topics.ts) **양쪽 모두**:

- `Service.CAMERA_CAPTURE_DEPTH_FRAMES = "omx/camera/srv/capture_depth_frames"` (카메라 Pi, 신설)
- `Service.POINTCLOUD_CAPTURE = "omx/pointcloud/srv/capture"` (PC, 신설)
- `Service.POINTCLOUD_LIST_SCANS = "omx/pointcloud/srv/list_scans"` (신설)
- `Service.POINTCLOUD_LOAD_SCAN = "omx/pointcloud/srv/load_scan"` (신설)
- `Service.POINTCLOUD_CLEAR_SNAPSHOT = "omx/pointcloud/srv/clear_snapshot"` (신설)
- `Service.POINTCLOUD_BUILD_MESH = "omx/pointcloud/srv/build_mesh"` (TSDF, 신설)
- `Topic.POINTCLOUD_SNAPSHOT = "omx/pointcloud/snapshot"` (이미 토픽맵에 예약돼 있을 가능성 — 확인 후 미정의면 추가)

#### feat/realsense-d405 브랜치에서 그대로 재사용 가능한 코드

단일 머신 전제로 작성됐지만 도메인 로직은 분산과 무관:

- `_build_pcd(color_bgr, depth_z16, intr, depth_scale)` — RGBD → PointCloud
- `_build_cam_to_base(quat, t_eb, R_ce, t_ce)` — 4x4 변환 행렬 빌드
- `_quat_to_rot(q)` — xyzw quaternion → 3x3
- `_publish_snapshot(pcd)` — PLY를 SNAPSHOT 토픽 binary 페이로드로 발행
- `_srv_list_ply` / `_srv_load_ply` — 디렉토리 탈출 검증 포함된 라이브러리 서비스
- 프론트 [PointCloudPanel.tsx](frontend/src/components/workspace3d/panels/PointCloudPanel.tsx) — Capture/Library/Clear UI (세션 개념만 얹으면 됨)

가져올 때 주의: `RealsenseCapture` 직접 사용 부분(`_srv_capture`의 `grab_aligned_blocking`)은 분산에 맞지 않으므로 위 § 데이터 흐름대로 재작성.

#### Pi 측 캡처 서비스 구현 핵심

```python
def _srv_capture_depth_frames(self, req):
    n = (req.get("data") or {}).get("num_frames", 5)
    # 스트림이 켜져있어도 같은 producer thread가 다음 N frame을 처리.
    # 별도 lock으로 latest_frame 캐시 비우고 N장 모일 때까지 대기.
    frames = self._rs.grab_n_aligned_blocking(n, timeout=2.0)
    if not frames:
        return {"success": False, "message": "frame 획득 실패", "data": {}}

    encoded = [depth_frame.encode(c, d, intr, depth_scale) for c, d in frames]
    payload = envelope_encode(encoded)  # [u32 N][u32 len1][frame1][u32 len2][frame2]...
    return {"success": True, "message": f"{n} frames", "data": {"payload_b64": ...}}
    # 또는 raw bytes 응답을 zenoh service가 받는 방식에 맞춰 변환
```

(envelope 포맷은 binary WS 프레이밍 패턴 그대로 차용 가능. base64는 service response가 dict라 어쩔 수 없이 필요할 가능성 — Zenoh service가 raw bytes 응답 가능한지 [base_node.py:call_service](backend/core/base_node.py) 시그니처 먼저 확인.)

#### PC 측 averaging

```python
depths = np.stack([f.depth for f in frames])  # (N, H, W) uint16
# 0(invalid)은 median에서 제외하기 위해 mask 처리
masked = np.where(depths == 0, np.iinfo(np.uint16).max, depths)
depth_med = np.median(masked, axis=0).astype(np.uint16)
depth_med[depth_med == np.iinfo(np.uint16).max] = 0
```

color는 일단 마지막 1장 채택. 차후 평균 필요해지면 LAB 색공간에서 평균.

#### 초기 기본값

- `num_frames`: **5** (시작값, 노이즈/시간 trade-off 보고 조정)
- `depth_trunc`: **0.8m** (TSDF용. 라이브 1.0m보다 짧게 잡아 noise 컷)
- TSDF voxel: **2mm** (시작값. 메시 디테일 vs 메모리 trade-off)
- RealSense post-processing filter: **일단 적용 안 함** (raw frame median으로 시작). 부족하면 PC 측에서 temporal/spatial filter 추가.

#### TSDF 빌드 (POINTCLOUD_BUILD_MESH)

```python
def _srv_build_mesh(self, req):
    session_id = req["data"]["session_id"]
    voxel = req["data"].get("voxel_size", 0.002)
    sdf_trunc = req["data"].get("sdf_trunc", voxel * 5)

    vol = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=voxel, sdf_trunc=sdf_trunc,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8,
    )

    for npz_path in sorted((SCANS_DIR / session_id).glob("*.npz")):
        s = np.load(npz_path)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(...)
        intr = o3d.camera.PinholeCameraIntrinsic(...)
        extrinsic = np.linalg.inv(T_cam_to_base(s))  # TSDF는 world←cam의 inverse를 받음 — Open3D 컨벤션 재확인 필요
        vol.integrate(rgbd, intr, extrinsic)

    mesh = vol.extract_triangle_mesh()
    mesh.compute_vertex_normals()
    out = MODELS_DIR / f"mesh_{session_id}.ply"
    o3d.io.write_triangle_mesh(str(out), mesh)
    return {"success": True, "data": {"path": ..., "vertex_count": ...}}
```

⚠️ Open3D `vol.integrate()`의 extrinsic 컨벤션이 `T_world←cam`인지 `T_cam←world`인지 문서 한 번 더 확인 (보통 후자).

#### 정확도 측면 메모

- hand-eye 1.5° 회전 오차가 TSDF 누적에서 ghosting/double-wall로 보일 가능성 있음. TSDF의 voxel averaging이 작은 오차는 흡수해주지만, 자세 간 회전 차이가 크면 누적됨. 메시 품질 보고 hand-eye 재캘이 더 우선일지 판단.
- 캡처 자세 다양성: 30~45° 간격으로 물체 둘레, 위/옆 mix. 5DOF 한계 안에서.
- D405 textureless 표면 hole: TSDF는 hole에 surface 안 만들지만 메시에 구멍 남음. 캡처 시 텍스처 있는 배경/표면 위에.

### 분산 / 인프라

- HW 회귀 테스트 — 분산 배치 코드 변경 후 미검증.
- Zenoh 멀티캐스트 scout 검증 (분산 시). 안 되면 host config의 `zenoh.connect`에 endpoint 명시.
- `pyrealsense2` 카메라 Pi 빌드 wheel 절차 정형화.
- 단일 머신 회귀 후 점진적 분산 시도 (모터 Pi → 카메라 Pi 순서 권장).

### 문서

- **CLAUDE.md 전체 재정비** — 코드와 안 맞는 부분 점검 + 갱신. 구조 재배치는 1차 완료(2026-05-14)했으나, 본문 내용이 현재 코드와 실제로 일치하는지 항목별 검증 필요. 특히 토픽/서비스 키, 노드 책임, 캘리브 서비스 시그니처, 호스트 config 항목.
- cluade.md 부분에서 하드웨어, 운영 등등 바뀌지 않는것들이나 지금 당장 필요하지 않는것들은 docs 에 md 로 분리하느것에 대해 논의하기

### Frontend

- panel 들 위치 하드코딩 말고 자동으로 배치하는 로직 필요

## 부록: 변경 이력 / 진단 히스토리

### D405 마이그레이션 Phase 진행

OpenCV USB 카메라 → **Intel RealSense D405** 전환. 단순 pick-and-place 센서가 아니라 범용 RGBD 디바이스로 취급 — 라이브 포인트클라우드, PLY 스냅샷, ICP 등록, detector의 Z=0 가정을 depth 직접 lookup으로 대체까지 단계적 진행.

1. ✅ **RGB-only swap** — `CameraCapture` 내부를 pyrealsense2로 교체. `pyrealsense2` 의존성 추가.
2. ✅ **Binary WebSocket 페이로드 프로토콜** — 브릿지 + `BridgeClient`가 binary WS 프레임 지원.
3. ✅ **Live 포인트클라우드 프리뷰** — `PointCloudNode` 도입. 분산 배치 이후 RealsenseCapture 직접 사용은 제거되고 `CAMERA_DEPTH_FRAME` 토픽 구독 기반으로 전환.
4. ⏳ Phase 4–6은 TODO 참조.

### 라이브 클라우드 좌표계 어긋남 — 배제된 가설

증상은 라이브 클라우드 바닥이 base XY 평면에서 들리고 사선으로 기울어지는 것. 원인 분석에서 다음 가설들은 검증되어 배제됨:

- ✅ **변환 누락 아님.** 백엔드는 camera-frame xyz를 그대로 publish하지만, 프론트 [PointCloudLayer.tsx](frontend/src/components/workspace3d/3d/PointCloudLayer.tsx)가 `<group position quaternion>` 부모 transform으로 `cameraMatrix = tcpMatrix * handEyeMatrix`을 적용 — three.js가 GPU에서 vertex별로 `parent.matrixWorld * local`을 곱하므로 base 프레임 렌더링과 수학적으로 동일.
- ✅ **TCP 링크 정의 불일치 — 배제.** 캘리브 측 [solver.py:66](backend/modules/kinematics/solver.py#L66)의 `end_effector_link` ↔ 프론트 [config.ts:53](frontend/src/lib/robot/config.ts#L53)의 `TCP_LINK_NAME = "end_effector_link"` — 같은 URDF 링크.
- ✅ **R/t 컨벤션 — 배제.** `cv2.calibrateHandEye` 출력 `R_cam2gripper, t_cam2gripper`는 OpenCV 정의상 `T_gripper←cam`. 프론트의 `cameraMatrix = tcpMatrix · handEyeMatrix = T_world←ee · T_gripper←cam = T_world←cam` 식이 정확히 일치 (inverse 빠뜨림 없음). [calibration_node.py:141-174](backend/nodes/calibration_node.py#L141-L174)의 `R/t_gripper2base`도 PyBullet `getLinkState` worldFrame pose = `T_base←gripper`로 OpenCV 입력 컨벤션과 일치.

결론: 남은 원인은 hand-eye 회전 정확도(`R_cam2gripper` 자체가 ~1.5° 틀어짐). 진단 도구로 multi-method 비교를 [hand_eye.py:calibrate()](backend/modules/calibration/hand_eye.py)에 추가, save 시 TSAI/PARK/DANIILIDIS Δrot/Δt 자동 로그 출력.

비고: "TCP 30° 돌려서 바닥 기울기 변화 보기" 같은 단순 진단은 **부적합** — hand-eye 회전 오차와 TCP 링크 불일치 둘 다 camera-local frame에서 우-곱해지는 상수 회전 오차로 수식이 같아서 거동 구별 안 됨.

### 분산 배치 작업 (PC + Pi×2) — 완료

원래 후속 단계로 분리하려던 작업을 USB 경합 문제 때문에 앞당겨 진행. 주요 코드 변경:

- **의존성 그룹 분리** ([backend/pyproject.toml](backend/pyproject.toml)) — `pi-motor` / `pi-camera` / `pc` / `all` PEP 735 그룹 + `default-groups=["dev","pc"]`.
- **노드 레지스트리** ([backend/core/node_registry.py](backend/core/node_registry.py)) — 문자열 매핑 → `importlib` lazy import.
- **호스트 config** ([backend/config/](backend/config/)) — `host_dev` / `host_pc` / `host_pi_motor` / `host_pi_camera`. main.py가 `--host` 또는 hostname으로 선택, 실패 시 `host_dev` fallback.
- **`ZenohSession.init(cfg_dict)`** — host config의 `zenoh` 섹션을 zenoh.Config로 변환 (mode/connect/listen).
- **FrameCache** ([backend/core/frame_cache.py](backend/core/frame_cache.py)) — detector/calibration이 토픽 기반으로 카메라 프레임 수신 → 분산 호환. TaskNode/StepExecutor의 카메라 인자는 미사용이라 제거.
- **CAMERA_DEPTH_FRAME 파이프라인** — 카메라 Pi에서 무손실 압축 depth + JPEG color + intrinsics 한 메시지로 발행, PC PointCloudNode가 구독해 cloud 생성. `POINTCLOUD_CONFIGURE`가 카메라 측 enable을 forward.
- **브릿지 백프레셔 + 바이너리 프레이밍** — 클라이언트별/토픽별 bounded queue, LATEST_WINS 기본 / SYSTEM_LOG는 BOUNDED_FIFO. 바이너리 토픽 binary WS 프레이밍.

이 변경은 **단일 머신(PC만)과 분산(PC+Pi×2) 양쪽 모두에서 동일하게 동작** — 같은 토픽이 Zenoh에 흐를 뿐.
