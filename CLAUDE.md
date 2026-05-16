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

### 정확도 향상 로드맵 (2026-05-17 설계)

#### 문제 진단

현재 1.5° 정체는 cv2.calibrateHandEye 알고리즘이 아니라 **FK 입력의 systematic 오차**가 원인일 가능성이 높음. 25자세 평균해도 줄지 않는다는 건 랜덤 노이즈가 아니라 자세마다 일관된 방향의 오차가 있다는 뜻.

FK 오차의 출처 (DIY 5축에서 큼):

- **모터 zero offset** — `joint_rad = (raw - 2048)/4095 * 2π`는 "raw 2048 = URDF zero"를 가정. Dynamixel 혼(horn) 조립 시 한 톱니 어긋나면 ~1°. `motors.yaml`에 `zero_offset` 보정 없음 → 각 모터 ±1~2° 오차 상존.
- **링크 길이** — OMX_F는 Robotis OMX의 커스텀 변형. URDF의 link geometry가 너의 물리 조립과 정확히 일치한다는 보장 없음. 3D프린트 파트면 ±0.3mm.
- **중력 처짐** — XL430 그룹이 11V (정격 10~14.8V 하한). joint 2/3에서 자세 의존적 sag.
- **체커보드 PnP** — D405 factory intrinsic + 체커보드면 보통 0.1~0.3° 수준. 보통 FK보다 정확.

→ "URDF가 검증됐다"는 운동학 수식의 검증이지 너의 물리 로봇과의 일치 보장이 아님.

#### 핀포인트 방식은 제외

지그(jig) 정확도 = 결과 정확도. CNC/정밀 캘 블록 없으면 자(尺)·3D프린트 정확도(±0.3~2mm)가 그대로 결과로 들어와서 현재 다포즈 방식보다 못함. **지그 제작 가능해지기 전까지 핀포인트는 보류.**

#### 4단계 작업 계획

UI는 **캘 패널 하나**에 섹션 4개로 통합 (탭 추가 X). 1~4단계 결과물이 같은 패널 안에 누적.

```
[Hand-Eye Calibration 패널]
├─ 라이브 프리뷰 (Step 1)
│    카메라 스트림 위에 체커보드 코너 오버레이
│    검출 성공/실패 색, 화면 점유율, 기울기, 자세 다양성 힌트
├─ 캡처 컨트롤 (Step 2)
│    START / RESET / 자세 리스트 + 개별 삭제
├─ 계산 (Step 2 + Step 4)
│    [COMPUTE: cv2 ▼ | bundle-adjust ▼]
│    결과 미리보기 (R, t, method 비교, 잔차, 포즈별 잔차)
│    [COMMIT] → hand_eye.npz 저장 (+ BA 시 joint_offset도 같이)
└─ 검증 (Step 3)
     [FK 흩어짐 분석 실행] — 캡처된 포즈로 후처리
     σ_rot, σ_t, 자세별 T_base←board 편차
```

##### Step 1 — 라이브 체커보드 검출 피드백 (Backend + Frontend)

배경: 지금은 자세 옮긴 뒤 `CALIB_HANDEYE_START` 호출해야 검출 결과를 알 수 있음. 시행착오 비용이 큼 — 자세 옮길 때 잘 보이는지 실시간으로 모르니까.

**Backend** ([backend/nodes/calibration_node.py](backend/nodes/calibration_node.py)):

- 5~10Hz `_preview_loop` 추가 (DetectorNode의 `_detection_loop` 패턴 차용).
- FrameCache에서 프레임 받아 체커보드 검출만 시도 (PnP는 선택). `pose_estimator.find_corners()` 같은 메서드 활용.
- 발행 토픽: `omx/calibration/state/handeye_preview` (신규)
  - 페이로드: `{detected: bool, corners: [[x,y], ...], coverage_ratio: float, tilt_deg: float, timestamp: float}`
- 구독자가 있을 때만 루프 active (불필요한 CPU 안 씀). `subscribe`/`unsubscribe` 이벤트로 토글.
- `Topic` 클래스 ([backend/core/topic_map.py](backend/core/topic_map.py))에 `CALIB_HANDEYE_PREVIEW` 추가.

**Frontend**:

- [frontend/src/constants/topics.ts](frontend/src/constants/topics.ts)의 `Topic`에 동일 키 추가.
- 캘 패널 컴포넌트에서 이 토픽 구독 → 카메라 스트림 위에 SVG/Canvas 오버레이로 코너 표시.
- 패널이 마운트될 때만 구독 → 자동으로 백엔드 루프 켜짐.

##### Step 2 — 워크플로우 개선 (Backend + Frontend)

배경: `CALIB_HANDEYE_START`가 reset이 아니라 "캡처 + 추가" 동작. reset하려면 백엔드 재시작 필요. SAVE가 calibrate + 파일 저장을 한 번에 → "결과 확인 후 채택/기각" 흐름 없음.

**Backend 변경** ([backend/nodes/calibration_node.py](backend/nodes/calibration_node.py)):

- 신규 서비스:
  - `CALIB_HANDEYE_RESET` — `self.hand_eye.reset()` 호출 (메서드는 이미 존재, 노출만 안 됨)
  - `CALIB_HANDEYE_COMPUTE` — `calibrate()`만 실행. 결과 + method 비교 + AX=XB 잔차 + 포즈별 잔차 반환. 파일 저장 X.
  - `CALIB_HANDEYE_COMMIT` — 마지막 COMPUTE 결과를 `hand_eye.npz`로 저장.
  - `CALIB_HANDEYE_REMOVE_POSE` — 특정 인덱스 포즈 삭제 (outlier 제거용).
  - `CALIB_HANDEYE_LIST_POSES` — 현재 누적된 포즈 메타데이터 (이미지 썸네일, joint angles, 캡처 시각) 반환.
- `HandEyeCalibration`에 잔차 계산 추가 — AX=XB에서 모든 (A_i, B_i) 페어의 reprojection residual.
- `Service` 클래스 ([backend/core/topic_map.py](backend/core/topic_map.py)) + 프론트 `ServiceKey` 양쪽 갱신.

**Frontend 변경**:

- 캘 패널의 캡처 컨트롤 / 계산 섹션 재구성 (위 UI 도식 참조).
- 포즈별 잔차 표 + 개별 삭제 버튼.
- COMPUTE 결과 미리보기 후 COMMIT 따로 누르는 흐름.

##### Step 3 — FK 흩어짐 검증 (Backend + Frontend)

배경: FK가 진짜 병목인지 숫자로 확정. 같은 (고정된) 체커보드를 여러 자세에서 본 결과 `T_base←board = T_base←ee · T_ee←cam · T_cam←board`가 자세마다 얼마나 흩어지는지 측정.

좋은 캘이면 모든 자세에서 같은 값(체커보드가 안 움직였으니까). 흩어짐 = (hand-eye 오차) + (FK 오차) 합. 같은 자세 반복 시 PnP 분산이 0.1° 안이면 → 자세 간 분산은 FK 오차.

**구현 — 별도 데이터 캡처 불필요**. Step 2의 캡처된 포즈를 그대로 재사용.

- 신규 서비스 `CALIB_HANDEYE_VALIDATE`:
  - 입력: 사용할 hand_eye (현재 캘 파일 or 최근 COMPUTE 결과)
  - 처리: 모든 포즈에 대해 `T_base←board` 계산 → 평균과의 차이 (Δrot°, Δt mm) per pose
  - 출력: `{sigma_rot_deg, sigma_t_mm, per_pose: [{idx, drot_deg, dt_mm}, ...]}`
- 프론트는 "검증" 섹션에 버튼 + 결과 표/히스토그램.

##### Step 4 — Bundle Adjustment (Backend, 핵심 정확도 향상)

배경: 1~3은 UX/진단. 실제 정확도는 BA로 깬다.

**원리**: 모든 자세의 관측을 동시에 최적화해 FK 오차도 흡수.

```
변수: joint_zero_offset[5], R_cam2gripper(3 = rodrigues), t_cam2gripper(3)
       총 11개. 선택적으로 링크 길이 보정 추가.

목적함수: Σ_pose Σ_corner ||proj(corner_world_estimated) - corner_pixel_observed||²
  - corner_world: 체커보드의 알려진 3D 격자 위치 + 추정된 board pose (board pose는 marginalize)
  - proj: K · [R|t]_cam_world · corner_world
  - 또는 더 단순: Σ_pose ||T_base←board(pose) - mean_T_base←board||² (Step 3 흩어짐 직접 최소화)

해결: scipy.optimize.least_squares(method='lm') — Levenberg-Marquardt
초기값: 현재 cv2.calibrateHandEye 결과를 seed (joint_offset은 0으로 시작)
```

**파일 배치**:

- `backend/modules/calibration/bundle_adjust.py` 신규.
- `HandEyeCalibration.calibrate(method=...)` API에 `"bundle"` 모드 추가하거나 별도 클래스.
- joint_offset 결과는 `robot/calibration/joint_offsets.npz` (신규)에 저장.
- `motor_node`가 raw→rad 변환 시 이 파일을 로드해 자동 반영. 없으면 0으로 동작 (하위 호환).
- [backend/core/units.py](backend/core/units.py)의 `raw_to_rad` 시그니처에 offset 받도록 확장 검토.

**Frontend 변경**:

- 계산 섹션의 알고리즘 드롭다운에 `bundle-adjust` 추가.
- BA 결과에 joint_offset 5개도 같이 표시.
- COMMIT 시 "joint_offset도 함께 적용하시겠습니까?" 확인 다이얼로그.

**검증 흐름**: BA 적용 후 Step 3 흩어짐 테스트가 0.5° 미만 / 5mm 미만으로 떨어지면 성공. PARK/DANIILIDIS Δrot 비교도 같이 떨어져야 진짜 개선임 (안 그러면 BA가 다른 오차로 흡수해버린 거).

#### 작업 순서 정당화

- 1 → 2 순서: 라이브 피드백이 워크플로우 개선의 일부 (자세 평가 정보가 캡처 UI에 필요).
- 2 → 3 순서: 흩어짐 검증은 Step 2의 캡처 포즈를 재사용. 별도 캡처 안 만들려면 2 먼저.
- 3 → 4 순서: BA가 정말 필요한지 3의 숫자로 확정. 만약 3에서 σ < 0.3°면 BA 안 가도 됨.

#### 미사용 옵션 (참고만)

이전에 검토했던 안들. 위 4단계가 더 직접적인 진단/개선 경로라 후순위:

- **바닥 평면 RANSAC fit** — PointCloudNode가 plane fit → Z=0 대비 각도/거리 발행. 간접 검증이고 평평한 바닥이 카메라에 항상 잡혀야 함. Step 3 흩어짐이 더 직접적.
- **멀티 자세 누적 클라우드** — 시각적으로 강력하지만 정량 수치 없음. 이미 RobotScene의 `<Grid>`와 라이브 클라우드로 정성 비교는 됨.

## 운영 메모

- 카메라 Pi / 모터 Pi: Ubuntu 22.04, uv 설치 완료
- IP: 192.168.0.101 (모터), 192.168.0.102 (카메라)
- 일반 의존성은 Pi 둘 다 `uv sync --only-group <role>` 가능
- `pyrealsense2`와 `open3d`는 aarch64 wheel 이슈 있음 — `pyrealsense2`는 카메라 Pi 소스 빌드, `open3d`는 PC만 필요하니 무관

## TODO / 다음 단계

### 캘리브레이션 (정확도 향상 4단계 — 위 § 정확도 향상 로드맵 참조)

> TSDF 적용 시 캘 정확도가 핵심. 지그 제작 불가 → 핀포인트 방식 제외. **다포즈 + Bundle Adjustment**가 정해진 경로. UI는 캘 패널 1개에 섹션 4개로 통합.

**Step 1 — 라이브 체커보드 검출 피드백** (가장 시급, UX 차단)
- Backend: CalibrationNode에 5~10Hz `_preview_loop` + `CALIB_HANDEYE_PREVIEW` 토픽 발행 (`{detected, corners, coverage_ratio, tilt_deg}`)
- Frontend: 카메라 스트림 위 코너 오버레이 + 자세 품질 힌트
- Topic/ServiceKey 양쪽 동기화

**Step 2 — 워크플로우 개선** (캡처/계산/커밋 분리)
- Backend 신규 서비스: `CALIB_HANDEYE_RESET`, `CALIB_HANDEYE_COMPUTE`, `CALIB_HANDEYE_COMMIT`, `CALIB_HANDEYE_REMOVE_POSE`, `CALIB_HANDEYE_LIST_POSES`
- `HandEyeCalibration`에 포즈별 AX=XB 잔차 계산 추가
- Frontend: 포즈 리스트 + 개별 삭제, COMPUTE 결과 미리보기 → COMMIT 분리

**Step 3 — FK 흩어짐 검증** (BA 필요성 진단)
- Step 2의 캡처 포즈 재사용. 별도 캡처 X.
- 신규 서비스 `CALIB_HANDEYE_VALIDATE` — 모든 포즈의 `T_base←board` 분산 (σ_rot, σ_t, per-pose 편차)
- Frontend "검증" 섹션 버튼 + 결과 표
- σ < 0.3° 나오면 Step 4 불필요. 그 이상이면 FK가 floor 확정 → Step 4 진행.

**Step 4 — Bundle Adjustment** (실제 정확도 향상)
- `backend/modules/calibration/bundle_adjust.py` 신규
- 변수: joint_zero_offset[5] + R/t_cam2gripper (11개). scipy.optimize.least_squares(LM)
- 초기값: cv2.calibrateHandEye 결과를 seed
- joint_offset은 `robot/calibration/joint_offsets.npz`에 저장. `motor_node`가 raw→rad 시 자동 반영
- `units.raw_to_rad` 시그니처에 offset 추가 검토
- Frontend 계산 드롭다운에 `bundle-adjust` 모드 추가, COMMIT 시 joint_offset 적용 확인 다이얼로그
- 목표: PARK Δrot < 1°, Step 3 흩어짐 σ_rot < 0.5° / σ_t < 5mm

### D405 마이그레이션 잔여 phase

- **Phase 4 — PLY 스냅샷 캡처/로드.** `depth_frame` 페이로드 그대로 디스크 저장 (PLY + npz with TCP pose) + Open3D PLY 변환. `POINTCLOUD_SNAPSHOT` 토픽 자리만 예약돼 있음.
- **Phase 5 — ICP 등록.** PointCloudNode 서비스로.
- **Phase 6 — Detector depth lookup 전환.** 현재의 평면-Z=0 역산 자리에 depth 직접 lookup.

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
