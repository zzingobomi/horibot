# CLAUDE.md

이 파일은 Claude Code(claude.ai/code)가 이 저장소에서 작업할 때 참고하는 가이드입니다.

## 프로젝트 개요

OMX Control — **OMX_F**(OpenMANIPULATOR-X 커스텀 변형) 6DOF 로봇팔 제어 스택. 백엔드는 Dynamixel 서보를 구동하고, 카메라 + YOLO 디텍션 + Hand-Eye 캘리브레이션을 실행하며, Ruckig으로 trajectory를 계획하고, PyBullet으로 [robot/urdf/omx_f/](robot/urdf/omx_f/) URDF에 대한 FK/IK를 푼다. 프론트엔드는 teleop / 캘리브레이션 / 3D 디지털 트윈 워크스페이스를 제공하는 React 앱.

D405 RGBD가 한 메시지로 묶여 LAN에 흐르고, PC가 구독해 Open3D로 (a) 라이브 포인트클라우드 발행 + (b) 다중 자세 캡처 → TSDF mesh 빌드까지 처리한다 (아키텍처 § D405 파이프라인).

세부 주제별 문서는 [docs/](docs/) 디렉토리:
- [hardware.md](docs/hardware.md) — 모터/컨트롤러/전원 토폴로지
- [operations.md](docs/operations.md) — Pi/IP/OS, pyrealsense2 빌드 노트는 [pyrealsense2-build-guide.md](docs/pyrealsense2-build-guide.md)
- [calibration_workflow.md](docs/calibration_workflow.md) — 캡처 절차 + 결과 해석 가이드
- [calibration_apply_flow.md](docs/calibration_apply_flow.md) — 4종 캘 산출물의 적용 메커니즘
- [hand_eye_extended_ba.md](docs/hand_eye_extended_ba.md) — 확장 BA + 물리 sag 모델 (σ_rot 0.65°/σ_t 7.94mm 도달기)
- [tsdf_pipeline.md](docs/tsdf_pipeline.md) — multi-way ICP + TSDF mesh 빌드 결정사항
- [self_play_pick.md](docs/self_play_pick.md) — self-play pick 루프 설계 (active, WIP)
- [roadmap.md](docs/roadmap.md) — 진행 중/예정 작업

## 자주 쓰는 명령어

### Backend (Python 3.11, uv 관리, [backend/](backend/)에서 실행)

```powershell
cd backend
uv sync                  # PC 개발 환경 (default-groups = dev + all)
uv run python main.py    # hostname 자동 감지 → 매칭 실패 시 host_dev.yaml
uv run ruff check .      # 린트
uv run pyright           # 타입 체크
```

실행 모드:

```powershell
# 단일 머신 풀스택
uv sync
uv run python main.py                    # --host 미지정 → host_dev.yaml fallback

# 분산: PC 역할
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

분산 모터 컨트롤러는 OpenRB-150이며 USB CDC-ACM(`/dev/ttyACM*`), 기본 포트는 Windows `COM6` / Linux `/dev/ttyACM0` ([robot/config/motors.yaml](robot/config/motors.yaml)). 자세한 사양은 [docs/hardware.md](docs/hardware.md).

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
| PC              | detector, task, pointcloud, calibration, bridge, (gamepad) | YOLO, 포인트클라우드/TSDF 가공, 브릿지     |
| 모터 Pi (101)   | motor, motion                                              | Dynamixel + Ruckig + IK (제어 루프 로컬화) |
| 카메라 Pi (102) | camera                                                     | D405 캡처 + JPEG + 압축 depth 발행         |

이 분배의 핵심 이유:

- (a) USB 대역폭 경합 해소 — D405와 OpenRB-150이 한 USB 컨트롤러를 공유하지 않음
- (b) 100Hz 제어 명령(`MOTOR_CMD_JOINT`)을 네트워크로 안 보냄 — TrajectoryRunner와 MotorNode 같은 머신
- (c) 무거운 연산(YOLO, Open3D, PyBullet PC 측, TSDF build)은 PC

### 2-layer 트랜스포트: Zenoh (백엔드 내부) + WebSocket (브라우저)

백엔드 노드는 모두 **Zenoh** pub/sub + queryable로 통신. 프로세스당 하나의 `ZenohSession` 싱글톤 ([backend/core/zenoh_session.py](backend/core/zenoh_session.py))을 모든 노드가 `ZenohSession.get()`으로 공유. 토픽 페이로드는 보통 JSON, 카메라 JPEG / depth_frame / 포인트클라우드는 raw 바이너리. 서비스는 `{success, message, data}` 응답 봉투.

분산 시 각 머신이 독립 peer로 동작 — 같은 LAN이면 멀티캐스트 scout으로 자동 발견. 명시적 endpoint가 필요하면 host config의 `zenoh.connect`에 적음.

브라우저는 Zenoh를 못 하니까 [backend/bridge/zenoh_bridge.py](backend/bridge/zenoh_bridge.py)의 FastAPI가:

- `/ws` — WebSocket: `subscribe` / `unsubscribe` / `publish` / `service` 메시지 ↔ Zenoh 양방향 변환.
- `/camera/stream` — MJPEG `multipart/x-mixed-replace` 스트림, `omx/camera/stream/raw` Zenoh 토픽 소스.
- [robot/](robot/)를 `/robot`에 정적 마운트 (URDF, mesh, calibration .npz, TSDF .ply).
- "코어" 토픽을 미리 구독해 WS 클라이언트들에 재방송 (`_ALWAYS_SUBSCRIBE`).

JSON 토픽은 `{type:"topic_data", topic, data}`로 텍스트 송신. **바이너리 토픽**(카메라 raw 제외 — 그건 MJPEG로 별도 라우트)은 다음 프레이밍으로 binary WS 송신:

```
[u8 version=1][u8 type=1 (topic_data)][u16 BE topic_len][UTF-8 topic][payload]
```

현재 이 바이너리 프레이밍을 쓰는 토픽은 `omx/pointcloud/stream`. 프론트에서 동일 디코더로 풀어 React-Three-Fiber에 먹임.

#### 클라이언트별 송신 큐 + 백프레셔 ([backend/bridge/client_stream.py](backend/bridge/client_stream.py))

각 (WS 클라이언트, 구독 토픽) 짝마다 별도 `ClientStream` (bounded `asyncio.Queue` + sender task) — 느린 브라우저 한 명 때문에 메모리가 폭증하지 않게.

토픽별 정책:
- 기본: `LATEST_WINS` — 큐 크기 1, 새 값이 옛 값을 덮어씀.
- `SYSTEM_LOG`: `BOUNDED_FIFO`(128) — 로그는 일정량 보존.

프론트엔드는 `BridgeClient` 싱글톤 ([frontend/src/api/bridge.ts](frontend/src/api/bridge.ts))으로 감싸고 `ReconnectingWebSocket`, 토픽별 멀티플렉싱, `request_id` 기반 service promise를 처리. `useBridge` ([frontend/src/hooks/useBridge.ts](frontend/src/hooks/useBridge.ts))가 `App.tsx`에서 한 번 마운트되어 들어오는 데이터를 Zustand store로 라우팅.

### 토픽/서비스 레지스트리 — 두 곳에서 동기화

토픽/서비스 키는 **두 군데**에 선언됨: [backend/core/topic_map.py](backend/core/topic_map.py)의 `Topic`/`Service` 클래스와 [frontend/src/constants/topics.ts](frontend/src/constants/topics.ts)의 `Topic`/`ServiceKey`. 추가/변경 시 두 파일 같이 수정, 문자열 정확히 일치 (예: `omx/motion/srv/move_l`). 프론트에서 사용하지 않는 내부 토픽/서비스(예: `CAMERA_DEPTH_FRAME`, `MOTOR_GRIPPER`, `CAMERA_SET_DEPTH_STREAM`, `DETECT_SERVICE`)는 프론트 쪽 미러링 생략.

### 노드 패턴 + 노드 레지스트리 (lazy import)

모든 노드는 `BaseNode` ([backend/core/base_node.py](backend/core/base_node.py))를 상속:

- `create_subscriber(topic, callback)` — JSON 디코드된 pub/sub
- `create_raw_subscriber(topic, callback)` — JSON 디코드 없는 binary 페이로드
- `create_service(key, handler)` — Zenoh queryable 등록, 핸들러는 `{success, message, data}` 반환
- `call_service(key, data, timeout=5.0)` — 기본 5초 타임아웃 동기 호출
- `publish(topic, data)`, `log(level, msg)`, 1Hz heartbeat `omx/system/heartbeat`

라이프사이클: `start()` → heartbeat 스레드 + 노드별 워커 스레드 → `stop()`이 모든 subscriber/queryable undeclare.

[backend/core/node_registry.py](backend/core/node_registry.py)가 노드 이름 → `(모듈, 클래스)` 매핑을 **문자열로** 유지. `importlib`로 호출 시점에만 import → 모터 Pi가 `node_registry`를 import해도 open3d/pyrealsense2/ultralytics 등 PC 전용 의존성이 import 트리에 안 끌려옴. 등록된 노드: `motor / camera / motion / calibration / task / detector / pointcloud / gamepad`.

### 호스트 config (config-driven main.py)

[backend/main.py](backend/main.py)는 `--host` 인자(미지정 시 hostname을 lowercase/`-`→`_` 정규화 후 `host_<hostname>.yaml` 매칭 → 매칭 실패 시 `host_dev.yaml` fallback)로 [backend/config/](backend/config/)의 YAML을 로드. main.py는 `nodes`에 명시된 것만 lazy-import + 시작. `bridge.enabled`로 FastAPI/uvicorn 켜기. `camera` 노드가 활성일 때만 D405 factory intrinsic seed가 실행됨 (`seed_d405_intrinsic_if_missing`).

### 의존성 그룹 (`pyproject.toml`)

PEP 735 `[dependency-groups]`로 역할별 분리:

- 공통(`[project]`): `eclipse-zenoh`, `numpy`, `pyyaml`, `python-dotenv`
- `dev`: `ruff`, `pyright`
- `pi-motor`: `dynamixel-sdk`, `pybullet`, `ruckig`, `scipy`
- `pi-camera`: `pyrealsense2`, `opencv-python`, `zstandard`
- `pc`: `opencv-python`, fastapi 스택(`fastapi`/`uvicorn`/`websockets`), `pybullet`, `scipy`, `ultralytics`, `open3d`, `pygame`, `zstandard`
- `all`: 위 세 역할 그룹 include (개발용 한 머신 풀스택)

`tool.uv.default-groups = ["dev", "all"]` — PC 개발 환경에선 `uv sync`만으로 충분, Pi에서는 `--only-group <role>` 명시.

### 노드 간 공유 싱글톤들

- `ZenohSession` — 프로세스당 하나의 Zenoh 세션.
- `JointStateCache` ([backend/core/joint_state_cache.py](backend/core/joint_state_cache.py)) — `MOTOR_STATE_JOINT`를 한 번만 구독, `get_joint_angles_rad(arm_cfgs)`로 라디안 단위 최신 조인트각 노출 (raw→rad 시 joint_offset 자동 적용 — § 캘리브레이션 적용). motion/task/detector/calibration/pointcloud가 공유.
- `FrameCache` ([backend/core/frame_cache.py](backend/core/frame_cache.py)) — `CAMERA_STREAM_RAW`(JPEG) + `CAMERA_STATE_STATUS` 구독, `get_frame()`이 BGR ndarray 반환. detector/calibration이 토픽 기반으로 동작 → 카메라가 다른 머신에 있어도 동일 코드.
- `RealsenseCapture` ([backend/core/realsense_capture.py](backend/core/realsense_capture.py)) — pyrealsense2 파이프라인 1개를 공유. **카메라 호스트에서만 살아 있음**.
- `PybulletSolver` ([backend/modules/kinematics/solver.py](backend/modules/kinematics/solver.py)) — DIRECT 모드 PyBullet, thread-safe `fk()` / `ik()` / `fk_to_matrix()`. EE 링크 `end_effector_link`. **link_offset 패치된 URDF**를 로드하고 `fk`/`ik`에서 **sag** 보정을 양방향 적용 ([docs/calibration_apply_flow.md](docs/calibration_apply_flow.md)).
- `*Coordinates` 싱글톤 — `JointCoordinates` / `LinkCoordinates` / `SagCoordinates` ([backend/core/](backend/core/)). 각각 npz 1회 로드 후 메모리 캐시. raw↔rad / URDF patch / sag stiffness를 노출.

### Motion 파이프라인

`MotionNode` ([backend/nodes/motion_node.py](backend/nodes/motion_node.py))가 `move_j` / `move_l` / `move_c` / `move_p` / `move_tcp` 서비스 수신. 검증/실행은 `MotionCommand` 서브클래스로 분리. 실제 보간은 `TrajectoryRunner` ([backend/modules/kinematics/trajectory_runner.py](backend/modules/kinematics/trajectory_runner.py))가 **Ruckig** jerk-limited 프로파일로 처리하고 `omx/motion/state/trajectory`에 진행 발행. 조인트 명령은 `publish_cmd` 콜백 → `MOTOR_CMD_JOINT` 토픽 (urdf→raw 변환 시 joint_offset 자동 차감).

**MotionNode와 MotorNode는 같은 머신(모터 Pi)에 배치** — TrajectoryRunner가 100Hz로 publish하는 명령이 네트워크를 넘지 않게 해야 trajectory 끊김/지터를 막을 수 있음. PyBullet IK도 같은 머신.

아암은 운동학적으로 **5DOF** (모터 ID 1–5), ID 6은 그리퍼로 `core.common.GRIPPER_ID`로 필터링. 단위 변환은 [backend/core/units.py](backend/core/units.py) — Dynamixel raw는 `0..4095`, 중심 `2048`(=0°).

### Task 시스템

Task는 선언형 step 리스트 ([backend/modules/task/step_types.py](backend/modules/task/step_types.py): `MoveTCPStep`, `DetectStep`, `GripperStep`, `HomeStep`, `WaitStep`). `TASK_REGISTRY`는 [backend/nodes/task_node.py](backend/nodes/task_node.py)에. `StepExecutor`가 각 step 실행(motion/detector/motor 서비스 호출), `TaskRunner`가 상태 머신(`run/pause/resume/stop`), 진행은 `omx/task/state`로 발행. `DetectStep`은 결과를 context dict(`output_key`)에 쓰고, 이후 `MoveTCPStep`이 `position_key` + `offset`으로 소비 — [backend/modules/task/tasks/pick_and_place.py](backend/modules/task/tasks/pick_and_place.py)가 정규 예시.

### Detector → 월드 좌표 파이프라인

`DetectorNode._handle_detect` ([backend/nodes/detector_node.py](backend/nodes/detector_node.py)) 체인: `FrameCache.get_frame()` → YOLO centroid → `cv2.undistortPoints`로 intrinsic 보정 → `MOTION_GET_TCP`로 현재 EE 포즈 (joint/link/sag 보정 내장) → base-frame 평면 `Z=0` 제약으로 `Z_cam` 역산 → hand_eye matrix 곱해서 베이스 프레임의 물체 위치 반환. `intrinsic.npz`와 `hand_eye.npz` 둘 다 필요 — `load_calibration().is_ready()`가 서비스 가드. 별도 5fps `_detection_loop`가 `DETECTOR_STATE`에 raw detection을 stream으로 발행.

### D405 → 라이브 PointCloud + TSDF 파이프라인

```
CameraNode (카메라 Pi)
  ├─ 30 FPS color JPEG → omx/camera/stream/raw           (항상 켜짐)
  └─ 8 FPS depth_frame → omx/camera/stream/depth_frame   (CAMERA_SET_DEPTH_STREAM enable 시)
       페이로드 (modules/camera/depth_frame.py, little-endian):
         [u32 header_len][JSON header][u32 jpeg_len][aligned color JPEG][zstd Z16 depth]
       header: timestamp / width / height / depth_scale / fx fy cx cy
               / depth_uncompressed_bytes (검증용)

PointCloudNode (PC) — [backend/nodes/pointcloud_node.py](backend/nodes/pointcloud_node.py)
  ← CAMERA_DEPTH_FRAME 구독 (raw subscriber)
  ─── 라이브: depth_frame → Open3D RGBDImage → PointCloud → voxel_down_sample
         → omx/pointcloud/stream (raw binary: [u32 n][n*3 float32 xyz][n*3 uint8 rgb])
  ─── 캡처: POINTCLOUD_CAPTURE 호출 시 다중 frame consensus → raw motor positions와 함께
         scan_<id>.npz로 저장 ([backend/modules/pointcloud/scan_io.py](backend/modules/pointcloud/scan_io.py))
  ─── TSDF build: POINTCLOUD_BUILD_MESH 호출 시 다중 scan에 point-to-plane ICP +
         multi-way PoseGraph optimization + ScalableTSDFVolume 적용
         → robot/models/mesh_<session>.ply ([backend/modules/pointcloud/tsdf_builder.py](backend/modules/pointcloud/tsdf_builder.py))

프론트엔드 (브릿지가 binary WS 프레이밍으로 중계)
  → POINTCLOUD_CONFIGURE {enabled?, voxel_size?}  ─ 라이브 스트림 on/off + 다운샘플
  → POINTCLOUD_NEW_SESSION / _CAPTURE / _LIST_SCANS / _DELETE_SCAN  ─ 세션/scan 관리
  → POINTCLOUD_BUILD_MESH / _LIST_MESHES  ─ TSDF mesh
  ← POINTCLOUD_STATE {enabled, voxel_size}
```

핵심 설계:
- **Depth는 무손실(zstd)** — ICP/TSDF 정밀도 보존. Color만 JPEG 손실.
- **scan은 raw motor positions로 저장** — 캡처 시점 캘이 변해도 raw는 불변. build 단계가 *현재 캘*로 freshly 재계산.
- **scan_id는 monotonic** (재사용 X) — 캡처/삭제 반복해도 인덱스 시프트 없음. 캘 안정 ID와 같은 패턴.
- TSDF 빌드 결정사항(ICP 종류, voxel size 등)은 [docs/tsdf_pipeline.md](docs/tsdf_pipeline.md).

라이브 포인트클라우드는 백엔드에서 camera-frame xyz를 그대로 publish하고, 프론트 [PointCloudLayer.tsx](frontend/src/components/workspace3d/3d/PointCloudLayer.tsx)가 `<group position quaternion>` 부모 transform으로 `cameraMatrix = tcpMatrix · handEyeMatrix`를 적용. TSDF mesh는 이미 base 프레임이므로 [MeshLayer.tsx](frontend/src/components/workspace3d/3d/MeshLayer.tsx)가 추가 transform 없이 그대로 마운트.

### 캘리브레이션 — 4종 산출물 + intrinsic

다섯 가지 npz가 `robot/calibration/`에 있음. 각각 적용 메커니즘이 다름:

| 산출물       | 무엇을 보정         | 어디서 적용                                | COMMIT 후                     |
| ------------ | ------------------- | ------------------------------------------ | ----------------------------- |
| intrinsic    | D405 카메라 내부    | `cv2.undistortPoints` (Detector)           | DetectorNode 재시작           |
| hand_eye     | 카메라 ↔ EE 변환    | Detector 후처리 + 프론트 PointCloudLayer   | DetectorNode 재시작           |
| joint_offset | 모터 raw zero 오차  | `motor_to_urdf` / `urdf_to_motor` (raw↔rad 변환 양쪽) | 즉시              |
| link_offset  | URDF 링크 기하 오차 | **URDF 자체를 patch**해서 PyBullet 로드    | 백엔드 재시작 (PyBullet 1회 로드) |
| sag_offset   | J2/J3 자세 의존 중력 처짐 | `PybulletSolver.fk`/`ik` 양방향 적용  | 즉시 (`_reload_sag_cache`)    |

확장 BA + 물리 sag 모델로 현재 σ_rot **0.65°** / σ_t **7.94mm** ([docs/hand_eye_extended_ba.md](docs/hand_eye_extended_ba.md)). TSDF GOOD threshold(σ_rot <1°, σ_t <10mm) 안. 산출물별 코드 흐름 + COMMIT 후 어디까지 자동 반영되는지는 [docs/calibration_apply_flow.md](docs/calibration_apply_flow.md), 캘 절차/UI 사용법은 [docs/calibration_workflow.md](docs/calibration_workflow.md).

### Frontend stores & 3D 워크스페이스

상태는 [frontend/src/store/](frontend/src/store/)의 Zustand store로 분리 (`robotStore`, `cameraStore`, `motionStore`, `taskStore`, `detectorStore`, `systemStore`, `sceneStore`, `pointCloudStore`). `Workspace3D` 페이지 ([frontend/src/pages/Workspace3D.tsx](frontend/src/pages/Workspace3D.tsx))는 `dockview` 플로팅 패널 위에 `react-three-fiber` 씬 + `urdf-loader` — 패널은 [frontend/src/components/workspace3d/dockview/panelComponents.ts](frontend/src/components/workspace3d/dockview/panelComponents.ts)에 등록. URDF에 들어가는 조인트각은 `MOTOR_STATE_JOINT`에서 `(position - 2048) / 4095 * 2π` 형태로 도출 (`units.raw_to_rad`와 일치).

## 규약

- 로그 메시지, 주석, docstring은 한국어 자유롭게 — 주변 코드의 스타일을 유지.
- Backend는 **ruff**(line-length 88, target py311) + **pyright**, Frontend는 ESLint + Prettier + `editor.formatOnSave` (VS Code).
- 프론트엔드 import는 `@/`alias = [frontend/src/](frontend/src/) ([frontend/vite.config.ts](frontend/vite.config.ts)).
- 서비스 핸들러는 반드시 `{"success": bool, "message": str, "data": dict}` 반환 — 브릿지와 `BridgeClient.callService`가 이 모양에 의존.
- 모델 가중치(`*.pt`, `*.pth`), `.venv/`, `node_modules/`, `frontend/dist/`, `uv.lock`, patched URDF(`robot/urdf/omx_f/.patched/`)는 gitignore.
