# CLAUDE.md

이 파일은 Claude Code(claude.ai/code)가 이 저장소에서 작업할 때 참고하는 가이드입니다.

## 프로젝트 개요

Horibot — **OMX_F**(OpenMANIPULATOR-X 커스텀 변형) 6DOF 로봇팔 제어 스택. 백엔드는 Dynamixel 서보를 구동하고, 카메라 + YOLO 디텍션 + Hand-Eye 캘리브레이션을 실행하며, Ruckig으로 trajectory를 계획하고, PyBullet으로 [robot/omx_f/urdf/](robot/omx_f/urdf/) URDF에 대한 FK/IK를 푼다. 프론트엔드는 teleop / 캘리브레이션 / 3D 디지털 트윈 워크스페이스를 제공하는 React 앱.

**Multi-robot 일반화 진행 중** ([docs/multi_robot_architecture.md](docs/multi_robot_architecture.md)) — robot 데이터는 type 폴더 (`robot/<robot_type>/`) + instance 폴더 (`robot/instances/<robot_id>/`) 분리. `robot/robots.yaml` 이 registry. 현재 N=1 (omx_f_0). SO-101 도착 시 entry + FeetechBackend adapter 추가 = plug-and-play (자세한 status 는 multi_robot_architecture.md §12 Phase 1).

D405 RGBD가 한 메시지로 묶여 LAN에 흐르고, PC가 구독해 Open3D로 (a) 라이브 포인트클라우드 발행 + (b) 다중 자세 캡처 → TSDF mesh 빌드까지 처리한다 (아키텍처 § D405 파이프라인).

세부 주제별 문서는 [docs/](docs/) 디렉토리:
- [hardware.md](docs/hardware.md) — 모터/컨트롤러/전원 토폴로지
- [operations.md](docs/operations.md) — Pi/IP/OS, pyrealsense2 빌드 노트는 [pyrealsense2-build-guide.md](docs/pyrealsense2-build-guide.md)
- [calibration_workflow.md](docs/calibration_workflow.md) — 캡처 절차 + 결과 해석 가이드
- [calibration_apply_flow.md](docs/calibration_apply_flow.md) — 4종 캘 산출물의 적용 메커니즘
- [hand_eye_extended_ba.md](docs/hand_eye_extended_ba.md) — 확장 BA + 물리 sag 모델 (σ_rot 0.65°/σ_t 7.94mm 도달기)
- [tsdf_pipeline.md](docs/tsdf_pipeline.md) — multi-way ICP + TSDF mesh 빌드 결정사항
- [step_dsl.md](docs/step_dsl.md) — typed Slot 기반 lego Step DSL (Step/Slot/StepContext/Recipe + 다이어그램 + 확장 가이드)
- [random_palletizing.md](docs/random_palletizing.md) — 사이즈 가변 직육면체 팔레타이징 design (3-track: 휴리스틱 / 정석 / iterative sim2real RL)
- [so101_6dof_plan.md](docs/so101_6dof_plan.md) — SO-101 6DOF 두 번째 로봇 하드웨어 plan (모터 SDK 추상화 / wrist yaw mod / D405 마운트)
- [multi_robot_architecture.md](docs/multi_robot_architecture.md) — multi-robot platform 업그레이드 design (Adapter/Strategy/DIP 패턴 layer / robot identity / 토픽 namespace 재설계 / Coordinator / 마이그레이션 phase)
- [multi_robot_walkthrough.md](docs/multi_robot_walkthrough.md) — Phase 1 (foundation) 산출물 + 클래스/시퀀스 다이어그램 + Phase 2 남은 작업 follow-up 가이드. **코드 읽으며 학습할 때 anchor**
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
uv sync                                  # default-groups (dev + all) 다 받음
uv run --no-sync python main.py --host pc

# 분산: 모터 Pi (192.168.0.101)
uv sync --no-default-groups --group pi-motor
uv run --no-sync python main.py --host pi_motor

# 분산: 카메라 Pi (192.168.0.102)
# pyrealsense2는 사전 소스 빌드 후 별도 install
uv sync --no-default-groups --group pi-camera --no-install-package pyrealsense2
uv pip install ~/pyrealsense2-2.55.1-cp311-cp311-linux_aarch64.whl
uv run --no-sync python main.py --host pi_camera
```

호스트 config 파일들 ([backend/config/](backend/config/)):

- `host_dev.yaml` — 단일 머신 풀스택 (motor/camera/motion/calibration/task/detector/pointcloud + 브릿지; gamepad는 미포함)
- `host_pc.yaml` — 분산 PC (calibration/task/detector/pointcloud + 브릿지; motor/motion/camera 없음)
- `host_pi_motor.yaml` — 분산 모터 Pi (motor/motion)
- `host_pi_camera.yaml` — 분산 카메라 Pi (camera)

분산 모터 컨트롤러는 OpenRB-150이며 USB CDC-ACM(`/dev/ttyACM*`), 기본 포트는 Windows `COM6` / Linux `/dev/ttyACM0` ([robot/instances/omx_f_0/instance.yaml](robot/instances/omx_f_0/instance.yaml) — type-level 모터 spec 은 [robot/omx_f/motors.yaml](robot/omx_f/motors.yaml)). 자세한 사양은 [docs/hardware.md](docs/hardware.md).

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
- `CameraCapture` ([backend/modules/camera/capture.py](backend/modules/camera/capture.py)) — `CameraCaptureProtocol` 만족하는 RealSense wrapper (내부 `RealsenseCapture` 싱글톤). pyrealsense2 파이프라인 1개 공유. **카메라 호스트에서만 살아 있음**. multi_robot_architecture.md §3.4.
- `PybulletSolver()` ([backend/modules/kinematics/solver.py](backend/modules/kinematics/solver.py)) — facade. 내부적으로 `RobotRegistry().get_iksolver(default)` 호출 → `CorrectedIKSolver(PybulletIKSolver(urdf), link, sag)` 체인 반환. `IKSolver` Protocol 만족 (`fk` / `ik` / `fk_to_matrix` / `joint_limits` / `dof` / `ee_link_name`). **link_offset 패치된 URDF** 는 `PybulletIKSolver` 가 로드, **sag** 보정은 `CorrectedIKSolver` Decorator 가 양방향 적용 ([docs/calibration_apply_flow.md](docs/calibration_apply_flow.md), [docs/multi_robot_architecture.md](docs/multi_robot_architecture.md) §3.1-3.2).
- `RobotRegistry` ([backend/core/robot_registry.py](backend/core/robot_registry.py)) — `robot/robots.yaml` 의 single source of truth. `get(robot_id)` 로 `RobotConfig` (모든 path / 설정), `get_iksolver(robot_id)` / `get_motor_backend(robot_id)` factory (per-robot 인스턴스 캐시). `default()` / `default_robot_id()` 가 N=1 편의.
- `*Coordinates` 싱글톤 — `JointCoordinates` / `LinkCoordinates` / `SagCoordinates` ([backend/core/](backend/core/)). 각각 npz 1회 로드 후 메모리 캐시. raw↔rad / URDF patch / sag stiffness를 노출.

### Motion 파이프라인

`MotionNode` ([backend/nodes/motion_node.py](backend/nodes/motion_node.py))가 `move_j` / `move_l` / `move_c` / `move_p` / `move_tcp` 서비스 수신. 검증/실행은 `MotionCommand` 서브클래스로 분리. 실제 보간은 `TrajectoryRunner` ([backend/modules/kinematics/trajectory_runner.py](backend/modules/kinematics/trajectory_runner.py))가 **Ruckig** jerk-limited 프로파일로 처리하고 `omx/motion/state/trajectory`에 진행 발행. 조인트 명령은 `publish_cmd` 콜백 → `MOTOR_CMD_JOINT` 토픽 (urdf→raw 변환 시 joint_offset 자동 차감).

**MotionNode와 MotorNode는 같은 머신(모터 Pi)에 배치** — TrajectoryRunner가 100Hz로 publish하는 명령이 네트워크를 넘지 않게 해야 trajectory 끊김/지터를 막을 수 있음. PyBullet IK도 같은 머신.

아암은 운동학적으로 **5DOF** (모터 ID 1–5), ID 6은 그리퍼로 `core.common.GRIPPER_ID`로 필터링. 단위 변환은 [backend/core/units.py](backend/core/units.py) — Dynamixel raw는 `0..4095`, 중심 `2048`(=0°).

### Task 시스템 — typed Slot lego DSL

Task는 선언형 step 리스트. 각 step 은 `Step[T_out]` 상속한 dataclass + `execute(ctx)` 메서드 보유 ([backend/modules/task/step.py](backend/modules/task/step.py)). step 간 데이터 전달은 string key 가 아니라 **typed `Slot[T]`** reference — 한 step 의 출력을 다음 step 의 인자에 직접 넘김:

```python
pick_steps, pick_slot = search_and_detect("cube")   # → Slot[Detection]
grasp = GraspPolicy(target=pick_slot)               # → grasp.out: Slot[Position3]
MoveTCP(target=grasp.out, offset=Position3(0, 0, 0.06))
```

코드 구성 ([backend/modules/task/](backend/modules/task/)):

- `schema.py` — typed value classes (`Position3`/`Pose6`/`Detection`) + `Slot[T]` (covariant frozen) + `StepResult`
- `step.py` — `Step[T_out]` base + `StepContext` (`resolve` / `run_child` / `call_motion`) + `Task` + `task_tree` (재귀 직렬화) + `collect_step_ids`
- `steps.py` — primitive 8개 (Wait/MoveJByName/MoveTCP/Gripper/VerifyGrasp/GroundedDetect/GraspPolicy/PlacePolicy) + control flow 3개 (ForEach/BreakIf/Try)
- `recipes.py` — `home()` / `search_and_detect()` 같은 primitive 조합 단축형 함수
- `task_runner.py` — `_execute_one_step` 단일 진입점 (디버거 게이트 + status + step_result publish). ForEach/Try 가 `ctx.run_child(child)` 호출 시 재진입 → nested step 도 동일 인프라
- `tasks/pick_and_place.py` — 정규 예시 + lego acceptance test

토픽:
- `omx/task/tree` — task 시작/preview 시 전체 step tree (children 재귀) 1회
- `omx/task/state` — RUNNING/PAUSED/SUCCESS/FAILED + step_statuses + current_step_id
- `omx/task/step_result` — 각 step 완료 시 `{step_id, type, value}`. frontend `TaskResultLayer` 가 type 별 자동 렌더링 (Detection→sphere, Position3→marker)

상세 (Step/Slot/StepContext 작동, ForEach unroll, 새 step / task 짜는 법, lego test 통과 근거) 는 [docs/step_dsl.md](docs/step_dsl.md).

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
         → robot/instances/<robot_id>/meshes/mesh_<session>.ply ([backend/modules/pointcloud/tsdf_builder.py](backend/modules/pointcloud/tsdf_builder.py))

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

다섯 가지 npz가 `robot/instances/<robot_id>/calibration/` 에 있음 (instance 별 분리 — multi_robot_architecture.md §5.1). 각각 적용 메커니즘이 다름:

| 산출물       | 무엇을 보정         | 어디서 적용                                | COMMIT 후                     |
| ------------ | ------------------- | ------------------------------------------ | ----------------------------- |
| intrinsic    | D405 카메라 내부    | `cv2.undistortPoints` (Detector)           | DetectorNode 재시작           |
| hand_eye     | 카메라 ↔ EE 변환    | Detector 후처리 + 프론트 PointCloudLayer   | DetectorNode 재시작           |
| joint_offset | 모터 raw zero 오차  | `JointCoordinates.motor_to_urdf` / `urdf_to_motor` (raw↔rad 변환 양쪽) | 즉시              |
| link_offset  | URDF 링크 기하 오차 | **URDF 자체를 patch**해서 PyBullet 로드    | 백엔드 재시작 (PyBullet 1회 로드) |
| sag_offset   | J2/J3 자세 의존 중력 처짐 | `CorrectedIKSolver.fk`/`ik` 양방향 적용 (Decorator) | 즉시 (`_reload_caches`) |

확장 BA + 물리 sag 모델로 현재 σ_rot **0.65°** / σ_t **7.94mm** ([docs/hand_eye_extended_ba.md](docs/hand_eye_extended_ba.md)). TSDF GOOD threshold(σ_rot <1°, σ_t <10mm) 안. 산출물별 코드 흐름 + COMMIT 후 어디까지 자동 반영되는지는 [docs/calibration_apply_flow.md](docs/calibration_apply_flow.md), 캘 절차/UI 사용법은 [docs/calibration_workflow.md](docs/calibration_workflow.md).

### Frontend stores & 3D 워크스페이스

상태는 [frontend/src/store/](frontend/src/store/)의 Zustand store로 분리 (`robotStore`, `cameraStore`, `motionStore`, `taskStore`, `taskResultStore`, `detectorStore`, `systemStore`, `sceneStore`, `pointCloudStore`). `Workspace3D` 페이지 ([frontend/src/pages/Workspace3D.tsx](frontend/src/pages/Workspace3D.tsx))는 `dockview` 플로팅 패널 위에 `react-three-fiber` 씬 + `urdf-loader` — 패널은 [frontend/src/components/workspace3d/dockview/panelComponents.ts](frontend/src/components/workspace3d/dockview/panelComponents.ts)에 등록. URDF에 들어가는 조인트각은 `MOTOR_STATE_JOINT`에서 `(position - 2048) / 4095 * 2π` 형태로 도출 (`units.raw_to_rad`와 일치).

Task step 결과 시각화는 `taskResultStore` 가 `TASK_STEP_RESULT` 누적 → [TaskResultLayer.tsx](frontend/src/components/canvas/3d/TaskResultLayer.tsx) 가 type 별 dispatch (Detection→sphere, Position3→marker). 새 typed value 추가 시 layer 한 줄 추가로 끝 — task 코드는 안 건드림.

## 규약

- 로그 메시지, 주석, docstring은 한국어 자유롭게 — 주변 코드의 스타일을 유지.
- Backend는 **ruff**(line-length 88, target py311) + **pyright**, Frontend는 ESLint + Prettier + `editor.formatOnSave` (VS Code).
- 프론트엔드 import는 `@/`alias = [frontend/src/](frontend/src/) ([frontend/vite.config.ts](frontend/vite.config.ts)).
- 서비스 핸들러는 반드시 `{"success": bool, "message": str, "data": dict}` 반환 — 브릿지와 `BridgeClient.callService`가 이 모양에 의존.
- 모델 가중치(`*.pt`, `*.pth`), `.venv/`, `node_modules/`, `frontend/dist/`, `uv.lock`, patched URDF(`robot/<robot_type>/urdf/.patched/`), per-instance 런타임 산출물(`robot/instances/*/scans/`, `robot/instances/*/meshes/`, `robot/instances/*/logs/`) 는 gitignore.

### 프로젝트 design decision (다른 PC / 새 세션이 알아야 할 critical context)

- **self-play 는 폐기됨** — `backend/modules/task/self_play/`, `self_play_pick.py`, `docs/self_play_pick.md` 는 legacy. 본 방향은 pick_and_place + deterministic IK + 캘/자세 정확도 직접 강화. self-play 점프 제안 / 신규 기능 추가 금지
- **Study task 에선 industry standard 도구 / 플로우 우선** — RL / 실험 / 시뮬레이션 도구 추천 시 인프라 재사용 ROI 보다 산업 표준 (MuJoCo / Isaac / PPO / Stable-Baselines3 등) 우선. 결과 잘 만들기보다 "표준 단계 다 밟아보기" 자체가 study output. 비유: "회사에선 React 쓰는데 사이드 프로젝트 간단하다고 HTML 만 짜면 학습 효과 없잖아"
- **DSL "레고화" = typed Slot 기반, visual editor (Blockly 등) X** — [docs/step_dsl.md](docs/step_dsl.md) 참조. 코드 조립성 가져오기, UI 도구 도입 X
