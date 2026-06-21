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
- [hand_eye_extended_ba.md](docs/hand_eye_extended_ba.md) — 확장 BA + 물리 sag 모델 (OMX 시대, σ_rot 0.65°/σ_t 7.94mm 도달기)
- [handeye_sigma_floor_so101.md](docs/handeye_sigma_floor_so101.md) — **SO-101+D405 σ floor 진단 (2026-06-21)**. algorithmic optimum effective σ_R 0.801°/σ_t 7.53mm. STS3215 backlash ±0.87° 가 σ_R 0.5° hardware ceiling. cv2_seed + MCMC 4 chain (R̂ 1.0023) + Stage E + Kalib 다 reject. **다음 캘 trauma 진입 시 anchor — 동일 옵션 또 검토 안 해도 됨**
- [handeye_robust_irls_plan.md](docs/handeye_robust_irls_plan.md) — 캘 trauma 영구 fix plan (IRLS + Huber + Strategy 패턴). **§12 진행 결과 (2026-06-12) — PnP gate / IRLS / observability / JointPerturbationStrategy / LOOCV 외부 정확도 발견까지**
- [tsdf_pipeline.md](docs/tsdf_pipeline.md) — multi-way ICP + TSDF mesh 빌드 결정사항
- [scan_pipeline_readiness.md](docs/scan_pipeline_readiness.md) — **SO-101 scan/TSDF 시작 전 코드 검토 (2026-06-21)**. 4-노드 구조 mature, BLOCKING 2: `robot_poses.yaml (so101)` missing + frontend PLY mesh layer 없음. 첫 scan 체크리스트.
- [step_dsl.md](docs/step_dsl.md) — typed Slot 기반 lego Step DSL (Step/Slot/StepContext/Recipe + 다이어그램 + 확장 가이드)
- [random_palletizing.md](docs/random_palletizing.md) — 사이즈 가변 직육면체 팔레타이징 design (3-track: 휴리스틱 / 정석 / iterative sim2real RL)
- [so101_6dof_plan.md](docs/so101_6dof_plan.md) — SO-101 6DOF 두 번째 로봇 하드웨어 plan (모터 SDK 추상화 / wrist yaw mod / D405 마운트)
- [multi_robot_architecture.md](docs/multi_robot_architecture.md) — multi-robot platform 업그레이드 design (Adapter/Strategy/DIP 패턴 layer / robot identity / 토픽 namespace 재설계 / Coordinator / 마이그레이션 phase)
- [multi_robot_walkthrough.md](docs/multi_robot_walkthrough.md) — Phase 1 (foundation) 산출물 + 클래스/시퀀스 다이어그램 + Phase 2 남은 작업 follow-up 가이드. **코드 읽으며 학습할 때 anchor**
- [multi_robot_phase2_frontend.md](docs/multi_robot_phase2_frontend.md) — Phase 2 의 namespace + frontend / UX 결정문 + 구현 결과 (§6, Slice A/B/C-mechanical 완료)
- [slice_abc_verify.md](docs/slice_abc_verify.md) — Slice A/B/C 실 hardware + dev 서버 검증 순차 가이드
- [distributed_topology.md](docs/distributed_topology.md) — Phase 2 분산 토폴로지 + 카메라 배치 design (잠정) — `hori1/2/3` Pi 3대 + so101 D405 양도, `robots.yaml` host 필드 확장 후보
- [multi_robot_cross_calibration.md](docs/multi_robot_cross_calibration.md) — 두 robot base 사이 transform 캘 (6번째 종) design + hand-measure fallback + so101 도착 후 follow-up
- [storage_layer.md](docs/storage_layer.md) — 영속성 layer design (storage_node Zenoh gateway + RdbStore/ObjectStore Protocol, bridge 와 동일 패턴). Phase 1 = 캘 5종 + SQLite/fs (git push/pull 동기화 사라짐), Phase 2 scans/meshes/task_runs, Phase 3 NAS Postgres/MinIO
- [llm_preload_race_debug.md](docs/llm_preload_race_debug.md) — LLM/Grounding DINO preload meta-tensor race 진단 + 검증 plan (10회+ 깔짝 fix 박제 anchor, reproduction script 가 fix 보다 먼저)
- [motion_taxonomy.md](docs/motion_taxonomy.md) — Horibot motion primitive 4 계층 (Move/Servo/Jog/Task) + UR/ABB/KUKA 산업 표준 매핑 + Phase 1.5 (2026-06-17) 자리 — Servo 의미 자리 보존 (절대 target chase = RL/Vision servo) + 신규 **Jog 계층** (human velocity, backend latch + SE(3) 적분 SSOT, LeRobot delta-pose 패턴) + SpeedJ/SpeedTcp + Ruckig velocity stream 자리 폐기.
- [jog_drift_tuning.md](docs/jog_drift_tuning.md) — SpeedTcp/SpeedJ cartesian jog 의 transient drift 진단 + fix 박제. **2026-06-17 update v2**: root cause 자리 *resolved-rate velocity-streaming* 아키텍처 자체. SpeedJ/SpeedTcp 자리 폐기 + JogJ/JogTcp (LeRobot delta-pose) 자리 도입으로 directional transient 자체 사라짐 (수학적 보장).
- [naming_conventions.md](docs/naming_conventions.md) — wire schema class 이름 (verb-first + sub-domain prefix, Google AIP/Kubernetes 정석) + key expression helper (`key_for`, Zenoh 어휘 정렬) 컨벤션 + migration phase 진행 status. **신규 messages / RPC 추가 시 reference**
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

# Mock (단일 머신, 하드웨어 X — frontend UX 개발/검증)
uv run python main.py --host mock        # motor/camera 만 mock 노드로 swap, 나머지 그대로

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

- `host_dev.yaml` — 단일 머신 풀스택 (robots=[omx_f_0], device_nodes=[motor/motion/camera], application_nodes=[storage/calibration/task/detector/scene3d/reconstruction/gamepad] + 브릿지)
- `host_mock.yaml` — 단일 머신, 하드웨어 없이 frontend UX 검증 (device_nodes=[mock_motor/mock_camera/motion] + application_nodes 그대로)
- `host_pc.yaml` — 분산 PC (USB 직결 robot 없음: robots=[], device_nodes=[], application_nodes=[storage/calibration/task/detector/scene3d/reconstruction/gamepad] + 브릿지)
- `host_pc_sim.yaml` / `host_pi_motor_sim.yaml` / `host_pi_camera_sim.yaml` — 분산 sim (localhost multi-process, scene3d_decoupling.md §13.1 검증 자리). mock motor/camera + Zenoh peer 자동 발견 검증용
- `host_pi_motor.yaml` — 분산 모터 Pi (robots=[omx_f_0], device_nodes=[motor/motion])
- `host_pi_camera.yaml` — 분산 카메라 Pi (robots=[omx_f_0], device_nodes=[camera])

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
| PC              | detector, task, scene3d, reconstruction, storage, calibration, bridge, (gamepad) | YOLO, RGBD primitive, reconstruction (ICP/TSDF), storage gateway, 브릿지     |
| 모터 Pi (101)   | motor, motion                                              | Dynamixel + Ruckig + IK (제어 루프 로컬화) |
| 카메라 Pi (102) | camera                                                     | D405 캡처 + JPEG + 압축 depth 발행         |

이 분배의 핵심 이유:

- (a) USB 대역폭 경합 해소 — D405와 OpenRB-150이 한 USB 컨트롤러를 공유하지 않음
- (b) 100Hz 제어 명령(`MOTOR_CMD_JOINT`)을 네트워크로 안 보냄 — TrajectoryRunner와 MotorNode 같은 머신
- (c) 무거운 연산(YOLO, Open3D, PyBullet PC 측, TSDF build)은 PC

### 2-layer 트랜스포트: Zenoh (백엔드 내부) + WebSocket (브라우저)

백엔드 노드는 모두 **Zenoh** pub/sub + queryable로 통신. 프로세스당 하나의 `ZenohSession` 싱글톤 ([backend/core/transport/zenoh_session.py](backend/core/transport/zenoh_session.py))을 모든 노드가 `ZenohSession.get()`으로 공유. 토픽 페이로드는 보통 JSON, 카메라 JPEG / depth_frame / 포인트클라우드는 raw 바이너리. 서비스는 `{success, message, data}` 응답 봉투.

분산 시 각 머신이 독립 peer로 동작 — 같은 LAN이면 멀티캐스트 scout으로 자동 발견. 명시적 endpoint가 필요하면 host config의 `zenoh.connect`에 적음.

브라우저는 Zenoh를 못 하니까 [backend/bridge/zenoh_bridge.py](backend/bridge/zenoh_bridge.py)의 FastAPI가:

- `/ws` — WebSocket: `subscribe` / `unsubscribe` / `publish` / `service` 메시지 ↔ Zenoh 양방향 변환.
- `/robots/{robot_id}/camera/stream` — MJPEG `multipart/x-mixed-replace` 스트림, `horibot/{robot_id}/camera/stream/raw` Zenoh 토픽 소스.
- [robot/](robot/)를 `/robot`에 정적 마운트 (URDF, mesh, calibration .npz, TSDF .ply).
- "코어" 토픽을 미리 구독해 WS 클라이언트들에 재방송 (`_ALWAYS_SUBSCRIBE`).

JSON 토픽은 `{type:"topic_data", topic, data}`로 텍스트 송신. **바이너리 토픽**(카메라 raw 제외 — 그건 MJPEG로 별도 라우트)은 다음 프레이밍으로 binary WS 송신:

```
[u8 version=1][u8 type=1 (topic_data)][u16 BE topic_len][UTF-8 topic][payload]
```

현재 이 바이너리 프레이밍을 쓰는 토픽은 `horibot/{robot_id}/scene3d/stream`. 프론트에서 동일 디코더로 풀어 React-Three-Fiber에 먹임.

#### 클라이언트별 송신 큐 + 백프레셔 ([backend/bridge/client_stream.py](backend/bridge/client_stream.py))

각 (WS 클라이언트, 구독 토픽) 짝마다 별도 `ClientStream` (bounded `asyncio.Queue` + sender task) — 느린 브라우저 한 명 때문에 메모리가 폭증하지 않게.

토픽별 정책:
- 기본: `LATEST_WINS` — 큐 크기 1, 새 값이 옛 값을 덮어씀.
- `SYSTEM_LOG`: `BOUNDED_FIFO`(128) — 로그는 일정량 보존.

프론트엔드는 `BridgeClient` 싱글톤 ([frontend/src/api/bridge.ts](frontend/src/api/bridge.ts))으로 감싸고 `ReconnectingWebSocket`, 토픽별 멀티플렉싱, `request_id` 기반 service promise를 처리. `useBridge` ([frontend/src/hooks/useBridge.ts](frontend/src/hooks/useBridge.ts))가 `App.tsx`에서 한 번 마운트되어 들어오는 데이터를 Zustand store로 라우팅.

### 토픽/서비스 레지스트리 — 두 곳에서 동기화

토픽/서비스 키는 backend [api_contract.py](backend/api_contract.py) 가 SSOT — `PUBLIC_TOPICS` / `PUBLIC_SERVICES` dict 에 등재된 것만 frontend 공개. bridge 가 `custom_openapi()` 로 `/openapi.json::x-contract` 에 인라인 → frontend `pnpm gen:types` 가 [generated/contract.ts](frontend/src/api/generated/contract.ts) 자동 emit. 키 자체는 [backend/core/transport/topic_map.py](backend/core/transport/topic_map.py) 의 `Topic` / `Service` 에 정의 (예: `horibot/{robot_id}/motion/srv/move_l`). robot-scoped 키는 `{robot_id}` placeholder template — `BaseNode.r()` / `BridgeClient` 자동 expand ([docs/multi_robot_phase2_frontend.md §1](docs/multi_robot_phase2_frontend.md)).

프론트에서 사용하지 않는 내부 토픽/서비스(예: `CAMERA_DEPTH_FRAME`, `MOTOR_GRIPPER`, `CAMERA_SET_DEPTH_STREAM`, `DETECT_SERVICE`)는 `PUBLIC_*` 에 등재 X — generated contract 에 자동 누락.

### 노드 패턴 — Device / Application 2-layer + 노드 레지스트리 (lazy import)

노드는 **두 layer** 로 분리, 클래스 계층 + 폴더 구조로 명시:

- **[DeviceNode](backend/core/transport/device_node.py)** ([backend/nodes/device/](backend/nodes/device/)) — vendor-shipped 하드웨어/컨트롤러 bundle. UR Control Box 등가물. robot 마다 별도 인스턴스 (per-robot). `robot_id` 필수.
  - `motor` / `motion` / `camera` / `mock_motor` / `mock_camera`
- **[ApplicationNode](backend/core/transport/application_node.py)** ([backend/nodes/application/](backend/nodes/application/)) — robot driver 위에 얹는 algorithm/scenario layer. 호스트당 1 인스턴스, 내부 `dict[robot_id]` 로 multi-robot dispatch. base 가 `self.enabled_robot_ids` 노출.
  - `calibration` / `task` / `detector` / `scene3d` / `reconstruction` / `storage` / `gamepad`

architecture layer 판정은 클래스 계층이 SSOT — `issubclass(cls, DeviceNode)` 로 확인. [node_registry.py](backend/core/transport/node_registry.py) 의 `NodeSpec` 은 `(module, cls_name)` 두 string 만 들고 있는 순수 lazy-import 컨테이너 — `importlib` 가 호출 시점에만 import 해서 모터 Pi 가 `node_registry` import 해도 open3d/pyrealsense2/ultralytics 등 PC 전용 dep 가 import 트리에 안 끌려옴. main.py 는 host yaml 의 `device_nodes` / `application_nodes` 위치 검증을 `issubclass` 로 수행 (placement 잘못이면 부팅 시 fail-fast).

공통 [BaseNode](backend/core/transport/base_node.py) 기능:

- `create_subscriber(topic, callback)` — JSON 디코드된 pub/sub
- `create_raw_subscriber(topic, callback)` — JSON 디코드 없는 binary 페이로드
- `create_service(key, handler)` — Zenoh queryable 등록, 핸들러는 `{success, message, data}` 반환
- `call_service(key, data, timeout=5.0)` — 기본 5초 타임아웃 동기 호출
- `publish(topic, data)`, `log(level, msg)`, 1Hz heartbeat `horibot/system/heartbeat`
- `self.r(template)` — `{robot_id}` placeholder 자동 expand (robot-scoped 토픽/서비스 호출 자리에서 사용)

라이프사이클: `start()` → heartbeat 스레드 + 노드별 워커 스레드 → `stop()`이 모든 subscriber/queryable undeclare.

### Mock backend (UX 개발 / 하드웨어 없는 검증)

`--host mock` 으로 띄우면 [host_mock.yaml](backend/config/host_mock.yaml) 이 `mock_motor` / `mock_camera` + 나머지 실 노드 (motion/storage/calibration/task/detector/scene3d/reconstruction/bridge) 를 같이 띄움. mock 노드는 topic/service contract 만 충족:

- [MockMotorNode](backend/nodes/device/motor_node_mock.py) — motors.yaml 의 home raw 로 초기화, `MOTOR_CMD_JOINT` 받으면 internal position 즉시 갱신, `MOTOR_STATE_JOINT` 20Hz publish. TrajectoryRunner 가 100Hz 보간으로 명령 보내니 UI 시각상 부드러움. 서비스 (enable/reboot/gripper/…) 는 success no-op.
- [MockCameraNode](backend/nodes/device/camera_node_mock.py) — 합성 JPEG (MOCK CAMERA 라벨 + frame counter + 움직이는 dot) 30Hz publish + status connected=true. depth payload 미발행 → PointCloud 패널은 빈 상태.

motion / storage / calibration / detector / scene3d / reconstruction / bridge 는 모두 topic 기반이라 mock motor/camera 가 publish 만 정상이면 평소 코드 경로 그대로 동작. mock_camera 도 set_stream ON 시 합성 gradient depth + JPEG color 8 FPS publish — Scene3DNode snapshot e2e 자체 host_mock 단일 process 에서 검증 가능 (mock_scan_e2e.py). 진짜 검증 (calibration capture / detection accuracy / 실 D405 scan + ScanTask) 은 실 hardware 자리.

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
- `JointStateCache` ([backend/core/cache/joint_state_cache.py](backend/core/cache/joint_state_cache.py)) — `MOTOR_STATE_JOINT`를 한 번만 구독, `get_joint_angles_rad(arm_cfgs)`로 라디안 단위 최신 조인트각 노출 (raw→rad 시 joint_offset 자동 적용 — § 캘리브레이션 적용). motion/task/detector/calibration/scene3d/reconstruction가 공유.
- `FrameCache` ([backend/core/cache/frame_cache.py](backend/core/cache/frame_cache.py)) — `CAMERA_STREAM_RAW`(JPEG) + `CAMERA_STATE_STATUS` 구독, `get_frame()`이 BGR ndarray 반환. detector/calibration이 토픽 기반으로 동작 → 카메라가 다른 머신에 있어도 동일 코드.
- `RealsenseCapture` ([backend/modules/camera/adapters/realsense_capture.py](backend/modules/camera/adapters/realsense_capture.py)) — `CameraCapture` Protocol ([modules/camera/capture.py](backend/modules/camera/capture.py)) 만족하는 RealSense wrapper (내부 `RealsenseDriver` ([backend/modules/camera/adapters/realsense_driver.py](backend/modules/camera/adapters/realsense_driver.py)) 싱글톤이 raw SDK 담당 — motor 의 `DynamixelBackend`/`DynamixelDriver` 와 동형). pyrealsense2 파이프라인 1개 공유. **카메라 호스트에서만 살아 있음**. multi_robot_architecture.md §3.4.
- `get_default_kinematics()` ([backend/modules/kinematics/registry.py](backend/modules/kinematics/registry.py)) — facade. 내부적으로 `RobotRegistry().get_kinematics(default)` 호출 → `SagCorrectedKinematics(PybulletKinematics(urdf), link, sag)` 체인 반환. `Kinematics` Protocol 만족 (`fk` / `ik` / `fk_to_matrix` / `joint_limits` / `dof` / `tcp_link_name`). **link_offset 패치된 URDF** 는 `PybulletKinematics` 가 로드, **sag** 보정은 `SagCorrectedKinematics` Decorator 가 양방향 적용 ([docs/calibration_apply_flow.md](docs/calibration_apply_flow.md), [docs/multi_robot_architecture.md](docs/multi_robot_architecture.md) §3.1-3.2).
- `RobotRegistry` ([backend/core/robot/robot_registry.py](backend/core/robot/robot_registry.py)) — `robot/robots.yaml` 의 single source of truth. `get(robot_id)` 로 `RobotConfig` (모든 path / 설정), `get_kinematics(robot_id)` / `get_motor_backend(robot_id)` / `get_camera_capture(robot_id)` factory (per-robot 인스턴스 캐시). `default()` / `default_robot_id()` 가 N=1 편의.
- `*Coordinates` 싱글톤 — `JointCoordinates` / `LinkCoordinates` / `SagCoordinates` ([backend/core/](backend/core/)). 각각 npz 1회 로드 후 메모리 캐시. raw↔rad / URDF patch / sag stiffness를 노출.

### Motion 파이프라인 — 4 계층 motion primitive

[`MotionNode`](backend/nodes/device/motion_node.py) 가 산업 표준 4 계층 (motion_taxonomy.md) 의 motion primitive 수신:

| 계층 | Joint | Cartesian | caller | server | 의미 |
|---|---|---|---|---|---|
| **Move** (trajectory-planned) | `MOTION_MOVE_J` | `MOTION_MOVE_L` / `MOTION_MOVE_C` / `MOTION_MOVE_P` | 사용자 단발 명령 | Ruckig jerk-limited 보간 | 단발 절대 target → trajectory |
| **Servo** (external target chase) | `MOTION_SERVO_J` | `MOTION_SERVO_TCP` | RL/Vision servo 자리 (미래) | direct IK + publish | 외부가 자기가 계산한 절대 target 빠른 rate 스트림 |
| **Jog** (human/manual velocity) | `MOTION_JOG_J` (+`_STREAM` topic) | `MOTION_JOG_TCP` (+`_STREAM` topic) | frontend / gamepad 50Hz | backend latched ref + 실 dt 적분 (SE(3) 자리 scipy `Rotation` SSOT) → IK → publish | LeRobot delta-pose 패턴 |
| **Task** | — | — | task_node scripted | step DSL 자리 직접 motion 호출 | scripted recipe |

검증/실행은 `MotionCommand` 서브클래스 ([`motion_commands.py`](backend/modules/kinematics/motion_commands.py)) — `MoveJ/L/C/P`, `ServoJ`, `ServoTcp`, `JogJ`, `JogTcp`. Move/Cartesian path 자리는 [`TrajectoryRunner`](backend/modules/kinematics/trajectory_runner.py) 가 **Ruckig position mode** 으로 처리. Servo / Jog 자리는 trajectory_runner 안 거치고 *직접* IK + publish. 모두 `publish_cmd` 콜백 → `MOTOR_CMD_JOINT` 토픽 (urdf→raw 변환 시 joint_offset 자동 차감). `MOTION_STATE_TRAJ` 에 trajectory 진행 publish.

**Jog stream wire 자리** — frontend / gamepad 자리 50Hz `bridge.publish(Topic.MOTION_JOG_*_STREAM)` 자리 fire-and-forget topic publish (service RTT 회피, UR teach pendant 정석). backend motion_node 가 직접 subscribe → `JogJCommand` / `JogTcpCommand` 자리에서 적분 + IK + publish. JogTcp 자리 `frame`: `"base"` (world axes) / `"tcp"` (EE-local) — backend SE(3) 적분 자리 frame 변환 (base=premultiply, tcp=postmultiply). 5DOF robot 자리는 server-side IK 자리 position-only fallback.

**MotionNode와 MotorNode는 같은 머신(모터 Pi)에 배치** — Jog/Move 자리 50Hz publish 명령이 네트워크를 넘지 않게 해야 끊김/지터를 막을 수 있음. PyBullet IK도 같은 머신.

robot dof 는 `MotorLayout.arm` 길이 (motors.yaml SSOT — OMX-F=5, SO-101=6). Dynamixel raw 단위 (`0..4095`, 중심 `2048`) 와 URDF rad 의 변환은 [backend/core/units.py](backend/core/units.py). gripper 는 `MotorLayout.gripper` 별도.

### Gamepad mini 펜던트

[`GamepadNode`](backend/nodes/application/gamepad_node.py) — `robots.yaml::capabilities` 에 `"gamepad"` 있는 enabled robot 정확히 1개로 자동 매칭 (N>1 fail-fast). 6DOF + cartesian jog 자연 robot 자리만 (SO-101). 8BitDo Ultimate 2C 50Hz polling:

- **LT (hold) = deadman** — 안 누름 = motion publish X. publish 중단 자리 모터는 마지막 target 머무름, 0.2s 후 backend fresh latch 준비 (ISO 10218 enabling switch 등가).
- **Back** = TCP ↔ Joint mode 토글, **Start** = base ↔ tcp frame 토글 (TCP mode 자리만)
- **TCP mode**: 왼스틱 X/Y → linear, 오른스틱 X/Y → angular yaw/pitch, LB/RB → angular roll, D-Pad → linear Z. `MOTION_JOG_TCP_STREAM` topic publish.
- **Joint mode**: 6축 스틱/트리거 → JogJ 6-vector. `MOTION_JOG_J_STREAM` topic publish.
- **X**=토크 토글 / **Y**=홈 / **A**=그리퍼 / **B**=캘 캡처

8BitDo 의 실 pygame button index 가 [`mapper.py`](backend/modules/gamepad/mapper.py) 가정 매핑과 일치하는지는 hand-on 검증 자리 (controller / OS / driver 종속).

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

토픽 (global — task 가 robot 을 포함, [docs/multi_robot_phase2_frontend.md §1](docs/multi_robot_phase2_frontend.md) 결정):
- `horibot/task/tree` — task 시작/preview 시 전체 step tree (children 재귀) 1회
- `horibot/task/state` — RUNNING/PAUSED/SUCCESS/FAILED + step_statuses + current_step_id
- `horibot/task/step_result` — 각 step 완료 시 `{step_id, type, value}`. frontend `TaskResultLayer` 가 type 별 자동 렌더링 (Detection→sphere, Position3→marker)

상세 (Step/Slot/StepContext 작동, ForEach unroll, 새 step / task 짜는 법, lego test 통과 근거) 는 [docs/step_dsl.md](docs/step_dsl.md).

### Detector → 월드 좌표 파이프라인

`DetectorNode._handle_detect` ([backend/nodes/application/detector_node.py](backend/nodes/application/detector_node.py)) 체인: `FrameCache.get_frame()` → YOLO centroid → `cv2.undistortPoints`로 intrinsic 보정 → `MOTION_GET_TCP`로 현재 EE 포즈 (joint/link/sag 보정 내장) → base-frame 평면 `Z=0` 제약으로 `Z_cam` 역산 → hand_eye matrix 곱해서 베이스 프레임의 물체 위치 반환. `intrinsic.npz`와 `hand_eye.npz` 둘 다 필요 — `load_calibration().is_ready()`가 서비스 가드. 별도 5fps `_detection_loop`가 `DETECTOR_STATE`에 raw detection을 stream으로 발행.

### D405 → 라이브 PointCloud + TSDF 파이프라인

```
CameraNode (카메라 Pi)
  ├─ 30 FPS color JPEG → horibot/{robot_id}/camera/stream/raw           (항상 켜짐)
  └─ 8 FPS depth_frame → horibot/{robot_id}/camera/stream/depth_frame   (CAMERA_SET_DEPTH_STREAM enable 시)
       페이로드 (modules/camera/depth_frame.py, little-endian):
         [u32 header_len][JSON header][u32 jpeg_len][aligned color JPEG][zstd Z16 depth]
       header: timestamp / width / height / depth_scale / fx fy cx cy
               / depth_uncompressed_bytes (검증용)

Scene3DNode (PC) — [backend/nodes/application/scene3d_node.py](backend/nodes/application/scene3d_node.py)
  ← CAMERA_DEPTH_FRAME 구독 (raw subscriber, rgbd capability robot 만 dispatch)
  ─── 라이브 stream: depth_frame → Open3D RGBDImage → PointCloud → voxel_down_sample
         → horibot/{robot_id}/scene3d/stream (raw binary: [u32 n][n*3 float32 xyz][n*3 uint8 rgb])
  ─── snapshot: SCENE3D_SNAPSHOT 호출 시 N frame consensus median → JPEG color + zstd depth + intrinsic + motor positions return (memory)
         consensus 자리 [backend/modules/scene3d/consensus.py](backend/modules/scene3d/consensus.py)
  ─── refcount: snapshot uuid + persistent "stream" 토큰 같은 set 공유 → CAMERA_SET_DEPTH_STREAM enable/disable 한 자리만

ScanTask (TaskNode 의 Step DSL — [backend/modules/task/tasks/scan.py](backend/modules/task/tasks/scan.py)):
  NewSession → ForEach(MoveJByName + CaptureScan) → BuildReconstruction
  - CaptureScan: SCENE3D_SNAPSHOT → scan_workflow.blob.encode → STORAGE_PUT_SCAN
  - BuildReconstruction: RECONSTRUCTION_BUILD (5-30s) → step progress = RECONSTRUCTION_PROGRESS topic 5 stage

ReconstructionNode (PC, host-level 1개) — [backend/nodes/application/reconstruction_node.py](backend/nodes/application/reconstruction_node.py)
  ─── service: RECONSTRUCTION_BUILD(session_row_id) → STORAGE_LIST_SCANS + STORAGE_GET_BLOB (각 scan) → multi-view ICP + PoseGraph + TSDF + mesh extract → STORAGE_PUT_RECONSTRUCTION (.ply blob)
  ─── 5 stage progress publish (loading_scans / pairwise_registration / pose_graph_optimization / tsdf_integration / mesh_extraction)
  ─── build pipeline [backend/modules/reconstruction/build.py](backend/modules/reconstruction/build.py)

StorageNode (PC) — Phase 2 scan workflow CRUD (storage_layer.md §11):
  ─── 3 entity: scan_sessions / scans / reconstructions (append-only blob + immutable metadata row, is_active X)
  ─── blob: ObjectStore key `scans/<robot_id>/<session_id>/<scan_id:03d>.bin` + `reconstructions/<robot_id>/<session_id>/recon_<ts>.ply`
  ─── 10 service: NEW_SCAN_SESSION / LIST_SCAN_SESSIONS / DELETE_SCAN_SESSION (CASCADE) / PUT_SCAN / LIST_SCANS / DELETE_SCAN / GET_BLOB / PUT_RECONSTRUCTION / LIST_RECONSTRUCTIONS / DELETE_RECONSTRUCTION

프론트엔드 (브릿지가 binary WS 프레이밍으로 중계)
  → SCENE3D_SET_STREAM {enabled}  ─ 라이브 stream on/off (Scene Controls 토글, mode 무관)
  → TASK_RUN(task=scan, scan_poses=[...])  ─ ScanTask 실행 (TasksPage)
  ← SCENE3D_STATE {enabled, voxel_size}
  ← horibot/{robot_id}/scene3d/stream (binary point cloud)
  ← RECONSTRUCTION_PROGRESS {stage, percent, message}
  ← TASK_TREE / TASK_STATE / TASK_STEP_RESULT
```

핵심 설계 (scene3d_decoupling.md 4-노드 분리):
- **Scene3DNode = primitive 만** — RGBD sensor 한 자리. scan/mesh/session/storage 자리 X
- **StorageNode = persistence** — RDB row (metadata) + ObjectStore blob (raw depth zstd + color JPEG). 다른 노드는 SQL/S3 모름
- **ReconstructionNode = heavy compute** — ICP + PoseGraph + TSDF + mesh extract. 5 stage progress publish
- **ScanTask = workflow** — TaskRunner 의 pause/resume/breakpoint/progress 자연 흡수. mode 무관 (TasksPage 의 task select)
- **`scan` 어휘 3 갈래 분리** — capability `rgbd` (sensor) / mode 자리 제거 / task `scan` (workflow). robots.yaml SSOT
- **Depth는 무손실(zstd)** — ICP/TSDF 정밀도 보존. Color만 JPEG 손실
- **scan은 raw motor positions로 저장** — 캡처 시점 캘 변해도 raw 불변. ReconstructionNode 가 build 시 현재 캘로 fresh 재계산
- **scan_id monotonic** — RDB 의 `allocate_scan_id(session_row_id)` 가 transaction MAX+1. 삭제 후에도 안 줄어듦
- **Pydantic `Base64Bytes`** — wire 시 base64 string, Python 코드는 raw bytes (binary blob 의 utf-8 인코딩 fail 우회)

라이브 점군은 백엔드에서 camera-frame xyz publish, 프론트 [Scene3DLayer.tsx](frontend/src/components/scene/Scene3DLayer.tsx)가 `<group position quaternion>` 부모 transform 으로 `cameraMatrix = tcpMatrix · handEyeMatrix` 적용. Reconstruction mesh layer 자리는 후속 (scene3d_decoupling.md §13.3 의 hardware 검증 진입 시).

### 캘리브레이션 — 4종 산출물 + intrinsic

다섯 가지 npz가 `robot/instances/<robot_id>/calibration/` 에 있음 (instance 별 분리 — multi_robot_architecture.md §5.1). 각각 적용 메커니즘이 다름:

| 산출물       | 무엇을 보정         | 어디서 적용                                | COMMIT 후                     |
| ------------ | ------------------- | ------------------------------------------ | ----------------------------- |
| intrinsic    | D405 카메라 내부    | `cv2.undistortPoints` (Detector)           | DetectorNode 재시작           |
| hand_eye     | 카메라 ↔ EE 변환    | Detector 후처리 + 프론트 PointCloudLayer   | DetectorNode 재시작           |
| joint_offset | 모터 raw zero 오차  | `JointCoordinates.motor_to_urdf` / `urdf_to_motor` (raw↔rad 변환 양쪽) | 즉시              |
| link_offset  | URDF 링크 기하 오차 | **URDF 자체를 patch**해서 PyBullet 로드    | 백엔드 재시작 (PyBullet 1회 로드) |
| sag_offset   | J2/J3 자세 의존 중력 처짐 | `SagCorrectedKinematics.fk`/`ik` 양방향 적용 (Decorator) | 즉시 (`_reload_caches`) |

확장 BA + 물리 sag 모델 도달 σ:
- OMX_F (2026-05): σ_rot **0.65°** / σ_t **7.94mm** ([hand_eye_extended_ba.md](docs/hand_eye_extended_ba.md))
- SO-101 + D405 (2026-06): **effective σ_R 0.801° / σ_t 7.53mm** (algorithmic floor — STS3215 backlash ±0.87° 가 σ_R 0.5° hardware ceiling, [handeye_sigma_floor_so101.md](docs/handeye_sigma_floor_so101.md))

σ **두 metric** ([[project-calibration-sigma-dual-metric]]):
- **effective σ** = `measure_effective_sigma` (board_in_base std, data consistency, commit 결정 metric). DB `calibration_results.effective_sigma_rot/t` 컬럼
- **Jacobian σ** = `(JᵀJ)⁻¹·σ²` block trace (parameter confidence). DB `calibration_results.sigma_rot/t` 컬럼
- [commit_results](backend/scripts/calibrate_offline.py) 가 hand_eye row INSERT 시 둘 다 박음

산출물별 코드 흐름 + COMMIT 후 어디까지 자동 반영되는지는 [docs/calibration_apply_flow.md](docs/calibration_apply_flow.md), 캘 절차/UI 사용법은 [docs/calibration_workflow.md](docs/calibration_workflow.md).

### 캘 분석 / 진단 script ([backend/scripts/](backend/scripts/))

새 캘 분석 시 순서:
1. [calibrate_offline.py](backend/scripts/calibrate_offline.py) — 5 stage BA + sanity + LOOCV. `--commit` 으로 DB activate
2. [calibrate_squeeze.py](backend/scripts/calibrate_squeeze.py) — outlier squeeze iter (LOOCV 줄어들 때까지 drop 추가). best drop set 식별만, commit X. 출력 마지막 줄에 `calibrate_offline.py --commit --drop-poses ...` 명령 안내

trauma cycle 진입 시 [handeye_sigma_floor_so101.md](docs/handeye_sigma_floor_so101.md) 먼저 — algorithmic optimum 진단 결과 (cv2_seed + MCMC unimodal + Stage E reject + Kalib NO-GO) 박혀있음. **진단용 script 들 (cv2_seed / validate_opencv / mcmc / push / diagnose / effective_sigma) 은 한 번 답 나왔으니 2026-06-21 정리 시 제거** — 필요 시 git history 에서 복원 가능.

**Commit + Rollback (DB 기반, storage_layer)** — `_srv_handeye_commit` 가 `_storage.finalize_run` (draft run status in_progress→success + Result rows INSERT, is_active=0) → 각 result `_storage.activate` (atomic: 같은 robot+kind 직전 active 해제 → 새 result 활성). 즉 **COMMIT = DB 저장 + activate 동시**. 적용은 storage invalidation → CalibrationCache refetch + (link_offset 은 URDF patch 라) 백엔드 재시작.
**Rollback = DB history 의 과거 run 재활성** — 모든 commit 이 run 으로 영구 보존. frontend [`CalibrationHistoryPanel`](frontend/src/components/panels/calibration/CalibrationHistoryPanel.tsx) 가 run 목록 → kind 별 `ACTIVATE` (`STORAGE_ACTIVATE_CALIBRATION`) → `activate_result` 가 atomic 하게 현재 해제 + 과거 활성. (옛 npz `.history` / `backup.py` / `CALIB_BACKUP_*` 는 storage DB 전환으로 폐기 — DB history 가 롤백.)

**ChArUco 검출 (2026-06-10)** — [`board.py`](backend/modules/calibration/board.py) 가 보드 spec SSOT. calib.io PDF generator 입력 (`Rows=5, Columns=7`) → OpenCV `CharucoBoard.size = (squaresX=Columns=7, squaresY=Rows=5)` 컨벤션 매핑. spec: 7×5 squares / 25mm checker / 18mm marker / DICT_4X4_50 / start_id 0 / modern pattern. plain chessboard → ChArUco 로 전환되어 일부 가림에도 검출 살아남음. `detect()` / `detect_full()` (marker outline 포함, preview overlay 용) / `match_object_points()` / `draw()` 한 진입점을 intrinsic / handeye_capture / preview_loop 가 공유.

**tilt SSOT (2026-06-10)** — [`thresholds.py`](backend/modules/calibration/thresholds.py) 의 `TILT_MIN_DEG=30 / TILT_MAX_DEG=70` 가 PnP 권장 범위. backend `next_pose_planner.is_pose_visible` (추천 자세 가시성 게이트) + `capture_quality` (Phase 1 Traffic Light) + frontend [`CaptureGuideOverlay`](frontend/src/components/panels/calibration/parts/CaptureGuideOverlay.tsx) 가 같은 임계 공유 — 추천 따라 토크오프로 맞춘 자세가 캡처 가능 자세와 일치.

**자동 BA + σ live (2026-06-10)** — `_srv_handeye_capture` 끝에 `pose_count >= MIN_POSES_FOR_COMPUTE` 면 자동 BA → `CALIB_HANDEYE_SIGMA` topic 으로 `HandeyeSigmaState` publish. 사용자가 [COMPUTE] 별도로 안 눌러도 매 capture 후 frontend σ badge 갱신. visibility gate (`next_pose_planner.is_pose_visible`) 가 추천 후보의 보드 reproject → 화면 밖이면 `visible=false` 마크 (UI 회색 hint, hard filter 아님).

**PnP 품질 gate (2026-06-12)** — `_srv_handeye_capture` 의 `cv2.solvePnP` 직후 reprojection error RMS 계산. `thresholds.HANDEYE_PNP_RMS_REJECT_PX=1.5px` 초과 시 capture **자동 reject** + 사용자 안내 ("이미지 품질 부족, 다시 시도해주세요"). ChArUco 코너 가림 / blur / 광량 부족 / board 미세 움직임이 만든 안 좋은 자세 자체를 *애초에 안 들임* — trauma source 입구 차단.

**IRLS+Huber on `_physical_sag` (2026-06-12, [docs/handeye_robust_irls_plan.md §12](docs/handeye_robust_irls_plan.md))** — 운영 BA = [`bundle_adjust_hand_eye_physical_sag_irls`](backend/modules/calibration/bundle_adjust.py). 자세별 Huber weight `w_i = min(1, κ/r_i)`, κ=1.345·MAD/0.6745. outlier 자세의 X drift *수학적으로 bounded*. acceptance test (8장 + 합성 outlier 5°/20mm) — IRLS ΔR=baseline×0.55, w_outlier=0.118. 결과 dataclass `BundleAdjustPhysicalSagResult` 가 weights / outer_iter / cost_history / sigma_hat_history 통합 (non-IRLS BA 가 default 채움 호환). per-pose weight 가 `last_compute.per_pose_residual[i].weight` 로 frontend 전파 → [`PoseList`](frontend/src/components/panels/calibration/parts/PoseList.tsx) 가 color dot (정상/낮음/제외) 표시 — 수치 노출 X.

**관측성 진단 (2026-06-12, [`observability.py`](backend/modules/calibration/observability.py))** — 매 capture 후 [`analyze_pose_data`](backend/modules/calibration/observability.py) 자동 호출 + `CALIB_HANDEYE_OBSERVABILITY` topic publish. 4 metric (광축 펼침 / tilt 분포 / 회전축 spanning σ₃/σ₁ / wrist roll) → verdict (A/B/mid). frontend [`HandEyePanel`](frontend/src/components/panels/calibration/HandEyePanel.tsx) 의 `ObservabilityBanner` 가 verdict 만 색깔 안내 (metric 숫자 노출 X). A=다양성 충분 / B=구조적 부족 (보드 위치 변경 권고) / mid=중립.

**추천 자세 Strategy 패턴 (2026-06-12)** — robot kinematic 별 추천 전략 분리. [`next_pose_planner.py`](backend/modules/calibration/next_pose_planner.py) 의 `PoseRecommendationStrategy` Protocol + `RecommendContext` dataclass + `make_strategy(name)` factory:

- **`JointPerturbationStrategy`** (5DOF, OMX-F) — `recommend_joint_sample`. current 자세 위 joint perturbation (J1/J2/J3/J4/J5 ±10°~30°) + FK + visibility hard filter + 기존 캡처 자세와의 joint-space distance 다양성 score. **"로봇이 갈 수 있는 자세 중 좋은 걸 고른다"** — IK 안 풀고 forward only → robot kinematic manifold 안에서만 sample → 항상 reachable.
- **`GeometryStrategy`** (6DOF, SO-101) — `recommend_geometry` (기존). board 주변 sphere shell anchor 5개 + IK + visibility. 임의 R 만들 수 있는 robot 에 자연.

`robots.yaml::pose_recommend_strategy` SSOT — `joint_perturbation` | `geometry`. [`RobotRegistry`](backend/core/robot/robot_registry.py) 가 `RobotConfig.pose_recommend_strategy` 로 노출. [`calibration_node._compute_recommendations`](backend/nodes/application/calibration_node.py) 가 strategy factory 통해 호출. 발견 — OMX-F 5DOF + wrist yaw 없음 = recommend_geometry 의 임의 R IK 가 5 anchor 중 1만 풀림 → joint_perturbation 으로 5+ candidates 안정.

**Hand-Eye UX + Solver v3 (2026-06-18, [docs/handeye_ux_solver_v3_plan.md](docs/handeye_ux_solver_v3_plan.md))** — 스펙(일반 수학)을 실제 코드와 버무린 대형 개편. 핵심 철학: *시스템이 자세를 대신 정하지 않음 — 사용자가 좋은 데이터셋을 만들도록 실시간 안내.*

- **Phase 1 Traffic Light (실시간 capture-quality)** ([`capture_quality.py`](backend/modules/calibration/capture_quality.py)) — preview loop(5Hz)가 현재 pose 를 *기존 캡처와 비교*해 🟢/🟡/🔴 판정: 검출 + tilt + **pose/rotation/translation diversity**. 토크오프로 움직이는 동안 "지금 찍으면 좋은 데이터셋이 되나" 실시간 안내 (검출+tilt 만이 아니라 **다양성까지**). `CALIB_HANDEYE_PREVIEW` payload 의 `capture_verdict`/`capture_reasons`. frontend [`CaptureGuideOverlay`](frontend/src/components/panels/calibration/parts/CaptureGuideOverlay.tsx) 가 색+사유 표시. 캡처 없을 땐 비교 대상 없어 거의 🟢.
- **Per-parameter observability** ([`observability_params.py`](backend/modules/calibration/observability_params.py)) — BA data residual Jacobian → Fisher 정보행렬 (Bayesian CRLB: `C=(JᵀJ/σ²+I)⁻¹`, prior 정규화). block (handeye_rot / handeye_trans / **joint_offset / link / sag** — 카메라만 아님) 별 식별성 score ∈[0,1]. 기존 `observability.py`(geometry A/B/mid)와 별개 — *parameter가 식별되느냐*. `CALIB_HANDEYE_PARAM_OBSERVABILITY` topic, frontend [`ParamObservabilityCard`](frontend/src/components/panels/calibration/parts/ParamObservabilityCard.tsx) block 색 dot (OK/WEAK/INSUFFICIENT, 수치 X).
- **Staged BA gating** — capture 개수 아니라 *식별성* 으로 block freeze. `bundle_adjust_hand_eye_physical_sag_irls(frozen_blocks=)`. handeye 항상 unlock, joint/link/sag 는 observability ≥ `OBS_UNLOCK_*` 일 때만 추정 (아니면 0 prior 고정). 정상 데이터=전부 unlock=무회귀, 병리 데이터=freeze 안전망. residual 수식은 `bundle_adjust.py` 공유 함수 (`physical_sag_data_residual`/`_unpack`/`_pack`/`physical_sag_block_indices`) **SSOT** — BA 2종 + observability 동일 모델.
- **Phase 1/2 분리** — `_RobotState.phase`: collection(Phase 1, geometry only, **auto-BA 안 돔** → 캡처 backlog 방지 + 스펙) → refinement(Phase 2). 전이 = `begin_refinement` 서비스(초기 solve, 옛 `multi_start`). frontend 버튼 "자동 추천 시작 (N/8)" — `MIN_POSES_FOR_TRUSTED_SIGMA`(8) 도달 시 활성.
- **sag joint SSOT** — `robots.yaml::sag_joint_motor_ids` → `RobotConfig` → `HandEyeCalibration(sag_arm_indices=)` + `SagCorrectedKinematics(sag_joint_motor_ids=)`. OMX-named 하드코딩 3곳 제거 → **5축/6축 동일 코드, yaml 값만 분기**.
- **Ghost robot preview primitive** — [`RobotModel`](frontend/src/components/scene/RobotModel.tsx) `tint` prop + [`RobotPreviewLayer`](frontend/src/components/scene/RobotPreviewLayer.tsx) (orange 0.4 opacity 두 번째 마운트) + [`previewStore`](frontend/src/domain/stores/preview.ts) (`setGhost`). **calibration 전용 아닌 공통 primitive** ([docs/pose_library_design.md](docs/pose_library_design.md) §5). 추천 후보 목록([`PoseCandidates`](frontend/src/components/panels/calibration/parts/PoseCandidates.tsx))에서 **클릭한 1개만** ghost 표시(동시 다중 X — 헷갈림 방지) → 토크오프 **수동 매칭** (자동주행 `[이동]` / 명시신호 `[👎]` 제거).
- **테스트** — [`test_capture_quality.py`](backend/tests/test_capture_quality.py)(traffic light G/Y/R) + [`test_observability_params.py`](backend/tests/test_observability_params.py)(Fisher/gating, 실 SO-101 σ=0.837°<GOOD, skip 가능 npz 픽스처) + **headless 캘 e2e** [`test_calibration_e2e.py`](backend/tests/test_calibration_e2e.py) (mock ChArUco eye-in-hand 시뮬 [`sim_board.py`](backend/modules/calibration/sim_board.py) + `camera_node_mock` `CALIB_SIM_BOARD` env → intrinsic→capture→BA→gating→observability→commit) + **분산 sim e2e** [`test_calibration_distributed_sim.py`](backend/tests/test_calibration_distributed_sim.py)(3 프로세스). 버그 2개 fix (geometry observability `np.array(dict)`, recommendations `KeyError(0)`).

**캘 유저 플로우 (구현됨)**: ① [캘 시작] → 토크오프 손으로 자세 → overlay 실시간 🟢/🟡/🔴 (다양성 포함) → 🟢에서 [캡처], 8장까지. ② [자동 추천 시작 (8/8)] → `begin_refinement` 초기 solve → Phase 2. ③ σ + 식별성 카드(5블록) 보며, 추천 후보 **클릭→고스트** → 토크오프 수동 매칭 → 🟢 [캡처] 반복. ④ [COMMIT] → DB active → **재시작**으로 적용 (SO-101 D405 = 공장 intrinsic 자동 seed, 내부캘 X).

### Frontend stores & 3D 워크스페이스

상태는 [frontend/src/domain/stores/](frontend/src/domain/stores/)의 Zustand store로 분리 (`robotStore`, `cameraStore`, `motionStore`, `taskStore`, `taskResultStore`, `detectorStore`, `systemStore`, `sceneStore`, `scene3DStore`).

**페이지 구조** ([docs/multi_robot_phase2_frontend.md §2](docs/multi_robot_phase2_frontend.md) — Slice B 완료):

| URL | 페이지 | 설명 |
|---|---|---|
| `/` | Dashboard | 시스템 운영 overview (Robots Online / System metrics) |
| `/robots/:id` | RobotsLayout (shared) | focus mode — 한 robot 불투명, 나머지 dim. R3F + meta 만 마운트, panel 은 mode 별 Outlet |
| `/robots/:id/move` | + RobotMoveMode | Robot State + Motion + Scene Controls |
| `/robots/:id/calibrate` | + RobotCalibrateMode | Robot State (Torque/Home/Jog 흡수) + Calibration (result) + Calibration Camera + Hand-Eye + Intrinsic + Rollback + Scene Controls |
| `/world` | WorldPage | multi-robot overview, focus=null |
| `/tasks/:name` | TasksPage | task multi-robot 실행 (focus=null + prompt/progress/camera panel). `scan` task 는 rgbd capability robot 만 (Task.required_capabilities filter) |
| `/settings` | Settings | 그대로 |

[RobotsLayout](frontend/src/pages/RobotsLayout.tsx) / `TasksPage` 둘 다 [dockview](https://dockview.dev/) 플로팅 패널 위에 [react-three-fiber](https://r3f.docs.pmnd.rs/) 씬 + [urdf-loader](https://github.com/gkjohnson/urdf-loaders). 패널 컴포넌트는 [frontend/src/components/canvas/dockview/panelComponents.ts](frontend/src/components/canvas/dockview/panelComponents.ts) 에 등록, mode 별 panel 셋은 [frontend/src/pages/robotModes/](frontend/src/pages/robotModes/) 의 RobotMoveMode / RobotCalibrateMode 에 선언 (RobotScanMode 자리는 scene3d_decoupling.md 4-노드 분리 작업으로 제거 — scan workflow 는 TasksPage 의 `scan` task 로 흡수). URDF에 들어가는 조인트각은 `MOTOR_STATE_JOINT`에서 `(position - 2048) / 4095 * 2π` 형태로 도출 (`units.raw_to_rad`와 일치).

**Mode sub-routes + capability 기반 sidebar** — `/robots/:id` 는 [RobotsLayout](frontend/src/pages/RobotsLayout.tsx) (R3F 마운트, mode 전환 시 unmount 안 됨) + `<Outlet>` 에 mode 컴포넌트. 사용 가능한 mode 는 `robots.yaml::capabilities` 의 *subset* — `SIDEBAR_MODES = ["move", "calibrate"]` 가 hardcoded list ([Sidebar.tsx](frontend/src/components/shared/Sidebar.tsx)). 다른 capability (`rgbd`, `gamepad`) 는 sensor/control 어휘라 sidebar entry 자리 아님 — point cloud 토글은 Scene Controls 의 mode-무관 자리, scan workflow 는 TasksPage 의 `scan` task. `RobotInfo` Pydantic → `pnpm gen:types` → frontend generated types. `/robots/:id` 인덱스는 capabilities 와 SIDEBAR_MODES 교집합의 첫 mode 로 redirect.

> dockview 라우팅 leak (이전에 "진행 중" 으로 적혀 있던 자리) 은 **해결됨** — 실제 root cause 는 [RobotModel.tsx:113-134](frontend/src/components/canvas/3d/RobotModel.tsx#L113-L134) 의 emitTCP effect 가 인라인 onTCPMatrix prop 을 dep 으로 두고 안에서 emit → 부모 setState → 새 ref → effect 재실행 → 무한 루프 → reconciler stall → 라우팅까지 정지. ref-stash 패턴 + dep 에서 prop 제거로 fix (commit f15a20b). react-rnd 마이그레이션 / dockview 라이브러리 교체 모두 불필요.

**Multi-robot 시각화** — [RobotLayer](frontend/src/components/canvas/3d/RobotLayer.tsx) 가 `useRobots()` (backend `/robots` fetch) 의 N robot 동시 마운트. `robots.yaml` 의 `base_pose: {x, y, z, yaw_deg}` 로 world frame 분리. focus 모드는 others dim (default opacity 0.25). joint state 는 *focus robot 만* `robotStore` 에서 받고, 나머지는 home pose — §4 결정 3 의 "store 임시 호환 코드" (충돌 자리 생기면 dict 화).

**Backend SSOT endpoint** ([bridge/zenoh_bridge.py](backend/bridge/zenoh_bridge.py)):
- `GET /robots` — robots.yaml 의 list + default + base_pose. frontend `useRobots()`.
- `GET /tasks` — `task_node.TASK_REGISTRY.keys()`. frontend `useTasks()`.
- `GET /system` — psutil CPU/Mem + zenoh peers. frontend `useSystemMetrics()` 5초 polling.
- `GET /robots/{robot_id}/camera/stream` — robot-scoped MJPEG.

Task step 결과 시각화는 `taskResultStore` 가 `TASK_STEP_RESULT` 누적 → [TaskResultLayer.tsx](frontend/src/components/canvas/3d/TaskResultLayer.tsx) 가 type 별 dispatch (Detection→sphere, Position3→marker). 새 typed value 추가 시 layer 한 줄 추가로 끝 — task 코드는 안 건드림.

## 규약

- 로그 메시지, 주석, docstring은 한국어 자유롭게 — 주변 코드의 스타일을 유지.
- Backend는 **ruff**(line-length 88, target py311) + **pyright**, Frontend는 ESLint + Prettier + `editor.formatOnSave` (VS Code).
- 프론트엔드 import는 `@/`alias = [frontend/src/](frontend/src/) ([frontend/vite.config.ts](frontend/vite.config.ts)).
- 서비스 핸들러는 반드시 `{"success": bool, "message": str, "data": dict}` 반환 — 브릿지와 `BridgeClient.callService`가 이 모양에 의존.
- 모델 가중치(`*.pt`, `*.pth`), `.venv/`, `node_modules/`, `frontend/dist/`, `uv.lock`, per-instance 런타임 산출물(`robot/instances/*/scans/`, `robot/instances/*/meshes/`, `robot/instances/*/logs/`) 는 gitignore.

### 프로젝트 design decision (다른 PC / 새 세션이 알아야 할 critical context)

- **self-play 는 폐기됨** — `backend/modules/task/self_play/`, `self_play_pick.py`, `docs/self_play_pick.md` 는 legacy. 본 방향은 pick_and_place + deterministic IK + 캘/자세 정확도 직접 강화. self-play 점프 제안 / 신규 기능 추가 금지
- **Study task 에선 industry standard 도구 / 플로우 우선** — RL / 실험 / 시뮬레이션 도구 추천 시 인프라 재사용 ROI 보다 산업 표준 (MuJoCo / Isaac / PPO / Stable-Baselines3 등) 우선. 결과 잘 만들기보다 "표준 단계 다 밟아보기" 자체가 study output. 비유: "회사에선 React 쓰는데 사이드 프로젝트 간단하다고 HTML 만 짜면 학습 효과 없잖아"
- **DSL "레고화" = typed Slot 기반, visual editor (Blockly 등) X** — [docs/step_dsl.md](docs/step_dsl.md) 참조. 코드 조립성 가져오기, UI 도구 도입 X
- **URDF TCP link 컨벤션: 모든 robot type 의 URDF 는 `tcp` 이름의 link 를 가져야 함** — UR `tool0` 와 같은 패턴. backend [pybullet_kinematics.py](backend/modules/kinematics/adapters/pybullet_kinematics.py) 의 `TCP_LINK_NAME` / frontend [config.ts](frontend/src/lib/robot/config.ts) 의 `TCP_LINK_NAME` 둘 다 `"tcp"` 하드코드 — yaml 추가 / per-robot config X. 새 robot type 통합 시 URDF 의 wrist link 끝에 fixed joint child 로 `<link name="tcp"/>` 박을 것 (없으면 `PybulletKinematics` 부팅 시 즉시 fail-fast). [docs/multi_robot_architecture.md §3.1](docs/multi_robot_architecture.md) 참조
