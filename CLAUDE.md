# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

OMX Control — a 6DOF robot-arm control stack for the **OMX_F** (OpenMANIPULATOR-X custom variant). The backend drives Dynamixel servos, runs camera + YOLO detection + hand-eye calibration, plans trajectories with Ruckig, and solves FK/IK in PyBullet against the URDF in [robot/urdf/omx_f/](robot/urdf/omx_f/). The frontend is a React app for teleoperation, calibration, and a 3D digital-twin workspace.

## Common commands

### Backend (Python 3.11, uv-managed, run from [backend/](backend/))

```powershell
cd backend
uv sync                  # install/refresh deps from pyproject.toml + uv.lock
uv run python main.py    # start ZenohSession + nodes + FastAPI bridge on :8000
uv run ruff check .      # lint
uv run pyright           # type-check
```

Notes:

- The default port is `COM6` on Windows / `/dev/ttyUSB0` on Linux — set in [robot/config/motors.yaml](robot/config/motors.yaml).

### Frontend (pnpm, run from [frontend/](frontend/))

```powershell
cd frontend
pnpm install
pnpm dev        # vite dev server on :5173 (CORS allowed by bridge)
pnpm build      # tsc -b && vite build
pnpm lint       # eslint
```

Bridge URL is read from `VITE_WS_URL` / `VITE_BASE_URL`, defaulting to `ws://localhost:8000/ws` and `http://localhost:8000` ([src/constants/index.ts](frontend/src/constants/index.ts)).

## Architecture

### Two-layer transport: Zenoh (backend internal) + WebSocket (browser-facing)

All backend nodes communicate over **Zenoh** pub/sub and queryables. There is a single process-wide `ZenohSession` singleton ([backend/core/zenoh_session.py](backend/core/zenoh_session.py)) that every node grabs via `ZenohSession.get()`. Topics carry JSON payloads (camera frames are raw JPEG bytes); services use Zenoh queryables with `{success, message, data}` reply envelopes.

The browser cannot speak Zenoh, so [backend/bridge/zenoh_bridge.py](backend/bridge/zenoh_bridge.py) runs a FastAPI app that:

- Exposes `/ws` — a WebSocket that translates `subscribe` / `unsubscribe` / `publish` / `service` messages into Zenoh ops in both directions.
- Exposes `/camera/stream` — an MJPEG `multipart/x-mixed-replace` stream fed from the `omx/camera/stream/raw` Zenoh topic.
- Mounts [robot/](robot/) (URDFs, meshes, calibration .npz) at `/robot` for the frontend's URDF loader.
- The bridge maintains an `_ALWAYS_SUBSCRIBE` list of "core" topics it eagerly subscribes to and rebroadcasts to all matching WS clients.

The frontend wraps this in a singleton `BridgeClient` ([frontend/src/api/bridge.ts](frontend/src/api/bridge.ts)) that uses `ReconnectingWebSocket`, multiplexes per-topic subscribers, and pairs `service` calls with a `request_id` for promise resolution. `useBridge` ([frontend/src/hooks/useBridge.ts](frontend/src/hooks/useBridge.ts)) is mounted once in `App.tsx` and routes incoming topic data into the relevant Zustand store.

### Topic & service registry — keep in sync

Topic / service keys are declared **twice**: [backend/core/topic_map.py](backend/core/topic_map.py) (`Topic`, `Service` classes) and [frontend/src/constants/topics.ts](frontend/src/constants/topics.ts) (`Topic`, `ServiceKey`). When adding or renaming a route, update both files — the strings must match exactly (e.g. `omx/motion/srv/move_l`).

### Node pattern

All backend nodes inherit from `BaseNode` ([backend/core/base_node.py](backend/core/base_node.py)), which provides:

- `create_subscriber(topic, callback)` — JSON-decoded pub/sub
- `create_service(key, handler)` — declares a Zenoh queryable; handler returns `{success, message, data}`
- `call_service(key, data)` — sync Zenoh `get()` with a 5s default timeout
- `publish(topic, data)`, `log(level, msg)`, automatic 1Hz heartbeat on `omx/system/heartbeat`

Lifecycle is `start()` → background heartbeat thread + (optionally) node-specific worker threads → `stop()` undeclares all subscribers/queryables. Most nodes register their hardware loops in their own `start()` override (see e.g. `MotorNode._state_loop` at 20Hz, `CameraNode._stream_loop` at 30 FPS, `DetectorNode._detection_loop` at 5 FPS).

### Singletons used across nodes

- `ZenohSession` — single Zenoh session per process.
- `JointStateCache` ([backend/core/joint_state_cache.py](backend/core/joint_state_cache.py)) — subscribes once to `MOTOR_STATE_JOINT` and exposes the latest joint angles in radians via `get_joint_angles_rad(arm_cfgs)`. Multiple nodes (motion, task, detector) call `_cache.subscribe(self)` to share it.
- `PybulletSolver` ([backend/modules/kinematics/solver.py](backend/modules/kinematics/solver.py)) — DIRECT-mode PyBullet client loaded with the URDF; provides thread-safe `fk()`, `ik()`, `fk_to_matrix()`. The end-effector link is identified by name `end_effector_link`.

### Motion pipeline

`MotionNode` ([backend/nodes/motion_node.py](backend/nodes/motion_node.py)) accepts `move_j` / `move_l` / `move_c` / `move_p` / `move_tcp` services. Validation + execution are split into `MotionCommand` subclasses ([backend/modules/kinematics/motion_commands.py](backend/modules/kinematics/motion_commands.py)); the actual interpolation runs in `TrajectoryRunner` ([backend/modules/kinematics/trajectory_runner.py](backend/modules/kinematics/trajectory_runner.py)) which uses **Ruckig** for jerk-limited profiles and publishes progress on `omx/motion/state/trajectory`. The runner emits joint commands via the `publish_cmd` callback that `MotionNode` wires to `MOTOR_CMD_JOINT`.

The arm is **5DOF** for kinematics (motor IDs 1–5); ID 6 is the gripper, filtered out via `core.common.GRIPPER_ID`. Conversions live in [backend/core/units.py](backend/core/units.py) — Dynamixel raw is `0..4095` centered at `2048` (= 0°). Each motor in [robot/config/motors.yaml](robot/config/motors.yaml) has a `reverse` flag and per-joint `limit.min/max` raw clamps that `rad_to_raw` enforces.

### Task system

Tasks are declarative step lists ([backend/modules/task/step_types.py](backend/modules/task/step_types.py): `MoveTCPStep`, `DetectStep`, `GripperStep`, `HomeStep`, `WaitStep`) factored into the `TASK_REGISTRY` in [backend/nodes/task_node.py](backend/nodes/task_node.py). `StepExecutor` runs each step (calling motion/detector/motor services), `TaskRunner` is the state machine (`run/pause/resume/stop`), and progress is published on `omx/task/state`. `DetectStep` writes its result into a context dict (`output_key`) that subsequent `MoveTCPStep`s consume via `position_key` + `offset` — see [backend/modules/task/tasks/pick_and_place.py](backend/modules/task/tasks/pick_and_place.py) for the canonical example.

### Detection → world-coordinate pipeline

`DetectorNode._handle_detect` ([backend/nodes/detector_node.py](backend/nodes/detector_node.py)) chains: YOLO centroid → `cv2.undistortPoints` with the saved intrinsic → `MOTION_GET_TCP` for the current end-effector pose → solves `Z_cam` from the constraint that the object lies on the base-frame plane `Z=0` → returns the object position in the base frame. This requires **both** [robot/calibration/intrinsic.npz](robot/calibration/intrinsic.npz) and [robot/calibration/hand_eye.npz](robot/calibration/hand_eye.npz); `load_calibration().is_ready()` gates the service.

### Frontend stores & 3D workspace

State is sliced into Zustand stores under [frontend/src/store/](frontend/src/store/) (`robotStore`, `cameraStore`, `motionStore`, `taskStore`, `detectorStore`, `systemStore`, `sceneStore`). The `Workspace3D` page ([frontend/src/pages/Workspace3D.tsx](frontend/src/pages/Workspace3D.tsx)) renders a `react-three-fiber` scene with `urdf-loader` over a `dockview` floating-panel layout — panels are registered in [frontend/src/components/workspace3d/dockview/panelComponents.ts](frontend/src/components/workspace3d/dockview/panelComponents.ts). Joint angles fed to URDF are derived from `MOTOR_STATE_JOINT` in `(position - 2048) / 4095 * 2π` form (matches `units.raw_to_rad`).

## Conventions

- Korean is used freely in log messages, comments, and docstrings — keep that style when editing nearby code.
- Backend uses **ruff** (line-length 88, target py311) and **pyright**; frontend uses ESLint + Prettier with `editor.formatOnSave` configured for VS Code.
- Frontend imports use the `@/` alias mapped to [frontend/src/](frontend/src/) (see [frontend/vite.config.ts](frontend/vite.config.ts)).
- Service handlers must always return `{"success": bool, "message": str, "data": dict}` — the bridge and `BridgeClient.callService` rely on this shape.
- Model weights (`*.pt`, `*.pth`) and `.venv/`, `node_modules/`, `frontend/dist/`, `uv.lock` are gitignored.

## In-progress: Intel RealSense D405 migration

The OpenCV-based USB camera ([backend/modules/camera/capture.py](backend/modules/camera/capture.py)) is being replaced by an **Intel RealSense D405**. D405 is treated as a general-purpose RGBD device, not just a pick-and-place sensor — eventual uses include live point cloud preview in Workspace3D, PLY snapshots for inspection, ICP registration against reference PLYs, and replacing the Z=0 plane assumption in the detector with direct depth lookup. This is a phased rollout — earlier phases must be merged and validated before later ones land.

### Phased plan

1. **RGB-only swap (smallest blast radius).** Keep `CameraNode`, all topics/services, and the frontend unchanged. Replace the internals of [CameraCapture](backend/modules/camera/capture.py) only — `cv2.VideoCapture` → `pyrealsense2` color stream. Add `pyrealsense2` to [backend/pyproject.toml](backend/pyproject.toml). At the end of this phase: pull D405 factory intrinsics into [robot/calibration/intrinsic.npz](robot/calibration/intrinsic.npz) (skip checkerboard re-cal — D405 factory cal is accurate), and **redo hand-eye calibration** since the physical mount has changed — the existing `hand_eye.npz` is invalid and detector results cannot be trusted until this is regenerated.

2. **Binary WebSocket payload protocol.** Extend [backend/bridge/zenoh_bridge.py](backend/bridge/zenoh_bridge.py) and [BridgeClient](frontend/src/api/bridge.ts) to support binary WS frames alongside the current JSON envelope. JSON-only forces base64 encoding for point cloud buffers (~33% overhead + parse cost), which is impractical at the rates we want. The MJPEG `/camera/stream` HTTP endpoint stays as-is.

3. **Live point cloud preview.** Introduce a new `PointCloudNode` plus a shared `RealsenseCapture` singleton (same pattern as [JointStateCache](backend/core/joint_state_cache.py)) so `CameraNode` (color) and `PointCloudNode` (depth + cloud) can both read from one `rs.pipeline()` — D405 is a single physical device, two pipelines cannot coexist. Live stream uses Open3D voxel downsampling (~5–10mm grid → ~10k points, 5–10Hz) emitted as binary `Float32 XYZ + UInt8 RGB`. Frontend renders via react-three-fiber `<points>` in a new dockview panel ([panelComponents.ts](frontend/src/components/workspace3d/dockview/panelComponents.ts)); toggleable, **off by default**. Backend exposes a service to enable/disable + adjust voxel size so it doesn't run when nothing is listening.

4. **PLY snapshot capture/load.** "Capture" button → full-resolution one-shot point cloud → saved to `robot/scans/scan_<timestamp>.ply` → also pushed once to the frontend as a static layer in the scene (separate from the live preview layer). Add services to list and re-load saved PLYs from `robot/scans/` and `robot/models/`. PLY I/O via Open3D `read_point_cloud` / `write_point_cloud`.

5. **ICP registration.** Open3D-based ICP (point-to-point and/or point-to-plane) as a `PointCloudNode` service. Inputs: source PLY path + target (current cloud or PLY path). Output: 4×4 transform. Frontend visualizes source/target overlaid in Workspace3D, with toggle for pre/post-transform.

6. **Detector depth-lookup swap.** Replace the plane-Z=0 inversion in [detector_node.py:120-135](backend/nodes/detector_node.py#L120-L135) (`Z_cam = -t_total[2] / denom`) with a direct depth lookup at the centroid pixel (NxN median around the centroid for noise robustness). Removes the on-plane-only constraint of object detection.

### Naming / structural decisions already made

- The new node is **`PointCloudNode`**, not `PerceptionNode` — matches the existing concrete-naming convention (`MotorNode`, `MotionNode`, `DetectorNode`). Topic/service namespace will likely be `omx/pointcloud/...`.
- Existing `CameraNode` is **not** renamed or replaced. After phase 3 it owns RGB + status; `PointCloudNode` owns depth + cloud + PLY + ICP. Both share one `RealsenseCapture` singleton.
- **Open3D** is the chosen library for PLY I/O, voxel downsampling, normal estimation, and ICP. Don't roll these by hand.
- Live preview is **not** PLY streaming. PLY only appears at capture/load events. The live track is just an `XYZ + RGB` binary buffer — PLY is a file format for persistence, not a streaming format.
- D405 RGB-only mode (phase 1) is exercised first to de-risk hardware/driver/install issues before adding depth complexity. If something breaks in phase 1, blast radius is one file.

### Files to keep in sync as phases land

- [backend/core/topic_map.py](backend/core/topic_map.py) ↔ [frontend/src/constants/topics.ts](frontend/src/constants/topics.ts) — every new topic/service in both files, exact string match.
- [robot/calibration/](robot/calibration/) — `intrinsic.npz` and `hand_eye.npz` must be regenerated in phase 1.
- `.gitignore` — `robot/scans/*.ply` should be ignored; `robot/models/*.ply` may be committed as ICP reference assets.

### 현재 진행상황

- D405 연결됨
- 핸드아이 캘리브레이션 진행
- Workspace3D LivePointCloud 작업중 (binary websocket 구현했음)
- snapshot 아직 진행 안함

### issue

- 포인터 클라우드 데이터가 잠깐 보이고

```
2026-05-10 05:22:20,403 [WARNING] modules.dynamixel.driver - SyncRead 실패: [TxRxResult] There is no status packet!
2026-05-10 05:22:20,453 [WARNING] nodes.motor_node - 모터 1(joint1) 위치 읽기 실패
2026-05-10 05:22:20,454 [WARNING] nodes.motor_node - 모터 2(joint2) 위치 읽기 실패
2026-05-10 05:22:20,455 [WARNING] nodes.motor_node - 모터 3(joint3) 위치 읽기 실패
2026-05-10 05:22:20,455 [WARNING] nodes.motor_node - 모터 4(joint4) 위치 읽기 실패
2026-05-10 05:22:20,455 [WARNING] nodes.motor_node - 모터 5(joint5) 위치 읽기 실패
2026-05-10 05:22:20,456 [WARNING] nodes.motor_node - 모터 6(gripper_joint_1) 위치 읽기 실패
```

에러 발생

- 실시간 포인트 클라우드 기능 안키면 동작 잘함
- pointcloud 들어오면 queue 에 최신 프레임만 유지 -> 별도 sender task 가 천천히 소비 구조로 수정하면 실시간은 나오는데 다이나믹셀 에러는 여전히 뜸
- D405를 PC 뒤에 꽂으면 마우스가 먹통됨
- 별도 sender task 가 천천히 소비 구조로 가는건 맞는지 아키텍처 검증 필요
  - 필요성 검증 (라즈베리 파이로 분리해도 필요한가?)
  - 필요하다면 지금 짜여져 있는 코드는 괜찮게 구현되어 있나?
- 어떻게 usb 분배해야 마우스 키보드 모터 다 잘 동작할지 아키텍처 설계 필요 (모터는 라즈베리 파이 적용?)
