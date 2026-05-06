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
