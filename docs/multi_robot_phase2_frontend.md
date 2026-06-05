# Multi-Robot Phase 2 — Frontend / Namespace / UX

Phase 1 (foundation) 완료 후 남은 자리. [multi_robot_architecture.md §12](multi_robot_architecture.md) 의 Phase 2 의 *frontend / UX / namespace* 슬라이스.

**상태 (2026-06-05 갱신):**
- §1 namespace — **완료** (`omx/` → `horibot/{robot_id}/...`, BaseNode.r() + BridgeClient 자동 expand)
- §2 페이지 역할 — Dashboard / Robots/`<id>` / World / Tasks/`<name>` 4-페이지 + **mode sub-route 추가** (`/robots/:id/{move,calibrate,scan}`, robots.yaml capabilities SSOT)
- §3 멀티로봇 UX — §2 에 흡수

**검증 가이드**: [slice_abc_verify.md](slice_abc_verify.md) 에 집에서 진행할 순차 절차.

## §0. 컨텍스트

### 완료된 자리 (본 작업의 *전제*)

- **Phase 1 foundation** ([multi_robot_walkthrough.md](multi_robot_walkthrough.md))
  - 폴더 split: `robot/<type>/` (URDF/mesh) + `robot/instances/<id>/` (calibration / scans / logs)
  - `RobotRegistry` ([robot/robots.yaml](../robot/robots.yaml) SSOT) → `RobotConfig` / `get_iksolver(id)` / `get_motor_backend(id)` / `get_camera_capture(id)`
  - Protocol abstraction: `IKSolver` / `MotorBackend` / `CameraCapture`
  - Coordinates → `dict[robot_id]` (joint/link/sag/tool)
  - **N=1 환경에서 SO-101 도착 시 SWAP 가능** (entry + FeetechBackend adapter 추가)

- **Typed messaging** ([typed_messaging.md](typed_messaging.md))
  - 모든 토픽/서비스 payload pydantic (면제 자리 제외)
  - `backend/api_contract.py` SSOT → frontend codegen 자동 (`pnpm gen:types`)
  - `BridgeClient` generic 화 (subscribe / publish / callService typed)

### 본 문서 scope (Phase 2 의 일부)

| § | 항목 | 성격 |
|---|---|---|
| §1 | Zenoh namespace 개편 | 기술 (transport 설계) |
| §2 | 페이지 역할 기획 | 기획 (UX 요구사항) |
| §3 | 멀티로봇 UX | 기획 + UX |
| §4 | 프론트 데이터 플로우 재정비 | 기술 (§1+§2+§3 의 downstream) |

§2/§3 (기획) 이 §1/§4 (코드 결정) 의 *upstream* — 기획 먼저 잡고 코드 결정 내려가는 게 자연스러움.

### Phase 2 의 *다른* 자리 (본 문서 scope 밖)

- [distributed_topology.md](distributed_topology.md) — hori1/2/3 Pi 3대 분산
- Coordinator (multi-robot 동시 동작 조율) — 별도 슬라이스
- dual-arm 시퀀스 (e.g. 한 로봇이 picks, 다른 로봇이 places) — 별도

## §1. Zenoh Namespace 개편 [DECIDED]

### 전제 (§2 → §1 dependency)

§2 결정이 §1 옵션을 거의 자동으로 좁힘:

| §2 결정 | §1 함의 |
|---|---|
| World = SSOT, multi-robot 한 scene | Frontend 가 N robot 데이터 동시 구독 → **key 로 robot 필터링 필수** |
| Robot = Navigation (`/robots/<id>`) | URL prefix ↔ topic prefix 자연 매핑 |
| Layer 도메인 단위 (RobotLayer 가 N robot 다룸) | topic prefix 로 robot 별 데이터 분리되어야 layer 깔끔 |
| Task = global | task 토픽 prefix 없음 |
| Camera 가 wrist 마운트 (instance 별) | Camera 도 robot-scoped |

§1 의 원래 4개 옵션 중 **옵션 2 (도메인별 차등)** 외에는 §2 와 충돌:
- 옵션 1 (prefix 무차별): task / heartbeat 도 prefix → "Task = global" 어김
- 옵션 3 (flat + payload): subscriber 필터링 불가 → multi-robot scene 부담 ↑
- 옵션 4 (하이브리드): 옵션 2 + 옵션 3 — payload validation 자리는 결정문 §4 에 따로

### 결정문 (irreversible)

1. **프로젝트 prefix: `omx` → `horibot`**
   - `omx` 는 OMX_F robot 타입 leftover. 멀티로봇 generalization 에 부적절. 현 프로젝트명 = `horibot` ([CLAUDE.md](../CLAUDE.md) 상단, working directory).

2. **도메인별 차등 namespace**:

   ```
   robot-scoped:  horibot/<robot_id>/motor/...        (state/joint, cmd/joint, srv/enable, ...)
                  horibot/<robot_id>/motion/...       (state/trajectory, srv/move_j, move_l, move_tcp, ...)
                  horibot/<robot_id>/camera/...       (state/status, stream/raw, stream/depth_frame, srv/set_depth_stream, ...)
                  horibot/<robot_id>/calib/...        (srv/handeye/capture, srv/handeye/commit, state/...)
                  horibot/<robot_id>/pointcloud/...   (stream, state, srv/capture, srv/build_mesh, ...)
                  horibot/<robot_id>/detector/...     (state, srv/detect)

   global:        horibot/task/...                    (tree, state, step_result, srv/run, srv/pause, ...)
                  horibot/system/heartbeat
                  horibot/system/log
   ```

   Detector 는 robot-scoped — 한 robot+camera 쌍에 종속 (CAMERA_STREAM_RAW 구독 + MOTION_GET_TCP 호출 + 그 robot base frame 으로 detection 출력).

3. **Prefix 위치: `horibot/<robot_id>/<domain>/...`**
   - `horibot/robot/<id>/...` 같은 `robot` 키워드 X. robot_id 자체가 `omx_f_0` / `so101_0` 처럼 unique 해서 키워드 불필요. ROS2 `/<robot_name>/joint_states` 와 같은 패턴.

4. **Payload 의 `robot_id` 필드 ([`BaseRobotMessage`](../backend/core/transport/messages/base.py)): 유지**
   - key 의 prefix 와 redundant 하지만 validation / debug / log 용. 제거는 reversible — 필요해지면 그때.

5. **외부 고정 카메라 추가 시**: 별도 namespace `horibot/world/camera/<cam_id>/...` (아직 없음 — 도착하면 추가).

6. **Heartbeat / system_log**: payload 의 `node_id` 로 식별. 노드가 어느 robot 담당인지는 node identity 로 역추적.

7. **Migration: 한 번에 갈아엎기**
   - N=1, solo project → backward compat shim 불필요
   - 같은 슬라이스에서 `omx → horibot` + `<robot_id>` prefix 동시 적용
   - 프로토타입 (§2) *전* 에 진행 — 안 그러면 새 코드 `omx/...` 로 짜고 바로 다시 쓰기

### 영향 자리 (mechanical refactor)

- [`backend/core/transport/topic_map.py`](../backend/core/transport/topic_map.py) — `Topic` / `Service` 클래스 키 문자열 전부 갱신.
  - robot-scoped 키: `@staticmethod` 패턴. e.g. `Topic.motor_state_joint(robot_id: str) -> str: return f"horibot/{robot_id}/motor/state/joint"`.
  - global 키: 0-arg `@staticmethod` 또는 class attribute (호출 측 `Topic.task_state()` 또는 `Topic.TASK_STATE`). 일관성을 위해 *모두 staticmethod* 권장.
- [`frontend/src/constants/topics.ts`](../frontend/src/constants/topics.ts) — 동기 갱신. 같은 staticmethod 패턴 (TS 의 `static`).
- [`backend/bridge/zenoh_bridge.py`](../backend/bridge/zenoh_bridge.py) — `_ALWAYS_SUBSCRIBE` 자리. MJPEG HTTP 라우트 `/camera/stream` → **`/robots/<robot_id>/camera/stream`** (frontend URL `/robots/<id>` 와 일관, RESTful).
- 각 노드 (`backend/nodes/*.py`) — `BaseNode` publish / subscribe / service create 자리에서 `robot_id` 채움. `RobotConfig` 에서 가져옴.
- [`backend/core/transport/messages/base.py`](../backend/core/transport/messages/base.py) — `BaseRobotMessage.robot_id` 유지 (위 결정 4).
- typed_messaging codegen — robot_id key 자동 반영되는지 Slice A 시작 시 확인. 안 되면 codegen template 갱신 (이 자리만 진짜 deferred).

### 참조

- [multi_robot_architecture.md §토픽 namespace 재설계 candidate](multi_robot_architecture.md)
- ROS2 의 namespace 패턴 (`/<robot_name>/joint_states`) — 산업 표준

## §2. 페이지 역할 기획 [DECIDED]

### 한 문장 요약

> 로봇 여러 대가 있는 하나의 작업 셀(World)을 보여주는 3D 씬은 하나만 만들고, Robots / World / Tasks 페이지는 그 씬을 서로 다른 관점으로 보는 UI 프리셋이다.

### 결정문 (irreversible)

1. **World = SSOT** — 시스템의 현실 매핑 단위는 World 하나. Robot / Camera / PointCloud / Mesh / Object 모두 World 안에 존재. 캘리브레이션도 결국 `world ↔ robot` / `world ↔ camera` 관계 수정.
2. **Single `WorldScene`** — R3F scene graph 는 *한 벌*. 페이지마다 별도 scene 만들지 않음. 페이지 차이는 *camera preset + visible layers + side panels* 로 표현. (이유: scene 두 벌 만들면 6개월 안에 95% 동일해지면서 sync 지옥. 또 "현실이 하나니까 모델도 하나" 라는 도메인 reasoning.)
3. **Pages = 사용자 의도 단위** — `Dashboard / Robots / World / Tasks`. 각 페이지는 `(layer set, view state, panel set)` 의 preset.
4. **Task = global** — task 가 robot 을 포함 (task 자체는 robot-scoped 아님). Tasks 페이지에 robot selector 없음. §1 namespace 의 task scope 질문도 이 결정으로 환원.
5. **Focus Robot = Navigation** — `/robots/so101_0` URL 자체가 focus 를 인코딩. selector 별도 UI 없음. 메뉴에서 로봇 클릭 = focus 변경.
6. **Layer 단위 = 도메인 default** — `<RobotLayer />` / `<PointCloudLayer />` / `<MeshLayer />` / `<CalibrationLayer />` / `<DetectionLayer />` / `<TaskLayer />`. 객체 단위로 잘게 쪼개지 않음 (필요 시 internal sub-toggle). 이유: "import 한 줄 = 도메인 전체" 가 자연스러움.
7. **3D 객체 → Layer, 폼/버튼 → Panel** — 예: Calibration 의 `CameraFrustum / Checkerboard / HandEyeAxes` 는 `<CalibrationLayer />`, `Capture / Commit / Reject` 버튼은 `<CalibrationPanel />`.

### 화면 sketch

#### 왼쪽 메뉴

```
Dashboard
Robots
  ├─ omx_f_0
  ├─ so101_0
  └─ so101_1
World
Tasks
  └─ <task list>
```

Robots / Tasks 하위는 `robots.yaml` / task registry 에서 자동 enumeration. 로봇 / task 추가 = registry entry 추가 = 메뉴 자동 갱신.

#### `Robots > so101_0` (`/robots/so101_0`)

```
+-----------------------------------------+
|                                         |
|            WorldScene                   |
|        (camera = focus on so101_0)      |
|        (다른 로봇 dim / 숨김)            |
|                                         |
+-------------------+---------------------+
| Motion Panel      | Calibration Panel   |
| MoveJ / MoveL     | Capture / Commit    |
| Home / Teleop     | Status              |
+-------------------+---------------------+
```

- Layer set: `Robot / PointCloud / Calibration`
- View state: `focus=so101_0, cameraPreset=orbit_focus, dimOthers=true`
- Panels: `Motion / Calibration / Camera / Diagnostics`
- 명령 권한: focus robot 에만 허용 (다른 로봇 토크 실수 방지)

#### `World` (`/world`)

```
+------------------------------------------------+
|                                                |
|             WorldScene                         |
|       (camera = world_overview / free orbit)   |
|                                                |
|       Robot A           Bottle                 |
|             Robot B                            |
|                                                |
+------------------------------------------------+
[PointCloud] [Mesh] [Detection] [Trajectory]      ← layer toggle bar
```

- Layer set: `Robot / PointCloud / Mesh / Detection / Task`
- View state: `cameraPreset=world_overview, dimOthers=false`
- Panels: 없음 또는 layer toggle bar
- 병따기 데모 / 듀얼암 협업 시각화 무대

#### `Tasks > OpenBottleTask` (`/tasks/open_bottle`)

```
+----------------------+-------------------------+
|  Task Tree           |                         |
|                      |    WorldScene           |
|  OpenBottleTask      |    (task 에 참여하는    |
|   ├─ Pick(robot_a)   |     로봇 다 보임)        |
|   ├─ Hold(robot_b)   |                         |
|   ├─ Twist(robot_a)  |                         |
|   └─ Release(robot_b)|                         |
|                      |                         |
|  [Run][Pause][Step]  |                         |
+----------------------+-------------------------+
```

- Layer set: `Robot / Task / Detection`
- View state: `cameraPreset=world_overview` (task 가 multi-robot 이라)
- Panels: `TaskTree / TaskControl / TaskLog`
- **robot selector 없음** — task 가 자기 안에 robot 포함

#### `Dashboard` (`/`)

```
Robots Online: 2 / 3
  omx_f_0    OK
  so101_0    OK
  so101_1    Offline

Bridge       OK    Zenoh peers: 3
Camera       OK    CPU: 22%   Mem: 1.4GB
```

- 시스템 운영 상태 overview. 우선순위 낮음. 3D scene 없음.

### 첫 프로토타입 scope (그대로 구현, 확장 금지)

```
- robot/robots.yaml 에 가짜 SO101 entry: `type=omx_f` (URDF/mesh 복제 — 시각화만이라 충분), `id=so101_0`, `robot/instances/so101_0/` 폴더만 추가. 진짜 SO-101 type 폴더는 하드웨어 도착 또는 URDF 생성 시점에 별도
- 왼쪽 메뉴: robots.yaml 자동 enumeration
- WorldScene 생성: <RobotLayer /> 만 포함 (omx_f_0, so101_0 두 URDF 동시 띄움)
- /robots/<id> 라우팅 + focus 처리 (camera lookAt + 다른 로봇 opacity)
- /world 라우팅 (free orbit, 양쪽 다 보임)
- Page Preset / Layer registry / ViewState store / Panel 페어링 — 일단 안 만듦
- Motion / Calibration / Camera / Task / Pointcloud Panel — stay (기존 코드 재사용 시도)
- Tasks 페이지 — stay (구조만 위 sketch)
- §1 namespace 개편 — stay
- §4 store 재정비 — stay
```

## §3. 멀티로봇 UX [§2 에 흡수됨]

§2 결정에 의해 환원:

| 원 질문 | §2 결정으로 환원 |
|---|---|
| robot selector 위치 | navigation (URL `/robots/<id>` 자체가 focus) — 별도 selector UI 없음 |
| 동시 운영 시각화 | 같은 WorldScene 에 N robot URDF 동시 렌더 (World 페이지) |
| 페이지별 single vs multi view | Robots = focus mode (dim others), World = multi visible, Tasks = task 가 결정 |
| 3D scene 좌표계 | World = SSOT → 모든 로봇 한 world frame. 각 로봇 base transform 은 `robots.yaml` 에 |
| 새 로봇 추가 | `robots.yaml` 자동 enumeration → 메뉴 자동 갱신 |
| 분산 끊김 표시 | Dashboard + 메뉴 로봇별 status dot (Robot.Layer internal) |

### 참조 (구현 시 참고용)

- ROS rqt — robot namespace selector + tool 별 dropdown
- Foxglove Studio — multi-source panel 추가, robot 별 topic prefix
- 산업용 로봇 컨트롤러 (Teach Pendant) — 보통 single robot per pendant

## §5. 작업 순서

```
Slice A — §1 namespace migration                          [완료, 검증 대기]
   - omx → horibot prefix + robot-scoped 에 {robot_id} placeholder
   - BaseNode.r() / BridgeClient 자동 expand
   ↓
Slice B — §2 프로토타입 (시각화 only)                      [완료, 검증 대기]
   - robots.yaml 가짜 so101_0 + base_pose
   - /robots endpoint + useRobots + RobotLayer + RobotsPage / WorldPage
   - Sidebar enum + Workspace3D 흡수
   ↓
Slice C mechanical                                          [완료, 검증 대기]
   - Motion/Calibration/PickAndPlace 페이지 → Panel 흡수 + 페이지 삭제
   - Tasks 페이지 + Dashboard 재작성
   - /tasks /system endpoint, heartbeat robot_id
```

## §6. 구현 결과 (2026-06-04)

### Slice A — namespace
- 토픽 / 서비스 key: `omx/...` → `horibot/{robot_id}/...` (robot-scoped) /
  `horibot/<domain>/...` (task/system global)
- `BaseNode.r(template)` 헬퍼 — placeholder 자동 expand, global 노드 / multi-robot
  의 default fallback
- frontend `BridgeClient` 가 subscribe/publish/callService 콜 시 자동 expand
- `DEFAULT_ROBOT_ID` (env `VITE_DEFAULT_ROBOT_ID`, default `omx_f_0`) 가 fallback
- MJPEG `/camera/stream` → `/robots/{robot_id}/camera/stream`

### Slice B — 페이지 / multi-robot 시각화
- `robots.yaml` 에 `base_pose: {x, y, z, yaw_deg}` 필드 추가
- 가짜 `so101_0` entry (enabled=false, type=omx_f, base_pose.x=0.4) — 시각화만
- `RobotConfig` / `RobotRegistry.default()` 가 enabled robot 만 카운트
- backend `GET /robots` — list + default 반환
- frontend `useRobots()` — module-cached fetch + `BridgeClient.setDefaultRobotId()` sync
- `RobotModel` 일반화 (`robotType` / `basePose` / `opacity` props), z-up world 의 base_pose translation
- `RobotLayer` — N robot 동시 마운트, focus 모드 others dim (default 0.25)
- `RobotsPage` (`/robots/:id`) + `WorldPage` (`/world`)
- `Sidebar` Robots 섹션 자동 enumeration
- 기존 `Workspace3D` 페이지 → `RobotsPage` 에 dockview 인프라 흡수 (layout key
  `workspace3d.<id>` 로 robot 별 분리)

### Slice C mechanical — panel 흡수 / 새 페이지 / 메트릭
- `MotionPanel` ← Motion 페이지 Tabs
- `CalibrationActionsPanel` ← IntrinsicTab + HandEyeTab Tabs (기존 CalibrationPanel = 조회 전용 유지)
- `TasksPage` (`/tasks/:name`) ← PickAndPlace 의 rename + multi-robot focus=null
- `Dashboard` 재작성 — §2 sketch (Robots Online + System metrics)
- 삭제: `Motion.tsx` / `Calibration.tsx` / `PickAndPlace.tsx` / `Workspace3D.tsx`
- backend `GET /tasks` — `TASK_REGISTRY.keys()` lazy enumerate
- backend `GET /system` — psutil CPU/Mem + zenoh peers (`session.info.routers_zid/peers_zid`)
- frontend `useTasks()` / `useSystemMetrics()` (5초 polling)
- Heartbeat / LogMessage schema 에 `robot_id: str | None` 추가, BaseNode 자동 채움
- systemStore 에 `nodesByRobot: Record<robotId|"global", Record<nodeName, NodeInfo>>` 추가, Dashboard 가 robot 별 motor heartbeat 로 OK/No-Heartbeat 구분

### 검증 통과 (코드 정합성)
- backend: `uv run ruff check .` ✓ / `uv run pyright` ✓ (0 errors)
- frontend: `pnpm exec tsc -b` ✓ / `pnpm lint` ✓
- codegen: `pnpm gen:types` ✓ (topics 13 / binary 1 / services 39)

## §7. 구현 결과 (2026-06-05)

### Mode sub-routes + capabilities SSOT
- `robots.yaml` 에 `capabilities: [move, calibrate, scan]` 필드. `RobotRegistry` 의 `RobotCapability` Literal 로 yaml typo 부팅 시 fail-fast.
- `/robots/:id` shared layout ([RobotsLayout.tsx](../frontend/src/pages/RobotsLayout.tsx) — R3F + meta) + `<Outlet>` 에 mode 컴포넌트 ([robotModes/](../frontend/src/pages/robotModes/)). mode 전환 시 R3F unmount X.
- Mode 별 panel 셋: Move (Robot State + Motion + Scene Controls) / Calibrate (+ Calibration + Calibration Actions) / Scan (+ Point Cloud, depth camera 있는 robot 만).
- Sidebar 가 robot 별 capabilities 로 sub-item 렌더 — collapsed 모드에선 robot 1 아이콘 → `/robots/:id` → `RobotModeRedirect` 가 첫 capability 로 navigate.
- 옛 `RobotsPage.tsx` 삭제.

### Backend ↔ frontend SSOT — `/robots` Pydantic 화
- 이전엔 bridge `list_robots()` 가 dict 반환 + frontend `RobotInfo` 인터페이스 hand-write → drift 위험. Phase 2 mode sub-routes 가 `RobotCapability` 를 양쪽에 따로 정의하는 hand-sync 갭을 노출.
- 정공법: [bridge/schemas.py](../backend/bridge/schemas.py) 에 `RobotInfo` / `RobotsListResponse` / `BasePoseSchema` Pydantic 정의, `RobotCapability` 는 `core/robot/robot_registry.py` 에서 single import. `list_robots()` 에 `response_model=RobotsListResponse` → OpenAPI auto-emit → `pnpm gen:types` → frontend `components["schemas"]["RobotInfo"]` import.

### Mock backend
- [backend/nodes/motor_node_mock.py](../backend/nodes/motor_node_mock.py) — `MOTOR_CMD_JOINT` 받으면 internal position 즉시 갱신, `MOTOR_STATE_JOINT` 20Hz publish. 서비스는 success no-op.
- [backend/nodes/camera_node_mock.py](../backend/nodes/camera_node_mock.py) — 합성 JPEG 30Hz publish (MOCK CAMERA 라벨 + frame counter + 움직이는 dot). depth 미발행.
- [backend/config/host_mock.yaml](../backend/config/host_mock.yaml) + `nodes.transport.node_registry` 에 `mock_motor` / `mock_camera` 등록.
- `uv run python main.py --host mock` 한 줄. main.py 수정 X.

### dim opacity 실제 hookup
- 두 robot 동시 마운트 시 URDFLoader 가 URDF `<material name="grey">` 을 robot 안 모든 mesh 간 공유 + STL 가 async attach 라 `URDFLoader.load` 의 onComplete 가 mesh 들 attach 보다 먼저 fire. 정공법: [RobotModel.tsx](../frontend/src/components/canvas/3d/RobotModel.tsx) 에서 `loader.loadMeshCb` override → mesh 1개 load 직후 그 자리에서 material clone + 현재 opacity (ref 로 stash) 적용.

### Cleanup
- 옛 `docs/dockview_to_rnd_migration.md` 삭제 — dockview "라우팅 leak" 의 진짜 root cause 가 [RobotModel.tsx](../frontend/src/components/canvas/3d/RobotModel.tsx) 의 emitTCP 무한 루프였음 (commit f15a20b 에서 fix). dockview 교체 불필요.

### 검증 통과 (코드 정합성)
- backend: `uv run ruff check .` ✓ / mock startup ✓ (7 노드 + bridge 정상 기동)
- frontend: `pnpm tsc -b` ✓ / `pnpm lint` ✓
- codegen: `pnpm gen:types` ✓ (Robot schemas 자동 emit)

### 실 hardware / dev 서버 검증 — 대기 중
순차 절차: [slice_abc_verify.md](slice_abc_verify.md).
