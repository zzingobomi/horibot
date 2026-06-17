# 3D 캡처 / scan workflow / reconstruction decoupling — 4-노드 분리 plan

> **요약** — 현재 `PointCloudNode` 가 (a) RGBD primitive 변환 / (b) 라이브 스트림 / (c) scan capture (npz 저장) / (d) TSDF mesh 빌드 / (e) session/scan/mesh CRUD 다섯 책임을 한 묶음. SRP 위반 + frontend 의 `PointCloudPanel` (scan mode 전용) lifetime 이 라이브 점군 lifetime 을 잡아먹어 **scan UI 가 사라지면 점군 자체가 사라짐**.
>
> **4-노드 분리 (capability / persistence / workflow / heavy compute 4 축)**:
> 1. **`Scene3DNode`** (기존 `PointCloudNode` 대체) — RGBD primitive sensor 자리. snapshot service + stream toggle 만. scan / mesh / session 자리 X
> 2. **`StorageNode`** ([storage_layer.md](storage_layer.md) Phase 2) — session / scan / reconstruction CRUD
> 3. **`TaskNode` + Task DSL** (기존 재사용) — `ScanTask` 가 `NewSession` → `ForEach(MoveJ + CaptureScan)` → `BuildReconstruction`. TaskRunner 의 pause/resume/breakpoint/progress 자연 흡수
> 4. **`ReconstructionNode`** (신규, `MesherNode` 보다 정확한 어휘) — multi-view reconstruction heavy compute (ICP + pose graph + TSDF + mesh extract). PC host-level 1개
>
> **6 핵심 결정**:
> 1. Scene3DNode `SNAPSHOT` 서비스 — 단일, memory 반환. storage 저장은 caller 책임
> 2. `CAMERA_DEPTH_FRAME` default OFF + refcount enable (snapshot / stream 두 소비자 union)
> 3. Live stream toggle UI → Scene Controls 패널 (모든 mode 에서 가능)
> 4. `rgbd` capability gate — robots.yaml SSOT, USB 카메라 robot 자동 비노출
> 5. `ScanTask` (Task DSL) — scan workflow 가 task. UI 가 TasksPage 와 같은 자리
> 6. `ReconstructionNode` long compute progress topic — registration / pose graph / TSDF / mesh extract 단계 publish
>
> **`scan` 어휘 3 갈래 분리** (§4) — capability `scan` → `rgbd`, mode `scan` → 제거, task `scan` 신설.
>
> **상태** — **2026-06-17 구현 완료**. backend + frontend + 단위 테스트 (88 passed) + 분산 sim (localhost 3 process) + host_mock e2e 모두 통과. 사용자 hardware 검증 자리 §16.

## 0. 구현 완료 anchor (2026-06-17)

본 plan 의 4-노드 분리 + 어휘 정리 + scan workflow 자리 모두 구현 완료.

### 0.1 구현된 실 코드 anchor

| Plan 자리 | 실 코드 |
|---|---|
| `Scene3DNode` (primitive RGBD sensor) | [backend/nodes/application/scene3d_node.py](../backend/nodes/application/scene3d_node.py) — `_RobotState.consumers` set + `_acquire/release_depth_consumer` refcount + `_srv_snapshot` + `_srv_set_stream` + `_stream_loop` |
| Scene3D consensus (N frame median) | [backend/modules/scene3d/consensus.py](../backend/modules/scene3d/consensus.py) |
| Scene3D schema | [backend/core/transport/messages/scene3d.py](../backend/core/transport/messages/scene3d.py) — `Scene3DSnapshot{Req,Res}` + `Scene3DSetStream{Req,Res}` + `Scene3DState` + `Scene3DIntrinsic` |
| `ReconstructionNode` (heavy compute) | [backend/nodes/application/reconstruction_node.py](../backend/nodes/application/reconstruction_node.py) — `RECONSTRUCTION_BUILD` service + `RECONSTRUCTION_PROGRESS` topic 5 stage publish |
| Reconstruction build pipeline | [backend/modules/reconstruction/build.py](../backend/modules/reconstruction/build.py) — `build_mesh(scans, ..., progress=)` 자리 + `BuildScanInput` + `BuildResult.mesh_bytes` |
| Storage Phase 2 entity | [backend/modules/scan_workflow/persistence_models.py](../backend/modules/scan_workflow/persistence_models.py) — `ScanSessionRecord` / `ScanRecord` / `ReconstructionRecord` |
| Storage Phase 2 RDB | [backend/modules/storage/rdb/store.py](../backend/modules/storage/rdb/store.py) + [adapters/sqlite.py](../backend/modules/storage/rdb/adapters/sqlite.py) + [adapters/memory.py](../backend/modules/storage/rdb/adapters/memory.py) — `insert_scan_session` / `allocate_scan_id` / `insert_scan` / `insert_reconstruction` / `delete_scan_session` (FK CASCADE) 등 12 method |
| Storage Phase 2 service handler | [backend/nodes/application/storage_node.py](../backend/nodes/application/storage_node.py) — 10 handler (NEW_SCAN_SESSION / LIST_SCAN_SESSIONS / DELETE_SCAN_SESSION / PUT_SCAN / LIST_SCANS / DELETE_SCAN / GET_BLOB / PUT_RECONSTRUCTION / LIST_RECONSTRUCTIONS / DELETE_RECONSTRUCTION) |
| Scan blob format | [backend/modules/scan_workflow/blob.py](../backend/modules/scan_workflow/blob.py) — `[u32 jpeg_len LE][JPEG color][zstd depth]` |
| ScanTask Step DSL | [backend/modules/task/steps.py](../backend/modules/task/steps.py) — `NewSession` / `CaptureScan` / `BuildReconstruction` step 추가 |
| ScanTask factory | [backend/modules/task/tasks/scan.py](../backend/modules/task/tasks/scan.py) — `create_scan_task` (NewSession → ForEach(MoveJByName + CaptureScan) → BuildReconstruction) |
| `TaskDefinition` + TASK_REGISTRY | [backend/nodes/application/task_node.py](../backend/nodes/application/task_node.py) — `TaskDefinition(factory, required_capabilities)` 도입, ScanTask `required_capabilities=("rgbd",)` |
| `/tasks` endpoint | [backend/bridge/zenoh_bridge.py](../backend/bridge/zenoh_bridge.py) — `TaskInfo` (name + required_capabilities) list |
| `rgbd` capability + `scan` mode 제거 | [robot/robots.yaml](../robot/robots.yaml) — `so101_6dof_0` capabilities `[move, calibrate, gamepad, rgbd]` / [backend/core/robot/robot_registry.py](../backend/core/robot/robot_registry.py) `RobotCapability` Literal / [frontend/src/components/shared/Sidebar.tsx](../frontend/src/components/shared/Sidebar.tsx) `SIDEBAR_MODES` 만 |
| Frontend store / Layer rename | [frontend/src/domain/stores/scene3D.ts](../frontend/src/domain/stores/scene3D.ts) + [Scene3DLayer.tsx](../frontend/src/components/scene/Scene3DLayer.tsx). 기존 pointCloudStore / PointCloudPanel / MeshLayer / RobotScanMode 모두 제거 |
| Scene Controls "Point Cloud" 토글 | [frontend/src/components/panels/SceneControlsPanel.tsx](../frontend/src/components/panels/SceneControlsPanel.tsx) — `useScene3DStore.enabled` 자리 |
| 분산 sim host config | [backend/config/host_pc_sim.yaml](../backend/config/host_pc_sim.yaml) + [host_pi_motor_sim.yaml](../backend/config/host_pi_motor_sim.yaml) + [host_pi_camera_sim.yaml](../backend/config/host_pi_camera_sim.yaml) |
| 분산 sim test script | [backend/scripts/sim_smoke.py](../backend/scripts/sim_smoke.py) (3 process Zenoh peer 발견 + cross-process service call) + [mock_scan_e2e.py](../backend/scripts/mock_scan_e2e.py) (host_mock 단일 process Scene3D + Storage round-trip) |
| 단위 테스트 | [backend/tests/test_storage_phase2.py](../backend/tests/test_storage_phase2.py) + [test_scan_blob.py](../backend/tests/test_scan_blob.py) + [test_task_registry.py](../backend/tests/test_task_registry.py) + [test_scene3d_refcount.py](../backend/tests/test_scene3d_refcount.py) — **88 passed** (본 작업 39 + 기존 49) |

### 0.2 자동 catch 자리 (정합 게이트 + 단위 + sim + e2e)

| 자리 | 결과 |
|---|---|
| ruff + pyright (backend) | ✅ pass (본 변경 자리 무관 pre-existing pyright 1개 — node_registry line 63 fk_chain) |
| ESLint + tsc + vite build + gen:types (frontend) | ✅ pass (topics 21, services 50) |
| 단위 테스트 88 passed | Storage Phase 2 CRUD (Memory + Sqlite 같은 contract, CASCADE, UNIQUE, allocate_scan_id monotonic) / scan_blob round-trip (depth bit-exact) / TaskDefinition + ScanTask factory typed slot 연결 / Scene3DNode refcount race (concurrent 20-thread × 50-op × 5 반복) / ObjectStore |
| 분산 sim (localhost 3 process) | Zenoh peer 자동 발견 + cross-process STORAGE/CAMERA/MOTION service call + idempotent NEW_SCAN_SESSION |
| host_mock e2e | SET_STREAM → SNAPSHOT (640x480 JPEG/zstd) → NEW_SCAN_SESSION → PUT_SCAN (blob 770B + auto blob_key) → LIST_SCANS → SET_STREAM(off) round-trip |

### 0.3 e2e 가 catch 한 진짜 버그 (사용자 hardware 시간 절약)

1. **Pydantic `bytes` field → `Base64Bytes` 변경 필요** — `bytes` 는 `model_dump_json` 시 utf-8 string 인코딩 시도 → binary blob (JPEG, zstd) 실패. `color_bgr_jpeg`, `depth_z16_zstd`, storage `blob_bytes` 모두 `Base64Bytes` 로 swap. wire 시 base64 string, Python 코드는 raw bytes 그대로
2. **mock_camera 의 depth_frame publish 누락** — 기존 enable 토글만 echo, 실 payload X. 본 작업 자리 추가 — set_stream ON 시 합성 gradient depth (300-800mm) + JPEG color 8 FPS publish. host_mock 단일 process Scene3D snapshot e2e 가능 (실 D405 없이도 wire/serde 검증)
3. **SO-101 의 6DOF motor positions** — `motors=[2048,2048,2048,2048,2048,2048]` 6개 정상 (arm motor M1~M6). robots.yaml SSOT 와 wire 정합 확인

## 1. 동기 — 현재 결합 매트릭스

[backend/nodes/application/pointcloud_node.py](../backend/nodes/application/pointcloud_node.py) 의 현재 책임 5 자리:

| 책임 | 자리 | 결합 |
|---|---|---|
| RGBD primitive 변환 | depth_frame → point cloud builder | (공통 자리) |
| 라이브 점군 스트림 publish | `_stream_thread` (8 FPS) | `POINTCLOUD_CONFIGURE.enabled` 가 `CAMERA_SET_DEPTH_STREAM` 동기 호출 (긴밀) |
| Scan 캡처 (depth/color/motor → npz) | `POINTCLOUD_CAPTURE` | `scan_io.save_scan` 직접 호출 (storage layer 우회) |
| TSDF mesh 빌드 | `POINTCLOUD_BUILD_MESH` | `tsdf_builder.build_mesh` 직접 호출 → PLY 디스크. 5-30s 무거운 연산 |
| Session / scan / mesh CRUD | 6 서비스 | scan workflow 전용 |

[frontend/src/components/panels/PointCloudPanel.tsx](../frontend/src/components/panels/PointCloudPanel.tsx) 의 결합:
- "Live Stream" 토글 버튼이 panel 안에 있음 → panel 마운트가 토글 UI 의 lifetime
- panel 은 `RobotScanMode` 에만 등록 → **scan mode 가 아닌 자리에선 점군 토글 불가**

→ 라이브 점군은 카메라 JPEG 와 같은 primitive 데이터인데 scan workflow 결합 때문에 다른 자리 (캘 verification fuse 등) 에서 못 씀. mesh 빌드 자리도 같은 노드 안 — primitive 노드인지 workflow 노드인지 정체 모호.

### 진짜 use case 들 (현재 + 미래)

| Use case | 필요한 자리 | 현재 가능? |
|---|---|---|
| Scan workflow (npz 저장 + reconstruction) | snapshot + storage put + 무거운 compute | ✅ (현재 PointCloudNode 안 다 박힘) |
| 캘 verification "ghosting" 시각화 | snapshot N장 + 메모리 fuse | ❌ panel 결합 |
| 라이브 3D 뷰어 (mode 무관) | live stream toggle | ❌ scan mode 만 |
| 미래 3D detection (2D YOLO → 3D segment) | snapshot 또는 stream | ❌ |
| 미래 grasp planning | snapshot | ❌ |
| 미래 multi-robot scan fusion | 여러 robot 의 snapshot 모아 reconstruction | ❌ |

## 2. 산업 표준 모델 — Photoneo / Zivid

3D 산업 카메라 SDK 의 정석 패턴:

```python
# Zivid SDK 의사 코드
frame = camera.capture(settings)   # ← 명령 떨어진 순간 1 프레임
point_cloud = frame.point_cloud()  # ← 처리는 응용 코드
frame.save("out.zdf")              # ← 영속화는 별도 method
```

| 모드 | 트리거 | 용도 |
|---|---|---|
| **단발 capture (trigger)** | 명령 호출 | scan / detection / 일반 응용 |
| **continuous / preview** | 명시적 ON | setup 자리 (aiming, exposure 조정 등) |

핵심 — capture 자체는 **단일 책임**. save / process / visualize 는 모두 그 위에 얹힘. SDK 가 workflow (scan/mesh) 를 미리 박지 않음.

같은 원칙을 우리 자리에 적용:
- 센서 primitive (`Scene3DNode`) = snapshot + stream 만
- workflow (`ScanTask`) = task DSL 로 orchestration
- heavy compute (`ReconstructionNode`) = ICP / TSDF / mesh extract 별도 노드
- persistence (`StorageNode`) = session / scan / reconstruction CRUD 별도

## 3. 4-노드 분리 아키텍처

### 3.1 책임 매트릭스

| 노드 | 책임 자리 | lifetime | 마운트 조건 |
|---|---|---|---|
| **Scene3DNode** | RGBD primitive (snapshot + stream) | 항상 살아있음 | `rgbd` capability 인 robot 마다 dispatch |
| **StorageNode** | session / scan / reconstruction CRUD ([storage_layer.md](storage_layer.md)) | 항상 살아있음 | host-level 1개 (PC) |
| **TaskNode** | workflow orchestration (TaskRunner) | 항상 살아있음 | host-level 1개 (PC) — 기존 |
| **ReconstructionNode** | heavy compute (ICP / pose graph / TSDF / mesh) | 항상 살아있음 | host-level 1개 (PC) |

각 축의 정체성:
- **capability** (Scene3DNode) = 무엇을 측정/제공 하는가
- **persistence** (StorageNode) = 무엇을 보관 하는가
- **workflow** (TaskNode) = 어떤 순서로 실행 하는가
- **compute** (ReconstructionNode) = 무거운 연산 어디서 도는가

### 3.2 데이터 흐름

```
RGBD Camera (D405)
  │
  ▼
CameraNode  ── horibot/{robot_id}/camera/stream/depth_frame ──┐
                                                              │
                                                              ▼
                                                       Scene3DNode
                                                       (primitive)
                                                              │
                              ┌───────────────────────────────┤
                              │ snapshot service              │ stream toggle
                              │ (단발)                        │ (continuous)
                              │                               ▼
                              │                    horibot/{robot_id}/scene3d/stream
                              │                                    │
                              ▼                                    ▼
                       caller 들 ──────                    Live Viewer
                         │                                 (R3F Layer,
                         │                                  mode 무관)
                         ├─ ScanTask 의 CaptureScan step
                         ├─ 캘 verification ghosting
                         ├─ 미래 3D detection
                         └─ 미래 grasp planning
```

ScanTask 실행 시:

```
TaskNode (Task DSL)
  │
  └─ ScanTask
      ├─ NewSession  ─────►  StorageNode (STORAGE_NEW_SESSION)
      │
      ├─ ForEach(scan_pose):
      │    ├─ MoveJ       ──►  MotionNode (MOTION_MOVE_J)
      │    └─ CaptureScan ──►  Scene3DNode (SCENE3D_SNAPSHOT)
      │                    └►  StorageNode (STORAGE_PUT_SCAN)
      │
      └─ BuildReconstruction ──►  ReconstructionNode
                                   (RECONSTRUCTION_BUILD)
                                   │
                                   ├─ StorageNode (STORAGE_LIST_SCANS)
                                   ├─ ICP + pose graph + TSDF + mesh extract
                                   │  (progress topic publish)
                                   └─ StorageNode (STORAGE_PUT_RECONSTRUCTION)
```

각 step 은 *thin orchestrator* — service 호출 + step result publish 만. 무거운 compute 자리 없음.

### 3.3 노드별 책임 상세

**Scene3DNode (기존 PointCloudNode 대체)**

| 자리 | 책임 |
|---|---|
| 입력 | `CAMERA_DEPTH_FRAME` 구독 (refcount enable) |
| 변환 | depth_frame → point cloud builder (공유 함수) |
| 모드 A — snapshot | `SCENE3D_SNAPSHOT` service. memory 반환. caller 가 scan / verification / detection 등 결정 |
| 모드 B — stream | `SCENE3D_SET_STREAM` service + `horibot/{robot_id}/scene3d/stream` topic |
| 안 함 | session / scan / mesh / reconstruction / storage |

**StorageNode** ([storage_layer.md](storage_layer.md) Phase 2 진입 자리)

| 자리 | 책임 |
|---|---|
| Session | NEW / LIST / DELETE |
| Scan | PUT (blob + metadata) / LIST / GET / DELETE |
| Reconstruction | PUT / LIST / GET / DELETE |
| 도메인 모델 | `Session → Scan[] → Reconstruction` 관계 owner |
| 안 함 | 어떤 sensor 도 직접 안 봄, 어떤 compute 도 안 함 |

**TaskNode + Task DSL** (기존 [task_node.py](../backend/nodes/application/task_node.py) 재사용)

| 자리 | 책임 |
|---|---|
| Step DSL | NewSession / CaptureScan / BuildReconstruction step 신설 ([modules/task/steps.py](../backend/modules/task/steps.py)) |
| ScanTask | [modules/task/tasks/scan.py](../backend/modules/task/tasks/) 신설. factory + step 시퀀스 |
| TASK_REGISTRY | `"scan"` 추가 ([task_node.py:32](../backend/nodes/application/task_node.py#L32)) |
| 진행 시각화 | 기존 `TASK_TREE` / `TASK_STATE` / `TASK_STEP_RESULT` 재사용 — frontend 같은 패턴 |

**ReconstructionNode (신규)**

| 자리 | 책임 |
|---|---|
| 서비스 | `RECONSTRUCTION_BUILD` (session_id 받음 → mesh 결과 반환 / storage put) |
| Compute | multi-way ICP / pose graph optimization / scalable TSDF / mesh extract |
| Progress | `RECONSTRUCTION_PROGRESS` topic (stage / percent / message) |
| 입력 | StorageNode 에서 scan 리스트 받음 |
| 출력 | StorageNode 에 reconstruction put |

이름 근거 — TSDF build 는 단순 mesh 생성이 아니라 *multi-view 3D reconstruction* 전체 pipeline (registration + pose graph + fusion + mesh extraction). `MesherNode` 보다 `ReconstructionNode` 가 알고리즘 본질과 1:1.

## 4. 어휘 정리 — `scan` 의 3 갈래 분리

현재 `scan` 한 어휘가 capability / mode / task 세 자리에서 mix. 4-노드 분리 안에서는 각 자리가 다른 어휘로 분기.

### 4.1 현재 mix 상태

| 어휘 자리 | 현재 값 | 의미 |
|---|---|---|
| capability | `scan` | robot 이 D405 보유 (RGBD sensor 있음) |
| mode | `scan` | `/robots/:id/scan` route, Sidebar sub-item, RobotScanMode 페이지 |
| task | — | (없음) workflow 자리는 PointCloudNode 안에 박혀 있음 |

세 자리 다 `scan` 으로 불려서 어휘가 무엇을 의미하는지 매번 문맥에서 derive 필요.

### 4.2 4-노드 안에서 갈래

| 어휘 자리 | 신규 값 | 의미 |
|---|---|---|
| **capability** (robot 능력) | **`rgbd`** | sensor capability — D405/RGBD 카메라 보유 |
| **mode** (robot 한 대 보는 sidebar entry) | **제거** | `move` / `calibrate` 만 남음. scan workflow 자리는 task 로 이동 |
| **task** (workflow) | **`scan`** | TasksPage 의 task select 옵션. ScanTask factory |

핵심 — `scan` 어휘를 *task 자리* 한 곳에 모으고, capability 는 sensor 어휘 (`rgbd`), mode 는 robot 자체 보는 자리만 (`move` / `calibrate`).

### 4.3 어휘 정의 (참조)

| 어휘 | 정의 | 예시 |
|---|---|---|
| **capability** | robot 의 능력 list (sensor / control input / feature). `robots.yaml::capabilities` SSOT | `move`, `calibrate`, `rgbd`, `gamepad` |
| **mode** | 한 robot 보는 sidebar entry. capability 의 subset (현재 `move`/`calibrate` 만) | `/robots/:id/move`, `/robots/:id/calibrate` |
| **task** | multi-step workflow (TaskRunner orchestration). `TASK_REGISTRY` SSOT | `pick_and_place`, `scan` |

mode 가 capability 의 *subset* 인 이유 — 모든 capability 가 mode 자리 되진 않음. `rgbd` 는 sensor 어휘라 robot 보는 mode 가 아님 (sensor 데이터는 Scene Controls 토글 / TasksPage 등 다른 자리에서 노출), `gamepad` 도 control input 어휘라 mode 자리 아님.

### 4.4 변경 면적

**Backend**:
- [robot/robots.yaml](../robot/robots.yaml) 의 `capabilities` 에서 `scan` → `rgbd` (모든 robot)
- TASK_REGISTRY 에 `"scan"` 추가, `ScanTask` factory (결정 5, §9)
- (선택) `Task` 에 `required_capabilities: list[str]` field — task select 시 robot 필터 (e.g., `ScanTask.required_capabilities = ["rgbd"]`)

**Frontend**:
- [Sidebar](../frontend/src/components/common/Sidebar.tsx) 의 capability → mode 매핑이 sensor 어휘 (`rgbd`, `gamepad`) 를 mode entry 로 안 보냄. mode 어휘 hardcoded list (`["move", "calibrate"]`) + capabilities 와 교집합
- `/robots/:id/scan` route 제거, [RobotScanMode](../frontend/src/pages/robotModes/) 파일도 같이
- [PointCloudPanel](../frontend/src/components/panels/PointCloudPanel.tsx) 완전 제거 — session / scan list 는 TasksPage 의 사이드 panel (StorageNode query) 로 흡수
- TasksPage 의 task select 에 `"scan"` 옵션 추가. `required_capabilities` 가 있으면 robot select dropdown 필터

## 5. 결정 1 — Scene3DNode snapshot 서비스 시그니처

> **답: 단일 서비스, 항상 frame data memory 반환. storage 저장은 caller 책임.**

### 시그니처 (대략)

```python
# backend/core/transport/messages/scene3d.py (신설)
class Scene3DSnapshotReq(StrictModel):
    voxel_size: float | None = None  # None = raw, 값 있으면 downsample
    timeout_s: float = 1.0           # depth_frame wait timeout

class Scene3DSnapshotRes(StrictModel):
    frame_bytes: bytes               # [u32 n][n*3 float32 xyz][n*3 uint8 rgb]
    motor_positions: list[int]       # 캡처 시점 raw motor (scan 복원용)
    timestamp: float
    intrinsic: IntrinsicData         # K, fx/fy/cx/cy, depth_scale
```

### 왜 memory 반환

| 옵션 | 평가 |
|---|---|
| **A. 항상 memory 반환** (채택) | ROS 2 / Photoneo / Zivid SDK 정합. SRP. caller 가 fuse/save/process 결정 |
| B. `save_to_storage` 옵션 인자 | wire round-trip 1번 절감되지만 Scene3DNode↔StorageNode 가 같은 PC 라 Zenoh inter-process ~1ms. 거의 무시 가능 |
| C. 두 서비스 분리 (`SNAPSHOT` + `CAPTURE_TO_SCAN`) | 책임 분리 명확하지만 두 번째는 첫 번째의 wrapper — 과한 분리 |

A 채택 근거:
- Scene3DNode + StorageNode 둘 다 PC → wire 비용 무시
- 4.6 MB raw frame 도 buffer copy 수준
- caller 가 무엇을 할지 명시적 (scan = storage put, verification = memory fuse) → API 가 use case 를 강제 안 함

### caller 코드 모양 (예시)

```python
# Task DSL 의 CaptureScan step
class CaptureScan(Step[Scan]):
    session: Slot[SessionId]

    def execute(self, ctx: StepContext) -> Scan:
        snap = ctx.call_service(Service.SCENE3D_SNAPSHOT, ...)
        scan = ctx.call_service(
            Service.STORAGE_PUT_SCAN,
            session_id=ctx.resolve(self.session),
            frame_bytes=snap.frame_bytes,
            motor_positions=snap.motor_positions,
            intrinsic=snap.intrinsic,
        )
        return scan  # Slot[Scan] 다음 step 에 넘김

# 캘 verification ghosting (별도 verifier 또는 calibration_node)
frames = []
for pose in capture_poses:
    await move_to(pose)
    res = await call_service(Service.SCENE3D_SNAPSHOT, ...)
    frames.append(transform_to_base_frame(res, current_calib))
fused = open3d_concat(frames)
publish_to_frontend(fused)  # 사용자 눈으로 ghosting 확인
```

## 6. 결정 2 — depth_frame default 상태 + refcount

> **답: default OFF (현재 그대로). 내부 refcount 로 두 소비자 (snapshot / live stream toggle) 가 union 으로 enable.**

### 옵션 비교

| 옵션 | 평가 |
|---|---|
| A. 항상 ON | 분산 시 카메라 Pi → PC 3-5 MB/s LAN idle, idle CPU 점유 |
| B. Trigger-only (snapshot 호출 시만 ON) | live stream 자리에선 안 맞음 |
| **C. Refcount (채택)** | snapshot = 짧게 잡았다 놓음, live stream = 토글 OFF 까지 잡고 있음. 두 자리 다 만족 |

### 내부 모델

```python
class Scene3DNode(ApplicationNode):
    _depth_consumers: dict[str, int] = {}   # robot_id -> refcount
    _consumer_lock: Lock

    @contextmanager
    def _depth_consumer(self, robot_id: str):
        with self._consumer_lock:
            self._depth_consumers[robot_id] += 1
            if self._depth_consumers[robot_id] == 1:
                self.call_service(Service.CAMERA_SET_DEPTH_STREAM, enabled=True)
        try:
            yield
        finally:
            with self._consumer_lock:
                self._depth_consumers[robot_id] -= 1
                if self._depth_consumers[robot_id] == 0:
                    self.call_service(Service.CAMERA_SET_DEPTH_STREAM, enabled=False)

    def _srv_snapshot(self, req):
        with self._depth_consumer(req.robot_id):
            self._wait_fresh_frame(req.timeout_s)
            return self._build_frame_from_latest()

    def _srv_set_stream(self, req):
        if req.enabled:
            self._acquire_depth_consumer(req.robot_id)   # +1, return until release
        else:
            self._release_depth_consumer(req.robot_id)
```

Snapshot latency = `1/DEPTH_FPS` = ~125ms @ 8FPS. 사용자 체감 OK.

### 산업 정합

ROS 2 의 lifecycle node `ACTIVATE` / `DEACTIVATE` 와 의미 동치 — sensor 가 필요한 자리만 켜고, 아무도 안 보면 끔. lazy publisher 패턴.

## 7. 결정 3 — Live stream toggle UI 자리

> **답: Scene Controls 패널로 이동. Layer 는 이미 R3F 항상 마운트 (그대로). 기존 PointCloudPanel 은 제거 (scan UI 는 TasksPage 로 흡수, 결정 5).**

### 근거

- [frontend/src/components/scene/PointCloudLayer.tsx](../frontend/src/components/scene/PointCloudLayer.tsx) 의 `LivePointCloudLayer` 는 이미 `Scene.tsx` children — **mode 무관 항상 마운트**. `enabled` 플래그로만 게이트
- 현재 결합 source = `PointCloudPanel` 안의 "Live Stream" 토글 — panel 마운트가 토글 UI 의 lifetime
- Scene Controls 패널은 모든 mode 에 있음 — layer 토글이 자연스럽게 들어가는 자리 (이미 다른 layer 토글들 자리)
- 산업 정합 — rviz 의 Display panel / Zivid Studio 의 메인 controls — 모두 mode 무관

### UI 자리

```
Scene Controls 패널 (모든 mode)
  ├─ Show robots
  ├─ Show world axes
  ├─ Show point cloud   ← 신규. 토글 시 SCENE3D_SET_STREAM enable/disable
  ├─ Show mesh (reconstruction)
  └─ ...

TasksPage 의 ScanTask 실행 자리 (결정 5)
  ├─ task select: "scan"
  ├─ run / pause / resume / breakpoint (TaskRunner 기존 UI)
  ├─ scan list (StorageNode query)
  └─ reconstruction list (StorageNode query)

scan mode 자리 자체는 제거 (capability=rgbd 인 robot 의 TasksPage 진입으로 흡수)
```

## 8. 결정 4 — `rgbd` capability gate

> **답: `robots.yaml::capabilities` 의 `scan` 이 현재 사실상 이 역할. `rgbd` 로 rename 권고 (sensor capability 어휘 정확). backend / frontend 모두 이 SSOT 확인. 어휘 정리는 §4 참조.**

### 현재 상태

[robot/robots.yaml](../robot/robots.yaml) 주석에 이미 anchor:

```yaml
omx_f_0:
  capabilities: [move, calibrate, scan]   # D405 보유 → scan 가능. SO-101 도착 시 USB cam 으로 바뀌면 [move, calibrate] 로 변경.
so101_6dof_0:
  capabilities: [move, calibrate, gamepad]   # camera 후속 session — D405 patch 끝나면 scan 추가
```

사용자 의도에서 `scan` 이 이미 "3D capability" gate 로 작동 중. 단지 이름이 workflow 어휘.

### Rename: `scan` → `rgbd` (§4 결정)

| 측면 | `scan` | `rgbd` |
|---|---|---|
| 어휘 정확도 | workflow ("scan mode" 페이지) | sensor capability (RGBD 카메라) |
| Downstream gate 들 | scan mode / 라이브 토글 / 캘 verification / 미래 detection | 같음 — sensor 기반이라 자연스럽게 derive |
| 미래 "depth 있지만 scan 워크플로 안 함" 자리 | 불가능 | 가능 (별도 workflow capability 추가) |

→ `rgbd` 가 sensor capability 어휘로 더 정확.

### Backend 게이트

```python
# backend/core/transport/application_node.py
class ApplicationNode:
    def __init__(self, name: str, require_capability: str | None = None):
        ...
        self._require_capability = require_capability

    @property
    def enabled_robot_ids(self) -> list[str]:
        all_ids = self._registry.enabled_robot_ids()
        if self._require_capability:
            return [
                rid for rid in all_ids
                if self._require_capability in self._registry.get(rid).capabilities
            ]
        return all_ids


# backend/nodes/application/scene3d_node.py
class Scene3DNode(ApplicationNode):
    def __init__(self):
        super().__init__("scene3d_node", require_capability="rgbd")
```

USB 카메라 robot 은 `enabled_robot_ids` 에서 자동 제외 → snapshot service 등록 안 됨, stream loop 안 돔. depth_frame 구독도 안 함.

### Frontend 게이트

```ts
// Scene Controls 의 "Show point cloud" 토글
const robot = useRobot(robotId);
const hasRgbd = robot.capabilities.includes("rgbd");
if (!hasRgbd) return null;

// TasksPage 의 scan task 등록 조건도 capability 기반
// (ScanTask 가 rgbd 가진 robot 만 select 가능)
```

## 9. 결정 5 — ScanTask Step DSL

> **답: scan workflow 가 task. `NewSession` → `ForEach(scan_pose): MoveJ + CaptureScan` → `BuildReconstruction`. TaskRunner 의 pause/resume/breakpoint/progress/state publish 자연 재사용. UI 가 TasksPage 와 같은 자리.**

### 신규 Step (typed Slot)

[backend/modules/task/steps.py](../backend/modules/task/steps.py) 에 3 step 추가:

```python
@dataclass(frozen=True)
class NewSession(Step[SessionId]):
    label: str | None = None

    def execute(self, ctx: StepContext) -> SessionId:
        res = ctx.call_service(Service.STORAGE_NEW_SESSION, label=self.label)
        return SessionId(res.session_id)


@dataclass(frozen=True)
class CaptureScan(Step[Scan]):
    session: Slot[SessionId]

    def execute(self, ctx: StepContext) -> Scan:
        snap = ctx.call_service(Service.SCENE3D_SNAPSHOT)
        return ctx.call_service(
            Service.STORAGE_PUT_SCAN,
            session_id=ctx.resolve(self.session),
            frame_bytes=snap.frame_bytes,
            motor_positions=snap.motor_positions,
            intrinsic=snap.intrinsic,
        )


@dataclass(frozen=True)
class BuildReconstruction(Step[Reconstruction]):
    session: Slot[SessionId]

    def execute(self, ctx: StepContext) -> Reconstruction:
        # long compute — 5-30s. progress topic 으로 진행 publish (결정 6)
        return ctx.call_service(
            Service.RECONSTRUCTION_BUILD,
            session_id=ctx.resolve(self.session),
            timeout_s=60.0,   # 무거운 compute 자리 timeout 키움
        )
```

### ScanTask factory

[backend/modules/task/tasks/scan.py](../backend/modules/task/tasks/) 신설:

```python
def create_scan_task(scan_poses: list[str]) -> Task:
    session_step = NewSession(label="scan")
    foreach_steps, _ = scan_at_each_pose(session_step.out, scan_poses)
    build_step = BuildReconstruction(session=session_step.out)
    return Task(
        name="scan",
        steps=[session_step, *foreach_steps, build_step],
        required_capabilities=["rgbd"],  # §4 변경 면적 — task 가 robot 필터링
    )


def scan_at_each_pose(session_slot, poses):
    inner = ForEach(
        items=poses,
        body=lambda pose: [
            MoveJByName(pose=pose),
            CaptureScan(session=session_slot),
        ],
    )
    return [inner], None
```

### TASK_REGISTRY 등록

```python
# backend/nodes/application/task_node.py
TASK_REGISTRY: dict[str, Callable[[dict], Task]] = {
    "pick_and_place": _factory_pick_and_place,
    "scan": _factory_scan,                   # 신규
}

def _factory_scan(data: dict) -> Task:
    poses = data.get("scan_poses", DEFAULT_SCAN_POSES)
    return create_scan_task(scan_poses=poses)
```

### UI 자리

기존 TasksPage 가 그대로 — `task: "scan"` select → run / pause / resume / breakpoint / step tree / step result 모두 무료로 받음. scan mode 페이지 / PointCloudPanel 의 scan UI 부분 제거.

session / scan / reconstruction 리스트는 StorageNode query 로 별도 panel (TasksPage 안의 사이드 panel 또는 settings 쪽).

## 10. 결정 6 — ReconstructionNode progress publish

> **답: `RECONSTRUCTION_PROGRESS` topic — `BuildReconstruction` step 실행 중 stage 별 진행 publish. step status update 자연 흡수.**

### 동기

`RECONSTRUCTION_BUILD` 가 5-30s 무거운 compute. `call_service` timeout 키우기만 하면 step 동안 사용자가 진행 상황 못 봄 (그냥 멈춘 화면).

### Topic 시그니처

```python
# backend/core/transport/messages/reconstruction.py
class ReconstructionProgress(StrictModel):
    session_id: str
    stage: Literal[
        "loading_scans",        # storage 에서 scan 끌어옴
        "pairwise_registration", # initial ICP
        "pose_graph_optimization",
        "tsdf_integration",
        "mesh_extraction",
    ]
    percent: float    # 0.0 ~ 1.0
    message: str      # "scan 3/8 등록 중"
```

### ReconstructionNode publish 자리

```python
class ReconstructionNode(ApplicationNode):
    def _srv_build(self, req):
        scans = self._load_scans(req.session_id)
        self._progress("loading_scans", 1.0, f"{len(scans)} scan 로드")

        # pairwise ICP
        for i, (a, b) in enumerate(pairs):
            self._progress(
                "pairwise_registration",
                i / len(pairs),
                f"ICP {i+1}/{len(pairs)}",
            )
            ...

        self._progress("pose_graph_optimization", 0.0, "pose graph 최적화")
        pose_graph = self._optimize_pose_graph(...)
        ...

        self._progress("tsdf_integration", 0.0, "TSDF volume 적분")
        ...

        self._progress("mesh_extraction", 0.0, "mesh extract")
        mesh = self._extract_mesh(...)

        return self._put_to_storage(mesh)
```

### Frontend

`BuildReconstruction` step 의 status indicator 가 `RECONSTRUCTION_PROGRESS` 구독 → step label 옆 progress bar. TasksPage 의 step tree 위에 자연스럽게 얹힘.

진척 무거운 step 자리 패턴 — 미래 다른 heavy step (e.g., grasp planning, motion planning) 자리도 같은 패턴 (progress topic + step status update) 일반화 가능.

## 11. Rename 면적 + 신규

### 노드

| 현재 | 신규 |
|---|---|
| `PointCloudNode` | `Scene3DNode` |
| — | `ReconstructionNode` (신규) |
| — | `StorageNode` (별도 plan — [storage_layer.md](storage_layer.md)) |

### 토픽 / 서비스

**Scene3DNode** (sensor primitive):

| 현재 | 신규 | 비고 |
|---|---|---|
| `horibot/{robot_id}/pointcloud/stream` | `horibot/{robot_id}/scene3d/stream` | binary |
| `horibot/{robot_id}/pointcloud/state` | `horibot/{robot_id}/scene3d/state` | enabled / voxel_size |
| `horibot/{robot_id}/pointcloud/srv/configure` | `horibot/{robot_id}/scene3d/srv/set_stream` | refcount enable |
| — | `horibot/{robot_id}/scene3d/srv/snapshot` | **신설**, 결정 1 |

**ReconstructionNode** (heavy compute):

| 신규 | 비고 |
|---|---|
| `horibot/reconstruction/srv/build` | session_id 받음 → mesh 결과 |
| `horibot/reconstruction/progress` | stage / percent / message (결정 6) |

session/scan/reconstruction CRUD 는 [storage_layer.md](storage_layer.md) 의 `horibot/storage/srv/*` 자리.

기존 `pointcloud/srv/new_session`, `pointcloud/srv/capture`, `pointcloud/srv/list_scans`, `pointcloud/srv/delete_scan`, `pointcloud/srv/list_sessions`, `pointcloud/srv/build_mesh`, `pointcloud/srv/list_meshes` 는 모두 제거 → StorageNode + ReconstructionNode 로 이동.

### Frontend

| 현재 | 신규 |
|---|---|
| `pointCloudStore` | `scene3DStore` |
| `PointCloudPanel` | 제거 (TasksPage 의 ScanTask 로 흡수, scan 리스트는 StorageNode panel) |
| `PointCloudLayer.tsx` | `Scene3DLayer.tsx` |
| `MeshLayer.tsx` | `ReconstructionLayer.tsx` |
| `/robots/:id/scan` route | 제거 |
| `RobotScanMode` | 제거 |
| Sidebar 의 capability→mode 매핑 | mode hardcoded list (`["move","calibrate"]`) + capabilities 교집합 (§4.4) |

## 12. 마이그레이션 plan

### 의존성

본 plan 의 일부는 [storage_layer.md](storage_layer.md) Phase 2 (scans / reconstructions storage) 에 의존. Phase 2 진입 자리 = 본 plan 의 마이그레이션 중반.

### 순서 (commit 단위)

1. **Scene3DNode 부분 분리 (snapshot + refcount)** — 새 `SCENE3D_SNAPSHOT` service + refcount 로직 추가, 기존 `POINTCLOUD_*` 그대로 둠. coexist 단계
   - test: snapshot 호출 → fresh frame 받음 / live stream toggle 과 동시 호출 → refcount 정상

2. **Frontend Scene Controls 에 토글 이동** — `pointCloudStore.setEnabled` 호출 자리 옮기기. PointCloudPanel 의 "Live Stream" 버튼 제거
   - test: 다른 mode (calibrate / move) 에서 토글 → 점군 보임

3. **`rgbd` capability rename + gate 적용 + scan mode 제거** (§4 변경 면적)
   - robots.yaml `scan` → `rgbd`, 모든 caller grep 교체
   - Sidebar 의 capability→mode 매핑이 sensor 어휘 제외 (mode hardcoded list)
   - `/robots/:id/scan` route + RobotScanMode 파일 제거
   - test: omx_f_0 (capabilities 에 `rgbd` 있음) sidebar 에 scan 메뉴 안 보임, Scene Controls 토글 보임

4. **`PointCloudNode` → `Scene3DNode` rename** — file rename + 클래스 rename + topic_map / api_contract / node_registry 갱신 + frontend store/component rename
   - rename-only commit, behavior 변경 X
   - 이 시점 노드 책임은 아직 5 자리 (snapshot/stream/scan/mesh/CRUD) — 다음 commit 들이 분리

5. **[storage_layer.md](storage_layer.md) Phase 2 — StorageNode scan/reconstruction 추가** — session / scan / reconstruction CRUD service 신설
   - test: scan put → list → get → delete round-trip

6. **ReconstructionNode 분리** — 무거운 compute (ICP / TSDF / mesh extract) 를 Scene3DNode 에서 새 노드로 이동. `RECONSTRUCTION_BUILD` service + `RECONSTRUCTION_PROGRESS` topic
   - test: 기존 scan 데이터로 build 호출 → progress publish → 결과 동등

7. **ScanTask Step DSL 도입** — `NewSession` / `CaptureScan` / `BuildReconstruction` step + `create_scan_task` factory + TASK_REGISTRY 등록 + `Task.required_capabilities` field
   - test: TasksPage 에서 task=scan select → robot dropdown 이 rgbd 인 robot 만 보임 → 진행 시각화 + step result 정상

8. **Scene3DNode 의 scan/mesh CRUD 제거** — `POINTCLOUD_CAPTURE` / `POINTCLOUD_BUILD_MESH` / session CRUD 서비스 모두 제거. Scene3DNode 가 진짜 primitive 만 남음
   - frontend PointCloudPanel 완전 제거 — TasksPage 의 ScanTask + StorageNode sidebar 로 흡수
   - test: snapshot + stream 외 서비스 호출 → 모두 unknown service

### 면적 추정

- **backend**: ~800-1200 LOC (4 노드 분리 + Task DSL step + progress topic + storage Phase 2 + `Task.required_capabilities`)
- **frontend**: ~400-600 LOC (Scene Controls 토글 + Scene3DLayer rename + ScanTask UI 흡수 + ReconstructionLayer rename + Sidebar capability/mode 분리 + scan route 제거)
- **테스트**: snapshot service / refcount edge case / ScanTask end-to-end / reconstruction progress / storage Phase 2 round-trip

## 13. 검증 plan — catch 가능 자리 / hardware 필수 자리

본 plan 구현은 외부 자리에서 진행되어 사용자가 매 commit 사이 hardware 검증 불가. 한 번에 hardware-ready 상태로 만들고 집에서 1회 검증 후 일괄 commit. *자동 테스트로 catch 가능한 자리* 는 무조건 잡아서 hardware 검증 시간 낭비 X 가 본 § 의 목적.

### 13.1 자동 테스트로 catch 가능 (반드시 잡을 자리)

**단위 테스트**

| 자리 | 테스트 |
|---|---|
| refcount race | concurrent snapshot + stream toggle 동시 호출 → enable 호출 1회만, disable 정상 정렬 |
| `_consumer_lock` 정합 | snapshot 진행 중 stream toggle off → refcount 0 안 됨 |
| Snapshot fresh frame wait | 오래된 frame 들고 있을 때 snapshot 호출 → timeout 또는 fresh frame 만 반환 |
| Step DSL typed slot | `NewSession.out` → `CaptureScan.session`, `BuildReconstruction.session` 의 typed slot 정합 (pyright + dataclass) |
| Reconstruction progress 5 stage publish | mock reconstruction 으로 5 stage `RECONSTRUCTION_PROGRESS` publish 순서/percent 검증 |
| StorageNode CRUD | session put → list → get → delete round-trip 각 round 의 invariant |
| `Task.required_capabilities` filter | rgbd 없는 robot 으로 ScanTask 호출 → reject. rgbd 있는 robot → accept |
| `enabled_robot_ids` capability gate | mock RobotRegistry 로 capability subset 별 enabled list 검증 |
| Sidebar mode 매핑 | `rgbd` / `gamepad` capability 가 mode entry 로 안 보임 |

**e2e (host_mock 단일 process)**

mock_motor + mock_camera + Scene3DNode + StorageNode + ReconstructionNode + TaskNode 모두 띄우고 ScanTask 전체 시나리오 자동 실행:
- `TASK_RUN` 에 `task="scan"` → `TASK_TREE` / `TASK_STATE` / `TASK_STEP_RESULT` 토픽 정상 흐름
- `CaptureScan` step 의 `SCENE3D_SNAPSHOT` + `STORAGE_PUT_SCAN` 양쪽 호출 검증
- `BuildReconstruction` 의 `RECONSTRUCTION_PROGRESS` 5 stage publish + 최종 `step_result` mesh path 검증
- 진짜 D405 데이터 X → mock_camera 의 합성 depth frame 으로 reconstruction 결과는 빈/단순 mesh 일 수 있음. e2e 흐름 검증만 목적

**분산 sim (localhost multi-process)**

memory anchor "mock 통과 ≠ distributed 검증" — cross-process state 자리는 단일 process e2e 로 안 잡힘. localhost 에 host config 분리해서 3 process 띄움:
- process 1: `--host pc` (application_nodes 다 포함)
- process 2: 신규 `host_pi_motor_sim.yaml` (mock_motor + motion)
- process 3: 신규 `host_pi_camera_sim.yaml` (mock_camera)
- Zenoh peer 자동 발견 자리 검증
- refcount enable 의 cross-process service call 정합 (Scene3DNode (PC) 의 `CAMERA_SET_DEPTH_STREAM` 호출이 다른 process 의 mock_camera 까지 전달되는지)
- StorageNode 가 PC process 에 있고, 다른 process 에서 `STORAGE_PUT_SCAN` 호출 정합
- 진짜 LAN / 다른 머신 / Pi ARM 자리는 hardware (§13.2)

**정합 게이트**

- `cd backend && uv run ruff check . && uv run pyright` — backend 정합
- `cd frontend && pnpm lint && pnpm gen:types && pnpm build` — frontend 정합 + contract sync ([api_contract.py](../backend/api_contract.py) ↔ [generated/contract.ts](../frontend/src/api/generated/contract.ts))

**Frontend manual smoke (작업자가 직접)**

`pnpm dev` + browser manual click-through. dev console error 모니터:
- `/robots/:id` 진입 → Sidebar sub-item 이 mode (`move` / `calibrate`) 만, `rgbd` / `gamepad` 어휘 안 보임
- Scene Controls 의 "Show point cloud" 토글 → 다른 mode 에서도 토글 가능 (라이브 점군은 mock 이라 빈 자리 가능, 토픽 흐름만 확인)
- `/tasks/scan` 진입 → robot select dropdown 이 rgbd capability robot 만 / task select 에 `"scan"` 등록
- 라우팅 leak (memory: commit f15a20b 자리, [RobotModel.tsx:113-134](../frontend/src/components/canvas/3d/RobotModel.tsx#L113-L134) 패턴) 검증 — task 실행 중 dockview panel 전환해도 R3F mount/unmount 흐름 정상

**Race / lifetime 자리 강조** (memory: "intermittent 버그는 reproduction script 가 fix 보다 먼저")

- refcount 의 동시 enter / exit race — concurrent test 10회 이상 반복 실행 통계 검증
- Frontend useEffect dep loop — 본 plan 신규 자리 (Scene3DLayer, ReconstructionLayer, TasksPage 의 `BuildReconstruction` progress subscribe 자리) 에서 ref-stash 패턴 위반 자리 없는지 review

### 13.2 hardware 필수 (집가서 검증)

| 자리 | 왜 hardware 필요 |
|---|---|
| 실제 D405 RGBD frame quality | mock_camera 의 합성 depth 는 진짜 noise / depth_scale / intrinsic 정합 안 됨 |
| 진짜 reconstruction PLY 정확도 | 진짜 scan 데이터의 ICP / pose graph 수렴, 결과 mesh quality |
| Dynamixel motor 실제 동작 | MotionNode 의 진짜 trajectory + raw motor 명령 |
| Pi ARM 빌드 | pyrealsense2 wheel / open3d ARM / zstandard ARM 자리 |
| 다른 머신 간 Zenoh peer 발견 | 진짜 LAN multicast scout, WiFi vs 유선, hostname / 방화벽 |
| D405 라이브 점군 visual quality | 사용자 눈으로 확인 자리 |
| 라이브 토글 ON 시 LAN bandwidth | [distributed_topology.md](distributed_topology.md) anchor — WiFi 한계 자리 (§14.2) |

### 13.3 첫 hardware 검증 흐름 (commit 전 / 2026-06-17 update — SO-101 + D405 기준)

D405 는 **SO-101 6DOF 에 장착** (`so101_6dof_0.capabilities = [move, calibrate, gamepad, rgbd]`). `omx_f_0` 는 SO-101 통합 중 `enabled: false` 유지. 본 plan 의 검증 진입 흐름:

1. **frontend manual smoke (browser)** — `cd backend && uv sync && uv run python main.py --host mock` + `cd frontend && pnpm dev`
   - `/robots/so101_6dof_0` 진입 → Sidebar sub-item 이 `Move` / `Calibrate` 만 (rgbd / gamepad 어휘 안 보임)
   - Scene Controls 의 "Point Cloud" 토글 ON → mock_camera 의 합성 depth (gradient 300-800mm) 자리 라이브 점군 보임
   - `/tasks/scan` 진입 → task select 에 `scan` 등록 (required_capabilities=["rgbd"])
   - 라우팅 leak 검증 — task 실행 중 dockview panel 전환 시 R3F mount/unmount 정상
2. **단일 머신 풀스택 + 실 hardware** — `uv run python main.py` (host_dev) + SO-101 결선 + D405
   - **선행**: hand_eye 캘 먼저 (`/robots/so101_6dof_0/calibrate`). `BuildReconstruction` 자리 `hand_eye` 필수 (현재 없으면 raise)
   - `ScanTask` 의 `scan_poses` 자리 **N≥2** 필요 (`BuildReconstruction.MIN_SCANS = 2`). `DEFAULT_SCAN_POSES = ["home"]` 1개 자리만 default 라 본 자리 변경 필요 — robot_poses.yaml 에 scan_top / scan_front 등 추가 후 task data 로 전달
   - ScanTask run → step tree (NewSession → ForEach(MoveJ + CaptureScan) × N → BuildReconstruction) + RECONSTRUCTION_PROGRESS bar 5 stage
3. **storage 결과 확인** — `storage/horibot.db` 의 `scan_sessions` / `scans` / `reconstructions` 테이블 row + `storage/blobs/scans/so101_6dof_0/<session>/<scan_id>.bin` + `storage/blobs/reconstructions/so101_6dof_0/<session>/recon_<ts>.ply` 파일
4. **진짜 분산** — PC + 모터 Pi (`host_pi_motor`) + 카메라 Pi (`host_pi_camera`) LAN 환경. localhost sim 과 별개 — multicast scout / WiFi / 방화벽 자리 (안 잡히면 `zenoh.connect` endpoint 명시)
5. **D405 위치 자리** — [distributed_topology.md](distributed_topology.md) 따라 SO-101 의 카메라 Pi 또는 PC 결정
6. 모두 OK → 일괄 commit (또는 의미 단위 분리)

## 14. 미해결 / 보류 자리

### 14.1 캘 verification UI 자체

본 plan 은 capability/primitive 만 만들고, "캘 후 ghosting 시각화 UI" 자체는 별도 자리. snapshot service + 메모리 fuse 가 있으면 만들 수 있지만 UI/UX 설계 자리 별도.

→ 본 plan 끝난 뒤 별도 doc / session.

### 14.2 분산 토폴로지에서 depth_frame 의 LAN 비용

현재 [distributed_topology.md](distributed_topology.md) anchor 상 카메라 Pi → PC depth_frame 가 LAN 으로 흐름. refcount 가 idle 자리 0 으로 만들지만, 라이브 토글 ON 시 ~3-5 MB/s 가 LAN 위. WiFi 자리에서 한계 자리 확인 필요.

→ hardware 자리 검증 (집).

### 14.3 voxel_size advanced 자리

snapshot 의 voxel_size 인자 / live stream voxel_size 의 frontend control 자리. 일단 default 박고 advanced 자리 미룸.

→ 사용자 피드백 시점에 settings modal 또는 expandable section 으로 추가.

### 14.4 ScanTask 의 scan_poses 입력 자리

`scan_poses` 가 named pose list (e.g., `["scan_top", "scan_front", ...]`) — 어디서 정의? motors.yaml 의 `named_poses` 자리? 별도 scan_config.yaml? robot type 별 default + instance override 자리.

→ ScanTask 구현 진입 시 결정. 일단 hardcoded list 로 prototype.

### 14.5 미래 3D detection / grasp planning use case

snapshot service primitive 위에 얹힐 자리. 구체 API 는 use case 진입 시 별도.

→ 본 plan 의 primitive 가 만족하는지만 확인 (snapshot frame data shape 이 그쪽 use case 도 fit) — fit 안 하면 본 plan 의 시그니처 revisit.

### 14.6 multi-robot scan fusion

여러 robot 의 snapshot 모아 single reconstruction. ReconstructionNode 가 robot-scoped 아닌 host-level 이라 자연 가능 — `RECONSTRUCTION_BUILD` 요청에 multi robot scan 모음 지원만 추가. 다만 base frame cross-calibration ([multi_robot_cross_calibration.md](multi_robot_cross_calibration.md)) 선행 필요.

→ 본 plan 의 직접 범위 밖. 두 robot + cross calib 완료 후 별도 plan.

## 15. 다음 세션 anchor

본 plan 구현 진입 시:

1. **결정 4 (capability rename) + §4 어휘 정리 의 사용자 최종 confirm** — `scan` → `rgbd` capability + scan mode 제거 + task `scan` 신설. 본 doc 권고지만 사용자 선호 확인
2. **결정 1 의 wire format 자리** — `frame_bytes` 가 우리 기존 `POINTCLOUD_STREAM` 의 `[u32 n][n*3 float32 xyz][n*3 uint8 rgb]` 와 동일 컨벤션. service response 가 bytes 직접 받을 수 있는지 transport 확인 (Pydantic StrictModel + bytes field — Pydantic OK, Zenoh wire OK)
3. **결정 5 의 step 신호 흐름 검증** — `NewSession` 의 `SessionId` slot 이 ForEach body 안의 `CaptureScan` 에 전달되는 자리 — 기존 [step_dsl.md](step_dsl.md) 의 `ForEach` 가 outer scope slot 받을 수 있는지 확인
4. **결정 6 의 progress topic 과 step status integration** — TaskRunner 가 step 안에서 외부 topic 받아 step status 갱신하는 자리 — 기존 패턴 있나? 없으면 step DSL 확장 자리
5. **`Task.required_capabilities` field 자리** (§4.4 + §9) — TASK_REGISTRY 의 task factory 가 반환하는 Task 에 capability requirement 박는 자리. frontend 의 robot select 필터 SSOT
6. **storage Phase 2 timing 결정** — 본 plan 의 마이그레이션 step 5 가 storage Phase 2. 본 plan 단독 진입 vs storage Phase 2 와 같이 진입. 후자가 자연 (둘 다 scan/reconstruction 자리 만짐)
7. **작업 모델 — 외부 자리, 일괄 hardware 검증** — 사용자가 매 commit 사이 검증 불가. §12 의 step 시퀀스는 *작업 순서* 만 의미 (commit 단위 아님). 매 작업 step 별 §13.1 의 자동 테스트 자리 통과시키며 진행. 모두 통과 후 §13.3 의 hardware 검증 흐름 → 사용자 집에서 1회 확인 → 일괄 commit (또는 사용자 선호 시 의미 단위 분리)

본 plan 의 핵심 — **3D 캡처는 카메라 JPEG 같은 primitive 데이터 자리, workflow 는 task DSL, persistence 는 storage, heavy compute 는 reconstruction.** 모든 결정이 이 4-축 분리의 derive.
