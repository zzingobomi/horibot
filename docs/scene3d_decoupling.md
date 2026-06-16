# 3D 캡처 capability decoupling — `Scene3DNode` 분리 plan

> **요약** — 현재 `PointCloudNode` 가 (a) 라이브 스트림 / (b) scan 캡처 (npz 저장) / (c) TSDF mesh 빌드 세 자리를 한 묶음으로 다룸. frontend 에선 PointCloudPanel (scan mode 전용) 마운트가 라이브 스트림 토글의 lifetime — **scan UI 가 사라지면 점군 자체가 사라짐**. → 3D 캡처는 카메라 JPEG 같은 primitive 센서 데이터 자리이지 scan/TSDF workflow 의 일부가 아님.
>
> **산업 표준 모델 (Photoneo PhoXi / Zivid)** — `snapshot()` = on-command pull, `live stream` = 명시적 toggle. 모든 downstream workflow (scan / 캘 verification / 미래 3D detection) 는 이 두 primitive 위에 얹힘.
>
> **4 결정 + rename**:
> 1. Snapshot service — 단일, 항상 frame data memory 반환. storage 저장은 caller 책임
> 2. `CAMERA_DEPTH_FRAME` default OFF + refcount enable (snapshot 호출 / live stream toggle 두 트리거 union)
> 3. Live stream toggle UI → Scene Controls 패널로 이동 (모든 mode 에서 가능). Layer 는 이미 R3F 항상 마운트
> 4. `rgbd` capability 게이트 — robots.yaml SSOT. USB 카메라 robot 은 Scene3DNode dispatch / 토글 / scan mode 다 자동 비노출
> 5. Rename `PointCloudNode` → `Scene3DNode`, 토픽/서비스 prefix `pointcloud/*` → `scene3d/*`
>
> **상태** — 2026-06-16 설계 완료. 구현은 별도 세션.

## 1. 동기 — 현재 결합 매트릭스

[backend/nodes/application/pointcloud_node.py](../backend/nodes/application/pointcloud_node.py) 의 현재 책임:

| 책임 | 자리 | 결합 |
|---|---|---|
| 라이브 점군 스트림 publish | `_stream_thread` (8 FPS) | `POINTCLOUD_CONFIGURE.enabled` 가 `CAMERA_SET_DEPTH_STREAM` 동기 호출 (긴밀) |
| Scan 캡처 (depth/color/motor → npz) | `POINTCLOUD_CAPTURE` | `scan_io.save_scan` 직접 호출 (storage layer 우회) |
| TSDF mesh 빌드 | `POINTCLOUD_BUILD_MESH` | `tsdf_builder.build_mesh` 직접 호출 → PLY 디스크 |
| Session / scan / mesh 관리 (list/delete) | 6 서비스 | scan workflow 전용 |

[frontend/src/components/panels/PointCloudPanel.tsx](../frontend/src/components/panels/PointCloudPanel.tsx) 의 결합:
- "Live Stream" 토글 버튼이 panel 안에 있음 → panel 마운트가 토글 UI 의 lifetime
- panel 은 `RobotScanMode` 에만 등록 → **scan mode 가 아닌 자리에선 점군 토글 불가**

→ 라이브 점군은 카메라 JPEG 와 같은 primitive 데이터인데 scan workflow 결합 때문에 다른 자리 (캘 verification fuse 등) 에서 못 씀.

### 진짜 use case 들 (현재 + 미래)

| Use case | 필요한 자리 | 현재 가능? |
|---|---|---|
| Scan workflow (npz 저장 + TSDF mesh) | snapshot + storage put | ✅ (현재 PointCloudNode) |
| 캘 verification "ghosting" 시각화 | snapshot N장 + 메모리 fuse | ❌ panel 결합 |
| 라이브 3D 뷰어 (mode 무관) | live stream toggle | ❌ scan mode 만 |
| 미래 3D detection (2D YOLO → 3D segment) | snapshot 또는 stream | ❌ |
| 미래 grasp planning | snapshot | ❌ |

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

우리 자리도 같은 모델로 가면:
- `SCENE3D_SNAPSHOT` 서비스 = trigger 단발
- `SCENE3D_STREAM` 토픽 = continuous, 명시적 ON
- scan workflow / 캘 verification / 미래 use case 는 모두 위 두 primitive 의 caller

## 3. 결정 1 — Snapshot 서비스 시그니처

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
| B. `save_to_storage` 옵션 인자 | wire round-trip 1번 절감되지만 PointCloudNode↔storage_node 가 같은 PC 라 Zenoh inter-process ~1ms. 거의 무시 가능 |
| C. 두 서비스 분리 (`SNAPSHOT` + `CAPTURE_TO_SCAN`) | 책임 분리 명확하지만 두 번째는 첫 번째의 wrapper — 과한 분리 |

A 채택 근거:
- PointCloudNode + storage_node 둘 다 PC → wire 비용 무시
- 4.6 MB raw frame 도 buffer copy 수준
- caller 가 무엇을 할지 명시적 (scan = storage put, verification = memory fuse) → API 가 use case 를 강제 안 함
- 만약 wire 비용 실제 병목으로 드러나면 옵션 B 인자 추가 (additive)

### caller 코드 모양 (예시)

```python
# scan workflow (Scene3DNode 내 _srv_capture_scan, Phase 2 storage 연동 후)
res = await self.call_service(Service.SCENE3D_SNAPSHOT, ...)
scan_record = ScanRecord(
    robot_id=rid,
    session_id=session_id,
    motor_positions=res.motor_positions,
    intrinsic=res.intrinsic,
)
await self.call_service(Service.STORAGE_PUT_SCAN, ...)  # blob + metadata

# 캘 verification ghosting check (calibration_node 또는 별도 verifier)
frames = []
for pose in capture_poses:
    await move_to(pose)
    res = await call_service(Service.SCENE3D_SNAPSHOT, ...)
    frames.append(transform_to_base_frame(res, current_calib))
fused = open3d_concat(frames)
publish_to_frontend(fused)  # 사용자 눈으로 ghosting 확인
```

## 4. 결정 2 — depth_frame default 상태 + refcount

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

## 5. 결정 3 — Live stream toggle UI 자리

> **답: Scene Controls 패널로 이동. Layer 는 이미 R3F 항상 마운트 (그대로). PointCloudPanel 은 scan workflow 전용으로 축소.**

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
  ├─ Show point cloud   ← 신규. 토글 시 SCENE3D_STREAM enable/disable
  ├─ Show mesh (TSDF)
  └─ ...

PointCloudPanel (scan mode 전용, 축소)
  ├─ Session 관리 (new / list)
  ├─ Capture 버튼 → SCENE3D_CAPTURE_SCAN (저장형 service)
  ├─ Scan list + delete
  └─ BUILD MESH 버튼

(voxel_size 같은 advanced 파라미터는 default 박고 미루기. settings modal 자리 후속)
```

## 6. 결정 4 — `rgbd` capability 게이트

> **답: `robots.yaml::capabilities` 의 `scan` 이 현재 사실상 이 역할. `rgbd` 로 rename 권고 (sensor capability 어휘 정확). backend / frontend 모두 이 SSOT 확인.**

### 현재 상태

[robot/robots.yaml](../robot/robots.yaml) 주석에 이미 anchor:

```yaml
omx_f_0:
  capabilities: [move, calibrate, scan]   # D405 보유 → scan 가능. SO-101 도착 시 USB cam 으로 바뀌면 [move, calibrate] 로 변경.
so101_6dof_0:
  capabilities: [move, calibrate, gamepad]   # camera 후속 session — D405 patch 끝나면 scan 추가
```

사용자 의도에서 `scan` 이 이미 "3D capability" gate 로 작동 중. 단지 이름이 workflow 어휘.

### Rename 권고: `scan` → `rgbd`

| 측면 | `scan` | `rgbd` |
|---|---|---|
| 어휘 정확도 | workflow ("scan mode" 페이지) | sensor capability (RGBD 카메라) |
| Downstream gate 들 | scan mode / 라이브 토글 / 캘 verification / 미래 detection | 같음 — sensor 기반이라 자연스럽게 derive |
| 미래 "depth 있지만 scan 워크플로 안 함" 자리 | 불가능 | 가능 (별도 workflow capability 추가) |

→ `rgbd` 가 sensor capability 어휘로 더 정확. 변경 면적 = robots.yaml + 모든 caller 의 `"scan"` 체크 grep 교체.

### 두 layer 분리 옵션 (보류)

```yaml
# 옵션 B (지금 도입 X — N≤2 이고 1:1 이라 over-engineering)
so101_6dof_0:
  capabilities: [move, calibrate, rgbd, scan_workflow, gamepad]
  #                                ↑ sensor    ↑ workflow (implies rgbd)
```

→ 지금은 `rgbd` 단일 capability 로 충분. 미래 "depth 있는데 scan workflow 안 함" use case 가 진짜 생기면 그때 split (additive, 미리 할 필요 X).

### Backend 게이트

```python
# backend/core/transport/application_node.py (또는 base_node)
class ApplicationNode:
    def __init__(self, name: str, require_capability: str | None = None):
        ...
        self._require_capability = require_capability

    @property
    def enabled_robot_ids(self) -> list[str]:
        # base 가 robots.yaml capability 자동 필터링
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
if (!hasRgbd) return null;  // 또는 disabled + tooltip

// Sidebar 의 scan mode entry (이미 capability 기반 패턴)
// scan mode 표시 조건: capabilities.includes("rgbd")  (현재 "scan" → "rgbd")
```

## 7. Rename 면적

### 노드

`PointCloudNode` → `Scene3DNode`

### 토픽 / 서비스

| 현재 | 신규 | 비고 |
|---|---|---|
| `horibot/{robot_id}/pointcloud/stream` | `horibot/{robot_id}/scene3d/stream` | binary |
| `horibot/{robot_id}/pointcloud/state` | `horibot/{robot_id}/scene3d/state` | enabled / voxel_size |
| `horibot/{robot_id}/pointcloud/snapshot` (미사용) | 제거 | |
| — | `horibot/{robot_id}/scene3d/srv/snapshot` | **신설**, 결정 1 |
| `horibot/{robot_id}/pointcloud/srv/configure` | `horibot/{robot_id}/scene3d/srv/set_stream` | refcount enable, 결정 2 |
| `horibot/{robot_id}/pointcloud/srv/new_session` | `horibot/{robot_id}/scene3d/srv/new_session` | scan workflow |
| `horibot/{robot_id}/pointcloud/srv/capture` | `horibot/{robot_id}/scene3d/srv/capture_scan` | snapshot + storage 합성 |
| `horibot/{robot_id}/pointcloud/srv/list_scans` | `horibot/{robot_id}/scene3d/srv/list_scans` | |
| `horibot/{robot_id}/pointcloud/srv/delete_scan` | `horibot/{robot_id}/scene3d/srv/delete_scan` | |
| `horibot/{robot_id}/pointcloud/srv/list_sessions` | `horibot/{robot_id}/scene3d/srv/list_sessions` | |
| `horibot/{robot_id}/pointcloud/srv/build_mesh` | `horibot/{robot_id}/scene3d/srv/build_mesh` | TSDF |
| `horibot/{robot_id}/pointcloud/srv/list_meshes` | `horibot/{robot_id}/scene3d/srv/list_meshes` | |

### Frontend

| 현재 | 신규 |
|---|---|
| `pointCloudStore` | `scene3DStore` |
| `PointCloudPanel` | `ScanWorkflowPanel` (scope 축소: capture / scans / mesh) |
| `PointCloudLayer.tsx` | `Scene3DLayer.tsx` |
| `MeshLayer.tsx` | (그대로 — TSDF mesh 자리, scene3d 와 별개 layer) |

## 8. 마이그레이션 plan

### 순서 (commit 단위)

1. **`SCENE3D_SNAPSHOT` 신설 + refcount depth_frame** — 새 service 추가, 기존 `POINTCLOUD_*` 그대로 둠. coexist 단계
   - backend: `Scene3DNode` 신설 (또는 `PointCloudNode` 안에 새 service 추가, rename 후속), refcount 로직, snapshot service handler
   - test: snapshot 호출 → fresh frame 받음 / live stream toggle 과 동시 호출 → refcount 정상

2. **Frontend Scene Controls 에 토글 이동 + Layer 유지** — `pointCloudStore.setEnabled` 호출 자리 옮기기. PointCloudPanel 의 "Live Stream" 버튼 제거
   - test: 다른 mode (calibrate / move) 에서 토글 → 점군 보임

3. **`rgbd` capability rename + gate 적용** — robots.yaml `scan` → `rgbd`, 모든 caller grep 교체 (backend `enabled_robot_ids` 필터 + frontend 사이드바 / 토글 check)
   - test: omx_f_0 (capabilities 에 `rgbd` 없음) 에서 Scene3D 토글 / scan mode 비노출

4. **`PointCloudNode` → `Scene3DNode` rename** — file rename + 클래스 rename + topic_map / api_contract / node_registry 갱신 + frontend store/component rename
   - rename-only commit, behavior 변경 X
   - test: 부팅 + 기존 동작 동일

5. **(Phase 2 storage 진입 시) `capture_scan` 을 storage_node 의 `STORAGE_PUT_SCAN` 호출로 swap** — scan_io 직접 호출 → storage 거치도록. TSDF build 도 같이 — storage 에서 scan blob 받아 build
   - 본 plan 범위 밖, [storage_layer.md §11 Phase 2](storage_layer.md) anchor

### 면적 추정

- **backend**: ~300-500 LOC (신규 service + refcount + rename + capability gate)
- **frontend**: ~200-300 LOC (toggle 이동 + store/component rename + gate)
- **테스트**: snapshot service / refcount edge case (동시 진입) / capability gate

## 9. 미해결 / 보류 자리

### 9.1 캘 verification UI 자체

본 plan 은 capability/primitive 만 만들고, "캘 후 ghosting 시각화 UI" 자체는 별도 자리. snapshot service + 메모리 fuse 가 있으면 만들 수 있지만 UI/UX 설계 자리 별도.

→ 본 plan 끝난 뒤 별도 doc / session.

### 9.2 분산 토폴로지에서 depth_frame 의 LAN 비용

현재 [distributed_topology.md](distributed_topology.md) anchor 상 카메라 Pi → PC depth_frame 가 LAN 으로 흐름. refcount 가 idle 자리 0 으로 만들지만, 라이브 토글 ON 시 ~3-5 MB/s 가 LAN 위. WiFi 자리에서 한계 자리 확인 필요.

→ hardware 자리 검증 (집).

### 9.3 voxel_size advanced 자리

snapshot 의 voxel_size 인자 / live stream voxel_size 의 frontend control 자리. 일단 default 박고 advanced 자리 미룸.

→ 사용자 피드백 시점에 settings modal 또는 expandable section 으로 추가.

### 9.4 미래 3D detection / grasp planning use case

snapshot service primitive 위에 얹힐 자리. 구체 API 는 use case 진입 시 별도.

→ 본 plan 의 primitive 가 만족하는지만 확인 (snapshot frame data shape 이 그쪽 use case 도 fit) — fit 안 하면 본 plan 의 시그니처 revisit.

## 10. 다음 세션 anchor

본 plan 구현 진입 시:

1. 결정 4 (capability rename) 의 사용자 최종 confirm — `scan` 유지 vs `rgbd` rename. 본 doc 권고는 `rgbd` 지만 사용자 선호 들어보고
2. 결정 1 의 wire format 자리 — `frame_bytes` 가 우리 기존 `POINTCLOUD_STREAM` 의 `[u32 n][n*3 float32 xyz][n*3 uint8 rgb]` 와 동일 컨벤션. service response 가 bytes 직접 받을 수 있는지 transport 확인 (Pydantic StrictModel + bytes field — Pydantic OK, Zenoh wire OK)
3. 순서 §8 따라 commit. 1-2 commit 끝나면 hardware 자리에서 동작 확인 (집) 후 3-4

본 plan 의 핵심 — **3D 캡처는 카메라 JPEG 같은 primitive 데이터 자리. workflow 가 아니다.** 모든 결정이 이 invariant 의 derive.
