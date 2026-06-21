# Scan / TSDF Pipeline Readiness — SO-101 시작 전 검토 (2026-06-21)

> SO-101 + D405 실 hardware 로 첫 scan task 시작 전 코드 전수 검토.
> 4-노드 분리 (Scene3D / Storage / Reconstruction / ScanTask) 구조는 mature 하나
> SO-101 instance 의 `robot_poses.yaml` missing + frontend mesh 시각화 gap 두 자리 자리.

## 상태 요약

| 모듈 | 상태 | 비고 |
|---|---|---|
| [Scene3DNode](backend/nodes/application/scene3d_node.py) | ✓ active | depth_frame 구독 → snapshot/stream 서비스. [consensus.py](backend/modules/scene3d/consensus.py) 10-frame median 구현됨 |
| [ReconstructionNode](backend/nodes/application/reconstruction_node.py) | ✓ active | multi-way ICP + PoseGraph + TSDF + mesh extract 완전. progress 5 stage publish |
| [build.py](backend/modules/reconstruction/build.py) | ✓ active | ICP + TSDF kernel |
| StorageNode (scan workflow handlers) | ✓ active | 10 service (sessions/scans/reconstructions CRUD + blob). RdbStore + ObjectStore |
| [ScanTask](backend/modules/task/tasks/scan.py) | ✓ active | NewSession → ForEach(MoveJByName + CaptureScan) → BuildReconstruction |
| [CameraNode (D405)](backend/nodes/device/camera_node.py) | ✓ active | 30Hz JPEG + 8Hz depth_frame publish. `CAMERA_SET_DEPTH_STREAM` refcount |
| [depth_frame.py](backend/modules/camera/depth_frame.py) | ✓ active | JSON header + JPEG color + zstd depth — 분산 LAN transfer 최적화됨 |
| [api_contract.py](backend/api_contract.py) | ✓ active | SCENE3D_SNAPSHOT / RECONSTRUCTION_BUILD / STORAGE_* 다 PUBLIC_SERVICES 등재 |
| [host_dev.yaml](backend/config/host_dev.yaml) | ✓ active | storage + scene3d + reconstruction + camera 다 active |
| [robots.yaml](robot/robots.yaml) — so101_6dof_0 | ✓ active | enabled + `rgbd` capability 있음 |
| frontend TasksPage | ✓ active | `useTasks()` + TASK_REGISTRY 의 `scan` 등록 (`required_capabilities=["rgbd"]`) |
| Hand-Eye 캘 (so101 5종) | ✓ active | **DB run_id=2 의 id=6/7/8/9 active** (storage_layer 자리 자리 자리 npz 자리 자리 자리 자리 자리, [handeye_sigma_floor_so101.md](handeye_sigma_floor_so101.md)) |
| **`robot/instances/so101_6dof_0/robot_poses.yaml`** | ❌ **missing** | omx_f_0 에는 있음. **scan 시작 시 BLOCKING** — `MoveJByName("home")` KeyError |
| Frontend mesh layer (PLY 시각화) | ❌ missing | Reconstruction 결과 PLY blob 자리 박혀도 *frontend 자리 자리 자리 자리 X* — 사용자가 결과 그림 못 봄 |

## BLOCKING — scan 시작 전 필수

### 1. `robot_poses.yaml` 생성
- 경로: `robot/instances/so101_6dof_0/robot_poses.yaml`
- 최소 `home` 자세 + scan 자세 3-5개
- omx_f_0 의 [robot_poses.yaml](robot/instances/omx_f_0/robot_poses.yaml) 형식 참고
- **사용자가 robot 자세 직접 잡고 motor position 읽어서 박는 게 정확** — 6 DOF 자유롭게 가능, 자세별 scan 대상 물체 다른 angle 보이게

### 2. Frontend PLY mesh layer
- Reconstruction 끝나면 `STORAGE_PUT_RECONSTRUCTION` 자리 PLY blob 박힘 — *사용자가 결과 검증할 자리 자리*
- frontend 3D canvas 자리 mesh layer 추가:
  - `STORAGE_LIST_RECONSTRUCTIONS` → 최신 active 가져오기
  - `STORAGE_GET_BLOB` → PLY 다운로드
  - three.js PLYLoader 자리 자리 자리 + Scene3DLayer 옆 자리 mesh group
- 효과 — visual quality 즉시 확인 가능. 안 짜면 결과 있어도 검증 X

## Secondary (runtime watch)

### Motor connection
- [JointStateCache.get_raw_motor_positions()](backend/core/cache/joint_state_cache.py) 자리 motor_state 없으면 snapshot fail
- MotorNode 20Hz publish 정상 자리 OK. SO-101 첫 부팅 시 connection 실패하면 timing issue 가능
- 시작 전 Sidebar 의 joint display 활성 + 숫자 변화 확인

### Snapshot N-frame consensus
- [`DEFAULT_VOXEL_SIZE = 0.005`](backend/nodes/application/scene3d_node.py) (5mm) + [`N_FRAMES_DEFAULT = 10`](backend/modules/scene3d/consensus.py) → snapshot 1.25s @ 8 FPS
- 손떨림 / robot 움직임 있으면 median 효과 감소. *robot settle 후 capture* 권장
- `CaptureScan` step 의 `num_frames` parameter 자리 override 가능 (ScanTask config 자리 자리 자리 박을 자리)

### Hand-eye σ_t 7.53mm 영향
- TSDF GOOD threshold (σ_t <10mm) 안 — scan 자리 충분
- 단 detail 자리 자리 자리 ChArUco 자리 자리 자리 자리 자리 자리 자리 자리 자리 (multi-pose alignment drift ~7mm)

## 미구현 (필요 자리 때 추가)

| 항목 | 현황 | 필요성 |
|---|---|---|
| Colored ICP | OFF (design spec) | 텍스처 의존성 자리 자리 자리. 첫 scan 결과 보고 결정 |
| Mesh smoothing | OFF | detail 손실 우려 |
| Fragment stage | OFF | scan <10개 자리 자리 자리 자리 자리. 자리 자리 자리 자리 자리 |
| Multi-robot TSDF fusion | 설계만 | ReconstructionNode 자리 `list[robot_id]` 받게 자리 (Phase 2+) |

## 첫 scan 체크리스트

### MUST
1. **`robot_poses.yaml` 생성** — 5분, 사용자 + 코드. home + scan poses 3-5
2. **Frontend mesh layer 추가** — 1-2일 (PLY loader + scene 3D group)

### SHOULD
3. **Mock backend e2e scan** — 20분. `--host mock` 으로 task 실행 → contract 흐름 + storage 박힘 검증 (합성 depth 자리 자리 자리 자리 자리 자리 자리 자리 자리 자리)
4. **Scan poses 디자인** — 15분. 대상 물체 배치 후 4-6 자세 (3-view overlap)
5. **Motor 연결 검증** — 10분. 부팅 후 sidebar joint display 확인

### CAN (나중에)
6. Colored ICP / mesh smoothing (시각 검증 결과 따라)
7. Multi-robot TSDF fusion (2대+ robot)

## 관련 문서

- [tsdf_pipeline.md](tsdf_pipeline.md) — multi-way ICP + TSDF 빌드 결정사항 (design)
- [storage_layer.md](storage_layer.md) — Phase 2 scans/reconstructions schema
- [handeye_sigma_floor_so101.md](handeye_sigma_floor_so101.md) — SO-101 hand-eye 캘 floor (scan 자리 자리 자리 σ_t 7.53mm 적용)
