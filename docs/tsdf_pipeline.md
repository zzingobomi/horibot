# TSDF Pipeline 구현 가이드

> **이 문서의 목적**: 다음 세션의 Claude가 컨텍스트 없이 읽고 바로 구현 시작 가능하게 하는 self-contained 명세. 결정사항/이유/코드 위치/Open3D 호출 패턴까지 박혀 있음.
>
> **읽기 순서**: §1 컨텍스트 → §2 결정 요약 → §3 참고 자료 위치 → §4부터 구현.

---

## 1. 왜 이걸 하는가 (컨텍스트)

OMX_F는 D405 RGBD + 5DOF arm. 현재 라이브 단일 frame 포인트클라우드 스트림만 있음 ([backend/nodes/pointcloud_node.py](../backend/nodes/pointcloud_node.py)). 여러 자세에서 캡처한 RGBD를 base 프레임으로 정합해 **TSDF mesh**로 합치는 게 이번 작업.

캘 현황 (BA doc § 16 기준): σ_rot **0.65°** / σ_t **7.94mm** (확장 BA + 물리 sag 모델 적용 완료). TSDF GOOD threshold(σ_rot <1°, σ_t <10mm) 안에 들어와 있음.

**원칙: TSDF 품질은 타협하지 않음**. 캘 σ_t ~8mm를 그대로 박으면 voxel 2mm TSDF에서 2~4 voxel 어긋남이 mesh 두께/이중벽으로 나타남. 그래서 **처음부터 ICP refinement + multi-way pose graph optimization**까지 깔고 시작. 산업 표준 Open3D RGBD reconstruction tutorial 흐름.

**오버엔지니어링 경계** — 자동화 가드(모터 속도 검증, 환경 통제 자동화, KPI 회귀 감지 등)는 안 깐다. 캡처는 사람이 수동으로 정지 후 클릭. 자세 수도 10개 안팎이라 fragment 단계 / colored ICP / mesh smoothing은 처음 commit에서 제외 — 결과 보고 부족하면 그때.

---

## 2. 결정 요약

| 항목 | 결정 | 이유 |
|------|------|------|
| Capture pose 출처 | **(c) raw motor positions 박고 build 때 재계산** | 캘 진화 robust. capture 단계가 캘 변환 안 끼게 |
| build_mesh의 actual_ee 계산 | `PybulletSolver.fk(joint_angles_rad)` 직접 호출 | solver가 이미 sag/link 모두 적용해 actual ee 반환 |
| ICP refinement | **point-to-plane**, 인접 자세 + 가까운 페어 | 캘 floor를 mm 단위로 끌어내림. RGBD reconstruction 표준 |
| Pose graph optimization | **multi-way** 무조건 같이 | 누적 드리프트 제거. Open3D 함수 호출 수 줄이라 비용 작음 |
| Colored ICP | **안 깜** | 텍스처 의존 + JPEG 압축 영향. point-to-plane 부족할 때 추가 |
| Fragment 단계 | **안 깜** | 자세 10개 안팎이라 불필요 |
| Mesh smoothing | **안 깜** | 디테일 손실 우려. 비타협 원칙이면 default OFF |
| Depth pre-filter | **bilateral filter** | edge 보존하면서 D405 stereo 노이즈 ↓ |
| Post-process | **cluster_connected_triangles**로 작은 fragment 제거 | 떠다니는 mesh 정리 |
| §17.2 옵션 C (frontend 시각화) | **별건. 이번 commit 범위 X** | TSDF 본체와 독립. 시각 검증 안 깔끔하면 그때 |

---

## 3. 참고 자료 위치

### 3a. 이전 시도 — `feat/pcd-capture` 브랜치 (main+2)

```
5724f5c "feat: capture 기능 적용"
eac7e7d "feat: TSDF 적용"
```

읽는 법: `git show 5724f5c -- backend/nodes/pointcloud_node.py` 같은 식. **그대로 가져오지 말 것** — 그때는 분산 토폴로지 도입 전 + BA 도입 전이라 다음 두 가지가 다름:
- (1) `_srv_capture_depth_frames`라는 *camera 노드 동기 RPC*를 새로 추가했음. **불필요** — 지금은 PointCloudNode가 이미 `CAMERA_DEPTH_FRAME`(8 FPS) 구독 중. capture 호출 시 `_latest_frame`을 N번 폴링/대기로 모으면 됨.
- (2) `T_cam_to_base`를 commanded angle FK로 계산했음. **틀림** — 지금은 `PybulletSolver.fk`가 sag+link 적용 actual 반환. 또는 raw motor → 다시 계산.

가져올 수 있는 거:
- TSDF 빌드 `_srv_build_mesh`의 ScalableTSDFVolume 사용 패턴은 거의 그대로 OK ([git show eac7e7d -- backend/nodes/pointcloud_node.py](#))
- 프론트엔드 `MeshLayer.tsx`의 PLYLoader + z-up→y-up 회전 패턴 그대로 OK
- 세션 관리/디렉토리 구조: `robot/scans/{session_id}/scan_NNN.npz`, `robot/models/mesh_*.ply`

### 3b. 현재 코드의 핵심 진입점들

| 위치 | 용도 |
|------|------|
| [backend/nodes/pointcloud_node.py](../backend/nodes/pointcloud_node.py) | 확장 대상. 현재 187줄, 라이브 스트림만 |
| [backend/core/transport/topic_map.py](../backend/core/transport/topic_map.py) | Topic/Service 키 추가 |
| [backend/modules/camera/depth_frame.py](../backend/modules/camera/depth_frame.py) | `DepthFrame` dataclass + `decode/encode` |
| [backend/modules/kinematics/solver.py](../backend/modules/kinematics/solver.py) | `PybulletSolver.fk(joint_angles_rad) → (pos, quat)` — sag+link 적용된 actual ee |
| [backend/modules/kinematics/fk_chain.py](../backend/modules/kinematics/fk_chain.py) | numpy FK + `apply_gravity_sag` (참고용. solver.fk만 쓰면 호출 안 해도 됨) |
| [backend/core/cache/joint_state_cache.py](../backend/core/cache/joint_state_cache.py) | `get_raw_motor_positions(arm_cfgs) → dict[int, int]` ← capture에 쓸 것 |
| [backend/core/coords/joint_coordinates.py](../backend/core/coords/joint_coordinates.py) | `motor_to_urdf(raw, cfg) → rad` — capture raw로부터 URDF rad 변환 |
| [backend/core/coords/link_coordinates.py](../backend/core/coords/link_coordinates.py) | `LinkCoordinates().snapshot()` — link offsets 메타 |
| [backend/core/coords/sag_coordinates.py](../backend/core/coords/sag_coordinates.py) | sag stiffness 메타 |
| [backend/modules/calibration/loader.py](../backend/modules/calibration/loader.py) | `load_calibration()` → `CalibrationData` (intrinsic + hand_eye) |
| `robot/calibration/hand_eye.npz` | `R` (3x3), `t` (3x1) — T_cam2gripper |
| `robot/calibration/{joint,link,sag}_offsets.npz` | 캘 메타 (mtime을 capture npz에 박을 거) |
| [frontend/src/store/pointCloudStore.ts](../frontend/src/store/pointCloudStore.ts) | 확장 대상 |
| [frontend/src/components/workspace3d/panels/PointCloudPanel.tsx](../frontend/src/components/workspace3d/panels/PointCloudPanel.tsx) | 확장 대상 |
| [frontend/src/components/workspace3d/3d/RobotScene.tsx](../frontend/src/components/workspace3d/3d/RobotScene.tsx) | MeshLayer 추가 |
| [frontend/src/constants/topics.ts](../frontend/src/constants/topics.ts) | Service 키 추가 |

### 3c. 분산 토폴로지 주의

PointCloudNode가 도는 머신:
- `host_dev`: 단일 풀스택 (모든 노드)
- `host_pc`: 분산 PC (pointcloud 포함, motor/motion/camera는 분리)

PointCloudNode가 도는 머신엔 `PybulletSolver`가 *반드시* 있음 (calibration/task가 같은 머신). 즉 build_mesh에서 `PybulletSolver().fk(...)` 직접 호출 안전.

분산일 때 카메라는 다른 머신(`host_pi_camera`). depth_frame은 Zenoh 토픽으로 흐르고, PointCloudNode가 raw subscriber로 받음. **이미 구현돼 있음** — capture 로직은 그 `_latest_frame` 캐시를 N번 폴링해서 모으면 끝.

### 3d. Open3D 좌표계 / 함수 컨벤션 (중요)

- `volume.integrate(rgbd, intrinsic, extrinsic)` — **extrinsic = T_camera ← world** (world point를 camera frame으로 변환). 즉 `T_cam_to_base`의 *역행렬*을 넘김.
- `registration_icp(source, target, max_distance, init, ...)` — **transformation = T_target ← source** (source point를 target frame으로). pair-wise registration에서 자세 i를 자세 0 기준으로 정합한다면 init = T_0←i.
- PoseGraph의 `node[i].pose` — **T_world ← cam_i**. Open3D 컨벤션이라 ICP 결과와 inverse 관계라 헷갈리기 쉬움. 항상 명시적으로 변환하라.
- PointCloud는 base 프레임으로 변환한 후 일관된 정합 권장. 즉 cloud = `pcd.transform(T_base ← cam_i)` 처리 후 ICP. 또는 ICP를 cam frame에서 풀고 결과를 PoseGraph node에 박는다.

좌표계 변환 한 번 정리:
```
T_base←cam_i = T_base←ee_i  ·  T_ee←cam      (hand_eye = T_cam2gripper = T_ee←cam)
T_cam_i←base = inv(T_base←cam_i)
TSDF.integrate의 extrinsic = T_cam_i←base
ICP 결과를 PoseGraph node에 박을 때 = T_base←cam_i (= world←cam_i)
```

---

## 4. 데이터 스키마

### 4a. scan_*.npz 스키마

각 자세에서 캡처한 raw 데이터 + 캘 메타. *(c) 방식*이라 변환된 pose는 박지 않음.

```python
np.savez_compressed(
    scan_path,
    # ─── RGBD raw (depth_frame의 직접 추출) ───
    color_bgr=color_bgr,          # (H, W, 3) uint8 — capture 시점 마지막 프레임
    depth_z16=depth_z16,          # (H, W) uint16 — N프레임 median
    # ─── camera intrinsic (D405 factory or recalibrated, depth_frame에서 옴) ───
    fx=fx, fy=fy, cx=cx, cy=cy,   # float64
    width=width, height=height,    # int32
    depth_scale=depth_scale,       # float64 (D405는 보통 0.0001 m/unit)
    # ─── robot raw state (FK 미적용) ───
    raw_motor_positions=raw_pos_array,   # (5,) int32 — JointStateCache.get_raw_motor_positions
    arm_motor_ids=motor_ids,             # (5,) int32 — capture 시점 arm_cfgs의 id 순서
    # ─── 캘 파일 메타 (build 시점 일관성 검증용) ───
    calib_meta=json.dumps({               # str → np.array(str) 박기
        "joint_offsets_mtime": float(joint_offsets_path.stat().st_mtime),
        "link_offsets_mtime":  float(link_offsets_path.stat().st_mtime),
        "sag_offsets_mtime":   float(sag_offsets_path.stat().st_mtime),
        "hand_eye_mtime":      float(hand_eye_path.stat().st_mtime),
        "intrinsic_mtime":     float(intrinsic_path.stat().st_mtime),
    }),
    # ─── 캡처 메타 ───
    timestamp=time.time(),         # float64
    num_frames=num_frames,         # int32 (예: 10)
)
```

`raw_motor_positions`를 박는 이유: capture 시점 캘이 변해도 (joint offset 갱신, sag 모델 변경 등) raw 값은 변하지 않음. build_mesh가 *현재 캘*로 재계산. raw가 ground truth.

`calib_meta`는 단순 mtime이면 충분. build 시점에 mtime이 다르면 로그 경고 후 진행 (refuse는 X — 사용자가 의도적으로 캘 갱신 후 재빌드할 수 있으니).

### 4b. 디렉토리 구조

```
robot/
├── scans/
│   └── {session_id}/        # e.g. session_20260522_153012 or user-supplied
│       ├── scan_001.npz
│       ├── scan_002.npz
│       └── ...
└── models/
    └── mesh_{session_id}.ply
```

`.gitignore`에 `robot/scans/`, `robot/models/` 추가 (대용량 + 머신별).

`session_id` 정규식: `^[A-Za-z0-9_\-]+$` (path traversal 방지).

---

## 5. 신규 토픽 / 서비스 키

### 5a. Backend ([backend/core/transport/topic_map.py](../backend/core/transport/topic_map.py))

`Service` 클래스에 추가:
```python
# PointCloud — capture
POINTCLOUD_NEW_SESSION  = "omx/pointcloud/srv/new_session"
POINTCLOUD_CAPTURE      = "omx/pointcloud/srv/capture"
POINTCLOUD_LIST_SESSIONS = "omx/pointcloud/srv/list_sessions"
POINTCLOUD_LIST_SCANS   = "omx/pointcloud/srv/list_scans"
POINTCLOUD_DELETE_SCAN  = "omx/pointcloud/srv/delete_scan"
# PointCloud — TSDF
POINTCLOUD_BUILD_MESH   = "omx/pointcloud/srv/build_mesh"
POINTCLOUD_LIST_MESHES  = "omx/pointcloud/srv/list_meshes"
```

`Topic`은 `POINTCLOUD_SNAPSHOT` 이미 있음 — capture 직후 미리보기 발행에 재사용 가능 (생략해도 무방).

### 5b. Frontend ([frontend/src/constants/topics.ts](../frontend/src/constants/topics.ts))

`ServiceKey`에 위 7개 동일 문자열로 미러링. **문자열 정확히 일치 필수**.

### 5c. Bridge `_ALWAYS_SUBSCRIBE` ([backend/bridge/zenoh_bridge.py](../backend/bridge/zenoh_bridge.py))

신규 토픽 없음 (서비스 응답만 사용). 추가 변경 X. PLY 파일은 `/robot` 정적 마운트로 자동 노출 (이미 마운트됨).

---

## 6. Backend 구현

### 6a. 파일 배치

신규:
- `backend/modules/pointcloud/__init__.py` (빈 파일)
- `backend/modules/pointcloud/scan_io.py` — scan_*.npz 저장/로드 + 디렉토리 헬퍼
- `backend/modules/pointcloud/scan_capture.py` — capture 로직 (depth_frame N개 모아 median)
- `backend/modules/pointcloud/tsdf_builder.py` — ICP + PoseGraph + TSDF 통합 빌드

확장:
- `backend/nodes/pointcloud_node.py` — 신규 서비스 핸들러들 (얇은 wrapper, 비즈니스 로직은 위 모듈로 위임)
- `backend/core/transport/topic_map.py` — § 5a
- `.gitignore` — `robot/scans/`, `robot/models/`

### 6b. `scan_io.py` 핵심 함수

```python
ROBOT_DIR  = Path(__file__).parents[3] / "robot"
SCANS_DIR  = ROBOT_DIR / "scans"
MODELS_DIR = ROBOT_DIR / "models"
CALIB_DIR  = ROBOT_DIR / "calibration"

_SESSION_RE = re.compile(r"^[A-Za-z0-9_\-]+$")

def validate_session_id(sid: str) -> str:
    if not _SESSION_RE.match(sid):
        raise ValueError("session_id: 영문/숫자/_- 만")
    return sid

def make_default_session_id() -> str:
    return time.strftime("session_%Y%m%d_%H%M%S")

def session_dir(sid: str) -> Path:
    return SCANS_DIR / validate_session_id(sid)

def next_scan_index(session_dir_: Path) -> int:
    existing = sorted(session_dir_.glob("scan_*.npz"))
    return len(existing) + 1

def calib_meta_dict() -> dict:
    """robot/calibration/*.npz의 mtime 묶음."""
    paths = {
        "joint_offsets_mtime": CALIB_DIR / "joint_offsets.npz",
        "link_offsets_mtime":  CALIB_DIR / "link_offsets.npz",
        "sag_offsets_mtime":   CALIB_DIR / "sag_offsets.npz",
        "hand_eye_mtime":      CALIB_DIR / "hand_eye.npz",
        "intrinsic_mtime":     CALIB_DIR / "intrinsic.npz",
    }
    return {k: (p.stat().st_mtime if p.exists() else 0.0) for k, p in paths.items()}

def save_scan(scan_path: Path, *, color_bgr, depth_z16, fx, fy, cx, cy,
              width, height, depth_scale, raw_motor_positions, arm_motor_ids,
              num_frames) -> None:
    """위 § 4a 스키마로 저장."""
    np.savez_compressed(
        scan_path,
        color_bgr=color_bgr,
        depth_z16=depth_z16,
        fx=np.float64(fx), fy=np.float64(fy),
        cx=np.float64(cx), cy=np.float64(cy),
        width=np.int32(width), height=np.int32(height),
        depth_scale=np.float64(depth_scale),
        raw_motor_positions=np.asarray(raw_motor_positions, dtype=np.int32),
        arm_motor_ids=np.asarray(arm_motor_ids, dtype=np.int32),
        calib_meta=json.dumps(calib_meta_dict()),
        timestamp=np.float64(time.time()),
        num_frames=np.int32(num_frames),
    )

def load_scan(scan_path: Path) -> dict:
    s = np.load(scan_path, allow_pickle=False)
    return {
        "color_bgr": s["color_bgr"],
        "depth_z16": s["depth_z16"],
        "fx": float(s["fx"]), "fy": float(s["fy"]),
        "cx": float(s["cx"]), "cy": float(s["cy"]),
        "width": int(s["width"]), "height": int(s["height"]),
        "depth_scale": float(s["depth_scale"]),
        "raw_motor_positions": s["raw_motor_positions"].tolist(),
        "arm_motor_ids": s["arm_motor_ids"].tolist(),
        "calib_meta": json.loads(str(s["calib_meta"])),
        "timestamp": float(s["timestamp"]),
        "num_frames": int(s["num_frames"]),
    }
```

### 6c. `scan_capture.py` 핵심 흐름

```python
DEPTH_STREAM_WAIT_TIMEOUT = 3.0    # depth 스트림 안 들어오면 실패
N_FRAMES_DEFAULT = 10
FRAME_GATHER_TIMEOUT = 5.0          # N프레임 모으는 데 최대 대기

def gather_frames(
    get_latest_frame: Callable[[], DepthFrame | None],
    n: int = N_FRAMES_DEFAULT,
) -> list[DepthFrame]:
    """`_latest_frame`을 폴링해서 *서로 다른 timestamp*의 frame n개 모음.

    depth_frame이 8 FPS로 들어오므로 n=10이면 1.25s 정도. 그동안 사용자는
    이미 정지해 있다고 가정 (수동 캡처).
    """
    out: list[DepthFrame] = []
    last_ts = -1.0
    deadline = time.time() + FRAME_GATHER_TIMEOUT
    while len(out) < n and time.time() < deadline:
        f = get_latest_frame()
        if f is not None and f.timestamp > last_ts + 1e-6:
            out.append(f)
            last_ts = f.timestamp
        else:
            time.sleep(0.02)
    if len(out) < n:
        raise TimeoutError(f"depth_frame {n}장 못 모음: {len(out)}장만 들어옴")
    return out

def consensus_depth(frames: list[DepthFrame]) -> np.ndarray:
    """N장의 depth_z16을 픽셀별 median으로 합침.

    median이라 outlier에 robust. 0(invalid) 픽셀은 별도 처리:
    절반 이상이 invalid면 결과도 invalid(0). 아니면 nonzero만으로 median.
    """
    stack = np.stack([f.depth_z16 for f in frames], axis=0)   # (N, H, W) uint16
    h, w = stack.shape[1:]
    out = np.zeros((h, w), dtype=np.uint16)
    valid_mask = stack > 0                                     # (N, H, W) bool
    valid_count = valid_mask.sum(axis=0)                       # (H, W) int
    threshold = (len(frames) + 1) // 2                          # 과반 valid 요구
    # 빠른 path: 전부 valid인 픽셀은 그냥 median
    all_valid = valid_count == len(frames)
    if all_valid.any():
        out[all_valid] = np.median(stack[:, all_valid], axis=0).astype(np.uint16)
    # 일부 valid (>= threshold): nonzero만으로 median
    partial = (valid_count >= threshold) & ~all_valid
    if partial.any():
        # 픽셀별 sort 후 valid 중 중앙값 — 단순 loop 또는 mask 후 nanmedian
        masked = np.where(valid_mask, stack, np.nan).astype(np.float32)
        with np.errstate(all="ignore"):
            med = np.nanmedian(masked, axis=0)
        out[partial] = np.nan_to_num(med[partial], nan=0).astype(np.uint16)
    return out

def consensus_color(frames: list[DepthFrame]) -> np.ndarray:
    """color는 마지막 frame 사용 — JPEG 압축 손실이라 median 의미 작음.

    노이즈 더 짜내고 싶으면 픽셀별 mean이 옵션. 일단 last frame.
    """
    return frames[-1].color_bgr.copy()
```

### 6d. `tsdf_builder.py` — 산업 표준 ICP+PoseGraph+TSDF

```python
import open3d as o3d
import numpy as np
from pathlib import Path

# ─── 디폴트 파라미터 (§ 9 참조) ───
DEFAULT_VOXEL_SIZE  = 0.002   # 2mm
DEFAULT_SDF_TRUNC   = 0.010   # 10mm (= 5 × voxel)
DEFAULT_DEPTH_TRUNC = 0.5     # m — D405 sweet spot 안
DEFAULT_ICP_MAX_DIST = 0.010  # m — voxel 5배
DEFAULT_BILATERAL_DIAMETER = 5
DEFAULT_BILATERAL_SIGMA_COLOR = 50.0
DEFAULT_BILATERAL_SIGMA_SPACE = 50.0
MIN_TRIANGLE_CLUSTER_SIZE = 500    # 그보다 작은 cluster는 제거


def build_mesh(
    scans: list[dict],            # scan_io.load_scan 결과 리스트
    voxel_size: float = DEFAULT_VOXEL_SIZE,
    sdf_trunc: float = DEFAULT_SDF_TRUNC,
    depth_trunc: float = DEFAULT_DEPTH_TRUNC,
    icp_max_dist: float = DEFAULT_ICP_MAX_DIST,
    bilateral_diameter: int = DEFAULT_BILATERAL_DIAMETER,
    out_path: Path,
) -> BuildResult:
    """
    1. scan별 (rgbd, intrinsic, T_base_cam_init) 준비
       - raw_motor_positions → JointCoordinates.motor_to_urdf → arm rad
       - PybulletSolver().fk(arm_rad) → (pos_actual, quat_actual)
       - T_base_ee_actual = compose(pos, quat)
       - T_base_cam = T_base_ee · T_ee_cam (hand_eye)
    2. depth bilateral filter
    3. RGBD → PointCloud (cam frame), normal 추정
    4. pair-wise point-to-plane ICP — 인접 + 가까운 페어
    5. PoseGraph 빌드 → global_optimization
    6. refined T_base_cam으로 TSDF integrate (extrinsic = inv(T_base_cam))
    7. extract_triangle_mesh + cluster_connected_triangles 정리
    8. PLY 저장
    """
    # ─── 1. 자세별 초기 T_base_cam 계산 ────────────────────────────
    from core.joint_coordinates import JointCoordinates
    from modules.kinematics.solver import PybulletSolver
    from modules.motor.motor_config import MotorConfig
    from modules.calibration.loader import load_calibration

    calib = load_calibration()
    if calib.hand_eye is None or calib.intrinsic is None:
        raise RuntimeError("hand_eye.npz / intrinsic.npz 없음")

    T_ee_cam = np.eye(4)
    T_ee_cam[:3, :3] = calib.hand_eye.R
    T_ee_cam[:3, 3]  = calib.hand_eye.t.reshape(3)

    solver = PybulletSolver()
    coords = JointCoordinates()

    T_base_cam_init: list[np.ndarray] = []
    pcds: list[o3d.geometry.PointCloud] = []
    rgbds: list[o3d.geometry.RGBDImage] = []
    intrinsics: list[o3d.camera.PinholeCameraIntrinsic] = []

    for s in scans:
        # raw motor → URDF rad (joint_offsets 적용)
        # arm_cfgs 재구성 필요 — id, reverse만 있으면 됨. motor_to_urdf 시그니처 확인.
        # MotorConfig 가져오기 어려우면 JointCoordinates 패턴 보고 직접 변환:
        #   raw - 2048 → reverse 반영 → rad → joint_offset 가산.
        # 가장 깔끔: PointCloudNode가 자기 _arm_cfgs를 build_mesh에 넘김.
        arm_rad = _raw_to_urdf_rad(s["raw_motor_positions"], s["arm_motor_ids"])

        # solver.fk가 sag+link 다 적용된 actual ee를 반환
        pos, quat = solver.fk(arm_rad)
        T_base_ee = _pose_to_matrix(pos, quat)
        T_bc = T_base_ee @ T_ee_cam
        T_base_cam_init.append(T_bc)

        # depth bilateral filter (uint16 → float32 → bilateral → uint16)
        depth_f = s["depth_z16"].astype(np.float32)
        depth_f = cv2.bilateralFilter(
            depth_f, bilateral_diameter,
            DEFAULT_BILATERAL_SIGMA_COLOR, DEFAULT_BILATERAL_SIGMA_SPACE,
        )
        depth_filtered = depth_f.astype(np.uint16)

        # RGBD 생성
        color_rgb = np.ascontiguousarray(s["color_bgr"][:, :, ::-1])
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(color_rgb),
            o3d.geometry.Image(depth_filtered),
            depth_scale=1.0 / s["depth_scale"],
            depth_trunc=depth_trunc,
            convert_rgb_to_intensity=False,
        )
        intrinsic = o3d.camera.PinholeCameraIntrinsic(
            s["width"], s["height"], s["fx"], s["fy"], s["cx"], s["cy"],
        )
        rgbds.append(rgbd)
        intrinsics.append(intrinsic)

        # cloud (camera frame). normal 추정 (point-to-plane ICP 필수).
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)
        # voxel down은 ICP 속도용. TSDF integrate엔 원본 depth/rgb 사용.
        pcd_down = pcd.voxel_down_sample(voxel_size)
        pcd_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2.0, max_nn=30)
        )
        pcds.append(pcd_down)

    n = len(scans)

    # ─── 4. Pair-wise ICP ─────────────────────────────────────────
    # 인접: (i, i+1). 가까운 페어: 거리 임계 안의 (i, j) j>i+1.
    # 거리는 T_base_cam_init의 cam center 차이.
    cam_centers = np.array([T[:3, 3] for T in T_base_cam_init])
    pair_uncertain_dist = 0.15   # m — 이 안의 페어는 loop closure 후보

    edges: list[tuple[int, int, np.ndarray, np.ndarray, bool]] = []
    # (i, j, T_target_source, information, uncertain)

    for i in range(n - 1):
        # 인접 페어 (i, i+1) — uncertain=False
        T_cami_camj_init = np.linalg.inv(T_base_cam_init[i]) @ T_base_cam_init[i + 1]
        result = o3d.pipelines.registration.registration_icp(
            source=pcds[i + 1], target=pcds[i],
            max_correspondence_distance=icp_max_dist,
            init=T_cami_camj_init,
            estimation_method=
                o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        )
        info = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
            pcds[i + 1], pcds[i], icp_max_dist, result.transformation,
        )
        edges.append((i, i + 1, result.transformation, info, False))

    for i in range(n):
        for j in range(i + 2, n):
            dist = float(np.linalg.norm(cam_centers[i] - cam_centers[j]))
            if dist > pair_uncertain_dist:
                continue
            T_init = np.linalg.inv(T_base_cam_init[i]) @ T_base_cam_init[j]
            result = o3d.pipelines.registration.registration_icp(
                source=pcds[j], target=pcds[i],
                max_correspondence_distance=icp_max_dist,
                init=T_init,
                estimation_method=
                    o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            )
            if result.fitness < 0.3:
                continue  # 정합 약하면 edge에 안 박음
            info = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
                pcds[j], pcds[i], icp_max_dist, result.transformation,
            )
            edges.append((i, j, result.transformation, info, True))

    # ─── 5. PoseGraph 빌드 ─────────────────────────────────────────
    # PoseGraph node[i].pose = T_world←cam_i = T_base←cam_i
    pose_graph = o3d.pipelines.registration.PoseGraph()
    for T in T_base_cam_init:
        pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(T.copy()))
    for (i, j, T_ij, info, uncertain) in edges:
        # ICP transformation: source=j, target=i → T_target←source = T_cam_i←cam_j
        # PoseGraphEdge는 T_target_source (PoseGraph 내부 컨벤션 동일)
        pose_graph.edges.append(
            o3d.pipelines.registration.PoseGraphEdge(
                source_node_id=j,
                target_node_id=i,
                transformation=T_ij,
                information=info,
                uncertain=uncertain,
            )
        )

    # global_optimization — Levenberg-Marquardt
    option = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=icp_max_dist,
        edge_prune_threshold=0.25,
        reference_node=0,                  # node 0의 자세를 anchor로 고정
    )
    o3d.pipelines.registration.global_optimization(
        pose_graph,
        o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
        o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
        option,
    )

    T_base_cam_refined = [pose_graph.nodes[i].pose for i in range(n)]

    # ─── 6. TSDF integrate ─────────────────────────────────────────
    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=voxel_size,
        sdf_trunc=sdf_trunc,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8,
    )
    for i in range(n):
        # Open3D extrinsic = T_camera←world
        extrinsic = np.linalg.inv(T_base_cam_refined[i])
        volume.integrate(rgbds[i], intrinsics[i], extrinsic)

    # ─── 7. Mesh 추출 + cluster 정리 ──────────────────────────────
    mesh = volume.extract_triangle_mesh()
    mesh.compute_vertex_normals()

    # 작은 떠다니는 cluster 제거
    cluster_ids, cluster_sizes, _ = mesh.cluster_connected_triangles()
    cluster_ids = np.asarray(cluster_ids)
    cluster_sizes = np.asarray(cluster_sizes)
    small_clusters = np.where(cluster_sizes < MIN_TRIANGLE_CLUSTER_SIZE)[0]
    if len(small_clusters) > 0:
        triangle_mask = np.isin(cluster_ids, small_clusters)
        mesh.remove_triangles_by_mask(triangle_mask)
        mesh.remove_unreferenced_vertices()

    # ─── 8. 저장 ──────────────────────────────────────────────────
    out_path.parent.mkdir(parents=True, exist_ok=True)
    o3d.io.write_triangle_mesh(str(out_path), mesh, write_vertex_normals=True)

    return BuildResult(
        path=out_path,
        vertex_count=len(mesh.vertices),
        triangle_count=len(mesh.triangles),
        n_scans=n,
        n_edges=len(edges),
        pose_graph_node_poses=T_base_cam_refined,   # 디버깅용
    )
```

`_raw_to_urdf_rad`: PointCloudNode가 arm_cfgs를 들고 있어야 함. `MotorConfig` 객체를 어디서 얻을지 — calibration_node가 `self._arm_cfgs`를 들고 있으니 같은 패턴으로 PointCloudNode도 자기 arm_cfgs 들기. config 로딩은 [backend/main.py](../backend/main.py)의 패턴 참고.

`_pose_to_matrix`: `(pos, quat=[x,y,z,w])` → 4x4. `quaternion → R`은 PyBullet의 `p.getMatrixFromQuaternion` 또는 `scipy.spatial.transform.Rotation`.

### 6e. PointCloudNode 확장 핵심

```python
class PointCloudNode(BaseNode):
    def __init__(self) -> None:
        super().__init__("pointcloud_node")
        # ... 기존 필드 ...
        self._arm_cfgs: list[MotorConfig] = []   # main.py에서 주입 또는 motors.yaml 로드
        self._cache = JointStateCache()
        self._capture_lock = threading.Lock()

    def start(self) -> None:
        # 기존 라이브 stream 서비스 + 신규 capture/build 서비스 등록
        self.create_service(Service.POINTCLOUD_CONFIGURE, self._srv_configure)
        self.create_service(Service.POINTCLOUD_NEW_SESSION, self._srv_new_session)
        self.create_service(Service.POINTCLOUD_CAPTURE, self._srv_capture)
        self.create_service(Service.POINTCLOUD_LIST_SESSIONS, self._srv_list_sessions)
        self.create_service(Service.POINTCLOUD_LIST_SCANS, self._srv_list_scans)
        self.create_service(Service.POINTCLOUD_DELETE_SCAN, self._srv_delete_scan)
        self.create_service(Service.POINTCLOUD_BUILD_MESH, self._srv_build_mesh)
        self.create_service(Service.POINTCLOUD_LIST_MESHES, self._srv_list_meshes)
        self.create_raw_subscriber(Topic.CAMERA_DEPTH_FRAME, self._on_depth_frame)
        super().start()
        self._cache.subscribe(self)   # JointStateCache가 motor state 구독
        # ... 기존 stream thread 시작 ...

    def _srv_capture(self, req: dict) -> dict:
        """req.data: { session_id (필수), num_frames (선택, default 10) }"""
        with self._capture_lock:
            try:
                data = req.get("data", {}) or {}
                sid = scan_io.validate_session_id(str(data.get("session_id", "")))
                num_frames = int(data.get("num_frames", scan_capture.N_FRAMES_DEFAULT))

                # depth 스트림 켜져 있어야 함
                if not self._enabled:
                    return {"success": False, "message": "depth 스트림 OFF 상태 — 먼저 enable", "data": {}}

                # 프레임 모으기
                frames = scan_capture.gather_frames(
                    lambda: self._latest_frame, n=num_frames,
                )

                # consensus
                depth_z16 = scan_capture.consensus_depth(frames)
                color_bgr = scan_capture.consensus_color(frames)

                # 현재 raw motor positions
                raw_dict = self._cache.get_raw_motor_positions(self._arm_cfgs)
                if raw_dict is None:
                    return {"success": False, "message": "motor state 없음", "data": {}}
                arm_motor_ids = [cfg.id for cfg in self._arm_cfgs]
                raw_positions = [raw_dict[mid] for mid in arm_motor_ids]

                # 저장 디렉토리
                sdir = scan_io.session_dir(sid)
                sdir.mkdir(parents=True, exist_ok=True)
                idx = scan_io.next_scan_index(sdir)
                scan_path = sdir / f"scan_{idx:03d}.npz"

                # depth_frame의 intrinsic은 (color stream의 K). frames[0]에서 가져옴.
                f0 = frames[0]
                scan_io.save_scan(
                    scan_path,
                    color_bgr=color_bgr, depth_z16=depth_z16,
                    fx=f0.fx, fy=f0.fy, cx=f0.cx, cy=f0.cy,
                    width=f0.width, height=f0.height,
                    depth_scale=f0.depth_scale,
                    raw_motor_positions=raw_positions,
                    arm_motor_ids=arm_motor_ids,
                    num_frames=len(frames),
                )
                return {
                    "success": True,
                    "message": f"scan_{idx:03d}.npz 저장",
                    "data": {
                        "session_id": sid,
                        "scan_index": idx,
                        "path": scan_path.relative_to(scan_io.ROBOT_DIR).as_posix(),
                        "num_frames": len(frames),
                    },
                }
            except Exception as e:
                logger.exception("capture 실패")
                return {"success": False, "message": str(e), "data": {}}

    def _srv_build_mesh(self, req: dict) -> dict:
        """req.data: { session_id (필수), voxel_size?, sdf_trunc?, depth_trunc?, icp_max_dist? }"""
        try:
            data = req.get("data", {}) or {}
            sid = scan_io.validate_session_id(str(data.get("session_id", "")))
            sdir = scan_io.session_dir(sid)
            npz_paths = sorted(sdir.glob("scan_*.npz"))
            if not npz_paths:
                return {"success": False, "message": "scan 없음", "data": {}}

            scans = [scan_io.load_scan(p) for p in npz_paths]

            out_path = scan_io.MODELS_DIR / f"mesh_{sid}.ply"
            t0 = time.time()
            result = tsdf_builder.build_mesh(
                scans,
                voxel_size=float(data.get("voxel_size", tsdf_builder.DEFAULT_VOXEL_SIZE)),
                sdf_trunc=float(data.get("sdf_trunc", tsdf_builder.DEFAULT_SDF_TRUNC)),
                depth_trunc=float(data.get("depth_trunc", tsdf_builder.DEFAULT_DEPTH_TRUNC)),
                icp_max_dist=float(data.get("icp_max_dist", tsdf_builder.DEFAULT_ICP_MAX_DIST)),
                out_path=out_path,
            )
            elapsed = time.time() - t0

            return {
                "success": True,
                "message": f"mesh: {result.vertex_count} vertices, {result.triangle_count} triangles",
                "data": {
                    "session_id": sid,
                    "path": out_path.relative_to(scan_io.ROBOT_DIR).as_posix(),
                    "vertex_count": result.vertex_count,
                    "triangle_count": result.triangle_count,
                    "n_scans": result.n_scans,
                    "n_edges": result.n_edges,
                    "elapsed": elapsed,
                },
            }
        except Exception as e:
            logger.exception("build_mesh 실패")
            return {"success": False, "message": str(e), "data": {}}

    # _srv_new_session, _srv_list_sessions, _srv_list_scans, _srv_delete_scan,
    # _srv_list_meshes — 단순한 디렉토리 조작. feat/pcd-capture 참고.
```

**`_arm_cfgs` 어디서 얻나** — calibration_node가 어떻게 주입받는지 확인 후 동일 패턴. [backend/main.py](../backend/main.py)에서 motor 설정 YAML을 노드들에 주입하는 흐름.

---

## 7. Frontend 구현

### 7a. `pointCloudStore.ts` 확장

기존 `enabled / voxelSize / frame` 유지 + 추가:

```typescript
export interface ScanMeta {
  index: number;
  path: string;
  timestamp: number;
  num_frames: number;
}
export interface MeshMeta {
  session_id: string;
  path: string;        // robot 기준 상대경로 (e.g. "models/mesh_session_xxx.ply")
  vertex_count?: number;
  triangle_count?: number;
  size?: number;
  mtime?: number;
}

interface PointCloudState {
  // ... 기존 ...
  // capture
  currentSessionId: string | null;
  sessions: string[];
  scans: ScanMeta[];
  capturing: boolean;
  newSession: (sid?: string) => Promise<string | null>;
  capture: (numFrames?: number) => Promise<void>;
  selectSession: (sid: string) => Promise<void>;
  refreshSessions: () => Promise<void>;
  refreshScans: () => Promise<void>;
  deleteScan: (index: number) => Promise<void>;
  // mesh
  meshes: MeshMeta[];
  meshVisible: boolean;
  meshPath: string | null;
  meshBusy: boolean;
  buildMesh: (params?: BuildParams) => Promise<void>;
  refreshMeshes: () => Promise<void>;
  showMesh: (path: string) => void;
  hideMesh: () => void;
  setMeshVisible: (v: boolean) => void;
}

interface BuildParams {
  voxel_size?: number;
  sdf_trunc?: number;
  depth_trunc?: number;
  icp_max_dist?: number;
}
```

actions는 `bridge.callService(ServiceKey.POINTCLOUD_*, payload)` 패턴. feat/pcd-capture의 [frontend/src/store/pointCloudStore.ts](#) 패턴 참고하되 시그니처는 위.

### 7b. `PointCloudPanel.tsx` 섹션 추가

기존 "Live Stream" / "Voxel Size" / "Stats" 위에/아래 추가:

```
┌─ Live Stream ─────────────┐  (기존)
├─ Voxel Size (live) ────────┤  (기존)
├─ Stats ───────────────────┤  (기존)
├─ Session ─────────────────┤  ← 신규
│  Current: session_xxx       │
│  [새 세션]  [세션 변경 ▾]   │
├─ Capture ─────────────────┤  ← 신규
│  scans: 5                  │
│  [캡처]                    │
│  scan_001 14:32 (10f)  🗑  │
│  scan_002 14:33 (10f)  🗑  │
├─ Build Mesh ──────────────┤  ← 신규
│  voxel  [2mm] [1mm]        │
│  sdf_trunc [10mm]          │
│  depth_trunc [0.5m]        │
│  icp_max_dist [10mm]       │
│  [BUILD]                    │
├─ Meshes ──────────────────┤  ← 신규
│  ☑ visible                  │
│  ○ mesh_session_xxx        │
│    23k verts, 41k tris      │
└────────────────────────────┘
```

UI 라이브러리: 기존 `PanelShell / Section / ToggleRow`. 추가 컴포넌트 필요 시 [frontend/src/components/workspace3d/ui/](../frontend/src/components/workspace3d/ui/)에 같은 스타일로.

### 7c. `MeshLayer.tsx` 신규

```typescript
// frontend/src/components/workspace3d/3d/MeshLayer.tsx
import { useEffect, useState, useMemo } from "react";
import { useLoader } from "@react-three/fiber";
import * as THREE from "three";
import { PLYLoader } from "three/examples/jsm/loaders/PLYLoader.js";
import { usePointCloudStore } from "@/store/pointCloudStore";
import { BASE_URL } from "@/constants";

export function MeshLayer() {
  const meshVisible = usePointCloudStore((s) => s.meshVisible);
  const meshPath    = usePointCloudStore((s) => s.meshPath);

  const [geometry, setGeometry] = useState<THREE.BufferGeometry | null>(null);

  useEffect(() => {
    if (!meshPath) { setGeometry(null); return; }
    const loader = new PLYLoader();
    const url = `${BASE_URL}/robot/${meshPath}`;
    loader.load(url, (geo) => {
      geo.computeVertexNormals();
      setGeometry(geo);
    });
  }, [meshPath]);

  // Open3D(z-up, base frame) → three.js(y-up). RobotScene 다른 layer와 동일 변환.
  // 단 RobotScene가 이미 base→three 회전을 부모 group에서 적용하면 여기서 추가 변환 X.
  // PointCloudLayer.tsx의 transform과 일치시킬 것.

  if (!meshVisible || !geometry) return null;
  return (
    <mesh geometry={geometry}>
      <meshStandardMaterial vertexColors color="white" side={THREE.DoubleSide} />
    </mesh>
  );
}
```

좌표계 — [frontend/src/components/workspace3d/3d/PointCloudLayer.tsx](../frontend/src/components/workspace3d/3d/PointCloudLayer.tsx)가 base 프레임 cloud를 어떻게 그리는지 확인 후 같은 변환 사용. **feat/pcd-capture의 MeshLayer는 z-up→y-up `mesh.rotation.x = -Math.PI/2` 같은 거 박았으나 현재 RobotScene 좌표계와 일치 확인 필요.**

### 7d. `RobotScene.tsx`

```typescript
import { MeshLayer } from "./MeshLayer";
// ...
<group /* base frame parent */>
  <PointCloudLayer />
  <MeshLayer />        {/* 신규 */}
  {/* robot model, grid, etc */}
</group>
```

---

## 8. 검증 시나리오 (수동)

1. 책상 위 작은 물체 (커피잔, 박스, 손 안 들어가는 작은 인형 같은). 표면 광택/투명 피하기.
2. depth 스트림 enable.
3. 새 세션 생성.
4. 로봇 토크 OFF → 자세 잡음 → 토크 ON → 0.5s 대기 → 캡처. 자세는 위에서, 양옆에서, 살짝 비스듬에서 — 8~10자세.
5. BUILD 클릭. 응답에 `vertex_count / triangle_count / n_edges / elapsed`.
6. MeshLayer로 시각 확인:
   - 표면이 이중벽/두꺼움 없이 깔끔하면 ICP+PoseGraph가 잘 작동 (캘 floor 보완 성공)
   - 두꺼움이 보이면 voxel_size 키우거나 (예: 2→3mm) icp_max_dist 조정 (10→15mm)
   - hole이 크면 자세 추가 또는 depth_trunc 늘려보기 (D405 한계 안에서)
   - 떠다니는 fragment 보이면 `MIN_TRIANGLE_CLUSTER_SIZE` 키움
7. 결과 미흡 시 colored ICP / fragment 단계 / mesh smoothing 추가 검토 — 그 전 단계가 아님.

---

## 9. 파라미터 디폴트와 튜닝 가이드

| 파라미터 | 기본값 | 의미 | 튜닝 |
|----------|--------|------|------|
| `voxel_size` | 2mm | TSDF voxel + ICP downsample 단위 | 1mm로 더 디테일, 3mm로 노이즈 ↓. D405 noise floor 고려 2mm 무난 |
| `sdf_trunc` | 10mm (= 5×voxel) | TSDF 표면 두께. depth noise 부족하면 multi-view fusion 안 됨 | 표면이 흐리면 줄임 (5mm), hole 많으면 키움 (15mm) |
| `depth_trunc` | 0.5m | TSDF가 무시할 depth 상한 | D405 sweet spot 0.07~0.5m. 멀리 객체면 0.6 |
| `icp_max_dist` | 10mm (= 5×voxel) | ICP 매칭 거리 임계 | 캘 σ_t 8mm 고려. 너무 크면 잘못된 매칭, 너무 작으면 init 부정확 시 못 잡음 |
| `pair_uncertain_dist` (in `tsdf_builder.py`) | 0.15m | 비인접 페어 ICP 후보 거리 임계 | 너무 크면 edge 폭증 |
| `MIN_TRIANGLE_CLUSTER_SIZE` | 500 | post-process cluster 최소 크기 | 작은 디테일 잘리면 줄임 |
| `bilateral_diameter` | 5 | depth bilateral filter kernel | 노이즈 심하면 7. 너무 키우면 edge 손실 |
| `N_FRAMES_DEFAULT` | 10 | capture 시 median 입력 프레임 수 | 8 FPS 기준 1.25s. 짧게 하려면 5 |

---

## 10. 주의사항

**Open3D 좌표계 컨벤션** (§ 3d 다시):
- TSDF integrate `extrinsic` = T_cam←world (= inv(T_world←cam))
- ICP `transformation` = T_target←source
- PoseGraph `node.pose` = T_world←cam_i
- 헷갈리면 한 번 inv 잘못 박아서 mesh가 뒤집힘. 작성 후 sanity test: scan 1개일 때 (n=1) mesh가 base frame 어디에 박히는지 시각 확인.

**solver.fk와 hand_eye 일관성**:
- `solver.fk(arm_rad)`는 *URDF rad*을 입력으로 받음. raw motor → URDF rad 변환은 `JointCoordinates.motor_to_urdf` 사용 (joint_offsets 자동 적용).
- solver가 sag+link 적용한 actual ee를 돌려주는데, hand_eye.npz는 BA가 그 actual ee 기준으로 fit한 결과. **둘이 짝**. 한쪽만 쓰면 일관성 깨짐 — 둘 다 같은 라운드 캘 결과 써야.
- capture 시점 npz의 `calib_meta`에 mtime 박아두는 이유 — build 시점에 캘이 갱신됐으면 로그 경고.

**ScalableTSDFVolume 메모리**:
- voxel 1mm + 큰 scene이면 GB 단위 메모리 소비. 책상 위 작은 물체 (~30cm 박스 안) 기준 2mm면 수십 MB 수준. monitor `htop`으로 확인.

**`raw_to_urdf_rad` 구현**:
- `MotorConfig.id, MotorConfig.reverse` 필요. PointCloudNode가 main.py 부팅 시 arm_cfgs를 받아 보관. calibration_node와 같은 주입 패턴 따라가면 됨 — [backend/main.py](../backend/main.py)에서 어떻게 motor config가 로드되고 노드들에 주입되는지 확인.
- 변환 식: `(raw - 2048) / 4095 * 2π * (reverse ? -1 : 1)` → 결과에 `JointCoordinates().get_offset(motor_id)` 가산. **단 `JointCoordinates.motor_to_urdf(raw, cfg)`가 이미 다 묶어 처리** — 그거 쓰면 끝.

**분산 모드에서 카메라가 다른 머신**:
- depth_frame은 Zenoh 토픽으로 흘러옴. 이미 처리됨. capture 호출 시 추가 RPC 없음.
- 단 LAN latency가 있어서 N프레임 모으는 데 약간 더 걸릴 수 있음 (`FRAME_GATHER_TIMEOUT = 5.0`이면 충분).

**.gitignore 확인**:
```
robot/scans/
robot/models/
```
대용량 + 머신별. 캘 npz는 trace하는 것과 대조.

---

## 11. 첫 commit 범위 체크리스트

- [ ] `backend/core/transport/topic_map.py` — Service 키 7개 추가
- [ ] `backend/modules/pointcloud/__init__.py`
- [ ] `backend/modules/pointcloud/scan_io.py`
- [ ] `backend/modules/pointcloud/scan_capture.py`
- [ ] `backend/modules/pointcloud/tsdf_builder.py`
- [ ] `backend/nodes/pointcloud_node.py` — 신규 7개 서비스 핸들러 + `_arm_cfgs` 주입
- [ ] `backend/main.py` — PointCloudNode 인스턴스 생성 시 arm_cfgs 주입 (calibration_node와 동일 패턴)
- [ ] `frontend/src/constants/topics.ts` — ServiceKey 7개 추가
- [ ] `frontend/src/store/pointCloudStore.ts` — capture/mesh state + actions
- [ ] `frontend/src/components/workspace3d/panels/PointCloudPanel.tsx` — Session/Capture/Build/Meshes 섹션
- [ ] `frontend/src/components/workspace3d/3d/MeshLayer.tsx` — 신규
- [ ] `frontend/src/components/workspace3d/3d/RobotScene.tsx` — MeshLayer 마운트
- [ ] `.gitignore` — `robot/scans/`, `robot/models/` 추가
- [ ] `CLAUDE.md` — TSDF 섹션 추가 또는 § 아키텍처의 D405 파이프라인에 mesh 라인 추가

---

## 12. 미래 작업 (이번 commit 범위 아님)

- §17.2 옵션 C: `omx/motor/state/ee_pose_actual` publish — frontend live PC 시각화 어긋남 해결. TSDF 결과 검증 시 mesh와 live PC가 안 맞아 보이면 진입.
- Colored ICP — point-to-plane 부족 시.
- Fragment 단계 — 자세 100개 단위 큰 scene reconstruction.
- BA doc § 15g step 2~4 (intrinsic 재캘, J2 자세 추가) — TSDF 결과가 캘 floor에 막히면 진입. 보통 mesh 두께/이중벽이 voxel_size 줄여도 안 줄어들면 캘 floor.
