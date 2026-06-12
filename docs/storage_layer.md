# Storage Layer — 영속성 인프라

> **요약** — 영속성 필요한 데이터 (캘, scans, meshes, task_runs 등) 를 한 곳에 모으는 **인프라 layer**.
> 단일 entity 만을 위한 게 아니라 generic 토대. 캘은 Phase 1 의 첫 사용 사례.
> 다른 노드는 SQL/S3 모름. `storage_node` 가 Zenoh service gateway 로 격리.
> bridge 가 브라우저 ↔ Zenoh 사이 통로듯, storage_node 는 노드들 ↔ DB/blob store 사이 통로.
>
> 핵심 design pattern (캘 특유) — **commit ≠ activate**. 캘 계산 결과 저장(`COMMIT`)과 시스템 반영(`ACTIVATE`) 을 분리.
> 결과로 rollback 이 first-class 가 되고, capture session 중 race 가 자연 해결됨.
>
> **데이터 모델** — 3계층 (Result / Evidence / Artifact) + Run vs Result 분리. 캘은 3 테이블 (`calibration_runs` / `calibration_results` / `calibration_captures`).

## 1. 동기

지금은 영속성 필요한 데이터들이 흩어져 있고 각자 다른 방식으로 다뤄짐:

| 데이터 | 현재 위치 | 문제 |
|---|---|---|
| 캘 결과 (`*.npz`) | git repo 안 | 분산 머신 간 git push/pull 수동 동기화 |
| scans (`scan_*.npz`) | PC 로컬, gitignored | 다른 PC 에서 접근 불가, history index 없음 |
| meshes (`mesh_*.ply`) | PC 로컬, gitignored | 동일 |
| `.history/` 캘 백업 | PC 로컬 | "지난달 σ 추이" query 불가, 폴더 단위 백업 |
| task 실행 결과 | 휘발성 (topic publish) | "어제 무슨 task 실패했나" history 없음 |

→ 영속성 데이터를 **한 source 에 모으고**, 노드들이 **runtime 에 동기화**되게, 그리고 **history 가 indexed query** 가능하게.

캘은 가장 무거운 case (active + cache + invalidation 다 가짐) 라 Phase 1 의 첫 적용 대상. 이후 entity 들은 더 단순한 패턴 (append-only) 으로 같은 인프라 위에 추가.

## 2. 큰 그림 architecture

```
[NAS — 미래] 또는 [PC 로컬 — 1차]
  Postgres + MinIO    ←   sqlite + 파일시스템 (1차) / memory (mock 모드)

      ↑↓ (SQL / S3 protocol)

[PC]
  storage_node (Zenoh gateway)
    ├─ RdbStore Protocol      ─ SqliteStore / PostgresStore / MysqlStore / MemoryRdbStore
    ├─ ObjectStore Protocol   ─ FilesystemObjectStore / MinioObjectStore / MemoryObjectStore
    │                            (universal 4 method: put/get/delete/list)
    │
    │  [Phase 1 — 캘 service]
    ├─ service: STORAGE_GET_ACTIVE_CALIBRATION(robot_id, kind) → 활성 result 의 수치
    ├─ service: STORAGE_LIST_CALIBRATIONS(robot_id, kind)      → result list (history)
    ├─ service: STORAGE_COMMIT_CALIBRATION(run + result + captures) → INSERT (is_active=false)
    ├─ service: STORAGE_ACTIVATE_CALIBRATION(result_id)        → row 활성화 + invalidation
    ├─ topic:   STORAGE_CALIBRATION_INVALIDATED(robot_id, kind)
    │
    │  [Phase 2 — entity 추가 자리]
    ├─ service: STORAGE_LIST_SCANS / STORAGE_PUT_SCAN / ...
    ├─ service: STORAGE_LIST_MESHES / STORAGE_PUT_MESH / ...
    └─ service: STORAGE_INSERT_TASK_RUN / STORAGE_LIST_TASK_RUNS / ...

      ↑↓ (Zenoh)

[모터 Pi] motor/motion           [카메라 Pi] camera          [PC] detector/task/pointcloud/calibration
  └─ 부팅 시 fetch + 메모리 캐시 + invalidation 구독 + (gateway 못 찾으면 spill fallback)  ← 캘만
  └─ scans/meshes/task_runs 는 사용 시점에 ad-hoc 호출 (cache 없음)
```

핵심 — **다른 노드는 SQL 도 S3 도 모름**. Zenoh service 호출만 함. storage_node 내부의 backend (SQLite vs Postgres, 파일시스템 vs MinIO) 변경은 다른 노드 영향 X.

### bridge 와의 비유

| | bridge_node | storage_node |
|---|---|---|
| 격리하는 외부 시스템 | 브라우저 (Zenoh 못 씀) | DB / object store (SQL/S3) |
| 안쪽 protocol | Zenoh | Zenoh |
| 바깥쪽 protocol | WebSocket + HTTP | psycopg / boto3 / sqlite3 |
| 핵심 가치 | 다른 노드는 WS 모름 | 다른 노드는 SQL 모름 |
| 위치 | PC 만 | PC 만 |

산업 표준 정합 — ROS `rosbridge_suite`, MLflow `mlflow server` 다 같은 모양 (외부 system 의존을 한 노드에 격리).

## 3. Generic 토대 vs Entity-specific 부분

storage_layer 는 *영속성 인프라* — 단일 entity (캘) 전용 아님. design boundary 가 명확:

| 자리 | Generic (모든 entity 공통) | Entity-specific |
|---|---|---|
| `storage_node` 노드 / `RdbStore`·`ObjectStore` Protocol 존재 / Factory + URI / spill fallback / Adapter (Sqlite/Postgres/MinIO 등) | ✅ Phase 1 에서 완성, 이후 재사용 | |
| Service 이름 / Schema 컬럼 / Protocol method 들 | | ✅ entity 별 다름, 점점 늘어남 |
| `is_active` / ACTIVATE / Invalidation topic | | ✅ **캘 특유**. scans/meshes/task_runs 에 강제 X |

### Entity 별 패턴 차이

| Entity | 패턴 | 이유 |
|---|---|---|
| **캘 5종** | active row + runtime cache + invalidation + ACTIVATE step | 런타임에 모든 노드가 사용, 변경 시 동기화 필요, 결과 보고 적용 결정 필요 |
| **scans/meshes** | append-only blob + immutable metadata row | 캡처/빌드 시점만 사용, 런타임 cache 없음, 한 번 만들면 변경 안 함 |
| **task_runs** | append-only record (immutable) | 실행 종료 시 INSERT, 이후 history view 용. 변경 / 활성화 개념 없음 |
| **scan_sessions** | mutable metadata (label/note), 자식 scan 들 가짐 | 사람이 label 달거나 noting. 활성 개념 없음 |

→ **캘이 가장 무거운 case**. Phase 1 에서 캘 풀면 나머지 entity 들은 더 단순한 패턴으로 추가.

→ **캘 특유 패턴 (is_active / ACTIVATE / invalidation) 을 미래 entity 에 강제하지 말 것.** scans/meshes/task_runs 에 `is_active` 컬럼이나 `STORAGE_ACTIVATE_*` service 박지 말 것.

## 4. 핵심 패턴 (캘 특유) — Commit vs Activate 분리

> 이 section 의 내용은 **캘 5종에만 적용**. scans/meshes/task_runs 는 ACTIVATE step 없음 — append-only.

캘 결과의 **저장**과 **시스템 반영**을 분리. 산업 표준의 정석 패턴.

### 흐름

```
capture 자세 1~N → COMPUTE (BA 결과 계산, σ 표시) → COMMIT (Run + Result + Captures INSERT, Result.is_active=false)
                                                       ↓
                                          "캘 결과 list 에 row 추가됨" (시스템 영향 X)
                                                       ↓
                                  사용자가 결과 (σ_rot, σ_t, 자세 다양성) 보고 결정
                                                       ↓
                                              ACTIVATE 버튼 클릭 (Result 단위)
                                                       ↓
                                  대상 Result.is_active=true 토글, 같은 (robot_id, kind) 다른 Result 들 false
                                                       ↓
                                  STORAGE_CALIBRATION_INVALIDATED publish → 노드들이 fetch
```

### 산업 표준 정합

| 시스템 | "결과 저장" | "적용" |
|---|---|---|
| Git | `commit` (로컬) | `push` / `checkout` (반영) |
| MLflow | run logging (모델 학습) | "register model" / production deploy |
| Kubernetes | image push | deployment rollout |
| Database migration | migration 파일 작성 | `migrate up` |

전부 같은 패턴. **저장과 적용을 분리** 가 production 시스템의 정석.

### 가치 — 세 가지 동시 해결

**(a) Rollback first-class** — 옛 Result 의 ACTIVATE 클릭 = rollback. 별도 backup 시스템 필요 X.

→ 현재 `.history/<ts>_pre-commit/` 백업 폴더 + `backup.py` + `CALIB_BACKUP_LIST` / `CALIB_BACKUP_RESTORE` 서비스 obsolete.

**(b) Capture session 중 자동 적용 race 자연 해결**

옛 design (commit = 즉시 적용) 의 race: capture 자세 1~N 진행 중 COMPUTE → COMMIT 누르면 즉시 모터 노드 캐시 갱신 → 자세 N+1 명령 IK 가 새 sag/joint_offset 적용 → 자세 1~N 과 자세 N+1 의 EE_in_base 가 다른 캘 baseline 위에서 계산 → BA 입력 inconsistent → 캘 결과 garbage.

현재 시스템에선 git push/pull 의 수동 step 이 자연 lock. storage_layer 가 자동 동기화 가져오면 이 자연 lock 소실 → race window 새로 생김.

→ **commit/activate 분리가 이 race 까지 자연 해결**: COMMIT 만으로는 시스템 영향 X. ACTIVATE 는 명시적 사용자 step 이라 capture session 중에 누르지 않음. invariant 가 UI flow 차원에서 자연 박힘. 추가 lock 패턴 (freeze 등) 불필요.

**(c) "확신할 때만 COMMIT" 부담 사라짐** — "일단 저장해서 list 에 보고, 비교한 후 결정", "결정 미루기" 도 가능.

### Capture race 의 진짜 위치

추가 invariant — 캘 ACTIVATE 권한은 `calibration_node` 만 (UI 통해서). 미래 다른 노드가 ACTIVATE 권한 가지려 한다면 capture race 재검토 필요.

## 5. 용어 결정사항

| 용어 | 이유 |
|---|---|
| `storage_node` | 우리 codebase 의 노드 명명 컨벤션 (`motor_node`, `camera_node` 등) 과 정합. DDD 의 "repository pattern" 은 클래스-level 추상화라 노드 이름으로 stretching 됨 |
| `RdbStore` Protocol | "MetadataStore" 가 흔한 어휘긴 하지만, 캘 수치/task_run 은 본체 데이터지 metadata 아님 → 용어 mismatch. "Relational DB store" 가 정확 |
| `ObjectStore` Protocol | S3/MinIO 의 산업 표준 어휘 ("object storage"). cloud 어휘와 정합 |
| `*Store` 후미 | Protocol = `RdbStore`/`ObjectStore`, 구현체 = `SqliteStore`/`PostgresStore`/`FilesystemObjectStore` 등 |
| `COMMIT` vs `ACTIVATE` | git/Kubernetes 어휘와 정합. COMMIT = DB 저장, ACTIVATE = 시스템 반영 (캘 특유) |
| `Run` / `Result` / `Captures` | "한 번의 실행" / "그 산출물" / "입력 evidence" — DB 도메인 모델 어휘 |

casing — Python PEP 8 권장 (단어별 capitalize). `RdbStore`/`FilesystemObjectStore` 식. 우리 codebase 의 `ZenohSession`/`PybulletKinematics` 풀스펠 PascalCase 와 정합.

## 6. 데이터 모델 — 3계층 + Run vs Result 분리

### 3계층 분류

| 계층 | 내용 | 저장소 |
|---|---|---|
| **Result** | 시스템에 적용되는 최종값 | RDB |
| **Evidence** | 결과 만든 입력/통계 — 재현 가능 최소 정보 (수 KB ~ 수십 KB) | RDB |
| **Artifact** | 대용량 원본 (이미지/PLY/mesh) | ObjectStore |

새 entity 추가 시 "이 데이터는 어느 계층?" 만 물어봐도 자리 결정. universal design language.

### 캘 5종의 3계층 매핑

| 캘 | Result | Evidence | Artifact |
|---|---|---|---|
| intrinsic | K, distCoeffs, image_size, rms | per-pose ChArUco corners + residuals | (Phase 2) capture image — Phase 1 검증 후 도입 |
| hand_eye | T_eef_cam, σ_rot, σ_t | per-pose (joint_angles, board_in_cam, residual, IRLS weight) | (Phase 2) capture image — Phase 1 검증 후 도입 |
| joint_offset | 5 motor raw offsets | (같은 Run 의 captures 공유) | — |
| link_offset | link 별 보정량 | 동일 | — |
| sag_offset | J2/J3 stiffness | 동일 | — |

→ **Phase 1 = Result + Evidence 만 RDB**. Artifact (raw image) 는 **Phase 2 도입 확정** (사용자 결정 2026-06-12): Phase 1 의 storage 인프라 + 캘 commit/activate flow 가 검증된 후 진행. 가치는 캘 실패 사후 분석 + 새 알고리즘 재계산 + 재현성.

### Run vs Result 분리

"한 번의 캘 실행" 과 "그 산출물 중 시스템 적용분" 의 lifecycle 이 다름:

| 차원 | Run | Result |
|---|---|---|
| Lifecycle | 한 번 생기면 immutable | `is_active` 자주 토글 |
| Cardinality | 1번 캘 실행 | 한 Run 이 여러 Result 만들 수 있음 (예: 확장 BA → hand_eye + sag 동시 산출) |
| 의미 | "언제 / 누가 / 어떤 알고리즘으로" | "그 산출물 중 시스템 적용분" |

→ 별도 테이블이 정석. 한 row 에 묶으면 Run metadata 가 Result 별로 중복되고 lifecycle 어색해짐.

### 3 테이블 schema

```sql
-- 한 번의 캘 실행 이력 (immutable)
calibration_runs (
  id                INTEGER PRIMARY KEY,
  robot_id          TEXT NOT NULL,
  started_at        TIMESTAMP NOT NULL,
  ended_at          TIMESTAMP,
  operator          TEXT,
  note              TEXT,
  algorithm         TEXT NOT NULL,      -- 'extended_ba_irls' 등
  algorithm_params  JSON,               -- BA 하이퍼파라미터
  status            TEXT NOT NULL       -- 'success' / 'failed'
)

-- Run 의 산출물, kind 별 row, is_active 토글 자리
calibration_results (
  id            INTEGER PRIMARY KEY,
  run_id        INTEGER NOT NULL REFERENCES calibration_runs(id),
  robot_id      TEXT NOT NULL,          -- denormalized for query
  kind          TEXT NOT NULL,          -- 'intrinsic' / 'hand_eye' / 'joint_offset' / 'link_offset' / 'sag'
  created_at    TIMESTAMP NOT NULL,
  is_active     BOOLEAN NOT NULL DEFAULT FALSE,
  sigma_rot     REAL,                   -- nullable (joint_offset 등 σ 없음)
  sigma_t       REAL,
  result_data   JSON NOT NULL           -- T_eef_cam, K, offsets 등 kind 별 실제 수치
)
-- UNIQUE INDEX (robot_id, kind) WHERE is_active=true   ← per-kind active 1개만

-- Evidence — per-pose 자세 정보 (BA 입력 + 출력 residual)
calibration_captures (
  id               INTEGER PRIMARY KEY,
  run_id           INTEGER NOT NULL REFERENCES calibration_runs(id),
  pose_index       INTEGER NOT NULL,
  joint_angles     JSON NOT NULL,
  board_in_cam     JSON,                -- ChArUco 검출 결과 (4x4)
  residual_rot     REAL,
  residual_trans   REAL,
  weight           REAL                 -- IRLS Huber weight
)
```

자주 쓰는 쿼리:

| 의도 | SQL |
|---|---|
| 현재 활성 hand_eye | `SELECT * FROM calibration_results WHERE robot_id=? AND kind='hand_eye' AND is_active=true` |
| 한 BA Run 의 모든 산출물 | `SELECT * FROM calibration_results WHERE run_id=?` (hand_eye + sag 같이) |
| Rollback | `UPDATE calibration_results SET is_active=true WHERE id=?` (+ 같은 kind 다른 row 들 false 토글 — transaction) |
| 캘 history view | `SELECT * FROM calibration_results JOIN calibration_runs ORDER BY created_at DESC` |
| IRLS weight 낮은 자세 | `SELECT * FROM calibration_captures WHERE run_id=? ORDER BY weight ASC` |
| 특정 algorithm 사용 history | `SELECT * FROM calibration_runs WHERE algorithm=?` |

### Phase 2 의 entity 별 3계층 매핑 (미리 sketch)

| Entity | Result | Evidence | Artifact |
|---|---|---|---|
| scans | scan metadata (id, pose_count, motor_positions) | — | `scan_*.npz` (raw depth + color blob) |
| meshes | mesh metadata (point_count, voxel_size, source scan_ids) | TSDF/ICP 통계 | `mesh_*.ply` |
| task_runs | task 실행 결과 record | step_results | (미래) video log 등 |
| scan_sessions | session metadata (label, note) | — | — |

scans/meshes 만 진짜 ObjectStore 사용. nature 가 Artifact 큰 binary 라.

## 7. 노드 측 패턴

### 캘 — 싱글톤 캐시 + invalidation 구독 (캘 특유)

storage_node 가 데이터 제공하고, 다른 노드는 **싱글톤 캐시 + invalidation 구독** 으로 동기화. 우리 codebase 의 `JointCoordinates` / `LinkCoordinates` / `SagCoordinates` 가 npz 1회 로드 + `_reload_caches()` 패턴 쓰는 거와 동일한 모양 — npz 자리에 Zenoh service 호출이 들어감.

```python
# 의사 코드
class CalibrationCache:  # 각 노드의 싱글톤
    def __init__(self, base_node):
        self._data = base_node.call_service(STORAGE_GET_ACTIVE_CALIBRATION, ...)
        base_node.create_subscriber(STORAGE_CALIBRATION_INVALIDATED, self._on_invalidated)

    def _on_invalidated(self, msg):
        self._data = base_node.call_service(STORAGE_GET_ACTIVE_CALIBRATION, ...)
```

#### Invalidation 트리거 — ACTIVATE 만

- `COMMIT` (Run + Result + Captures INSERT, Result.is_active=false) → invalidation 발생 X
- `ACTIVATE` (Result.is_active=true 토글) → `STORAGE_CALIBRATION_INVALIDATED` publish → 노드들 refetch

### 다른 entity — ad-hoc 호출, cache 없음

scans/meshes/task_runs 는 런타임에 모든 노드가 사용하지 않음. 사용 시점에만 호출 (캡처/빌드/실행 종료).

→ 캐시 패턴 / invalidation 강제 X. 더 단순.

### Gateway 못 찾을 때 — bounded retry + spill fallback (캘에만)

| 시나리오 | 동작 |
|---|---|
| storage_node 정상 | service 호출 → 응답 받음 → 메모리 캐시 + spill 디스크에 저장 (`~/.cache/horibot/<robot_id>/calibration_<kind>.json`) |
| storage_node 부팅 안 됨 (30초 retry) | 30초 후 spill 디스크에서 load → 노드 부팅 진행 + UI 에 "NAS unreachable, last-known cache" 경고 |
| 첫 부팅 (spill 없음) + gateway 없음 | default 캘 (identity matrix 등) 으로 부팅 + UI 강하게 "캘 안 됨" 경고 |
| **write/commit/activate** | gateway 필수 — 없으면 error. stale 캐시로 write 받기 시작하면 일관성 깨짐 |

부수 효과 — NAS 가 "필수 인프라" 아니라 "있으면 좋은 인프라". NAS 정비 / 재부팅 동안에도 시스템 운영 가능.

scans/meshes/task_runs 는 fallback 없음 — 캐시 없는 entity 라 spill 패턴이 의미 없음.

## 8. Backend 추상화 — Strategy/Adapter 패턴

```python
# backend/modules/storage/rdb_store.py
from typing import Protocol

class RdbStore(Protocol):
    # Phase 1 — 캘 (3 테이블)
    def get_active_result(self, robot_id: str, kind: str) -> CalibrationResultRecord | None: ...
    def list_results(self, robot_id: str, kind: str, limit: int) -> list[CalibrationResultRecord]: ...
    def insert_run(self, run: CalibrationRunRecord) -> int: ...
    def insert_result(self, result: CalibrationResultRecord) -> int: ...   # is_active=false 로 INSERT
    def insert_captures(self, captures: list[CalibrationCaptureRecord]) -> None: ...
    def activate_result(self, result_id: int) -> None: ...                 # 같은 (robot_id, kind) 다른 row deactivate + 대상 activate (transaction)
    def get_run(self, run_id: int) -> CalibrationRunRecord | None: ...
    def list_captures(self, run_id: int) -> list[CalibrationCaptureRecord]: ...

    # Phase 2 — 추가 entity (signature sketch, 실제는 Phase 2 진입 시)
    # def insert_scan(self, record: ScanRecord) -> int: ...
    # def list_scans(self, robot_id: str, session_id: int) -> list[ScanRecord]: ...
    # def insert_task_run(self, record: TaskRunRecord) -> int: ...

# backend/modules/storage/object_store.py
class ObjectStore(Protocol):
    """작고 보수적인 universal interface. Phase 2 진입 시 streaming/multipart 등 *추가* — 기존 method 변경 X."""
    def put(self, key: str, data: bytes) -> None: ...
    def get(self, key: str) -> bytes: ...
    def delete(self, key: str) -> None: ...
    def list(self, prefix: str) -> list[str]: ...

# backend/modules/storage/adapters/
class SqliteStore:              ...  # RdbStore Phase 1 — host_dev/host_pc 실 사용
class MemoryRdbStore:           ...  # RdbStore Phase 1 — host_mock backend + 테스트 base
class FilesystemObjectStore:    ...  # ObjectStore Phase 1 — Phase 1 사용 X, Phase 2 entity 진짜 사용
class MemoryObjectStore:        ...  # ObjectStore Phase 1 — host_mock backend + 테스트

# Phase 3 추가
class PostgresStore:            ...  # RdbStore Phase 3
class MinioObjectStore:         ...  # ObjectStore Phase 3 (S3 호환)
```

### ObjectStore 인터페이스 — 작고 보수적

- ❌ Phase 1 에 streaming / multipart upload / checksum / metadata method 미리 박지 말 것
- ❌ entity-specific method (`put_scan_with_pose_metadata` 등) 미리 박지 말 것
- ✅ universal 4 method 만 — S3/MinIO/GCS/Azure/local fs 다 fit
- Phase 2 진입 시 실제 use case 보고 method **추가** (additive — 기존 method 변경 X)

이 인터페이스가 universal 이라 Phase 1 에 만들어도 wrong 위험 거의 없음. Phase 2 에서 streaming 같은 게 필요해지면 *추가*, 기존 4 method 깨지지 않음.

### Factory + URI 컨벤션 (MLflow 식)

```python
def make_rdb_store(uri: str) -> RdbStore:
    if uri == "memory://":              return MemoryRdbStore()
    if uri.startswith("sqlite:///"):    return SqliteStore(...)
    if uri.startswith("postgresql://"): return PostgresStore(uri)
    raise ValueError(...)

def make_object_store(uri: str) -> ObjectStore:
    if uri == "memory://":              return MemoryObjectStore()
    if uri.startswith("file:///"):      return FilesystemObjectStore(...)
    if uri.startswith("s3://"):         return MinioObjectStore(uri)
    raise ValueError(...)
```

### host yaml 의 storage URI

```yaml
# host_mock.yaml — Memory backend (매번 fresh, 영속화 X)
storage:
  rdb_uri:    "memory://"
  object_uri: "memory://"

# host_dev.yaml / host_pc.yaml — Phase 1
storage:
  rdb_uri:    "sqlite:///~/.local/horibot/storage.db"
  object_uri: "file:///~/.local/horibot/blobs"

# host_pc.yaml — Phase 3 (NAS 도입 후)
storage:
  rdb_uri:    "postgresql://horibot@nas.local:5432/horibot"
  object_uri: "s3://nas.local:9000/horibot"
```

backend 갈 때 — adapter 파일 추가 + host yaml URI 만 바꿈. 다른 노드 / storage_node service handler 코드 변경 X.

## 9. Phase / 페이스

### 핵심 — Phase 1 의 generic 토대가 Phase 2/3 에서 재사용

| Phase | Generic 토대 | Entity 추가 |
|---|---|---|
| **1** | `storage_node` + `RdbStore`/`ObjectStore` Protocol + Factory + Sqlite/Filesystem/Memory adapter + spill fallback + 노드 측 cache 패턴 | **캘 5종** (3 테이블: runs/results/captures) |
| **2** | (변경 X, 재사용. ObjectStore 진짜 사용 시작) | scans / meshes / scan_sessions / task_runs (append-only, cache 없음) |
| **3** | Postgres/MinIO adapter 추가 (URI 만 변경) | (entity 변경 X) |

Phase 1 의 generic 토대가 Phase 2/3 에서 그대로 재사용. Phase 2 는 entity 만 추가, Phase 3 은 backend swap.

### Phase 1 (지금) — 상세

- ✅ `storage_node` 노드 + Zenoh service contract
- ✅ `RdbStore` / `ObjectStore` Protocol + factory + URI 분기
- ✅ Phase 1 adapter (4개):
  - `SqliteStore` (host_dev/host_pc 실 사용)
  - `MemoryRdbStore` (host_mock backend, 테스트 base)
  - `FilesystemObjectStore` (Phase 1 사용처 없지만 Phase 2 entity 진짜 사용 위해)
  - `MemoryObjectStore` (host_mock backend)
- ✅ 1차 entity: **캘 5종만** — 3 테이블 (`calibration_runs` / `calibration_results` / `calibration_captures`)
- ✅ Service: `STORAGE_GET_ACTIVE_CALIBRATION` / `STORAGE_LIST_CALIBRATIONS` / `STORAGE_COMMIT_CALIBRATION` / `STORAGE_ACTIVATE_CALIBRATION`
- ✅ Topic: `STORAGE_CALIBRATION_INVALIDATED`
- ✅ 노드 측 패턴: 싱글톤 캐시 + invalidation + spill fallback (캘에만)
- ✅ commit/activate 분리 UI flow (rollback first-class, capture race 해결)
- ✅ ObjectStore 인터페이스 작고 보수적 (4 method)
- ✅ host yaml storage URI 박기 — host_mock=`memory://`, host_dev/host_pc=`sqlite:///` + `file:///`
- ✅ 마이그레이션: 기존 `robot/instances/*/calibration/*.npz` → 3 테이블 import 스크립트 1회 실행. captures 정보 없으면 빈 captures + placeholder run 으로. 각 result 는 import 후 is_active=true.

**가치 — git push/pull 동기화 사라짐.** SQLite 가 PC 로컬이어도 storage_node Zenoh gateway 통해 모터 Pi / 카메라 Pi 가 동기 접근. NAS 없이도 분산 동기화 해결.

### Phase 2 — entity 확장

- scans (`scan_*.npz`) — ObjectStore 진짜 사용 시작
- meshes (`mesh_*.ply`)
- scan_sessions (label / note)
- task_runs
- 캘 row 에 observability metrics / PnP reject 기록 통합 가능
- **캘 raw image 보존 (Artifact 계층) 도입** — Phase 1 검증 후 진행. `calibration_captures.image_blob_key` 컬럼 추가 + ObjectStore key 컨벤션 (`calibration_captures/<run_id>/<pose_index>.jpg`). 마이그레이션 — 옛 row 들은 image_blob_key=null

### Phase 3 — NAS backend

- `PostgresStore` / `MinioObjectStore` adapter 추가
- host yaml URI 변경 (SQLite → Postgres, file → s3)
- 다른 노드 코드 변경 없음

## 10. 사라지는 자리들

### Phase 1 (캘) 도입 후

| 사라지는 것 | 자리 | 대체 |
|---|---|---|
| `.history/<ts>_pre-commit/` 백업 폴더 + `backup.py` | [backend/modules/calibration/backup.py](../backend/modules/calibration/backup.py) | `calibration_results.is_active` flag + `created_at` (자연 history) |
| `CALIB_BACKUP_LIST` / `CALIB_BACKUP_RESTORE` 서비스 | calibration_node | `STORAGE_LIST_CALIBRATIONS` + `STORAGE_ACTIVATE_CALIBRATION(result_id)` |
| `commit_absolute()` 의 disk overwrite + memory reload | calibration_node | INSERT 새 row (Run + Result + Captures) → ACTIVATE → invalidation |
| "COMMIT 누르면 즉시 반영" 부담 | UX | COMMIT 은 list 에 row 추가만, ACTIVATE 별도 step |
| npz savez/load key 컨벤션 (캘) | 곳곳 | Pydantic 모델 (type safe) |
| git push/pull 캘 동기화 | 운영 step | invalidation topic 자동 |
| capture session 중 race 우려 (phantom freeze 패턴) | — | commit/activate 분리가 자연 해결 |
| BA 입력 자세 정보 휘발성 (npz 안 묻혀 있던 자리) | npz | `calibration_captures` 테이블 (per-pose row, IRLS weight/residual 포함) |

### Phase 2 (scans/meshes/task_runs) 도입 후

| 사라지는 것 | 자리 | 대체 |
|---|---|---|
| `scan_id` monotonic 파일시스템 로직 | pointcloud_node | RDB UNIQUE(robot_id, scan_id) + auto-increment |
| `robot/instances/<id>/scans/` / `meshes/` 로컬 폴더 (gitignored) | 파일시스템 | ObjectStore (`scans/<robot_id>/<session>/scan_<id>.npz` 등) |
| task 실행 history 휘발성 | topic publish 만 | RDB `task_runs` 테이블 + history view |

## 11. 남은 open 문제 — 다음 논의

### Phase 1 (캘) specific

- **Service contract** — service key 이름 (`STORAGE_*` prefix 통일), request/response Pydantic 모델 정의
- **Pydantic 모델** — `CalibrationRunRecord` / `CalibrationResultRecord` / `CalibrationCaptureRecord` field 정확히
- **마이그레이션 스크립트** — 기존 npz → 3 테이블 import. captures 정보 없는 옛 npz 는 빈 captures + placeholder run 으로
- **storage_node 의 host yaml 등록** — `host_dev.yaml` / `host_pc.yaml` / `host_mock.yaml` 의 application 노드 그룹
- **calibration_node 와의 책임 경계** — calibration_node 가 capture/COMPUTE 책임, storage_node 가 COMMIT/ACTIVATE/list 책임. 서로의 service 호출 경로 명확히
- **Activate granularity** — Run 단위로 hand_eye + sag 한 묶음 ACTIVATE vs Result 별 individual. Phase 1 = individual 부터, group activate 는 미래 (run_id 보존되어 있으니 미래 확장 자연)
- **UI flow** — list 패널, ACTIVATE 버튼, 현재 활성 강조, Run 별 grouping view, captures detail drill-down

### Phase 2 / 3 generic — 미리 박지 말 것

- scans/meshes/task_runs 의 schema/service/Protocol method — Phase 2 진입 시 논의
- NAS Postgres/MinIO adapter 의 connection pool / transaction 정책 — Phase 3 진입 시 논의
