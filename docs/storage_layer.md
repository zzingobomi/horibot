# Storage Layer — 영속성 인프라

> **요약** — 영속성 필요한 데이터 (캘, scans, meshes, task_runs 등) 를 한 곳에 모으는 **인프라 layer**.
> 단일 entity 만을 위한 게 아니라 generic 토대. 캘은 Phase 1 의 첫 사용 사례.
> 다른 노드는 SQL/S3 모름. `storage_node` 가 Zenoh service gateway 로 격리.
> bridge 가 브라우저 ↔ Zenoh 사이 통로듯, storage_node 는 노드들 ↔ DB/blob store 사이 통로.
>
> 핵심 design pattern (캘 특유) — **commit ≠ activate**. 캘 계산 결과 저장(`COMMIT`)과 시스템 반영(`ACTIVATE`) 을 분리.
> 결과로 rollback 이 first-class 가 되고, capture session 중 race 가 자연 해결됨.

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
  Postgres + MinIO    ←   sqlite + 파일시스템 (1차)

      ↑↓ (SQL / S3 protocol)

[PC]
  storage_node (Zenoh gateway)
    ├─ RdbStore Protocol      ─ SqliteStore / PostgresStore / MysqlStore / MemoryRdbStore
    ├─ ObjectStore Protocol   ─ FilesystemStore / MinioStore / S3Store / MemoryObjectStore
    │
    │  [Phase 1 — 캘 service]
    ├─ service: STORAGE_GET_ACTIVE_CALIBRATION(robot_id, kind) → 활성 row 의 수치 dict
    ├─ service: STORAGE_LIST_CALIBRATIONS(robot_id, kind)      → row list (history view 용)
    ├─ service: STORAGE_COMMIT_CALIBRATION(...)                → row INSERT (is_active=false)
    ├─ service: STORAGE_ACTIVATE_CALIBRATION(row_id)           → row 활성화 + invalidation publish
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
| **scan_sessions** | mutable metadata (label/note 수정 가능), 자식 scan 들 가짐 | 사람이 label 달거나 noting. 그러나 활성 개념 없음 |

→ **캘이 가장 무거운 case**. Phase 1 에서 캘 풀면 나머지 entity 들은 더 단순한 패턴으로 추가.

→ **캘 특유 패턴 (is_active / ACTIVATE / invalidation) 을 미래 entity 에 강제하지 말 것.** scans/meshes/task_runs 에 `is_active` 컬럼이나 `STORAGE_ACTIVATE_*` service 박지 말 것. 그 entity 들의 자연스러운 패턴 따라가야.

## 4. 핵심 패턴 (캘 특유) — Commit vs Activate 분리

> 이 section 의 내용은 **캘 5종에만 적용**. scans/meshes/task_runs 는 ACTIVATE step 없음 — append-only.

캘 결과의 **저장**과 **시스템 반영**을 분리. 산업 표준의 정석 패턴.

### 흐름

```
capture 자세 1~N → COMPUTE (BA 결과 계산, σ 표시) → COMMIT (DB row INSERT, is_active=false)
                                                       ↓
                                          "캘 결과 list 에 row 추가됨" (시스템 영향 X)
                                                       ↓
                                  사용자가 결과 (σ_rot, σ_t, 자세 다양성) 보고 결정
                                                       ↓
                                              ACTIVATE 버튼 클릭
                                                       ↓
                                  대상 row is_active=true 토글, 다른 row is_active=false
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

**(a) Rollback first-class** — 옛 row 의 ACTIVATE 클릭 = rollback. 별도 backup 시스템 필요 X.

→ 현재 `.history/<ts>_pre-commit/` 백업 폴더 + `backup.py` + `CALIB_BACKUP_LIST` / `CALIB_BACKUP_RESTORE` 서비스 obsolete. row 토글 한 동작으로 통일.

**(b) Capture session 중 자동 적용 race 자연 해결**

옛 design (commit = 즉시 적용) 의 race 시나리오:

```
capture 자세 1~N → COMPUTE → COMMIT (즉시 시스템 반영)
                                   ↓
                       모터 노드 캐시 즉시 갱신 (sag/joint_offset 변경)
                                   ↓
                       자세 N+1 명령 → IK 가 새 sag 적용
                                   ↓
                       자세 1~N 의 EE_in_base 와 자세 N+1 의 EE_in_base
                       서로 다른 캘 baseline 위에서 계산
                                   ↓
                       BA 입력 inconsistent → 캘 결과 garbage
```

→ 현재 시스템에선 git push/pull 의 수동 step 이 자연 lock 역할. storage_layer 가 자동 동기화 가져오면 이 자연 lock 소실 → race window 새로 생김.

→ **commit/activate 분리가 이 race 까지 자연 해결**: COMMIT 만으로는 시스템 영향 X, ACTIVATE 는 명시적 사용자 step 이라 capture session 중에 누르지 않음. invariant 가 UI flow 차원에서 자연 박힘. 추가 lock 패턴 (freeze 등) 불필요.

**(c) "확신할 때만 COMMIT" 부담 사라짐** — 옛 design 에선 COMMIT 누르면 즉시 반영이라 신중해야 함. 분리 후엔 "일단 저장해서 list 에 보고, 다른 캘들과 비교한 후 결정", "결정 미루기" 도 가능.

### Capture race 의 진짜 위치

추가 invariant — 캘 ACTIVATE 권한은 `calibration_node` 만 (UI 통해서). 미래 다른 노드가 ACTIVATE 권한 가지려 한다면 capture race 재검토 필요.

## 5. 용어 결정사항

| 용어 | 이유 |
|---|---|
| `storage_node` | 우리 codebase 의 노드 명명 컨벤션 (`motor_node`, `camera_node` 등) 과 정합. DDD 의 "repository pattern" 은 클래스-level 추상화라 노드 이름으로 stretching 됨 |
| `RdbStore` Protocol | "MetadataStore" 가 흔한 어휘긴 하지만, 캘 수치/task_run 은 본체 데이터지 metadata 아님 → 용어 mismatch. "Relational DB store" 가 정확 |
| `ObjectStore` Protocol | S3/MinIO 의 산업 표준 어휘 ("object storage"). "BlobStore" 도 OK 지만 ObjectStore 가 cloud 어휘와 더 정합 |
| `*Store` 후미 | Protocol = `RdbStore`/`ObjectStore`, 구현체 = `SqliteStore`/`PostgresStore`/`MinioStore` 등 — Protocol 과 adapter 가 같은 suffix |
| `COMMIT` vs `ACTIVATE` | git/Kubernetes 어휘와 정합. COMMIT = DB 저장, ACTIVATE = 시스템 반영 (캘 특유) |

casing — Python PEP 8 권장 (단어별 capitalize) 따라 약어도 `RdbStore` 식. 우리 codebase 의 `ZenohSession`/`PybulletKinematics` 같은 풀스펠 PascalCase 와 정합.

## 6. 데이터 표현 — npz/파일이 아니라 column + blob

큰 그림에서 보면:
- 작은 정형 수치 → RDB column (캘 5종 다 해당)
- 큰 raw binary → ObjectStore blob (scans/meshes 만)
- 자유 형식 데이터 → JSONB (per_pose_residuals, step_results 등)

### 캘 5종 — 모두 RDB column (ObjectStore 안 씀)

| 캘 종류 | 내용 | 자연스러운 표현 |
|---|---|---|
| intrinsic | K(3x3), distCoeffs(5), image_size(2), rms | 17 float — RDB column |
| hand_eye | T_eef_cam(4x4), σ_rot, σ_t, per_pose_residuals[] | column + 별도 residual 테이블 (or JSONB) |
| joint_offset | 5 motor 의 raw offset | 5 int — RDB column |
| link_offset | link 별 axis 별 보정량 | JSONB or 별도 테이블 |
| sag_offset | J2/J3 stiffness 등 | JSONB |

→ 캘은 npz binary blob 일 본질적 이유 없음. np.savez/load key 컨벤션 (`np.load(x)['matrix']` 같은) 의존 사라짐. Pydantic / SQLAlchemy 모델로 type safe.

추가 메타 컬럼 — `is_active`, `created_at`, `operator`, `note`, σ 등 history view 와 ACTIVATE 토글을 위한 자리.

### Phase 2 의 entity 별 표현 (미리 sketch)

| Entity | RDB | ObjectStore | active 패턴? |
|---|---|---|---|
| scans | `(id, robot_id, session_id, captured_at, pose_count, blob_key, motor_positions jsonb)` | `scan_<id>.npz` (raw depth + color) | ❌ append-only |
| meshes | `(id, robot_id, session_id, built_at, scan_ids[], voxel_size, blob_key, point_count)` | `mesh_<id>.ply` | ❌ append-only |
| scan_sessions | `(id, robot_id, started_at, label, note)` | ❌ | ❌ (label 변경만 가능) |
| task_runs | `(id, name, started_at, ended_at, status, robot_ids[], step_results jsonb)` | ❌ | ❌ immutable |

→ scans/meshes 만 ObjectStore 진짜 사용. 나머지는 RDB column / JSONB.

## 7. 노드 측 패턴

### 캘 — 싱글톤 캐시 + invalidation 구독 (캘 특유 패턴)

storage_node 가 데이터 제공하고, 다른 노드는 **싱글톤 캐시 + invalidation 구독** 으로 동기화. 우리 codebase 의 `JointCoordinates` / `LinkCoordinates` / `SagCoordinates` 가 npz 1회 로드 + `_reload_caches()` 패턴 쓰는 거와 동일한 모양 — npz 자리에 Zenoh service 호출이 들어감.

```python
# 의사 코드
class CalibrationCache:  # 각 노드의 싱글톤
    def __init__(self, base_node):
        self._data = base_node.call_service(STORAGE_GET_ACTIVE_CALIBRATION, ...)  # 부팅 시 fetch
        base_node.create_subscriber(STORAGE_CALIBRATION_INVALIDATED, self._on_invalidated)

    def _on_invalidated(self, msg):
        self._data = base_node.call_service(STORAGE_GET_ACTIVE_CALIBRATION, ...)  # refetch
```

#### Invalidation 트리거 — ACTIVATE 만

- `COMMIT` (RDB INSERT, is_active=false) → invalidation 발생 X (시스템 영향 X)
- `ACTIVATE` (is_active=true 토글) → `STORAGE_CALIBRATION_INVALIDATED` publish → 노드들 refetch

### 다른 entity — ad-hoc 호출, cache 없음

scans/meshes/task_runs 는 런타임에 모든 노드가 사용하지 않음:

- scans 캡처: pointcloud_node 가 capture 시점에 `STORAGE_PUT_SCAN` 호출, 끝
- mesh build: pointcloud_node 가 build 시점에 `STORAGE_LIST_SCANS` 로 가져오고 `STORAGE_PUT_MESH` 로 저장, 끝
- task_runs: task_node 가 실행 종료 시점에 `STORAGE_INSERT_TASK_RUN` 호출, 끝

→ 캐시 패턴 / invalidation 강제 X. 사용 시점에만 호출. 더 단순한 model.

### Gateway 못 찾을 때 — bounded retry + spill fallback (캘에만 적용)

| 시나리오 | 동작 |
|---|---|
| storage_node 정상 | service 호출 → 응답 받음 → 메모리 캐시 + spill 디스크에 저장 (`~/.cache/horibot/<robot_id>/calibration_<kind>.json`) |
| storage_node 부팅 안 됨 (30초 retry) | 30초 후 spill 디스크에서 load → 노드 부팅 진행 + UI 에 "NAS unreachable, last-known cache" 경고 |
| 첫 부팅 (spill 없음) + gateway 없음 | default 캘 (identity matrix 등) 으로 부팅 + UI 강하게 "캘 안 됨" 경고 |
| **write/commit/activate** | gateway 필수 — gateway 없으면 error 명시. stale 캐시로 write 받기 시작하면 일관성 깨짐 |

부수 효과 — NAS 가 "필수 인프라" 아니라 "있으면 좋은 인프라". NAS 정비 / 재부팅 동안에도 시스템 운영 가능.

scans/meshes/task_runs 는 fallback 없음 — 사용 시점에 gateway 없으면 그냥 작업 실패 (캡처 안 됨 / mesh build 안 됨). 캐시 없는 entity 라 spill 패턴이 의미 없음.

## 8. Backend 추상화 — Strategy/Adapter 패턴

```python
# backend/modules/storage/rdb_store.py
from typing import Protocol

class RdbStore(Protocol):
    # Phase 1 — 캘
    def get_active_calibration(self, robot_id: str, kind: str) -> CalibrationRecord | None: ...
    def list_calibrations(self, robot_id: str, kind: str, limit: int) -> list[CalibrationRecord]: ...
    def insert_calibration(self, record: CalibrationRecord) -> int: ...   # is_active=false 로 INSERT
    def activate_calibration(self, row_id: int) -> None: ...              # 다른 row deactivate + 대상 activate (transaction)

    # Phase 2 — 추가 entity (signature 는 sketch, 실제 형태는 Phase 2 논의)
    # def insert_scan(self, record: ScanRecord) -> int: ...
    # def list_scans(self, robot_id: str, session_id: int) -> list[ScanRecord]: ...
    # def insert_task_run(self, record: TaskRunRecord) -> int: ...
    # def list_task_runs(self, filters: TaskRunFilter) -> list[TaskRunRecord]: ...

class ObjectStore(Protocol):
    # Phase 2 부터 진짜 사용 — Phase 1 은 캘 column 화라 ObjectStore 안 씀
    def put(self, key: str, data: bytes) -> None: ...
    def get(self, key: str) -> bytes: ...
    def delete(self, key: str) -> None: ...
    def list(self, prefix: str) -> list[str]: ...

# backend/modules/storage/adapters/sqlite_store.py
class SqliteStore:
    def __init__(self, db_path: Path): ...
```

**Factory + URI 컨벤션** (MLflow 가 정확히 이 방식):

```python
def make_rdb_store(uri: str) -> RdbStore:
    if uri.startswith("sqlite:///"): return SqliteStore(...)
    if uri.startswith("postgresql://"): return PostgresStore(uri)
    if uri == "memory://": return MemoryRdbStore()
    ...
```

```yaml
# host_pc.yaml — phase 1
storage:
  rdb_uri:    "sqlite:///~/.local/horibot/storage.db"
  object_uri: "file:///~/.local/horibot/blobs"  # phase 1 캘만 쓰면 안 쓰이지만 phase 2 대비

# host_pc.yaml — phase 3 (NAS 도입 후)
storage:
  rdb_uri:    "postgresql://horibot@nas.local:5432/horibot"
  object_uri: "s3://nas.local:9000/horibot"
```

backend 갈 때 — adapter 파일 추가 + host yaml URI 만 바꿈. 다른 노드 / storage_node service handler 코드 변경 X.

## 9. Phase / 페이스

뼈대는 미리, 적용은 점진.

### 핵심 — Phase 1 의 generic 토대가 Phase 2/3 에서 재사용

| Phase | Generic 토대 | Entity 추가 |
|---|---|---|
| **1** | `storage_node` + `RdbStore`/`ObjectStore` Protocol + Factory + Sqlite/Filesystem adapter + spill fallback + 노드 측 cache 패턴 | **캘 5종** (active + cache + invalidation 패턴 검증) |
| **2** | (변경 X, 재사용) | scans / meshes / scan_sessions / task_runs (append-only, cache 없음) |
| **3** | Postgres/MinIO adapter 추가 (URI 만 변경) | (entity 변경 X) |

Phase 1 의 generic 토대가 Phase 2/3 에서 그대로 재사용. Phase 2 는 entity 만 추가 (Protocol method, Service, Schema), generic infrastructure 손 X. Phase 3 은 entity 변경 없이 backend adapter 만 swap.

### Phase 1 (지금) — 상세

- ✅ `storage_node` 노드 + Zenoh service contract
- ✅ `RdbStore` / `ObjectStore` Protocol + factory
- ✅ adapter: `SqliteStore` / `FilesystemStore` / `MemoryRdbStore` / `MemoryObjectStore`
- ✅ 1차 entity: **캘 5종만** (intrinsic / hand_eye / joint_offset / link_offset / sag)
- ✅ Service: `STORAGE_GET_ACTIVE_CALIBRATION` / `STORAGE_LIST_CALIBRATIONS` / `STORAGE_COMMIT_CALIBRATION` / `STORAGE_ACTIVATE_CALIBRATION`
- ✅ Topic: `STORAGE_CALIBRATION_INVALIDATED`
- ✅ 노드 측 패턴: 싱글톤 캐시 + invalidation + spill fallback (캘에만)
- ✅ commit/activate 분리 UI flow (rollback first-class, capture race 해결)
- ✅ 마이그레이션: 기존 `robot/instances/*/calibration/*.npz` → SQLite import 스크립트 1회 실행 (각 row 는 import 후 is_active=true 로 시작)

**가치 — git push/pull 동기화 사라짐.** SQLite 가 PC 로컬이어도 storage_node Zenoh gateway 통해 모터 Pi / 카메라 Pi 가 동기 접근. NAS 없이도 분산 동기화 해결.

### Phase 2 — entity 확장

- scans (`scan_*.npz`) — ObjectStore 진짜 사용 시작
- meshes (`mesh_*.ply`)
- scan_sessions (label / note 컬럼 포함)
- task_runs
- 캘 row 에 observability metrics / PnP reject 기록 통합 가능

### Phase 3 — NAS backend

- `PostgresStore` / `MinioStore` adapter 추가
- host yaml URI 변경 (SQLite → Postgres, file → s3)
- 다른 노드 코드 변경 없음

## 10. 사라지는 자리들

### Phase 1 (캘) 도입 후

| 사라지는 것 | 자리 | 대체 |
|---|---|---|
| `.history/<ts>_pre-commit/` 백업 폴더 + `backup.py` | [backend/modules/calibration/backup.py](../backend/modules/calibration/backup.py) | RDB row 의 `is_active` flag + `created_at` (자연 history) |
| `CALIB_BACKUP_LIST` / `CALIB_BACKUP_RESTORE` 서비스 | calibration_node | `STORAGE_LIST_CALIBRATIONS` + `STORAGE_ACTIVATE_CALIBRATION(row_id)` (rollback first-class) |
| `commit_absolute()` 의 disk overwrite + memory reload | calibration_node | RDB INSERT 새 row → ACTIVATE → invalidation topic |
| "COMMIT 누르면 즉시 반영" 부담 | UX | COMMIT 은 list 에 row 추가만, ACTIVATE 별도 step |
| npz savez/load key 컨벤션 (캘) | 곳곳 | Pydantic 모델 (type safe) |
| git push/pull 캘 동기화 | 운영 step | invalidation topic 자동 동기화 |
| capture session 중 race 우려 (phantom freeze 패턴) | — | commit/activate 분리가 자연 해결 |

### Phase 2 (scans/meshes/task_runs) 도입 후

| 사라지는 것 | 자리 | 대체 |
|---|---|---|
| `scan_id` monotonic 파일시스템 로직 | pointcloud_node | RDB UNIQUE(robot_id, scan_id) + auto-increment |
| `robot/instances/<id>/scans/` / `meshes/` 로컬 폴더 (gitignored) | 파일시스템 | ObjectStore (`scans/<robot_id>/<session>/scan_<id>.npz` 등) |
| task 실행 history 휘발성 | 휘발성 (topic publish 만) | RDB `task_runs` 테이블 + history view |

## 11. 남은 open 문제 — 다음 논의

### Phase 1 (캘) specific

- **schema 그림** — 캘 5종의 정확한 컬럼 모양. `hand_eye.per_pose_residuals[]` 같은 nested 데이터를 JSONB vs 별도 테이블. is_active UNIQUE constraint 형태 (partial index)
- **Service contract** — service key 이름 (`STORAGE_*` prefix), request/response Pydantic 모델
- **마이그레이션 스크립트** — 기존 npz 파일들 → SQLite import. 각 kind 의 row 가 import 후 is_active=true 로 시작 (현재 시스템의 active 캘이 새 시스템의 active 가 되게)
- **storage_node 의 host yaml 등록** — `host_pc.yaml` 의 application 노드 그룹
- **calibration_node 와의 책임 경계** — calibration_node 가 capture/COMPUTE 책임, storage_node 가 COMMIT/ACTIVATE/list 책임. 서로의 service 호출 경로 명확히
- **Activate granularity** — capture session 산출물 (hand_eye + sag) 을 한 묶음으로 ACTIVATE 할지, kind 별 individual 로 ACTIVATE 할지. 한 묶음이 자연스러우면 group_id 같은 자리 필요
- **UI flow** — list 에 row 들 보여주는 패널, σ_rot/σ_t/created_at/operator/note 표시, ACTIVATE 버튼, 현재 활성 row 강조

### Phase 2 / 3 generic — 미리 박지 말 것

- scans/meshes/task_runs 의 schema/service/Protocol method — Phase 2 진입 시 논의
- NAS Postgres/MinIO adapter 의 connection pool / transaction 정책 — Phase 3 진입 시 논의
