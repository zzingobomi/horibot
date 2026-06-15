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

## 7. 라이프사이클 + ownership layer

### Layer 분리 — Storage 의존 vs Calibration 의존

서로 다른 자리:

| Layer | 의존 | 누가 |
|---|---|---|
| **Infrastructure** | Storage Node | 모든 노드 (간접) |
| **Domain Service** | Storage gateway 책임 (캘 5종 fetch/commit/activate) | `calibration_node` 하나만 |
| **Consumer** | Calibration 데이터 (intrinsic / link / sag / joint / hand_eye) | `Coordinates` / `PybulletKinematics` / `Detector` / `Motor` 등 |

핵심 원칙 (Stage 2 design 결정):

1. **Calibration 소비자는 Storage 모름** — 코드에 `storage.get_*` 호출 0.
2. **CalibrationService (= calibration_node) 만 Storage 앎** — 한 자리 gateway.
3. **Calibration 은 push / DI 방식 전달** — calibration_node 가 소비자에 `set_offsets(...)` 또는 `apply_link_offsets(...)` 호출.
4. **PybulletKinematics 는 offset 주입 후 1회 초기화** — `kin.apply_link_offsets(offsets)` → `kin.initialize()` 패턴. URDF patch 자리.
5. **런타임 calibration reload 는 범위 밖** — 캘 자주 변경되는 데이터 X. "캘 수행 → 새 calibration → 재시작" 이 정상 운영.

### 왜 이게 중요한가 — cascading lazy 의 진짜 원인

본 design 잡기 전 패턴:
```
Coordinates lazy → 또 storage 호출 발견 → LinkCoordinates lazy → 또 발견 → PybulletKinematics lazy → ...
```

이건 *증상 치료*. 진짜 root cause = **소비자 가 storage 직접 호출** = layer 위반. owner 한 자리로 좁히면 cascade 자체가 사라짐.

### Assumption

- **Storage Node 는 필수 인프라.** Calibration 소비자가 Ready 되려면 calibration_node 의 push 받아야 → 결과적으로 Storage 필요.
- 다른 노드는 calibration 가 준비될 때까지 **대기** (state = WAITING_CALIBRATION) — fail X.
- **부팅 순서는 강제하지 않음.** Motor Pi 가 PC 보다 먼저 켜져도 OK.
- Storage 가 나중에 시작되어도 calibration_node 가 자동 연결, 다른 노드에 push.
- *cache-first 가 아니라 SSOT-first*. cache 는 hot path 성능용 in-memory copy.

본 가정의 진짜 의미 — calibration 데이터는 *git pull 지옥 없애기* 목적으로 중앙화한 운영 데이터. PC 꺼져 있을 때 Pi 단독 운영 요구사항 없음 (캘 없으면 Motor 가 raw↔rad / IK 못함 = 의미 있는 동작 X). 분산 fallback / version / conflict 자리 다 사라짐.

### 흐름

```
calibration_node 부팅
  ↓ Storage 연결 시도 (retry loop, 1초 간격)
  ↓ 발견
  ↓ 5종 STORAGE_GET_ACTIVE_CALIBRATION
  ↓ Coordinates / PybulletKinematics / Detector 등 소비자에 push
  ↓ topic 발행 (frontend / 다른 노드 구독)
  ↓
  state = READY

다른 노드 (motor / motion / detector / pointcloud 등)
  ↓ state = WAITING_CALIBRATION
  ↓ calibration_node 의 push 받음
  ↓ 자기 의존 init (PybulletKinematics URDF patch 등)
  ↓
  state = READY
```

분산 시나리오:
```
09:00 Motor Pi 부팅 → WAITING (PC 안 떠 있음)
09:05 PC 부팅 → storage_node 등장
09:05:01 Motor Pi 자동 연결 → 캘 load → Ready
```

부팅 순서 무관 — Motor Pi 가 PC 보다 먼저 켜져도 *대기* 만. 운영 재부팅 / 크래시 / 네트워크 지연 다 회복 가능.

### 첫 부팅 robot — storage 에 active 없음

so101 처럼 캘 한 번도 안 한 robot — storage 응답이 `found=false` (timeout / unreachable 와 다른 정상 상태).

- Coordinates 의 메모리 cache = empty
- default 캘 (identity matrix / joint offset 0) 로 ready
- UI 강하게 "캘 안 됨, 캘 먼저 실행" 경고

### Invalidation 구독 — runtime data 변경 알림

ACTIVATE 마다 storage_node 가 `STORAGE_CALIBRATION_INVALIDATED` publish. 구독한 노드 (Coordinates 등) 가 storage 다시 호출 → 메모리 cache refresh.

```python
# 의사 코드
class CalibrationCache:  # 각 노드의 싱글톤
    def __init__(self, transport):
        self._data = self._fetch_with_retry()
        transport.subscribe_topic(
            STORAGE_CALIBRATION_INVALIDATED,
            CalibrationInvalidated,
            self._on_invalidated,
        )

    def _on_invalidated(self, msg):
        if msg.robot_id == self.robot_id and msg.kind == self.kind:
            self._data = self._fetch_with_retry()
```

cache 의 의미 — *runtime hot path 의 성능* (매 IK 호출마다 SQL X). authoritative 데이터는 storage. cache = SSOT 의 in-memory copy, last-known 보존 X.

### 다른 entity — ad-hoc 호출, cache 없음

scans/meshes/task_runs 는 런타임에 모든 노드가 사용하지 않음. 사용 시점에만 호출 (캡처/빌드/실행 종료). cache 패턴 / invalidation 강제 X.

### 빠진 자리들 — cache-first / fallback / version

이전 design (spill_cache / 30초 retry / legacy npz fallback / version 비교) 는 모두 "Storage 없어도 단독 운영" 가정의 잔여물. 본 모델에선:

- ❌ spill cache (`~/.cache/horibot/...`) — Storage 필수라 의미 없음
- ❌ legacy npz fallback — Stage 3 마이그레이션 1회면 storage 가 source
- ❌ version 컬럼 / conflict resolution — SSOT 라 충돌 자리 자체 없음
- ❌ "storage 안 떠도 노드 부팅 진행" — Storage 필수 = 대기

write/commit/activate 도 Storage 필수 (본래 design 유지). stale 캐시로 write 받기 시작하면 일관성 깨지는 자리는 본 모델에 아예 없음 — cache 가 last-known 아니라 SSOT-mirror 라.

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
| **1** | `storage_node` + `RdbStore`/`ObjectStore` Protocol + Factory + Sqlite/Filesystem/Memory adapter + 노드 측 cache 패턴 (서비스 대기 + retry) | **캘 5종** (3 테이블: runs/results/captures) |
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
- ✅ 노드 측 패턴 (§7): Storage 필수 가정 + 서비스 대기 (retry loop) + 싱글톤 cache + invalidation 구독. spill / version / fallback 없음.
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
| `robot/<robot_type>/urdf/.patched/` 디렉토리 + `write_patched_urdf` / `patched_urdf_path` 함수 | [backend/core/coords/urdf_patcher.py](../backend/core/coords/urdf_patcher.py) | `patch_urdf_text` (in-memory string) + `tempfile.mkstemp` (PyBullet 의 `loadURDF` path-only 한계 우회용 1회성). 상세 §13 |

### Phase 2 (scans/meshes/task_runs) 도입 후

| 사라지는 것 | 자리 | 대체 |
|---|---|---|
| `scan_id` monotonic 파일시스템 로직 | pointcloud_node | RDB UNIQUE(robot_id, scan_id) + auto-increment |
| `robot/instances/<id>/scans/` / `meshes/` 로컬 폴더 (gitignored) | 파일시스템 | ObjectStore (`scans/<robot_id>/<session>/scan_<id>.npz` 등) |
| task 실행 history 휘발성 | topic publish 만 | RDB `task_runs` 테이블 + history view |

## 11. 구현 진행 상태 (2026-06-15 session)

### Stage 1 — storage 인프라 ✅ commit `62232a9`

- `modules/storage/`:
  - `models.py` 삭제 (캘 record 가 `modules/calibration/persistence_models.py` 로 이동, ownership layer 분리)
  - `transport.py` — `StorageTransport` (generic, entity 어휘 0) — typed Zenoh service call envelope + topic subscribe helper
  - `rdb/store.py` (Protocol), `rdb/adapters/{sqlite, memory}.py`
  - `object_store/store.py` (Protocol, 4 method), `object_store/adapters/{filesystem, memory}.py`
  - `factory.py` (URI 분기 — MLflow 식), `registry.py` (singleton)
- `modules/calibration/persistence_models.py` — DB row shape 의 discriminated union (`CalibrationResultRecord = Annotated[Union[HandEyeResultRecord | ...], discriminator="kind"]`). kind ↔ result_data shape invariant 를 Pydantic 차원 강제.
- `modules/calibration/result_models.py` — 계산 결과 shape (`HandEyeResultData` / `IntrinsicResultData` / ...).
- `nodes/application/storage_node.py` — ApplicationNode, Zenoh service handler 4 + invalidation topic publish.
- `topic_map.py` / `node_registry.py` / `api_contract.py` / host yaml (`dev` / `mock` / `pc`) 등록.

### Stage 2 — calibration_node 통합 ⏳ (현재 uncommitted, 사용자가 commit 예정)

ownership layer 정리 (docs §7):
```
Storage              ← SQLite (~/.local/horibot/storage.db)
   ↓
Calibration Node     ← 부팅 시 fetch + 소비자 push, write path 책임
   ↓ push
JointCoordinates / LinkCoordinates / SagCoordinates / PybulletKinematics / CalibrationCache
```

완료된 자리:
- **Coordinates 3종** (`joint` / `link` / `sag`) — `set_offsets(robot_id, offsets)` DI method. storage 호출 0. lazy 패턴 X (소유자 = calibration_node).
- **`PybulletKinematics`** — `__init__` 에서 URDF load 안 함. `apply_link_offsets(offsets)` + `initialize()` 분리. hot path 에 `_require_initialized()` raise.
- **`SagCorrectedKinematics`** — `__init__` 에서 cache build X (empty). `reload_calibration()` public method — calibration_node 가 호출.
- **`CalibrationCache`** (`modules/calibration/calibration_cache.py`) — intrinsic + hand_eye 공유 자료실. **ready Event** 보유 (`wait_ready` / `signal_ready` / `is_ready`) — atomic snapshot 보장.
- **`calibration_node._setup_runtime_calibration`** — background thread, robot 별 `_push_calibration(rid)` 호출. **atomic snapshot** — 5종 다 fetch 후 push (partial state 차단).
- **`_srv_intrinsic_save` / `_srv_handeye_commit`** — storage commit + activate + `_push_calibration` 재호출. SSOT.
- **`StorageTransport` + `CalibrationStorageClient`** (`modules/calibration/storage_client.py`) — entity 별 client (Phase 2 entity 도 같은 transport 위에 자기 client).
- **bridge router / task_node / tsdf_builder / detector_node** — `load_calibration` 호출 → `CalibrationCache().get(rid)` swap.

### Stage 2 남은 자리 (Stage 2d cleanup)

본 commit 후 다음 자리에서 이어서:

- **`.history/` + `backup.py` 제거** — `_srv_handeye_commit` 에서 `calib_backup.snapshot(...)` 호출 이미 제거됨. 단 `backup.py` 파일 + `CALIB_BACKUP_LIST` / `CALIB_BACKUP_RESTORE` 서비스 자체는 살아있음 (frontend Rollback 탭 호환 위해). Stage 4 frontend swap 후 제거.
- **`loader.py::load_calibration_from_npz`** — 옛 npz 직접 load. Stage 3 마이그레이션 스크립트 전용. 부팅 path 에서 호출 X.
- **`hand_eye_path = ... ; st.hand_eye.save(hand_eye_path)`** — 이미 제거됨. legacy npz disk save 0.
- **`handeye_poses.npz`** — capture session 의 working state (자세 임시 보존, 재시작 후 복원용). storage 와 별개 — 유지.

### Stage 3 — 마이그레이션 스크립트 (불필요, 폐기)

**폐기 (2026-06-16 사용자 결정)** — omx_f_0 카메라가 D405 → USB 카메라로 swap 진행 중. 옛 5종 npz 의 intrinsic / hand_eye 는 다른 카메라 캘이라 *틀린 값* (detector 가 엉뚱한 위치 찾음). "아예 처음부터" = kinematic 캘 (joint / link / sag) 도 다시. → **옛 npz 모두 garbage, import 가 의미 없음.**

본 흐름 — `storage.db` 빈 채로 첫 부팅 → hardware 자리에서 5종 캘 처음부터:
1. backend 첫 부팅 → `storage.db` 빈 채.
2. UI 의 [캘 패널] 에서 omx_f_0 / so101_6dof_0 각각 5종 캘 처음부터 (intrinsic capture → save / hand_eye capture/compute/commit / ...).
3. 각 COMMIT → 새 Run + Result row INSERT (is_active=false). ACTIVATE 별도 step.
4. ACTIVATE 후 invalidation publish → 노드들이 캘 fetch + push.

(이력 — 본 세션에서 `backend/scripts/migrate_calibration_to_storage.py` 작성했으나 omx 카메라 swap 으로 무의미해져 삭제. 미래 유사 자리 필요해지면 git history 에서 재현 가능.)

### Stage 4 — Frontend UI (진행 중)

**Design A — Run 단위 통합 list (2026-06-16 결정)**

List 모델 — `STORAGE_LIST_CALIBRATIONS(robot_id, kind, limit)` 는 *kind 별*. RollbackPanel 의 직관 ("한 시점 = 한 캘 세션") 으로 보려면 **Run 단위 그루핑** 이 자연. 산업 표준 (MLflow Model Registry / git history) 정합.

→ backend 에 **`STORAGE_LIST_CALIBRATION_RUNS(robot_id, limit)`** 추가 (commit 본 세션). 한 Run + 그 Run 의 모든 kind Result 가 묶여 옴. frontend 가 5번 호출 안 하고 1번에 받음.

```python
# backend/core/transport/messages/storage.py
class CalibrationRunSummary(StrictModel):
    run: CalibrationRunRecord
    results: list[CalibrationResultRecord]  # 그 Run 의 모든 kind

class StorageListRunsReq(StrictModel):
    robot_id: str
    limit: int = 50

class StorageListRunsRes(StrictModel):
    runs: list[CalibrationRunSummary]  # run.started_at DESC 정렬
```

다른 옵션 (보류):
- B. kind 별 tab — 5 tab. σ_rot 후퇴 비교 (hand_eye 만) 자연하지만 5종 묶음 rollback = 5번 클릭. UX 비용.
- C. 통합 list + kind filter — A + B 절충, 화면 복잡.

**진행 단계:**

1. ✅ **Backend `list_runs` API 추가** — `STORAGE_LIST_CALIBRATION_RUNS` Service / `CalibrationRunSummary` / `StorageListRunsReq` / `StorageListRunsRes` + RdbStore Protocol + SqliteStore (N+1 query 회피 single IN query) + MemoryRdbStore + storage_node handler + CalibrationStorageClient.list_runs + api_contract.PUBLIC_SERVICES.

2. ⏳ **`pnpm gen:types`** — backend `/openapi.json` 의 `x-contract` 가 새 service + invalidation topic 다 포함. frontend codegen 실행 → `contract.ts` / `types.ts` 갱신.

3. ⏳ **`STORAGE_CALIBRATION_INVALIDATED` topic 구독** — frontend 의 `BridgeClient` 가 본 topic 받으면 `useCalibrationRuns` hook 의 query 강제 refetch + 패널 갱신. 현재 frontend 는 invalidation 받아 refetch 안 함 → page reload 시점에만 fresh.

4. ⏳ **list / ACTIVATE 패널 (RollbackPanel swap)** — `STORAGE_LIST_CALIBRATION_RUNS(robot_id, limit)` 호출 → Run 별 row → 펼침 → 5 kind 의 σ_rot / σ_t / created_at + ACTIVATE 버튼 (Run 전체 / kind 별 양쪽). 현재 활성 강조 (is_active=true row).
   - 기존 Rollback 탭 swap (panel 이름 그대로 두고 내부 구현만 swap, 또는 `CalibrationHistoryPanel` 로 이름 변경).
   - `.history/` 의존 (옛 `CALIB_BACKUP_LIST` / `CALIB_BACKUP_RESTORE`) 사라짐.

5. ⏳ **`HandeyeCommitRes.path`** — `"storage:run/3"` 식 새 형식. frontend 가 *파일 경로* 표기로 사용 중인지 확인 → "run #3" 식으로 변경 (또는 그대로 둠).

6. **Stage 2d cleanup** — Stage 4 끝나면 backend 의 `backup.py` + `CALIB_BACKUP_*` 서비스 제거.

### Phase 2 / 3 generic — 미리 박지 말 것

- scans/meshes/task_runs 의 schema/service/Protocol method — Phase 2 진입 시 논의
- 캘 raw image (Artifact 계층) — Phase 2 진입 시 도입 (§6 매핑)
- NAS Postgres/MinIO adapter 의 connection pool / transaction 정책 — Phase 3 진입 시 논의

## 12. 다음 세션 anchor (2026-06-16 갱신)

본 세션 (2026-06-15 → 06-16) 완료된 자리:

- ✅ `.patched/` URDF 디스크 영속화 폐기 (Option A — in-memory `patch_urdf_text` + tempfile, §13).
- ✅ omx 5DOF 하드코딩 일부 fix (`_default_joint_id_map` 자동 도출 / `joint_distribution` cap 제거 / `thresholds` 주석). 보류 자리는 §13.6 (5.5).
- ✅ Backend `STORAGE_LIST_CALIBRATION_RUNS` API (Run 단위 통합 list, §13.7 Stage 4 design A).
- ✅ Frontend `useCalibrationRuns` hook + invalidation 자동 refetch.
- ✅ Frontend `CalibrationHistoryPanel` (옛 `RollbackPanel` swap. registry / RobotCalibrateMode 갱신, `BackupEntry` type 삭제).
- ❌ Stage 3 마이그레이션 스크립트 (작성했으나 omx 카메라 swap 으로 폐기, §11 Stage 3).

**다음 세션 시작 자리** (hardware 자리에서 검증):

1. **첫 부팅 검증** — `storage.db` 빈 채로 host_dev 시작. 빈 storage 에서 calibration_node 가 `STORAGE_GET_ACTIVE_CALIBRATION` 호출 시 `found=false` 정상 응답 + 노드들이 default 캘 (identity / joint_offset=0 등) 로 wait 모드 진입 확인. UI 의 `CalibrationHistoryPanel` 이 빈 list 정상 표시.

2. **omx_f_0 첫 캘** (USB 카메라 결선 후) — UI 의 [Intrinsic] 패널 capture/save → storage.db 에 intrinsic Run + Result INSERT. ACTIVATE → invalidation → 다른 노드 refetch. 이어서 hand_eye / joint_offset / link_offset / sag 순서. **각 단계마다 `CalibrationHistoryPanel` 의 Run row 추가 확인 + ACTIVATE 시 활성 강조 색깔 확인.**

3. **so101_6dof_0 첫 캘** — 결선 후 동일 흐름. `pose_recommend_strategy: geometry` (6DOF anchor IK) 동작 검증.

4. **Stage 2d cleanup** — Stage 4 (frontend) 가 hardware 검증으로 동작 확인되면 backend 의 `backup.py` + `CALIB_BACKUP_LIST` / `CALIB_BACKUP_RESTORE` 서비스 제거. frontend 의 `RollbackPanel` 은 이미 삭제됨.

5. **so101 link_offset 캘 본격 진입 시점에** — §13.6 (5.5) 의 omx 하드코딩 anchor 처리:
   - `fk_chain.py` (omx 5DOF literal numpy array) 일반화 — pinocchio / KDL / Drake 의 URDF parse → numpy FK chain build 패턴 산업 표준 리서치 먼저.
   - `sag_corrected._ARM_DOF=5` (fk_chain 묶임).
   - `observability.wrist_roll_axis=4` default — `robots.yaml::wrist_roll_motor_index` per-robot config 추가 + `RobotConfig` 노출 + caller 명시 주입 + observability default 제거.

**(보류) 검증 필요 자리** (§13.6):
- (5.1) PyBullet 가 `loadURDF` 후 URDF 파일 다시 안 봄 확인 — `.patched/` 폐기 + tempfile 패턴 의존. fk / ik 동작 확인 (수렴 + EE pos 정상).
- (5.2) Option B (xacro 도입) — SO-101 URDF refactor 시점에 묶기.
- (5.3) mesh `package://` 도입 — ROS 2 stack 과 URDF 공유 필요해질 때.
- (5.4) hand_eye 도 URDF 안의 `camera_link` joint origin 으로 박는 게 정통 — Option B 시점.

## 13. link_offset 적용 메커니즘 — `.patched/` 폐기 + in-memory render

> **결정 요약 (2026-06-15)** — `robot/<robot_type>/urdf/.patched/` 디스크 영속화 폐기. 매 부팅 시 `patch_urdf_text` 로 in-memory render → PyBullet `loadURDF` 가 path-only 라 OS temp 파일로 1회성 우회 → load 직후 unlink. SSOT = storage DB, URDF 는 process 시작 시점에 render 된 임시 string.

### 13.1 동기 — `.patched/` 가 git 시대 잔재

`link_offset` 만 다른 4 종 (intrinsic/hand_eye/joint_offset/sag) 과 적용 메커니즘이 달랐음:
- 다른 4 종 → 메모리 cache 의 값을 함수에서 곱하는 식 (런타임 hot-reload)
- `link_offset` → URDF 의 `<joint><origin xyz/rpy>` 에 delta 가산 → 새 URDF 파일을 `robot/<type>/urdf/.patched/` 디스크에 저장 → PyBullet 이 그 파일 load

디스크 영속화의 이유 — git push/pull 분산 동기화 시대에 `.patched/` URDF 를 분산 머신끼리 git 으로 공유해야 했음. storage_node 도입 후:
- `link_offset` SSOT = storage DB row (kind='link_offset')
- 분산 동기화 = invalidation topic + push (자동)
- `.patched/` URDF = DB 의 stale render = **두 곳 동기화 책임 발생 (SSOT 원칙 위반)**

추가 문제 — `.patched/` 경로가 `robot/<robot_type>/urdf/` (TYPE 폴더). 같은 type 다중 instance (omx_f_0, omx_f_1) 가 같은 `.patched/` 디렉토리 공유 → last-writer-wins 충돌. N=1 라서 가려진 multi-robot 버그.

### 13.2 산업 표준 리서치 (2026-06-15)

robot stack 들의 캘 결과 URDF 적용 패턴 — 전부 같은 모양으로 수렴:

```
YAML (캘 값, SSOT) → xacro/template render → URDF string → parser → in-memory model
```

| Stack | 캘 결과 저장 자리 | URDF 적용 방식 |
|---|---|---|
| ROS 2 `robot_state_publisher` | xacro args / robot_description parameter | parameter 에 URDF *string* 으로 inject — file 도 topic 도 아님 |
| Universal Robots `ur_calibration` | `<robot>_calibration.yaml` (DH delta) | launch 시 xacro 가 YAML 읽어 `<joint><origin>` 변수 치환 |
| Franka Emika | `kinematics.yaml` + 3 종 (`inertials/joint_limits/dynamics`) | xacro `load_yaml` + `${var}` 치환 |
| Drake | YAML / xacro | `Parser.AddModelsFromString(urdf_text, "urdf")` — in-memory 직접 |
| Isaac Sim | URDF / USD | UrdfImporter 가 in-memory string parse 지원 |

**공통 원칙**:
1. URDF = *파생물*, SSOT 는 YAML/DB
2. render 는 매 process 시작 시 1회 (런타임 reload 안 함)
3. rendered URDF 는 string 으로 살아 있고 **디스크에 영속 X**
4. mesh path 는 parser 의 resolver (PackageMap 등) 또는 절대경로 rewrite

### 13.3 PyBullet 의 제약

- **bullet3 C++ 코어** `UrdfParser::loadUrdf(const char* urdfText, ...)` — in-memory string load **지원**
- **PyBullet Python wrapper** `pybullet.loadURDF(fileName=...)` — **path-only** (SharedMemoryCommand 가 path 만 직렬화, wrapper 가 string API 안 노출)
- **런타임 override** — `changeDynamics` 가 mass/inertia/CoM/friction 만 변경, **joint origin xyz/rpy 못 건드림**. `resetJointState` 는 joint *position* 만. 런타임 patch 경로 **없음**

→ PyBullet user 입장에선 *tempfile 1회성 우회* 가 커뮤니티 정석 workaround.

### 13.4 옵션 비교

| 옵션 | 메커니즘 | 산업 표준 정합 | 면적 | 채택 |
|---|---|---|---|---|
| **A. tempfile in-memory render** | `patch_urdf_text` → tempfile → loadURDF → unlink | ROS 2 `robot_description` 정신 (PyBullet wrapper 한계 위에서 emulation) | ~30 LOC | ✅ 즉시 |
| **B. xacro/jinja2 템플릿** | URDF.xacro + storage.link_offset → template render → string → tempfile | UR/Franka/ROS 정통 패턴 그대로 | ~200-400 LOC + URDF 재작성 | SO-101 URDF refactor 시점에 묶기 |
| C. Drake adapter | `Parser.AddModelsFromString` (in-memory) | Drake 정통 | ~300 LOC + 의존성 거대 + Win11 약함 | 오버킬 |
| D. 디스크 유지 + per-instance | `.patched/` 를 instance 폴더로 옮김 | 낮음 — 어떤 stack 도 안 함 | 작음 | SSOT 원칙 위반 잔존 |

### 13.5 채택 — A → B 단계적

**즉시 (지금, Option A)**:
- `patch_urdf_text(...)` 는 그대로 유지 (이미 in-memory string 반환)
- `write_patched_urdf` + `patched_urdf_path` 제거
- `PybulletKinematics.initialize` 안:
  ```python
  patched_text = patch_urdf_text(self._urdf_path, self._link_offsets)
  fd, temp_path = tempfile.mkstemp(suffix=".urdf", prefix="horibot_")
  try:
      with os.fdopen(fd, "w", encoding="utf-8") as f:
          f.write(patched_text)
      self._client = p.connect(p.DIRECT)
      self._robot = p.loadURDF(temp_path, useFixedBase=True, ...)
  finally:
      try: os.unlink(temp_path)
      except OSError: pass
  ```
- `.patched/` 디렉토리 통째 삭제 (gitignored 라 그대로 사라짐)
- mesh 절대경로 rewrite (`patch_urdf_text` 안의 자리) **유지** — PyBullet 가 `package://` 지원 안 함

**중기 (Option B, SO-101 URDF refactor 시점에 묶기)**:
- omx_f / so101_6dof URDF 둘 다 `*.urdf.xacro` 화
- `urdf_patcher` → xacro/jinja2 wrapper 로 교체
- LinkOffsets semantic 재정의 (delta 가산 → nominal + override 의 어느 자리에 박힐지) — UR/Franka 패턴 학습 자리

### 13.6 검증 + 보류

**Option A 검증 step** — PyBullet 가 `loadURDF` 후 URDF 파일 자체를 다시 안 본다는 가정 의존. 실 hardware 자리에서 (or DIRECT 모드 single fk/ik test 로):
1. tempfile 에 patched URDF write → loadURDF → 즉시 unlink
2. fk(joint_angles) 호출 → EE pos 정상 반환
3. ik(target_pos) 호출 → 수렴 + 정상 joint_angles 반환
4. mesh 가 정상 보이는지 (visual 모드 또는 collision check) — mesh 는 lazy load 라 unlink 후에도 원본 mesh 파일에서 읽어야 함, mesh 절대경로 rewrite 가 그 자리

**보류 자리들**:
- (5.1) PyBullet 가 loadURDF 후 URDF 다시 보는 자리 검증 미완 — Option A 채택 시 1회 확인
- (5.2) Option B 의 LinkOffsets semantic (delta vs absolute) — Option B 진입 시 재논의
- (5.3) mesh `package://` 도입 — ROS 2 stack 과 URDF 공유 필요해질 때 별도 결정
- (5.4) hand_eye 도 URDF 안의 `camera_link` joint origin 으로 박는 게 정통 (MoveIt 패턴) — Option B 진입 시 같이 검토
- (5.5) **omx 5DOF 하드코딩 잔재 정리** (2026-06-15 사용자 짚어주기) — 본 storage_layer 작업과 별개 multi-robot 일반화 자리. 발견 자리:
  - [backend/modules/kinematics/fk_chain.py](../backend/modules/kinematics/fk_chain.py) — `JOINT_ORIGINS` / `JOINT_AXES` / `EE_ORIGIN` / `N_JOINTS=5` 가 omx_f URDF geometry 의 *literal numpy array*. BA + sag 가 이걸 씀 (PybulletKinematics 아닌 별도 numpy FK — BA 가 매 LM iteration 마다 link_offset 변수로 호출하려 PyBullet 우회). **so101 link_offset 캘 진입 시 omx geometry 위에서 풀어버림 = 완전 잘못된 결과.**
  - [backend/modules/kinematics/adapters/sag_corrected.py:39](../backend/modules/kinematics/adapters/sag_corrected.py#L39) — `_ARM_DOF: int = 5` 모듈 상수. fk_chain 의 `apply_gravity_sag` 가 5-element array 전제라 sag_corrected 단독 일반화 불가 — fk_chain 과 묶인 자리.
  - [backend/modules/calibration/observability.py:58](../backend/modules/calibration/observability.py#L58) — `wrist_roll_axis: int = 4` default = OMX-F joint5 (motor index 4, 0-indexed). `analyze_pose_data` caller (`calibration_node._srv_handeye_capture`) 가 명시 주입 안 함 — so101 가 omx 가정 사용. fix = `robots.yaml::wrist_roll_motor_index` per-robot config 추가 + `RobotRegistry::RobotConfig` 노출 + caller 가 `cfg.wrist_roll_motor_index` 주입 + observability default 제거.

  **fix 면적**:
  - (a) **fk_chain.py 일반화** = 큼 (~100+ LOC). 산업 표준 리서치 필요 — pinocchio / KDL / Drake 의 "URDF parse → numpy FK kinematics chain build" 패턴 어떻게 하는지 분석 후 결정 ([feedback-research-before-decide](../memory/feedback_research_before_decide.md)). BA 의 매 LM iteration FK 호출 hot path 라 성능 / 정확성 양쪽 검토.
  - (b) sag_corrected ARM_DOF = (a) 와 묶임. fk_chain 일반화 시 자동 정리.
  - (c) observability wrist_roll_axis = robots.yaml schema 결정 자리. fix 면적 ~30 LOC (yaml + RobotConfig + caller + observability).

  **트리거** — so101 의 link_offset 캘 / hand_eye 캘 본격 진입 시점. 그 전엔 omx_f_0 만 운영 (so101 enabled 지만 결선 안 됨) 이라 silent.

  완료된 자리 (2026-06-15 본 세션):
  - `urdf_patcher.py::_default_joint_id_map` (range(1,6) 하드코딩) → `LinkOffsets.ids` 기반 자동 도출 (SSOT)
  - `joint_distribution.py::n_axes = min(..., 5)` → cap 제거 + `JOINT_NAMES_KO` i 범위 밖 fallback
  - `thresholds.py::JOINT_DIVERSITY_THRESHOLD_DEG` 주석을 multi-robot 컨텍스트로 갱신 (코드는 `joint_distribution` 의 fallback 패턴이 이미 처리)

### 13.7 산업 표준 references

- [bullet3 UrdfParser::loadUrdf](https://github.com/bulletphysics/bullet3/blob/master/examples/Importers/ImportURDFDemo/UrdfParser.cpp) — C++ string load 지원
- [Universal_Robots_ROS_Driver/ur_calibration](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/tree/master/ur_calibration) — YAML SSOT + xacro substitute (우리와 가장 닮은 패턴)
- [frankaemika/franka_description](https://github.com/frankaemika/franka_description) — kinematics.yaml + xacro
- [Drake Parser.AddModelsFromString](https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_parser.html) — in-memory 정통
- [ROS 2 robot_description parameter pattern](https://medium.com/@zakerima/understanding-robot-description-in-ros-2-its-not-a-topic-f2f365f2b496)
