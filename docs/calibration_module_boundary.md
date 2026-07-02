# Calibration Module — boundary spec (v2 Step E 진입 전 설계 확정)

> **상태**: **풀스택 구현·검증 완료 (2026-07-02) — 회사(mock+sim) 검증 가능한 전부 green. 남은 건 집의 실물 D405+SO-101.**
>
> 구현 (folder 구조 — contract/module 은 module root, 나머지는 concern 별 서브폴더):
> ```
> modules/calibration/
>     contract.py            # wire (Service/Stream/Event enum + pydantic + Bundle)
>     module.py              # CalibrationModule (@service/@publishes, robot-scoped)
>     persistence/           # orm.py (4 entity, 공유 Base) · repository.py (advanced-alchemy caged)
>     vision/                # board.py · processing.py(detect+PnP) · capture_quality.py · thresholds.py · se3.py · sim_board.py
> ```
> + 루트 `backend_v2/alembic/` + `infra/database/{base,types,boot}.py` + registry/resolve/config/mock·pc.yaml + `apps/contract_export.py` 노출 + Camera `GET_FACTORY_INTRINSIC` + mock camera sim board + frontend `CalibrationPanel`/`RobotCalibrateMode`/registry/sidebar/route + `contract.ts` 재생성.
>
> **검증**: backend **168 PASS** (persistence 실 horibot.db fixture + known σ 0.818°/7.538mm / alembic upgrade==metadata + partial unique / module @service + 이벤트 / **capture sim-image e2e** detect→PnP→gate→DB+blob / preview 5Hz / boot Zenoh 도달 + **§10.1 factory intrinsic auto-seed over-wire**), ruff/pyright clean. frontend tsc/eslint clean. **Playwright headed 4/4 PASS** (실 브라우저→vite→bridge WS→서비스→DB): WS연결/패널·bundle / preview toggle / start_run→history / **preview 검출(green)+capture accepted** (sim board→intrinsic→CameraDecoded→PnP→DB row+ `.calib_blobs_mock/.../000_color.jpg` 디스크 기록 확인).
>
> **offline BA 이월 완료 (2026-07-02, §11.1)** — fk_chain/depth_frame/calibrate_offline/calibrate_squeeze/physical.yaml 포팅, **포팅 faithful 증명됨 (old==new bit-identical)**, 177 PASS. ⚠️ committed σ 0.818°/7.538mm 은 미기록 hand-tuned drop-9 subset 결과라 bit-exact 재현 불가 — **port 버그 아님, 재현 재시도 금지** (§11.1). 다음 = **결정론적 auto outlier-selection 설계 §11.2** (매 캘 운 제거).
>
> **집(실물)에서 남은 것**: 실 D405 factory intrinsic (mock synthetic 대체) / 실 ChArUco 캡처 정확도 / **Motion boot consumer** (Motion.start() 가 snapshot_bundle 읽어 kinematics build — §9, 별도 slice 아직 미배선).
>
> 핵심 설계 결정: Calibration Bundle = boot-time configuration (Mirror 안 씀). §6 / §10.1 / §10.2 재작성 완료. SSOT spec = [backend_v2.md](backend_v2.md) + [backend_v2_modules.md](backend_v2_modules.md).
>
> **v2 개선 (옛 → v2, 기계적 복사 X)**: ① ORM 이 shared storage Base → **calibration 자기 Base** (Database-per-Module). ② `UtcDateTime` storage 소유 → **`infra/database/types.py` 승격** (전역 UTC 컨벤션). ③ `infra/database/sqlite.py::open_sqlite` 에 **sqlite FK pragma 추가** (없으면 `ondelete=CASCADE` 조용히 no-op — calibration 이 첫 FK-CASCADE 테이블이라 드러남). ④ result data 모델 = **순수 pydantic** (옛 numpy helper 는 consumer 로). ⑤ result 계열 `extra="forbid"` (DB drift fail-fast). ⑥ **advanced-alchemy `SQLAlchemySyncRepository` = 모듈 내부 CRUD 헬퍼로만** (add/commit/refresh·get·get_many·delete ceremony 제거) — framework 는 전제 X (§10.4, plain Protocol 유지), 라이브러리 의존은 repository.py 안에 갇힘. 도메인 로직(activate atomic / bundle aggregate / undo)은 직접. ⑦ migration = **루트 단일 Alembic** (§8, 소유권≠마이그레이션 권위).
>
> **원칙 1 — 영속성 (2026-07-01)**: Storage Module 자리 없음 (v2 §9 폐기 확정). 각 domain module 이 자기 영속성 owner. Repository / ObjectStore 는 framework Protocol (이미 존재), 도메인 module 이 자기 Repository impl 소유 (Database-per-Module + DIP).
>
> **원칙 2 — Configuration vs Runtime State (2026-07-02, framework 차원 decision rule)**:
> - **Configuration** — boot-time query 로 프로세스 시작 시 1회 조회, 프로세스 동안 불변. 변경은 다음 부팅부터 효력. Mirror 안 씀.
> - **Runtime State** — Owner→Mirror→Reader 로 read-only 공유, event 로 갱신 (JointRad / TcpState / RobotState 등).
> - **Calibration Bundle 은 Configuration 이다.** commit / rollback / activate_result 로 active row 가 바뀌어도, 그 효력은 **다음 부팅부터** 발생한다. 런타임 중 Motion kinematics 를 재초기화하지 않는다. 근거: offline BA commit (`calibrate_offline.py --commit`) 은 backend 종료 상태에서 실행 (RDB lock) → Motion 살아있는 동안 Bundle 이 바뀌는 순간이 아예 없음. rollback 도 재시작 flow 로 통일. link_offset / joint_offset / sag_offset / intrinsic / hand_eye 5종 전부 "설정 갈아끼우기 (config swap)" 이지 "상태 업데이트 (state update)" 가 아님. MoveIt 이 URDF/SRDF/kinematics plugin 변경 시 노드 재로드하는 것과 동형.
>
> **2026-07-01 → 07-02 방향 정정**: 07-01 판단은 §10.2 를 "옛 restart-only 이월 = Mirror real-time 원칙 위반" 이라 보고 `PybulletKinematics.reinitialize()` 런타임 재로드를 넣었다. **이건 "Mirror 니까 실시간이어야 한다" 는 아키텍처적 연역이었고, 실제 워크플로우엔 그 트리거가 없다.** 07-02 재판단: restart-only 가 위반이 아니라 **정석** (Bundle=config). §10.2 machinery 제거, Mirror 제거, boot-query 로 대체.

## 0. TL;DR

Calibration Module = **5 종 산출물 Bundle owner (intrinsic / hand_eye / joint_offset / link_offset / sag_offset)** + capture loop + Bundle 발행. Backend 는 capture 수집 + storage append 만, BA / σ / observability 는 **offline 스크립트** (옛 backend 결정 이월). Bundle 은 **boot-time configuration** — consumer (Motion 등) 는 자기 start() 때 `snapshot_bundle` service 로 1회 읽고, 변경은 다음 부팅부터 반영 (원칙 2). **Mirror 안 씀.**

## 1. Ownership

### 1.1 Calibration Module 이 소유

| Entity | 위치 | 성격 |
|---|---|---|
| `CalibrationRun` | DB (`calibration_runs`) | capture session lifecycle (draft → ready_for_analysis → success/failed) |
| `CalibrationResult` | DB (`calibration_results`) | 5 kind 별 산출물 row + is_active flag (kind 별 1 개 active) |
| `CalibrationCapture` | DB (`calibration_captures`) | 자세별 raw ChArUco corners + motor_positions (raw SSOT) |
| `CalibrationCaptureArtifact` | DB (`calibration_capture_artifacts`) | blob metadata (ObjectStore key + kind + size) |
| capture blob | ObjectStore (`calibration/<robot_id>/runs/<run_id>/captures/<pose_index>/primary.bin`) | color JPEG + depth zstd + intrinsic snapshot |

### 1.2 소유하지 않는 것 (다른 module 의 domain)

| Not owns | 소유 module | Calibration 이 접근 방법 |
|---|---|---|
| robot motion state | Motion | `Motion.Service.TCP_SNAPSHOT` 호출 (capture 시 point-in-time) |
| camera runtime state / factory intrinsic | CameraDriver | `CameraDecoded.Stream.*` 구독 (preview 5Hz), factory intrinsic 은 driver internal (§7.7 anchor #13) |
| joint offset 실 적용 | Motion (kinematics rebuild) | `Mirror[CalibrationBundle]` 발행, Motion 이 reader |
| scene graph / detection | Scene3D / Detector | 접근 X (Calibration 은 upstream) |

**invariant**: 같은 `(robot_id, kind)` 의 `is_active=True` result 는 **최대 1개** (partial UNIQUE index — [db_schema.md §2.3](db_schema.md)).

## 2. Repository boundary

framework 의 `Repository(Protocol[T])` (get/save/delete) 위에 Calibration 자기 impl 이 얹힘. Entity-specific method 추가.

```python
# modules/calibration/repository.py (설계 초안 — 코드 아님)

class CalibrationRepository:
    def __init__(self, session_factory: Callable[[], Session]):
        self._session_factory = session_factory

    # ── run lifecycle ─────────────────────────────────────────
    def create_run(self, robot_id: str, kind: str, algorithm: str) -> CalibrationRun: ...
    def get_run(self, run_id: int) -> CalibrationRun | None: ...
    def get_in_progress_run(self, robot_id: str, kind: str) -> CalibrationRun | None: ...
    def finalize_run(self, run_id: int, status: str) -> None: ...     # in_progress → ready_for_analysis

    # ── capture (per-pose) ────────────────────────────────────
    def append_capture(self, run_id: int, capture: CalibrationCapture) -> int: ...
    def list_captures(self, run_id: int) -> list[CalibrationCapture]: ...
    def undo_last_capture(self, run_id: int) -> None: ...
    def save_artifact(self, capture_id: int, artifact: CalibrationCaptureArtifact) -> None: ...

    # ── result (5 kind, activate atomic) ─────────────────────
    def save_result(self, run_id: int, result: CalibrationResult) -> int: ...   # is_active=False
    def activate_result(self, result_id: int) -> None: ...   # atomic: 같은 (robot_id, kind) active 해제 + 새 activate
    def get_active(self, robot_id: str, kind: str) -> CalibrationResult | None: ...
    def get_active_bundle(self, robot_id: str) -> CalibrationBundle: ...   # 5 kind snapshot

    # ── history (readonly view) ──────────────────────────────
    def list_runs(self, robot_id: str, kind: str | None) -> list[CalibrationRun]: ...
    def list_results(self, robot_id: str, kind: str | None) -> list[CalibrationResult]: ...
```

**핵심 결정**:
- `get_active_bundle(robot_id)` = 5 kind 를 한번에 뽑아 `CalibrationBundle` (atomic snapshot). Mirror payload 원천.
- `activate_result` = transaction 안 두 operation (이전 active 해제 + 새 activate) — atomic.
- `finalize_run` = capture 세션 종료. status = `ready_for_analysis`. 이후 offline 스크립트가 BA + `save_result` + `activate_result`.

## 3. ObjectStore boundary — DB vs blob 분리

| 항목 | 저장 자리 | 이유 |
|---|---|---|
| capture 시각 (`timestamp`) | DB row | metadata, 조회 |
| capture 자세 (`motor_positions`) | DB row (JSON text) | raw SSOT, post-hoc reinterpret (joint_offset 갱신돼도 raw 불변) |
| ChArUco detected corners | DB row 또는 blob | 결정 자리: 코너 개수 × 8byte float × N 자세 ≤ 수 KB — DB row 로 (조회 편함) |
| Color JPEG (per capture) | **ObjectStore blob** | ~100 KB × N 자세, DB 부담 |
| Depth zstd (per capture) | **ObjectStore blob** | ~수 MB × N 자세 |
| Intrinsic snapshot at capture time | DB row (JSON) | 작음, 조회 편함 |
| Result parameter (matrix, σ, etc.) | DB row (JSON) | 작음 (SE(3) 하나 = 12 float) |

**Blob key convention** (실 DB horibot.db 확인 — 초안의 `calibration/.../primary.bin` 은 이상화였음): 실제는 `calib_captures/<robot_id>/<run_id>/<pose_index:03d>.bin`. 한 capture 당 artifact **5 종** (`primary` / `color` / `depth` / `depth_vis` / `ply`) 각각 별 blob + `calibration_capture_artifacts` row. `primary` 가 위 key, 나머지는 kind 별 suffix.

## 4. Service API

### 4.1 Commands (write)

| Service key | 책임 | 요청 → 응답 | Caller | Frontend 노출 |
|---|---|---|---|---|
| `srv/calibration/{robot_id}/start_run` | 새 run 생성 (draft), intrinsic snapshot 박음 | `{kind, algorithm}` → `{run_id}` | frontend | ✅ |
| `srv/calibration/{robot_id}/capture` | ChArUco preview 위 현재 자세 capture | `{run_id, pose_index}` → `{capture_id, quality}` | frontend | ✅ |
| `srv/calibration/{robot_id}/undo_last_capture` | 마지막 capture row + blob 삭제 | `{run_id}` → `{ok}` | frontend | ✅ |
| `srv/calibration/{robot_id}/finalize_run` | run 종료 → `ready_for_analysis` | `{run_id}` → `{ok}` | frontend | ✅ |
| `srv/calibration/{robot_id}/activate_result` | offline BA 후 activate (rollback 자리도) | `{result_id}` → `{ok}` (publishes `CalibrationActivated`) | frontend + offline script | ✅ |
| `srv/calibration/{robot_id}/preview_enable` | ChArUco preview loop on/off (5Hz) | `{enabled}` → `{ok}` | frontend | ✅ |

**Note**: 옛 backend 의 Camera → Storage RPC write 방향 (factory intrinsic seed) 은 v2 자리 `SEED_INTRINSIC` service 로 짤 뻔했으나 Owner/Reader 위반 자리 (§10.1). 대신 Camera 가 factory intrinsic 을 event 로 publish → Calibration 이 subscribe 하는 방향으로 재설계 (§4.4 subscribers 참조).

### 4.2 Queries (read)

| Service key | 책임 | 요청 → 응답 |
|---|---|---|
| `srv/calibration/{robot_id}/snapshot_bundle` | 현재 active 5 kind bundle 반환 (consumer 가 boot 때 1회 조회 — boot-time query, §6) | `{}` → `CalibrationBundle` |
| `srv/calibration/{robot_id}/list_runs` | run history | `{kind?}` → `[CalibrationRun]` |
| `srv/calibration/{robot_id}/list_results` | result history (rollback UI 자리) | `{kind?}` → `[CalibrationResult]` |
| `srv/calibration/{robot_id}/get_thresholds` | Traffic Light 임계 (min/max tilt / pose diversity 등) | `{}` → `CalibrationThresholds` |

### 4.3 Streams (output)

| Stream key | 방향 | payload |
|---|---|---|
| `stream/calibration/{robot_id}/preview` | Calibration → frontend (5Hz) | ChArUco detected overlay + traffic light verdict + capture_verdict/reasons |

### 4.4 Boot-time dependency (Calibration 이 다른 module 을 pull)

event subscribe 아님 — Calibration 이 자기 start() 에서 Camera 를 **pull** (§10.1 확정, Configuration 성격이라 event refetch 불필요).

| Pull target | 발행자 | 처리 | 근거 |
|---|---|---|---|
| `srv/camera/{robot_id}/get_factory_intrinsic` (internal, FRONTEND_EXPOSED X) | CameraDriver (요청 시 SDK 짧게 open) | Calibration.start() 에서 1회 호출 → idempotent seed: `get_active(robot_id, "intrinsic")` 없으면 create_run + save_result + activate_result. 이미 있으면 skip (사용자 chessboard 캘 결과 덮어쓰지 않음) | §10.1 — Camera 는 SDK 데이터를 read-only service 로 노출 (자기 domain), Calibration 이 caller 로 pull 후 자기 domain rule (intrinsic table owner) 로 seed. Write 방향 뒤집힘 없음. factory intrinsic 은 static 값 + boot-time config 라 event 스트림 불필요 |

## 5. Events (@publishes)

| Event key | 발행 시점 | payload | 성격 |
|---|---|---|---|
| `event/calibration/{robot_id}/activated` | `activate_result` 성공 시 (activate 또는 rollback) | `{robot_id, bundle_id, version}` | **알림 only** — "새 bundle 이 active 됐으니 재시작하면 적용됨". frontend 가 "재시작 필요" badge 띄우는 용도. **Mirror refetch 트리거 아님** (Bundle=config, 런타임 반영 X) |
| `event/calibration/{robot_id}/committed` | `finalize_run` 성공 시 | `{robot_id, run_id}` | 알림 |

**versioning 원칙**: `bundle_id` = DB row id, `version` = monotonic. audit / debug 및 frontend 의 "현재 active bundle vs 실행 중 프로세스가 로드한 bundle" 비교 (재시작 필요 여부 판단) 자리. 런타임 rebuild 추적 아님 (rebuild 자체가 없음).

## 6. Bundle consumption — boot-time query (Mirror 아님)

**Calibration Bundle 은 Configuration** (원칙 2) — consumer 는 자기 start() 에서 `snapshot_bundle` service 를 **1회 호출**해 5 kind snapshot 을 받고, 그 위에서 자기 것(kinematics / detection cache 등)을 build. 이후 프로세스 동안 재조회 X. Bundle 이 바뀌면(commit / rollback) 그 효력은 **다음 부팅부터** — `CalibrationActivated` event 는 refetch 트리거가 아니라 "재시작 필요" 알림 (§5).

> **Mirror 안 쓰는 이유**: Mirror = runtime state (계속 바뀌는 값) 를 event 로 실시간 공유하는 primitive. Calibration Bundle 은 runtime state 가 아니라 config 라 성격이 다름. framework 에 Mirror primitive 가 있다는 이유로 config 를 Mirror 로 노출하면 목적/수단이 뒤바뀜 (§10.2 07-01 실수의 교훈). Mirror primitive 자체는 Motion 의 JointRad / TcpState 등 실 runtime state 를 위해 유지됨 — 여기서 안 쓸 뿐.

### 6.1 `CalibrationBundle` — active 5 kind snapshot (boot query payload)

**구현됨** (`modules/calibration/contract.py`). 초안의 `bundle_id` / monotonic `version` 은 **제거** — 실 DB 엔 bundle 엔티티가 없고 kind 별 result row 가 독립(각자 id / is_active). "변경됐나?" 는 active result id 집합(`signature()`)으로 비교. field 이름은 DB kind 문자열과 일치 (`sag_offset` 아니라 **`sag`**).

```python
class CalibrationBundle(BaseModel):
    robot_id: str
    intrinsic:    IntrinsicResultRecord | None = None
    hand_eye:     HandEyeResultRecord | None = None
    joint_offset: JointOffsetResultRecord | None = None
    link_offset:  LinkOffsetResultRecord | None = None
    sag:          SagOffsetResultRecord | None = None

    def signature(self) -> tuple[tuple[str, int], ...]:
        """(kind, result_id) 정렬 튜플 — 두 Bundle 이 같은 active 조합인지 비교."""
```

> **kind 문자열 SSOT** (실 DB 확인): `intrinsic` / `hand_eye` / `joint_offset` / `link_offset` / **`sag`** (`sag_offset` 아님). "sag_offset" 은 개념 명칭이고 DB `kind` 컬럼 / contract Literal 값은 `sag`.

### 6.2 Consumer 계약 (전부 boot-time load)

| Consumer Module | 왜 필요 | 어떤 field 씀 | 언제 읽음 |
|---|---|---|---|
| **Motion** | kinematics build (link_offset → patched URDF PyBullet 로드, sag decorator, joint_offset raw↔rad) | `link_offset`, `sag_offset`, `joint_offset` | **start() 1회**. link_offset patched URDF 로 `PybulletKinematics` 생성, `SagCorrectedKinematics(joint/sag)` decorator. 런타임 재초기화 없음 |
| **Detector** (Step F+) | detection base-frame 변환 | `intrinsic`, `hand_eye` | start() 1회 |
| **Scan** (Step H+) | scan capture 시 intrinsic + hand_eye | `intrinsic`, `hand_eye` | start() 1회 |
| **Reconstruction** (Step I+) | build 시 intrinsic seed | `intrinsic`, `hand_eye`, `joint_offset` | start() 1회 (또는 build job 시작 시 1회 — 어느 쪽이든 config 라 프로세스 내 불변) |

### 6.3 Boot ordering (runtime lifecycle §3.6)

- Consumer 가 start() 에서 `snapshot_bundle` 을 호출하려면 그 시점에 Calibration 이 이미 seed 를 끝냈어야 함 → **Calibration.start() 가 Motion/Detector.start() 보다 먼저** (§3.6 instantiate→register→start 순서에서 Calibration 을 dependency 앞단으로).
- Calibration.start() 는 §4.4 factory intrinsic pull → idempotent seed 를 자기 start 안에서 완료.
- 같은 프로세스에 없으면(분산) — snapshot_bundle 은 Zenoh queryable 이라 network 로 resolve. Calibration runtime 이 아직 안 떴으면 consumer 는 retry/wait (framework lifecycle 처리 자리).

## 7. Migration mapping (옛 backend → v2)

### 7.1 자산 재배치

| 옛 위치 | 자산 | v2 위치 | 형태 |
|---|---|---|---|
| `backend/modules/calibration/orm.py` | 4 entity ORM | `modules/calibration/persistence/orm.py` | 이월 (schema 동일, 공유 `infra.database.base.Base`) |
| `backend/modules/calibration/persistence_models.py` | wire pydantic | `modules/calibration/contract.py` | wire ↔ ORM 변환 |
| `backend/modules/calibration/result_models.py` | 5 kind result dataclass | `modules/calibration/contract.py` | Bundle payload 로 재사용 (numpy helper 는 제거 — 순수 pydantic) |
| `backend/modules/calibration/board.py` | ChArUco detect SSOT | `modules/calibration/vision/board.py` | 이월 (도메인 logic) |
| `backend/modules/calibration/capture_quality.py` | Phase 1 Traffic Light | `modules/calibration/vision/capture_quality.py` | 이월 |
| `backend/modules/calibration/thresholds.py` | tilt / PnP RMS / diversity 임계 | `modules/calibration/vision/thresholds.py` | 이월 |
| `backend/modules/calibration/se3.py` | SE(3) math helper | `modules/calibration/vision/se3.py` | 이월 |
| `backend/modules/calibration/sim_board.py` | ChArUco 시뮬 | `modules/calibration/vision/sim_board.py` | 이월 (mock camera sim board 모드 + test 공용) |
| (신규) capture detect+PnP+tilt+RMS | — | `modules/calibration/vision/processing.py` | 신규 — module 밖 순수 함수 (sim image 단위 검증) |
| `backend/modules/calibration/intrinsic.py` | intrinsic calibrate 로직 | `modules/calibration/intrinsic.py` | 이월 (offline logic 이지만 backend 도 함께 사용) |
| `backend/modules/calibration/applier.py` | 캘 → kinematics 적용 | **Motion.start() 안 boot-time build 로 재배치** | Motion 이 boot 때 snapshot_bundle 읽어 자기 kinematics build — Calibration 이 apply 책임 X (boundary 정정) |
| `backend/modules/calibration/loader.py` | storage 에서 캘 로드 | Repository.get_active_bundle 로 흡수 | 폐기 (Repository 가 흡수) |
| `backend/modules/calibration/calibration_cache.py` | in-memory snapshot cache | consumer 의 boot-time local 변수로 흡수 | 폐기 (config 라 프로세스 시작 시 1회 로드 = 그 자체가 cache) |
| `backend/modules/calibration/storage_client.py` | Storage Module RPC client | **폐기** (Storage Module 자체 폐기) | 직접 Repository 호출로 대체 |
| `backend/scripts/calibrate_offline.py` | 5 stage BA + LOOCV + IRLS + observability | `backend_v2/scripts/calibrate_offline.py` | 이월 (backend 프로세스 밖 offline 도구 — v2 도 동일 위치) |

### 7.2 폐기 자리 정리

- `storage_client.py` — Storage Module RPC → 직접 Repository (Database-per-Module)
- `loader.py` — Repository.get_active_bundle 로 흡수 (별도 loader 자리 불필요)
- `calibration_cache.py` — Mirror 가 cache 자체 (별도 in-memory 자리 불필요)
- `applier.py` — 책임 이관 (Calibration 이 apply X, Motion 이 boot-time snapshot_bundle 로 self-build)

### 7.3 새로 짜는 자리

- `modules/calibration/repository.py` — CalibrationRepository (§2 spec)
- `modules/calibration/module.py` — Module class (@service / @publishes / @subscriber decorators, Repository + ObjectStore constructor 주입)
- ~~`modules/calibration/alembic/`~~ → **루트 `backend_v2/alembic/`** (§8 정정 — migration 은 루트 단일)
- `modules/calibration/module.py` — Module class (@service / @publishes, Repository + ObjectStore constructor 주입)
- `tests/fixtures/mock_calibration_owner.py` — Step D Motion 검증 자리에서 이미 쓰이던 mock fixture 는 Step E 진입 후 real Calibration 로 대체

## 8. Migration = 루트 단일 Alembic (소유권 ≠ 마이그레이션 권위)

> **2026-07-02 정정 — 초안의 "Alembic per-module" 폐기.** 소유권과 마이그레이션 권위는 다른 문제다:
> - **테이블 / ORM / Repository 소유 = 모듈별** (calibration 이 `calibration_*` 소유, `modules/calibration/orm.py`). 옛 중앙 Storage *Module*(런타임 RPC 중개자) 폐기는 그대로 — 각 모듈이 자기 Repository 직접.
> - **마이그레이션 권위 = 루트 하나** (`backend_v2/alembic/`). calibration/scan/task/reconstruction 은 같은 프로세스의 모듈이지 독립 서비스가 아니고 DB 도 공유 인프라 → Database-per-**Service** 가 아님. per-module Alembic 은 version_table 충돌 / cross-module FK(reconstruction→scan) 순서 / 전체 초기화 복잡도만 들여옴.

**구현됨** (2026-07-02):

```
backend_v2/
    alembic.ini              # 루트, ASCII-only (cp949 트랩), path_separator=os
    alembic/
        env.py               # 모든 DB 모듈 orm import → 공유 Base.metadata (REGISTER 블록에 한 줄씩)
        versions/            # 단일 history
    infra/database/
        base.py              # 공유 Base(DeclarativeBase) — 모든 DB 모듈 ORM 상속
    modules/calibration/orm.py   # calibration_* 를 공유 Base 에 등록 (소유는 이 모듈)
```

- **공유 물리 DB** (sqlite 파일 하나 now, Postgres later) — 각 모듈은 자기 테이블만 소유.
- env.py 의 `_render_item` — `UtcDateTime` 은 migration 상 `sa.DateTime(timezone=True)` 로 스냅샷 (app TypeDecorator 결합 회피).
- runtime `upgrade head` = DB owner 모듈(또는 apps boot)이 `config.attributes["connection"]` 로 프로그래매틱 실행.
- role 격리 유지 — 루트 alembic 은 PC 전용 도구, Pi(motor/motion/camera)는 실행/ import 안 함.
- 검증: `tests/modules/test_alembic.py` (upgrade head == metadata, partial unique 강제, cross-check).

## 9. Motion consumer — 실 wire (boot-time)

Step E 진입 시 Step D 자리 mock owner (tests/fixtures/mock_calibration_owner.py) 제거 + 실 Calibration 로 e2e 검증. Step E 검증의 핵심은 **Bundle atomic snapshot + Motion 의 boot-time kinematics build** ([backend_v2_modules.md §11.2](backend_v2_modules.md) — Step E 목표는 Mirror e2e 가 아니라 config boot-query e2e 로 정정됨).

즉 시나리오:
1. offline `calibrate_offline.py --commit` → CalibrationResult INSERT + activate (backend 종료 상태에서 실행)
2. backend 재시작 → Motion.start() 가 `snapshot_bundle` service 1회 호출 → atomic 5 kind Bundle 받음
3. Motion 이 kinematics **build** — `link_offset` patched URDF 로 `PybulletKinematics` 생성, `SagCorrectedKinematics(joint_offset, sag_offset)` decorator wrap
4. 이후 fk/ik 가 이 kinematics 로 동작. 프로세스 동안 재빌드 없음 (config 불변)

**런타임 rebuild 시나리오는 없음** — 위 flow 는 재시작 사이에서만 일어남. Motion 살아있는 동안 Bundle 은 고정.

## 10. Decisions (옛 backend 실 자산 근거)

### 10.1 Factory intrinsic seed = Calibration.start() 가 Camera internal service pull ✅ 구현됨

> **구현·검증 완료 (2026-07-02)**: Camera `GET_FACTORY_INTRINSIC` internal service (FRONTEND_EXPOSED X) + `CalibrationModule.start()` 의 `_seed_factory_intrinsic` (async pull → idempotent seed). mock camera synthetic intrinsic 으로 boot test + Playwright e2e 에서 over-wire seed 확인 ("factory intrinsic seeded" 로그). 아래는 결정 근거.

**옛 자산 인사이트**: [backend/modules/camera/factory_intrinsic.py](../backend/modules/camera/factory_intrinsic.py) — Camera 가 pyrealsense2 pipeline 을 SDK internal 접근용으로 소유. Boot 시 자기 pipeline 잠깐 open → factory intrinsic fetch → Storage Module RPC commit. Idempotent (이미 active 있으면 skip).

**옛 자산이 왜 그렇게 짰나**: Camera 가 SDK 접근 owner (host: pi_camera 자리 자연). 저장은 Storage Module (모두의 데이터 hub) 이 owner. 두 module 협력 자리 Camera → Storage direct RPC write.

**v2 원칙 대조**:
- **Storage Module 폐기** — 이제 Calibration 이 intrinsic table owner (Database-per-Module + Owner/Reader 비대칭, [backend_v2.md §2.3](backend_v2.md))
- Owner/Reader 원칙 = "다른 module 이 owner 의 데이터를 write 하지 못함"
- 옛 방향 그대로 (Camera → Calibration service write call) 이월 = **Owner/Reader 위반**

**v2 결정 (2026-07-02 확정 — A/B/C 중 A 채택, "pull")**: Calibration 이 자기 start() 에서 Camera 의 read-only internal service 를 **pull** → 자기 domain rule (idempotent seed) 적용.

factory intrinsic 은 **static 값 + boot-time config** 라 event 스트림(방향 뒤집힌 publish)이 필요 없다. event 로 밀면 오히려 boot 순서 유실 문제(late subscriber 에게 Zenoh replay 안 함)를 스스로 만든다. pull 이 자연스럽다.

```python
# Camera side — CameraDriver Module (Step B 확장, host: pi_camera or pc mock)
class CameraDriverModule:
    @service("srv/camera/{robot_id}/get_factory_intrinsic")  # internal — FRONTEND_EXPOSED X
    def get_factory_intrinsic(self, req) -> FactoryIntrinsic | None:
        """요청 시 SDK 짧게 open 해서 factory intrinsic 반환. D405 아니면 None."""
        if isinstance(self._driver, RealSenseD405Driver):
            return self._driver.get_factory_intrinsic()
        return None

# Calibration side — 자기 start() 안에서 pull (event subscribe 아님)
async def start(self):
    intr = self.runtime.call("srv/camera/{robot_id}/get_factory_intrinsic", ...)
    if intr is not None:
        self._seed_intrinsic_if_missing(intr)

def _seed_intrinsic_if_missing(self, intr) -> None:
    """idempotent — 이미 active 있으면 skip. domain rule 은 Calibration 소유."""
    if self._repository.get_active(self.robot_id, "intrinsic"):
        return  # 사용자 chessboard 캘 결과 덮어쓰지 않음
    run_id = self._repository.create_run(self.robot_id, "intrinsic", "d405_factory")
    result_id = self._repository.save_result(run_id, intr)
    self._repository.finalize_run(run_id, "success")
    self._repository.activate_result(result_id)
```

**Ownership 유지**: Camera 는 자기 SDK 데이터를 read-only service 로 노출 (자기 domain). Calibration 이 caller 로 pull 후 자기 domain rule 로 seed (intrinsic table 소유). Write 방향 뒤집힘 없음. §7.7 anchor #13 (Camera public service 에 intrinsic 노출 금지) 은 **internal only (FRONTEND_EXPOSED X)** 로 회피. **옛 자산의 인사이트** (SDK 접근 owner ≠ 저장 owner, idempotent seed) 유지.

**A 를 고른 이유** (B/C reject): B (periodic republish) 는 static 값에 낭비. C (Mirror 응용) 는 factory intrinsic 을 runtime state 처럼 취급 — 원칙 2 위반 (intrinsic 은 config). pull 1회면 충분.

boot 순서: Calibration.start() 의 pull 이 성공하려면 Camera runtime 이 먼저 떠 있어야 함 — §3.6 lifecycle 순서 (Camera → Calibration → Motion). 분산 시 Camera runtime 지연이면 Calibration 이 retry/wait (framework lifecycle 처리).

### 10.2 Motion 의 kinematics = boot-time build (restart-only, v2 정석)

> **2026-07-02 방향 정정 — 07-01 판단 폐기.** 07-01 에는 이 자리를 "real-time reinitialize (Mirror 원칙 준수)" 로 적고 `PybulletKinematics.reinitialize()` 런타임 재로드를 넣었다. **그건 "Mirror 니까 실시간이어야 한다" 는 아키텍처적 연역이었지, 실제 요구사항이 아니었다.** 아래가 재판단 결과.

**옛 자산 인사이트**: [backend/modules/calibration/applier.py](../backend/modules/calibration/applier.py) — "부팅 시 1회 apply, link_offset 은 backend restart" 원칙. PyBullet API 상 URDF 재로드는 client disconnect+reconnect+reload 자리 복잡. 옛 backend 는 restart-only.

**왜 restart-only 가 위반이 아니라 정석인가** (원칙 2):
- calibration commit (`calibrate_offline.py --commit`) 은 **backend 종료 상태에서** 실행 (RDB lock 충돌, CLAUDE.md 명시) → Motion 살아있는 동안 Bundle 이 바뀌는 순간이 **아예 없음**.
- rollback 도 재시작 flow 로 통일 (§5 activated = "재시작 필요" 알림). → 런타임 rebuild 트리거가 존재하지 않음.
- Calibration Bundle 은 **Configuration** (설정 갈아끼우기), runtime state (상태 업데이트) 가 아님. URDF / DH parameter 를 바꾸면 시스템을 다시 초기화하는 게 자연스러운 것과 동형. MoveIt 도 URDF/SRDF/kinematics plugin 변경 시 노드 재로드 — 실시간 link 길이 변경을 전제하지 않음.

**v2 결정**: Motion 은 start() 에서 snapshot_bundle 을 읽어 kinematics 를 **1회 build**.
- `link_offset` — patched URDF 로 `PybulletKinematics` 생성 (§10.2 old 의 `reinitialize` 런타임 교체 **제거**)
- `joint_offset` / `sag_offset` — `SagCorrectedKinematics(...)` decorator 로 wrap (in-memory)
- 프로세스 동안 이 kinematics 고정. 새 Bundle 은 다음 부팅에서 fresh build.

**제거된 복잡성**: `_lock` 잡은 채 PyBullet client 교체 / in-flight fk/ik stall / TrajectoryRunner 캐시된 kinematics ref 재검증 — 전부 불필요 (런타임 교체 자체가 없으니 발생 안 함). 07-01 의 "미해결 자리 (_lock 스레드 자원 재검증)" 도 함께 소멸.

### 10.3 Preview 5Hz stream 부하 = 옛 자산 검증 완료, 이월

**옛 자산 인사이트**: [backend/nodes/application/calibration_node.py](../backend/nodes/application/calibration_node.py) 의 `preview_loop` 이미 실 hardware (D405 + SO-101) 자리 검증 완료 ([docs/calibration_workflow.md](calibration_workflow.md) + CLAUDE.md § "자동 BA + σ live (2026-06-10)" — capture 후 자동 preview / traffic light 자리 실 사용).

**v2 원칙 대조**: state stream 5Hz publish 는 [backend_v2.md §3.2](backend_v2.md) stream 원칙과 자연 정합. 별 원칙 상충 없음.

**v2 결정**: 5Hz 그대로 이월. ChArUco detect + traffic light + preview payload publish. 이건 옛 자산의 구현 detail 을 그대로 재사용해도 v2 원칙 위반 없음 자리 (드문 경우 — architectural 결정 아니고 실측 tuning 자리).

## 11. 구현 진행 (2026-07-02)

**✅ 완료 (회사 mock+sim 검증):**
1. ✅ `persistence/orm.py` + `contract.py` (4 entity + wire pydantic + Bundle)
2. ✅ `persistence/repository.py` (§2, advanced-alchemy caged)
3. ✅ 루트 `alembic/` (initial migration, upgrade==metadata + partial unique 검증)
4. ✅ `module.py` (@service 11 + @publishes 이벤트/preview stream + factory intrinsic pull seed)
5. ✅ `apps/registry.py` + `resolve.py` (session_factory/object_store/motor_ids 주입) + config rdb_uri/object_uri
6. ✅ tests — persistence(실 DB fixture) + alembic + module + capture(sim image) + preview + boot (168 PASS)
7. ✅ Camera `GET_FACTORY_INTRINSIC` internal service + mock camera sim board 모드 (§10.1)
8. ✅ `vision/{board,processing,capture_quality,thresholds,se3,sim_board}.py` 도메인 이월 + capture 서비스 (detect→PnP→gate→DB+blob)
9. ✅ contract_export FRONTEND_EXPOSED += calibration + `contract.ts` 재생성
10. ✅ frontend `CalibrationPanel`/`RobotCalibrateMode`/registry/sidebar/route + **Playwright headed e2e 4/4** (capture-success over-wire)

11. ✅ **offline BA 이월 완료 (2026-07-02, §11.1)** — `fk_chain.py`(modules/motion) + `depth_frame.py`(modules/camera) + `calibrate_offline.py` + `calibrate_squeeze.py`(scripts) + `physical.yaml`(robot_v2/<type>, sag_joint SSOT) + RobotConfig 로딩. **포팅 faithful 증명됨** (old==new bit-identical, §11.1). **177 PASS**, ruff/pyright clean, FK gate 35자세 일치.

**🔜 남음 (별도 slice):**
- **결정론적 auto outlier-selection (§11.2 — 다음 세션 핵심 anchor)**. offline BA 가 매 캘마다 사람 손질/운 없이 최적 캘을 뽑는 게 목표. 설계 논의 완료, 구현 대기.
- **Motion boot consumer** — Motion.start() 가 snapshot_bundle 읽어 kinematics build (§9). Motion 은 현재 calibration bundle 소비 안 함. 별도 slice.
- 실 D405 (mock synthetic intrinsic → 실 SDK) / 실 ChArUco 캡처 정확도 — 집 hardware.

### 11.1 offline BA 이월 — 완료 + 검증 결과 (2026-07-02)

**⚠️ 미래 세션 anchor — 아래 결론 다시 뒤집지 말 것 ("재현 안 된다" 삽질 반복 금지).**

**포팅 산출물** (faithful copy, BA 수학 = 순수 numpy/scipy/cv2):
- `modules/motion/fk_chain.py` — 해석적 FK 모델 (Motion·Calibration 공용, PybulletKinematics=실행기/FkChain=모델). sag torque 용 `fk_with_axes` 포함. **왜 별도 FK냐**: BA 가 link/joint/sag offset 을 변수로 least_squares 미분 → PyBullet(black-box) 불가.
- `modules/camera/depth_frame.py` — RGBD primary blob codec (Stage E depth residual 용).
- `scripts/calibrate_offline.py` — 5-stage BA(A~E) + IRLS(Huber) + sanity + LOOCV + commit. data-load 를 `CalibrationRepository` + `FilesystemObjectStore` 로 재배선.
- `scripts/calibrate_squeeze.py` — outlier greedy drop 진단 도구 (LOOCV plateau).
- `robot_v2/<type>/physical.yaml` + `apps/config.py` — `sag_joint_motor_ids` 타입레벨 SSOT (모델링 선택만, URDF 물리량 복제 금지).

**포팅 faithful 증명됨 (이게 진짜 gate — σ 숫자 매칭보다 강함)**: 옛 `backend/scripts/calibrate_offline.py` 를 같은 실 `horibot.db` 에 돌려 v2 포팅과 **bit-identical** 확인 (no-drop + drop 두 경로, Stage A~D reproj/σ/모든 파라미터 joint·link·sag·\|t\|). FK gate = FkChain.fk vs PybulletKinematics.fk 35자세 0.1mm/0.05° 일치. **BA 수학 손상 없이 이월.**

**σ 0.818°/7.538mm 는 bit-exact 재현 "안 됨" — 이건 port 버그 아님 (중요):**
- 그 committed 값(DB run 2, result id=2)은 **hand-tuned drop-9 subset** 으로 나온 것 ([handeye_sigma_floor_so101.md](handeye_sigma_floor_so101.md) 이 "25 cap drop 9" 로 기록 — 단 **어느 9개인지 인덱스는 미기록**, config 담았던 `cal_v3.json` 은 삭제됨).
- no-drop BA = 0.898°/9.546mm (old==new 동일). auto-squeeze greedy drop-8 = **J3 6.9°/J5 -5.1°/σ_t 8.26mm, |t| 117.7mm** → committed(J3 6.5°/J5 -5.3°/**|t| 117.88mm**) **바로 그 이웃**. 즉 committed 는 greedy 를 넘어 사람이 몇 개 더 고른 결과이고, 그 손질은 [handeye_sigma_floor_so101.md §6](handeye_sigma_floor_so101.md) 스스로 **anti-pattern("1일 6번 다시")** 이라 부른 그 날 삽질. 8.26 도 7.53 도 **둘 다 hardware floor(~7.5mm) 근방** — 0.7mm 차이는 floor 노이즈.
- **결론: 정확한 drop-9 인덱스가 기록에 없어 bit-exact 재현 불가. 하지만 포팅은 faithful 하고, 자동 경로가 committed 이웃으로 결정론적 수렴. 미래 세션은 이 숫자 재현 재시도 말 것.**

**reproducibility gap (실제 결함, flag)**: commit 시 `drop_poses` 를 result/run 에 안 박아 과거 캘 재현 불가. 향후 commit 경로에 drop set 영속화 필요.

### 11.2 결정론적 auto outlier-selection — 설계 (2026-07-02 논의, 구현 대기)

**문제**: 캘은 매번 하는데, 사람이 손으로 outlier 자세 골라내고 운 좋으면 잘 나오는 구조면 안 됨 (사용자에게 manual workaround 떠넘기기 = 원칙 위반). 매 캘마다 결정론적으로 최적 캘이 나와야 함.

**"최적"의 정확한 경계 (미래 세션 오해 금지)**:
- **주어진 자세 집합 → BA 해 = 전역 최적: 보장됨** (single-basin, globally identifiable — cv2 5-seed + MCMC 4-chain R̂ 1.0023 증명, [handeye_sigma_floor_so101.md §3.1~3.2]). solve 엔 운 0. **이게 offline 전환의 실이득** (런타임 = 대충 수렴 / offline = 수렴된 전역해 + LOOCV/IRLS 검증).
- **subset 선택(어느 outlier drop) → 전역 최적 보장 없음**: 2^N 이산 조합. greedy/수동 = 휴리스틱.
- **함정: "σ 최소 = 최적" 아님.** 자세 계속 버리면 σ→0 (overfit, 25→8장). Stage E reject 이유가 이것. **목적함수 = LOOCV(일반화) 여야 함, σ-min 아님.**
- **hardware floor**: STS3215 backlash ±0.87° → σ_R ~0.8° / 종이보드 σ_t ~7.5mm 가 상한. 알고리즘으로 못 뚫음. "절대 최적"의 천장은 hardware 가 정함.

**제안 (bounded 조합 탐색 — full C(34,9)≈5200만 불가, 15분~1시간 feasible)**:
- **A (pooled 전수, 추천)**: IRLS/RMS 로 "명백히 좋은" 자세 고정 + "의심" pool(10~12장) 안에서만 ≤K drop 조합 전수. `ΣC(10,≤7)≈968` → ~16분. **pool·K 한정 provably 최적** 보장 (사용자가 원하는 신뢰).
- **B (beam search)**: greedy 일반화 (beam width 5). 더 싸고 K 크게, 전역최적은 아님.
- **필수 조건**: 채점 = LOOCV(+cap K_max + sanity RED 배제, 과drop 방지). 탐색 중엔 싼 proxy, 최종 후보만 full LOOCV.
- **또는 (가장 단순) IRLS-only**: drop 자체를 없애고 Huber soft weight 만 → 완전 결정론적, 사람 개입 0, no-drop σ_t 9.5mm(floor 근방). "운 제거" 만 목표면 이게 제일 깔끔.

**미결 (구현 전 확정)**: 탐색 전략(A/B/IRLS-only) · 목적함수(LOOCV 확정) · 상한(K_max, pool 크기). + drop set 영속화(§11.1 gap).

## 12. Status (2026-07-02)

**해결됨**:
- ✅ **§10.1**: factory intrinsic = Calibration.start() 에서 Camera internal service **pull** (A 채택). boot 순서 유실 문제 소멸.
- ✅ **§10.2**: Motion kinematics = **boot-time build (restart-only)**. `PybulletKinematics.reinitialize()` 런타임 재로드 제거. `_lock` 스레드 자원 재검증 자리도 소멸.
- ✅ **§6**: Calibration Bundle = boot-time query (Mirror 제거). 원칙 2 (Configuration vs Runtime State) 확정.

**남은 자리 (framework 문서 반영 — 별도 단위)**:
- **backend_v2.md**: 원칙 2 (Configuration vs Runtime State) first-class 추가 + anchor #2 (`Mirror[CalibrationBundle] 단일`) supersede 표시. worked code 예제(Motion 이 Mirror[CalibrationBundle] 쓰는 부분)는 boot-query 패턴으로 전면 rewrite 필요 — 대공사, 별도 진행.
- **backend_v2_modules.md**: Step E 목표 "Mirror[Bundle] e2e" → "Runtime state 대표 모듈 Mirror e2e + calibration boot-query e2e" 로 일반화 (§11.2 표). Mirror consumer 로 calibration 쓰는 예제/표 rewrite.
- **§1-§5 재대조**: 원칙 2 관점에서 남은 산발 표현 점검 (이번 rewrite 로 대부분 정리됨).
