# Calibration (통합)

> **통합본 (2026-07-11 문서 다이어트)** — 아래 문서들을 원문 그대로 병합. 옛 파일명으로의
> 링크는 본 문서 내 해당 부(또는 git history). 각 부의 제목/상태 배너는 병합 당시 그대로.
> - `calibration.md`
> - `calibration.md`
> - `calibration.md`
> - `calibration.md`
> - `calibration.md`


---
---

<!-- ═══════════ [통합 원문] calibration.md ═══════════ -->

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
> + 루트 `backend/alembic/` + `infra/database/{base,types,boot}.py` + registry/resolve/config/mock·pc.yaml + `apps/contract_export.py` 노출 + Camera `GET_FACTORY_INTRINSIC` + mock camera sim board + frontend `CalibrationPanel`/`RobotCalibrateMode`/registry/sidebar/route + `contract.ts` 재생성.
>
> **검증**: backend **168 PASS** (persistence 실 horibot.db fixture + known σ 0.818°/7.538mm / alembic upgrade==metadata + partial unique / module @service + 이벤트 / **capture sim-image e2e** detect→PnP→gate→DB+blob / preview 5Hz / boot Zenoh 도달 + **§10.1 factory intrinsic auto-seed over-wire**), ruff/pyright clean. frontend tsc/eslint clean. **Playwright headed 4/4 PASS** (실 브라우저→vite→bridge WS→서비스→DB): WS연결/패널·bundle / preview toggle / start_run→history / **preview 검출(green)+capture accepted** (sim board→intrinsic→CameraDecoded→PnP→DB row+ `.calib_blobs_mock/.../000_color.jpg` 디스크 기록 확인).
>
> **offline BA 이월 완료 (2026-07-02, §11.1)** — fk_chain/depth_frame/calibrate_offline/calibrate_squeeze/physical.yaml 포팅, **포팅 faithful 증명됨 (old==new bit-identical)**, 177 PASS. ⚠️ committed σ 0.818°/7.538mm 은 미기록 hand-tuned drop-9 subset 결과라 bit-exact 재현 불가 — **port 버그 아님, 재현 재시도 금지** (§11.1). 다음 = **결정론적 auto outlier-selection 설계 §11.2** (매 캘 운 제거).
>
> **집(실물)에서 남은 것**: 실 D405 factory intrinsic (mock synthetic 대체) / 실 ChArUco 캡처 정확도 / **Motion boot consumer** (Motion.start() 가 snapshot_bundle 읽어 kinematics build — §9, 별도 slice 아직 미배선).
>
> 핵심 설계 결정: Calibration Bundle = boot-time configuration (Mirror 안 씀). §6 / §10.1 / §10.2 재작성 완료. SSOT spec = [backend.md](backend.md) (§16 Module catalog 포함 — 옛 backend_modules.md 통합).
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

**invariant**: 같은 `(robot_id, kind)` 의 `is_active=True` result 는 **최대 1개** (partial UNIQUE index — [dev_reference.md §2.3](dev_reference.md)).

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
| `backend/scripts/calibrate_offline.py` | 5 stage BA + LOOCV + IRLS + observability | `backend/scripts/calibrate_offline.py` | 이월 (backend 프로세스 밖 offline 도구 — v2 도 동일 위치) |

### 7.2 폐기 자리 정리

- `storage_client.py` — Storage Module RPC → 직접 Repository (Database-per-Module)
- `loader.py` — Repository.get_active_bundle 로 흡수 (별도 loader 자리 불필요)
- `calibration_cache.py` — Mirror 가 cache 자체 (별도 in-memory 자리 불필요)
- `applier.py` — 책임 이관 (Calibration 이 apply X, Motion 이 boot-time snapshot_bundle 로 self-build)

### 7.3 새로 짜는 자리

- `modules/calibration/repository.py` — CalibrationRepository (§2 spec)
- `modules/calibration/module.py` — Module class (@service / @publishes / @subscriber decorators, Repository + ObjectStore constructor 주입)
- ~~`modules/calibration/alembic/`~~ → **루트 `backend/alembic/`** (§8 정정 — migration 은 루트 단일)
- `modules/calibration/module.py` — Module class (@service / @publishes, Repository + ObjectStore constructor 주입)
- `tests/fixtures/mock_calibration_owner.py` — Step D Motion 검증 자리에서 이미 쓰이던 mock fixture 는 Step E 진입 후 real Calibration 로 대체

## 8. Migration = 루트 단일 Alembic (소유권 ≠ 마이그레이션 권위)

> **2026-07-02 정정 — 초안의 "Alembic per-module" 폐기.** 소유권과 마이그레이션 권위는 다른 문제다:
> - **테이블 / ORM / Repository 소유 = 모듈별** (calibration 이 `calibration_*` 소유, `modules/calibration/orm.py`). 옛 중앙 Storage *Module*(런타임 RPC 중개자) 폐기는 그대로 — 각 모듈이 자기 Repository 직접.
> - **마이그레이션 권위 = 루트 하나** (`backend/alembic/`). calibration/scan/task/reconstruction 은 같은 프로세스의 모듈이지 독립 서비스가 아니고 DB 도 공유 인프라 → Database-per-**Service** 가 아님. per-module Alembic 은 version_table 충돌 / cross-module FK(reconstruction→scan) 순서 / 전체 초기화 복잡도만 들여옴.

**구현됨** (2026-07-02):

```
backend/
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

Step E 진입 시 Step D 자리 mock owner (tests/fixtures/mock_calibration_owner.py) 제거 + 실 Calibration 로 e2e 검증. Step E 검증의 핵심은 **Bundle atomic snapshot + Motion 의 boot-time kinematics build** (Step E 목표는 Mirror e2e 가 아니라 config boot-query e2e 로 정정됨 — [backend.md §16.3](backend.md)).

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
- **Storage Module 폐기** — 이제 Calibration 이 intrinsic table owner (Database-per-Module + Owner/Reader 비대칭, [backend.md §2.3](backend.md))
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

**옛 자산 인사이트**: [backend/nodes/application/calibration_node.py](../backend/nodes/application/calibration_node.py) 의 `preview_loop` 이미 실 hardware (D405 + SO-101) 자리 검증 완료 ([docs/calibration.md](calibration.md) + CLAUDE.md § "자동 BA + σ live (2026-06-10)" — capture 후 자동 preview / traffic light 자리 실 사용).

**v2 원칙 대조**: state stream 5Hz publish 는 [backend.md §3.2](backend.md) stream 원칙과 자연 정합. 별 원칙 상충 없음.

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

11. ✅ **offline BA 이월 완료 (2026-07-02, §11.1)** — `fk_chain.py`(modules/motion) + `depth_frame.py`(modules/camera) + `calibrate_offline.py` + `calibrate_squeeze.py`(scripts) + `physical.yaml`(robot/<type>, sag_joint SSOT) + RobotConfig 로딩. **포팅 faithful 증명됨** (old==new bit-identical, §11.1). **177 PASS**, ruff/pyright clean, FK gate 35자세 일치.

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
- `robot/<type>/physical.yaml` + `apps/config.py` — `sag_joint_motor_ids` 타입레벨 SSOT (모델링 선택만, URDF 물리량 복제 금지).

**포팅 faithful 증명됨 (이게 진짜 gate — σ 숫자 매칭보다 강함)**: 옛 `backend/scripts/calibrate_offline.py` 를 같은 실 `horibot.db` 에 돌려 v2 포팅과 **bit-identical** 확인 (no-drop + drop 두 경로, Stage A~D reproj/σ/모든 파라미터 joint·link·sag·\|t\|). FK gate = FkChain.fk vs PybulletKinematics.fk 35자세 0.1mm/0.05° 일치. **BA 수학 손상 없이 이월.**

**σ 0.818°/7.538mm 는 bit-exact 재현 "안 됨" — 이건 port 버그 아님 (중요):**
- 그 committed 값(DB run 2, result id=2)은 **hand-tuned drop-9 subset** 으로 나온 것 ([calibration.md](calibration.md) 이 "25 cap drop 9" 로 기록 — 단 **어느 9개인지 인덱스는 미기록**, config 담았던 `cal_v3.json` 은 삭제됨).
- no-drop BA = 0.898°/9.546mm (old==new 동일). auto-squeeze greedy drop-8 = **J3 6.9°/J5 -5.1°/σ_t 8.26mm, |t| 117.7mm** → committed(J3 6.5°/J5 -5.3°/**|t| 117.88mm**) **바로 그 이웃**. 즉 committed 는 greedy 를 넘어 사람이 몇 개 더 고른 결과이고, 그 손질은 [calibration.md §6](calibration.md) 스스로 **anti-pattern("1일 6번 다시")** 이라 부른 그 날 삽질. 8.26 도 7.53 도 **둘 다 hardware floor(~7.5mm) 근방** — 0.7mm 차이는 floor 노이즈.
- **결론: 정확한 drop-9 인덱스가 기록에 없어 bit-exact 재현 불가. 하지만 포팅은 faithful 하고, 자동 경로가 committed 이웃으로 결정론적 수렴. 미래 세션은 이 숫자 재현 재시도 말 것.**

**reproducibility gap (실제 결함, flag)**: commit 시 `drop_poses` 를 result/run 에 안 박아 과거 캘 재현 불가. 향후 commit 경로에 drop set 영속화 필요.

### 11.2 결정론적 auto outlier-selection — 설계 (2026-07-02 논의, 구현 대기)

**문제**: 캘은 매번 하는데, 사람이 손으로 outlier 자세 골라내고 운 좋으면 잘 나오는 구조면 안 됨 (사용자에게 manual workaround 떠넘기기 = 원칙 위반). 매 캘마다 결정론적으로 최적 캘이 나와야 함.

**"최적"의 정확한 경계 (미래 세션 오해 금지)**:
- **주어진 자세 집합 → BA 해 = 전역 최적: 보장됨** (single-basin, globally identifiable — cv2 5-seed + MCMC 4-chain R̂ 1.0023 증명, [calibration.md §3.1~3.2]). solve 엔 운 0. **이게 offline 전환의 실이득** (런타임 = 대충 수렴 / offline = 수렴된 전역해 + LOOCV/IRLS 검증).
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
- **backend.md**: 원칙 2 (Configuration vs Runtime State) first-class 추가 + anchor #2 (`Mirror[CalibrationBundle] 단일`) supersede 표시. worked code 예제(Motion 이 Mirror[CalibrationBundle] 쓰는 부분)는 boot-query 패턴으로 전면 rewrite 필요 — 대공사, 별도 진행.
- ~~backend_modules.md rewrite~~ — 반영 후 2026-07-03 [backend.md](backend.md) §16 으로 통합·폐기 (Mirror deferred + boot-query = §16.3 / anchor #2).
- **§1-§5 재대조**: 원칙 2 관점에서 남은 산발 표현 점검 (이번 rewrite 로 대부분 정리됨).


---
---

<!-- ═══════════ [통합 원문] calibration.md ═══════════ -->

# Calibration Workflow

> ⚠️ **부분 stale (2026-07-11 문서 전수 감사)**: §1 in-UI Compute/COMMIT 절차는 폐기된 흐름 — 현행 = capture-only 세션(내부캘/핸드아이, abort 포함) + `scripts/calibrate_offline.py` offline 분석 + DB(runs/results) rollback. §2 자세 다양성·§5 ChArUco board spec 은 현행 유효.
> 본 감사에서 삭제된 v1 문서 참조가 남아있을 수 있음 — git history 에서 복원 가능.

캘리브레이션 페이지의 Hand-Eye 탭을 사용하는 절차와 결과 해석 가이드. **무엇이 어떻게 적용되는가**는 [calibration_apply_flow.md](calibration_apply_flow.md), **BA 자유도/알고리즘**은 [calibration.md](calibration.md) 참조.

---

## 1. Capture → Compute → Commit 절차

좌측 카메라 피드 위에 라이브 체커보드 코너 오버레이가 자동 표시되어 자세 평가가 실시간으로 됨.

1. (필요 시) **Capture 카드 [리셋]** — 누적 포즈 비움 (백엔드 재시작 불필요).
2. 자세 잡기 (Move TCP / 토크 OFF 후 수동). 라이브 오버레이가 초록색이면 검출 OK.
3. **[캡처]** — 프레임 캡처 + 체커보드 검출 + PnP + 포즈 추가. 검출 실패면 사유 표시되고 포즈 미추가.
4. 8~10자세 반복 (자세 다양성 가이드 ↓).
5. (자동) **σ live** — 매 [캡처] 끝에서 backend 가 자동 BA → `CALIB_HANDEYE_SIGMA` topic publish → frontend Hand-Eye 패널 상단 σ badge 자동 갱신. 사용자 별도 [COMPUTE] 안 눌러도 즉시 결과 확인. (BA 모드 변경 등 수동 [COMPUTE] 는 admin 기능, 일반 사용자 불필요)
6. 결과 해석 (§ 결과 해석 가이드). outlier 포즈는 Capture 리스트의 휴지통(`#<id>` 클릭)으로 삭제 후 다시 COMPUTE — Pose ID는 안정 ID라 삭제해도 인덱스 시프트 없음.
7. 만족스러우면 **Commit 카드 [COMMIT]** — `hand_eye.npz` + (BA 모드에 따라) `joint_offsets.npz` / `link_offsets.npz` / `sag_offsets.npz` 저장.
8. (선택) **Validate 카드** — 저장된 .npz 또는 최근 COMPUTE 결과로 T_target←base 흩어짐 σ_rot/σ_t 측정.

### COMMIT 후 재시작 필요 여부

[calibration_apply_flow.md § 0](calibration_apply_flow.md) 표 참조. 요약:

| 산출물       | 즉시 반영 | 재시작 필요               |
| ------------ | --------- | ------------------------- |
| hand_eye     | O         | DetectorNode 재시작 필요  |
| joint_offset | O         | 불필요                    |
| link_offset  | O (mem)   | **백엔드 재시작 필요** (PyBullet URDF는 부팅 시 1회 로드) |
| sag_offset   | O         | 불필요                    |

---

## 2. 자세 다양성 가이드

5DOF 한계 안에서 최대한 다양하게:

- **joint 1 base yaw** — 좌우 회전 (월드 yaw)
- **joint 4 wrist pitch** — 위아래 끄덕임
- **joint 5 wrist roll** — 비틀기
- 셋을 골고루 섞기. 한 축만 위주로 돌리면 TSAI 회전 추정이 부정확.
- 체커보드는 화면 중앙 가깝게. **tilt 30~70° 범위 안에서만 [캡처 가능]** ([backend/modules/calibration/thresholds.py](../backend/modules/calibration/thresholds.py) `TILT_MIN_DEG / TILT_MAX_DEG` SSOT). tilt<30° = 너무 정면 (PnP depth ambiguous), tilt>70° = edge-on (corner 픽셀 정확도 ↓).
- 매 자세 캡처 직전 로봇 완전 정지 (모터 명령 전송 후 ~0.5s 대기).

---

## 3. 결과 해석 가이드

COMPUTE / Validate 결과를 보고 어떤 조치를 취할지 판단하는 룰. 색 임계값은 [HandEyeResults.tsx](../frontend/src/components/calibration/HandEyeResults.tsx)에 박혀 있음.

### 색 임계값

| 항목                       | 의미                                                                                                                                           | 초록 (좋음)  | 노랑 (경계)   | 빨강 (나쁨)   |
| -------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------- | ------------ | ------------- | ------------- |
| **σ_rot**                  | T_target←base 회전 분산. 캡처한 모든 포즈에서 본 체커보드를 base 프레임으로 환산했을 때 얼마나 흩어지나. 체커보드는 안 움직였으니 이상적이면 0 | <0.5°        | <1.5°         | ≥1.5°         |
| **σ_t**                    | 위 위치 버전 (mm)                                                                                                                              | <5           | <15           | ≥15           |
| **PARK / DANIILIDIS Δrot** | TSAI 대비 다른 알고리즘 결과의 차이. 같은 입력을 세 가지 다른 수학으로 풀어서 합의 정도 → 입력 self-consistency 척도                           | <1°          | <3°           | ≥3°           |
| **per-pose drot / dt**     | 각 포즈가 평균(또는 첫 포즈) 대비 벗어난 양. outlier 식별                                                                                      | <0.5° / <5mm | <1.5° / <15mm | ≥1.5° / ≥15mm |

### 진단 룰 — 읽는 순서: PARK Δrot → per-pose → σ

1. **PARK Δrot 노랑/빨강** (≥1°) → 알고리즘 자체 문제 아니라 **입력 포즈에 outlier가 섞여 있음**. PARK이 TSAI보다 outlier에 민감해 가장 먼저 빨강이 됨. per-pose 표에서 빨강 행 식별 → 삭제(#`id`) → 재 COMPUTE.
2. **PARK ≤1°인데 σ_rot 빨강** (≥1.5°) → outlier는 정리됐지만 **시스템 전반 오차**. 자세 다양성 부족 가능 → joint 1/4/5 분포 점검 후 추가 캡처. 그래도 안 떨어지면 [calibration.md](calibration.md) 참조해서 BA mode 변경 (`extended` / `physical_sag`).
3. **σ_rot 초록 (<0.5°) + σ_t 초록 (<5mm)** → 캘 품질 충분. COMMIT. TSDF/ICP에 사용 OK.

### 액션 플레이북

| 상황                                | 조치                                                                      |
| ----------------------------------- | ------------------------------------------------------------------------- |
| per-pose에 빨강 1~3개 (나머진 깨끗) | 빨강 포즈 삭제 → COMPUTE 재실행                                           |
| per-pose에 빨강/노랑이 절반 이상    | 캡처 절차 문제 (로봇 정지 안 함 / 체커보드 가림 / 비스듬). 리셋 후 재캡처 |
| PARK 노랑, σ_rot 경계               | 자세 다양성 부족 가능 → joint 1/4/5 분포 점검 후 추가 캡처                |
| 모든 게 깨끗한데 σ_rot ~ 1° 정체    | BA mode `standard` → `extended` → `physical_sag` 단계적으로 시도          |
| Validate σ가 Compute σ보다 큼       | 정상 (Validate는 평균 대비 흩어짐, Compute는 첫 포즈 대비)                |

> **TSDF 목표치**: σ_rot < 1° / σ_t < 10mm. 현재 달성치는 BA `physical_sag`로 σ_rot **0.65°** / σ_t **7.94mm** ([calibration.md § 16](calibration.md)).

---

## 4. Intrinsic

[robot/calibration/intrinsic.npz](../robot/calibration/intrinsic.npz) — D405 color 1280x720, **factory seed 기반** (`seed_d405_intrinsic_if_missing`이 카메라 노드 기동 시 채움).

- camera_matrix: fx=649.75, fy=648.10, cx=632.67, cy=359.60
- dist_coeffs: [-0.0525, 0.0596, -0.000246, 0.000545, -0.0198]
- rms_error=0.0 — 재캘리브 잔차가 아니라 factory seed라서 0.

D405의 color stream 공장 캘리브는 일반적으로 정확하므로 별도 재캘리브는 보류. UI에서 Intrinsic 탭으로 재캘 가능하지만 현재 권장하지 않음.

---

## 5. Calibration Board (ChArUco)

Hand-Eye / Intrinsic 캡처에 사용하는 ChArUco 보드.

### Pattern spec (OpenCV 입력)

```yaml
# calib.io PDF generator 입력
Pattern:        ChArUco
Rows:           5
Columns:        7
Square Length:  25 mm
Marker Length:  18 mm
Dictionary:     DICT_4X4_50   # 17 markers 만 쓰지만 50 ID dict 로 충분
Start Id:       0
```

OpenCV `CharucoBoard.size` 컨벤션은 **`(squaresX=Columns, squaresY=Rows)`** — 즉 PDF 의 (Rows=5, Columns=7) → OpenCV `size=(7, 5)`. [board.py:35-36](../backend/modules/calibration/board.py#L35-L36) 의 `SQUARES_X=7 / SQUARES_Y=5` 가 그것. (2026-06-10: 초기 코드는 swap 되어 있었고 marker 는 검출되는데 ChArUco corner 매핑이 fail 하던 hardware 검증 시 정정.)

내부 코너 = (7-1) × (5-1) = **24개** / pose. 마커 17개. modern pattern (`setLegacyPattern(False)`).

### 물리적 사양

- 보드 외곽: 200 × 150 mm (패턴 175 × 125 mm + 여백)
- 재료: 포맥스 5T (PVC foam, white)
- 표면: PP 유포지 + 무광코팅 (수분/조명 반사 무관)
- 모서리: 라운드 처리 (안전, 캘 영향 0)

선정 근거: OMX 5DOF 자유도 제약 + 책상 55×34cm 환경 ([hardware.md § 작업대](hardware.md)) 에서 "작은 보드 + 다양한 pose" 가 "큰 보드 + 적은 pose" 보다 유리. 6×8 (35 코너) 도 후보였으나 OMX 도달 영역 위주로 5×7 선택. SO-101 (6DOF) 도 같은 보드 공용 가능 (D405 stays 가정 — OMX swap 후 USB UVC 시나리오는 [hardware.md § 카메라](hardware.md) 참조).

### 재제작 정보

- **PDF 생성**: calib.io Pattern Generator
  - Target Type: ChArUco
  - Board Width 200 / Height 150
  - Rows 5 / Columns 7 / Checker Width 25
  - Dictionary DICT_4X4 / Start Id 0
- **합지**: 출력스토리 견적 의뢰 → 포맥스 5T 무광. 참고 단가 16,280원 + 배송비 (2026-06)
- **견적 의뢰 시 명시**: "카메라 캘리브레이션용, square 25mm 치수 정확도 중요" (자동 fit-to-page 방지)

### 사용 시 주의

- **PDF 설계치(25mm) ≠ 실측치 가능** — 합지 시 인쇄 스케일 ±1% 오차 흔함
- 받은 보드는 캘리퍼스로 square 실측 → **실측치를 OpenCV `squareLength` 에 입력** (PDF 설계치 X)
- 실측 결과 (2026-06-10, 벌니어 캘리퍼스): **25 mm** — 벌니어 정밀도 (±0.05mm) 내 PDF 설계치 일치. [board.py:37](../backend/modules/calibration/board.py#L37) `SQUARE_LENGTH_M = 0.025` 그대로 유지. (square 0.05mm 오차 → 작업거리 250mm 환산 0.5mm pose error, σ_t <10mm 목표 대비 무시 가능)


---
---

<!-- ═══════════ [통합 원문] calibration.md ═══════════ -->

# Hand-Eye σ Floor 진단 — SO-101 + D405 (2026-06-21)

> SO-101 6DOF + Intel RealSense D405 eye-in-hand 의 σ floor 진단 종합. **현재 best**
> effective σ_R 0.801° / σ_t 7.53mm (DB run_id=2, id=6 active, 25 cap drop 9).
> 사용자 목표 0.5° / 5mm 를 못 뚫는 이유 + 가능한 다음 step.

## 1. 핵심 결론

| 항목 | 결과 |
|---|---|
| Algorithmic floor | **σ_R 0.801° / σ_t 7.53mm** (effective σ) |
| Hardware floor | STS3215 backlash ±0.87° → σ_R 0.5° 목표 **impossible** |
| Algorithm 으로 짜낼 여지 | **없음** (5 axis cross-check + Stage E reject) |
| σ_t 5mm 진입 가능 path | Hardware fix (알루미늄 보드 가장 가성비) |

## 2. σ Dual Metric ([[project-calibration-sigma-dual-metric]])

| Metric | 정의 | 우리 best |
|---|---|---|
| **effective σ** ([`measure_effective_sigma`](../backend/scripts/calibrate_offline.py)) | BA fit 적용 후 모든 capture 의 board_in_base 의 std. *accuracy* (commit 결정 기준) | **σ_R 0.801° / σ_t 7.53mm** |
| **Jacobian σ** ([`run_ba_stage`](../backend/scripts/calibrate_offline.py)) | `(JᵀJ)⁻¹·σ²` 의 handeye block trace. *parameter confidence* (BA solver 의 self-reported uncertainty) | σ_R 5.20° / σ_t 7.33mm |

DB schema 자리 두 metric 분리 컬럼:
- `calibration_results.sigma_rot` / `sigma_t` — Jacobian σ
- `calibration_results.effective_sigma_rot` / `effective_sigma_t` — effective σ

[commit_results](../backend/scripts/calibrate_offline.py) 가 hand_eye row INSERT 시 둘 다 박음.

## 3. 진단 시도 (2026-06-21)

### 3.1 cv2 PARK seed BA (`calibrate_validate_opencv.py` + `calibrate_cv2_seed.py` — 2026-06-21 제거, git history 에서 복원 가능)

cv2 4 method (TSAI/PARK/HORAUD/DANIILIDIS) 가 자기들끼리 매우 tight cluster (|t|=85-87mm, ΔR<1°/Δt<2mm). 우리 BA stage-A 는 |t|=91.58mm, 9°/22mm 벗어남 (44% outlier) — 처음엔 *BA bug 의심* 했음.

근데 stage D + drop 9 적용한 BA 를 cv2 PARK seed vs 우리 TSAI seed × IRLS on/off 4 config 비교:

| config | \|t\| | effective σ_R | effective σ_t | outlier |
|---|---|---|---|---|
| D_park_irls_off | 89.82mm | — | — | 14/25 |
| D_park_irls_on | 89.11mm | 0.801° | 7.53mm | 8/25 |
| D_ours_irls_off | 89.82mm | — | — | 14/25 |
| D_ours_irls_on | 89.11mm | 0.801° | 7.53mm | 8/25 |

→ **seed 무관 같은 basin 수렴**. joint/link/sag 추정치도 100% 일치. **BA globally identifiable, single basin**. stage-A 의 9°/22mm 벗어남은 bug 가 아니라 6 DOF 가 joint/link offset 흡수 못해서 생기는 정상 거리.

### 3.2 Bayesian MCMC NUTS 4 chain (`calibrate_mcmc.py` — 2026-06-21 제거, git history 에서 복원 가능)

NumPyro + jaxlie. cv2 5 method 각각의 결과로 4 chain dispersed init.

- Wall time 33.2s (4 chain, 200 warmup + 500 samples)
- **R̂ = 1.0023 < 1.01 threshold**
- chain mean handeye t (mm) 4 chain 다 같음: `[-66.98, -5.36, -57.72]`
- chain mean handeye R euler (deg) 4 chain 다 같음: `[+65.06, +3.34, +87.75]`
- Posterior σ_R 0.034° / σ_t 0.16mm (한 데이터셋 내 credible width)

→ **UNIMODAL 결정적 확정**. LM wrong basin 가설 reject. cv2_seed + MCMC 둘 다 globally identifiable 확인.

### 3.3 Stage E (depth-augmented) 재시도

cal_v3.json 의 Stage E (full 28 cap, drop 6) train 3.79px, LOOCV 9.74, σ_R 4.36°/σ_t 6.15mm — LOOCV/train 2.57× RED 만 보고 reject 했지만 LOOCV 절대값은 D 와 같음. drop 9 적용 안 했음.

**Stage E + drop 9 + default HUBER 시도**:

| Stage | reproj | LOOCV | Jacobian σ_R / σ_t | **Effective σ_R / σ_t** |
|---|---|---|---|---|
| D (commit) | 4.50px | 8.03 | 5.20° / 7.33mm | **0.801° / 7.53mm** |
| **E (depth)** | **3.04px** | 8.11 | **3.50° / 4.92mm** | **0.828° / 7.62mm** |

Jacobian σ 는 떨어졌으나 **effective σ 사실상 동일** (E 가 약간 더 나쁨). LOOCV/train 2.67× RED 가 정확히 잡은 *parameter overfitting*. depth 가 BA solver confidence 만 ↑, data consistency 개선 X. **Stage E reject 확정**.

### 3.4 5 axis cross-check ([`calibration.md`](calibration.md))

| Axis | 결과 |
|---|---|
| 1. Community benchmark | LeRobot SO-101 + STS3215 setup 에 0.5°/5mm 달성 reported 사례 **0건**. STS3215 backlash **±0.87°** (실측) — 우리 σ_R 0.801° 와 같은 차수 → 0.5° **hardware impossible** |
| 2. Observability | `observability_params.py` 가 실제로는 없음 (CLAUDE.md 의 spec 과 다름). 측정 인프라 구축 필요 — 시도 안 함 |
| 3. PnP rms | 25 cap mean 0.176px, max 0.381px — *sub-pixel*. corner detection noise 자리 floor 아님 |
| 4. Stage E rehab | 위 3.3 — reject |
| 5. 보드 거리 / corner pixel | 25 cap mean 보드 z=22.7cm, 25mm square → 71.5 pixel. tilt mean 36° (30-70° 권장 19/25). 모두 정상 |

### 3.5 Kalib neural hand-eye (NO-GO)

[Kalib](https://github.com/robotflow-initiative/Kalib) (arXiv 2408.10562):
- **6 DOF only** (handeye 만). 우리 11+ DOF (handeye + joint + link + sag) 보다 *구조적으로 weaker model*
- SpaTracker continuous video 필수 — 우리 34 sparse cap 안 됨, 새 video 캡처 필요
- Reported "0.5°/3mm" 은 simulation only (RFUniverse, Franka, perfect URDF). real-world metric 은 mask IoU, σ 비교 불가
- Windows 비호환 (Ubuntu + CUDA 11.8 + Python 3.10 + 22GB VRAM)
- Best case σ_t 3mm 달성 확률 **5%**, realistic σ_t 8-15mm (우리 7.53 보다 나쁨) **55%**

## 4. BA Degeneracy — joint_offset vs link_offset Trade-off

현재 fit 의 의심 항목:
- **J3 offset = +6.57° + J3 link [-6.43, -4.69, 0] mm** — 둘 다 크고 OUTLIER_RATE_RED 초과
- J5 offset = -5.28° 도 비슷

수학적으로 *joint_offset(J3) Δθ* 와 *link_offset(J3) 의 R/t 6 DOF* 가 **동일한 EE 위치를 만들 수 있음** (frame chain 의 다른 자리에서 같은 효과). BA 의 prior 강도 (joint 1°, link_r 0.2°, link_t 1mm) 가 어느 쪽으로 흡수할지 결정. 둘 중 진짜 mechanical origin 이 무엇인지 BA 결과만으론 판별 불가.

**중요 — 적용 메커니즘**:

| 산출물 | 적용 자리 |
|---|---|
| `joint_offset` | [`JointCoordinates.motor_to_urdf`](../backend/core/coords/joint_coordinates.py) raw↔rad 변환 양쪽 가산 |
| `link_offset` | [`PybulletKinematics.apply_link_offsets`](../backend/modules/kinematics/adapters/pybullet_kinematics.py) → in-memory URDF patch → tempfile → `loadURDF`. **모든 fk/ik 가 patched URDF 사용** |
| `sag_k` | [`SagCorrectedKinematics`](../backend/modules/kinematics/adapters/sag_corrected.py) Decorator 양방향 |

즉 BA 가 J3 = 6.57° + J3 link [-6, -4]mm 으로 *분산 fit* 한 결과가 둘 다 robot motion 에 적용 중. **수학적으로 동등한 EE 위치 만들지만 어느 쪽이 진짜인지는 BA 가 모름**.

### Caliper 검증 protocol (5분 work)

1. J3 motor 회전축에서 J4 motor 회전축까지 caliper 측정 (직선거리 + orientation)
2. URDF 의 J3→J4 link origin 값과 비교
3. 판정:
   - 측정값 = URDF + ~7mm → **link offset 진짜**, joint_offset 6.57° 는 BA artifact
   - 측정값 = URDF (차이 거의 없음) → **servo zero offset 진짜**, link_offset 은 BA artifact

caliper 결과의 가치:
- *추가 URDF patch X* (이미 BA + LinkCoordinates 가 적용 중)
- *진단 용도* — 어느 mechanism 이 진짜인지 가름
- 다음 step:
  - 진짜 link → 그대로 (BA 가 이미 처리)
  - 진짜 servo zero → J3 motor manual home 다시 + 캘 재실행 (link_offset 가짜 fit 안 들어감 → 깔끔)

## 5. 남은 Hardware Fix 옵션

| 옵션 | 사용자 work | 코드 work | 예상 σ 개선 |
|---|---|---|---|
| **1. 알루미늄/아크릴 ChArUco 보드** ($15) | 새 보드 + 캡처 한 번 | 0 | **σ_t 1-3mm ↓** (가성비 ★★★) |
| 2. STS3215 backlash CW/CCW characterization | 새 캡처 50-120장 + 새 protocol | 1-2주 (directional joint_offset BA 추가) | σ_R 0.3° ↓ |
| 3. 3D-print link caliper 측정 + URDF default 수정 | 측정 1시간 | 1-2시간 (URDF patch) | σ minor |
| 4. 목표 조정 | — | — | 현 σ 가 SO-101+D405 tier 정상 영역 (LRBO2/PLOS 논문 0.16°-1°) — 받아들이기 |

## 6. 시도된 path summary (anti-pattern)

- 옵션 cycling 의 위험성 — 사용자가 1 일에 6번 "다시" 외치며 4 agent + 5 axis + Stage E + MCMC 다 던지고 결국 *hardware floor* 확정. 다음 캘 trauma 발생 시 본 문서 anchor 로 시작하면 같은 옵션 또 검토 안 해도 됨.
- Algorithmic optimum 확정에는 BA single-basin 증명 (cv2 multi-seed + MCMC 4 chain R̂) 이 충분 — 추가 algorithm 시도 전에 본 진단 먼저 돌릴 것.

## 관련 문서

- [calibration.md](calibration.md) — 확장 BA + 물리 sag (OMX 시대 진단, σ floor 1.5°→0.65°/7.94mm)
- [handeye_robust_irls_plan.md](handeye_robust_irls_plan.md) — IRLS+Huber plan
- [handeye_ux_solver_v3_plan.md](handeye_ux_solver_v3_plan.md) — Hand-Eye UX + Solver v3
- [calibration.md](calibration.md) — 캡처 절차
- [calibration_apply_flow.md](calibration_apply_flow.md) — 4종 산출물 적용 메커니즘


---
---

<!-- ═══════════ [통합 원문] calibration.md ═══════════ -->

# Hand-Eye 확장 BA — 원리와 코드

> σ_rot 1.5° / σ_t 17mm floor → 확장 BA(link offset, §1~§14)로 1.30°/9.3mm →
> 물리 sag 모델(§16)로 **0.65°/7.94mm**까지 내림. 수식 최소, **실제 코드 스니펫 + 줄별 설명** 중심.
>
> **2026-06-15 업데이트** — 본 문서 내 `write_patched_urdf` / `.patched/` 디렉토리 / "디스크에 patched URDF 저장" 언급은 *historical*. storage_node 도입 후 in-memory `patch_urdf_text` + tempfile 1회성 패턴으로 교체됨 ([storage_layer.md §13](storage_layer.md)). BA 의 link_offset 추정/적용 의미는 동일 (URDF 의 `<joint><origin>` 에 delta 가산), 적용 메커니즘만 다름.

---

## 1. 무엇이 문제였나

OMX_F의 Hand-Eye 캘리브레이션 결과가 **σ_rot ≈ 1.5° / σ_t ≈ 17mm**에서
정체. 자세 32개까지 캡처해도 안 떨어짐.

캘 σ가 의미하는 건 _"체커보드는 실제로 한 위치에 있는데, 캘 결과로 자세
마다 예측한 체커보드 위치가 얼마나 흩어지나"_. σ가 작을수록 모든 자세에서
일관된 EE 위치를 잡는다는 뜻 → detector pick&place, TSDF 정밀도에 직결.

TSDF/ICP 깔끔하게 돌리려면 σ_rot < 1° / σ_t < 10mm 필요.
**floor가 모델 한계라는 게 의심스러웠다.**

---

## 2. 진단 — 코드로 어떻게 알아냈나

기존 BA는 [bundle_adjust.py:81](../backend/modules/calibration/bundle_adjust.py)
의 `bundle_adjust_hand_eye()` — **11자유도** (joint_offset 5 + R/t 6).

11자유도가 진짜 한계인지 확인하려면 *같은 데이터*에 모드 4가지를 돌려
σ 비교 진단 실시.

핵심 부분 (4가지 시나리오 호출):

```python
# baseline=0 (디스크 offset 무시) — angles_zero
# baseline=현재 commit — angles_current (= angles_zero + JointCoordinates._offsets)
def run(label, angles, R_seed, t_seed, estimate):
    ba = bundle_adjust_hand_eye(
        joint_angles_per_pose=angles,
        R_target2cam=R_tc_list, t_target2cam=t_tc_list,
        X_init=(R_seed, t_seed), fk_fn=fk,
        estimate_joint_offsets=estimate,   # ← 핵심: 11자유도 ↔ 6자유도
    )
    sigma_rot = float(np.sqrt(np.mean(ba.residual_rot_deg**2)))
    sigma_t   = float(np.sqrt(np.mean(ba.residual_t_mm**2)))
    print(f"[{label}] σ_rot={sigma_rot} σ_t={sigma_t} offset={...}")

run("(1) est=True ", angles_zero,    R_seed_zero, t_seed_zero, True)   # 11 DOF
run("(2) est=False", angles_zero,    R_seed_zero, t_seed_zero, False)  # 6 DOF
run("(3) est=True ", angles_current, R_seed_cur,  t_seed_cur,  True)
run("(4) est=False", angles_current, R_seed_cur,  t_seed_cur,  False)
```

결과 표:

| 시나리오                   | σ_rot     | σ_t        | 의미                        |
| -------------------------- | --------- | ---------- | --------------------------- |
| (1) baseline=0, est=ON     | 2.05°     | 19.7mm     | joint_offset 흡수 효과 있음 |
| (2) baseline=0, est=OFF    | 3.45°     | 24.9mm     | 아무 보정 없는 raw 한계     |
| (3) baseline=현재, est=ON  | **1.50°** | **16.9mm** | 한 라운드 commit 후 floor   |
| (4) baseline=현재, est=OFF | 1.50°     | 17.1mm     | (3)과 같음                  |

결정적 두 줄:

- (1) vs (2): joint_offset이 진짜 systematic 흡수 (3.45→2.05, 1.4° 차이 = 진짜 효과)
- **(3) ≈ (4)**: 현재 baseline에서는 est ON/OFF가 같음 → **joint_offset 자유도가 이미 소진**

→ 알고리즘 문제 아니라 **모델 자유도 부족**.

---

## 3. 진짜 원인은 URDF의 link 기하학

1차 commit 결과를 보면 J2/J3 offset이 **+5.75° / +3.67°로 같은 방향, 비슷한
크기**. horn 오차라면 모터마다 독립이라 _같은 방향으로 함께 어긋날 일이 거의 없음_.
이건 다른 원인의 signature:

- **URDF link 길이 미스매치** — 3D프린트 부품 실측 vs URDF 수치 불일치
- **link frame 기울기** — 조립 시 약간 비스듬, URDF는 rpy="0 0 0" 가정
- **중력 처짐** — XL430이 11V(정격 하한)에서 동작, joint 2/3 토크 크면 sag

이 셋은 *joint 회전축*이 아니라 _link 본체의 transform_ 오차. joint_offset
하나당 1자유도는 "모든 자세에 일정한 보정"인데, link 오차는 자세에 따라
EE 위치에 다르게 영향 → 11자유도가 그걸 어거지로 흡수하다 limit 도달.

---

## 4. 해결 — link offset을 BA 변수로

URDF의 각 joint origin은:

```xml
<joint name="joint2" type="revolute">
  <origin rpy="0 0 0" xyz="0 0 0.0635"/>   <!-- 이 두 값을 변수화 -->
  <axis xyz="0 1 0"/>
  ...
</joint>
```

`xyz` 3개 + `rpy` 3개 = joint마다 6자유도 추가. 5 joint × 6 = **30 자유도 추가** → 총 **41자유도**.

### 4a. numpy FK chain — PyBullet 우회

PyBullet은 URDF 로드 후 transform 변경 불가. 근데 BA는 link_offset을
*변수로 매 iteration마다 다른 값으로 평가*해야 함.

[fk_chain.py](../backend/modules/kinematics/fk_chain.py) — URDF chain을
numpy 행렬 곱으로 직접 구현:

```python
# URDF에서 추출한 상수 (motor id 1~5와 일치)
JOINT_ORIGINS = np.array([
    [-0.01125, 0.0, 0.034],     # joint1 (link0→link1)
    [0.0, 0.0, 0.0635],          # joint2
    [0.0415, 0.0, 0.11315],      # joint3
    [0.162, 0.0, 0.0],            # joint4
    [0.0287, 0.0, 0.0],           # joint5
])
JOINT_AXES = np.array([
    [0, 0, 1],  [0, 1, 0], [0, 1, 0], [0, 1, 0], [1, 0, 0],
])
EE_ORIGIN = np.array([0.09193, -0.0016, 0.0])  # link5→ee fixed


def fk_chain(joint_angles, link_trans=None, link_rot=None):
    """link_trans/link_rot이 BA 변수로 들어가는 entry point."""
    T = np.eye(4)
    for i in range(5):
        # (1) joint i의 origin transform — URDF base + BA delta
        T_o = np.eye(4)
        T_o[:3, :3] = rotvec_to_R(link_rot[i])    # ← BA가 푸는 회전 보정
        T_o[:3, 3]  = JOINT_ORIGINS[i] + link_trans[i]  # ← 위치 보정
        T = T @ T_o
        # (2) joint i 회전 (revolute axis만큼)
        T_r = np.eye(4)
        T_r[:3, :3] = axis_angle_to_R(JOINT_AXES[i], joint_angles[i])
        T = T @ T_r
    # (3) fixed tcp_joint
    T_ee = np.eye(4); T_ee[:3, 3] = EE_ORIGIN
    Tee = T @ T_ee
    return Tee[:3, :3], Tee[:3, 3]
```

`link_trans=None / link_rot=None`이면 zero로 처리 → URDF 원본 그대로 FK.
BA에서는 `link_trans/link_rot`이 매번 다른 변수 값으로 들어감.

### 4b. 확장 BA — bundle_adjust_hand_eye_extended

[bundle_adjust.py](../backend/modules/calibration/bundle_adjust.py)에 신규 추가.
변수 layout:

```python
# 변수 layout (총 41):
#   [0:5]    joint_offset (rad)
#   [5:20]   link_translation (5×3, m)   ← 신규
#   [20:35]  link_rotation (5×3, rad)    ← 신규
#   [35:38]  rod (cam2gripper)
#   [38:41]  t (cam2gripper, m)

def unpack(x):
    return (
        x[:5],                        # joint_offset
        x[5:20].reshape(5, 3),         # link_translation
        x[20:35].reshape(5, 3),        # link_rotation
        x[35:38],                      # rod
        x[38:41],                      # t
    )
```

핵심 함수 — *체커보드는 한 위치*라는 제약을 잔차로 표현:

```python
def compute_T_target_in_base(x):
    """현재 변수 값으로 모든 포즈의 체커보드 위치 계산."""
    offset, link_t, link_r, rod, t_x = unpack(x)
    R_x = cv2.Rodrigues(rod)[0]
    T_x = make_T(R_x, t_x)          # T_cam2gripper (hand-eye)
    out = []
    for i in range(N):
        # joint angle에 offset 더한 후 FK (link 변형 반영)
        R_gb, t_gb = fk_chain(angles_arr[i] + offset, link_t, link_r)
        T_gb = make_T(R_gb, t_gb)    # T_gripper2base
        # T_target2base = T_gb @ T_cam2gripper @ T_target2cam (PnP 결과)
        out.append(T_gb @ T_x @ T_tc_list[i])
    return out


def residual(x):
    """모든 포즈의 T_target2base가 *평균*과 얼마나 다른지 = 흩어짐."""
    offset, link_t, link_r, _, _ = unpack(x)
    T_list = compute_T_target_in_base(x)
    positions = np.array([T[:3, 3] for T in T_list])
    mean_pos = positions.mean(axis=0)                       # 모든 포즈의 평균 위치
    mean_R   = _mean_rotation([T[:3,:3] for T in T_list])    # SVD chordal mean

    res = np.empty(6 * N + n_off + n_lt + n_lr)
    for i, T in enumerate(T_list):
        # 회전 편차 (axis-angle 형태)
        R_dev = T[:3,:3] @ mean_R.T
        rod_dev, _ = cv2.Rodrigues(R_dev)
        res[6*i : 6*i+3]   = rod_dev.flatten()              # 잔차[0:3]
        # 위치 편차
        res[6*i+3 : 6*(i+1)] = T[:3, 3] - mean_pos          # 잔차[3:6]

    # regularization 잔차 (다음 섹션)
    res[6*N : 6*N + n_off]                   = joint_offset_reg * offset
    res[6*N + n_off : 6*N + n_off + n_lt]    = link_trans_reg  * link_t.flatten()
    res[6*N + n_off + n_lt :]                = link_rot_reg    * link_r.flatten()
    return res

# scipy LM이 잔차 norm 최소화로 x를 푼다
result = least_squares(residual, x0, method="lm", ...)
```

**왜 mean 기준 잔차?** 체커보드의 "진짜 위치"를 변수로 두면 X(hand-eye)와
T_b(보드 위치)가 곱 형태로 entwine돼서 BA가 잘못된 minimum에 빠짐(gauge
freedom). 매 iter에서 *현재 추정의 평균*을 진짜 위치로 가정하면 그 자유도가
사라지고 LM이 안정적으로 수렴. 이게 hand_eye.py 주석에 적힌 'mean-based BA'.

결과 — 같은 32포즈에서:

|              | σ_rot     | σ_t       |
| ------------ | --------- | --------- |
| 11자유도     | 1.50°     | 16.9mm    |
| **41자유도** | **1.30°** | **9.3mm** |

σ_t가 거의 절반. TSDF GOOD threshold(10mm) 진입.

---

## 5. Gauge freedom — 왜 regularization이 필요한가

자유도 늘릴 때 위험: **link 길이 줄이고 hand-eye t 늘리면 같은 EE 위치**가
나옴. BA가 어느 값이 맞는지 못 정하고 어느 쪽으로든 흘러감.

증거 — regularization 없이 풀었더니:

```
joint2 link_translation dx = -60.97mm    ← 원본 link 길이 113mm의 절반!
joint2 joint_offset    = +22.83°          ← 비정상적으로 큼
σ_rot = 1.40°, σ_t = 9.36mm               ← fit은 좋음
```

fit은 좋은데 _값 자체는 의미 없음_. 다른 자세에 generalize 안 함.

해결 — 잔차에 _penalty 항_ 추가. 변수가 작은 값에 머물도록.

```python
# bundle_adjust.py — residual() 끝부분
res[6*N : 6*N + n_off]                = joint_offset_reg * offset       # weight=0.5
res[6*N + n_off : 6*N + n_off + n_lt] = link_trans_reg  * link_t        # weight=1.0
res[6*N + n_off + n_lt :]             = link_rot_reg    * link_r        # weight=1.0
```

`least_squares`는 잔차의 합을 최소화 → 이 항이 크면 그 변수도 작게 유지하려 함.
**weight 의미:** `link_trans_reg=1.0`이면 link_t가 0.01m(=10mm)일 때 잔차에
0.01 기여 → 데이터 잔차(보통 ~0.01 m) 비교해서 같은 수준. 즉 _10mm 부근에서
중립_. 그보다 큰 값을 쓰려면 데이터 fit이 추가로 그만큼 좋아져야 함.

weight 튜닝 실험:

| `link_trans_reg` | 결과                                                    |
| ---------------- | ------------------------------------------------------- |
| 10 (너무 강)     | link 모두 ≈0, BA가 joint_offset에 다시 흡수 (J2 +14.4°) |
| 5                | link ±3mm 정도, σ_t 14.9mm                              |
| 1                | link ±15mm, σ_t **9.3mm** ← sweet spot                  |
| 0 (없음)         | link 60mm 폭주, σ_t 9.4mm지만 의미 없음                 |

---

## 6. URDF patch — 변경 결과를 production에 어떻게 적용하나

BA가 풀어준 link_offset을 production code (motion/detector/task)에도 반영해야
함. 이들은 `Kinematics`로 FK/IK를 푸는데 PyBullet은 URDF 로드 후 변경 불가.

해결: **URDF 텍스트를 patch한 파일을 따로 만들고 PyBullet에 그걸 로드**.

[urdf_patcher.py](../backend/core/coords/urdf_patcher.py) 핵심:

```python
def patch_urdf_text(source_urdf_path, offsets, joint_id_map=None):
    """원본 URDF를 읽어 link_offsets patch한 텍스트 반환."""
    tree = ET.parse(str(source_urdf_path))
    root = tree.getroot()

    # (1) mesh 상대경로 → 절대경로 (patched URDF가 다른 폴더로 가니까)
    urdf_dir = src.parent.resolve()
    for mesh_el in root.iter("mesh"):
        filename = mesh_el.get("filename")
        if filename and not filename.startswith(("package://","file://","/")):
            abs_path = (urdf_dir / filename).resolve()
            mesh_el.set("filename", str(abs_path).replace("\\", "/"))

    # (2) joint origin patch
    for joint_el in root.findall("joint"):
        name = joint_el.get("name")
        if name not in joint_id_map: continue            # joint1~joint5만
        jid = joint_id_map[name]
        origin_el = joint_el.find("origin")

        d_trans = offsets.get_trans(jid)                  # 예: J2 [-0.02861, 0.00041, 0]
        d_rot   = offsets.get_rot(jid)                    # 예: J2 [-0.0108, 0.0035, 0]

        xyz = _parse_xyz(origin_el.get("xyz", "0 0 0"))
        rpy = _parse_xyz(origin_el.get("rpy", "0 0 0"))
        origin_el.set("xyz", _fmt_xyz(xyz + d_trans))     # 원본 + delta
        origin_el.set("rpy", _fmt_xyz(rpy + d_rot))

    return ET.tostring(root, encoding="unicode")
```

`(1)` mesh 절대경로화가 _중요_ — patched URDF가 `.patched/omx_f.urdf`에 저장되는데,
mesh가 원본의 `../../meshes/...`라면 상대 위치가 어긋나 PyBullet이 mesh 못 찾음.

`(2)` `xyz + d_trans`는 그냥 가산. `rpy + d_rot`는 _small-angle 가정_. URDF rpy는
ZYX 오일러 (`R = Rz·Ry·Rx`), `d_rot`는 BA의 rotation vector. 다른 표현이지만
각이 작으면 (<5°) 차이 무시 가능 (실제 v3 결과 최대 0.85°). 정확한 변환이
필요해지면 별도 함수.

저장 — [urdf_patcher.py](../backend/core/coords/urdf_patcher.py)의 `write_patched_urdf`:

```python
def write_patched_urdf(source_urdf_path, offsets, ...):
    src = Path(source_urdf_path)
    out = src.parent / ".patched" / src.name   # robot/urdf/omx_f/.patched/omx_f.urdf
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(patch_urdf_text(src, offsets), encoding="utf-8")
    return out
```

`.patched/`는 [.gitignore](../.gitignore)에 추가 → push 안 됨. 머신마다 자체 생성.

---

## 7. 백엔드 통합 — HandEyeCalibration 분기

`bundle_adjust_hand_eye_extended()`를 만들어둬도 호출돼야 의미가 있다.
[hand_eye.py](../backend/modules/calibration/hand_eye.py)의 `compute_with_diagnostics()`
가 mode 따라 기존 BA / 확장 BA로 분기.

### 7a. import 추가 + 헬퍼 함수

```python
# hand_eye.py 맨 위
from .bundle_adjust import (
    BundleAdjustExtendedResult,           # ← 확장 BA 결과 타입
    BundleAdjustResult,                    # 기존
    FkFn,
    bundle_adjust_hand_eye,
    bundle_adjust_hand_eye_extended,       # ← 확장 BA 함수
)
```

기존 `_run_ba_lists()` / `_multiseed_ba_lists()` 패턴 그대로 확장 버전 추가:

```python
@staticmethod
def _run_ba_extended_lists(*, ja_list, R_tc_list, t_tc_list, seed):
    """확장 BA 한 번 실행 — fk_fn 인자 없음 (내부에서 numpy fk_chain 호출)."""
    try:
        return bundle_adjust_hand_eye_extended(
            joint_angles_per_pose=[list(a) for a in ja_list],
            R_target2cam=R_tc_list,
            t_target2cam=[np.asarray(t).reshape(3) for t in t_tc_list],
            X_init=(seed.R_cam2gripper, seed.t_cam2gripper),
        )
    except Exception as e:
        logger.exception("확장 BA 실패: %s", e)
        return None


def _multiseed_ba_extended_lists(self, *, ja_list, R_gb_list, t_gb_list,
                                  R_tc_list, t_tc_list):
    """TSAI/PARK/DANIILIDIS 3 seed로 확장 BA 실행, cost 최소 채택."""
    best_ba, best_seed_name = None, None
    for method in _COMPARE_METHODS:
        R, t = cv2.calibrateHandEye(R_gb_list, t_gb_list, R_tc_list, t_tc_list,
                                     method=method)
        seed = HandEyeResult(R_cam2gripper=R, t_cam2gripper=t,
                              method=_METHOD_NAMES[method])
        ba = self._run_ba_extended_lists(ja_list=ja_list, R_tc_list=R_tc_list,
                                          t_tc_list=t_tc_list, seed=seed)
        if ba is None or not ba.success: continue
        if best_ba is None or ba.cost < best_ba.cost:
            best_ba, best_seed_name = ba, _METHOD_NAMES[method]
    return best_ba, best_seed_name
```

cv2 seed 3개로 돌리는 이유 — BA가 nonlinear라 seed 따라 다른 local minimum.
cost 최소를 채택하면 robust.

### 7b. compute_with_diagnostics 분기

기존 메서드에 `use_extended_ba` 인자 추가:

```python
def compute_with_diagnostics(self, *, fk_fn, arm_motor_cfgs, joint_limits_rad,
                              estimate_joint_offsets=True,
                              use_extended_ba=False):   # ← 신규
    """
    use_extended_ba=True면 확장 BA(41 DOF) 사용.
    fk_fn 대신 fk_chain.fk_chain 내부 호출.
    """
    ...
```

BA 호출 분기:

```python
# ── 2. 1차 BA (multiseed) — outlier 식별용 ────────────────
ba_first: BundleAdjustResult | BundleAdjustExtendedResult | None
if use_extended_ba:
    ba_first, ba_first_seed = self._multiseed_ba_extended_lists(
        ja_list=ja_list, R_gb_list=R_gb_list, t_gb_list=t_gb_list,
        R_tc_list=R_tc_list, t_tc_list=t_tc_list,
    )
else:
    ba_first, ba_first_seed = self._multiseed_ba_lists(
        ja_list=ja_list, R_gb_list=R_gb_list, t_gb_list=t_gb_list,
        R_tc_list=R_tc_list, t_tc_list=t_tc_list,
        fk_fn=fk_fn, estimate_joint_offsets=estimate_joint_offsets,
    )
```

`Union 타입`을 쓰는 이유: 두 BA 결과 공통 인터페이스(residual_rot_deg,
residual_t_mm, R_cam2gripper, t_cam2gripper, joint_offset_rad)는 동일.
**link_trans_m, link_rot_rad는 BundleAdjustExtendedResult만 가짐** → isinstance로 분기.

### 7c. 결과 처리 (joint_offset + link_offset 추출)

outlier 자동 제거 후 ba_final 결과에서 변수 추출:

```python
# ── 5. 최종 X / 잔차 / σ 결정 ────────────────────────────
joint_offset_rad      = np.zeros(len(arm_motor_ids))
joint_offsets_estimated = False
link_trans_delta = np.zeros((5, 3))
link_rot_delta   = np.zeros((5, 3))
link_offsets_estimated = False

if ba_final is not None and ba_final.success:
    final_R = ba_final.R_cam2gripper
    final_t = ba_final.t_cam2gripper.reshape(3, 1)

    # method_name 분기 (UI 표시용)
    if isinstance(ba_final, BundleAdjustExtendedResult):
        method_name = f"BA(+offset+link, seed={ba_final_seed})"
    elif ba_final.n_joint_vars > 0:
        method_name = f"BA(+offset, seed={ba_final_seed})"
    else:
        method_name = f"BA(seed={ba_final_seed})"

    # 변수 추출 분기
    if isinstance(ba_final, BundleAdjustExtendedResult):
        joint_offset_rad = ba_final.joint_offset_rad.copy()
        joint_offsets_estimated = True
        link_trans_delta = ba_final.link_trans_m.copy()        # ← 확장 BA만
        link_rot_delta   = ba_final.link_rot_rad.copy()         # ← 확장 BA만
        link_offsets_estimated = True
    elif ba_final.n_joint_vars > 0:
        joint_offset_rad = ba_final.joint_offset_rad.copy()
        joint_offsets_estimated = True
```

### 7d. 응답 dict — Frontend에 link offset 전달

```python
n_link = min(5, len(arm_motor_ids))
link_trans_list = [
    {
        "motor_id": int(arm_motor_ids[i]),
        "x_mm": float(link_trans_delta[i, 0] * 1000.0),
        "y_mm": float(link_trans_delta[i, 1] * 1000.0),
        "z_mm": float(link_trans_delta[i, 2] * 1000.0),
        "x_m":  float(link_trans_delta[i, 0]),                  # 정밀 저장용
        "y_m":  float(link_trans_delta[i, 1]),
        "z_m":  float(link_trans_delta[i, 2]),
    }
    for i in range(n_link)
]
# link_rot도 비슷한 dict 리스트 (rx_deg/rx_rad 둘 다)
...

return {
    ...                                          # 기존 필드
    "joint_offset_estimated": joint_offsets_estimated,
    "joint_offset_delta": joint_offset_list,
    "link_offset_estimated":  link_offsets_estimated,    # ← 신규
    "link_trans_delta":       link_trans_list,            # ← 신규
    "link_rot_delta":         link_rot_list,              # ← 신규
    ...
}
```

`mm`과 `m` 둘 다 보내는 이유: UI는 mm으로 표시(사람 친화), commit 시
정밀 저장은 m(np.float64 손실 없음).

---

## 8. 백엔드 통합 — CalibrationNode 핸들러

[calibration_node.py](../backend/nodes/application/calibration_node.py)는 Zenoh 서비스
핸들러를 들고 있다. compute / commit 둘 다 수정.

### 8a. import + commit 핸들러에서 LinkCoordinates 사용

```python
# calibration_node.py 맨 위
from core.joint_coordinates import JointCoordinates
from core.link_coordinates import LinkCoordinates                  # ← 신규
from modules.calibration.link_offsets import LinkOffsets           # ← 신규
```

### 8b. compute 핸들러 — mode 인자 + use_extended_ba 전달

```python
def _srv_handeye_compute(self, req: dict) -> dict:
    arm_motor_ids = [cfg.id for cfg in self._arm_cfgs]
    joint_limits = self.solver.joint_limits(len(arm_motor_ids))

    # mode: "extended" (기본) / "standard" (회귀 진단용 fallback)
    mode = str(req.get("mode", "extended")).lower()
    use_extended_ba = mode != "standard"

    diag = self.hand_eye.compute_with_diagnostics(
        fk_fn=self.solver.fk_to_matrix,
        arm_motor_cfgs=self._arm_cfgs,
        joint_limits_rad=joint_limits,
        use_extended_ba=use_extended_ba,                  # ← 신규 인자
    )
    ...
```

기본을 `"extended"`로 둔 이유 — validation으로 generalize 확인됐고,
σ 모든 면에서 더 좋음. Frontend는 mode 인자 안 보내면 자동 extended.

### 8c. commit 핸들러 — joint_offsets + link_offsets 둘 다 누적 저장

```python
def _srv_handeye_commit(self, req: dict) -> dict:
    ...
    # 1) hand_eye.npz — 카메라↔그리퍼 외부 보정
    self.hand_eye.save(hand_eye_path)

    # 2) joint_offsets.npz — 기존 패턴 그대로 (cumulative 합산)
    if self._last_compute.get("joint_offset_estimated"):
        delta_by_id = {int(e["motor_id"]): float(e["offset_rad"])
                       for e in self._last_compute["joint_offset_delta"]}
        applied = JointCoordinates().commit_offsets(delta_by_id,
                                                     method=self.hand_eye.result.method)

    # 3) link_offsets.npz — 신규 (cumulative 합산)
    link_msg = ""
    restart_required = False
    if self._last_compute.get("link_offset_estimated"):
        trans_list = self._last_compute["link_trans_delta"]
        rot_list   = self._last_compute["link_rot_delta"]
        # 응답 dict → LinkOffsets dataclass 변환
        delta = LinkOffsets(
            trans={int(e["motor_id"]): np.array([e["x_m"], e["y_m"], e["z_m"]])
                   for e in trans_list},
            rot={int(e["motor_id"]): np.array([e["rx_rad"], e["ry_rad"], e["rz_rad"]])
                 for e in rot_list},
        )
        # 디스크 누적 + PC 메모리 갱신
        link_applied = LinkCoordinates().commit_offsets(delta,
                                                         method=self.hand_eye.result.method)
        restart_required = True
        link_msg = f" + link_offsets 갱신 (n={len(link_applied.trans)})"

    return {
        "success": True,
        "message": f"저장 완료{offset_msg}{link_msg}",
        "data": {
            "joint_offsets_applied": ...,
            "link_offsets_applied":  link_offsets_estimated,
            "link_offsets":          link_applied_meta,
            "restart_required":      restart_required,      # ← UI에 표시
        },
    }
```

**`restart_required: true`가 중요** — `Kinematics`는 URDF를 부팅 시 1회만
로드하므로 commit 후 메모리 자동 갱신 X. 다음 부팅에 적용. UI가 사용자에게
"백엔드 재시작 필요" 알림.

---

## 9. 프론트엔드 통합 — 타입 + 결과 UI

[frontend/src/components/calibration/](../frontend/src/components/calibration/)
의 types.ts + HandEyeResults.tsx 수정.

### 9a. 타입 추가 — types.ts

```typescript
/** link translation 보정. URDF <joint><origin xyz/>에 더할 dx,dy,dz. */
export type LinkTransDelta = {
  motor_id: number;
  x_mm: number;
  y_mm: number;
  z_mm: number; // UI 표시용
  x_m: number;
  y_m: number;
  z_m: number; // commit 정밀 저장용
};

/** link rotation 보정 (small-angle 가정으로 rpy ≈ rotvec). */
export type LinkRotDelta = {
  motor_id: number;
  rx_deg: number;
  ry_deg: number;
  rz_deg: number;
  rx_rad: number;
  ry_rad: number;
  rz_rad: number;
};
```

기존 `ComputeData` 타입에 필드 추가:

```typescript
export type ComputeData = {
  ...                                              // 기존 필드
  joint_offset_estimated: boolean;
  joint_offset_delta: JointOffsetDelta[];
  // 확장 BA에서만 채워짐. standard fallback이면 false + 빈 배열.
  link_offset_estimated: boolean;                 // ← 신규
  link_trans_delta: LinkTransDelta[];              // ← 신규
  link_rot_delta:   LinkRotDelta[];                // ← 신규
  recommendations: NextPoseRecommendation[];
};
```

### 9b. 결과 테이블 — HandEyeResults.tsx

기존 `JointOffsetTable` 패턴 따라 두 컴포넌트 추가:

```typescript
/** link translation. |값| > 20mm면 gauge freedom 의심 — 노랑. */
function linkTransColor(mm: number): string {
  const mag = Math.abs(mm);
  if (mag < 5)  return "text-muted-foreground";
  if (mag < 20) return "text-foreground";
  return "text-amber-500";          // 의심 시 사용자에게 시각적 경고
}

function fmtSigned(v: number, frac: number): string {
  return (v >= 0 ? "+" : "") + v.toFixed(frac);
}

function LinkTransTable({ rows }: { rows: LinkTransDelta[] }) {
  return (
    <div>
      <p className="text-[10px] text-muted-foreground font-mono mb-1">
        link translation delta (mm) — joint origin xyz 보정, COMMIT 시 누적
      </p>
      <table className="w-full text-[11px] font-mono">
        <tbody>
          {rows.map((r) => (
            <tr key={r.motor_id}>
              <td className="py-0.5 text-muted-foreground">J{r.motor_id}</td>
              <td className={`py-0.5 text-right ${linkTransColor(r.x_mm)}`}>
                x {fmtSigned(r.x_mm, 2)}
              </td>
              <td className={`py-0.5 text-right ${linkTransColor(r.y_mm)}`}>
                y {fmtSigned(r.y_mm, 2)}
              </td>
              <td className={`py-0.5 text-right ${linkTransColor(r.z_mm)}`}>
                z {fmtSigned(r.z_mm, 2)}
              </td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
}

// LinkRotTable도 비슷 — 임계 0.5°/2.0°
```

색 임계의 의미:

- `<5mm`: 회색(정상 — 가산해도 시스템 무영향)
- `<20mm`: 흰색(주의 — 확인 필요)
- `≥20mm`: 노랑(gauge freedom 의심 — 진짜 link 미스매치인지 한 번 더 검토)

`ComputePreview`에서 렌더링:

```typescript
{data.joint_offset_estimated && data.joint_offset_delta.length > 0 && (
  <JointOffsetTable rows={data.joint_offset_delta} />
)}
{/* ↓ 신규 — 확장 BA일 때만 보임 */}
{data.link_offset_estimated && data.link_trans_delta.length > 0 && (
  <LinkTransTable rows={data.link_trans_delta} />
)}
{data.link_offset_estimated && data.link_rot_delta.length > 0 && (
  <LinkRotTable rows={data.link_rot_delta} />
)}
```

`link_offset_estimated`가 false면 (standard fallback) 자동으로 안 보임 →
기존 UI 회귀 없음.

---

## 10. 부팅 시 흐름 — LinkCoordinates + Kinematics

`link_offsets.npz`(디스크) → 메모리 캐시 → patched URDF → PyBullet 로드.

### 10a. LinkCoordinates (JointCoordinates 패턴 그대로)

[link_coordinates.py](../backend/core/coords/link_coordinates.py) — 싱글톤:

```python
LINK_OFFSETS_PATH = Path(__file__).parents[2] / "robot" / "calibration" / "link_offsets.npz"

class LinkCoordinates:
    _instance = None
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized: return
        self._initialized = True
        self._offsets: LinkOffsets = link_offsets_io.load(LINK_OFFSETS_PATH)  # 부팅 시 1회 로드

    def snapshot(self) -> LinkOffsets:
        with self._cache_lock:
            return LinkOffsets(trans=dict(self._offsets.trans), rot=dict(self._offsets.rot))

    def commit_absolute(self, offsets, method):
        """COMMIT 시 atomic 갱신: 디스크 *overwrite* + 메모리 reload."""
        link_offsets_io.save(LINK_OFFSETS_PATH, offsets, method=method)
        with self._cache_lock:
            self._offsets = LinkOffsets(trans=dict(offsets.trans), rot=dict(offsets.rot))
        return self.snapshot()
```

> **2026-06-10 정정**: 이 위 § 8c 의 "cumulative" 표현은 stale.
> `commit_absolute` 4종 (joint/link/sag/tool) 모두 overwrite contract. caller (calibration_node) 가 (현재 disk + BA delta) 를 absolute 로 변환한 후 한 번에 덮어쓴다.
> joint 만 옛 `commit_offsets(delta)` cumulative 였던 시절은 [calibration_ux_rewrite.md §6](calibration_ux_rewrite.md) 의 Bug A (last_compute stale double-add) 노출 자리 → API 통일로 제거.
> 진짜 수렴 신호는 "다음 라운드 BA 가 추정하는 *delta* 가 0 에 가까움" — 이건 그대로 (BA math 내부).

### 10b. Kinematics 수정

[solver.py:30~](../backend/modules/kinematics/registry.py) — 부팅 시 patched URDF
생성하고 그걸 로드:

```python
class Kinematics:
    def __init__(self):
        if self._initialized: return
        self._initialized = True

        # ← 신규: 디스크 link_offsets → patched URDF 생성
        link_offsets = LinkCoordinates().snapshot()
        urdf_to_load = write_patched_urdf(URDF_PATH, link_offsets)
        if not link_offsets.is_empty():
            logger.info(f"patched URDF 로드: {urdf_to_load}")

        self._client = p.connect(p.DIRECT)
        self._robot = p.loadURDF(str(urdf_to_load), useFixedBase=True, ...)
        # ↑ 원본 URDF_PATH 아니라 patched 경로
```

`link_offsets`가 비어있어도 `write_patched_urdf`는 호출됨 — 그러면 mesh 절대화만
적용된 URDF가 `.patched/`에 생성, joint origin은 원본 그대로. 즉 link_offsets
없을 때도 정상 동작.

### 10c. 전체 흐름

```
[Frontend] [COMPUTE]
  → calibration_node._srv_handeye_compute
  → HandEyeCalibration.compute_with_diagnostics(use_extended_ba=True)
  → bundle_adjust_hand_eye_extended()
  → 응답 dict: { joint_offset_delta, link_trans_delta, link_rot_delta, ... }

[Frontend] [COMMIT]
  → calibration_node._srv_handeye_commit
  → JointCoordinates().commit_offsets(...)  → joint_offsets.npz (누적)
  → LinkCoordinates().commit_offsets(...)   → link_offsets.npz   (누적, 신규)
  → 응답: restart_required=true

[Backend 재시작]
  → get_default_kinematics() 부팅
  → LinkCoordinates() 새 값 로드
  → write_patched_urdf(...) → .patched/omx_f.urdf 갱신
  → p.loadURDF(patched_path) → FK/IK가 새 모델로 동작
```

---

## 11. 검증 — patched URDF가 numpy fk_chain과 일치하는가

BA는 numpy `fk_chain`으로 푸는데 production은 PyBullet의 patched URDF.
**두 경로가 수치적으로 같아야** BA가 풀어준 값이 시스템에 그대로 반영됨.

URDF patcher 일치 검증 — 같은 random angles로 양쪽 FK 호출:

```python
for k in range(30):
    angles = rng.uniform(-np.pi/2, np.pi/2, 5)

    # (A) PyBullet (patched URDF)
    for j, idx in enumerate(arm_indices):
        p.resetJointState(robot, idx, float(angles[j]), ...)
    state = p.getLinkState(robot, ee_index, computeForwardKinematics=True, ...)
    pb_pos = np.array(state[4])
    pb_R   = quat_to_R(state[5])

    # (B) numpy fk_chain (같은 link_offset)
    np_R, np_t = fk_chain(angles, LINK_TRANS, LINK_ROT)

    pos_err_mm  = np.linalg.norm(pb_pos - np_t) * 1000
    rot_err_deg = ... # axis-angle 차이

결과: max pos_err = 0.047mm,  max rot_err = 0.012°
```

수치 정밀도 수준에서 일치. 즉:

- BA가 numpy로 푼 link_offset 값 = 같은 link_offset으로 patched URDF 만들면 같은 FK
- 자세 시뮬에서 BA가 예측한 EE 위치 = production code에서 본 EE 위치
- 시스템 일관성 보장

---

## 12. 진짜 system 보정인지 vs overfit인지 — Hold-out validation

41자유도가 32포즈에만 fit한 overfit일 수 있음. Hold-out validation:

```python
# 32포즈를 train(24)/test(8) random split — 3 seed 반복
for seed in range(3):
    rng = np.random.default_rng(seed)
    idx = rng.permutation(n)
    train_idx, test_idx = idx[:24], idx[24:]

    # train으로 BA 풀기
    x_opt = fit_train(angles_train, R_tc_train, t_tc_train)

    # train의 mean_pos / mean_R을 기준으로 test 포즈들의 σ 측정
    T_train = compute_T_list(x_opt, angles_train)
    T_test  = compute_T_list(x_opt, angles_test)

    mean_pos = positions_train.mean(axis=0)
    mean_R   = mean_rotation(...)

    sr_train, st_train = sigma_against_train(T_train)
    sr_test , st_test  = sigma_against_train(T_test)   # test가 train과 얼마나 다르나
    ratio = sr_test / sr_train

평균:
  train σ=(1.28°, 9.82mm)
  test  σ=(1.35°, 9.88mm)
  ratio = 1.06× / 1.01×    ← 1.5× 이내면 양호
```

test ≈ train → overfit 아님. BA가 진짜 system 파라미터 잡은 거.

---

## 13. 분산 동기화 — CLAUDE.md 패턴 그대로

`link_offsets.npz`는 git tracked → 모든 머신이 같은 commit = 같은 파일.
Zenoh 토픽 전파 X.

```
PC에서 [COMMIT]
  → robot/calibration/link_offsets.npz 저장
  → PC의 LinkCoordinates 메모리 즉시 갱신
  → 단 PybulletSolver는 부팅 시 로드라 *재시작 필요*

PC에서 git add + commit + push

모터 Pi (motion_node)
  → ssh + git pull + backend 재시작
  → Kinematics 부팅 시 새 link_offsets로 patched URDF 자동 생성
  → motion_node의 IK가 patched URDF로 풀음

카메라 Pi (camera_node)
  → 영향 없음 (Kinematics 안 씀)
```

`.patched/`가 .gitignore된 게 핵심:

- 머신마다 자기 link_offsets로 자체 생성
- git에 보이는 URDF는 원본 하나뿐
- push되는 건 `link_offsets.npz` 하나 → 분산 모드 깔끔

---

## 14. 결과 해석 가이드

확장 BA 결과 dict의 어떤 값을 보고 무엇을 판단하나:

| 필드                 | 색 임계 ([HandEyeResults.tsx](../frontend/src/components/calibration/HandEyeResults.tsx)) | 의미                                               |
| -------------------- | ----------------------------------------------------------------------------------------- | -------------------------------------------------- |
| `sigma_rot_deg`      | <1° good, <2° warn                                                                        | 회전 floor — link_rot가 흡수                       |
| `sigma_t_mm`         | <10mm good, <20mm warn                                                                    | 위치 floor — link_trans가 흡수                     |
| `joint_offset_delta` | abs<2° 정상, ≥2° 주의                                                                     | horn-level 보정. 첫 commit 후 잔여는 ≈0            |
| `link_trans_delta`   | abs<5mm 정상, <20mm 노랑, ≥20mm 주의                                                      | mm 단위. ≥20mm면 gauge freedom 의심                |
| `link_rot_delta`     | abs<0.5° 정상, <2° 노랑                                                                   | small-angle 가정 안. 2°↑면 ZYX vs rotvec 변환 검토 |

**정상 수렴 시그니처:** σ가 GOOD 안 + 다음 라운드 `*_delta`가 모두 ≈0.
그게 BA가 "더 흡수할 게 없음" 신호.

---

## 15. 더 정밀하게 — 0.5° / 5mm 도달 경로

확장 BA가 모델 변수로 풀 수 있는 만큼 다 풀었는데도 1.3°/9mm 가 남았다 →
**모델 *밖*의 노이즈가 floor를 결정한다.** 그 노이즈 출처를 줄이면 σ도 같이 떨어짐.

### 15a. floor 노이즈 출처 분석

| 노이즈 출처                             | 현재 영향                     | 개선 시 효과                  |
| --------------------------------------- | ----------------------------- | ----------------------------- |
| D405 color **intrinsic** (factory seed) | PnP에 0.1~0.3° 회전 노이즈    | σ_rot 0.2~0.4° ↓              |
| **체커보드 인쇄/평탄도**                | corner 위치 ±0.5mm            | σ_rot 0.2~0.3° ↓, σ_t 1~2mm ↓ |
| **PnP corner detection** 정밀도         | refine 안 하면 ~0.1°          | σ_rot 0.1~0.2° ↓              |
| **자세 다양성** (J1/J4/J5 std)          | 부족 시 ill-conditioned       | σ_rot 0.1~0.3° ↓              |
| **모션 블러**                           | 캡처 전 0.5s 대기로 무시 가능 | —                             |

각 출처가 *독립적으로 누적*되니까 여러 개 잡으면 곱하기로 효과 — 셋만
잡아도 σ_rot 1.3° → 0.5° 가능.

### 15b. 1순위 — 체커보드 정확도 (ROI 최대)

지금 일반 종이 인쇄면 격자 ±0.5~1mm 오차가 그대로 PnP에 반영. corner
검출이 sub-pixel 정확해도 _진짜 좌표가 틀린_ 거라 BA로 못 푼다.

대안:

- **레이저 컷 아크릴/금속판** + 정밀 인쇄 부착 (또는 plotter 인쇄 후
  유리/아크릴 마운트로 평탄도 확보)
- 격자 크기 **25~30mm 정사각형** (더 크면 PnP 안정성 ↑, 9×6 정도)
- D405 작업 거리 **30~40cm 고정** — 너무 가까우면 시야 부분만 차지, 너무
  멀면 corner 해상도 부족

이 하나로 σ_rot 0.3° / σ_t 2mm 빠질 가능성. **하드웨어 투자 필요** (3D프린트 또는 외주).

### 15c. 2순위 — D405 intrinsic 재캘리브

[intrinsic.npz](../robot/calibration/intrinsic.npz)는 factory seed로 채워졌다
(`rms_error=0.0` — factory 값을 그대로 적은 거라 0. 진짜 재캘 잔차가 아님).

D405는 factory 캘이 일반적으로 정확하지만, 0.1~0.3° 수준의 미세 노이즈는 남음.
체커보드로 재캘하면 그걸 잡을 수 있다.

코드는 이미 다 있음 — [backend/modules/calibration/intrinsic.py](../backend/modules/calibration/intrinsic.py):

```python
# cv2.findChessboardCornersSB (sector-based)
#   조명/블러에 강하고 sub-pixel 정확도까지 내장.
# cv2.calibrateCamera(obj_points, img_points, image_size, ...)
#   K, dist 추정 + per-image rms_error 반환
```

Frontend의 Intrinsic 탭에서:

1. 다양한 각도/거리/회전으로 **15~20장** 캡처 (체커보드가 화면 다른 위치를 골고루)
2. COMPUTE → `rms_error < 0.3px`면 좋은 결과
3. COMMIT → `intrinsic.npz` 갱신

새 intrinsic 적용된 후 Hand-Eye 캘 다시 돌리면 σ_rot 0.2~0.3° 추가 감소.

### 15d. 3순위 — PnP refineLM (검증 결과: 효과 0, 변경 X)

> 처음에는 "코드 10줄로 σ 0.1~0.2° 추가 감소" 후보로 검토했으나
> 합성 잡음 Monte Carlo로
> 효과가 0임을 확인. 현재 코드 유지.

가설은 — [pose_estimator.py:24](../backend/modules/calibration/pose_estimator.py)
의 `cv2.solvePnP(obj, img, K, dist)`가 "초기 해"만 풀고, 그 위에
`cv2.solvePnPRefineLM`을 추가하면 LM이 재투영 잔차를 더 최소화한다 — 였음.

검증: 기존 41포즈의 (R, t)를 ground truth로 취급, `cv2.projectPoints`로
ideal corner 생성 후 가우시안 0.05~0.5px 잡음 추가, 4가지 변형 비교.

| 변형                                    | rot 평균 오차 | t 평균 오차 |
| --------------------------------------- | ------------- | ----------- |
| (i) ITERATIVE alone _(= 현재 코드)_     | **0.3252°**   | **0.711mm** |
| (ii) EPNP alone                         | 1.0241°       | 2.314mm     |
| (iii) EPNP + refineLM                   | 0.3294°       | 0.717mm     |
| (iv) ITERATIVE + refineLM _(원래 제안)_ | **0.3252°**   | **0.711mm** |

**원인 — default flag**: `cv2.solvePnP`는 flag 미지정 시 `SOLVEPNP_ITERATIVE`로
폴백. 이 모드는 _내부에서 이미 LM refinement를 수행_. 그 위에 `solvePnPRefineLM`을
호출하면 같은 잔차에 같은 LM을 같은 seed에서 다시 돌리는 셈 → no-op. 모든
노이즈 레벨에서 (i) ≡ (iv) 가 numerically 정확히 일치.

(iii)이 (i)과 비슷한 건 LM이 EPNP의 거친 해(iii의 (ii) seed)를 ITERATIVE 수준까지
끌어올렸기 때문. 즉 LM은 _좋은 seed가 없을 때만_ 의미 있고, 우리 코드는 이미
ITERATIVE라 좋은 seed가 보장돼 있음.

합성 잡음 가정은 실제 corner 검출 노이즈와 분포가 다를 수 있어 _절대 σ 수치_
는 신뢰 X, 다만 *알고리즘 우위*는 잘 드러남 — refineLM 추가의 우위가 정확히
0이면 실측에서도 0.

**결론**: pose_estimator.py 변경 없음. § 15g 권장 경로에서 이 행 제거.

### 15e. 4순위 — 자세 다양성 점검

확장 BA가 잘 풀려면 J1/J4/J5(회전 추정 주요 축) 자세가 _고르게 흩어져_ 있어야 함.
한 축이 좁은 범위에 몰려 있으면 BA가 그 방향 정보를 못 받아 ill-conditioned.

기존 진단 코드 — [coach.py](../backend/modules/calibration/coach.py)의
`axis_distributions`가 각 축 std와 추천 추가 캡처 영역을 알려줌:

```python
# COMPUTE 응답의 coach.axis_distributions
# 각 항목: {motor_id, std_deg, min_deg, max_deg, is_low_diversity, suggested_deg, ...}
```

`is_low_diversity=true`인 축 있으면 그쪽 자세 5~10개 추가 캡처 후 재캘.
[thresholds.py](../backend/modules/calibration/thresholds.py)의
`JOINT_DIVERSITY_THRESHOLD_DEG=(25, 15, 15, 25, 30)` 미만이 low.

### 15f. 5순위 이하 — 장기/하드웨어

| 액션                                | 효과                                        | 비용                 |
| ----------------------------------- | ------------------------------------------- | -------------------- |
| 모터 horn 정밀 재조립 (각도 게이지) | J2/J3 큰 joint_offset 제거                  | 분해 필요            |
| 링크 부품 실측 → URDF 직접 갱신     | link_trans/rot이 0에 가까워짐, 모델 더 깨끗 | 캘리퍼스 + URDF 수정 |
| 더 큰 / 정밀한 체커보드 (50mm)      | PnP 안정성 한 단계 ↑                        | 새 보드 제작         |
| 멀티 자세 누적 ICP refinement       | 캘 결과 추가 검증                           | 별도 알고리즘        |

### 15g. 현실적 권장 경로

| 순서    | 액션                                                               | 누적 σ_rot / σ_t   | 비용            |
| ------- | ------------------------------------------------------------------ | ------------------ | --------------- |
| 0       | 확장 BA 적용                                                       | 1.30° / 9.3mm      | 완료            |
| ~~1~~   | ~~+ PnP refineLM~~ — § 15d 검증으로 폐기 (효과 0)                  | —                  | —               |
| ~~1.5~~ | ~~+ Robust loss (huber f=0.010)~~ — § 15i 검증으로 폐기 (artifact) | —                  | —               |
| **0.5** | **+ 물리 sag 모델 (§16, default)**                                 | **0.65° / 7.94mm** | 완료            |
| 2       | + intrinsic 재캘리 (UI에서)                                        | ~0.5° / ~6mm       | 1시간           |
| 3       | + 정밀 체커보드 (아크릴 마운트)                                    | **~0.4° / ~5mm**   | 3D프린트 / 외주 |
| 4       | + 자세 다양성 보강 (J2 수평 자세 추가)                             | ~0.3° / ~4mm       | 추가 캡처 30분  |

§16의 물리 sag로 step 0.5에서 σ_rot 절반(1.30→0.65). **2+3 조합으로 산업
정밀도 도달.** 4는 보너스이자 §16의 연속 split extrapolation 한계 해소책.

각 단계 적용 후 확장 BA 다시 돌려서 σ 측정 → 진짜로 떨어졌는지 검증.
한 단계씩 확인하면서 가는 게 안전 (어디서 효과 가장 큰지 데이터로).

### 15i. 검증 폐기 — Robust loss (huber/cauchy/soft_l1)

> Robust loss 진단(6변형 + hold-out + reg sweep)에서
> 으로 검증. σ_t 1mm 개선이 _모델 파라미터 폭주에 의존하는 numerical artifact_
> 임을 확인 → 채택 X. bundle_adjust.py default는 lm/linear 유지.

가설: 현재 잔차 분포에 약한 outlier 있음 (σ_t max/RMS = 2.19×). robust loss
(huber/cauchy/soft_l1)로 그 영향 down-weight하면 σ 추가 감소 가능. method='lm'은
robust loss 미지원이라 'trf' (Trust Region Reflective)로 함께 교체.

1차 결과 (41 포즈 fit):

| Variant               | σ_rot  | σ_t        | X Δt vs linear |
| --------------------- | ------ | ---------- | -------------- |
| (현재) lm/linear      | 1.295° | 9.17mm     | —              |
| trf/linear (sanity)   | 1.295° | 9.17mm     | 0mm            |
| **trf/huber f=0.010** | 1.294° | **8.29mm** | **16.80mm**    |
| trf/cauchy f=0.010    | 1.293° | 8.17mm     | 25.48mm        |

σ_t 1mm 개선처럼 보임 — but X(cam2gripper)가 17~25mm 움직임.

Hold-out (train 33 / test 8, 3 random seed) — generalize 확인:

| Variant        | train σ_t | test σ_t | ratio_t |
| -------------- | --------- | -------- | ------- |
| huber f=0.010  | 8.24mm    | 8.74mm   | 1.06× ✓ |
| cauchy f=0.010 | 8.02mm    | 9.31mm   | 1.16× ⚠ |

huber는 ratio 1.06× — 확장 BA 원본 1.01×와 비슷한 generalize. _통계적으로_
는 진짜 개선처럼 보임. 그러나:

**진짜 원인 발견 — reg weight sweep**:

| link_reg          | σ_rot  | σ_t         | maxLT (mm)  |
| ----------------- | ------ | ----------- | ----------- |
| 1.0/1.0 (default) | 1.294° | **8.29**    | **49.17** ⚠ |
| 2.0/1.0           | 1.295° | 9.03        | 38.56       |
| 2.0/2.0           | 1.301° | 9.14        | 38.46       |
| 3.0/2.0           | 1.302° | **10.14** ⚠ | 26.67       |

linear (현재)의 maxLT는 ~29mm. robust huber default는 maxLT 49mm로 link offset
이 **70% 폭주**. link_reg를 강화해 maxLT를 linear 수준(27mm)으로 묶으면 σ_t는
역으로 10.14mm — _linear(9.17mm)보다 나빠짐_.

즉: **robust loss는 1mm σ를 줄이지만 그건 link 모델이 50mm 더 풀린 결과**.
같은 link 크기에서는 linear가 우월. patched URDF가 link2 origin을 49mm 옮긴
상태로 IK/FK 풀게 되는 게 _physical sense_. → 다음 라운드부터 운동학 모델이
점점 비현실적으로 발산할 위험.

**결론**: bundle_adjust.py default = lm/linear 유지. method/loss/f_scale 파라
미터는 인터페이스에 남겨 비교 실험 가능 (회귀 진단/디버깅 용도).

**일반 교훈**: σ만 보고 robust loss 판단 금지. 모델 파라미터(여기선 link offset)
가 _물리적으로 안 변하는지_ 함께 확인. σ ↓ + 파라미터 폭주 = numerical artifact
(자유도가 다른 변수로 흘러감). σ ↓ + 파라미터 안정 = 진짜 개선.

### 15h. 한계 — 모터 zero point 측정의 어려움

위 액션 다 해도 σ_rot < 0.2° 가려면 *모터 zero point의 물리적 정확도*가 필요.
Dynamixel raw 2048이 URDF의 0°와 정확히 일치한다는 보장이 없는데, 이건 외부
정밀 측정기 (각도 게이지, encoder 등)로만 검증 가능. DIY 환경에선 BA가
joint_offset으로 보정하는 게 한계.

물리 sag(§16)로 자세 의존 부분을 분리하면서 *조립/모터/링크 정밀도*가 진짜
floor가 됨. 산업 로봇이 0.1°까지 가는 건 *외부 정밀 측정 장비를 캘 단계에
사용*하기 때문. DIY 5축에선 **0.3° 정도가 현실적 한계** (sag 적용 + intrinsic
재캘 + 정밀 체커보드 조합 기준). 그 이하는 비용 곡선이 가파르게 올라감.

---

## 16. 물리 sag 모델 — 자세 의존 중력 처짐 분리

> 확장 BA 이후 σ_rot 1.30°/σ_t 9.3mm 정체. 한 사용자 관찰이 다음 단계의 단서를 줌:
> _"토크 OFF → 자세 잡음 → 토크 ON → 살짝 처짐. 자세마다 다 다름 (팔 펴면 더 처짐)."_
> 이건 link offset(자세 _무관_ 상수 보정)으로는 표현 불가능 — _자세 의존_ 오차.

### 16.1. 진단 — 외부 의견을 데이터로 검증

가설: "확장 BA의 link offset이 흡수한 것 중 일부는 사실 자세 의존 중력 sag."

Sag 가설 진단 — 41 포즈에 6 시나리오 BA fit:

| 시나리오                           | σ_rot  | σ_t    | DOF    | link_t_max |
| ---------------------------------- | ------ | ------ | ------ | ---------- |
| (4) link on, sag off **[현 prod]** | 1.296° | 9.29mm | 41     | 29.1mm     |
| (6) link on, sag sincos            | 0.633° | 7.76mm | 45     | **5.6mm**  |
| (3) link off, sag sincos           | 0.814° | 9.55mm | **15** | —          |

발견 두 가지:

- **(3) vs (4)**: DOF 15짜리가 DOF 41 prod보다 σ_rot에서 압도 (0.81° vs 1.30°).
  link offset 30개가 사실상 *자세 의존 오차의 빈약한 대용품*이었다는 시사.
- **(6) → link_t_max 29.1→5.6mm**: sag 추가하니 link offset이 5배 줄어듦.
  § 15i robust loss와 정반대 signature (그땐 σ↓ + 파라미터 폭주 = artifact).
  여기는 **σ↓ + link 폭주 감소 = 진짜 분리되는 변수**.

### 16.2. sin/cos basis는 왜 부족한가

처음엔 `sag_J = a·sin(θ_J) + b·cos(θ_J)` 임의 sinusoidal basis로 시작. random
hold-out(1.05× ratio)에선 통과했지만 robustness 진단의 *J2 연속 split*에서 폭주:

| split                                        | sag off       | sag sincos           |
| -------------------------------------------- | ------------- | -------------------- |
| lower 70% → upper 30% (큰 sag로 extrapolate) | 2.35× / 2.79× | **6.61× / 7.84×** ❌ |

train J2 [-82°, -60°]만 보고 test의 J2 -5° (수평) 예측 → sin/cos가 잘못된 방향
폭주. 이유: 같은 J2 각도라도 _팔 펴짐 정도(J3, J4, J5 자세)에 따라 모멘트 암이
다른데_, single-joint angle basis는 그걸 못 잡음.

→ **물리 모델로 가야 함** — `error = f(자세 전체)`가 표현되도록.

### 16.3. 물리 모델 — 모멘트 암 ∝ 처짐

중력 토크의 1차 모델:

```
τ_J = (r × g_dir) · axis_J     where r = ee_pos - joint_origin   (base frame)
sag_J = k_J · τ_J              (k = 1/effective_stiffness, BA 변수)
```

`r`은 base frame에서 ee 위치 - joint 회전축 위치 = 모멘트 암 벡터. 같은 J2 각도라도
J3/J4/J5 자세가 다르면 ee 위치가 달라 `r`도 달라짐 → sag 다름. _전체 자세 의존성을
2 params(k_J2, k_J3)로 표현_.

[fk_chain.py](backend/modules/kinematics/fk_chain.py) helper:

```python
def fk_chain_with_axes(angles, link_trans=None, link_rot=None):
    """fk_chain + 각 joint origin/axis (base frame). 중력 토크 계산용."""
    T = np.eye(4)
    joint_origins_base = np.zeros((5, 3))
    joint_axes_base = np.zeros((5, 3))
    for i in range(5):
        T_o = np.eye(4)
        T_o[:3, :3] = rotvec_to_R(link_rot[i])
        T_o[:3, 3] = JOINT_ORIGINS[i] + link_trans[i]
        T = T @ T_o
        # 회전 적용 *전* 위치/방향이 토크 계산용
        joint_origins_base[i] = T[:3, 3]
        joint_axes_base[i] = T[:3, :3] @ JOINT_AXES[i]
        T = T @ axis_angle_to_R(JOINT_AXES[i], angles[i])
    ...
    return R_ee, t_ee, joint_origins_base, joint_axes_base


_GRAVITY_DIR = np.array([0.0, 0.0, -1.0])

def gravity_torque_lumped(ee_pos_base, joint_origin_base, joint_axis_base):
    """ee에 lumped mass 가정. τ = (r × g) · axis."""
    r = ee_pos_base - joint_origin_base
    return float(np.dot(np.cross(r, _GRAVITY_DIR), joint_axis_base))


def apply_gravity_sag(joint_angles, k_stiff, link_trans=None, link_rot=None):
    """commanded → sag 적용 actual. J2, J3에만 적용."""
    if k_stiff.size == 0 or float(np.max(np.abs(k_stiff))) < 1e-12:
        return joint_angles.copy()
    _, ee_pos, jo, ja = fk_chain_with_axes(joint_angles, link_trans, link_rot)
    out = joint_angles.copy()
    out[1] += k_stiff[0] * gravity_torque_lumped(ee_pos, jo[1], ja[1])
    out[2] += k_stiff[1] * gravity_torque_lumped(ee_pos, jo[2], ja[2])
    return out
```

J2, J3에만 sag — J1/J4/J5의 sag는 측정 noise 수준이라 모델 단순성 위해 제외
(물리 sag 진단에서 검증).

### 16.4. PyBullet vs lumped — URDF mass 부정확성 발견

URDF의 link5 mesh가 `follower_06_pan_Revised_d405.stl` (D405 버전)이지만
**inertial 데이터가 D405 무게 반영 안 됐을 가능성** 발견 — link5 mass=44g인데
D405 자체가 ~42g (마운트만 2g = 비현실).

PyBullet 비교 진단 — PyBullet의
`calculateInverseDynamics` (URDF mass 기반 정확 토크) vs lumped (mass × 모멘트
암 가정) 비교:

| 모델                     | σ_rot      | σ_t        |
| ------------------------ | ---------- | ---------- |
| **lumped**               | **0.651°** | **7.94mm** |
| PyBullet inverseDynamics | 0.766°     | 10.48mm    |

PyBullet이 lumped보다 σ_rot 0.115° 더 _나쁨_. 원인: URDF mass 부정확 → 토크
underestimate. lumped는 _k가 (1/stiffness × effective_mass) 비율을 통째로 흡수_
해서 mass 부정확성에 robust.

**결론**: 검증된 라이브러리(PyBullet)가 항상 우월하지 않음. URDF mass의
*정확도*가 보장 안 될 때는 lumped + k의 자유도로 mass 오차도 함께 fit하는 게
실용적. 미래 D405 mass center/inertia 측정 → URDF 업데이트 시 PyBullet 재검토.

### 16.5. 확장 BA + sag = 43 DOF

[bundle_adjust.py:bundle_adjust_hand_eye_physical_sag](backend/modules/calibration/bundle_adjust.py) — extended(41) + sag_k 2개 = 43:

```python
# 변수 layout:
#   [0:5]    joint_offset (rad)
#   [5:20]   link_translation (5×3, m)
#   [20:35]  link_rotation (5×3, rad rotvec)
#   [35:37]  sag_k (J2, J3) (rad / (m·g_unit))     ← 신규
#   [37:40]  rod (cam2gripper)
#   [40:43]  t (cam2gripper, m)

def compute_T_target_in_base(x):
    offset, link_t, link_r, sag_k, rod, t_x = unpack(x)
    R_x = cv2.Rodrigues(rod)[0]
    T_x = make_T(R_x, t_x)
    out = []
    for i in range(N):
        # joint angle에 offset + sag 둘 다 적용 후 FK
        a_corr = apply_gravity_sag(
            angles_arr[i] + offset, sag_k, link_t, link_r
        )
        R_gb, t_gb = fk_chain(a_corr, link_t, link_r)
        T_gb = make_T(R_gb, t_gb)
        out.append(T_gb @ T_x @ T_tc_list[i])
    return out
```

잔차 + reg는 extended와 동일 + sag_k에 `sag_k_reg=0.0` (default). reg sweep으로
0~0.1 sweet spot 확인 (robustness 진단 §F). k_J2/k_J3 자체가
작은 양수(~0.27, ~0.14)라 reg 없이도 폭주 안 함.

### 16.6. IK 역방향 — actual_to_commanded

FK는 `commanded → actual` (motor encoder reading → 실제 link end 자세). IK는
역방향이라 _implicit equation_:

```
actual = commanded + sag(commanded)   [BA 모델]
→ commanded = actual - sag(commanded)   [IK가 풀어야 할 것]
```

implicit이라 fixed-point. 1차 Taylor 근사 (sag ~2°라 잔차 < 0.05°):

```
commanded ≈ actual - sag(actual)
```

[fk_chain.py:actual_to_commanded](backend/modules/kinematics/fk_chain.py)가 이걸 처리. [Kinematics.ik](backend/modules/kinematics/registry.py)가 PyBullet IK 결과(`actual`)를 받아서 `actual_to_commanded` 한 번 호출 → motor 명령으로 변환.

### 16.7. SagCoordinates — joint/link와 다른 점

`sag_offsets.npz`는 joint/link와 같은 git tracked + cumulative merge 패턴
([SagCoordinates](backend/core/coords/sag_coordinates.py)). 차이점은 **PC 내부는
재시작 불필요**:

|                 | 어떻게 적용되나                                | 재시작                       |
| --------------- | ---------------------------------------------- | ---------------------------- |
| joint_offsets   | `raw_to_rad` 호출 시 JointCoordinates에서 읽음 | X (이미 즉시)                |
| link_offsets    | PyBullet URDF 로드 시점                        | **필요** (URDF 한 번만 로드) |
| **sag_offsets** | **매 FK/IK 호출 시 메모리에서 읽음**           | **X**                        |

[calibration_node.py:\_srv_handeye_commit](backend/nodes/application/calibration_node.py)에서 COMMIT 시 `solver._reload_sag_cache()` 호출 — 다음 FK/IK부터 자동 반영. 다른
머신은 git pull + 재시작 (joint/link와 동일).

### 16.8. 검증 한계 — 캡처 자세 범위 안 OK, 밖 미검증

물리 sag 진단의 연속 split:

| split                                               | sag off       | sag physical        |
| --------------------------------------------------- | ------------- | ------------------- |
| lower 70% → upper 30% (큰 sag 영역으로 extrapolate) | 2.35× / 2.79× | **4.44× / 6.35× ⚠** |
| upper 70% → lower 30% (작은 sag 영역)               | 2.45× / 2.86× | **2.08× / 2.28× ✓** |
| middle 60% → edges 40%                              | 2.34× / 2.21× | 2.86× / 2.88×       |

physical 모델은 sin/cos보다 _모든_ split에서 개선 (특히 작은 sag 영역으로
extrapolate는 sag off보다도 좋음). 그러나 가장 어려운 lower→upper는 여전히
4.44× — *J2 수평 자세를 캡처 안 한 영역으로 extrapolate*는 미검증.

**현재 41 포즈의 J2 분포 [-82°, -5°]**. OMX_F 일상 deployment(pick&place,
TSDF)는 책상 위 작업 → J2 ~-30°~-60° 정도가 보통. 즉 **캡처 영역 안**이라
production에선 평균적으로 σ_rot 0.65° 도달. 캡처 영역 밖(예: 팔 더 펴기)으로
가면 부분적으로 sag off보다 나쁠 수 있음.

이게 §15g의 step 4 "자세 다양성 보강" (J2 수평 자세 5~8개 추가 캡처)의
이유. 해결되면 lower→upper ratio도 2× 안으로 들어올 것.

### 16.9. 검증 — production BA가 진단 결과와 일치

smoke test로 새 `bundle_adjust_hand_eye_physical_sag`가 inline 진단과
_정확히_ 같은 결과 내는지 확인:

```
σ_rot = 0.651°   (diag 기대: 0.651°)   ✓
σ_t   = 7.94mm   (diag 기대: 7.94mm)   ✓
sag_k = (+0.26523, +0.14126)            ✓
max sag (deg): J2=4.67, J3=2.26         ✓
link_t_max = 7.7mm                      ✓
```

소수점 자리까지 일치. 진단 → production 이식 회귀 없음.

### 16.10. 결과 + 다음 단계

- σ_rot **1.30° → 0.65°** (절반)
- σ_t **9.29mm → 7.94mm** (15% 감소)
- link offset 폭주 감소 (29.1mm → 5.6mm) — 모델이 _물리적으로_ 더 깨끗

다음 단계는 §15g 표 그대로:

- step 2: intrinsic 재캘리브 (1시간)
- step 3: 정밀 체커보드 (3D프린트 / 외주)
- step 4: J2 수평 자세 캡처 (§16.8의 extrapolation 한계 해소)

추가로 — D405 mass center/inertia 정확히 측정 → URDF 업데이트하면 PyBullet
inverseDynamics 모델로 재검토 가능. 단 σ 0.1° 차이라 _라이브 PC 시각 검증_ 후
필요 시.

---

## 17. 미해결 항목 (다음 작업 후보)

§16까지로 backend 캘 정확도는 σ_rot 0.65°/σ_t 7.94mm 도달. 단 _그 정확도가
사용자 측에서 실제로 보이게_ 하려면 두 가지 작업이 남음.

### 17.1. 12V 전압 변경 (선택)

현재 메인 PSU 11V — XL430 정격(10~14.8V, 권장 12V) **하한 근처**. 12V로 올리면:

- XL430 모터 토크 출력 ~9% 증가
- 자세 의존 sag 크기 ~9% 감소 (max sag J2 4.67° → ~4.2°)
- 모터 안정성 ↑

단 **σ에 미치는 영향은 작음** (~0.05~0.1°) — §16 sag 모델이 11V 처짐을
_이미 보정_ 중이라 그 차이는 모델이 흡수. 캘 정확도 측면만 보면 12V 변경의
ROI는 작고, 진짜 가치는 _모터 토크 마진 + 안정성_.

변경 시 절차 — sag k 값이 11V에 fit돼 있어서 cumulative 누적으로 갈 경우 첫
라운드 delta가 큼 → **clean slate가 더 깔끔**:

```
1. 백업: robot/calibration/ 의 5 파일 (joint/link/sag_offsets.npz +
   handeye_poses.npz + hand_eye.npz)을 *_11v_backup.npz로 복사
2. 원본 5 파일 삭제 (intrinsic.npz는 카메라 자체 캘이라 유지)
3. backend 재시작 → SagCoordinates/JointCoordinates/LinkCoordinates 빈 상태
4. Hand-Eye 탭에서 30~40 자세 재캡처 → COMPUTE → COMMIT
5. 결과 검증:
   - σ_rot 0.6~0.7° (11V와 비슷한 수준이면 모델 재현성 OK)
   - sag k_J2 ~0.24, k_J3 ~0.13 (11V의 ~91%면 전압 효과 정량 확인)
   - joint_offset 11V와 비슷한 값 (전압 무관 변수의 안정성 검증)
```

**caveat**: XL330 그룹(J4/J5/그리퍼)는 XL4015 강압 모듈로 _5V 그대로_ 유지
([CLAUDE.md] 전원 토폴로지). 12V 변경은 XL430 (J1/J2/J3)만 영향 — sag 모델이
J2/J3에 적용된 것과 정확히 일치.

### 17.2. frontend 라이브 PC 시각화에 link/sag 반영 (중요)

§16의 sag 모델까지 적용됐지만 **frontend의 workspace3d 라이브 PC 시각화는
여전히 sag/link offset을 못 봄**. 사용자가 σ 0.65° 결과를 _시각으로_ 확인할
때 어긋남 잔존 가능.

#### 현재 반영 상태

|                  | frontend 반영 | 이유                                                                                                                       |
| ---------------- | ------------- | -------------------------------------------------------------------------------------------------------------------------- |
| **joint_offset** | ✅            | [Workspace3D.tsx](../frontend/src/pages/Workspace3D.tsx) 의 jointAngles 계산에서 `baseRad + jointOffsetsRad[id]` 적용      |
| **hand_eye**     | ✅            | calibration .npz fetch로 cameraMatrix 계산에 buildMatrix4 적용 — BA가 sag+link 적용해 fit한 값                             |
| **link_offset**  | ❌            | [RobotModel.tsx:53](../frontend/src/components/workspace3d/3d/RobotModel.tsx) 이 _원본 omx_f.urdf_ fetch. `.patched/` 아님 |
| **sag**          | ❌            | frontend URDF가 commanded angle로 시각화. `actual_to_commanded` 적용 X. URDF 정적이라 자세 의존 sag 표현 자체 불가         |

#### 결과로 일어나는 어긋남

- backend FK = `fk(commanded + sag, patched URDF)` → 정확한 actual ee 위치
- frontend FK (urdf-loader) = `fk(commanded, 원본 URDF)` → commanded ee + 미보정 link
- 둘이 다른 위치를 봄. cameraMatrix = tcpMatrix · handEyeMatrix 곱셈이 _어긋난 tcpMatrix_ 위에 곱해져서 PC가 commanded 자세 기준으로 렌더링 → 실제와 sag/link만큼 차이.

#### 해결 옵션

**옵션 A — patched URDF 두 버전 생성**

[urdf_patcher.py:write_patched_urdf](../backend/core/coords/urdf_patcher.py)에 `for_web: bool` 플래그 추가. True면 mesh path를 _상대 유지_ (PyBullet은 false로 절대화). frontend는 `.patched/omx_f_web.urdf` fetch.

단점: 두 버전 관리 (cache invalidation 등).

**옵션 B — backend FastAPI가 동적 변환** ⭐ 추천

[zenoh_bridge.py](../backend/bridge/zenoh_bridge.py)의 `/robot` 정적 마운트 대신, `/robot/urdf/omx_f/omx_f.urdf` 요청에 한해 _동적 라우트_ 추가:

```python
@app.get("/robot/urdf/omx_f/omx_f.urdf")
def serve_patched_urdf_for_web():
    link_offsets = LinkCoordinates().snapshot()
    text = patch_urdf_text(URDF_PATH, link_offsets, absolute_mesh=False)
    return Response(text, media_type="application/xml")
```

`patch_urdf_text`에 `absolute_mesh=False` 옵션 추가하면 mesh path 상대 유지. ETag/cache로 link_offsets 변경 시에만 무효화.

장점: frontend 코드 변경 X. backend의 단일 진실에서 동적으로 응답.

**옵션 C — sag 적용 ee pose를 backend가 publish** (옵션 A/B와 별개로 필요)

§17.2 옵션 C가 진짜 깔끔한 이유 — frontend의 RobotModel.tsx:92에서 emitTCP를 자체 계산 대신 backend가 publish한 ee_pose_actual을 받아 그대로 쓰면 됨. RobotModel은 시각화용 로봇 모양만 그리고, 진짜 TCP는 backend 권위.
그럼 주기는..? 주기가 문제일거 같긴한데..

sag는 자세 의존이라 URDF로 표현 불가. backend가 [Kinematics.fk](../backend/modules/kinematics/registry.py) 결과(actual ee pose)를 새 토픽으로 발행:

```
omx/motor/state/ee_pose_actual   # T_base_ee (sag 적용된 actual)
```

frontend는 자체 FK 안 하고 그 매트릭스를 tcpMatrix로 사용:

```typescript
// RobotScene.tsx — RobotModel 안 쓰고 받은 ee_pose 직접 사용
const tcpMatrix = useEEPoseStore((s) => s.eeMatrix);
```

장점: 모든 정확도가 backend 단일 진실에서 옴. URDF는 *robot 모양 시각화*만 담당, ee 자세는 backend 권위.

단점: backend가 motor state publish할 때마다 Kinematics.fk 호출 (현재 20Hz 정도면 부담 작음).

#### 작업 우선순위

라이브 PC 시각 검증 먼저:

- _충분히 정렬_ — 작업 보류, sag 모델 backend 적용으로 충분
- _명확한 어긋남_ — 옵션 B + 옵션 C 둘 다 진행. ROI 큰 순서로:
  1. 옵션 C (sag 적용 ee pose publish) — 가장 큰 어긋남 원인 (~2-4°)
  2. 옵션 B (동적 patched URDF) — 작은 어긋남 (~30mm link translation)

옵션 A는 두 버전 관리 부담이라 B로 통일이 깔끔.

#### 우회 — hand_eye가 이미 BA 결과 반영

부분 위안: **hand_eye.npz는 BA가 sag/link 적용해 fit**한 결과 (T_cam2gripper at _actual_ ee). 그래서 frontend가 _commanded_ tcpMatrix에 그 hand_eye를 곱해도 _부분적으로_ 보정됨 — hand_eye matrix가 sag만큼 카운터-오프셋 가지고 있어서. 단 자세에 따라 sag 변화량이 다르니 _완전 보정은 안 됨_. 옵션 C가 진짜 해결.

---

## 부록 — 진단 방법론 요약

본문에 인용된 진단들은 BA 락인 후 정리. 동일 데이터에 시나리오를 바꿔가며 σ 비교 + hold-out으로 generalize 확인 + 연속 split으로 robustness 확인이 핵심. 다른 robot이나 다른 BA 모델로 확장할 때 같은 방법론으로 재검증.

검증된 시나리오 요약:

- joint_offset ON/OFF, baseline 0/현재 비교 → §2의 표
- link translation/rotation 자유도 + regularization sweep → §6, §7
- Hold-out train/test split → §9
- Patched URDF vs numpy fk_chain 수치 일치 → §8
- bundle_adjust_hand_eye_extended sanity (진단 결과 재현) → §14
- 폐기 근거 — PnP refineLM (σ 변화 0) → §15d, robust loss (link offset 폭주 의존) → §15i
- Sag 모델 진화 — sin/cos basis(폭주) → lumped 물리 sag(채택) → PyBullet inverseDynamics(URDF mass D405 누락으로 σ 손해) → §16


---
---

<!-- ═══════════ [통합 원문] calibration.md ═══════════ -->

# Accuracy Squeeze Plan

> DIY 3D프린트 + XL430/XL330 + 5DOF 라는 하드웨어 한계 안에서 **TCP 절대 정확도를 짜낼 수 있는 만큼 짜내는 것**이 목표.
>
> 현재 캘 4종 + 물리 sag 모델로 도달한 floor: **σ_rot 0.647° / σ_t 7.77mm**
> ([calibration.md](calibration.md)). 이 위에서 더 짜낸다.
>
> **2026-05-28 이력**: 시스템 commit 버그 발견 + fix (§1.6 참조). link/sag commit 이
> 과거 cumulative 가산이라 매번 누적 손상되던 것을 overwrite 로 영구 fix.
> disk σ 회복됨 (19.5mm → 7.77mm).

---

## 1. 즉시 과제 — 큐브 grasp 갭 진단

### 1.1 증상

pick_and_place 가 20mm 큐브를 "옆면 중간" 이 아닌 **상단** 에서 집음.

### 1.2 코드 의도는 맞음

```
detect       → position = (x, y, top_z),  _meta = {base_z, height}
GraspPolicy  → grasp_z = base_z + height * 0.5
```

[step_executor.py:246-254](../backend/modules/task/step_executor.py#L246-L254)
([_grounded_detect](../backend/modules/task/step_executor.py#L240)),
[step_executor.py:369](../backend/modules/task/step_executor.py#L369)
([_grasp_policy](../backend/modules/task/step_executor.py#L352)).

20mm 큐브면 `0 + 20 * 0.5 = 10mm` 가 옆면 가운데. 코드는 맞게 돼있음.

그런데 실제 결과가 위쪽 → **셋 중 하나가 거짓말 중**.

### 1.3 의심 셋

| | 거짓말 위치 | 결과 |
|--|--|--|
| **A** | 측정 height 가 실제(20mm)보다 작게 잡힘 | grasp_z 가 floor 쪽으로 끌려 내려가서 상대적으로 윗부분 접촉으로 보임 |
| **B** | base_z(floor) 가 실제보다 높게 잡힘 (책상 위로 떠 있음) | grasp_z 가 큐브 상단 근처로 올라감 |
| **C** | URDF 의 `tcp` link 가 실제 그리퍼 끝점 (핑거 닫혔을 때 만나는 점) 이 아니라 더 위 (손목 근처) | 명령은 옆면 가운데지만 실제 그리퍼 끝점은 위쪽 |

A, B 는 perception 문제. C 는 모든 task 공통의 **TCP 프레임 정의** 문제 — 즉 grasp 만이 아니라 미래 모든 task 가 같은 오프셋만큼 어긋남.

### 1.4 진단 — 코드 변경 0, 5분

큐브 하나 + 자 + pick_and_place 1회 시도.

기존 로그가 그대로 사용 가능:

1. **Detect 로그** ([step_executor.py:240-244](../backend/modules/task/step_executor.py#L240-L244))
   ```
   GroundedDetect 성공: conf=... base=(x, y, top_z)
   ```
2. **GraspPolicy 로그** ([step_executor.py:371-374](../backend/modules/task/step_executor.py#L371-L374))
   ```
   GraspPolicy base_z=... height=... → grasp_z=...
   ```

자로 잰 실측치와 비교해 표 채움:

| 항목 | 코드 값 | 자 실측치 | 차이 |
|--|--|--|--|
| height | (log) | 0.020 | → A 여부 |
| base_z | (log) | 책상 표면 z | → B 여부 |
| grasp_z | (log) | 그리퍼 끝점이 실제로 닿은 z | → C 여부 |

해석:
- height 가 5\~15mm 로 작게 나옴 → **A** (depth 샘플링이 책상 픽셀로 끌려감)
- base_z 가 큐브 두께만큼 떠있음 → **B** (ring 픽셀이 객체 옆면/그림자 포함)
- 위 둘 다 정상인데 grasp_z 명령과 실제 그리퍼 끝점 위치가 어긋남 → **C** (TCP 프레임 = EE link 위치가 실제 그리퍼 끝점 아님)

> ⚠️ default 트랩: `_meta.base_z`/`_meta.height` 가 detector 응답에 없으면 0.0 으로 떨어짐
> ([step_executor.py:251-253](../backend/modules/task/step_executor.py#L251-L253)). 둘 다 0.0 으로 찍히면 detector 가 안 채우는 거 — 진단 이전에 그 버그부터 잡아야 함.

### 1.5 케이스별 fix 방향

- **A**: depth 샘플링 개선. 현재 bbox 안 percentile 25 → percentile 더 위로 / segmentation mask 로 객체 픽셀만 / bbox erosion 으로 책상 leak 제거.
- **B**: ring pad 공식 조정 (객체 그림자/옆면 미포함). depth gradient outlier 거르기.
- **C**: URDF 의 `tcp_joint` xyz (현재 92mm) 가 실제 그리퍼 끝점 (핑거 닫혔을 때 만나는 점) 과 mm 단위로 안 맞는 것. link_offset 캘 자유도에 `tcp_joint` 추가해서 BA 가 같이 풀게 확장 (§ 3 참조).

### 1.6 시스템 버그 — BA commit cumulative 누적 손상 (이력: 2026-05-28 발견 + fix)

**발견 경위.** 사용자가 painful 한 캘 작업으로 σ_t 7.94mm 도달 한 게 첫 commit
직후 잠깐의 정답이었고, 이후 commit 들이 link/sag 를 cumulative 누적시키며 disk 가
실제 σ_t 19.5mm 까지 악화돼 있었음. 사용자는 σ 모니터링 안 한 채로 진행해서 인지 못 함.

**원인.** semantics 불일치:
- `bundle_adjust_hand_eye_physical_sag` 의 `x0` 가 link_t / sag_k 를 **0 으로 초기화** +
  내부 `fk_chain` 이 original URDF 기준 → BA 출력은 **absolute total** 값
- 그러나 `_srv_handeye_commit` 의 link/sag 가 `commit_offsets` 통해 **cumulative 가산**
  (`existing + delta`) — joint_offset semantics 를 그대로 적용한 잘못된 디자인
- → 매 commit 마다 disk = optimal × N 누적

`joint_offsets` 는 정상 — ja 가 `motor_to_urdf` 통해 이미 disk joint_offset 적용된
상태로 BA 에 들어가서 BA offset 은 진짜 delta. cumulative 가산 정합.

**검증 (2026-05-28).** 41 자세에서 두 가설 비교:
- H1 (TOTAL, link_t=BA): σ_t 7.770mm — BA 자체 보고와 0.0000mm 차이 ✓
- H2 (CUMULATIVE, link_t=disk+BA): σ_t 33.608mm — 25.8mm 차이 ✗
- → BA 출력은 absolute total. commit cumulative 가산은 버그.

**Fix 적용 (2026-05-28).**
- `LinkCoordinates.commit_offsets` / `SagCoordinates.commit_offsets` → **overwrite** 로 변경
  (참조: [link_coordinates.py](../backend/core/coords/link_coordinates.py),
  [sag_coordinates.py](../backend/core/coords/sag_coordinates.py))
- `_srv_handeye_commit` 변수명 / 주석 명확화
  ([calibration_node.py](../backend/nodes/application/calibration_node.py) `_srv_handeye_commit`)
- 모듈 문서 갱신: `link_offsets.py`, `sag_offsets.py`, `hand_eye.py:compute_with_diagnostics`,
  `bundle_adjust.py:bundle_adjust_hand_eye_extended`
- 프론트엔드 라벨 갱신 — "delta" → "절대 보정값" (참조: HandEyeResults.tsx)

`merge_delta` 유틸은 io 모듈에 남아있음 — 분석/실험 용도로만 사용, commit 흐름에서는
호출 안 됨.

**디스크 상태 (2026-05-28).**
- σ_t 19.535mm → 7.770mm, σ_rot 2.079° → 0.647° 회복
- 사용자가 painful 작업으로 도달했던 7.94mm 가 진짜 system floor 임을 재확인

**향후 사용:** UI 의 COMPUTE → COMMIT 흐름 정상 사용 가능. 매 commit 이 disk 를
absolute 정답으로 덮어씀, 누적 손상 없음.

---

## 2. TCP 정확도 = 모든 task 의 공통 인프라

어떤 task 든 결국 **"base 프레임의 특정 지점에 도구 작용점을 정확히 가져다 둔다"** 가 본질. TCP 정확도는 task 별 코드가 아니라 캘리브레이션 layer 가 책임.

OMX_F 의 URDF 는 `tcp` link 를 link5 에서 92mm 떨어진 지점에 박아두고
([omx_f.urdf:13-17](../robot/urdf/omx_f/omx_f.urdf#L13-L17)),
그 위치가 그리퍼 끝점 (핑거 닫혔을 때 만나는 점) 을 노린 점임 — 즉 **URDF 의도 자체가 "TCP 프레임 = 그리퍼 끝점"**.
Hand-eye 는 이미 이 정의 위에서 풀려있으므로 별도 tool offset 산출물 불필요.

```
base
 └─ link 0..5     (joint_offset + link_offset + sag 보정)
     └─ tcp = 그리퍼 끝점 (핑거 닫혔을 때 만나는 점)
              ↑
              link5→tcp 의 92mm 자체도 보정 대상
              (link_offset 자유도 확장으로 BA 가 풀게)
```

이 구조의 장점:
- 캘 산출물 늘리지 않음 — 기존 link_offset BA 의 자유도만 확장
- 모든 task 가 동일 정확도 baseline 위에 얹힘
- "EE 프레임 = 그리퍼 끝점" 의 의미적 일관성 유지

> 도구 갈아끼움(디스펜서, 펜 등) 가능성을 가정하면 산업 관행처럼 hand_eye 분리 + 별도 tool offset 산출물이 유리하지만, OMX_F 는 gripper 영구 부착 + DIY 환경에서 도구 swap 비현실적 → 분리 안 함. 미래에 도구 추가 필요해지면 그때 별도 산출물 도입 검토.

---

## 3. DIY 환경에서 정확도 짜내기 — 전략 프레임워크

### 3.1 출발점: 완전 분해는 불가능

DIY 환경에는 오차원인이 복합적으로 얽혀있음:

- Dynamixel 백래시 + 인코더 양자화 + 자세별 토크 변형 (XL430/XL330 hobby 급)
- 3D 프린트 휨 + 조립 mm 단위 오차
- sag (현재 J2/J3 만 모델링, 잔재 다수)
- 케이블 텐션, 마운트 회전, thermal drift, ...

이걸 산업 환경처럼 **하나씩 분해해서 따로 캘** 하려면 CMM / 레이저 트래커 / 정밀 지그 필요. **DIY 에선 불가**.

→ 전략을 바꿔야 함. 다음 세 가지 인사이트.

### 3.2 인사이트 1 — 카메라가 너의 지그다

D405 가 산업용 측정기 역할을 함. depth 정확도 mm 단위 + RGB 코너 검출 sub-pixel.
산업에서 CMM 가 하는 일을 DIY 에선 **카메라가 함**.

이미 부분적으로 하고 있음 — extended BA + 물리 sag 가 카메라 데이터로 캘 짜낸 것
([calibration.md](calibration.md)). 이걸 더 깊이 쓰는 게 다음 단계.

### 3.3 인사이트 2 — 물리 모델 + **잔차 학습** (empirical residual) — **실측 후 폐기**

**2026-05-28 update**: 41 자세에서 LOO RBF 시험 결과 효과 없음 (§4 #3 참조).
정상 baseline σ_t 7.77mm 에서 hold-out σ_t 8.33mm 로 오히려 악화, 잔차 ↔
joint angle 상관 모두 \|corr\|<0.3. BA 가 이미 다 짜냈다는 의미.
**이 인사이트는 폐기**. mm 미만 정확도 필요시 §3.4 visual servoing 으로.

(아래는 시도 전 설계 기록 — 참고용)

캘 4종은 **구조적 보정** — 각 자유도가 물리적 의미 가짐 (joint zero, link geom, sag).
**기존 캘 4종은 그대로 frozen, 절대 재캘 안 함** (`joint_offset.npz` / `link_offset.npz` /
`sag.npz` / `hand_eye.npz` 모두 그대로). 그 위에 **5번째 산출물 `residual.npz`** 만 추가.
잔차를 통째로 학습시킴:

```
명령 (q1..q5)
    ↓
[기존 캘 4종 적용 — 그대로]
    - joint_offset 차감
    - link_offset patched URDF
    - sag 보정
    - hand_eye matrix
    ↓
Kinematics.fk(q) = 물리모델 예측 EE 위치
    ↓
+ learned_residual(q)   ← 신규 (residual.npz)
    ↓
= 실제 EE 예측 위치 (더 정확)

학습 phase (오프라인):
  - 자세 200~500개 캡처
  - 잔차 = (카메라가 본 실제) - (Kinematics.fk 예측)
  - smooth regressor (GP / 작은 NN / RBF) 로 q-space 보간
  - residual.npz 로 저장
IK 도 역방향으로 합성
```

**무엇이 흡수되나** — 모델링 안 한 sag 잔재 (J4/J5), 백래시 평균값, 3D 프린트
휨의 자세 의존, 케이블 텐션 패턴, **"분리해서 못 잡는 모든 자세 의존 오차"**.

**데이터 양** — 자세 200\~500개로 충분 (sub-cm residual smooth 가정 시).
자동으로 워크스페이스 격자 돌면서 캡처, BA 와 같은 인프라 재사용.

**왜 RL 이 아닌가** — 이건 **지도 학습** 문제. 정답(카메라 측정)이 모든 샘플에
있음. RL 의 exploration / credit assignment / 시퀀셜 결정 부담 없음. RL 쓰면
sample efficiency 가 100\~1000배 나빠짐 — DIY 데이터 양으로 못 함. 정답이 직접
주어지는데 trial-and-error 하면 안 됨.

#### 잔차 학습의 실패 모드 vs BA 자유도 확장의 실패 모드 — 헷갈리지 말 것

자유도 늘리는 것 = 위험, 이라는 직관은 valid 하지만 두 가지가 다른 메커니즘으로
망함. 같이 묶으면 잘못된 방어책을 씀.

| | BA 자유도 확장 (§4 #2 같은 것) | 잔차 학습 (§4 #3) |
|--|--|--|
| 무엇을 fit | 물리 파라미터 (joint/link/hand-eye) **동시** | 물리 모델은 **frozen**, 잔차만 |
| 파라미터끼리 경쟁? | **예** — link 줄이고 hand-eye 늘리면 같은 EE → gauge freedom | **아니오** — 물리 파라미터 안 건드림. leftover 회귀 |
| 주된 실패 모드 | **비물리값에 정착** ([extended_ba §5](calibration.md)). σ는 작지만 generalize 안 함 | **overfit** + **extrapolation** (학습 분포 밖에서 신뢰 X) |
| 방어책 | regularization sweep + hold-out σ + 파라미터 sanity check | smoothing prior (GP 자체가 smooth) + train/test split + repeatability 와 비교 (이 floor 아래로 fit 시키지 않기) |

→ 결론: 잔차 학습이 BA 의 gauge freedom 문제를 **재현하지 않음**. 단 자기 고유의
실패 모드가 있으니 별도 방어 (특히 workspace coverage + repeatability 한계 인지)
필요. 학습 후 σ 가 repeatability floor 근처면 더 fit 시키지 말기 — 그 이상은
noise overfit.

### 3.4 인사이트 3 — Closed-loop 으로 마무리 (필요한 task 에 한해)

§3.2 + §3.3 다 해도 absolute floor 는 **모터 repeatability**. XL430/XL330
hobby 급은 보통 absolute 정확도보다 repeatability 가 나음 — 같은 명령 두 번 →
같은 자리 (편향은 있어도 분산은 작음).

→ **편향(bias)은 (캘 + 잔차)가 잡고, repeatability 가 hard floor.**

그 floor 보다 더 가야 하는 task 는 **open-loop 으로 풀 수 없음**. 어떤 캘도
못 해결. → **visual servoing** 으로 마지막 1cm 만 카메라 보면서 close-loop.
산업 로봇도 이렇게 함.

**pick task 적용 예:**
- 거친 접근 (cm 단위): 캘 + 잔차로 TCP 갖다 댐
- 최종 descent (mm 단위): 카메라가 객체 보면서 EE 위치 보정
- → RL 아님. 고전 PBVS/IBVS 기법으로 충분

### 3.5 RL 의 위치 — 언제 의미 있나

| 풀려는 문제 | 적합한 도구 |
|--|--|
| "캘 4종 위에 잔차 잡기" | **지도 학습** (정답 = 카메라 측정) |
| "잡기 자체 성공률 올리기" (목표 정확도 부족 보완) | **Visual servoing** (closed-loop) |
| "스킬 자체를 학습" (흔들리는 물체 잡기, 새 도구 사용 적응) | **RL 의미 있음** |

RL 이 빛나는 데는 **시퀀셜 결정 + sparse reward + 모델 없음** 인 상황.
OMX_F 정확도 문제는 셋 다 아님 → RL 부적합.

---

## 4. Squeeze 축 — 4종 캘 위에 더 짜낼 곳

현재 잡혀있는 것:

| 산출물 | 어디서 적용 | 상태 |
|--|--|--|
| intrinsic | `cv2.undistortPoints` | ✓ |
| hand_eye | Detector + Frontend PC layer | ✓ |
| joint_offset | raw↔rad 변환 양쪽 | ✓ |
| link_offset | URDF patch (PyBullet 로드) | ✓ |
| sag_offset | Kinematics fk/ik 양방향 | J2/J3 만 |

위에 더 들어갈 후보들. **효과 큰 × 비용 작은** 순:

| # | 후보 | 무엇 | 효과 추정 | 비용 |
|--|--|--|--|--|
| 1 | **repeatability floor 측정** | 같은 명령 N회 → 분산 측정. 모든 squeeze 의 baseline (이 floor 보다 잘 짜내려는 시도는 무의미) | (측정만) | 30분 |
| 2 | **link_offset BA 자유도 확장 → `tcp_joint` xyz 포함** | URDF 의 92mm 가 실제 그리퍼 끝점 (핑거 닫혔을 때 만나는 점) 과 안 맞는 부분을 BA 가 같이 풀게. 별도 산출물 추가 X, 기존 link_offset 캘 흐름 안에서 처리. 진단이 C 로 나오면 1순위 fix. **적용 layer 변경 필요**: 현재 [urdf_patcher.py](../backend/core/coords/urdf_patcher.py) 의 `_default_joint_id_map` 은 joint1\~5 만 알기 때문에 `tcp_joint` 도 처리하도록 확장 필요 + `LinkOffsets` 자료 구조에 fixed joint 표현 추가. **⚠️ 자유도 추가 = gauge freedom 위험 ([extended_ba §5](calibration.md))** — hand-eye t 와 swap 가능. regularization 재튜닝 + hold-out + sanity check 필수 | mm 단위 (C 결판 시) | 1\~2시간 (BA + LinkOffsets + patcher map + reg sweep + hold-out) |
| 3 | ~~**empirical residual 학습 layer**~~ — **실측 후 효과 없음 확정** | 2026-05-28 [analyze_residuals.py](../backend/scripts/analyze_residuals.py) 로 LOO RBF 시험. **올바른 baseline σ_t 7.77mm 에서 hold-out σ_t 8.33mm (악화)**. 잔차 ↔ joint angle 상관도 모두 \|corr\|<0.3 — 자세 의존 시그널 없음. BA 가 이미 다 짜냄. **이 후보는 폐기.** mm 미만 정확도 필요시 §3.4 visual servoing 으로. | 효과 없음 (실측 확정) | — |
| 4 | **백래시 방향성 보정** | XL430/XL330 둘 다 backlash 존재. 같은 각도여도 CW 접근 vs CCW 접근에 raw 다름. direction-dependent offset | sub-degree, 끝단 mm 단위 | 측정 1시간 + raw 변환 layer 패치 (#3 의 잔차 학습이 흡수할 수도 — 중복 검증 필요) |
| 5 | **sag 모델 J4/J5 확장** | 현재 J2/J3 만. J4(wrist roll) 자세에 따라 J5 처짐 모멘트 바뀜 | sub-mm \~ mm | 데이터 캡처 + 모델 확장 (#3 잔차 학습이 흡수할 수도) |
| 6 | **D405 depth bias** | per-pixel / per-distance systematic bias. plane fit 으로 추출 → lookup table. detection / ICP / TSDF 모두 영향 | mm 단위 (특히 ICP/grasp) | 측정 1시간 + 보정 layer |
| 7 | **thermal drift** | XL430/XL330 발열 후 zero drift. 워밍업 protocol + cold/warm 캘 비교 | sub-degree | 측정 1시간 |
| 8 | **BA 자세 분포 개선** | 현재 BA 자세가 워크스페이스 cover 충분한지. residual 의 spatial pattern 보면 즉시 진단 | 모서리 자세에서 mm 단위 | 분석 30분 |
| 9 | **static settle 보장** | trajectory 끝 후 settle 시간 부족하면 잔진동 중 측정으로 노이즈. 이미 잡혀있을 수도 | 노이즈 floor | 확인 30분 |
| 10 | **Visual servoing (closed-loop)** (§3.4) | open-loop 한계(repeatability floor) 아래로 가야 하는 task 에만 적용. 마지막 1cm 카메라 보면서 보정. RL 아님 — PBVS/IBVS 고전 기법 | task 의존 (sub-mm 가능) | task 별로 끼움. pick descent 가 첫 적용 후보 |

> #4, #5 는 #3 의 잔차 학습이 결과적으로 흡수할 수 있음. #3 적용 후 잔차 패턴이 "방향 의존" 또는 "J4/J5 자세 의존" 으로 명확히 남으면 그때 별도 모델로 분리. 안 남으면 그냥 #3 안에 두고 진행. **분리 캘은 측정으로 정당화될 때만**.

---

## 5. 권장 진행 순서

> 2026-05-28 갱신. 잔차 학습 (§4 #3) 은 실측으로 효과 없음 확인되어 폐기.
> commit 버그 (§1.6) 는 영구 fix 완료. 정상 system floor σ_t 7.77mm.

1. **§1 큐브 진단** — 어차피 막힌 일. 표 한 줄 채워서 A/B/C 결판.
2. **repeatability 측정** — § 1 진단 돌리는 김에 같은 명령 N회 반복으로 분산도 같이 잡음. **이 값이 모든 후속 squeeze 의 hard floor**.
3. **진단이 C 면** §4 #2 (link_offset BA 자유도 확장). **A/B 면** depth 샘플링 / ring 픽셀 추정 개선. 끝나고 σ 재측정.
4. **구조적 squeeze 후보들** — §4 #4 (백래시), #5 (sag J4/J5), #6 (D405 depth bias), #7 (thermal), #8 (BA 자세 분포). 잔차 학습 폐기됐으니 이 구조적 후보들로 직접 짜내야 함. 측정 후 효과 큰 것부터.
5. **mm 미만 정확도 필요 task** — visual servoing (§3.4, §4 #10) 으로 closed-loop. open-loop 의 floor (~7.77mm) 아래로 내려가는 유일한 길.
6. 이후로는 **데이터 driven** — 매번 새 캘/보정 적용 후 σ 측정. 가장 큰 잔차를 만드는 다음 후보부터.

---

## 6. 작업 원칙

- **측정 없이 다음 캘 만들지 말기.** "할 수 있는 것" 말고 "**현재 가장 큰 잔차를 만드는 것**" 잡기.
- **물리 모델 vs 잔차 학습의 분업**: 물리적 의미 있고 측정으로 따로 분리되는 것만 별도 캘로. 분리 안 되는 잔차는 무리해서 모델링 말고 §3.3 잔차 학습 layer 가 흡수. 새 분리 캘은 "잔차 패턴이 그렇게 생겼다" 는 측정 근거 있을 때만.
- 캘 산출물 늘릴 때는 항상 **무엇을 보정 / 어디서 적용 / COMMIT 후 어디까지 자동 반영** 셋 다 [calibration_apply_flow.md](calibration_apply_flow.md) 표에 추가.
- 새 보정 도입 시 BA 잔차 / σ_rot / σ_t 가 실제로 떨어지는지 검증. 안 떨어지면 그 보정은 의미 없는 거 — 코드에 박지 말고 빼기.
- **자유도 늘리는 보정에는 항상 hold-out + sanity check** (§3.3 박스 참조). σ 떨어졌어도 (a) 학습 안 한 자세에서도 떨어지는가, (b) 파라미터 값이 물리적으로 합리적인가 둘 다 통과해야 채택. [extended_ba §5, §12](calibration.md) 의 교훈.
- **Open-loop floor 인정**: 모터 repeatability 가 open-loop 의 hard floor. 그 아래 필요한 정확도는 closed-loop (§3.4) 으로만 도달 가능. 캘 추가로 못 뚫는 벽.
- 5DOF 제약 인지: 도구 축이 직선인 작업(gripper, 노즐)은 자유, 도구 회전 자유도 필요한 작업은 reachable workspace 좁아짐.
- **RL 은 정확도 짜내기에 부적합** (§3.5). 정확도는 지도 학습 / closed-loop 영역. RL 은 스킬 학습용.

---

## 7. 관련 문서

- [calibration.md](calibration.md) — 현재 floor 0.647°/7.77mm 도달 과정
- [calibration_apply_flow.md](calibration_apply_flow.md) — 4종 산출물의 적용 메커니즘
- [calibration.md](calibration.md) — 캡처 절차 + 결과 해석
- [hardware.md](hardware.md) — 모터/링크/3D프린트 토폴로지
- [pick_and_place_walkthrough.md](pick_and_place_walkthrough.md) — 현재 task 흐름
