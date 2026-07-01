# Calibration Module — boundary spec (v2 Step E 진입 전 설계 확정)

> **상태**: 코드 진입 전 boundary 확정 목적 초안 (2026-07-01). **§10 두 자리 방향 재판단 후 문서 반영 완료. 사용자 최종 리뷰 대기 (2026-07-02 이어서 논의).** SSOT spec = [backend_v2.md](backend_v2.md) + [backend_v2_modules.md](backend_v2_modules.md). 본 문서 = 산발 spec 통합 + Repository / Service / Mirror 계약 신규 확정.
>
> **원칙 (오늘 박은 것)**: Storage Module 자리 없음 (v2 §9 폐기 확정). 각 domain module 이 자기 영속성 owner. Repository / ObjectStore 는 framework Protocol (이미 존재), 도메인 module 이 자기 Repository impl 소유 (Database-per-Module + DIP).
>
> **오늘 세션 과정 (2026-07-01)**: 처음 §10 에 옛 backend 자산 그대로 복사 → 사용자 지적 "옛 자산은 인사이트 원천, v2 원칙 대조 없이 이월 X". §10.1 (factory intrinsic) 은 옛 Camera→Storage RPC write 방향이 v2 Owner/Reader 비대칭 위반, §10.2 (URDF reload) 는 옛 restart-only 가 v2 Mirror real-time 원칙 위반. 두 자리 v2 원칙 기준 재판단해서 반영.

## 0. TL;DR

Calibration Module = **5 종 산출물 Bundle owner (intrinsic / hand_eye / joint_offset / link_offset / sag_offset)** + capture loop + Bundle 발행. Backend 는 capture 수집 + storage append 만, BA / σ / observability 는 **offline 스크립트** (옛 backend 결정 이월).

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

**Blob key convention**: `calibration/<robot_id>/runs/<run_id>/captures/<pose_index>/primary.bin` (옛 backend 컨벤션 그대로 이월).

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
| `srv/calibration/{robot_id}/snapshot_bundle` | 현재 active 5 kind bundle 반환 (Mirror initial fetch 도 이걸로) | `{}` → `CalibrationBundle` |
| `srv/calibration/{robot_id}/list_runs` | run history | `{kind?}` → `[CalibrationRun]` |
| `srv/calibration/{robot_id}/list_results` | result history (rollback UI 자리) | `{kind?}` → `[CalibrationResult]` |
| `srv/calibration/{robot_id}/get_thresholds` | Traffic Light 임계 (min/max tilt / pose diversity 등) | `{}` → `CalibrationThresholds` |

### 4.3 Streams (output)

| Stream key | 방향 | payload |
|---|---|---|
| `stream/calibration/{robot_id}/preview` | Calibration → frontend (5Hz) | ChArUco detected overlay + traffic light verdict + capture_verdict/reasons |

### 4.4 Subscribers (input — Calibration 이 다른 module event 구독)

| Subscribed event | 발행자 | 처리 | 근거 |
|---|---|---|---|
| `event/camera/{robot_id}/factory_intrinsic_available` | CameraDriver (boot 시 1회) | idempotent seed: `get_active(robot_id, "intrinsic")` 없으면 create_run + save_result(payload) + activate_result. 이미 있으면 skip (사용자 chessboard 캘 결과 덮어쓰지 않음) | §10.1 — Camera 가 SDK 데이터를 자기 event 로 publish, Calibration 이 자기 domain rule 로 seed. Owner/Reader 비대칭 준수 |

## 5. Events (@publishes)

| Event key | 발행 시점 | payload |
|---|---|---|
| `event/calibration/{robot_id}/activated` | `activate_result` 성공 시 (activate 또는 rollback) | `{robot_id, bundle_id, version}` (anchor #14 versioning) |
| `event/calibration/{robot_id}/committed` | `finalize_run` 성공 시 | `{robot_id, run_id}` |

**versioning 원칙** (§10 anchor #14): `bundle_id` = DB row id, `version` = monotonic. Consumer (Motion) 가 어느 bundle 위에서 kinematics rebuild 박았는지 추적 가능.

## 6. Mirror

### 6.1 `Mirror[CalibrationBundle]` — active 5 kind snapshot

```python
class CalibrationBundle(BaseModel):
    """모든 5 종의 현재 active row. Mirror payload = atomic snapshot."""
    robot_id: str
    bundle_id: int  # DB row id (activate 마다 새 값)
    version: int    # monotonic

    intrinsic: IntrinsicResult | None
    hand_eye: HandEyeResult | None
    joint_offset: JointOffsetResult | None
    link_offset: LinkOffsetResult | None
    sag_offset: SagOffsetResult | None
```

### 6.2 Reader 계약

| Reader Module | 왜 필요 | 어떤 field 씀 | 트리거 시 반응 |
|---|---|---|---|
| **Motion** | kinematics rebuild (link_offset URDF patch + sag decorator) | `link_offset`, `sag_offset`, `joint_offset` | **real-time reload** (§10.2). `link_offset` 변경 시 `PybulletKinematics.reinitialize(link_offsets)` — internal disconnect + patched URDF 재로드 (`_lock` 안, in-flight fk/ik 는 lock wait ~수 ms). `sag_offset` 변경 시 `SagCorrectedKinematics.set_offsets`. `joint_offset` 변경 시 units.raw↔rad 변환 갱신 |
| **Detector** (Step F+) | detection base-frame 변환 | `intrinsic`, `hand_eye` | 재구독만 (state cache 갱신) |
| **Scan** (Step H+) | scan capture 시 intrinsic seed + hand_eye | `intrinsic`, `hand_eye` | 다음 capture 부터 새 값 사용 |
| **Reconstruction** (Step I+) | build 시 intrinsic seed | `intrinsic`, `hand_eye`, `joint_offset` | 매 build 시 fresh fetch (Mirror local 캐시 대신 매번) — heavy compute 자리, mirror 지연 vs 최신성 tradeoff 자연 |

### 6.3 Mirror lifecycle (§3.3.1)

- Consumer boot 시 `snapshot_bundle` service call → 초기 fetch
- `CalibrationActivated` event 구독 → Bundle invalidate + refetch
- Refetch coalescing (동일 version 내 다중 event = 1 회 fetch) — framework level (latent, D4 자리)

## 7. Migration mapping (옛 backend → v2)

### 7.1 자산 재배치

| 옛 위치 | 자산 | v2 위치 | 형태 |
|---|---|---|---|
| `backend/modules/calibration/orm.py` | 4 entity ORM (Base metadata) | `modules/calibration/orm.py` | 이월 (schema 동일, `Base` 는 module 자기 것) |
| `backend/modules/calibration/persistence_models.py` | wire pydantic | `modules/calibration/contract.py` | wire ↔ ORM 변환 |
| `backend/modules/calibration/result_models.py` | 5 kind result dataclass | `modules/calibration/contract.py` | Bundle payload 로 재사용 |
| `backend/modules/calibration/board.py` | ChArUco detect SSOT | `modules/calibration/board.py` | 이월 (도메인 logic, 계약 무관) |
| `backend/modules/calibration/capture_quality.py` | Phase 1 Traffic Light | `modules/calibration/capture_quality.py` | 이월 |
| `backend/modules/calibration/thresholds.py` | tilt / PnP RMS / pose diversity 임계 | `modules/calibration/thresholds.py` | 이월 |
| `backend/modules/calibration/se3.py` | SE(3) math helper | `modules/calibration/se3.py` | 이월 |
| `backend/modules/calibration/sim_board.py` | ChArUco 시뮬 (test) | `tests/fixtures/sim_board.py` 또는 `modules/calibration/sim_board.py` | 이월 |
| `backend/modules/calibration/intrinsic.py` | intrinsic calibrate 로직 | `modules/calibration/intrinsic.py` | 이월 (offline logic 이지만 backend 도 함께 사용) |
| `backend/modules/calibration/applier.py` | 캘 → kinematics 적용 | **Motion 안 Mirror consumer 로 재배치** | Motion 이 Mirror event 받아 자기 kinematics rebuild — Calibration 이 apply 책임 X (boundary 정정) |
| `backend/modules/calibration/loader.py` | storage 에서 캘 로드 | Repository.get_active_bundle 로 흡수 | 폐기 (Repository 가 흡수) |
| `backend/modules/calibration/calibration_cache.py` | in-memory snapshot cache | Mirror consumer 자리로 흡수 | 폐기 (Mirror pattern 이 cache 겸함) |
| `backend/modules/calibration/storage_client.py` | Storage Module RPC client | **폐기** (Storage Module 자체 폐기) | 직접 Repository 호출로 대체 |
| `backend/scripts/calibrate_offline.py` | 5 stage BA + LOOCV + IRLS + observability | `backend_v2/scripts/calibrate_offline.py` | 이월 (backend 프로세스 밖 offline 도구 — v2 도 동일 위치) |

### 7.2 폐기 자리 정리

- `storage_client.py` — Storage Module RPC → 직접 Repository (Database-per-Module)
- `loader.py` — Repository.get_active_bundle 로 흡수 (별도 loader 자리 불필요)
- `calibration_cache.py` — Mirror 가 cache 자체 (별도 in-memory 자리 불필요)
- `applier.py` — 책임 이관 (Calibration 이 apply X, Motion 이 Mirror consumer 로 self-apply)

### 7.3 새로 짜는 자리

- `modules/calibration/repository.py` — CalibrationRepository (§2 spec)
- `modules/calibration/module.py` — Module class (@service / @publishes / @subscriber decorators, Repository + ObjectStore constructor 주입)
- `modules/calibration/alembic/` — Alembic per-module migration (versions/env.py/alembic.ini)
- `tests/fixtures/mock_calibration_owner.py` — Step D Motion 검증 자리에서 이미 쓰이던 mock fixture 는 Step E 진입 후 real Calibration 로 대체

## 8. Migration owner (Alembic per-module)

옛: 단일 Alembic in `backend/modules/storage/` — 모든 도메인 함께 upgrade.  
v2: 각 module 이 자기 Alembic dir 소유. Module start() 시 자기 Alembic `upgrade head`.

```
modules/calibration/
    alembic/
        env.py
        versions/
    alembic.ini
```

Postgres 공유 시 각 module 이 자기 table 만 만듦 → schema 충돌 0.

## 9. Motion reader — 실 wire

Step E 진입 시 Step D 자리 mock owner (tests/fixtures/mock_calibration_owner.py) 제거 + 실 Calibration 로 e2e 검증. 이 e2e 가 Step E 검증의 핵심 ([backend_v2_modules.md §11.2](backend_v2_modules.md) 표):

> "Bundle atomic + Mirror[Bundle] event broadcast 의 진짜 e2e (Motion 의 kinematics rebuild 자리)"

즉 시나리오:
1. offline `calibrate_offline.py --commit` → CalibrationResult INSERT + activate → `CalibrationActivated` event
2. Motion 이 event 구독 → Mirror refetch (`snapshot_bundle` service call) → Bundle 새로 받음
3. Motion 이 kinematics rebuild — `link_offset` 변경 시 URDF patch + PyBullet 재로드, `sag_offset` 변경 시 SagCorrectedKinematics 재설정, `joint_offset` 변경 시 raw↔rad 갱신
4. 새 kinematics 로 다음 fk/ik 자리 반영 확인

## 10. Decisions (옛 backend 실 자산 근거)

### 10.1 Factory intrinsic seed = Camera event publish → Calibration subscribe

**옛 자산 인사이트**: [backend/modules/camera/factory_intrinsic.py](../backend/modules/camera/factory_intrinsic.py) — Camera 가 pyrealsense2 pipeline 을 SDK internal 접근용으로 소유. Boot 시 자기 pipeline 잠깐 open → factory intrinsic fetch → Storage Module RPC commit. Idempotent (이미 active 있으면 skip).

**옛 자산이 왜 그렇게 짰나**: Camera 가 SDK 접근 owner (host: pi_camera 자리 자연). 저장은 Storage Module (모두의 데이터 hub) 이 owner. 두 module 협력 자리 Camera → Storage direct RPC write.

**v2 원칙 대조**:
- **Storage Module 폐기** — 이제 Calibration 이 intrinsic table owner (Database-per-Module + Owner/Reader 비대칭, [backend_v2.md §2.3](backend_v2.md))
- Owner/Reader 원칙 = "다른 module 이 owner 의 데이터를 write 하지 못함"
- 옛 방향 그대로 (Camera → Calibration service write call) 이월 = **Owner/Reader 위반**

**v2 결정**: 방향 뒤집기 — Camera 가 factory intrinsic 을 자기 domain event 로 publish, Calibration 이 subscribe 해서 자기 domain rule (idempotent seed) 적용.

```python
# Camera side — CameraDriver Module (Step B 확장, host: pi_camera or pc mock)
class CameraDriverModule:
    class Event(StrEnum):
        FACTORY_INTRINSIC_AVAILABLE = "event/camera/{robot_id}/factory_intrinsic_available"

    async def start(self):
        self._driver.open()
        if isinstance(self._driver, RealSenseD405Driver):
            intr = self._driver.get_factory_intrinsic()
            # boot 시 1회 publish — subscribers (Calibration) 가 자기 rule 로 처리
            self.runtime.publish(
                self.Event.FACTORY_INTRINSIC_AVAILABLE,
                FactoryIntrinsicAvailable(robot_id=self.robot_id, intrinsic=intr),
            )

# Calibration side
@subscriber(CameraDriver.Event.FACTORY_INTRINSIC_AVAILABLE)
def on_factory_intrinsic(self, event: FactoryIntrinsicAvailable) -> None:
    """idempotent — 이미 active 있으면 skip. domain rule 은 Calibration 소유."""
    if self._repository.get_active(event.robot_id, "intrinsic"):
        return  # 사용자 chessboard 캘 결과 덮어쓰지 않음
    run_id = self._repository.create_run(event.robot_id, "intrinsic", "d405_factory")
    result_id = self._repository.save_result(run_id, event.intrinsic)
    self._repository.finalize_run(run_id, "success")
    self._repository.activate_result(result_id)
```

**Ownership 유지**: Camera 는 자기 SDK 데이터를 event 로 노출 (자기 domain). Calibration 은 그 event 를 자기 domain rule 로 소비 (intrinsic table 소유). Write 방향 뒤집힘 없음. **옛 자산의 인사이트** (SDK 접근 owner ≠ 저장 owner, idempotent seed) 는 유지, **방향** 만 v2 원칙에 맞게 재설계.

미해결 자리: boot 순서 — Camera 가 event publish 시 Calibration 이 subscribe 준비 안 됐으면 event 유실. Zenoh 는 event 자리 late subscriber 에게 replay 안 함. 처리 방안 (내일 논의):
- **A**. Calibration 이 자기 start() 안에서 Camera 에 pull service 호출 (`GET_FACTORY_INTRINSIC`) — Camera 가 internal service 로 노출, 매 요청 시 SDK 짧게 open. Owner/Reader 반대 방향 (Calibration 이 caller, Camera 는 read-only publisher). 이건 §7.7 anchor #13 위반 소지 (Camera public service 에 intrinsic 노출 자리) — 단 internal 만 (FRONTEND_EXPOSED X) 이면 회피 가능?
- **B**. Camera 가 periodic republish (예: 30초마다) — 하지만 factory intrinsic 은 static 값이라 낭비
- **C**. Framework Mirror 자리 확장 — `Mirror` 는 원래 boot 시 snapshot fetch + event refetch 패턴. 여기 응용해서 Calibration 이 boot 시 `Camera.Service.GET_FACTORY_INTRINSIC` snapshot pull + event 로 갱신. 이게 Mirror 원칙과 자연 맞음

**내일 결정 자리** — A / B / C 중 하나.

### 10.2 Motion 의 URDF 재로드 = real-time reinitialize (v2 Mirror 원칙 준수)

**옛 자산 인사이트**: [backend/modules/calibration/applier.py](../backend/modules/calibration/applier.py) — "부팅 시 1회 apply, link_offset 은 backend restart" 원칙. PyBullet API 상 URDF 재로드는 client disconnect+reconnect+reload 자리 복잡. 옛 backend 는 이 복잡성 피하려고 restart-only 로 회피.

**옛 자산이 왜 그렇게 짰나**: 옛 backend 의 caliration reload 는 그리 잦지 X (사용자가 캘 결과 commit 하는 시점만). Restart 이 실용상 OK. 복잡성 자리 명시적으로 회피 결정 ([storage_layer.md §7 원칙 5](storage_layer.md)).

**v2 원칙 대조**:
- **Mirror = control correctness state real-time 반영** ([backend_v2_modules.md §3.2](backend_v2_modules.md)) — "link_offset 변경 시 kinematics rebuild"
- v2 는 Mirror 를 real-time 계약으로 박음 — event 발생 시 즉시 반영이 원칙
- 옛 restart-only 이월 = **Mirror 원칙 위반**. restart 요구는 v2 가 지향하는 self-healing wire 와 상충
- v2 재설계 phase 이므로 복잡성 감수하고 real-time reload 정석으로 짜는 게 자연

**v2 결정**: `PybulletKinematics.reinitialize(link_offsets)` internal method.
- `_lock` 안: `p.disconnect(client)` → new client `p.connect(DIRECT)` → patched URDF 재로드 → joint indices / chain 재계산
- 재로드 중 in-flight fk/ik 호출은 `_lock` wait (수 ms stall) — jog / trajectory 짧게 정지
- Motion 의 Mirror consumer 는 `link_offset` 변경 감지 시 이 method 호출 (module restart X)
- `joint_offset` / `sag_offset` 은 그대로 `set_offsets` idempotent (in-memory)

**옛 자산의 인사이트** (URDF 재로드 = disconnect+reconnect+reload 자리 복잡) 는 v2 도 사실. 다만 **v2 원칙 (Mirror real-time)** 이 이 복잡성보다 우선. 실측 stall 수 ms 는 jog / trajectory 자리 acceptable (사용자가 캘 commit 하는 순간 motion 짧게 정지 자연).

미해결 자리: `_lock` 잡은 채 PyBullet client 교체 시 다른 스레드에서 잡고 있는 자원 (e.g. TrajectoryRunner 안 캐시된 kinematics ref) 재검증 필요 — Motion 측 reader 구현 시 확인.

### 10.3 Preview 5Hz stream 부하 = 옛 자산 검증 완료, 이월

**옛 자산 인사이트**: [backend/nodes/application/calibration_node.py](../backend/nodes/application/calibration_node.py) 의 `preview_loop` 이미 실 hardware (D405 + SO-101) 자리 검증 완료 ([docs/calibration_workflow.md](calibration_workflow.md) + CLAUDE.md § "자동 BA + σ live (2026-06-10)" — capture 후 자동 preview / traffic light 자리 실 사용).

**v2 원칙 대조**: state stream 5Hz publish 는 [backend_v2.md §3.2](backend_v2.md) stream 원칙과 자연 정합. 별 원칙 상충 없음.

**v2 결정**: 5Hz 그대로 이월. ChArUco detect + traffic light + preview payload publish. 이건 옛 자산의 구현 detail 을 그대로 재사용해도 v2 원칙 위반 없음 자리 (드문 경우 — architectural 결정 아니고 실측 tuning 자리).

## 11. 다음 단계

이 문서 승인 후 코드 진입 순서 (§10 재판단 반영):
1. `modules/calibration/orm.py` + `contract.py` (entity + wire pydantic 이월 — 옛 `orm.py` + `persistence_models.py` + `result_models.py` 자산)
2. `modules/calibration/repository.py` (§2 shape)
3. `modules/calibration/alembic/` (initial migration — 옛 [db_schema.md §2](db_schema.md) 컨벤션 이월)
4. `modules/calibration/module.py` (@service + @publishes + @subscriber. **factory intrinsic 처리는 §10.1 미해결 자리 결정 후** — A/B/C 중 어느 방향?)
5. `apps/registry.py` + `apps/resolve.py` 에 calibration 등재
6. `tests/modules/test_calibration.py` — Repository + Bundle atomic + Mirror event 계약 test + factory intrinsic seed idempotent test
7. **Camera Module 확장** (Step B 재진입): `RealSenseD405Driver.get_factory_intrinsic()` method 추가. Wire 자리는 §10.1 결정 (event publish vs internal service) 따라 갈림
8. Motion 에 Mirror consumer wire (mock owner 제거, 실 Calibration e2e). **`link_offset` real-time reload 를 위한 `PybulletKinematics.reinitialize()` 추가** (§10.2). Bundle 갱신 시 field 별 분기 — joint/sag 은 `set_offsets` in-memory, link_offset 은 `reinitialize`
9. `scripts/calibrate_offline.py` 이월 (BA/IRLS/observability — 옛 backend 스크립트 그대로 v2 로 복사, DB session_factory + ObjectStore 만 재배선)
10. frontend 자리 계약 재생성 + Calibration panel 이식 (별도 단위)

## 12. 내일 (2026-07-02) 논의 시작점

**리뷰 대기 자리**:
- **§10.1 미해결 A/B/C**: factory intrinsic seed 시 Camera event publish 자리 boot 순서 문제 (Calibration subscribe 준비 전 event 유실). A (Calibration → Camera pull) / B (periodic republish) / C (Mirror 패턴 응용) 중 선택
- **§10.2 미해결**: `_lock` 잡은 채 PyBullet client 교체 시 다른 스레드 자원 재검증 필요 (Motion 측 reader 구현 시 확인)
- **§1-§9 전체**: v2 원칙 정합 다시 대조 (오늘 §10 두 자리 이월 실수 반복 방지)
- **§11 순서**: 위 결정 반영 후 실 코드 진입 순서 확정
