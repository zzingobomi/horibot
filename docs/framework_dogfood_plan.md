# Node Framework Dogfood Plan

> 본 문서는 [architecture_review_protocol.md](architecture_review_protocol.md) 의 검토 phase 첫 큰 산출물.
> **2026-06-25 update — §15 Runtime-centric reframe.** §14 까지의 plan 은 Node 가 최소 단위라는 잘못된 전제. 진짜 깨달음 = Runtime (Process) 이 최소 단위, Module = 기능 묶음. backend_v2/ 폴더 삭제, §15 위에 다시 짬.
> 새 세션에서 "framework 진행하자" / "Runtime" / "Module" / "Transport adapter" 톤 던지면 본 문서 진입.
> 결정된 것만 정리. 미정 항목은 §9.

## 1. 배경

검토 phase 진입 직후 사용자가 짚은 본질:

> "레이어 분리는 잘 됐는데 사람 (나 또는 추후 개발자) 이 코드 이해/파악이 너무 어렵다"

원인 분해 (사용자 + Claude + GPT 공동 reframing):

1. **기능 추적 비용** — 한 wire 추가 시 `messages` → `topic_map` → `api_contract` → handler → repo → client → frontend store → component 까지 N 파일 횡단
2. **반복 boilerplate** — schema + topic 등록 + contract 등재 + handler + client wrapper + frontend gen 패턴 매번 복붙. "작은 RPC 프레임워크를 손으로 쓰는 형태"
3. **Storage Node 책임 침범** — 워크플로우 단계 (run finalize / result activate 등) 가 storage service 안에 들어와 있음. CRUD 인프라가 도메인 로직 흡수

## 2. 3대 방향

| # | 방향 | 채택 |
|---|---|---|
| 1 | 미니 framework (boilerplate 제거, FastAPI DX 미러) | ✅ |
| 2 | system-docs 자동 생성 (노드별 service/topic/publish 한눈) | ✅ |
| 3 | Storage = CRUD only, Workflow = 도메인 노드 | ✅ (경계는 case-by-case 합의 후 진입) |

## 3. 설계 원칙 (다음 세션도 유지)

1. **매직 스트링 금지** — `Service` / `Topic` enum SSOT 유지. 데코레이터 인자는 enum *referent*, 새 string SSOT 신규 X
2. **데코레이터 = binding 메타** — 기존 SSOT (enum + Pydantic message) 들을 함수에 묶는 역할. 새 SSOT 만들지 X
3. **목표 = "transport boilerplate 몰라도 domain 코드만 작성"** — *프레임워크 만들기* 가 아닌 DX 개선
4. **`backend/framework/` 폴더** — frontend `framework/` 와 명명 일치 ([frontend/src/framework/index.ts](../frontend/src/framework/index.ts) 검증된 패턴)
5. **두 audience 분리**:
   - *운영 contract* = `PUBLIC_TOPICS / PUBLIC_SERVICES` 필터, frontend `contract.ts` 자동 emit (기존)
   - *dev system-docs* = 전체 노출, frontend `/system-docs` page (신규)
   - 같은 registry, 다른 exposure
6. **DI container 안 도입** (2026-06-24 결정) — FastAPI `Depends` 의 call-time lookup 본질은 HTTP request lifecycle 에 묶인 패턴. 우리는 process-scoped service라 정당화 약함. testability 는 monkey-patch 패턴 (test_gamepad) 이미 정착. lazy singleton + 명시적 `__init__` 호출로 충분 — cargo cult 회피
7. **production code 자리 dogfood 박지 말 것** (2026-06-24 결정) — test 만을 위한 메소드 / attribute 를 production class 에 박는 자체 noise. cross-process verification 안 되는 자리는 production 박을 정당화 없음. test 안 self-contained dummy class + raw string topic + dummy Pydantic 로 framework 자체만 검증
8. **점진 적용 = 검토 위함** (2026-06-24 명확화) — 호환성 보장 X (개발 단계). 작은 commit + 검토 + 한 자리씩 변환. 변경 때문에 다른 노드 깨지면 `host_mock.yaml::application_nodes` 에서 잠시 빼놓고 진행, 끝나면 다 변환 + 다시 활성

## 4. Framework API (현재까지 확정 — 2026-06-24)

### 4.1 `@service` — RPC handler

```python
from framework import service
from core.transport.messages.base import ServiceRequest, ServiceResponse

class StorageNode(ApplicationNode):
    @service(Service.STORAGE_LIST_CALIBRATIONS)
    def list_calibrations(
        self, req: ServiceRequest[ListCalibrationsReq]
    ) -> ServiceResponse[ListCalibrationsRes]:
        ...
```

- `key` = enum referent (`Service.X` 값)
- `req_cls` / `res_cls` = type hint 에서 자동 추출 (FastAPI 패턴)
- Pydantic v2 generic (`ServiceRequest[X]`) 은 `typing.get_args()` 가 빈 tuple 반환 → `__pydantic_generic_metadata__["args"]` fallback 박혀있음 ([framework/service.py](../backend/framework/service.py))

### 4.2 `@subscriber` — Topic subscriber

```python
from framework import subscriber

class FooNode(BaseNode):
    @subscriber(Topic.STORAGE_CALIBRATION_INVALIDATED)
    def on_invalidation(self, msg: CalibrationInvalidated) -> None:
        ...
```

- `key` = enum referent
- `msg_cls` = type hint 직접 (envelope X, service 와 다름)
- `from __future__ import annotations` 환경에서 `get_type_hints(func)` 가 `func.__globals__` 만 보고 local scope 못 봄 → type hint 의 Pydantic class 는 module-level import 필요

### 4.3 `@publishes` — Topic publisher (Phase B — 미구현)

```python
class MotorNode(BaseNode):
    @publishes(Topic.MOTOR_STATE_JOINT)
    def _publish_state(self, state: MotorJointState) -> None:
        self.publish(Topic.MOTOR_STATE_JOINT, state)
```

**mechanism — mark only** (FastStream wrap 패턴 채택 X). 이유:
- 우리 publish 패턴은 worker loop / event callback 안 `self.publish(...)` 호출 — 함수 return 자동 publish 패턴 안 맞음
- mark only 면 함수 본문 자유 + registry 가 *이 함수가 Topic.X 발행한다* 만 인식
- boilerplate 증가 = 데코 한 줄

class-level (`__publishes__ = (Topic.X,)`) 옵션은 *어디서* 정보 손실 — 함수-level mark 채택.

## 5. attach 가능한 객체 — class hierarchy 강제 X (2026-06-24 재결정)

**§5 이전 버전의 BaseComponent 다이어그램 폐기.** 박을 때 잘못된 진단 박힘 — "JointStateCache 가 `__init__` 안 `ZenohSession.declare_subscriber` 직접 호출하면 invisible" — 현재 cache 코드는 그렇게 안 돼있음. cache 는 `node.create_subscriber` 위임, lifecycle 은 노드가 가짐. hypothetical scenario 자체로 lifecycle 계층 (BaseComponent) 끌어온 cargo cult.

**진짜 문제** = `JointStateCache` 가 어떤 topic 듣는지 framework registry 에서 안 보임. 해결 = 메소드에 `@subscriber` 데코 박는 것만. base class 상속 강제 X.

```
framework helper (bind_decorated_subscribers / collect_*_specs_from_instance)
        ↑                ↑                ↑
        |                |                |
    BaseNode         Handler           Cache
    (start() 안 호출)  (node.attach_     (__init__ 안 호출)
                      handler(self))
```

**원칙** — 데코 박은 메소드 = 계약 + 실행 엔트리포인트. 별도 mark (`__subscribes__ = (...)`) X — dual source of truth 위반.

**framework 가 보고 dispatch 하는 객체 카테고리**:

| 카테고리 | 예시 | attach 시점 |
|---|---|---|
| Node (BaseNode 상속) | CalibrationNode / MotionNode | `start()` 안 self bind |
| Handler (composition member) | CalibrationHandlers / ScanWorkflowHandlers | 노드가 `attach_handler(self)` |
| Cache (process singleton) | JointStateCache / FrameCache | `__init__` 안 self bind |

class hierarchy 강제 X — 셋 다 동일 framework helper (`bind_decorated_subscribers(obj)`) 호출만 다름.

**Bridge 는 scan 제외** — infrastructure layer (FastAPI middleware 등가). application contract 가 아닌 plumbing. system-docs viewer 에 안 박힘.

**docs 두 레벨 분리 (2026-06-24, distributed 관점)**:

- **레벨 1 — 객체 contract**: 단일 객체가 *스스로* 박는 정보. subscribes / publishes / services. `@subscriber` / `@publishes` / `@service` 데코로 객체 안에 박힘. PC 어디 떠 있든 무관 = local declaration.
- **레벨 2 — 시스템 topology**: 여러 객체 contract 합쳐서 보임. "DetectorNode publish DETECTION_RESULT" → "Scene3DNode subscribes DETECTION_RESULT" 자리 연결. registry 전체 합치면 자동 생성.

caller 관계 (누가 service 호출하나) 는 docs 목표에서 제외 — *분산 observability* 문제 자체 별개 layer (process-local 정보 X, `@calls` 박을 수 없음). `@publishes` 의 process_name / module 같은 metadata 자체 후속 검토 (Phase B 진입 시).

## 6. 메타 질문 (2026-06-24 학습)

"새 노드 / framework 변환 시 항상 던질 질문":

1. **이 노드는 일반 노드인가, composite host 인가?** — composite host (예: StorageNode + CalibrationHandlers / ScanWorkflowHandlers) 면 sub-handler 패턴 + `attach_handler` 사용. 일반 노드면 `__init_subclass__` scan 만으로 충분.
2. **이 wire 는 cross-process verification 가능한가?** — service 면 mock backend spawn + test peer call → 응답 받음으로 verify. subscriber callback / production attribute 는 backend process 안 → test peer 가 read 못 함. 안 되면 production 에 박지 말고 test 안 self-contained dummy class 로 framework 만 검증.
3. **dogfood 가 test 만을 위한 production code 박는 경우인가?** — production class 에 dogfood 메소드 / attribute 박는 것 자체 noise. 메소드가 production 으로 실제로 사용되거나 (V2 service 같이 cross-process verification 경우) 아니면 박지 말 것.
4. **FastAPI / Spring / FastStream / Faust / ROS 2 의 어떤 패턴을 차용하나?** — 우리 use case 정당화되는 부분만 차용. 겉모양 / 명명만 흉내 X (cargo cult 회피 메모리).
5. **이 데코는 계약(선언)인가, 실행 흐름인가?** — `@service` / `@subscriber` 는 framework 가 *호출* 하는 자리 = 선언적 계약. `@publishes` 는 객체가 *호출* 하지만 클래스 scope 라 AST 로 데코 vs 실제 `self.publish` 일치 검증 가능 → 계약 OK. `@calls` 는 함수 scope + flow + wrapper + 조건부 → 검증 어려움, stale 위험 → 데코 X, runtime call graph 로 풀 것.
6. **이 hypothetical 진단 박을 때 실제 코드 봤나?** — §5 BaseComponent 다이어그램 박을 때 "cache 가 `__init__` 안 declare_subscriber 직접 호출" 이라고 잘못 진단. 실제 코드는 `node.create_subscriber` 위임. 가상 시나리오로 framework 계층 끌어오는 것 자체 cargo cult. 진단 박기 전 코드 grep 필수.
7. **이 정보는 process-local 인가, 시스템 layer 인가?** — distributed 환경에서 PC A 의 객체가 PC B 의 객체에 대해 *스스로* 박을 수 없는 정보는 객체 contract 가 아님 = 분산 observability layer. `@subscriber` / `@publishes` / `@service` = local declaration (객체 스스로 박힘) = contract OK. `@calls` / caller graph = 시스템 topology (누가 나를 호출하는지는 process 너머 정보) = docs 목표에서 제외.

## 7. Phase 순서

| Phase | 작업 | 산출물 | 상태 |
|---|---|---|---|
| 0 | Storage CRUD vs Workflow 경계 *합의* (코드 짚어서, 실제 이동 X) | 본 문서 §10 표 정밀화 | 미진행 — Phase 5 진입 전 자리 |
| 1 | 1 wire dogfood — framework MVP + `STORAGE_LIST_CALIBRATIONS_V2` 변환 + cross-process test | 동작 + 6 dogfood test PASS | ✅ **완료** (2026-06-24) |
| 2 | dogfood 평가 + 메타데이터 SSOT shape 확정 — `@service` / `@subscriber` 데코 + composite host (`attach_handler`) + production 미침범 | §3 / §5 / §6 결정 박힘 | ✅ **완료** (2026-06-24) |
| **A** | **framework 확장 (signature codec / robot_id inject / wildcard expand / dedup) + cache 한 곳 변환 (JointStateCache)** | cache 가 `@subscriber` 박힌 일반 객체로 동작, registry visible | ✅ **완료** (2026-06-24) — framework/topic.py 확장 + framework/binding.py 신규 + base_node refactor + JointStateCache 변환 + 호출자 6곳 정리. dogfood 6 PASS / calibration_e2e 2 PASS / 전체 pytest PASS |
| ~~B~~ | ~~`@publishes` 데코 + AST lint + 노드 publish 한 곳씩 변환~~ | ~~publish 도 contract SSOT~~ | **보류 — backend_v2 reframe (§14)** |
| ~~C~~ | ~~system-docs viewer~~ | ~~"누가 발행 / 누가 듣는지" 시각화~~ | **보류 — backend_v2** |
| ~~3~~ | ~~두 번째 wire (`MOTION_MOVE_L`) dogfood~~ | ~~robot-scoped placeholder + multi-dispatch 검증~~ | **보류 — backend_v2** |
| ~~4~~ | ~~나머지 wire 일괄 마이그레이션~~ | ~~모든 service/topic `@service` / `@subscriber`~~ | **보류 — backend_v2 완성 후 backend/ discard** |
| ~~5~~ | ~~Storage workflow service 들 도메인 노드로 재배치~~ | ~~Storage = 순수 CRUD~~ | **보류 — backend_v2 의 Component 분리 자체가 흡수** |

**Phase A 세부 변경 (framework)**:
1. robot-scoped key (`{robot_id}` placeholder) 감지 → wildcard subscribe (`horibot/*/...`) + sample.key_expr 에서 robot_id 추출
2. callback signature codec 판단 — `msg: Pydantic` → validate, `payload: bytes` → skip
3. callback signature inject — `(self, robot_id, msg)` / `(self, robot_id, payload: bytes)` / `(self, msg)` / `(self, payload)` (robot-scoped 여부 + codec 조합)
4. instance 단위 bound dedup (cache singleton 자리 N 개 노드가 attach 호출해도 한 번만 bind)
5. `bind_decorated_subscribers(obj)` 일반 helper — Node / Handler / Cache 모두 동일 호출 (BaseNode.start / node.attach_handler / cache.__init__ 안에서 각각)

**Phase A 진입 후 JointStateCache 변환 결과**:

```python
class JointStateCache:
    def __init__(self):
        if self._initialized: return
        self._initialized = True
        ...
        bind_decorated_subscribers(self)

    @subscriber(Topic.MOTOR_STATE_JOINT)
    def _on_motor_state(self, robot_id: str, msg: MotorJointState):
        ...
```

호출자 노드의 `cache.subscribe(self, rid)` 패턴 폐기 — cache 가 자기 subscribe 책임.

## 7.5 Reframe — backend_v2 실험실 (2026-06-24, §14 참조)

Phase A 까지 박은 후 사용자가 더 근본 질문 던짐:

1. **노드 자체 진짜 필요한가?** — framework 박힌 후 cache/handler 가 self-contained 객체로 동작. 노드는 *grouping convention* 일 뿐 — process / robot / component 가 진짜 unit. ROS mental model 의 유산.
2. **여러 노드가 여러 번 구독 = 정상** — Zenoh pub/sub 본질. cache 의 motivation 은 *상태 공유* (ROS-think) 아니라 *boilerplate + 변환 wrapper* (JointState) 또는 *expensive transformation memoization* (Frame decode).
3. **운영 X = 리라이트 cost 작음** — "이미 개발했음" 은 cost-based 근거 (메모리 위반). 진짜 합리적이면 처음부터 다시.
4. **Contract First + Binder = 확정. Node 완전 삭제 + Handler/Cache/Worker/Adapter 4분류 = 가설** — 코드로 검증 필요.

결정 = backend_v2/ 실험실 박음. §14 자체 plan.

Phase B / C / 3 / 4 / 5 자체 자체 보류 — backend_v2 결과에 흡수. 단 Phase A 산출물 (framework 확장 + JointStateCache 변환) 자체 자체 backend/ 안에 박혀 있음 — 회귀 0, 운영 (개발 단계) 안 깨짐.

## 8. 완료된 dogfood (Phase 1)

**wire = `STORAGE_LIST_CALIBRATIONS_V2`** (read-only, host-level, 단순)
- [topic_map.py:127](../backend/core/transport/topic_map.py#L127) — V2 enum 추가 (dogfood-only 임시 wire)
- [handlers/calibration.py:144](../backend/nodes/application/storage/handlers/calibration.py#L144) — `@service(STORAGE_LIST_CALIBRATIONS_V2)` 메소드. `_srv_list` 위임 (구현 재사용)
- [handlers/calibration.py:139](../backend/nodes/application/storage/handlers/calibration.py#L139) — `register()` 끝에 `node.attach_handler(self)` 한 줄로 sub-handler 의 `@service` 메소드 자동 발견 + bind

**dogfood test 6 PASS** ([tests/test_framework_service_dogfood.py](../backend/tests/test_framework_service_dogfood.py))
- `test_v2_same_response_as_v1` (same-process round-trip)
- `test_v2_round_trip_via_mock_backend` (L3 — host_mock subprocess + 분리 zenoh peer cross-process)
- `test_v2_spec_on_sub_handler` (composite host spec discovery)
- `test_subscriber_spec_on_general_node` (BaseNode `__init_subclass__` scan)
- `test_subscriber_spec_on_sub_handler` (`collect_subscriber_specs_from_instance`)
- `test_subscriber_callback_round_trip` (in-process publish → callback 발동)

subscriber test 자리 production 미참조 — `_DOGFOOD_TOPIC = "test/framework/dogfood"` raw string + `_DogfoodMsg(BaseModel)` test 안 dummy.

**갈아치움 step (Phase 4 자리)** — `_srv_list` 의 본문을 `list_calibrations_v2` 로 흡수, `register()` 안 `Service.STORAGE_LIST_CALIBRATIONS` 등록 줄 제거, V2 enum 제거 → frontend / 다른 caller 안 건드림 (key 자체 유지).

## 9. 미정 항목 (다음 세션 진입점)

| # | 미정 | 결정 시점 |
|---|---|---|
| 1 | ~~`@service` 가 owner 노드를 어떻게 식별~~ | ✅ Phase 2 — `__init_subclass__` 자동 scan + composite host 자리 `attach_handler` |
| 2 | robot_id scope 처리 — `BaseNode.r()` 위에 얹는지 / 데코레이터가 직접 관리 | Phase 3 (MOTION_MOVE_L) |
| 3 | ApplicationNode `dict[robot_id]` multi-dispatch 와 데코레이터 결합 | Phase 3 |
| 4 | Storage CRUD vs Workflow 경계 case-by-case 표 정밀화 | Phase 0 (Phase 5 진입 전) |
| ~~5~~ | ~~Phase A 진입 — A1 / A2 갈래~~ | ✅ Phase 2 재검토 — BaseComponent 폐기, framework 확장 + cache 변환만 (§5 / §7 정리) |
| 6 | callback signature inject 디테일 — robot-scoped 아닌 경우 robot_id 인자 자체 없애야 (signature 검사) / Pydantic vs bytes codec 판단 fail-fast | Phase A 구현 |
| 7 | AST lint 구현 — `@publishes` 데코 자리 vs 실제 `self.publish(Topic.X)` 일치 검증 | Phase B |

## 10. Storage 경계 case-by-case (Phase 0 입력)

현재 코드 의심 후보 (Phase 0 에서 추적해 확정):

| service | CRUD ✅ / Workflow ⚠️ | 비고 |
|---|---|---|
| `STORAGE_LIST_CALIBRATIONS` | ✅ | 단순 read |
| `STORAGE_COMMIT_CALIBRATION` | ⚠️ | run finalize + result INSERT + activate 묶음 |
| `STORAGE_ACTIVATE_CALIBRATION` | ⚠️ | 같은 (robot, kind) atomic toggle. 트랜잭션 + 도메인 로직 섞임 |
| `STORAGE_NEW_SCAN_SESSION` | ✅ | session row INSERT |
| `STORAGE_DELETE_SCAN_SESSION` | ✅ | CASCADE — 트랜잭션 무결성 |
| `STORAGE_PUT_SCAN` | ✅ | scan_id allocate + blob put + row INSERT |
| `STORAGE_PUT_RECONSTRUCTION` | ✅ | append-only blob + row |

→ 경계 원칙 *잠정*:
- **트랜잭션 (atomic 보장 필요)** → storage 안 OK
- **도메인 결정 (어느 result 가 active / run status transition)** → 도메인 노드로 이동

Phase 0 = 위 표 정밀화 + 원칙 fix.

## 11. 검토 protocol 와의 관계

본 작업은 [architecture_review_protocol.md](architecture_review_protocol.md) §산출물 의 첫 큰 docs 산출. protocol 제약 그대로 적용:
- 단편 reflex X — FastAPI 그대로 복사 X, 우리 use case 에 맞게 수정
- "cheap fix" / "한 줄이면 끝" 어휘 X
- "개인 학습 프로젝트라서" / "N=2 라서" scope 핑계 X
- 사용자 push 에 입장 뒤집기 X
- md/docs 인용 X — 실제 코드 우선
- 의도 떠넘기지 X
- "자리" placeholder 의미 없이 박지 X (메모리 자리)
- 짜기 전 hand-simulate + edge case 사전 질문 (memory)

## 12. 다음 세션 시작점

새 세션 진입 시:

1. 본 문서 + [architecture_review_protocol.md](architecture_review_protocol.md) 동시 anchor.
2. **§15 Runtime-centric reframe (2026-06-25)** 가 현재 결정 — Node 가 잘못된 전제, Runtime (Process) 이 최소 단위, Module = 기능 묶음.
3. **backend_v2/ 폴더 폐기 (2026-06-25)** — Phase 1 MVP 산출물 (framework + 7 test PASS) 이 Node 잘못된 전제 위 코드. §15 reframe 위에 다시 짬.
4. §14 = history (잘못된 전제 위 plan).
5. Phase 1 / 2 / A (§14 의 backend/ 변환) — docs 에 완료 기록 있으나 사용자가 이후 discard. main branch 코드에 framework/ 없음 (참고만).
6. 다음 step = Transport abstraction (§15.7).
7. 새 코드 작성 전 §6 메타 질문 7 가지 던질 것.
8. test 짤 때 production code 에 dogfood 넣지 말 것 — self-contained dummy class 로 framework 만 검증.

사용자가 "Runtime" / "Module" / "Transport adapter" / "Contract layer" / "distribution is runtime concern" 톤 던지면 §15 진입. "backend_v2" / "Component 4분류" / "Node 삭제" 톤은 §15 reframe 안내 (§14 history). "BaseComponent" 톤은 §13.8 정정 안내.

## 13. 결정 history (학습 anchor)

본 plan 진행 중 잘못 짚었다가 사용자가 정정한 자리 — 다음 세션 같은 실수 회피:

1. **handler 분리 패턴 자체 못 짚음** (대화 초기) — `StorageNode` 의 `CalibrationHandlers` / `ScanWorkflowHandlers` composition 자리 보자마자 "composite host?" 메타 질문 던졌어야. V2 메소드를 StorageNode 자체에 박은 자체 잘못. 정정 후 sub-handler `attach_handler` 패턴 박힘.
2. **FastAPI DI cargo cult** — Depends + call-time lookup 패턴 우리에게 적합한지 따져봤다가, HTTP request lifecycle 자체에 묶인 패턴 자리 우리 process-scoped service 자리 정당화 약함 발견. monkey-patch + lazy singleton 패턴 유지.
3. **production code 자리 dogfood 박음** — CalibrationNode + CalibrationHandlers 에 dogfood `@subscriber` 메소드 + attribute 박은 자체 잘못. test peer 가 backend process 안 attribute 못 read → test 가 결국 in-process dummy class 자리. production class 박힌 자리 unused 잔재. 정정 후 production 미참조 + test self-contained.
4. **`STORAGE_CALIBRATION_INVALIDATED_V2` enum 추가도 잘못** — dogfood-only wire 인데 production code 자리 V2 enum publish 자체 없음. enum 자리 잔재. 정정 후 raw string topic 으로 갈아치움.
5. **DI / Container 자리 너무 빨리 확장** — FastStream 패턴 차용 자리 사용자가 다시 짚은 자리 = "JointStateCache 같은 cache 가 framework registry visible 되어야 한다" 만. DI container 자리 사용자 의도 X. BaseComponent layer + `@subscriber` 자체 확장 자리만 정당화.
6. **publish 도 framework registry 자리 visible 필요** — 처음 plan 자리 `@service` / `@subscriber` 만. publish 자리 누락. 사용자가 짚은 자리 = "어떤 노드가 publish 한다" docs 자리 안 보임 → `@publishes` 데코 추가 결정.
7. **점진 적용 = 호환성 X, 검토만** (사용자 명확화) — 개발 단계라 한 곳 깔끔 변환. 다른 노드 깨지면 host_mock 에서 잠시 끄고 진행. *기존 패턴 호환 layer* 안 둠.
8. **BaseComponent 추출 — hypothetical scenario 로 박은 cargo cult** (2026-06-24 두 번째 정정) — §5 다이어그램 박을 때 "cache 가 `__init__` 안 `ZenohSession.declare_subscriber` 직접 호출하면 invisible" 진단 박음. 실제 코드는 `node.create_subscriber` 위임 — cache 가 ZenohSession 직접 안 만짐. 가상 시나리오로 lifecycle 계층 끌어옴. 사용자 reframe — "원래 문제는 docs visibility 만, lifecycle 은 따로 문제" — 정답. 진단 박기 전 코드 grep 필수 메모리 박힘 (§6 메타 6).
9. **`__subscribes__ = (...)` dual mark 추천 — dual source of truth 위반** (2026-06-24) — cache 변환 옵션으로 "class-level mark 한 줄" 박았는데 사용자 정정: 실행 로직 (`def subscribe(...)`) 과 계약 (`__subscribes__`) 분리되면 어긋날 수 있음. 정석 framework = 실행 엔트리포인트 = 계약 (FastAPI `@app.get` 등). 따라서 `@subscriber` 데코 박은 메소드 = 계약 + 실행 한 곳.
10. **`@calls` 와 `@publishes` 같은 카테고리로 묶은 잘못** (2026-06-24) — `@calls` 폐기 한 후 같은 논리로 `@publishes` 도 폐기 박았는데 사용자 반박: "그러면 system-docs viewer 가 누가 publish 하는지 어떻게 박나?" runtime instrumentation 은 idle 노드 / 조건부 publish 못 박음. 차이 = 검증 가능성 — `@publishes` 는 클래스 scope 라 AST 로 데코 vs 실제 `self.publish(Topic.X)` 일치 검증 가능, `@calls` 는 함수 scope + flow + wrapper + 조건부 → 검증 어려움. 따라서 `@publishes` 유지 + AST lint.

    **§10.5 distributed 관점 강화 (2026-06-24 후속)** — `@calls` 대안으로 "runtime caller graph 박자" 박았는데 사용자 반박: distributed 환경에서 `TaskNode (PC A) → MotionNode (PC B)` 호출 자리 MotionNode 프로세스는 호출자가 GroundedDetect 인지 PickTask 인지 모름. caller graph 자체 process-local 정보 X = *시스템 topology* 정보 = 분산 observability layer. 객체 contract 와 다른 layer. 따라서 runtime call graph 자체도 폐기 — *caller 관계는 docs 목표에서 제외*. `@service` / `@subscriber` / `@publishes` 는 객체가 스스로 박는 local declaration (PC 어디 떠 있든 무관) = 객체 contract layer. 두 layer 섞지 말 것.
11. **cost-based 추천 (메모리 `feedback_no_cheap_argument` 위반)** (2026-06-24) — cache 변환 옵션 추천 시 "코드 변경 최소" / "framework 변경 zero" 근거로 박음. 사용자 정정: "적용 비용 기준 빼고 설계적으로만 보면 답 바뀜" — 메모리 박혀 있는데도 reflex 적으로 cost 박음. 정석 / 원칙 / 일관성으로 평가할 것.
12. **자체 분석 안 박고 카탈로그 / 옵션 / "어때?" 패턴 반복** (2026-06-24) — 사용자 명시 지적: "너는 너가 생각 안 해? 왜 이렇게 분석 안 해?" 검토 phase 진행 중 거의 매 turn 옵션 나열 + 의견 물음 패턴. push 두려움 + 자체 틀림 회피 = 안전 자세. 검토 phase 의미 약화 — 자체 입장 박은 후 사용자가 평가해야 검토 의미 있음. 단 입장 박을 때 *근거* 박혀 있어야 (cost 같은 메모리 위반 근거 X).
13. **Cache motivation 잘못 진단 (ROS-think reflex)** (2026-06-24) — JointStateCache 박을 때 "중앙 상태 저장소 자체 자체 모든 노드 공유" motivation 박힘. 사용자 reframe: Zenoh pub/sub 에서 *여러 노드가 여러 번 구독 정상*. cache 진짜 motivation = (a) boilerplate 줄임 + (b) 변환 wrapper. *상태 공유* 아님 (cache 도 process-local, distributed 면 PC 별 별개). singleton 자체 = cost saving 만, correctness 아님. FrameCache 는 다름 — *JPEG decode dedup* 진짜 가치 (단 현재 raw bytes 보유라 미실현).
14. **노드 unique 책임 없음 — grouping convention** (2026-06-24) — "노드 왜 필요?" 질문에 처음 답 = deployment / identity / lifecycle / heartbeat / thread 호스트 — 현재 코드 정당화 답. 사용자 reframe: zero-base 면 process / robot / component (Handler/Cache/Worker/Adapter) 가 진짜 unit, 노드는 ROS mental model 유산 = grouping convention. framework 강제 X.
15. **cost-based reflex 재발 — "이미 개발했음"** (2026-06-24) — Component 분리 박은 후 두 번째 답에서 "분산 + 이미 개발된 코드 + 노드 mental model 자연" 박음. 사용자 정정: "노드 패턴 많이 개발함 이런건 빼고 — 진짜 합리적이면 처음부터 다시 짤 거". cost-based 근거 (메모리 위반) 또 박힘. 정석 / 원칙으로 평가.
16. **운영 X = 리라이트 cost 작음** (2026-06-24) — 사용자 명확화: 운영 단계 아님. 사용자 없음. 배포 안 함. 즉 "절대 갈아엎지 마라" 계열 조언 해당 X. framework cost << future maintenance cost 시점. 단 *바로 전체 삭제 X*, backend_v2 실험실 박은 후 판정.

17. **Node = 잘못된 전제** (2026-06-25) — §14 까지의 plan 이 "Node 라는 실행 단위가 있다" 전제 위에 서 있음. 진짜 깨달음 = Runtime (Process) 이 최소 단위, Node 가 Runtime 책임을 자기 이름에 가졌던 abstraction. Module + Runtime 분리가 진짜 reframe. §15 참조.

18. **"Local 호출처럼 보이게" 표현 잘못** (2026-06-25) — GPT 와 토론 중 framework 책임 표현이 "service 호출은 로컬 호출처럼 보이게" 였는데 사용자 정정: 핵심은 *통신 계약 (service / topic) 은 동일, transport (local memory / Zenoh) 만 바뀐다*. local memory path 도 transport 의 한 종류.

19. **`.` vs `/` — key path 형태** (2026-06-25) — 표현이 `storage.commit` 같은 객체 메서드 호출이었는데 사용자 정정: 통신 계약 이름은 path (`/storage/commit`). 이미 v2 framework MVP 가 `/` 사용 — 변경 없음.

20. **backend_v2/ 폴더 폐기** (2026-06-25) — 사용자가 폴더 삭제. 이유 = §14 의 Phase 1 산출물 (framework MVP + 7 test PASS) 이 Node 잘못된 전제 위에 서 있는 코드. §15 reframe 위에 다시 짬.

**메타 학습** — *코드 보자마자 메타 질문 던질 것* + *짜기 전 hand-simulate + verification path 사전 검증* + *FastAPI / 다른 framework 차용 시 우리 use case 정당화 박을 것* (cargo cult 회피) + *진단 박기 전 실제 코드 grep* + *데코 박을 자리 — 계약(framework 호출) vs 실행 흐름(객체 호출) 판단 + 검증 가능성 판단* + *카탈로그 / 옵션만 던지지 말고 자체 입장 + 근거 박을 것* + *cost-based reflex 재발 주의 (메모리 박혀 있어도 두 번 박음)* + *Zenoh pub/sub 본질 = 여러 구독 OK, ROS-think (중앙 상태 model) reflex 차단*.

## 14. backend_v2 — zero-base 실험실 (2026-06-24)

### 14.1 동기

§7.5 reframe — Phase A 까지 박은 후 발견:
1. Contract First + Binder 모델 = 거의 확정 (`@service` / `@subscriber` / `@publishes` → ContractSpec → Binder)
2. BaseNode = 계약 / 실행 / 배포 / lifecycle 결합. 분리 필요.
3. 일반 객체 (BaseNode 비상속) 도 framework 가 attach 가능해야 — 이미 backend/ 에 capability 박힘.
4. Transport (Zenoh) / Framework (Horibot) 분리 필요.

→ 갈아엎을 만한 확신 박힘. 단 *Node 완전 삭제 + 4분류* 는 가설 — 코드로 검증.

### 14.2 plan

운영 단계 아님 (사용자 / 배포 / production 자체 없음). 리라이트 cost 작음.

```
1. backend/ 의 framework 변경 (Phase A 산출물) 그대로 유지 — 회귀 0
2. backend_v2/ 새 폴더 zero-base 실험실 박음
3. backend_v2 framework 자체 박음 (Contract First + Binder + Transport/Framework 분리)
4. 첫 component 1개 박아서 검증
5. 며칠 사용 후 4 질문 판정:
   - 개발 더 빨라졌나?
   - 테스트 쉬워졌나?
   - 문서 생성 쉬워졌나?
   - 새 컴포넌트 머리 덜 아픈가?
6. YES 3-4개 면 → backend/ 의 도메인 logic (캘 BA / motion / task DSL / scan / detector / scene3d / reconstruction / storage 등) 다 backend_v2 의 component 로 옮겨심음
7. backend_v2 가 backend/ 의 모든 기능 가지면 → backend/ discard
```

### 14.3 규칙 (실패한 리라이트 방지)

많은 리라이트 실패 패턴:
```
v1 멈춤 → v2 시작 → 기능 부족 → 계속 추가 → 6개월 후 둘 다 망함
```

차단:
- **규칙 1**: backend/ 자체 계속 개발 가능 (캘 / motion / scan 등). 단 framework 부분 (BaseNode / 노드 hierarchy) 자체 자체 자체 *추가 변경 X* — Phase A 까지가 끝.
- **규칙 2**: backend_v2 자체 자체 *기능 개발 금지*. 오직 framework 검증. 첫 component 자체 자체 framework 가 진짜 동작하는지 검증용.
- **규칙 3**: 실제 hardware 자체 자체 1 robot (omx_f_0) 만 붙여보기. 설계는 종이에선 다 좋아 보임 — 실제 붙여봐야 Worker / Handler 경계 자체 자체 검증.
- **규칙 4**: backend/ 자체 자체 자체 자체 *코드 reference* 박을 수 있음 — 캘 BA / Ruckig / IRLS / ChArUco / step DSL 등은 자산. *재구성* 자체 자체 자체 — 그저 framework 모양 자체 다름.

### 14.4 폴더 구조 (잠정)

```
backend_v2/
  transport/
    session.py          — ZenohSession (process singleton)
  contract/
    subscriber.py       — @subscriber + SubscriberSpec
    service.py          — @service + ServiceSpec
    publishes.py        — @publishes + PublishesSpec
  binding/
    bind.py             — bind_decorated(obj, session, robot_id, ...)
  components/
    joint_state_read.py — process-singleton Cache 자체 (read model)
    dynamixel_adapter.py — hardware Adapter 자체
    motion_worker.py     — running thread Worker 자체
    calibration_handler.py — stateless Handler 자체
  main.py               — orchestrator (host config → component list → instantiate + start)
```

### 14.5 Component 분류 (가설 — 코드로 검증)

| 종류 | 책임 | lifecycle | state | robot scope |
|---|---|---|---|---|
| **Handler** | service handler 묶음 | 없음 (attach 시점만) | stateless | constructor 인자 |
| **Cache (read model)** | state holder | self-bind in `__init__` | process singleton | 모든 robot (wildcard) |
| **Worker** | thread / state machine | start/stop | 자체 보유 | constructor 인자 |
| **Adapter** | hardware driver wrapper | start/stop | hardware resource | constructor 인자 |

검증 포인트 — *실제 component 박을 때 경계 명확한가?* 예: `CalibrationWorker` 가 service 도 받고 state 도 들고 background 작업도 함 — Handler 인가 Worker 인가 애매. 이게 가설 시험.

### 14.6 Architecture detail

**4 Layer 분리**:
```
Application (Task DSL / Recipe / Step)
       ↓
Components (Handler / Cache / Worker / Adapter)
       ↓
Framework (Contract + Binding)
       ↓
Transport (ZenohSession + Pydantic schema + Key registry)
```

**Layer 별 책임**:

| Layer | 책임 | 폴더 |
|---|---|---|
| Transport | wire + serialize | `transport/` |
| Framework | 데코 + binding helper | `contract/` + `binding/` |
| Component | 객체 책임 (4 종) | `components/` |
| Application | 도메인 logic | `tasks/` + `recipes/` |

**Framework contract (1급 계약 3개)**:
- `@service(key)` — RPC handler
- `@subscriber(key)` — topic sub (signature 기반 codec + robot_id inject)
- `@publishes(key)` — mark only (docs + AST lint, 실제 publish 는 helper 호출)
- `bind_decorated(obj, session, robot_id, ...)` — 일반 helper. Node / Handler / Cache / Worker / Adapter 다 동일 호출.

**Identity model**:
- process: `process_id` (host config)
- robot: `robot_id` (constructor 인자)
- component: `cls.__name__` + optional `robot_id`
- BaseNode 같은 class hierarchy identity 없음 — 그저 plain class.

**Process orchestration**:
- `main.py` 가 host config 읽음
- host config = `process_id` + `transport` + `components` list (cls / robots)
- main 이 import → instantiate → start. lifecycle = component 단위.

**Heartbeat**:
- `ProcessHeartbeat` Worker (process-level 하나)
- payload: `process_id` + active component list (cls / robot_id)
- 노드별 heartbeat 폐기. visibility 는 component list 로 충분.

**Publish API**:
- `from framework import publish` — `publish(key, msg)` stateless helper
- BaseModel / bytes / dict 자동 codec 판단
- `@publishes` 데코 = 마킹만 (docs / lint)
- AST lint 가 데코 vs 실제 `publish(Topic.X, ...)` 호출 일치 검증 → stale 차단

**Data flow (subscribe)**:
```
Publisher → Zenoh router → bind_decorated callback
  → key_expr 에서 robot_id 추출 (template parse)
  → payload codec (Pydantic.model_validate_json / bytes)
  → component method 호출 (robot_id inject 여부는 signature 기반)
```

**Data flow (service)**:
```
call_service → Zenoh queryable → bind_decorated handler
  → req: ServiceRequest[X] model_validate_json
  → component method
  → res: ServiceResponse[Y] model_dump_json reply
```

**Data flow (publish)**:
```
component method → framework.publish(key, msg)
  → ZenohSession.put(key, encoded)
```

**핵심 결정**:
- **BaseNode 폐기**. 모든 객체 plain class. framework 가 wire.
- **Component 4분류 = 책임 분리 가설** — 가설 검증 위해 4 종 다 박아봄.
- **`@publishes` = mark only** — 실제 publish 는 `framework.publish()` helper.
- **Identity = process_id + robot_id** 두 축. component 식별 = cls name + optional robot.
- **Heartbeat = process 1개** — active components list 박음. node-level 자체 폐기.

### 14.7 폐기될 backend/ 의 framework 자산

backend_v2 자체 자체 완성 박힌 후 폐기:
- `core/transport/base_node.py` 자체 `BaseNode` / `start()` / `attach_handler` / `r()` 자체
- `core/transport/application_node.py` / `device_node.py` — 2-layer 분류 자체 자체 (`isinstance(cls, DeviceNode)` 자체 자체 main.py 검증)
- `core/transport/node_registry.py` — lazy-import factory
- `core/cache/joint_state_cache.py` (Phase A 변환된 모양 — backend_v2 자체 자체 자체 재구성)
- `core/cache/frame_cache.py` (Phase A 미적용 — backend_v2 자체 자체 자체 재구성)
- `framework/` 폴더 자체 자체 — backend_v2/contract / backend_v2/binding 자체 재구성

옮겨심을 도메인 logic (재배치, 폐기 X):
- 캘 BA / IRLS / Huber / observability / strategy / ChArUco / capture_quality
- Motion command / TrajectoryRunner / Ruckig / Jog 적분 SE(3) / IK
- Task DSL / Step / Slot / TaskRunner / Recipe / 정규 task (pick_and_place / scan)
- Detector / YOLO / Grounding DINO / search_and_detect
- Scene3D / depth_frame / consensus / pointcloud streaming
- Reconstruction / ICP / PoseGraph / TSDF / mesh extract
- Storage 자체 (RDB / ObjectStore Protocol / Alembic migration / 캘 5종 / scan workflow)
- Bridge (WebSocket + MJPEG + binary framing)
- Kinematics (Pybullet + SagCorrected + link_offset patch)
- Coordinates (Joint / Link / Sag)
- Gamepad / 8BitDo mapper
- Robot Registry (robots.yaml + RobotConfig + factory)

### 14.8 검증 후 시나리오

**Case A — backend_v2 좋음 (예상)**:
- 도메인 logic 다 backend_v2 component 로 옮겨심음 (1-2달 자체)
- backend/ discard

**Case B — backend_v2 별로 (백업)**:
- backend_v2 폐기
- backend/ 그대로 + framework 부분 강화 (Phase B/C 자체 자체 backend/ 안 자체 자체)

### 14.9 폐기 alert (2026-06-25)

§14 전체 plan 이 **잘못된 전제** 위에 서 있음 — Node 가 최소 단위 가정. 진짜 깨달음 = **Runtime (Process) 이 최소 단위, Module = 기능 묶음** (§15 참조). backend_v2/ 폴더 (Phase 1 MVP 산출물 + 7 test PASS) 를 사용자가 삭제 (2026-06-25). §15 reframe 위에 다시 짬.

§14.5 의 4분류 가설 (Handler / Cache / Worker / Adapter) 은 §15 에서 *Module 의 유형 힌트* 로 위치 변경 — framework 는 종류 모름 (duck typing).

§14.6 의 Architecture detail (BaseNode 폐기 / Component 4분류 / Heartbeat 1개 등) 은 §15 의 Contract / Runtime / Transport 3 layer 안 흡수되거나 재배치.

## 15. Runtime-centric Reframe (2026-06-25)

### 15.1 잘못된 전제 발견

§14 까지의 plan 의 전제 = "Node 라는 실행 단위가 있고, BaseNode 가 그 실행 단위의 공통 기능을 제공해야 한다". 이 전제 위에서 자연스럽게:

- BaseNode 가 bind 관리
- BaseNode 가 decorator 수집
- BaseNode 가 lifecycle 관리
- Node 가 서비스/토픽 제공자
- Node 를 없애면 싱글톤은? 실행 단위는?

모두 *Node 가 근본 개념* 가정 위에 서 있는 질문.

진짜 물어야 할 질문 = **"이 시스템에서 배포/실행의 최소 단위가 무엇인가?"**

답 = **Process / Runtime / Deployment Unit**. Node 가 *Runtime 의 책임을 자기 이름에 가졌던 잘못된 abstraction*. Module 과 Runtime 이 한 클래스에 묶여 있었음.

### 15.2 새 사고

```
Runtime (Process)
 |
 +-- Module (Service Provider)
 |
 +-- Module (Service Provider)
 |
 +-- Module (Subscriber)
```

- **Module** = 기능 묶음. `@service` / `@subscriber` / `@publishes` 가진 함수 보유. plain class.
- **Runtime** = 실행 컨테이너. lifecycle / transport 연결 / registry / DI / shutdown / thread 관리.

"Node 삭제" 의 진짜 의미 = **기능 제공자 (Module) 와 실행 컨테이너 (Runtime) 분리**.

### 15.3 Framework 핵심 책임

**"같은 코드가 어디 배치되든 그대로 동작하게 만들기"**

개발자가 절대 신경 쓰지 않아야 함:

- 같은 process 냐?
- 다른 process 냐?
- 다른 장비냐?
- Zenoh 쓰냐?

모두 framework 내부 결정. 한 줄 요약: **"distribution is not a code concern, it is a runtime concern"**.

### 15.4 Contract / Runtime / Transport 분리

```
Contract (계약 — @service / @subscriber / @publishes)
    ↓
Runtime (Module 등록 + lifecycle + Transport 선택)
    ↓
Transport (계약을 만족시키는 매체)
    ├── Local memory (같은 process)
    └── Zenoh (다른 process / 다른 장비)
```

핵심:

- **Service / Topic = 통신 추상화 계약**, key 는 path 형태 (`/storage/commit`, `/camera/frame`)
- **Zenoh = 그 계약을 만족시키는 외부 transport**
- **Local memory path 도 transport 의 한 종류** — 같은 process 의 provider 는 direct dispatch (serialize 없음)
- 어느 transport 쓸지 = **Runtime 의 결정** (provider 위치 resolver)

### 15.5 4분류 (Handler / Cache / Worker / Adapter) 의 위치

§14.5 의 4분류 가설은 **Module 의 유형 힌트** 로 위치 이동. framework 자체는 Module 종류 모름 (duck typing) — Lifecycle protocol (`start()` / `stop()`) 만 호출. 4분류는 *사용자 mental model* + *system-docs viewer 의 분류* 도구.

### 15.6 v2 framework MVP 의 현재 상태

| Layer | 상태 |
|---|---|
| Contract (`@service` / `@subscriber` / `@publishes`) | ✅ 구현됨 (backend_v2/ 박혔던 것, 폴더 삭제) |
| Module direct wire 등록 (`bind_decorated`) | ✅ 구현됨 |
| **Transport abstraction** (local vs Zenoh) | ❌ Zenoh hardcoded |
| **Runtime resolver** (provider 위치) | ❌ |
| **Lifecycle protocol** (start/stop) | ❌ |
| **DI / config** | ❌ |

진짜 reframe 핵심 — `bind_decorated` 가 지금 Zenoh 직접 호출. 같은 process call 도 Zenoh 통과 = wire serialize/deserialize. 이게 *distribution is runtime concern* 원칙 위반.

backend_v2/ 폴더 (2026-06-25 사용자 삭제) — 위 ✅ 두 layer 가 §15 reframe 위에 다시 짜짐 (모양은 거의 동일, transport 만 추상화).

### 15.7 다음 step

step 후보:

1. **Transport interface** — `class Transport(Protocol)`:
   - `call(key, req) → res`
   - `publish(key, msg)`
   - `subscribe(key, cb)`
   - `register_service(key, handler)`
2. **ZenohTransport** — 기존 v2 binding 의 동작 wrapping
3. **LocalTransport** — process-local registry (key → handler dict), direct dispatch, serialize 없음
4. **bind_decorated** → Transport 호출 (Zenoh 직접 X)
5. **Runtime** — config 받은 후 Module instantiate + transport 선택 (provider 위치 resolver)
6. test — 같은 7 case 가 ZenohTransport + LocalTransport 둘 다 PASS

진짜 첫 step = Transport abstraction. 그 위에 Runtime / Lifecycle / DI 쌓는다.

### 15.8 motion 영역에서 검증된 마찰점

§14 reframe 이후 backend/ motion_node 의 design decision 8 개를 problem statement 관점에서 추출 (2026-06-25). v2 의 Module + Runtime 분리 모델이 그 마찰을 어떻게 푸는지 검증:

1. **같은 도메인 4 entrypoint (Move/Servo/Jog/Task) × N 비즈니스 함수 = N×4 wrapper boilerplate** — backend/ 의 `_make_jog_j_topic_subscriber` / `_make_jog_j_service_handler` 등 8+ wrapper factory.
   → v2 풀이 = 한 method 에 데코 여러 개 (`@service + @subscriber`). signature 는 비즈니스 데이터만 (envelope X, error 는 raise). framework 가 entrypoint shape 변환. wrapper 8+ → 0 줄.

2. **Callback 순서 보장 X** — MOTOR_STATE_JOINT 를 JointStateCache + `_on_motor_state_publish_tcp` 둘 다 subscribe, 순서 보장 안 됨 → motion_node 가 cache 무시하고 직접 raw parse.
   → v2 풀이 = derived read model Module (`TcpStateRead` — JointState 받음 → FK → MotionTcpState publish). motion command handler 와 분리.

3. **한 클래스에 두 책임** — MotionNode = motion command handler + derived state publisher.
   → v2 풀이 = Module 분리.

4. **Service envelope vs topic raw 두 unwrap** — `req: ServiceRequest[JogJReq]` 와 `req: JogJReq` 두 signature.
   → v2 풀이 = framework 가 envelope 흡수. handler signature 는 둘 다 raw 비즈니스.

5. **JointStateCache.subscribe(node, robot_id) 패턴** — singleton 인데 어느 robot 받을지 모름.
   → v2 풀이 = 이미 해결 (wildcard subscribe + robot_id callback inject).

6. **self.r(template) 매 호출 명시** — service 등록 8 줄 + topic 등록 2 줄 + publish 3 줄 전부 명시.
   → v2 풀이 = framework 자동 (이미 부분 해결).

7. **100Hz publish boilerplate** — `self.publish(self.r(Topic.X), MotorCmd(...))`.
   → v2 풀이 = stateless `publish(Topic.X, msg, robot_id=...)` (이미 해결).

8. **Cross-process calibration apply** — start() 안 storage fetch + 자기 process 객체 mutate.
   → v2 풀이 = Module lifecycle hook `start()` 그대로 OR `CalibrationApplier` Module.

가장 큰 검증 — **MotionNode 가 한 일 = framework + Module 사이에 끼어있던 wrapper layer**. framework 가 두꺼워지고 Module 이 직접 wire 등록하면 Node 자체가 할 일 없음. §14 의 "Node 삭제" 결론을 코드로 재확인 → §15 Runtime-centric reframe 도달.

### 15.9 새 anchor

새 세션 진입 시:

1. 본 §15 anchor
2. §14 = history (잘못된 전제 위 plan — 폐기)
3. §13 결정 history 17~20 — Node 잘못된 전제 / "Local 호출처럼" 표현 잘못 / `.` vs `/` / backend_v2/ 폐기
4. backend_v2/ 폴더 없음 — 새로 시작

사용자가 "Runtime" / "Module" / "Transport adapter" / "Contract layer" / "distribution is runtime concern" 톤 던지면 §15 진입. "backend_v2" / "Component 4분류" / "Node 삭제" 톤은 §15 reframe 안내 (§14 history).

default plan = Case A. Case B 는 만약 4 질문 결과 NO 가 많을 때 backup.
