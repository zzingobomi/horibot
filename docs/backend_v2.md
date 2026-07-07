# backend_v2 — 아키텍처 spec (framework + Module catalog + Task 방향)

> 본 문서 = backend_v2 의 **아키텍처 SSOT 단일 문서**. 구성: §1–§14 framework spec
> (8 라운드 토론 2026-06-25 + 이후 정정), **§16 Module catalog** (옛
> backend_v2_modules.md 통합, 2026-07-03), **§17 Task-first 운영 원칙 + Task/PnP 설계**
> (옛 task_dsl_waypoint_port.md 통합). §2.7 에 robot-scoped/agnostic + robot_id 라우팅
> 최종 규칙 (옛 robot_agnostic_module_refactor.md 통합).
>
> **진행 status / 다음 작업 = [backend_v2_status.md](backend_v2_status.md)** (본 문서엔
> 진행 표기 안 둠). 결정 history = [framework_dogfood_plan.md](framework_dogfood_plan.md).

## 1. 개요

framework 의 목표 = **"같은 코드가 어디 배치되든 그대로 동작하게 만들기"**.

분산 시스템의 mechanical plumbing (topic string / serialize / subscriber routing / late-join snapshot / cache wiring / Zenoh queryable·subscriber 등록) 을 framework 가 흡수하고, 개발자는 domain 의 business intent 만 짠다.

단 *React / Redux / MobX 식 reactive state framework* 가 아님. Owner 쪽은 명시적 (`repo.save() + publish(Event)`), Reader 쪽만 framework primitive 로 흡수. 이 비대칭이 **현재 spec 의 가장 중요한 line**.

## 2. 핵심 원칙

### 2.1 Distribution is runtime concern

Module 코드는 자기가 같은 process / 다른 process / 다른 장비 어디서 도는지 모름. 같은 코드를 한 process 에 다 띄우든 Pi/PC/NAS 로 분산하든 동일 동작 — Zenoh same-session in-routing 이 같은 process 자리 처리, 다른 session 사이는 wire 통과. 배치는 deployment yaml 의 결정.

### 2.2 Framework 는 mechanical plumbing 만 흡수

흡수하는 것:
- contract key (service / event / stream) 관리
- payload serialize / deserialize
- subscriber registry + dispatch
- Zenoh queryable / subscriber 등록 + dispatch
- Reader 의 late-join snapshot fill
- Reader 의 event subscription wiring
- Reader 의 local cache management
- service contract 자동 generate (frontend `contract.ts`)

흡수하지 않는 것:
- domain logic (BA / IRLS / Ruckig / IK / TSDF 등)
- `repo.save()` 호출 — domain 이 "저장한다" 라는 의도 표현
- `publish(Event)` 호출 — domain 이 "사건이 발생했다" 라는 의도 표현
- DB schema (각 Module 의 SQLAlchemy class)
- Migration (각 Module 의 Alembic)

### 2.3 Owner / Reader 비대칭

같은 cross-module state read 문제도 두 쪽이 다름.

**Owner** (예: CalibrationModule) = 자기 상태 변경의 *의미* 를 안다. `repo.save() + publish(DomainEvent)` 명시적. framework 가 mutation tracking 으로 자동 event 생성 X — *DB update ≠ domain event*. 같은 row update 가 어떤 때는 ACTIVATE, 어떤 때는 그저 metadata 수정. 의미는 Owner 만 결정.

**Reader** (예: MotionModule) = 다른 Module 의 *현재 상태* 가 필요할 뿐. snapshot 가져오기 / event subscribe / cache update 는 mechanical. framework 가 흡수.

### 2.4 Database-per-Module

각 도메인 Module 이 자기 영속성 owner. 통합 Storage Module 없음 — *centralization 이 풀려 했던 문제 (cross-module 동기화) 의 진짜 답은 Reader primitive*. Storage Module 의 다른 motivation 들 (migration owner / DB dep 격리) 도 자연 해결 (§9 참조).

### 2.5 DIP — Framework Protocol vs Infra impl

framework 는 *기술 모름*. Protocol (`Repository`, `ObjectStore`, `Transport`) 만 정의.
infra/ 가 실 impl (`PostgresRepository`, `MinioObjectStore`, `ZenohTransport`).
Module 은 Protocol 만 의존.

motivation 두 개:
- **test mock** — pytest 시 in-memory transport + sqlite `:memory:` 박아서 framework 자체 검증.
- **import boundary** — Module 에 `import zenoh` / `import sqlalchemy` 안 새는 보장.

"미래 Zenoh → ROS2 갈아끼우기" 같은 자유도 motivation 은 over-engineering reflex. 박지 말 것.

### 2.6 한 사람 capacity 안

Phoenix / Django / Spring Boot 급 풀 framework 짜는 것 한 사람 무리. 단 우리 도메인 (calibration / scan / reconstruction / task) 패턴 좁고 반복적 — **2 패턴 (active-toggle + broadcast / append-only event)** 추출하면 한 사람 capacity 안.

NestJS / Spring 정도 + 분산 transport 흡수 정도. React / Redux / Apollo cache 수준 X.

### 2.7 Module scope — robot-scoped / robot-agnostic + robot_id 라우팅 (최종)

> 2026-07-03 확정 — 구현 드리프트 (calibration/scan/scene3d/waypoint 가 robot-scoped 로
> 잘못 구현) 정정 완료. 본 절 = scope + robot_id 라우팅의 잠긴 규칙. 폐기안 (Bridge
> 자동주입 / 생성 scope 메타데이터) 다시 꺼내지 말 것 — §2.7.3.

Module 두 종류. **기준 = "Module 이 robot 의 runtime state / 물리 자원을 소유하는가"**.

| 종류 | Module (구현 = 설계) | 인스턴스 |
|---|---|---|
| **robot-scoped (4)** | MotorDriver / CameraDriver / CameraDecoded / Motion | per-robot (Module type × robot_id) |
| **robot-agnostic** | Calibration / Detector / Scene3D / Scan / Waypoint / Bridge (+ 미래 Task / Gamepad) | host 당 1 |

- robot-scoped = *물리 자원 owner* (Feetech handle / RealSense handle / robot kinematics state). 자원은 robot 별 분리되어야 자연.
- robot-agnostic = *작업 / orchestration*. robot_id 는 매 service request 의 인자 (req 안 field). DB 의 `robot_id` column 으로 multi-tenant.

기존 backend 의 `DeviceNode` (per-robot) / `ApplicationNode` (host 당 1 + `enabled_robot_ids` dict) 패턴과 본질 동일 — 새 spec 의 차이는 ApplicationNode 의 `dict[robot_id, _state]` boilerplate 가 Repository 의 robot_id parameter 로 흡수.

#### 2.7.1 robot_id 라우팅 — "robot_id 는 두 개다"

같은 이름이지만 위치에 따라 **다른 레이어**의 것:

| | 키 안의 `{robot_id}` | body 안의 `robot_id` |
|---|---|---|
| 정체 | 어느 인스턴스로 라우팅할지 = **주소** | req 모델의 **필드** (`DetectRequest.robot_id`) |
| 책임 | **전송 계층** — Bridge/framework 가 키 확장 | **서비스 API** — 호출자가 req 에 넣음 (타입 강제) |

규칙 (기계적 3갈래 — 전부 구조적으로 갈림, 런타임 추론/메타데이터 0):

1. **robot-scoped 서비스** — 키에 `{robot_id}`. framework 가 `self.robot_id` 로 확장
   (`_register_service` — **scoped 판정의 SSOT**). caller 는 `robot_id=` kwarg.
2. **robot-agnostic + 로봇 대상** — 키에 placeholder 없음, **req 에 `robot_id` 필드**.
   호출자가 넣고 pydantic/TS 가 강제. 단 **다른 식별자(run_id / session_row_id /
   result_id / waypoint_row_id)로 robot 특정 가능하면 DB row 에서 파생** — req 에
   중복 robot_id 채널을 안 만든다 ("run A 에 robot B 캡처" 불일치 원천 차단).
3. **global** — req 에 robot_id 필드 자체가 없음 → 아무 데도 안 들어감 (구조적).

**stream/event 는 서비스와 성격이 다름 — 키에 `{robot_id}` 유지.** framework 가
payload 의 robot_id 로 확장(publish) / wildcard 구독(subscribe) → host-level 모듈도
robot-scoped 스트림을 그대로 발행/구독 (예: 호스트 1개 calibration 의 preview).
따라서 **`robot_scoped` 판정 = service 키만** (publish/subscribe 는 판정에 안 씀 —
snapshot.py `ModuleContract.robot_scoped`).

**Bridge = 순수 transport** — 키 확장(라우팅)만. domain body 는 손대지 않는다.
의도된 대가: 서비스가 scoped↔agnostic 바뀌면 call site 편집 필요 (robot_id 가
options↔req 이동) — **컴파일타임에 잡히는 기계적 수정**. "frontend 0 수정" 목표는
Bridge 에 서비스 의미를 넣는 비용이라 포기 (책임 분리 우선).

#### 2.7.2 robot-agnostic 모듈의 구현 패턴

- **runtime state → `dict[robot_id]`** (모듈 소유): 최신 frame / raw / preview on-off /
  seq. 실행 중에만 존재, 대부분 0~1 sparse.
- **config → resolve 가 robots.yaml 에서 lean 투영 주입** (모듈이 SSOT 복사·재보유 X).
  모듈별 필요만 — 스펙트럼: Calibration=`CalibrationRobotSpec`(motor_ids+has_camera) /
  Scan=`ScanRobotSpec`(kinematics+arm_specs, dataclass) / Scene3D=`robot_ids` 멤버십만
  (enabled+rgbd) / Waypoint·Detector=**0**. 투영 class 는 module.py 소유 (wire 아님) —
  bridge 의 RobotInfo 변환과 동형 (내부 config → module dep 는 apps 책임).
- `@subscriber` 는 framework wildcard → `payload.robot_id` 로 dict 캐시 (fleet 밖
  robot 은 skip).
- deployment yaml 에 `robots:` 없음 → `resolve_host_deps` 배선.

#### 2.7.3 acceptance (기능 검증 아니라 아키텍처 검증)

1. host-level (`self.robot_id` 없음)  2. robot-specific 정보 소유권 한 곳 (robots.yaml
SSOT, 모듈 복사 X)  3. runtime state ↔ config 명확 분리  4. **★ 새 로봇 추가 시 모듈
코드 0 수정** (진짜 리트머스).
- **눈속임 방지 테스트**: 단일 host-level 인스턴스로 **so101(6DOF) AND omx(5DOF)** 둘 다
  구동 (`test_single_instance_serves_so101_and_omx_isolated` 패턴 — 한쪽 하드코딩
  잔재는 다른쪽 경로에서 터짐). 한 robot 만 green = 기능 검증일 뿐.
- **폐기안 (다시 꺼내지 말 것)**: ① 생성 메타데이터 `robot_id_body_services`
  (contract 파생 목록 — SSOT 중복) ② Bridge 휴리스틱 자동주입 (agnostic vs global
  런타임 구분 불가 → "지금 global 없으니까" 타협 필요). 근거: 키의 robot_id(주소)와
  body 의 robot_id(req 필드)는 다른 레이어 — 자동주입 자체가 무근거.

#### Scope 결정 자리 — yaml primary, constructor 계약 검증

**scope 결정 주체 = deployment yaml**. 같은 Module class 가 host 별 다른 scope 가질 수 있음. constructor 는 그저 *계약 검증*.

```yaml
pc:
  modules:
    - module: CalibrationModule         # robots: 없음 → host-scoped 1 인스턴스
    - module: TaskModule
    - module: Bridge

pi_motor:
  modules:
    - module: MotorModule               # robots: 박힘 → per-robot N 인스턴스
      robots: [omx_f_0]
    - module: MotionModule
      robots: [omx_f_0]
```

framework 부팅 흐름:
```python
if "robots" in module_cfg:
    # 계약: __init__ 에 robot_id parameter 박혀있어야
    assert "robot_id" in inspect.signature(cls.__init__).parameters
    for rid in module_cfg["robots"]:
        instances.append(cls(robot_id=rid, ...))
else:
    # 계약: __init__ 에 robot_id parameter 박혀있으면 안 됨
    assert "robot_id" not in inspect.signature(cls.__init__).parameters
    instances.append(cls(...))
```

**규칙 표현**:
- ❌ "Module 이 robot_id 받으면 robot-scoped" (direction 반대)
- ✅ "robot-scoped 로 배치하려면 constructor 가 robot_id 받아야 한다"

차이 — 미래에 robot_id 받지만 scope 아닌 Module 가능 (예: `FleetMonitor(robot_id_filter=...)`). yaml 이 primary 이면 그 자리 자연 흡수.

base class / `@robot_scoped` 데코 박지 않음 — Module = plain class 유지 (§3 의 데코 인플레이션 회피).

## 3. 4 framework primitive

framework 가 제공하는 1급 시민 4 개. 이외 surface 박지 않음.

### 3.0 Contract key — 세 원칙

framework 의 contract key (service path / event topic / stream topic) 가 따르는 세 원칙 (hard rule):

**1. Explicit — 사람이 지정**

- 개발자가 key string 의 값 자체 명시
- 정의 = `contract.py` 의 nested `StrEnum` (string 정의 유일 위치)
- 모든 use site (service handler / subscriber / publisher / Mirror / caller) 가 key 를 직접 박음 — implicit lookup (예: class attribute / method `@service` spec lookup) 박지 X
- auto-derive (class name → topic regex) 박지 X

**2. Typed — class / enum (raw str X)**

- raw string 참조 박지 X (typo 차단)
- 모든 use site 가 typed identifier — `StrEnum value` / `event class` / `type hint`

**3. service 가리키는 방법 = 항상 `Service.X` enum 하나**

- method reference (`Module.method`) 박지 X — 박으면 service 가리키는 방법이 두 개 (enum + method ref) 가 됨
- Mirror / `runtime.call` / `@subscriber` / publish 모두 동일 패턴

**원칙 정합 = 다음 형태** (module 별 nested class + contract.py 통합):

```python
# modules/calibration/contract.py — 외부 Public Surface (Service / Event key + Pydantic payload)
from enum import StrEnum
from pydantic import BaseModel

class Calibration:
    class Service(StrEnum):
        ACTIVATE         = "srv/calibration/activate"
        SNAPSHOT_BUNDLE  = "srv/calibration/snapshot_bundle"

    class Event(StrEnum):
        ACTIVATED  = "event/calibration/activated"
        COMMITTED  = "event/calibration/committed"

# payload (event / req / res / bundle) — pure Pydantic data, key 정보 박지 X
class CalibrationActivated(BaseModel):
    robot_id: str
    bundle_id: int
# ... (req/res/bundle 자체 같은 파일 안)
```

```python
@service(Calibration.Service.ACTIVATE)                                  # handler
@subscriber(Calibration.Event.ACTIVATED)                                # subscriber
runtime.publish(Calibration.Event.ACTIVATED, event)                     # publisher
runtime.call(Calibration.Service.SNAPSHOT_BUNDLE, req, ResCls, ...)     # caller
Mirror(snapshot_service=Calibration.Service.SNAPSHOT_BUNDLE,
       change_topic=Calibration.Event.ACTIVATED, value_cls=Bundle, ...) # Reader (5 인자 모두 explicit)
```

상세 = §3.1 (service) / §3.2 (event) / §3.3 (Mirror) / §3.7 (ModuleRuntime).

**Nested class 패턴 — `Module.Service` / `Module.Event` / `Module.Stream`**:

- 도메인 별 단일 entry point — `Calibration.Service.X` / `Calibration.Event.X` 가 한 묶음
- 읽기 자체 자연어 — "Calibration 의 Service ACTIVATE", "Camera 의 Stream JPEG"
- IDE 자동완성: `Calibration.` → `Service` / `Event` 가지 자동 보임
- 도메인 격리 + module self-containment 정합 (§2.4 / §7.2)
- 새 종류 추가 = nested class 1개 (예: Camera 에 `Stream` 가지) — class 이름 prefix 반복 X

**contract.py = "Public Surface"** (외부 module 이 import 박는 모든 것):

| contract.py 안 | 이유 |
|---|---|
| ✅ `Module.Service` / `Module.Event` / `Module.Stream` (nested StrEnum) | 외부에서 `@subscriber` / `runtime.call` / Mirror 에 사용 |
| ✅ Event payload Pydantic class | `@subscriber` type hint / Mirror `change_event_cls` |
| ✅ Service Request / Response Pydantic class | caller 가 인자로 박음 |
| ✅ Bundle / Value Pydantic class (Mirror value_cls) | Mirror 의 cache type |
| ❌ SQLAlchemy ORM (`models.py`) | 영속성 internal — Repository 안에서만 |
| ❌ Repository / Business logic (`service.py`) | module.py 안에서만 |
| ❌ Module class (entry) | framework Runtime 만 instantiate |

기준 한 줄 = **"다른 module 이 이걸 import 박는가"**. 답이 yes 면 contract.py.

**진화 path** — 첫 박을 때 `contract.py` 단일 파일. 비대해지면 (예: 1000 줄+) `contract/` 패키지로:

```
contract/
  __init__.py     # re-export (외부 import path 자체 안 바뀜)
  keys.py
  events.py
  services.py
```

외부 module 의 import 자체 동일 (`from modules.X.contract import ...`) — 내부만 refactor.

**경로 convention** — 첫 chunk 가 통신 purpose 분리:

| prefix | 의미 | 형태 | nested class |
|---|---|---|---|
| `srv/` | request/response RPC | `srv/<module>/<verb>` / `srv/<module>/{robot_id}/<verb>` | `Module.Service` |
| `event/` | 상태 변화 notification (broadcast) | `event/<module>/<name>` / `event/<module>/{robot_id}/<name>` | `Module.Event` |
| `stream/` | 고빈도 raw 데이터 (camera / depth / pointcloud) | `stream/<module>/{robot_id}/<kind>` | `Module.Stream` |

`horibot/` prefix 박지 X — broker 단일 project, namespace 분리 motivation 약함. purpose 분리가 진짜 가치 (debugging / wildcard scope / Zenoh declare 명확).

### 3.1 `@service` — RPC handler

Service key 는 **사람이 explicit 지정** + **typed identifier (nested StrEnum)** — raw string 박지 X (§3.0 의 세 원칙).

```python
# modules/calibration/contract.py — string 정의 (유일)
from enum import StrEnum

class Calibration:
    class Service(StrEnum):
        ACTIVATE         = "srv/calibration/activate"
        SNAPSHOT_BUNDLE  = "srv/calibration/snapshot_bundle"


# modules/calibration/module.py
from .contract import Calibration

class CalibrationModule:
    @service(Calibration.Service.ACTIVATE)
    def activate(self, req: ActivateRequest) -> ActivateResponse:
        result = self._repo.get(req.result_id)
        if result is None:
            raise NotFound(f"result {req.result_id} 없음")    # exception propagation
        ...
        return ActivateResponse(ok=True)
```

- `req_cls` / `res_cls` = handler 의 type hint 에서 자동 추출.
- Service key = `@service` 인자의 StrEnum value — raw string 아님.
- framework Runtime 이 ZenohTransport 위에 service queryable 등록 (key = enum value).
- 같은 process caller = Zenoh same-session in-routing.
- 다른 process caller = Zenoh between-session.

**Caller — key + req + res_cls (모두 explicit)**:

```python
from modules.calibration.contract import Calibration, ActivateRequest, ActivateResponse

class OtherModule:
    async def do(self):
        try:
            result = await self.runtime.call(
                Calibration.Service.ACTIVATE,                  # service key
                ActivateRequest(result_id=10),                 # req
                ActivateResponse,                              # res_cls (return type narrow)
            )
        except RemoteError as e:
            if e.type == "NotFound": ...
        except TimeoutError:
            ...
```

framework 전체 단 하나의 규칙: service 가리키는 방법 = 항상 `Module.Service.X`. method reference 박지 X — Mirror / call / publish / subscribe 모두 같은 패턴.

`res_cls` 명시 — caller 가 받을 return type narrow + framework 가 wire payload decode 시 cls 직접 사용 (spec lookup indirection 없음).

**Robot-scoped service** — key 안 `{robot_id}` placeholder:

```python
class Motion:
    class Service(StrEnum):
        MOVE_L  = "srv/motion/{robot_id}/move_l"
        MOVE_J  = "srv/motion/{robot_id}/move_j"

class MotionModule:
    @service(Motion.Service.MOVE_L)
    def move_l(self, req: MoveLRequest) -> MoveLResponse: ...

# 호출 — caller 가 key + req + res_cls + robot_id 명시
await self.runtime.call(
    Motion.Service.MOVE_L, req, MoveLResponse, robot_id="omx_f_0",
)
```

framework register 시점 — Module instance 의 `self.robot_id` 로 placeholder 자동 substitute. caller 시점 — `robot_id=` 인자로 substitute.

**Error contract — exception propagation, envelope X**:

- 성공 path = `ServiceResponse[T]` (Pydantic generic) — 항상 valid `T`. caller 가 `res.success` 체크 박지 않음.
- handler exception → framework 가 type name + message 만 wire 통과 (traceback 박지 X).
- caller 측에서 `RemoteError(type=<name>, message=<msg>)` raise. caller 가 `except RemoteError as e: if e.type == "...":` 패턴 또는 generic catch.
- 같은 exception class 의 client-side 자동 raise (예: `NotFound` 실 class) 는 박지 않음 — Phase B detail.
- timeout = `Transport.call(timeout=5.0)` exceeded → `TimeoutError` raise.

이유 — Python 자연 = exception. caller 가 매 호출 `if not res.success: ...` envelope check 박는 자체 boilerplate. type-safe success path + exception path 분리가 정직.

### 3.2 `@subscriber` + `publish` — Domain event broadcast

Event 도 §3.0 원칙 정합 — key 가 publisher / subscriber 양쪽에서 직접 박힘. Event class = pure Pydantic data (key 정보 박지 X — separation of concerns).

```python
# modules/calibration/contract.py — Service + Event key + payload class 한 묶음
from enum import StrEnum
from pydantic import BaseModel

class Calibration:
    class Service(StrEnum):
        ACTIVATE         = "srv/calibration/activate"
        SNAPSHOT_BUNDLE  = "srv/calibration/snapshot_bundle"

    class Event(StrEnum):
        ACTIVATED  = "event/calibration/activated"
        COMMITTED  = "event/calibration/committed"

class CalibrationActivated(BaseModel):
    robot_id: str
    bundle_id: int

class CalibrationCommitted(BaseModel):
    robot_id: str
    bundle_id: int
```

**Owner 측 publish — key 첫 인자, event instance 두 번째**:

```python
class CalibrationModule:
    @service(Calibration.Service.ACTIVATE)
    def activate(self, req):
        result = self.repo.get(req.result_id)
        result.activate()
        self.repo.save(result)
        self.runtime.publish(
            Calibration.Event.ACTIVATED,
            CalibrationActivated(robot_id=req.robot_id, bundle_id=result.bundle_id),
        )
```

domain logic 바로 다음 줄에 어떤 event key 로 publish 하는지 보임.

**Subscriber 측 — `@subscriber(key)` factory + type hint 로 decode**:

```python
class AuditModule:
    @subscriber(Calibration.Event.ACTIVATED)
    def on_calibration_activated(self, event: CalibrationActivated):
        self.log_audit(event)
```

- event key = `@subscriber` 인자 (nested StrEnum value)
- event class = type hint (framework 가 payload decode)

**`@publishes` class decorator — (key, event_cls) pair self-declare**:

self-doc + contract.ts auto-generate 용. 실 publish 강제 X — declare 안 된 pair 도 publish 동작.

```python
@publishes(
    (Calibration.Event.ACTIVATED, CalibrationActivated),
    (Calibration.Event.COMMITTED, CalibrationCommitted),
)
class CalibrationModule:
    ...
```

**Robot-scoped event** — topic 안 `{robot_id}` placeholder:

```python
class Motion:
    class Event(StrEnum):
        COMPLETED = "event/motion/{robot_id}/completed"

class MoveCompleted(BaseModel):
    robot_id: str
    ...

# publish — event.robot_id 가 placeholder substitute
self.runtime.publish(
    Motion.Event.COMPLETED,
    MoveCompleted(robot_id=self.robot_id, ...),
)

# subscribe — framework 가 placeholder → Zenoh wildcard `*` substitute
@subscriber(Motion.Event.COMPLETED)
def on_completed(self, event: MoveCompleted):
    ...
```

framework 자동: publish 시점에 `event.robot_id` 로 substitute, subscribe 는 transport wildcard 로 substitute 후 payload 의 `robot_id` 로 self-filter (Mirror 도 동일).

### 3.3 `Mirror[T]` — Cross-module state read

> ✅ **STATUS: 활성 (2026-07-07) — 첫 consumer = MotionModule.calibration.** 옛 deferred (2026-07-02, consumer 0) 해제. 근거: "boot-query 1회" 는 분산 부팅 순서 종속성 (PC 늦으면 motion 이 무보정으로 영원히 운전 — silent degradation) 을 만들었다. Mirror 가 **liveliness** (owner 의 snapshot service 생존을 transport 가 관측 — anchor #23) 로 완성되어: owner 늦은 부팅 / 재시작 (죽어있는 동안 데이터 변경, event 영영 안 옴) 전부 자동 refetch 수렴. `on_change(old, new)` 훅 (`@mirror.on_change` decorator, 값이 실제로 바뀐 전이만 발화) 으로 consumer 반응. Motion 정책: 없음→값 = runner idle 때 live 적용 / 값→값′ = `calibration_stale` 표시만 (변경은 재부팅 유지). 상태는 `TcpState.calibration_applied/stale` 로 상시 표면화.

가장 중요한 primitive. Reader 쪽 boilerplate (snapshot fill / subscribe / cache) 흡수.

```python
from modules.calibration.contract import Calibration, CalibrationActivated, CalibrationBundle, SnapshotRequest

class MotionModule:
    calibration: Mirror[CalibrationBundle] = Mirror(
        snapshot_service=Calibration.Service.SNAPSHOT_BUNDLE,         # service key
        snapshot_req=lambda self: SnapshotRequest(robot_id=self.robot_id),  # req factory
        change_topic=Calibration.Event.ACTIVATED,                     # event key
        value_cls=CalibrationBundle,                                  # snapshot res_cls + cache type
        change_event_cls=CalibrationActivated,                        # event class (decode)
    )

    @service(Motion.Service.MOVE_L)
    def move_l(self, req):
        cal = self.calibration.value           # 매 호출 fresh cache read
        urdf_joints = [j + cal.joint_offsets[i] for i, j in enumerate(joints)]
        tf = cal.hand_eye                       # sub-field access — consumer 책임
        ...
```

Mirror 의 5 인자:
- `snapshot_service` = service key (StrEnum value). framework 가 호출할 RPC.
- `snapshot_req` = req factory `Callable[[self], BaseModel]`. Module instance 박힌 후 호출 — `self.robot_id` 등 활용 가능. robot-agnostic Reader 면 `lambda self: SnapshotRequest()`.
- `change_topic` = event key (StrEnum value). subscribe 할 topic.
- `value_cls` = snapshot response type = cache 의 T. `Mirror[T]` 의 T 자체.
- `change_event_cls` = event class. change_topic payload decode 시 type.

framework 전체 단 하나 규칙 (service = `Module.Service.X`, event = `Module.Event.X`) 정합 — Mirror 도 method reference 박지 X.

framework 자동:
1. Module start 시 `runtime.call(snapshot_service, snapshot_req(self), value_cls)` → local cache fill (단 fail OK, §3.3.1 참조).
2. `change_topic` subscribe → 받으면 cache refetch.
3. `self.calibration.value` access = cache read.
4. Module stop 시 subscription unregister.

Owner 쪽은 standard service + event 박는 것만, Mirror 가 wiring.

**명시적 mapping** (5 인자 모두 key + Pydantic class) 가 정직. method reference / class attribute lookup 0. framework 전체 *service 가리키는 방법 = 항상 `Module.Service.X`* 한 패턴.

#### 3.3.1 Startup ordering — empty + fallback fetch

```python
Module.start():
    ① state = INITIALIZING
    ② event_buffer = []
    ③ subscribe(change_event):
          if state == INITIALIZING: event_buffer.append(event)
          else: cache = fetch_snapshot()      # event 받음 → 다음 snapshot fetch 로 갱신
    ④ snapshot try (background, non-blocking):
          success → cache = result
          fail (Owner 안 떠 있음) → cache = None
    ⑤ buffer replay:
          if any event in buffer: cache = fetch_snapshot()  (Owner 가 그 사이 떴을 수 있음)
    ⑥ state = READY
```

- **blocking retry 박지 않음** — Owner 가 안 떠 있어도 Reader Module 의 start 가 영원히 block 되면 안 됨 (분산 partition tolerance).
- **race 차단 — buffer + replay** — subscribe 시점부터 받은 event 를 INITIALIZING 동안 buffer. snapshot 적용 후 buffer 가 비어있지 않으면 fresh fetch (가장 단순한 구현, 마지막 변경값으로 수렴).
- snapshot 실패 후에도 *첫 change event* 가 fallback fetch trigger — 결국 fresh cache 도달.

#### 3.3.2 Value access — `.value` 매 access fresh + `is_ready` flag

```python
class Mirror[T]:
    _cache: T | None = None
    _initialized: bool = False
    
    @property
    def is_ready(self) -> bool:
        return self._initialized       # snapshot/event 한 번이라도 받았나
    
    @property
    def value(self) -> T:
        if not self._initialized:
            raise NotReady(f"Mirror[{T.__name__}] 아직 snapshot/event 못 받음")
        return self._cache
```

**계약**:
- `self.calibration.value` 매 access 가 **fresh cache read**. consumer 가 local variable 에 capture 박지 X (stale 위험).
- `is_ready=False` 자리는 application 책임 — `if not self.calibration.is_ready: raise/return error`. *"값이 empty domain value"* (예: `bundle.hand_eye == identity`) 와 *"아직 안 받음"* 자리 명확 분리.
- **`.value` 는 partially updated state 노출 X** — Mirror update 가 *event callback thread* 에서 일어남, service handler 가 다른 thread 에서 access. 두 access 사이 race window 가 partial state (예: cache 만 새값, initialized 옛값) 보이면 안 됨. 구현 (lock / atomic reference swap / RCU / actor model) 은 자유 — 운영 model 바뀌면 함께 진화.

#### 3.3.3 Bundle 단위 — sub-field 분리 박지 않음

Mirror[T] 의 T = **도메인의 atomic 단위 (Bundle)**. 같은 BA / 같은 commit 이 만든 산출물은 한 type 으로 묶음 — sub-field 별로 4-5 개 Mirror 박지 않음.

예 — Calibration:
```python
class CalibrationBundle(BaseModel):
    joint_offsets: list[float]
    link_offsets:  list[LinkOffset]
    sag_offsets:   list[float]
    hand_eye:      Transform4x4
    intrinsic:     CameraIntrinsic
    commit_time:   datetime
    bundle_id:     int
```

이유:
- BA atomic = 한 ResultBundle. 사용자가 "joint_offset 만 commit" 박지 X — BA 가 동시 산출.
- 4-5 sub-field 별 별도 Mirror 박는 자체 *기존 backend implementation detail (4 종 npz 파일 분리) 의 매몰*. 도메인 의도 X.
- Bundle size 작음 (수 KB). 모든 consumer 가 전체 받아도 transport 비용 무관.
- consumer 가 sub-field 별 access 자체 책임 — `cal.hand_eye`, `cal.joint_offsets[i]`.

#### 3.3.4 Effective apply — framework 안 박힘, consumer 책임

> ⚠️ **이 절의 calibration 예제는 SUPERSEDED (2026-07-02).** 아래 `link_offsets 변경 → _rebuild_kinematics` 런타임 재로드는 **실제 calibration 에서 제거됨** — Bundle 은 boot-time config 라 Motion 은 start() 에서 1회 build 하고 런타임 rebuild 하지 않는다 ([calibration_module_boundary.md §10.2](calibration_module_boundary.md): "Mirror 니까 실시간이어야 한다" 는 아키텍처적 연역이었고 실제 트리거가 없었음). 아래는 *만약* control-correctness-state consumer 가 있었다면 effective-apply 를 framework 가 아니라 consumer 가 처리한다는 **패턴 illustration** 으로만 유지.

Mirror cache 갱신 = framework 자동. 단 *effective apply* (architectural side-effect) 는 consumer 책임 — framework 가 `@on_mirror_change` 같은 magic 데코 박지 X, 그저 **일반 `@subscriber(ChangeEvent)`** 박아 자기 도메인 처리.

```python
class MotionModule:
    calibration: Mirror[CalibrationBundle]   # 위 Mirror(...) 선언과 동일 instance

    @subscriber(Calibration.Event.ACTIVATED)             # Mirror 와 같은 event key
    def on_calibration_change(self, event: CalibrationActivated):
        # Mirror cache 는 framework 가 갱신함
        # 단 PyBullet kinematics 는 부팅 1회 load — 재로드는 consumer 책임
        if event.changed.contains("link_offsets"):
            self._rebuild_kinematics(self.calibration.value)
            # trajectory 실행 중이면 안전 timing 대기 후 rebuild — consumer 도메인 책임
        # joint_offsets / sag_offsets / hand_eye = 매 access fresh, rebuild 불필요
```

framework 가 *graceful restart / rebuild* 자체 처리하지 않음 — Module 이 자기 architectural side-effect 알아 처리. trajectory 중단 timing, queue drain 등 도메인 정책.

#### 3.3.5 동기화 패턴 — invalidate+refetch only (push update 박지 X)

Mirror 의 cache 갱신 방식 두 후보 패턴:

| 패턴 | event 역할 | Mirror 동작 |
|---|---|---|
| **Push Update** | event payload = 최신 상태 그 자체 | event 받으면 cache = event payload (서비스 호출 X) |
| **Invalidate + Refetch** (현재) | event = 변경 알림 (notification) | event 받으면 snapshot service 재호출 → cache 갱신 |

**현재 spec = Invalidate + Refetch 단일**. push update 패턴 박지 X.

이유:

1. **Bundle atomic invariant 보존** (§3.3.3) — Bundle 은 한 BA / 한 commit 의 atomic 산출물. push update 박으면 event payload 가 진실 source 가 되어 snapshot 과 diverge 위험. snapshot 호출이 항상 최신 보장.

2. **Mirror 의 진짜 use case = 다른 Module 의 event 가 trigger** — Owner 가 자기 전체 상태 모를 수 있음:
   ```
   CameraModule → publish(CameraIntrinsicChanged)
       ↓
   CalibrationModule 의 Mirror 가 event 받음
       ↓
   "Calibration 이 영향 받음" → snapshot 호출
       ↓
   최신 CalibrationBundle (intrinsic + extrinsic + sag 합산) cache
   ```
   여기서 event = trigger 신호일 뿐 payload 아님. push update 불가능 — event publisher 가 전체 Bundle 모름.

3. **same-module event 도 invalidate+refetch 로 통일** — `CalibrationActivated` 처럼 Owner 가 자기 Bundle 알 때도 같은 path. 두 갈래 비대칭 회피. wasted RPC cost 작음 (Bundle 수 KB, 변경 빈도 낮음 — calibration 은 BA 시점만).

4. **push update 필요하면 Mirror 안 박고 `@subscriber` 직접** — event 가 최신 상태 그 자체면 framework 흡수 가치 작음. `@subscriber(Module.Event.X) def on(self, e): self._cache = e` 박으면 충분. Mirror 의 진짜 가치 = "Notification + auto Refetch" 흡수, push update 패턴은 이 가치 안 만족.

새 use case 발견 시 본 결정 재검토. 단 첫 박힘은 invalidate+refetch 단일 path.

### 3.4 Transport (Zenoh 단일)

**Transport 의 의미** — Zenoh 추상화 객체가 아니라, framework 가 Module 에게 *허용한 통신 어휘 그 자체*. 4 surface (publish / subscribe / call / register_service) 외 통신 박지 X — Module 짤 때 첫 질문이 "Zenoh 로 어떻게 보내지?" 가 아니라 "이건 4 어휘 중 어떤 거지?" 가 되도록 강제. 결과 = 모든 Module 의 통신 모양이 균일. Module 코드에 `import zenoh` 절대 안 나옴 (import boundary §2.5) — 이건 "Zenoh 갈아끼우기" 가 목적이 아니라 **"Module 이 4 어휘 밖으로 못 나가게 막는 차단막"**.

Module 코드는 transport object 본 적 없음. `self.runtime.publish` / `self.runtime.call` 만 호출 (`ModuleRuntime` Protocol — §3.7). `@subscriber` 는 데코레이터로 framework 가 wire — Module 코드 직접 subscribe 호출 X.

framework Runtime 이 transport 를 hold:

- **ZenohTransport** (infra/transport/zenoh.py) — Zenoh session + `put` / `declare_subscriber` / `declare_queryable`. 같은 process / 다른 process 동일 어휘.

**Wire encoding — Pydantic + msgpack layered (DIP)**:

```
Module                  Pydantic                  Transport
─────────               ────────                  ─────────
event instance          schema validation         msgpack bytes
   │                       │                          │
   ├─ runtime.publish ─→ model_dump() ─→ msgspec ─→ transport.publish
       (key, event)       (dict)         .encode      (str(key), bytes)

   ◀── decode_event ───── model_validate ◀── msgspec ◀── subscriber callback
                          (instance)         .decode     (bytes)
```

- **Pydantic** = schema validation + Python ↔ dict 변환. Module 코드는 도메인 의도만 표현.
- **msgspec.msgpack** = wire serialization (transport boundary). Module 코드는 모름.
- Module 은 Pydantic 만 알고, Transport boundary 가 msgpack 처리 — DIP 정합.
- native `bytes` field pass-through — JSON 의 base64 33% overhead 회피. camera JPEG / depth zstd / pointcloud 에서 영향 큼.

("Wire encoding" 의 *wire* 어휘는 transport boundary 의 raw byte 의미 — §3.0 의 contract key 와 다른 layer.)

```python
# framework/contract/publisher.py
import msgspec

def encode_event(event: BaseModel) -> bytes:
    return msgspec.msgpack.encode(event.model_dump())

def decode_event(event_cls: type[T], payload: bytes) -> T:
    return event_cls.model_validate(msgspec.msgpack.decode(payload))
```

key lookup helper 박지 X — key 가 use site (publish 첫 인자 / `@subscriber` 인자) 에 직접 박혀있어 추가 lookup 불필요.

같은 process 안 Module 간 호출도 Zenoh same-session 통과 — `session.put` → in-session routing → subscriber callback. wire 0 (TCP/UDP 안 거침), application boundary 의 Pydantic encode/decode + Python ↔ Rust ZBytes copy 만 비용.

**LocalTransport (process-local `dict[key] → callback` direct dispatch) 박지 않음.** 측정 결과 ([backend/scripts/bench_transport.py](../backend/scripts/bench_transport.py)):

| Payload | Zenoh same-session | LocalTransport 가 절감 |
|---|---|---|
| Pydantic small (32B) | 3.5us | ~3.5us, 무관 |
| JPEG 200KB × 30Hz | 52us = 1.5ms/sec | 무관 |
| **PointCloud 5MB × 30Hz** | 1.27ms = 38ms/sec | ~4% CPU × N consumer |

5MB transport 비용 중 ~97% 가 Python ↔ Rust ZBytes boundary memcpy. Zenoh in-session routing 자체는 28us. 즉 LocalTransport 가 우회하는 진짜 비용은 *boundary memcpy*.

큰 ndarray fanout 만 의미 있는 절감 (~4% × N CPU). 단 framework 두 갈래 (Transport 두 impl + resolver + behavior 일관성) 유지 비용보다 작음. **Zenoh 단일 + derived read model 패턴** (§3.5) 으로 카메라 ~13% CPU 도달 — 추가 7-8% 는 측정 후 진짜 bottleneck 으로 드러나면 그때 박음.

### 3.5 Derived read model Module — decode dedup 패턴

framework primitive 가 아닌 **Module 패턴**. 큰 payload (카메라 JPEG, depth zstd) 의 decode 가 N consumer × decode 비용으로 누적되는 문제를 푸는 표준 형태. framework 는 모름 — 그저 일반 Module + `@subscriber` + `publish` + `@service`.

stream key 도 §3.0 원칙 정합 — publish / subscribe 양쪽에서 key 직접 박힘. 큰 payload (jpeg bytes / zstd depth) 는 `bytes` field (msgpack native bytes pass-through — §3.4).

**naming — `Event` vs `Stream` 분리**: `event/` prefix = 상태 변화 notification, `stream/` prefix = 고빈도 raw 데이터. nested class 이름도 `Camera.Stream` (Event 아님) — stream 은 event 가 아님.

```python
# modules/camera/contract.py
from enum import StrEnum
from pydantic import BaseModel

class Camera:
    class Service(StrEnum):
        DECODED_SNAPSHOT = "srv/camera/{robot_id}/decoded_snapshot"

    class Stream(StrEnum):
        JPEG          = "stream/camera/{robot_id}/jpeg"
        DEPTH_FRAME   = "stream/camera/{robot_id}/depth_frame"
        DECODED       = "stream/camera/{robot_id}/decoded"

# payload — pure data, key 정보 박지 X
class CameraJpegFrame(BaseModel):
    robot_id: str
    timestamp: float
    jpeg_bytes: bytes

class CameraDecodedFrame(BaseModel):
    robot_id: str
    timestamp: float
    width: int
    height: int
    ndarray_bytes: bytes        # 압축 안 된 BGR raw
```

```
Pi process:
  CameraDriver Module (robot-scoped, self.robot_id)
      ├─ RealSense capture
      ├─ JPEG encode + zstd depth encode
      └─ self.runtime.publish(
             Camera.Stream.JPEG,                                      ← stream key 첫 인자
             CameraJpegFrame(robot_id=..., jpeg_bytes=...),           ← event instance
         )   ← ~600KB × 30Hz
           │
           ▼ Zenoh (Pi → PC, wire = topic substituted, payload = encoded event)
           │
PC process:
  CameraDecoded Module (robot-scoped)              ← derived read model
      @subscriber(Camera.Stream.JPEG)
      def on_jpeg(self, event: CameraJpegFrame):    ← type hint 으로 decode
          ndarray = cv2.imdecode(event.jpeg_bytes, IMREAD_COLOR)   ← decode 1회
          self.runtime.publish(
              Camera.Stream.DECODED,
              CameraDecodedFrame(robot_id=event.robot_id, ...),
          )
           │
           ▼ Zenoh same-session (PC 안)
           │
      ┌────┴────┬────────────┐
      ▼         ▼            ▼
   Detector  Calibration   Scene3D
      각자 @subscriber(Camera.Stream.DECODED) + event: CameraDecodedFrame
```

**핵심** — Decode 가 별도 Module 책임. 각 consumer 가 decode 박지 않음.

측정 ([backend/scripts/bench_transport.py](../backend/scripts/bench_transport.py)):
- JPEG 1280x720 decode = **4.34ms**.
- 각 consumer 가 decode: 4.34ms × 30Hz × N = 130 × N ms/sec (N=3 → **39% CPU**).
- decode dedup 만 (Zenoh 단일): 4.34ms × 30 + ndarray transport × N = (130 + ~21 × N) ms/sec (N=3 → **21% CPU**).
- decode dedup + LocalTransport: 130 ms/sec (N=3 → **13% CPU**, 추가 절감 8%).

→ **decode dedup 이 first-order 절감** (39% → 21%). LocalTransport 의 추가 8% 는 단순성 우선으로 박지 X.

비슷한 패턴 적용 후보:
- **CameraDecoded** — JPEG → ndarray.
- **DepthDecoded** — zstd depth → uint16 ndarray + intrinsic.
- **TcpState** — joint → FK → TCP pose (sag 보정 포함) — 기존 backend 의 `motion_node._on_motor_state_publish_tcp` 가 한 일.
- **JointRad** — raw int → rad (joint_offset 적용) — 기존 `JointStateCache` 가 한 일.

framework 가 모름 — Module 의 한 유형 힌트.

### 3.6 Runtime lifecycle — instantiate → register → start

framework primitive 가 아닌 **Runtime contract**. `Runtime.start()` 의 부팅 순서:

```
① 모든 Module instantiate
       → constructor 호출 (DI: Repository / ObjectStore / robot_id 등 주입)
       → 모든 Module 의 객체 self 만들어짐
       
② 모든 Module 의 @service / @subscriber 등록
       → ZenohTransport 에 queryable / subscriber declare
       → 이 시점에 service 들이 cluster 안 visible
       
③ 모든 Module 의 start() 호출
       → Mirror snapshot fetch / background thread 시작 / hardware init 등
       → Mirror 가 다른 Module 의 service 호출하므로 ② 이후 박힘
       
④ Heartbeat / background workers
```

**왜 이 순서**:
- ③ 의 Mirror snapshot 이 다른 Module 의 `@service` 호출. ② 가 아직 안 됐으면 service register 안 된 상태 → snapshot fail (§3.3.1 의 fallback 으로 떨어짐. 단 항상 fallback 으로 떨어지는 건 design 의도 X).
- ② 와 ③ 분리 = framework 의 진짜 contract. instantiate + register 가 *모든 Module 동시* 끝난 후 start.

**③ 중간 실패 = 이미 start 된 Module rollback** — `start()` 가 예외(또는 SystemExit
등 BaseException) 로 중단되면, 그 전까지 성공한 Module 을 역순 `stop()` + endpoint
undeclare 후 re-raise. 방치하면 앞 Module 의 background thread / uvicorn task 가 좀비로
남아 프로세스 종료 자체를 막는다 (2026-07-07 사고: 유령 backend 가 :8000 점유 →
BridgeModule.start 실패 → rollback 없던 시절엔 pytest 프로세스 hang, [[project-verify-hang-stale-backend]]).
그래서 **BridgeModule.start 는 uvicorn 에 넘기기 전에 소켓을 직접 pre-bind** 한다 —
uvicorn 이 bind 실패 시 `sys.exit(1)` (SystemExit 로 이벤트 루프째 붕괴, rollback 스킵)
하는 걸 평범한 `RuntimeError` 로 바꿔 위 rollback 경로에 태우기 위함. 손으로 bind 하는
코드를 "불필요" 로 보고 되돌리지 말 것.

같은 process 의 Module 간 호출은 ZenohSession same-session in-routing 통과. 다른 process Owner 와는 Zenoh discovery / partition tolerance (§3.3.1 의 empty + fallback 그대로).

### 3.7 ModuleRuntime — Module 의 통신 surface

Module 이 framework 에 publish / call 요청하는 surface. Protocol 박고 constructor 로 주입.

```python
# framework/runtime/api.py
class ModuleRuntime(Protocol):
    """Module 이 Framework 에 요청하는 통신 surface."""

    def publish(self, key: str, event: BaseModel) -> None:
        """event publish. key 첫 인자 (explicit, typed StrEnum), event instance."""
        ...

    async def call(
        self,
        key: str,                              # contract key (StrEnum value)
        req: BaseModel,
        res_cls: type[TRes],                   # explicit — return type narrow + decode 시 사용
        *,
        robot_id: str | None = None,           # robot-scoped service 만 박힘
        timeout: float = 5.0,
    ) -> TRes:
        """service 호출. key + req + res_cls 세 인자 모두 explicit. method reference 박지 X."""
        ...
```

Module 측:

```python
from modules.motion.contract import Motion, MoveCompleted
from modules.calibration.contract import Calibration, CalibrationBundle, SnapshotRequest
from modules.motor.contract import Motor, SetTorqueRequest, SetTorqueResponse

class MotionModule:
    def __init__(
        self,
        runtime: ModuleRuntime,
        robot_id: str,
        repo: CalibrationRepository,
    ):
        self.runtime = runtime
        self.robot_id = robot_id
        self._repo = repo

    @service(Motion.Service.MOVE_L)
    def move_l(self, req: MoveLRequest) -> MoveLResponse:
        # key + event 두 인자 모두 typed
        self.runtime.publish(
            Motion.Event.COMPLETED,
            MoveCompleted(robot_id=self.robot_id, ...),
        )
        ...

    async def some_caller(self):
        # 다른 service 호출 — key + req + res_cls (모두 explicit)
        bundle = await self.runtime.call(
            Calibration.Service.SNAPSHOT_BUNDLE,                            # service key
            SnapshotRequest(robot_id=self.robot_id),                        # req
            CalibrationBundle,                                              # res_cls
        )
        # robot-scoped target — robot_id 명시
        await self.runtime.call(
            Motor.Service.SET_TORQUE,
            SetTorqueRequest(enabled=True),
            SetTorqueResponse,
            robot_id=self.robot_id,
        )
```

Runtime 측 — 인스턴스화 시점에 transport 어휘로 adapter 박아 inject:

```python
class _TransportRuntime:                # ModuleRuntime Protocol 만족
    def __init__(self, transport: Transport):
        self._transport = transport

    def publish(self, key: str, event: BaseModel) -> None:
        topic = str(key)                                # StrEnum value → str
        if "{robot_id}" in topic:
            # source = event payload 의 robot_id field (Module scope 무관 — uniform)
            assert hasattr(event, "robot_id"), (
                f"key {topic!r} 에 {{robot_id}} placeholder 박혀있지만 "
                f"event {type(event).__name__} payload 에 robot_id field 없음"
            )
            topic = topic.format(robot_id=event.robot_id)
        self._transport.publish(topic, encode_event(event))

    async def call(self, key, req, res_cls, *, robot_id=None, timeout=5.0):
        key_str = str(key)                               # StrEnum value → str
        if "{robot_id}" in key_str:
            assert robot_id is not None, (
                f"service {key_str} 가 robot-scoped — call 시 robot_id= 인자 명시 필요"
            )
            key_str = key_str.format(robot_id=robot_id)
        payload_bytes = await self._transport.call(key_str, encode(req), timeout)
        return decode(res_cls, payload_bytes)            # res_cls 명시 — spec lookup 없음

# Runtime 부팅:
runtime_api = _TransportRuntime(transport)
instance = MotionModule(runtime=runtime_api, robot_id=rid, repo=repo)
```

**placeholder substitution source — 네 경로**:

| 위치 | source | 시점 |
|---|---|---|
| service queryable register | Module instance 의 `self.robot_id` (robot-scoped Module 만) | Runtime register 시 |
| event publish | event payload 의 `robot_id` field | `runtime.publish(event)` 시 |
| service call (caller side) | caller 의 `robot_id=` kwarg | `runtime.call(key, req, res_cls, robot_id=...)` 시 |
| event subscribe (robot-scoped event) | placeholder → Zenoh wildcard (transport detail) | `@subscriber` register 시 |

**event subscribe 의 wildcard 는 framework contract X** — transport (Zenoh) 의 single-chunk `*` 활용일 뿐, framework primitive 어휘로 노출 X. `@subscriber("*")` 같은 implicit pattern 금지. robot-scoped event subscribe 시 framework 가 placeholder 를 transport wildcard 로 substitute, 사용자 코드에 wildcard 어휘 등장 X.

→ robot-agnostic Module 이 robot-scoped event publish 도 자연 동작 (event payload 의 robot_id field 활용). subscriber 는 wildcard 후 payload `event.robot_id` 로 self-filter (Mirror 도 동일).

**왜 base class / setattr / ctx 박지 않나**:

- **base class** (`class MotionModule(Module)`) — backend/ `BaseNode` 부풀음 경험 (15+ method 누적: publish / log / heartbeat / lifecycle / placeholder expand …) 반복. 얇게 박아도 `_transport` 채우려면 setattr magic 또는 `super().__init__` 강제 → §10.6 의 "lifecycle 강제 X" 위반.
- **setattr inject** (`instance.publish = transport.publish`) — pyright 가 `self.publish` 못 보고 IDE 자동완성 X. §3.4 의 "4 surface 밖 통신 박지 X" 가 IDE 에 보이지 않으면 흔들림.
- **ctx (`RuntimeContext`)** — "context" 가 너무 광범위 (HTTP request context / Go context 와 충돌). 한 문장 정의 fail.
- **composition (`ModuleRuntime` Protocol)** — 명시 deps + Protocol type-safe + naming convention (`X 가 사용하는 Y` — `CalibrationRepository` / `JointStateCache` 와 정합).

**discipline — ModuleRuntime 에 박힐 surface 기준** (hard rule X, PR review 가이드):

| 후보 | ModuleRuntime | constructor 별도 parameter |
|---|---|---|
| publish (event broadcast) | ✅ | |
| call (RPC) | ✅ | |
| logger / metrics / clock | | ✅ |
| repository / object_store | | ✅ |
| Mirror (cross-module read) | | Mirror[T] descriptor (§3.3) |

기준 = **"Module 간 통신 surface 인가 vs 별도 framework concern 인가"**. 후자 = constructor 별도 parameter default. ModuleRuntime 에 박을 경우 PR description 에 정당화 명시.

평가 기준일 뿐 hard list 아님 — 새 후보 들어올 때마다 위 기준으로 판단.

## 4. Owner / Reader 비대칭 — code 형태

> ⚠️ **아래 §4.1–§4.2 의 Calibration↔Motion `Mirror[CalibrationBundle]` 코드는 Mirror 메커니즘 illustration (stand-in) 이다 (2026-07-02).** 실제 calibration 은 boot-time configuration 이라 Mirror 를 쓰지 않고 Motion 은 boot-time `snapshot_bundle` query 로 읽는다 ([calibration_module_boundary.md §6](calibration_module_boundary.md), anchor #2). Owner/Reader **비대칭 원칙 자체는 유효** — 다만 Reader 의 실제 접근이 Mirror 가 아니라 boot-query 인 경우 (calibration) 와 per-request service call 인 경우 (§4.4 Detector) 가 현 도메인의 실제 형태.

### 4.1 Owner side — Calibration Module

```python
# modules/calibration/contract.py — 외부 Public Surface
from enum import StrEnum
from datetime import datetime
from pydantic import BaseModel

class Calibration:
    class Service(StrEnum):
        ACTIVATE         = "srv/calibration/activate"
        SNAPSHOT_BUNDLE  = "srv/calibration/snapshot_bundle"

    class Event(StrEnum):
        ACTIVATED  = "event/calibration/activated"
        COMMITTED  = "event/calibration/committed"

# event payload
class CalibrationActivated(BaseModel):
    """active bundle 변경 (시스템 effective)."""
    robot_id: str
    bundle_id: int

class CalibrationCommitted(BaseModel):
    """새 bundle 저장 완료 (capture / BA → DB insert)."""
    robot_id: str
    bundle_id: int

# service request / response
class ActivateRequest(BaseModel):
    robot_id: str
    result_id: int

class ActivateResponse(BaseModel):
    ok: bool

class SnapshotRequest(BaseModel):
    robot_id: str

# Mirror value (snapshot service 의 response = Mirror 의 cache type)
class CalibrationBundle(BaseModel):
    """한 BA / 한 commit 의 atomic 단위. consumer 의 Mirror[T] type."""
    robot_id: str
    bundle_id: int
    joint_offsets: list[float]
    link_offsets:  list[LinkOffset]
    sag_offsets:   list[float]
    hand_eye:      Transform4x4
    intrinsic:     CameraIntrinsic
    commit_time:   datetime


# modules/calibration/models.py — SQLAlchemy ORM (internal)
class CalibrationResult(Base):
    __tablename__ = "calibration_results"
    id: Mapped[int] = mapped_column(primary_key=True)
    run_id: Mapped[int]
    transform: Mapped[bytes]       # 4x4 matrix serialize
    sigma_rot: Mapped[float]
    sigma_t: Mapped[float]
    is_active: Mapped[bool] = mapped_column(default=False)


# modules/calibration/repository.py — internal
class CalibrationRepository:
    def get_active_bundle(self, robot_id: str) -> CalibrationBundle | None: ...
    def save_result(self, robot_id: str, result: CalibrationResult) -> None: ...
    def activate(self, robot_id: str, result_id: int) -> None: ...    # atomic toggle


# modules/calibration/module.py — robot-agnostic, host 당 1 인스턴스
from .contract import (
    Calibration, CalibrationActivated, CalibrationCommitted,
    ActivateRequest, ActivateResponse, SnapshotRequest, CalibrationBundle,
)

@publishes(
    (Calibration.Event.ACTIVATED, CalibrationActivated),
    (Calibration.Event.COMMITTED, CalibrationCommitted),
)
class CalibrationModule:
    def __init__(self, runtime: ModuleRuntime, repo: CalibrationRepository):
        self.runtime = runtime
        self._repo = repo

    @service(Calibration.Service.ACTIVATE)
    def activate(self, req: ActivateRequest) -> ActivateResponse:
        self._repo.activate(req.robot_id, req.result_id)    # atomic toggle, transaction
        bundle = self._repo.get_active_bundle(req.robot_id)
        self.runtime.publish(
            Calibration.Event.ACTIVATED,
            CalibrationActivated(robot_id=req.robot_id, bundle_id=bundle.bundle_id),
        )
        return ActivateResponse(ok=True)

    @service(Calibration.Service.SNAPSHOT_BUNDLE)
    def snapshot_bundle(self, req: SnapshotRequest) -> CalibrationBundle:
        bundle = self._repo.get_active_bundle(req.robot_id)
        if bundle is None:
            raise NotFound(f"active calibration bundle 없음 (robot={req.robot_id})")
        return bundle
```

특징:
- **robot-agnostic** Module — host 당 1 인스턴스. 매 service request 안 `robot_id` field 로 dispatch.
- Repository 가 `robot_id` parameter 받음 — DB 의 `robot_id` column 으로 자연 multi-tenant.
- 도메인 event 의 `robot_id` field — Reader 측 Mirror 가 자기 robot 의 event 만 필터.
- `repo.save()` / `publish(...)` 직접. framework 가 mutation tracking 으로 자동화 X.
- `snapshot_bundle` 이 *명시적* service. Reader 가 부팅 시 호출할 endpoint.
- domain logic (active toggle 의 atomic 보장) Module 안.

### 4.2 Reader side — Motion Module

```python
# modules/motion/module.py
# robot-scoped — per-robot 인스턴스 (yaml 의 robots: [...] 박힘)
from modules.motion.contract import Motion, MoveLRequest, MoveLResponse
from modules.calibration.contract import (
    Calibration, CalibrationActivated, CalibrationBundle, SnapshotRequest,
)
from modules.motor.contract import Motor, MotorCmdJoint

class MotionModule:
    calibration: Mirror[CalibrationBundle] = Mirror(
        snapshot_service=Calibration.Service.SNAPSHOT_BUNDLE,
        snapshot_req=lambda self: SnapshotRequest(robot_id=self.robot_id),
        change_topic=Calibration.Event.ACTIVATED,
        value_cls=CalibrationBundle,                                    # cache T + snapshot res_cls
        change_event_cls=CalibrationActivated,                          # event class (decode)
        # framework 가 wire:
        #   snapshot 호출 = runtime.call(snapshot_service, snapshot_req(self), value_cls)
        #   event 필터링 = robot_id == self.robot_id (Mirror 가 payload.robot_id 검사)
    )

    def __init__(self, runtime: ModuleRuntime, robot_id: str):
        self.runtime = runtime
        self.robot_id = robot_id
        self._kinematics: Kinematics | None = None     # rebuild on link_offset change

    def start(self):
        # Mirror 가 ready 되면 첫 kinematics build
        if self.calibration.is_ready:
            self._kinematics = self._build_kinematics(self.calibration.value)

    @subscriber(Calibration.Event.ACTIVATED)             # Mirror 와 같은 event key
    def on_calibration_change(self, event: CalibrationActivated):
        # link_offset 이 PyBullet URDF 에 박혀있어 재로드 필요 — consumer 책임
        # joint / sag / hand_eye 는 매 access fresh 라 rebuild 불필요
        self._kinematics = self._build_kinematics(self.calibration.value)

    @service(Motion.Service.MOVE_L)
    def move_l(self, req: MoveLRequest) -> MoveLResponse:
        if not self.calibration.is_ready:
            raise NotReady("calibration 아직 동기화 안 됨")
        cal = self.calibration.value           # 매 호출 fresh
        target_in_base = cal.hand_eye @ req.target_in_camera
        joints = self._kinematics.ik(target_in_base)
        # cal.joint_offsets / cal.sag_offsets 도 kinematics 내부 매 호출 fresh access
        self.runtime.publish(
            Motor.Stream.CMD_JOINT,                            # stream key (100Hz)
            MotorCmdJoint(robot_id=self.robot_id, joints=joints),
        )
        return MoveLResponse(ok=True)
```

특징:
- `self.calibration.value` 매 호출 fresh — sub-field (`cal.hand_eye`, `cal.joint_offsets`) 는 access 시점에 골라 씀.
- `CalibrationActivated` event 받으면 framework 자동 cache 갱신. 단 **PyBullet 재로드 같은 architectural side-effect 는 consumer 가 같은 event 박아 자기 처리**.
- `Mirror` mapping 한 번 박으면 lifecycle 전체 흡수.

### 4.3 비대칭 표

| 자리 | 누가 박나 |
|---|---|
| Owner 의 `repo.save()` | 개발자 (business intent) |
| Owner 의 `publish(Event)` | 개발자 (domain 의미) |
| Owner 의 `snapshot_*` service | 개발자 (service 한 줄, repo.get_active 호출만) |
| Reader 의 부팅 시 snapshot 호출 | framework |
| Reader 의 event subscribe | framework |
| Reader 의 cache management | framework |
| Reader 의 `self.calibration.value` access surface | framework |
| Reader 의 architectural side-effect (PyBullet 재로드 등) 처리 | 개발자 (consumer 가 같은 event 박아 자기 처리) |

### 4.4 robot-agnostic Reader — Detector Module

```python
# robot-agnostic — host 당 1 인스턴스. YOLO model robot 무관.
# 매 detect 호출 시 req.robot_id 로 dispatch. Mirror 박지 않음 — service call 로.
from modules.detector.contract import Detector, DetectRequest, DetectResponse
from modules.camera.contract import Camera, CameraDecodedFrame
from modules.camera.contract import SnapshotRequest as CameraSnapshotRequest
from modules.calibration.contract import Calibration, CalibrationBundle, SnapshotRequest

class DetectorModule:
    def __init__(self, runtime: ModuleRuntime):
        self.runtime = runtime
        self._yolo = YOLO(...)    # model load 1 회

    @service(Detector.Service.DETECT)
    async def detect(self, req: DetectRequest) -> DetectResponse:
        # robot 별 frame / calibration = 매 호출 service call (Mirror 안 박음)
        # key + req + res_cls + robot_id 모두 explicit
        frame = (await self.runtime.call(
            Camera.Service.DECODED_SNAPSHOT,
            CameraSnapshotRequest(),
            CameraDecodedFrame,
            robot_id=req.robot_id,
        )).to_ndarray()
        bundle = await self.runtime.call(
            Calibration.Service.SNAPSHOT_BUNDLE,
            SnapshotRequest(robot_id=req.robot_id),
            CalibrationBundle,
        )
        boxes = self._yolo(frame)
        # 카메라 → base 변환 (calibration_apply_flow §4)
        objects_in_base = self._project(boxes, bundle.hand_eye, bundle.intrinsic, req.tcp_pose)
        return DetectResponse(objects=objects_in_base)
```

특징:
- **robot-agnostic** — YOLO model robot 무관 (같은 가중치), 매 detect 호출 시 robot_id 로 dispatch.
- Mirror 박지 않음 — `detect` 호출 빈도 낮음 (5Hz / 사용자 trigger). 매 호출 service call OK.
- 고빈도 detect 필요 (예: realtime visual servo) 가 생기면 그때 Mirror 또는 robot-scoped sub-module 고려.

## 5. 폴더 구조

```
backend_v2/
│
├── framework/                    # 변하지 않는 시스템 기반
│   │
│   ├── contract/                 # Service / Event / Mirror 데코 + spec
│   │   ├── service.py            # @service(key) factory + ServiceSpec
│   │   ├── subscriber.py         # @subscriber(key) factory + SubscriberSpec
│   │   ├── publisher.py          # @publishes((key, event_cls) pairs) + encode/decode_event (msgpack)
│   │   ├── mirror.py             # Mirror[T] descriptor + MirrorSpec (5 인자)
│   │   └── envelope.py           # ServiceRequest/ServiceResponse Pydantic generic
│   │
│   ├── runtime/                  # Module lifecycle + DI 주입
│   │   ├── api.py                # ModuleRuntime Protocol — Module 의 통신 surface (§3.7)
│   │   ├── app.py                # Runtime: yaml → Module instantiate → start
│   │   ├── lifecycle.py          # Lifecycle Protocol (start / stop)
│   │   └── discovery.py          # Module instance 의 @service / @subscriber / Mirror scan
│   │
│   ├── transport/                # Transport Protocol (Zenoh 단일)
│   │   └── protocol.py           # Transport(Protocol)
│   │
│   ├── persistence/              # Repository Protocol (DB 모름)
│   │   └── protocol.py           # Repository(Protocol)
│   │
│   └── storage/                  # ObjectStore Protocol (S3/MinIO/fs 모름)
│       └── protocol.py           # ObjectStore(Protocol)
│
├── infra/                        # framework Protocol 의 실 구현 (외부 dep 가짐)
│   │
│   ├── transport/
│   │   └── zenoh.py              # ZenohTransport — Zenoh session wrap
│   │
│   ├── database/
│   │   ├── sqlite.py             # SQLAlchemy + sqlite (dev / mock)
│   │   └── postgres.py           # SQLAlchemy + psycopg (운영 NAS)
│   │
│   └── object_store/
│       ├── filesystem.py         # local fs (dev / mock)
│       └── minio.py              # boto3 (S3 compat, 운영 NAS)
│
├── modules/                      # 도메인 기능 — entity 추가 시 여기만 큼
│   │
│   ├── calibration/              # business domain (영속성 owner)
│   │   ├── contract.py           # Public Surface — Service/Event nested StrEnum + Pydantic payload
│   │   ├── models.py             # SQLAlchemy ORM class (internal)
│   │   ├── repository.py         # CalibrationRepository (internal)
│   │   ├── service.py            # business logic (BA / IRLS / observability) (internal)
│   │   └── module.py             # @publishes + @service + @subscriber entry
│   │
│   ├── scan/                     # business domain
│   │   ├── contract.py
│   │   ├── models.py
│   │   ├── repository.py
│   │   ├── artifact.py           # ObjectStore 사용 (scans blob)
│   │   └── module.py
│   │
│   ├── reconstruction/           # business domain (Reader of scan)
│   │   ├── contract.py
│   │   ├── models.py
│   │   ├── pipeline.py           # ICP + PoseGraph + TSDF
│   │   ├── artifact.py
│   │   └── module.py
│   │
│   ├── task/                     # business domain (orchestrator)
│   │   ├── contract.py
│   │   ├── models.py
│   │   ├── repository.py
│   │   ├── dsl/                  # Step / Slot / Recipe — 기존 step_dsl 옮겨심음
│   │   └── module.py
│   │
│   ├── motion/                   # robot-scoped (per-robot kinematics state)
│   │   ├── contract.py           # {robot_id} placeholder 박힌 nested StrEnum
│   │   ├── kinematics.py         # PyBullet + sag corrected
│   │   ├── trajectory.py         # Ruckig
│   │   ├── jog.py                # SE(3) 적분
│   │   └── module.py             # MotionModule(robot_id) + boot-time snapshot_bundle query
│   │
│   ├── motor/                    # robot-scoped (Dynamixel device handle)
│   │   ├── contract.py
│   │   ├── driver/
│   │   │   ├── dynamixel.py
│   │   │   └── feetech.py
│   │   └── module.py             # MotorModule(robot_id)
│   │
│   ├── camera/                   # robot-scoped (RealSense device + per-robot frame)
│   │   ├── contract.py
│   │   ├── driver/
│   │   │   ├── realsense.py
│   │   │   └── mock.py
│   │   ├── module.py             # CameraDriver(robot_id) — raw JPEG / zstd depth
│   │   ├── decoded.py            # CameraDecoded(robot_id) — JPEG → ndarray (derived)
│   │   └── depth_decoded.py      # DepthDecoded(robot_id) — zstd depth → uint16
│   │
│   ├── detector/                 # robot-agnostic (YOLO model robot 무관)
│   │   ├── contract.py
│   │   ├── yolo.py
│   │   └── module.py             # DetectorModule — 매 detect 호출에 req.robot_id
│   │
│   ├── scene3d/                  # robot-agnostic (RGBD primitive service)
│   │   ├── contract.py
│   │   └── module.py
│   │
│   └── gamepad/                  # robot-agnostic (UI input)
│       ├── contract.py
│       └── module.py
│
├── deployments/                  # 어떤 process 에 어떤 Module 띄울지
│   ├── pc.yaml                   # 예시 ↓
│   ├── pi_motor.yaml
│   ├── pi_camera.yaml
│   ├── dev.yaml                  # PC 한 process 에 다 띄움 (Zenoh same-session)
│   └── mock.yaml                 # hardware mock 으로 swap
│
├── apps/
│   └── main.py                   # 한 entry. uv run python apps/main.py --host pc
│
└── tests/
    ├── framework/                # framework 단위 test
    └── modules/                  # Module integration test (Zenoh in-process peer)
```

**module 안 파일 책임 분리** (도메인 boundary):

| 파일 | 역할 | 외부 import 가능? |
|---|---|---|
| `contract.py` | Public Surface — Service/Event/Stream nested StrEnum + Pydantic event/req/res/bundle | ✅ |
| `module.py` | framework entry — `@publishes` / `@service` / `@subscriber` / `Mirror` 박힌 class | ❌ (framework Runtime 만 instantiate) |
| `models.py` | SQLAlchemy ORM | ❌ (Repository 안에서만) |
| `repository.py` | Repository class (framework Protocol 만족) | ❌ (module.py 가 DI 받음) |
| `service.py` | business logic (BA / IRLS 등) | ❌ (module.py 가 호출) |
| `artifact.py` | ObjectStore 사용 (scan blob / mesh 등) | ❌ |
| `driver/` (motor / camera) | 하드웨어 driver | ❌ |
| `alembic/` | migration | ❌ |

**contract.py 의 진화 path** — 첫 박을 때 단일 파일, 비대해지면 (1000 줄+) `contract/` 패키지로:
```
modules/calibration/contract/
  __init__.py     # re-export (외부 import path 자체 안 바뀜)
  keys.py
  events.py
  services.py
```
외부 module 의 `from modules.calibration.contract import ...` 자체 동일 — 내부만 refactor.

## 6. 데이터 흐름

### 6.1 Calibration activate (Owner side)

```
사용자 UI
   │
   ▼
runtime.call(
    Calibration.Service.ACTIVATE,                         # service key
    ActivateRequest(robot_id="omx_f_0", result_id=10),    # req
    ActivateResponse,                                     # res_cls
)
   │  ↑ 세 인자 모두 explicit — service 가리키는 방법 = enum 한 패턴
   ▼
CalibrationModule.activate:
   repo.get(10)
   result.activate()
   repo.save(result)
   runtime.publish(
       Calibration.Event.ACTIVATED,                              ← event key 첫 인자
       CalibrationActivated(robot_id="omx_f_0", bundle_id=...),  ← event instance 두 번째
   )
   ▼
ZenohTransport:
   ├─ 같은 process subscriber → Zenoh same-session in-routing
   └─ 다른 process subscriber → Zenoh between-session (network)
```

### 6.2 Motion read calibration (Reader side)

> ⚠️ **SUPERSEDED (2026-07-02) — 아래 Mirror 흐름은 stand-in illustration.** 실제 calibration = boot-time config → Motion 의 실제 흐름은: `start()` 에서 `runtime.call(Calibration.Service.SNAPSHOT_BUNDLE, ...)` **1회** → kinematics build → 끝. subscribe / event refetch / 런타임 cache 갱신 **없음**. 아래 "런타임: Calibration 측 activate → refetch" 부분은 일어나지 않는다 (activate = "재시작 필요" 알림, [calibration_module_boundary.md §5/§9](calibration_module_boundary.md)). Mirror 흐름의 메커니즘 예시로만 유지.

```
부팅 시점:
   MotionModule.start()
        │
        ▼
   framework discovery 가 Mirror[CalibrationBundle] 발견
        │  Mirror(snapshot_service=..., snapshot_req=..., change_topic=..., value_cls=..., change_event_cls=...)
        ▼
   runtime.call(
       Mirror.snapshot_service,                    # Calibration.Service.SNAPSHOT_BUNDLE
       Mirror.snapshot_req(self),                  # SnapshotRequest(robot_id=self.robot_id)
       Mirror.value_cls,                           # CalibrationBundle
   )
        │  ↑ Mirror 가 key + req factory + res_cls 모두 explicit 박음 (lookup 없음)
        ▼
   결과 local cache 저장
        │
        ▼
   subscribe(Mirror.change_topic)  ─ Calibration.Event.ACTIVATED (Mirror config)
        ← payload.robot_id 로 filter (Mirror invariant: self.robot_id 만 박음)
        ← decode 는 change_event_cls (CalibrationActivated)


런타임:
   MotionModule.move_l(...)
        │
        ▼
   self.calibration.value  ← fresh cache read (network 0)

   ─────

   Calibration 측 activate 발생
        │
        ▼
   runtime.publish(Calibration.Event.ACTIVATED, CalibrationActivated(...))
        │
        ▼
   Reader subscriber callback → cache refetch (snapshot_bundle 재호출, §3.3.5)
```

### 6.3 Scan capture → Reconstruction (cross-module 영속성)

```
TaskModule.scan_task 실행:
   │
   ├─ for each pose:
   │     MotionModule.move_j(...)
   │     ScanModule.capture()   ─ camera frame + zstd depth + ObjectStore put
   │
   └─ ReconstructionModule.build(session_id)
         │
         ├─ ScanModule.list_scans(session_id)  ─ scan metadata
         ├─ ScanModule.get_blob(scan_id)        ─ ObjectStore get
         ├─ ICP + PoseGraph + TSDF
         └─ ObjectStore put (mesh.ply)
              + publish(ReconstructionBuilt(...))
```

각 Module 이 자기 DB + ObjectStore 영역 owner. cross-module call 은 standard `@service`.

### 6.4 Camera frame — decode dedup 흐름

```
Pi process:
  CameraDriver Module (robot-scoped, self.robot_id="omx_f_0")
       ├─ RealSense capture (BGR ndarray + uint16 depth)
       ├─ cv2.imencode JPEG / zstd compress depth
       └─ runtime.publish(
              Camera.Stream.JPEG,                                      ← stream key 첫 인자
              CameraJpegFrame(robot_id=self.robot_id, jpeg_bytes=...), ← event instance
          )
            │
            ▼ Zenoh (Pi → PC, network)
            │
PC process — 한 process / 한 Zenoh session:
  CameraDecoded Module
       @subscriber(Camera.Stream.JPEG)
       on_jpeg(self, event: CameraJpegFrame):       ← type hint 으로 decode
           ndarray = cv2.imdecode(event.jpeg_bytes, ...)         ← decode 1회 (4.34ms × 30Hz)
           self.runtime.publish(
               Camera.Stream.DECODED,
               CameraDecodedFrame(robot_id=event.robot_id, ndarray_bytes=...),
           )
            │
            ▼ Zenoh same-session (PC 안)
            │
       ┌────┴──────┬───────────┬─────────────────┐
       ▼           ▼           ▼                 ▼
   Detector   Calibration   Scene3D     Bridge (raw JPEG forward)
                                          ← Bridge 는 @subscriber(Camera.Stream.JPEG)
                                            (decode 안 함, jpeg_bytes 그대로 WS)
```

Bridge 는 WebSocket 에 raw JPEG bytes 그대로 forward — decode 0. `Camera.Stream.JPEG` 직접 subscribe (CameraDecoded 안 거침).

decoded ndarray 가 필요한 consumer (Detector, Calibration, Scene3D) 는 `Camera.Stream.DECODED` subscribe.

## 7. Module 구조

### 7.1 Module = plain class

base class 강제 X, `@module` 데코 X. framework 가 `@service` / `@subscriber` / `Mirror` 박힌 메소드/속성만 inspect.

```python
# robot-agnostic
from .contract import Calibration, ActivateRequest, ActivateResponse, SnapshotRequest, CalibrationBundle

class CalibrationModule:
    # 생성자 — Runtime 이 DI injection (ModuleRuntime + Repository + ObjectStore 등)
    def __init__(self, runtime: ModuleRuntime, repo: CalibrationRepository):
        self.runtime = runtime
        self._repo = repo

    # lifecycle — Lifecycle Protocol (선택, 안 박아도 됨)
    def start(self) -> None: ...
    def stop(self) -> None: ...

    # contract — framework 가 발견. @service 의 인자 = nested StrEnum value.
    @service(Calibration.Service.ACTIVATE)
    def activate(self, req: ActivateRequest) -> ActivateResponse: ...

    @service(Calibration.Service.SNAPSHOT_BUNDLE)
    def snapshot_bundle(self, req: SnapshotRequest) -> CalibrationBundle: ...


# robot-scoped — yaml `robots: [...]` 박힘. constructor 의 robot_id 가 계약 검증.
from .contract import Motion

class MotionModule:
    def __init__(self, runtime: ModuleRuntime, robot_id: str):
        self.runtime = runtime
        self.robot_id = robot_id

    @service(Motion.Service.MOVE_L)                      # = "srv/motion/{robot_id}/move_l"
    def move_l(self, req): ...                           # Module register 시 {robot_id} 자동 substitute
```

scope 결정 = §2.7 참조 (yaml primary, constructor 계약 검증).

### 7.2 Module 안 책임 분리

(폴더 구조 §5 의 표와 동일 — 한 번 더 정리):

| 파일 | 책임 | 외부 import? |
|---|---|---|
| `contract.py` | Public Surface — nested StrEnum (Service / Event / Stream) + Pydantic (event / req / res / bundle) | ✅ |
| `module.py` | framework entry — `@publishes` / `@service` / `@subscriber` / `Mirror` 박힌 class | ❌ |
| `models.py` | SQLAlchemy ORM (Aggregate root + child relationship) | ❌ |
| `repository.py` | Repository (framework Repository Protocol 만족) | ❌ |
| `service.py` | business logic (BA / IRLS / orchestration — module.py 가 호출) | ❌ |
| `artifact.py` | ObjectStore 사용 (scan blob / mesh 등) | ❌ |

DDD 폴더 모양 (`domain/entities.py`, `domain/value_objects.py`) 박지 않음. Aggregate boundary 의 사고만 가져옴 — 클래스 관계 (SQLAlchemy `relationship` + cascade) 로 표현.

### 7.3 Aggregate root 예 — CalibrationRun

```python
class CalibrationRun(Base):
    __tablename__ = "calibration_runs"
    id: Mapped[int] = mapped_column(primary_key=True)
    status: Mapped[str]
    started_at: Mapped[datetime]

    captures: Mapped[list["Capture"]] = relationship(cascade="all, delete-orphan")
    results: Mapped[list["CalibrationResult"]] = relationship(cascade="all, delete-orphan")

    def finalize(self, ba_output) -> None:
        self.status = "ready_for_analysis"
        self.results.append(CalibrationResult.from_ba(ba_output))
```

Aggregate boundary = transaction boundary. `finalize()` 호출 = run row update + result INSERT 가 한 transaction.

### 7.4 Derived read model Module — 코드 형태

큰 payload decode 비용이 N consumer 마다 누적되는 자리에 박는 패턴 (§3.5).

```python
# modules/camera/contract.py — Public Surface
from enum import StrEnum
from pydantic import BaseModel
import numpy as np

class Camera:
    class Service(StrEnum):
        DECODED_SNAPSHOT = "srv/camera/{robot_id}/decoded_snapshot"

    class Stream(StrEnum):
        JPEG     = "stream/camera/{robot_id}/jpeg"
        DECODED  = "stream/camera/{robot_id}/decoded"

class CameraJpegFrame(BaseModel):
    robot_id: str
    timestamp: float
    jpeg_bytes: bytes

class CameraDecodedFrame(BaseModel):
    robot_id: str
    timestamp: float
    width: int
    height: int
    ndarray_bytes: bytes        # 압축 안 된 BGR raw

    def to_ndarray(self) -> np.ndarray:
        return np.frombuffer(self.ndarray_bytes, dtype=np.uint8).reshape(
            self.height, self.width, 3
        )

class SnapshotRequest(BaseModel):
    pass  # robot-scoped service — robot_id 는 caller 가 인자로 명시


# modules/camera/decoded.py — derived read model Module
from .contract import Camera, CameraJpegFrame, CameraDecodedFrame, SnapshotRequest

@publishes(
    (Camera.Stream.DECODED, CameraDecodedFrame),
)
class CameraDecoded:
    def __init__(self, runtime: ModuleRuntime, robot_id: str):
        self.runtime = runtime
        self.robot_id = robot_id
        self._latest: CameraDecodedFrame | None = None

    @subscriber(Camera.Stream.JPEG)
    def on_jpeg(self, event: CameraJpegFrame) -> None:
        arr = cv2.imdecode(np.frombuffer(event.jpeg_bytes, np.uint8), cv2.IMREAD_COLOR)
        if arr is None:
            return
        frame = CameraDecodedFrame(
            robot_id=event.robot_id,
            timestamp=event.timestamp,
            width=arr.shape[1],
            height=arr.shape[0],
            ndarray_bytes=arr.tobytes(),
        )
        self._latest = frame
        self.runtime.publish(Camera.Stream.DECODED, frame)

    @service(Camera.Service.DECODED_SNAPSHOT)
    def snapshot(self, req: SnapshotRequest) -> CameraDecodedFrame:
        if self._latest is None:
            raise NotReady("아직 첫 jpeg 안 옴")
        return self._latest
```

특징:
- Decode 1 회, 결과 publish 로 fanout.
- `@service(...) snapshot` 박아두면 consumer 가 `Mirror[CameraDecodedFrame]` 으로 받음 — late-join + reactive.
- framework primitive 아님 — 그저 일반 Module + `@subscriber` + `publish` + `@service`. 개발자 책임.

consumer 측 (robot-scoped Reader 예 — robot-agnostic Detector 는 §4.4 처럼 매 호출 service call 이 더 자연):
```python
from modules.camera.contract import Camera, CameraDecodedFrame, SnapshotRequest
from modules.detector.contract import Detector

class DetectorModule:
    camera: Mirror[CameraDecodedFrame] = Mirror(
        snapshot_service=Camera.Service.DECODED_SNAPSHOT,
        snapshot_req=lambda self: SnapshotRequest(),
        change_topic=Camera.Stream.DECODED,
        value_cls=CameraDecodedFrame,                              # cache T
        change_event_cls=CameraDecodedFrame,                       # event class (decode)
    )

    @service(Detector.Service.DETECT)
    def detect(self, req):
        frame = self.camera.value                            # fresh CameraDecodedFrame
        arr = frame.to_ndarray()                             # 이미 decoded
        return self._yolo(arr)
```

## 8. DIP — Framework Protocol vs Infra impl

### 8.1 Repository Protocol

```python
# framework/persistence/protocol.py
class Repository(Protocol[T]):
    def get(self, id: int) -> T | None: ...
    def save(self, entity: T) -> None: ...
    def delete(self, id: int) -> None: ...
```

Module 의 Repository 가 이 Protocol 만족 + entity-specific method 추가:

```python
# modules/calibration/repository.py
class CalibrationRepository:
    def __init__(self, session_factory: Callable[[], Session]):
        self._session_factory = session_factory

    def get(self, result_id: int) -> CalibrationResult: ...
    def save(self, result: CalibrationResult) -> None: ...
    def get_active(self) -> CalibrationResult | None: ...    # entity-specific
    def list_by_kind(self, kind: str) -> list[CalibrationResult]: ...
```

framework 가 Repository class 자체 만들지 않음. Module 이 자기 ORM 알고 짜는 게 정직. framework Protocol 은 *type bound* 만.

### 8.2 ObjectStore Protocol

```python
# framework/storage/protocol.py
class ObjectStore(Protocol):
    def put(self, key: str, data: bytes) -> None: ...
    def get(self, key: str) -> bytes: ...
    def delete(self, key: str) -> None: ...
    def list(self, prefix: str) -> list[str]: ...
```

infra impl:
- `infra/object_store/filesystem.py` — local dev / mock
- `infra/object_store/minio.py` — production (boto3)

### 8.3 Transport Protocol

```python
# framework/transport/protocol.py
class Transport(Protocol):
    async def call(self, key: str, payload: bytes, timeout: float) -> bytes: ...
    def publish(self, key: str, payload: bytes) -> None: ...
    def register_service(self, key: str, handler: Callable[[bytes], bytes]) -> Handle: ...
    def subscribe(self, key: str, callback: Callable[[bytes], None]) -> Handle: ...
```

impl:
- `infra/transport/zenoh.py` — `ZenohTransport`. Zenoh session wrap. 유일.

LocalTransport 박지 않음 (§3.4 / §10.8 — 측정 결과 기반 결정). test 는 Zenoh in-process peer 사용.

### 8.4 DI 주입

`apps/main.py` 가 deployment yaml 파싱 + infra 인스턴스 생성 + Runtime 에 주입:

```python
# apps/main.py
def main(host: str):
    cfg = load_yaml(f"deployments/{host}.yaml")

    transport = make_transport(cfg.transport)        # Zenoh (single impl)
    session_factory = make_session(cfg.database)     # sqlite / postgres
    object_store = make_object_store(cfg.storage)    # fs / minio

    runtime = Runtime(transport=transport)           # Runtime 이 ModuleRuntime adapter 박음
    for mod_cfg in cfg.modules:
        mod_cls = MODULE_REGISTRY[mod_cfg.name]
        if mod_cfg.robots:
            # robot-scoped — per-robot 인스턴스
            for rid in mod_cfg.robots:
                runtime.add_module(
                    mod_cls,
                    robot_id=rid,
                    session_factory=session_factory,
                    object_store=object_store,
                )
        else:
            runtime.add_module(
                mod_cls,
                session_factory=session_factory,
                object_store=object_store,
            )

    runtime.start()
```

Runtime 내부 `add_module` 은 `inspect.signature(cls.__init__)` 로 constructor parameter list 추출 후 매칭 inject (`runtime: ModuleRuntime` / `robot_id: str` / `repo: CalibrationRepository` / `object_store: ObjectStore` 등). Module 은 자기 dep 를 constructor 로 받음. **FastAPI Depends 식 lazy DI container 박지 않음** — manual constructor injection 으로 충분.

## 9. Storage Module 폐기

기존 [storage_layer.md](storage_layer.md) 의 Storage Module 은 본 spec 에서 사라짐. 그 3 motivation 이 다음으로 흡수:

### 9.1 Centralization (분산 동기화)

기존: 모든 entity 가 한 Storage Module 의 service 통해 영속화. Cross-module read 도 Storage Module 거침.

새 spec: 각 도메인 Module 이 자기 영속성 owner. Cross-module read = `Mirror[T]` primitive. Storage Module 가운데 끼는 wire 사라짐.

### 9.2 Migration owner — 루트 단일 Alembic (2026-07-02 정정)

> **초안의 "Module N 개 = Alembic N 개" 폐기.** 소유권 ≠ 마이그레이션 권위 ([calibration_module_boundary.md §8](calibration_module_boundary.md)):
> - **테이블/ORM/Repository 소유 = 모듈별** (Storage *Module* RPC 중개자 폐기는 그대로).
> - **마이그레이션 = 루트 하나** (`backend_v2/alembic/`, 공유 `infra/database/base.py::Base`). 같은 프로세스 + 공유 DB = Database-per-**Service** 아님. per-module Alembic 은 version_table 충돌 / cross-module FK 순서 / 전체 초기화 복잡도만 들여옴.

기존: Storage Module 부팅 시 Alembic `upgrade head` 한 번.

새 spec: 루트 `backend_v2/alembic/env.py` 가 모든 DB 모듈 ORM 을 import → 공유 `Base.metadata` 단일 history. runtime `upgrade head` 는 apps boot(또는 DB owner 모듈)가 프로그래매틱 실행. Pi 는 alembic 실행/import 안 함 (PC 전용 도구 — role 격리 유지). **구현·검증됨** (`tests/modules/test_alembic.py`).

### 9.3 DB dependency 격리

기존: Pi 가 Storage Module service 호출, SQLAlchemy import 0.

새 spec: Pi 의 Module 들 (motor / motion / camera) 은 *Reader 만*, 자기 DB 안 가짐. PC 의 Calibration Module 이 owner, Pi 의 Motion 은 boot 시 `snapshot_bundle` query (PC 의 Calibration service 호출) 로 받음. Pi 에 SQLAlchemy / Postgres driver import 0 유지 — DB 접근은 owner(PC) 만, Reader 는 wire 로 받으니 dependency 격리는 boot-query 든 Mirror 든 동일하게 성립.

→ Storage Module 사라지고도 3 motivation 다 만족.

## 10. 하지 않는 것

### 10.1 React-style reactive state framework

`@state` 데코 / mutation tracking / partial state diff / reactive dependency graph 박지 않음. Owner 의 `repo.save()` + `publish(Event)` 가 명시적. DB update ≠ domain event — 의미는 Owner 만 결정.

### 10.2 DI container (FastAPI Depends 식)

call-time lazy resolution 안 박음. HTTP request lifecycle 에 묶인 패턴이라 우리 process-scoped service 에는 정당화 약함. Manual constructor injection + lazy singleton (Repository / ObjectStore 등) 으로 충분.

### 10.3 DDD tactical 폴더 (entities / value_objects / domain layer)

DDD 의 *사고* (Aggregate boundary / 소유 / 변경 동시성) 만 가져옴. 폴더 모양 (`domain/entities.py`, `domain/value_objects.py`) 박지 않음. Aggregate 는 SQLAlchemy `relationship` + cascade 로 자연 표현.

### 10.4 Generic Repository ORM framework

framework 가 SQLAlchemy class 자동 generate / migration auto-apply / query builder 박지 않음. 그저 Repository Protocol 만 정의. Module 이 자기 ORM 직접 짬.

### 10.5 "기술 갈아끼우기 자유도" 명분

"미래 Zenoh → ROS2", "미래 Postgres → MongoDB" 같은 자유도 motivation 으로 Protocol 박지 않음. 진짜 motivation = test mock + import boundary 두 개만.

### 10.6 `@module` 데코 / 클래스 hierarchy 강제

Module = plain Python class. `@module(...)` 데코 박지 않음 (deployment 결정은 yaml 의 책임, 코드 안 host 박지 X). Lifecycle 도 Protocol — base class 상속 강제 X.

### 10.7 한 entry point 여러 개

`apps/robot_runtime.py` + `backend_runtime.py` 식 분리 박지 않음. `apps/main.py` 한 entry + `--host` 인자 + deployment yaml. 기존 backend `main.py` 의 host 자동 감지 + yaml 로딩 패턴 그대로.

### 10.8 LocalTransport / process-local fast-path

같은 process 안 Module 간 호출이 Zenoh 안 거치고 `dict[key] → callback` direct dispatch 박는 자리 = **박지 않음** (§3.4). 측정 ([backend/scripts/bench_transport.py](../backend/scripts/bench_transport.py)) 결과:

- 작은 message: Zenoh same-session ~3us — 가치 0.
- 큰 ndarray (5MB) fanout: ~4% × N CPU 절감 — 단 framework 두 갈래 유지 비용 (Transport 두 impl + resolver + behavior 일관성 risk) 보다 작음.

decode dedup 패턴 (§3.5) 으로 카메라 자리 39% → 21% CPU. LocalTransport 추가는 21% → 13% (8% 더), 단 측정 후 진짜 bottleneck 으로 드러나면 그때 추가. 지금부터 박지 않음.

### 10.9 Runtime resolver / provider locality 결정

§10.8 의 LocalTransport 박지 않음 결정의 자연 귀결. transport 한 갈래 (Zenoh) 라 *어디로 보낼지* 선택할 자리 자체 없음. Runtime 의 책임은 lifecycle + DI + Zenoh queryable/subscriber wire 만.

## 11. 달성 단계

순차. 각 step 끝 = 검증 가능한 산출물.

### Step 1 — Transport abstraction (Zenoh 단일)

`framework/transport/protocol.py` + `infra/transport/zenoh.py`.

검증:
- `ZenohTransport.publish(key, b"...") → 같은 session 안 subscriber callback 발동` (same-process, in-session routing).
- `ZenohTransport.publish(...) → 다른 process subscriber callback 발동` (cross-process, host_mock subprocess).

같은 process 안 routing 도 Zenoh same-session 통과 — 측정 결과 ([backend/scripts/bench_transport.py](../backend/scripts/bench_transport.py)) 작은 message ~3us, 5MB ~1.27ms.

**✅ 완료** — [backend_v2/framework/transport/protocol.py](../backend_v2/framework/transport/protocol.py) + [backend_v2/infra/transport/zenoh.py](../backend_v2/infra/transport/zenoh.py). **7 test PASS** — same-session pub/sub + service call + handler exception → `RemoteError` + timeout → `TimeoutError` + callback exception swallow + cross-process pub/sub (subprocess).

### Step 2 — Contract layer

`framework/contract/{service,subscriber,publisher,envelope}.py`. Pydantic generic envelope + `@service` / `@subscriber` / `@publishes` 데코 + spec 수집.

검증:
- `@service` 박은 메소드를 framework 가 inspect 해서 ServiceSpec 추출.
- ZenohTransport 위에 service register + same-session call round-trip.

**✅ 완료** — [backend_v2/framework/contract/](../backend_v2/framework/contract/). **19 test PASS** — service / subscriber spec 추출 + invalid type hint fail-fast + `@publishes(*pairs)` class 데코 + envelope round-trip + E2E ZenohTransport wire + handler exception E2E + event publish/subscribe E2E.

### Step 3 — Runtime + Module discovery

`framework/runtime/{api,app,lifecycle,discovery}.py`. Module 인스턴스 → spec 수집 → transport 바인딩 → lifecycle.

산출물:
- `api.py` — `ModuleRuntime` Protocol (§3.7). `publish(key, event)` + `call(key, req, res_cls, *, robot_id=, timeout=)`.
- `app.py` — `Runtime` (add_module + start + stop) + `_TransportRuntime` adapter. `{robot_id}` placeholder substitute 4 경로 (register / publish / call / subscribe wildcard).
- `lifecycle.py` — `Lifecycle` Protocol (`start` / `stop`, 선택). sync / async 둘 다.
- `discovery.py` — `discover_services` / `discover_subscribers` helper.

부팅 순서 = **instantiate → register → start** (§3.6).

**✅ 완료** — [backend_v2/framework/runtime/](../backend_v2/framework/runtime/). **12 test PASS**:
- 빈 Module runtime start → stop
- 두 Module + service call round-trip (`runtime.call(Module.Service.X, req, ResCls)`)
- publish → `@subscriber` callback 도달
- Module A start() 가 Module B service 호출 (phase 2 register → phase 3 start 순서 검증)
- robot-scoped service register / call / event publish substitute
- robot_id 누락 fail-fast
- add_module missing dep fail-fast
- sync / async start/stop

### Step 4 — Persistence + Storage Protocol + Infra

`framework/persistence/protocol.py` + `framework/storage/protocol.py` + `infra/database/{sqlite,postgres}.py` + `infra/object_store/{filesystem,minio}.py`.

검증:
- SQLite session 생성 → ORM class INSERT/SELECT round-trip.
- FilesystemObjectStore put/get round-trip.

**✅ 완료** — Repository `Protocol[T]` (sync `get/save/delete`) + ObjectStore `Protocol` (runtime_checkable) + `open_sqlite() / open_postgres() -> (Engine, sessionmaker)` + FilesystemObjectStore (atomic `.tmp + os.replace`, path escape 차단) + MinioObjectStore (boto3, optional dep). **17 test PASS** (4 persistence + 13 storage).

### Step 5 — `Mirror[T]` primitive

`framework/contract/mirror.py`. 5 인자 모두 explicit (snapshot_service, snapshot_req factory, change_topic, value_cls, change_event_cls).

검증:
- Owner Module 이 snapshot service + event publish.
- Reader Module 이 `Mirror[T]` 선언 → 부팅 시 cache fill + event 받으면 cache update.
- Same-process (Zenoh same-session) + cross-process (Zenoh between sessions) 두 case PASS.

**✅ 완료** — Mirror descriptor + MirrorState (per-instance state via `__set_name__` + `__get__`) + NotReady + discover_mirrors + Runtime 통합 (`_register_mirror_subscriber` phase 2 + `_initialize_mirrors` phase 3a + `_refetch_mirror`). **10 test PASS** (same-process snapshot / Owner activate event refetch / Owner-not-up non-blocking / robot-scoped snapshot + event filter / cross-process subprocess + NotReady + per-instance state).

### Step 6 — 첫 Module 박아서 검증 (Calibration)

`modules/calibration/`. ORM + Repository + Module + Alembic.

검증:
- `CalibrationModule.activate(result_id)` round-trip (Zenoh same-session).
- 두 result row, activate 시 한쪽만 is_active=True 자연.
- `CalibrationActivated` event publish 확인.

### Step 7 — Reader 박아서 검증 (Motion)

> ⚠️ **2026-07-02 정정**: 실제 Motion Reader 는 Mirror 아니라 **boot-time `snapshot_bundle` query** (calibration = boot-time config, anchor #2). 아래 "Mirror + activate → 갱신" 검증은 stand-in. 실제 검증 = 부팅 시 1회 조회 → kinematics build (§4 banner + [calibration_module_boundary.md §9](calibration_module_boundary.md)). Mirror primitive 자체 (Step 5) 는 별도로 이미 테스트됨 — domain consumer 만 없음.

`modules/motion/`. boot-time `snapshot_bundle` query + kinematics + IK.

검증:
- 부팅 시 MotionModule 이 `snapshot_bundle` 1회 조회 → kinematics build.
- offline commit → 재시작 → 새 bundle 로 fresh build (런타임 갱신 없음).
- Same-process (PC 한 process) + cross-process (PC + 모터 Pi sim) 두 case PASS.

### Step 7.5 — Derived read model 검증 (CameraDriver + CameraDecoded)

`modules/camera/module.py` (CameraDriver) + `modules/camera/decoded.py` (CameraDecoded).

검증:
- CameraDriver mock impl 이 JPEG bytes publish (실 hardware 없이 합성 frame).
- CameraDecoded 가 `/camera/jpeg` subscribe + `cv2.imdecode` + `CameraFrame` publish.
- Consumer Module (테스트용 dummy) 가 `@subscriber(Camera.Stream.DECODED)` **stream 구독**으로 받음 (Mirror 아님 — derived read model = telemetry stream, §3.5 / §3.2).
- Consumer N=3 일 때 decode 가 1 회만 일어남 (각 consumer 별 decode X).
- decode dedup 의 CPU 절감 측정 (consumer 가 직접 decode 박는 case 와 비교).

### Step 8 — 2-3 entity 추가 (Scan / Reconstruction)

`modules/scan/` (append-only blob + metadata) + `modules/reconstruction/` (Reader of scan + ObjectStore put).

검증:
- ScanModule capture 시 ObjectStore blob put + metadata INSERT.
- ReconstructionModule build 시 scan blob get + mesh ObjectStore put.

### Step 9 — backend/ 의 도메인 logic 옮겨심음

각 Module 의 business logic (BA / IRLS / Ruckig / IK / TSDF / step DSL) 을 `modules/<name>/service.py` 또는 그 안 sub-module 로 옮겨심음. framework 부분은 새로 짠 framework 사용.

옮겨심을 자산 (framework_dogfood_plan §14.7):
- 캘 BA / IRLS / Huber / observability / strategy / ChArUco / capture_quality
- Motion command / TrajectoryRunner / Ruckig / Jog 적분 / IK
- Task DSL / Step / Slot / TaskRunner / Recipe / pick_and_place / scan task
- Detector / YOLO / Grounding DINO / search_and_detect
- Scene3D / depth_frame / consensus / pointcloud stream
- Reconstruction / ICP / PoseGraph / TSDF / mesh extract
- Kinematics (PyBullet + SagCorrected + link_offset patch)
- Coordinates (Joint / Link / Sag)
- Gamepad / 8BitDo mapper
- Robot Registry (robots.yaml + RobotConfig + factory)

### Step 10 — backend/ discard

backend_v2 가 backend 의 모든 기능 가지면 backend/ 폐기. 새 코드 = backend_v2/.

## 12. 알려진 risk

### 12.1 `Mirror[T]` 가 진짜 얇은지 검증

snapshot + subscribe + cache 패턴이 우리 use case 전체에 fit 한지는 entity 3-4 박아본 후 검증. 의심 자리:

- **partial update vs full refetch** — 큰 entity (예: scan_sessions 100 row) 의 한 row update 시 event 가 어떻게? `event = {row_id, delta}` 박고 cache merge? 또는 `event 받으면 snapshot 다시 fetch`? 첫 박을 때는 *full refetch* 가 단순. 부족하면 partial 추가.
- **concurrent write** — 두 process 가 동시에 같은 entity 변경하면? 우리 use case 에 진짜 있는지부터 (각 Module 이 owner = single writer 자연).
- **event ordering** — Reader 가 부팅 snapshot 한 후 event 받기 전 window 에 다른 process 가 변경 → 놓침. snapshot 시점에 subscribe 먼저 + buffer 패턴 박아야 (subscribe-before-snapshot).

### 12.2 N Module × N Alembic 운영 복잡도

Module 8-10 개 = Alembic 8-10 개. 부팅 시 각 Module 자기 schema ensure. risk:
- **부팅 시 lock contention** — 같은 NAS Postgres 면 8 Module 이 동시 `upgrade head` → Alembic version table lock 경쟁. 첫 부팅만 issue, 이후엔 noop. 부팅 순서 hint 또는 retry 박으면 OK.
- **Schema 충돌** — 각 Module 이 자기 table prefix (`calibration_*`, `scan_*`) 만 만들면 0. naming convention 준수.

### 12.3 한 사람 framework capacity

framework 짜는 자체 무거움. Protocol + Runtime + Contract + Transport + Mirror 5 layer. mitigation:
- **MVP 부터 시작** — Step 1-5 끝낼 때까지 Module 0 개. framework 검증.
- **`Mirror[T]` 가 가장 위험** — snapshot + subscribe + cache lifecycle 박는 자리. 첫 박을 때 simplest version (full refetch on event) 으로.
- **infra adapter 는 wrapping 만** — Zenoh / SQLAlchemy / Alembic / boto3 기능 자체는 활용, framework 가 wrap 만.
- **Transport 한 갈래** (Zenoh 만, §3.4) — LocalTransport 박지 않아서 resolver / behavior 일관성 risk / 두 path 유지 부담 0. capacity 절약.

### 12.4 backend/ 와 backend_v2/ 병행 risk

framework_dogfood_plan §14.3 규칙 그대로:
- backend/ 의 framework 부분 (BaseNode / 노드 hierarchy) 추가 변경 X.
- backend_v2 자체 *기능 개발 금지*, framework 검증만.
- 실 hardware 1 robot (omx_f_0) 만 붙여보기.
- backend/ 의 코드 reference OK (BA / Ruckig / IRLS / step DSL 등 자산), 재구성.

## 13. 인접 문서

- [framework_dogfood_plan.md](framework_dogfood_plan.md) — 결정 history + plan + §13 결정 history (20 항목) + §14 backend_v2 reframe + §15 Runtime-centric reframe. 본 문서는 §15 위 정리.
- [architecture_review_protocol.md](architecture_review_protocol.md) — 검토 phase protocol. 본 문서는 그 산출물의 한 단계.
- [storage_layer.md](storage_layer.md) — 기존 Storage Module 설계. 본 문서에서 폐기 결정. 단 ORM / Repository 자산 (SQLAlchemy 패턴 / Alembic 운영) 재활용.
- [motion_taxonomy.md](motion_taxonomy.md) — Move / Servo / Jog / Task 4 계층. modules/motion/ 안 그대로 옮겨심음.
- [step_dsl.md](step_dsl.md) — Step / Slot / Recipe DSL. modules/task/dsl/ 안 그대로.
- [multi_robot_architecture.md](multi_robot_architecture.md) — multi-robot platform 설계. 본 framework 위 robot dispatch 패턴 자연 흡수 (Module 안 `robot_id` 인자).
- [backend/scripts/bench_transport.py](../backend/scripts/bench_transport.py) — Transport latency 측정 script. §3.4 (LocalTransport 박지 않음) + §3.5 (derived read model decode dedup) 결정의 evidence. spec 변경 시 재실행.

## 14. 핵심 결정 anchor

새 세션 진입 시 본 표를 진실 source 로. 결정에 의심 들면 spec 위치 다시 읽기.

| # | 결정 | spec | 핵심 근거 |
|---|---|---|---|
| 1 | Zenoh 단일 (LocalTransport X) | §3.4 + §10.8 | [bench_transport.py](../backend/scripts/bench_transport.py) 측정: 작은 message ~3us, 5MB ndarray ~4% × N CPU 절감 — framework 두 갈래 유지 비용 < 절감. decode dedup (§3.5) 으로 39% → 21% 흡수 |
| 2 | ~~boot-query 1회~~ **→ RE-SUPERSEDED (2026-07-07)**: CalibrationBundle = **Mirror[CalibrationBundle]** (Motion 이 첫 consumer). 옛 boot-query (2026-07-02) 는 분산 부팅 순서 종속성 + silent 무보정 운전을 만들어 폐기 — liveliness 기반 Mirror 로 owner 가 언제 뜨든 수렴. "변경은 재부팅" 은 유지 (없음→값만 live 적용, 값→값′ 은 stale 표시). "atomic Bundle 단위" 유지 | §3.3 배너 + anchor #23 |
| 3 | Exception propagation (envelope `{success, message, data}` X) | §3.1 | Python 자연 = exception. caller 매 호출 `if not res.success` boilerplate 회피. transport 가 `RemoteError(type, message)` raise |
| 4 | Database-per-Module (Storage Module 폐기) — **테이블/ORM/Repository 소유만 모듈별**. **마이그레이션은 루트 단일 Alembic** (2026-07-02 정정, §9.2): 같은 프로세스+공유 DB = Database-per-Service 아님 | §2.4 + §9 | centralization motivation 은 Mirror 흡수 (지금 boot-query). DB dep 격리 = Pi 가 DB 모듈/alembic 안 가짐 |
| 5 | Module = plain class (`@module` 데코 / base class 강제 X) | §3 + §7.1 | backend/ BaseNode 부풀음 (15+ method 누적) 경로 차단. framework 는 `@service` / `@subscriber` / `Mirror` 박힌 attribute 만 inspect |
| 6 | ModuleRuntime Protocol + constructor 주입 (base class / setattr / ctx X) | §3.7 + §4 | base class 부풀음 + ctx 의 한 문장 정의 fail. composition 한 Protocol 이 sweet spot |
| 7 | Wire key = explicit + typed at every use site | §3.0 / §3.1 / §3.2 / §3.3 / §3.7 | 세 원칙: ① 사람이 explicit 지정 모든 use site ② raw string X (typed StrEnum) ③ **service 가리키는 방법 = 항상 `Service.X` enum 하나** (method reference X) |
| 8 | `runtime.call(key, req, res_cls, *, robot_id, timeout) -> TRes` (method ref X) | §3.7 + §4 | 원칙 ③ 정합. publish/subscribe 와 동일 패턴. res_cls 명시로 return type narrow |
| 9 | `Mirror(snapshot_service, snapshot_req, change_topic, value_cls, change_event_cls)` (method ref X) — **활성 (2026-07-07, 첫 consumer = Motion)** + `@mirror.on_change` 반응 훅 | §3.3 + §4.2 | 5 인자 모두 key + Pydantic class. `snapshot_req` = factory `Callable[[self], Req]`. cross-module method import 사라짐. on_change 는 값이 실제로 바뀐 전이만 (동일값 refetch 무발화) |
| 10 | Wire encoding = Pydantic + msgpack layered (DIP) | §3.4 | Module 은 Pydantic schema 만 알고, Transport boundary 가 msgpack 처리. `bytes` field native pass-through (JPEG base64 33% overhead 회피) |
| 11 | Topic prefix = `srv/` / `event/` / `stream/` (`horibot/` X) | §3.0 | 세 종류 첫 chunk 분리 — srv=RPC / event=state notification / stream=고빈도 raw. broker 단일 project 라 namespace prefix 불필요 |
| 12 | `stream` ≠ `event` (nested class naming) | §3.5 / §7.4 | `Module.Stream` 으로 명명 (`Camera.Stream`). `Module.Event` 는 진짜 state notification 만 |
| 13 | Wildcard subscribe = transport detail, framework 어휘 X | §3.7 | `@subscriber("*")` 같은 implicit pattern 금지. robot-scoped event 는 framework 가 `{robot_id}` → transport wildcard substitute, 사용자 코드에 등장 X |
| 14 | robot scope = yaml primary (constructor 가 계약 검증) | §2.7 | yaml `robots: [...]` 박힘 = robot-scoped Module. constructor 에 robot_id parameter 있어야 |
| 15 | Derived read model 패턴 (decode dedup) | §3.5 | 큰 payload (JPEG/depth) 의 decode 비용 = N consumer × decode. framework primitive 가 아닌 Module 패턴 — CameraDecoded 가 1회 decode 후 fanout |
| 16 | Runtime 부팅 순서 = instantiate → register → start | §3.6 | Mirror snapshot / Module A start 이 다른 Module service 호출 — Phase 2 (register) 가 Phase 3 (start) 이전 완료 보장 |
| 17 | Mirror invariant — partial state 노출 X | §3.3.2 | event callback thread vs handler thread race 차단. 구현 자유 (lock / atomic / RCU) |
| 18 | Mirror 동기화 = invalidate+refetch only (push update X) | §3.3.5 | event = 변경 알림 / snapshot = 진실. Bundle atomic 보존 + 다른 Module event 가 trigger 인 use case 자연 표현. push update 필요하면 Mirror 안 박고 `@subscriber` 직접 |
| 19 | **robot_id 는 두 개** — 키의 `{robot_id}` = 주소 (transport, framework/Bridge 키 확장) / body 의 robot_id = **req 필드** (service API, 호출자가 넣고 타입 강제). **Bridge 자동주입 / 생성 scope 메타데이터 / stub 전부 폐기** — global 은 "req 에 필드 없음" 으로 구조적 해결 | §2.7.1 / §2.7.3 | 2026-07-03. agnostic vs global 을 런타임에 구분하려는 순간 메타데이터나 "지금 없으니까" 타협이 필요해짐 — 레이어 재구성으로 문제 자체 소거 |
| 20 | **req robot_id 파생 규칙** — 다른 식별자 (run/session/result/waypoint row id) 로 robot 특정 가능하면 req 에 robot_id 안 둠 (DB row 에서 파생) | §2.7.1 | "run A 에 robot B 캡처" 류 불일치 채널 원천 차단 |
| 21 | **`robot_scoped` 판정 = service 키만** — stream/event 는 payload 라우팅/wildcard 라 host-level 도 robot-scoped 키를 다룸 | §2.7.1 | framework 가 `self.robot_id` 를 요구하는 유일한 자리 = service 키 확장 (`app.py::_register_service`) |
| 22 | robot-agnostic 모듈의 per-robot config = resolve 의 lean 투영 주입 (필요만, 모듈이 robots.yaml 재보유 X). acceptance = 새 로봇 추가 시 모듈 코드 0 수정 + so101+omx 눈속임 방지 테스트 | §2.7.2 / §2.7.3 | 기능("한 robot 되니 끝") 아니라 아키텍처가 코드에 드러나야 |
| 23 | **liveliness = transport 기본 제공** (zenoh liveliness token) — Runtime 이 service 등록 시 같은 key 로 token **자동 선언**, **Mirror 가 이를 구독해 자동 refetch 수렴** (현재 유일 소비자). "모듈이 나 떴어요 publish" 손 컨벤션 금지 | infra/transport/zenoh.py + framework/runtime/app.py | 2026-07-07. 부팅 순서 = distribution 문제 = framework 책임 ("같은 코드가 어디 배치되든 그대로 동작"). 커스텀 ready 이벤트와 달리 **연속적 참** (세션 종료 시 자동 gone = 크래시/재시작 감지). probe 검증 4전이 = test_transport.py::test_liveliness_presence_lifecycle. (owner-대기 전용 `runtime.wait_for` 는 만들었다가 소비자 0 → 제거, 필요 시 subscribe_liveliness 위 5줄로 부활) |

### 작업 원칙

- 본 문서 = framework spec SSOT. 박힌 결정 (위 18개) 의심하지 말고 따를 것.
- 기존 backend/ 코드 = 도메인 logic reference 만 (BA / IRLS / Ruckig / ChArUco / Step DSL / Open3D ICP / TSDF / YOLO 등 알고리즘 자산). framework 부분 (BaseNode / `dict[robot_id]` dispatch / Cache singleton 등) 매몰 X.
- Step 1 부터 순차 implementation. 점프 X.
- 박지 말 패턴: 추가 옵션 카탈로그 던지기 / cost-based reflex ("한 줄 fix") / cargo cult (외부 framework 명명 흉내) / flipflop (사용자 push 자동 반대편 점프) / measurement 없는 추정.
- test 짤 때 production code 에 dogfood 넣지 X.

## 15. 구현 진행 → [backend_v2_status.md](backend_v2_status.md)

진행 status / 검증 수치 / 다음 작업은 전부 status 문서로 이동 (본 문서 = spec 만,
진행 표기 안 둠). 아래 test 원칙만 spec 으로 유지:

- **단순 통과 X** — test 통과 ≠ 설계 검증. 모든 test 가 "spec 의 어느 invariant 검증" 명시 박혀야 함
- 새 test 박을 때 = docstring 에 spec ref + invariant 명시 (예: `spec §3.3.2 — Mirror partial state 노출 X`)
- 구현 중 spec 충돌 / 새 invariant 발견 시 §14 anchor 표 update 박은 후 진행

## 16. Module catalog (옛 backend_v2_modules.md 통합, 2026-07-03 현행화)

### 16.1 4 layer + Module catalog

46 책임을 한 발 떨어져 보면 **4 layer 의 자연 분리** — 강제 layer architecture 아닌,
책임의 본질이 다른 묶음. framework 는 layer 모름 (duck typing).

```
┌──────────────────────────────────────────────────────────────┐
│ Layer 4 — Boundary    : Bridge, (Gamepad — 미래)             │
│ Layer 3 — Orchestration: (Task — 미래, §17)                  │
│ Layer 2 — Domain      : Motion, Calibration, Detector,       │
│                         Scene3D, Scan, Waypoint              │
│ Layer 1 — Hardware    : MotorDriver, CameraDriver,           │
│           + Derived     CameraDecoded                        │
└──────────────────────────────────────────────────────────────┘
```

| # | Module | Layer | Scope (§2.7) | Host | 한 줄 책임 | 영속성 |
|---|---|---|---|---|---|---|
| 1 | **MotorDriver** | Hardware | robot-scoped | pi_motor | Feetech/Dynamixel raw 통신 (state 20Hz + command + torque) | X |
| 2 | **CameraDriver** | Hardware | robot-scoped | pi_camera | RealSense capture + JPEG + depth zstd | X |
| 3 | **CameraDecoded** | Derived | robot-scoped | pc | JPEG→BGR + zstd→uint16 (decode dedup, 두 stream) | X |
| 4 | **Motion** | Domain | robot-scoped | pi_motor | kinematics(PyBullet) + Move/Jog primitive + TcpState | X |
| 5 | **Calibration** | Domain | robot-agnostic | pc | 5종 산출물 Bundle owner + capture/preview | DB + ObjectStore |
| 6 | **Detector** | Domain | robot-agnostic | pc | `Detect Object` (GDINO adapter 뒤) → base 3D | X |
| 7 | **Scene3D** | Domain | robot-agnostic | pc | RGBD primitive (라이브 cloud + consensus snapshot) | X |
| 8 | **Scan** | Domain | robot-agnostic | pc | scan 세션/캡처 + **TSDF build** (옛 Reconstruction 흡수) | DB + ObjectStore |
| 9 | **Waypoint** | Domain | robot-agnostic | pc | Robot Asset Layer — 티칭 joint 자세(rad) + group | DB |
| 10 | **Bridge** | Boundary | robot-agnostic | pc | WS relay + MJPEG + HTTP (`/robots` `/system` `/contract*`) + `/robot` static | X |
| — | Task / Gamepad | 미래 | robot-agnostic | pc | §17 task-first / 8BitDo jog dispatch | (DB) |

**합치지 / 더 잘라지 않은 근거** (한 Module = 한 정직한 책임 묶음):
- MotorDriver+Motion 분리 — vendor SDK swap 시 kinematics/Ruckig 변경 0.
- CameraDriver+CameraDecoded 분리 — host 횡단 (pyrealsense2 USB vs decode CPU).
- Scene3D+Scan 분리 — primitive vs workflow+persistence (trigger/cost profile 다름).
  단 옛 설계의 Scan/Reconstruction 분리는 v2 에서 **Scan 한 모듈** (build = `@service`
  + to_thread — 별도 모듈이 줄 격리 이득이 없었음).
- TcpState/JointRad 별도 Module X — small payload, Motion 안 fk+publish SSOT.
- LLM/검출 모델 별도 Module X — Detector 안 adapter (§17 "인터페이스 ≠ 구현").

### 16.2 Host 배치 + deployment yaml

| 머신 | Module | 이유 |
|---|---|---|
| **pi_motor** (192.168.0.101) | MotorDriver, Motion | 100Hz 명령 network 안 넘는 강제. IK 도 RTT 0 |
| **pi_camera** (192.168.0.102) | CameraDriver | pyrealsense2 USB 강제 |
| **pc** | 나머지 전부 | decode CPU + 무거운 연산 (Open3D/GDINO) + DB owner + browser |

| yaml | 의미 | driver |
|---|---|---|
| `pc.yaml` / `pi_motor.yaml` / `pi_camera.yaml` | 운영 분산 | real |
| `mock.yaml` | hardware 없이 전 Module 한 process (UX/wire 검증 + contract gen) | mock |

mock 은 별도 Module 아님 — `modules/<domain>/drivers/mock.py` driver subdir swap.
`dev.yaml`(단일 머신 풀스택 real) 안 둠 — 옛 backend 실사용 결과 불필요.

### 16.3 Cross-module 의존 + 데이터 성격 → primitive rule

| Reader | 패턴 | Source |
|---|---|---|
| Motion | **boot-time query** (`snapshot_bundle` 1회 — §9.3) | Calibration |
| Calibration | subscribe 캐시 (decoded frame + motor raw) | CameraDecoded / MotorDriver |
| Detector | call (매 detect) | Calibration / CameraDecoded / Motion |
| Scene3D | subscribe 캐시 (depth/color) + intrinsic query | CameraDecoded / Calibration |
| Scan | call (capture 시 scene3d SNAPSHOT / build 시 calibration bundle) | Scene3D / Calibration |
| Waypoint | subscribe 캐시 (TcpState) | Motion |
| Bridge | subscribe + relay | 모두 |

**framework 차원 decision rule** — 새 module 설계 시 "이 데이터는 어느 칸인가" 만
판단하면 primitive 가 결정된다:

| 데이터 성격 | 예시 | primitive |
|---|---|---|
| **Runtime telemetry** (지속 변화) | TCP pose 20Hz, joint/motor state, frame | **Stream** 또는 **snapshot service**. Mirror ❌ |
| **Boot-time configuration** (부팅 시 확정) | Capabilities | **Query** (boot 1회). 변경은 다음 부팅부터 |
| **Slowly-changing shared state** (가용 시점 불명 / 갱신 알림 필요) | CalibrationBundle (Motion 소비) | **Mirror** (liveliness 수렴 + on_change 반응, §3.3). 2026-07-07 활성 |

stream vs snapshot service 분리 예 — `Motion.Stream.TCP_STATE` (20Hz continuous,
frontend 시각화) vs `Motion.Service.TCP_SNAPSHOT` (Detector/Scan 의 point-in-time 1회).

### 16.4 Module SDK — bounded context

각 Module 이 자기 도메인의 SDK. **driver 공통 abstraction = Module SDK 안**
(`modules/<domain>/drivers/protocol.py`), framework X — 안 그러면 Gripper/Lidar/PLC
추가마다 framework 가 부풀음.

- 3 계층: ① framework (도메인 모름) ② Module SDK (contract + driver Protocol + impl)
  ③ consumer Module (framework 어휘만, driver Protocol 도 모름).
- `drivers/` 박을 자리 = hardware adapter swap 책임 (motor / camera / 미래 gripper).
  logic 자체가 책임인 Module (motion / calibration / detector / ...) 은 drivers/ 없음
  (detector 는 검출 모델 adapter 를 `backend.py` 로 — 같은 원리).
- 새 vendor / 새 도메인 추가 시 framework 변경 0, 다른 Module 변경 0.

### 16.5 Topology / Capability / Config 어휘 (driver self-declare)

> **Topology = "무엇이 존재하는가"** (구조 — consumer 가 구조 자체를 소비할 때만: Motor ✅
> `motors[id,kind]` / Camera ❌). **Capability = "무엇을 할 수 있는가"** (flags +
> supported max metadata). **Config = "현재 설정 값"**. 셋 다 부팅 1회 확정 → snapshot
> service, Mirror X.

- 값의 SSOT = **driver self-declare** (`driver.topology()` / `driver.capabilities()` —
  yaml 에 박으면 duplication/불일치). module 은 boot 1회 read + cache + service relay.
- Motor 의 GRIPPER / POSITION_PID capability 박지 X — Topology derived / baseline.
- **Intrinsic SSOT 분리**: CameraDriver 의 `get_factory_intrinsic` = Calibration seed
  전용 (internal). 모든 consumer 는 **Calibration Bundle 의 intrinsic** 만 봄 — Camera
  public contract 에 `GET_INTRINSICS` 박지 X (한 어휘 두 의미 ambiguity 차단).
- UI 는 capability flag 만 봄 (D405/UR/Basler 모름) — 새 hardware 추가 시 UI 변경 0.

### 16.6 Public contract surface — 두 소비자 + Bridge invariant

`contract.py` = 두 소비자의 SSOT (둘 다 **runtime-served**):
① **frontend TS gen** — bridge `GET /contract.json` EXPORT → `pnpm gen:types` 가
contract.ts 조립 ([frontend_v2.md §2.1](frontend_v2.md)). 노출 =
`apps/contract_export.py::FRONTEND_EXPOSED` opt-in allowlist 한 곳.
② **developer contract graph viewer** — bridge `GET /contract/graph` (unfiltered
declared universe) → frontend `/contract` React Flow ([contract_graph_viewer.md](contract_graph_viewer.md)).

- contract.py 만 generator 의 read 대상 — module.py / drivers/ / orm / repository 는
  internal (§3.0 "다른 module 이 import 박는가" 기준과 동일).
- **Stream payload invariant** — 모든 stream payload 에 `robot_id` + `seq: int` +
  `timestamp_unix: float` (frontend reconnect / lag / out-of-order 검출 기본 어휘).
- **Bridge = runtime relay only** — domain Module logic 박지 X. framework helper
  (`/robots` robot list / `/system` metric / `/contract*` export) 의 read-only relay 는
  OK. domain 데이터는 반드시 해당 Module 의 service 로 (Bridge 가 DB direct read 등
  우회 금지). heartbeat / logging 등 framework infra 는 Runtime 이 자동 흡수.
- `@service(description=, tags=)` metadata 확장은 viewer v2 자리 (현재 key 만).

### 16.7 후속 자리 (미래 조건 명시)

- **multi-camera per robot** — robots.yaml `camera:` 단수 강제 유지. wrist+workspace
  다중 필요 시 `(robot_id, camera_id)` device-scope 확장 (framework anchor 변경).
- **pyproject role-split** — 현재 단일 deps (bring-up 편의). 실 Pi 배포 시 PEP 735
  group (pi-motor / pi-camera / pc) 분리 — pyrealsense2 소스빌드/open3d 무게가
  load-bearing 해지는 시점.
- **Effective capability** (hardware ∧ runtime condition) / high-level composition
  (pick_and_place = depth ∧ cartesian ∧ gripper) — 실요구 시.

## 17. Task-first 운영 원칙 + Task/PnP 설계 (옛 task_dsl_waypoint_port.md 통합)

### 17.1 운영 원칙 (2026-07-03 잠금 — 프로젝트 전역 지배)

**핵심 관찰** — 실제 task(pick-and-place / 병따기 / 수건 접기)의 어려움은 **조합·순서가
아니라 primitive 안**에 있다 (`GraspCap()` 하나가 R&D 전체, 순서는 짧은 고정 스크립트).
n8n 식 워크플로 플랫폼을 지금 만들면 팔레트가 빈 조합기.

1. **산출물 = "실제 task 를 해내는 로봇"** (자란 DSL/플랫폼 아님). 인프라 중력의 균형추.
2. **승격 3-층 분류**:

   | 층 | 예 | 규칙 |
   |---|---|---|
   | **Day-1 primitive** | MoveJ/MoveL/Stop/Gripper/TCP pose/IK·FK/Detect Object | **처음부터 구축** (표준 산업 로봇 공통 제공) |
   | **Domain primitive** | GraspBottle/FoldTowel | **절대 미리 안 만듦** — task 로컬 → rule of three 승격 |
   | **Orchestration** | Loop/Retry/Parallel/비주얼 에디터 | **rule of three** (task 2개가 요구할 때) |

   판별 기준 2개 (둘 다 ✅ = 지금, 하나라도 ✗ = 대기): ① industry 가 표준 primitive 로
   출하하나? ② 하드웨어·대상·알고리즘 무관한 의미인가?
3. **인터페이스 ≠ 구현** — Day-1 은 *능력/의미* 만 계약 노출, 구현체는 adapter 뒤
   (예: `Detect Object` = Day-1 계약, Grounding DINO = 구현체 — YOLO/FoundationPose 교체
   가능). DSL/Runtime 은 "Detect Object" 만 앎.
4. **dev 안전장치는 분류와 직교, 지금 짓는다** — async runner + 디버거 (step/pause/
   breakpoint). hardware burn 직접 절감.

**Phase 순서 (task-first)** — "DSL 먼저 다 짓기" 폐기:
① 첫 task 선정 (= **단팔 pick-and-place**, 2026-07-03 확정) → ② 필요 primitive 정의 →
③ task #1 을 거의 DSL 없이 평범한 async 함수(`await runtime.call`) + 디버거만 얹어 구현
→ ④ task #2 에서 반복 보이면 Step/Slot 추출 → ⑤ 실요구 시 DSL 보강 (각 확장 rule of
three 게이트). 비주얼 에디터는 "문만 열어둠" — task 정본을 직렬화 가능한 스펙 (typed
Slot/Step) 으로 유지, primitive 가 쌓여 조합이 변수가 되는 시점에 얹음.

### 17.2 결정 로그 (D1–D11, 2026-07-03 잠금)

| # | 항목 | 결정 |
|---|---|---|
| D1 | 계층 | Motion → **Robot Asset Layer** (Waypoint = 첫 자산) → Consumer (PnP/Scan) |
| D2 | 실행 모델 | **async runner** — step `async def execute(ctx)` + `await runtime.call` (sync 유지 시 run_coroutine_threadsafe 브리지 부활) |
| D3 | 자산 이름 | Waypoint / WaypointGroup / WaypointGroupMember (UR 어휘 — "Pose" 는 TCP 연상) |
| D4 | 저장 | **joint-only, rad** (Motion.TcpState.joints 소비 — raw encoder 모름, 계층 준수) |
| D5 | Group | 3-테이블 + `order` 컬럼 — reorder/add/remove 가 행 단위, 드래그 UI 와 1:1 |
| D6 | 소유권 | Waypoint.robot_id = **instance id** (설치 위치/캘이 instance 별) |
| D7 | 티칭 | jog → 현재 joint 저장 (IK 재계산 X) |
| D8 | DSL 표면 | `MoveJ(waypoint=<ref>)` — resolve 는 runtime, 식별 방식과 DSL 분리 |
| D9 | Calibration pose | 별도 개념 유지 (알고리즘 생성 — 티칭 자산과 출처/생명주기 다름) |
| D10 | detection | **Top-K + 기하 prior 우선**, multi-view 3D 합의는 후속 (§17.5) |
| D11 | 자동 scan | defer (Waypoint Group 순회로 나중 흡수, UI/UX 사용자 결정 후) |

### 17.3 Motion 완료 계약 (잠금 — 흔들리지 말 것)

- **외부 계약**: `await motion.move_j()` / `move_l()` 는 **trajectory 정상 종료(DONE)
  거나 오류로 끝났을 때 반환**. DONE → return / IK 실패·충돌·Ruckig 오류 (FAILED) →
  **exception** (`MotionFailed`, v2 exception propagation) / STOP → cancellation.
  Task 규칙 하나: **await 성공 = 완료, 다음 step.** (구현: traj thread terminal 상태를
  `asyncio.Future` 로 resolve — 내부 방식은 driver 별 자유, "인터페이스 ≠ 구현".)
- **MoveL v1 제약**: ✓ XYZ 직선 / ✗ orientation 보장 안 함 (position-only IK).
  orientation interpolation (SLERP) = MoveL v2 — 실 task 요구 시 (rule of three).

### 17.4 Task DSL 재이주 매핑 (구현 대기 — 옛 backend 자산의 v2 적응)

옛 `backend/` 의 성숙한 Task DSL (typed Slot lego + 디버거 UI) 을 v2 primitive 로 매핑
(기계적 복사 X):

| 옛 | → v2 |
|---|---|
| TaskNode (ApplicationNode) | TaskModule plain class, PC |
| `ctx.call_motion` (sync + traj Event) | `await self.runtime.call(...)` |
| TaskRunner (threading + Event) | async runner (`asyncio.Event` 게이트, `await pause_event.wait()`) |
| Slot / StepResult / Step / StepContext / task_tree | 거의 그대로 (ctx 가 node 대신 runtime 보유) |
| run/preview/stop/resume/step/run_to/toggle_breakpoint | `@service async def` |
| TASK_STATE / TREE / STEP_RESULT | `@publishes` streams |

디버거 게이트 (`_should_pause_before` → pause → 실행 → publish) 동작 보존, ForEach/Try
의 `ctx.run_child` 재진입도 동일. 검증 = 도메인 step 0개 trivial task (Wait + no-op) 로
runner+디버거 e2e 부터. frontend = TaskProgressPanel / PromptPanel / TaskResultLayer 포팅.
공유 값 타입 (Position3/Pose6/Detection) 위치는 빌드 시 결정 (task+detector 공유).

### 17.5 PnP consumer 설계 (구현 대기)

- step 매핑: `MoveJ(waypoint=<ref>)` (D8) / `MoveTCP`→MOVE_L / Gripper+VerifyGrasp /
  GraspPolicy·PlacePolicy (순수 계산 그대로) / GroundedDetect → Detector.DETECT Top-K.
- **detection 개선** (옛 first-match-wins 오검출 — 흰 안경닦이를 흰 큐브로 오인):
  ① **Top-K** (진짜 물체가 2등이면 top-1 은 영원히 누락) ② **기하 prior** (depth 의
  height/base_z 로 예상 범위 밖 reject — confidence 무관 구분) ③ multi-view 3D 합의는
  후속 (후보 누적 구조만 먼저, 스코어링은 실 데이터 보며).
- recipe 재설계: BreakIf 제거 → **Waypoint Group 순회하며 후보 누적** →
  `SelectTarget(candidates, prompt, priors)` 스코어 → 최종 Detection.
- 검증: 구조/계약/mock e2e = 회사, **detection 정확도 = 집 하드웨어만**.
