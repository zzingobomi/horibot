# backend_v2 — Framework 구현 spec

> 본 문서는 [framework_dogfood_plan.md](framework_dogfood_plan.md) 의 §15 Runtime-centric reframe 위에 추가된 8 라운드 토론 (2026-06-25) 의 정리. framework_dogfood_plan = 결정 history + plan, 본 문서 = **현재 결정의 구현 spec**.
>
> 다음 세션 진입 시 framework 구현 시작점 = 본 문서.

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

### 2.7 Module scope — robot-scoped / robot-agnostic

Module 두 종류. **기준 = "Module 이 robot 의 runtime state / 물리 자원을 소유하는가"**.

| 종류 | 예시 | 인스턴스 |
|---|---|---|
| **robot-scoped** | MotorModule / MotionModule / CameraDriver / CameraDecoded | per-robot (Module type × robot_id) |
| **robot-agnostic** | CalibrationModule / ScanModule / ReconstructionModule / TaskModule / DetectorModule / Bridge | host 당 1 |

- robot-scoped = *물리 자원 owner* (Dynamixel handle / RealSense handle / robot kinematics state). 자원은 robot 별 분리되어야 자연.
- robot-agnostic = *작업 / orchestration*. robot_id 는 매 service request 의 인자 (req 안 field). DB 의 `robot_id` column 으로 multi-tenant.

기존 backend 의 `DeviceNode` (per-robot) / `ApplicationNode` (host 당 1 + `enabled_robot_ids` dict) 패턴과 본질 동일 — 새 spec 의 차이는 ApplicationNode 의 `dict[robot_id, _state]` boilerplate 가 Repository 의 robot_id parameter 로 흡수.

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

Mirror cache 갱신 = framework 자동. 단 *effective apply* (예: PyBullet 의 URDF 재로드 같은 architectural side-effect) 는 consumer 책임. framework 가 `@on_mirror_change` 같은 magic 데코 박지 X — 그저 **일반 `@subscriber(ChangeEvent)`** 박아 자기 도메인 처리.

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
│   │   └── module.py             # MotionModule(robot_id) + Mirror[CalibrationBundle]
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

### 9.2 Migration owner

기존: Storage Module 부팅 시 Alembic `upgrade head` 한 번.

새 spec: 각 Module 의 `start()` 안 자기 Alembic 호출. 각 Module 이 자기 schema directory 가짐:

```
modules/calibration/
    alembic/
        versions/
        env.py
    alembic.ini
```

Module N 개 = Alembic N 개. 한 사람 환경에선 OK. 같은 NAS Postgres 면 각 Module 이 자기 table 만 만들어서 schema 충돌 0.

### 9.3 DB dependency 격리

기존: Pi 가 Storage Module service 호출, SQLAlchemy import 0.

새 spec: Pi 의 Module 들 (motor / motion / camera) 은 *Reader 만*, 자기 DB 안 가짐. PC 의 Calibration Module 이 owner, Pi 의 Motion 은 `Mirror[ActiveCalibration]` 로 받음. Pi 에 SQLAlchemy / Postgres driver import 0 유지.

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

`modules/motion/`. `Mirror[ActiveCalibration]` + kinematics + IK.

검증:
- 부팅 시 MotionModule 의 `self.calibration` 이 fresh cache.
- Owner 측 `activate(new_result)` 호출 → 잠시 후 Reader 의 `self.calibration` 갱신.
- Same-process (PC 한 process) + cross-process (PC + 모터 Pi sim) 두 case PASS.

### Step 7.5 — Derived read model 검증 (CameraDriver + CameraDecoded)

`modules/camera/module.py` (CameraDriver) + `modules/camera/decoded.py` (CameraDecoded).

검증:
- CameraDriver mock impl 이 JPEG bytes publish (실 hardware 없이 합성 frame).
- CameraDecoded 가 `/camera/jpeg` subscribe + `cv2.imdecode` + `CameraFrame` publish.
- Consumer Module (테스트용 dummy) 가 `Mirror[CameraFrame]` 으로 받음.
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
| 2 | Mirror[CalibrationBundle] 단일 (4 종 sub-field 분리 X) | §3.3.3 + §4.1 | atomic BA 단위 = 한 ResultBundle. sub-field 별 4-5 개 Mirror = 옛 4 종 npz 파일 분리의 매몰 |
| 3 | Exception propagation (envelope `{success, message, data}` X) | §3.1 | Python 자연 = exception. caller 매 호출 `if not res.success` boilerplate 회피. transport 가 `RemoteError(type, message)` raise |
| 4 | Database-per-Module (Storage Module 폐기) | §2.4 + §9 | centralization motivation 은 Mirror 가 흡수. migration / DB dep 격리도 자연 해결 |
| 5 | Module = plain class (`@module` 데코 / base class 강제 X) | §3 + §7.1 | backend/ BaseNode 부풀음 (15+ method 누적) 경로 차단. framework 는 `@service` / `@subscriber` / `Mirror` 박힌 attribute 만 inspect |
| 6 | ModuleRuntime Protocol + constructor 주입 (base class / setattr / ctx X) | §3.7 + §4 | base class 부풀음 + ctx 의 한 문장 정의 fail. composition 한 Protocol 이 sweet spot |
| 7 | Wire key = explicit + typed at every use site | §3.0 / §3.1 / §3.2 / §3.3 / §3.7 | 세 원칙: ① 사람이 explicit 지정 모든 use site ② raw string X (typed StrEnum) ③ **service 가리키는 방법 = 항상 `Service.X` enum 하나** (method reference X) |
| 8 | `runtime.call(key, req, res_cls, *, robot_id, timeout) -> TRes` (method ref X) | §3.7 + §4 | 원칙 ③ 정합. publish/subscribe 와 동일 패턴. res_cls 명시로 return type narrow |
| 9 | `Mirror(snapshot_service, snapshot_req, change_topic, value_cls, change_event_cls)` (method ref X) | §3.3 + §4.2 | 5 인자 모두 key + Pydantic class. `snapshot_req` = factory `Callable[[self], Req]`. cross-module method import 사라짐 |
| 10 | Wire encoding = Pydantic + msgpack layered (DIP) | §3.4 | Module 은 Pydantic schema 만 알고, Transport boundary 가 msgpack 처리. `bytes` field native pass-through (JPEG base64 33% overhead 회피) |
| 11 | Topic prefix = `srv/` / `event/` / `stream/` (`horibot/` X) | §3.0 | 세 종류 첫 chunk 분리 — srv=RPC / event=state notification / stream=고빈도 raw. broker 단일 project 라 namespace prefix 불필요 |
| 12 | `stream` ≠ `event` (nested class naming) | §3.5 / §7.4 | `Module.Stream` 으로 명명 (`Camera.Stream`). `Module.Event` 는 진짜 state notification 만 |
| 13 | Wildcard subscribe = transport detail, framework 어휘 X | §3.7 | `@subscriber("*")` 같은 implicit pattern 금지. robot-scoped event 는 framework 가 `{robot_id}` → transport wildcard substitute, 사용자 코드에 등장 X |
| 14 | robot scope = yaml primary (constructor 가 계약 검증) | §2.7 | yaml `robots: [...]` 박힘 = robot-scoped Module. constructor 에 robot_id parameter 있어야 |
| 15 | Derived read model 패턴 (decode dedup) | §3.5 | 큰 payload (JPEG/depth) 의 decode 비용 = N consumer × decode. framework primitive 가 아닌 Module 패턴 — CameraDecoded 가 1회 decode 후 fanout |
| 16 | Runtime 부팅 순서 = instantiate → register → start | §3.6 | Mirror snapshot / Module A start 이 다른 Module service 호출 — Phase 2 (register) 가 Phase 3 (start) 이전 완료 보장 |
| 17 | Mirror invariant — partial state 노출 X | §3.3.2 | event callback thread vs handler thread race 차단. 구현 자유 (lock / atomic / RCU) |
| 18 | Mirror 동기화 = invalidate+refetch only (push update X) | §3.3.5 | event = 변경 알림 / snapshot = 진실. Bundle atomic 보존 + 다른 Module event 가 trigger 인 use case 자연 표현. push update 필요하면 Mirror 안 박고 `@subscriber` 직접 |

### 작업 원칙

- 본 문서 = framework spec SSOT. 박힌 결정 (위 18개) 의심하지 말고 따를 것.
- 기존 backend/ 코드 = 도메인 logic reference 만 (BA / IRLS / Ruckig / ChArUco / Step DSL / Open3D ICP / TSDF / YOLO 등 알고리즘 자산). framework 부분 (BaseNode / `dict[robot_id]` dispatch / Cache singleton 등) 매몰 X.
- Step 1 부터 순차 implementation. 점프 X.
- 박지 말 패턴: 추가 옵션 카탈로그 던지기 / cost-based reflex ("한 줄 fix") / cargo cult (외부 framework 명명 흉내) / flipflop (사용자 push 자동 반대편 점프) / measurement 없는 추정.
- test 짤 때 production code 에 dogfood 넣지 X.

## 15. 구현 진행 (2026-06-27)

### 완료 — Step 1-5

| Step | 산출물 | test |
|---|---|---|
| 1. Transport | `framework/transport/protocol.py` + `infra/transport/zenoh.py` | 7 |
| 2. Contract | `framework/contract/{envelope,service,subscriber,publisher}.py` | 19 |
| 3. Runtime | `framework/runtime/{api,app,lifecycle,discovery}.py` | 12 |
| 4. Persistence + ObjectStore | `framework/{persistence,storage}/protocol.py` + `infra/database/{sqlite,postgres}.py` + `infra/object_store/{filesystem,minio}.py` | 17 (4+13) |
| 5. Mirror | `framework/contract/mirror.py` + `framework/runtime/app.py` 통합 | 12 |

**total 67 PASS** / ruff / pyright clean.

### Step 5 신규 추가 (catalog v3.2 의 invariant 검증)

- `test_mirror_event_during_init_not_lost` — **M5.1 / M5.4**: INITIALIZING 중 받은 event 가 snapshot 후 fresh refetch trigger. spec §3.3.1 의 result invariant (READY 이전 event 안 잃음) 검증
- `test_mirror_concurrent_read_write_no_partial_state` — **M6.1**: 동시 read/write 박혀도 partial state 노출 X. RLock 검증 (writer/reader 2 thread × 1000 회, invariant = `bundle_id == hand_eye[0]`)

### 다음 진입점 = Step 6 (Calibration Module)

`modules/calibration/` — ORM + Repository + Module + Alembic. 검증 = §11 Step 6.

### 박힌 결정 일람

모든 design decision = §14 anchor 표 단일 source. 본 §15 = 진행 status 만.

### 다음 세션 진입 시 — test 검증 원칙

2026-06-27 review 박힌 원칙 (대화 안만 박혔던 자리 — 잊지 말 것):

- **단순 통과 X** — test 통과 ≠ 설계 검증. 모든 test 가 "spec 의 어느 invariant 검증" 명시 박혀야 함
- 새 test 박을 때 = docstring 에 spec ref + invariant 명시 (예: `spec §3.3.2 — Mirror partial state 노출 X`)
- 구현 중 spec 충돌 / 새 invariant 발견 시 §14 anchor 표 update 박은 후 진행 — 발견 외에는 catalog 더 다듬지 X
- 박지 말 것: 단순 어휘 정리 + "통과 박았으니 설계 검증 완료" 같은 얕은 결론
