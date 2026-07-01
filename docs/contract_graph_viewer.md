# contract_graph_viewer — backend_v2 계약 토폴로지 그래프 뷰어

> **상태: 구현 완료 (2026-07-01).** backend export (framework `build_module_contracts`
> + apps `build_contract_graph` + bridge `GET /contract/graph`) + frontend
> `features/contract-viewer/` (`/contract` lazy route) 다 짜고 검증됨. 검증 = backend
> pytest 141 PASS (신규 `test_module_contracts.py` 6 + `test_contract_export.py` 그래프
> 8) + frontend vitest 40 PASS (신규 `toReactFlow.test.ts` 6) + build (React Flow
> chunk code-split 확인) + 실 `--host mock` 서버 `GET /contract/graph` curl + **L4
> Playwright headed e2e 2 PASS** (`e2e/contract-graph.spec.ts` — /contract 로드 → 4
> module 노드 + 엣지 렌더 + 엣지 클릭 스키마 드릴다운, mock backend + vite dev). 4계층
> 검증 전부 통과. 아래 §0~§9 는 구현 근거로 보존. **구현 결과 편차는 §11.**

## 0. TL;DR

backend_v2 의 module 계약(service/stream/event)을 **Swagger 식 flat 나열이 아니라 노드+방향엣지 그래프**로 보여주는 **개발자 도구**. `backend_v2_modules.md §8` 의 "Developer contract viewer (Swagger-like)" 두 번째 소비자를 구현하는 것 ([backend_v2_modules.md](backend_v2_modules.md) §8.1 + §14 anchor 15).

**왜 그래프인가**: HTTP 엔드포인트는 서로 독립이라 flat 나열로 충분하지만, 우리 계약은 **pub/sub 토폴로지** — module 들이 topic 으로 물려 돌아간다. 나열로는 그 관계가 안 드러난다.

```
Motor  ──RAW_STATE(stream)──▶  Motion
Motion ──COMMAND(stream)────▶  Motor
frontend ──JOG_J(stream)────▶  Motion
Motion : MOVE_J(service) 소유(server)
```

**결정 (이번 세션 확정)**:
- **runtime-served** — Swagger `/docs` 도 서버 떠 있어야 보는 것처럼, 뷰어도 떠 있는 backend 가 데이터를 내준다. "서버 떠 있어야 볼 수 있음" 은 문제 아님 (§8.1 의 "런타임 무관 CLI" 원안은 폐기 — frontend gen 이 runtime `/contract.json` 으로 pivot 한 것과 동일 이유 + description/tags 가 module.py 에 살아 runtime 필요 + Swagger 자체가 runtime-served).
- **위치 = frontend `/contract` 페이지 (React Flow), 책임 = 별도 consumer.** frontend 에 얹혀살되 frontend "기능" 이 아니라, gen:types 와 동일한 **HTTP consumer**. 위치만 같이, 책임은 분리.
- **backend = EXPORT (`GET /contract/graph`), frontend = CONSUME (HTTP only).** frontend 는 backend 코드도, 앱의 필터된 `contract.ts` 도 참조 X.
- **unfiltered** — `FRONTEND_EXPOSED` subset 이 아니라 **전 module 의 전 계약** (개발자 가시성 목적).

```
backend runtime (전 module 로드)
  │ framework: build_module_contracts()  = module 별 serve/publish/subscribe attribution
  │ apps: build_contract_graph()          = modules + keys + models(schema) + edges JSON
  │ bridge: GET /contract/graph           = 위 JSON serve (relay only, unfiltered)
  ▼
{ modules, keys, models, edges }
  │ HTTP fetch  (frontend Node/React, backend 접근 0)
  ▼
frontend_v2/src/features/contract-viewer/  →  /contract 페이지 (React Flow + dagre)
```

---

## 1. 목적 + 비-목적

- **목적**: "지금 backend_v2 아키텍처가 실제로 어떻게 연결돼 있나" 를 한눈에. 개발 중 머릿속으로만 알던 module↔module wiring 을 시각화.
- **비-목적**: 외부 고객용 문서 X / 독립 제품 X / 별도 배포 서비스 X. **개발 도구.** 그래서 frontend 에 얹혀사는 게 합리적 (§8 경계만 지키면).

## 2. 무엇을 그리나 (관계의 종류 + 한계)

| 어휘 | 엣지 유도 | 완전성 |
|---|---|---|
| **Stream** | `@publishes` module → `@subscriber` module (방향) | ✅ 계약에서 완전히 유도 |
| **Event** | `@publishes` module → `@subscriber` module (방향) | ✅ 완전 |
| **Service** | `@service` module = **owner(server)** 노드에 attach | ⚠️ **caller 엣지는 없음** |

**⚠️ service caller 한계 (반드시 인지)**: service 를 "누가 call 하나" 는 런타임에 `runtime.call(key)` 로 **동적 호출**이라 계약(decorator)에 선언이 없다. 그래서 service 는 **owner node 의 속성**으로만 표시되고, "caller → service" 화살표는 **정적으로 안 잡힌다**. v1 은 이 한계를 그대로 수용 (owner-attached, caller 엣지 없음). caller 엣지를 원하면 별도 장치 필요 (§9).

## 3. 이미 있는 토대 (2026-07-01 frontend gen 작업 산출물 — 재사용)

이 뷰어는 무에서 짓는 게 아니다. 계약 gen 작업에서 만든 것들이 토대:

- **[framework/runtime/snapshot.py](../backend_v2/framework/runtime/snapshot.py)** — `Runtime.contract_snapshot()` + `build_snapshot(modules)`. 로드된 module 의 `@service`/`@subscriber`/`@publishes` 를 열거. **단 지금은 module attribution + 방향(publish/subscribe)을 버리고** `services`/`topics` flat dict 로 collapse. → 그래프는 이 attribution 을 살려야 함 (§5.1 enrich).
- **[framework/runtime/discovery.py](../backend_v2/framework/runtime/discovery.py)** — `discover_services(m)` / `discover_subscribers(m)`. `build_snapshot` 이 이걸 module 별로 이미 호출 중 (attribution 이 그 지점에 있음, 버려질 뿐).
- **[framework/contract/publisher.py](../backend_v2/framework/contract/publisher.py)** — `get_publishes_spec(type(m))`.
- **[apps/contract_export.py](../backend_v2/apps/contract_export.py)** — `ts_type()` / model field 추출 / name-conflict resolution. **그래프의 payload 스키마 표시에 재사용 가능** (또는 `model_json_schema()`).
- **bridge endpoint 패턴** — [apps/resolve.py](../backend_v2/apps/resolve.py) 가 runtime 을 capture 하는 provider closure 를 `BridgeModule` 에 주입, bridge 가 `GET /contract.json` serve. **`GET /contract/graph` 는 완전히 같은 패턴** (provider 만 unfiltered graph 빌더로).

즉 **frontend gen 과 대칭**: `contract_snapshot` 이라는 같은 source, 다른 빌더(unfiltered + attribution), 다른 소비자(그래프 뷰어).

## 4. 데이터 계약 — `GET /contract/graph` 응답 스키마

backend 는 **React Flow 형식을 모르는 중립 그래프**를 낸다 (position/React-Flow 세부는 frontend 가 layout). `/contract.json` 처럼 backend = 데이터, frontend = 표현.

```jsonc
{
  "modules": [
    {
      "id": "MotorDriverModule",          // module class name (per-robot 인스턴스는 dedupe)
      "domain": "motor",                  // wire_key prefix 에서 유도 (그룹 색상 등)
      "robot_scoped": true,               // {robot_id} 템플릿 포함 여부
      "services":   ["srv/motor/{robot_id}/set_torque", "srv/motor/{robot_id}/capabilities"],
      "publishes":  ["stream/motor/{robot_id}/raw_state", "event/motor/{robot_id}/torque_changed"],
      "subscribes": ["stream/motor/{robot_id}/command"]
    }
    // ... motion / camera / camera_decoded / bridge ...
  ],
  "keys": {
    "stream/motor/{robot_id}/raw_state":  { "category": "stream",  "payload": "JointState" },
    "stream/motor/{robot_id}/command":    { "category": "stream",  "payload": "JointCommand" },
    "event/motor/{robot_id}/torque_changed": { "category": "event", "payload": "TorqueChanged" },
    "srv/motion/{robot_id}/move_j":       { "category": "service", "req": "MoveJRequest", "res": "MoveJResponse" }
  },
  "models": {
    // 드릴다운용 — 참조된 payload/req/res 모델의 field:type. model_json_schema()
    // 또는 contract_export.ts_type 기반 field map 재사용.
    "JointState":   { "robot_id": "str", "seq": "int", "timestamp_unix": "float", "positions_raw": "list[int]" },
    "MoveJRequest": { "target_joints": "list[float]" }
  },
  "edges": [
    // stream/event 만. publisher module → subscriber module (key 별 publisher×subscriber cross product).
    { "source": "MotorDriverModule", "target": "MotionModule",       "key": "stream/motor/{robot_id}/raw_state", "category": "stream" },
    { "source": "MotionModule",      "target": "MotorDriverModule",  "key": "stream/motor/{robot_id}/command",   "category": "stream" }
    // service 엣지 없음 (§2 caller 한계) — service 는 modules[].services 로 owner 에 attach
  ]
}
```

**edge 유도 규칙**: 각 stream/event `key` 에 대해 `publishes` 에 그 key 를 가진 module(들) × `subscribes` 에 가진 module(들) 의 곱 = 방향 엣지. (mock/dev = 전 module 로드라 완전. partial host 는 엣지 불완전 — 뷰어는 mock/dev 대상, gen 과 동일.)

## 5. Backend 구현

### 5.1 framework — module attribution 살린 열거
[framework/runtime/snapshot.py](../backend_v2/framework/runtime/snapshot.py) 에 추가 (기존 `ContractSnapshot`/`build_snapshot` 은 frontend gen 이 쓰므로 **건드리지 말고 병렬 추가**):

```python
@dataclass(frozen=True)
class ModuleContract:
    module_id: str            # type(m).__name__
    robot_scoped: bool        # any wire_key 에 {robot_id}
    services: tuple[str, ...]     # @service wire_keys
    publishes: tuple[str, ...]    # @publishes wire_keys
    subscribes: tuple[str, ...]   # @subscriber wire_keys

def build_module_contracts(modules: list[Any]) -> list[ModuleContract]:
    # per-robot 인스턴스 (같은 class) 는 module_id 로 dedupe (wire_key 템플릿 동일).
    # discover_services / discover_subscribers / get_publishes_spec 를 module 별로 호출,
    # attribution 을 버리지 말고 유지.
```
`Runtime.module_contracts()` 메서드로 노출 (`contract_snapshot()` 옆).

### 5.2 apps — 그래프 JSON 빌더
[apps/contract_export.py](../backend_v2/apps/contract_export.py) (또는 신규 `apps/contract_graph.py`) 에 `build_contract_graph(module_contracts, snapshot) -> dict`:
- `modules` = ModuleContract → dict (+ `domain` = wire_key prefix 파싱).
- `keys` = 각 wire_key 의 category(prefix) + payload/req/res 모델명 (snapshot 에서).
- `models` = 참조 모델의 field map (`model_json_schema()` 또는 `contract_export` 의 field 추출 재사용).
- `edges` = §4 규칙으로 stream/event key 별 publisher×subscriber.
- **필터 없음** — `FRONTEND_EXPOSED` 무시, 전 계약.

### 5.3 bridge — `GET /contract/graph`
`GET /contract.json` 과 동일 패턴. [apps/resolve.py](../backend_v2/apps/resolve.py) 의 bridge dep 에 두 번째 provider closure (`graph_provider = lambda: build_contract_graph(runtime.module_contracts(), runtime.contract_snapshot())`) 주입, [modules/bridge/module.py](../backend_v2/modules/bridge/module.py) 에 route 추가 (memoize, relay only, 도메인 로직 0).

## 6. Frontend 구현

### 6.1 위치 + 격리
```
frontend_v2/src/features/contract-viewer/    # feature 격리 (나중에 별도 앱 추출 가능)
├── ContractGraphPage.tsx    # /contract route entry
├── api.ts                   # fetch GET /contract/graph
├── types.ts                 # graph payload 타입 (아래 §6.3)
├── toReactFlow.ts           # {modules,edges} → React Flow {nodes,edges} + dagre layout
└── nodes/ModuleNode.tsx     # module 노드 (services/streams/events 목록 + 클릭 → 스키마 패널)
```
- **route `/contract` 는 lazy import** (`React.lazy` + dynamic import) — React Flow 가 control/simulator 번들 안 불리게 code-split.
- Sidebar 에 `/contract` 링크 추가 (dev 도구 자리).

### 6.2 라이브러리
- **React Flow = [`@xyflow/react`](https://reactflow.dev)** (v12+ 패키지명. 옛 `react-flow-renderer` 아님). `pnpm add @xyflow/react`.
- **auto-layout** — React Flow 는 위치를 안 정해주므로 layouting 필요. **[`@dagrejs/dagre`](https://github.com/dagrejs/dagre)** (방향 그래프 계층 레이아웃) 또는 `elkjs`. `toReactFlow.ts` 에서 dagre 로 좌표 계산 후 nodes 에 주입.

### 6.3 타입 — 앱의 `contract.ts` 재사용 금지
`src/api/generated/contract.ts` 는 **FRONTEND_EXPOSED subset (앱이 실제 쓰는 wire)** 이라 뷰어(전 계약)와 성격이 다르다. 뷰어는 `/contract/graph` payload 전용 타입을 `features/contract-viewer/types.ts` 에 **직접** 둔다 (작아서 hand-write 충분, 또는 graph 엔드포인트용 별도 gen). **앱 generated types 를 import 하지 말 것.**

### 6.4 렌더
- 노드 = module (domain 별 색). 노드 안에 services / streams(pub) / events / subscribes 섹션.
- 엣지 = stream/event, `key` 라벨 + 방향 화살표. category 별 스타일(stream vs event).
- 노드/엣지 클릭 → 사이드 패널에 payload 스키마(`models`) 펼침 = "Swagger 드릴다운" 부분.

## 7. 구현 순서 (다음 세션 체크리스트)

1. [ ] backend: `ModuleContract` + `build_module_contracts` + `Runtime.module_contracts()` (framework/runtime/snapshot.py, 병렬 추가). **L2 test** = mock runtime 에서 module attribution 정확 (Motor 가 raw_state publish + command subscribe 등).
2. [ ] backend: `build_contract_graph()` (apps). **L2 test** = edges 방향 정확 (Motor→Motion via raw_state, Motion→Motor via command), service 엣지 0, unfiltered (COMMAND 등 내부 wire 포함).
3. [ ] backend: `GET /contract/graph` (bridge + resolve provider). **L3 test** = HTTP 200 + shape (test_contract_export.py 패턴).
4. [ ] frontend: `pnpm add @xyflow/react @dagrejs/dagre`.
5. [ ] frontend: `features/contract-viewer/` (api + types + toReactFlow + ModuleNode + Page).
6. [ ] frontend: `/contract` lazy route + Sidebar 링크.
7. [ ] frontend: **L2 test** = toReactFlow 매핑 (graph JSON → nodes/edges 개수·방향). 렌더는 **L4 e2e** 후보 (mock backend + 페이지 로드 → 노드 N개 보임).
8. [ ] 검증: 4계층 다 (§frontend_v2.md §12) — "verified" 는 backend pytest + frontend vitest + build + (해당되면) e2e 다 돌린 뒤에만.

## 8. 경계 규칙 (⚠️ 어기지 말 것 — 이번 세션에 힘들게 세운 원칙)

- **frontend 는 backend 코드/폴더/python 환경에 접근 X.** 오직 `GET /contract/graph` HTTP fetch. (gen:types 를 `cd ../backend_v2 && uv run python` 으로 짰다가 갈아엎은 것과 같은 실수 반복 금지 — [frontend_v2.md §2.1](frontend_v2.md).)
- **backend = EXPORT, frontend = CONSUME.** 데이터 생성은 backend, 표현은 frontend.
- **뷰어는 feature 로 격리 + lazy** — control/simulator 앱에 안 섞이게. "위치만 같이, 책임은 분리."
- **앱의 필터된 `contract.ts` 재사용 X** (§6.3).
- **backend CLI 로 짓지 말 것** — §8.1 원안(`python -m horibot.contract_viewer`)은 폐기. runtime-served (frontend 페이지 + bridge endpoint).
- **contract.py / module.py 순수 유지** — 뷰어 개념 안 넣음 (frontend gen 과 동일 — [frontend_v2.md §2.1](frontend_v2.md)).

## 9. Open questions / 후속 (v1 에선 defer)

- **service caller 엣지** (§2 한계) — 정적으로 안 잡힘. 옵션: (a) v1 = owner-attached, caller 엣지 없음 (추천); (b) 후속: caller 를 어딘가 선언 (registry) 또는 runtime call 로그 수집. v1 은 (a).
- **`@service(description=, tags=)` metadata** ([backend_v2_modules.md §8.4](backend_v2_modules.md)) — 현재 `@service` 는 key 만 받음 ([framework/contract/service.py](../backend_v2/framework/contract/service.py)). 뷰어에 human 설명/태그 달려면 이 확장 필요. **v1 defer** — key + payload 스키마 + wiring 만. metadata 는 v2 (framework `@service` 확장 + graph JSON 에 desc/tags 추가).
- **노드 granularity** — v1 = module 노드 + topic 엣지 + service owner-attach. topic/service 를 별도 노드로 승격은 후속 (규모 커지면).

## 11. 구현 결과 + 설계 대비 편차 (2026-07-01)

설계(§4~§9)를 거의 그대로 따르되, 기존 인프라 재사용을 위해 3곳 미세 조정 (땜빵/중복 회피):

1. **`models` field type = `ts_type` (TS 방언)** — §4 예시는 `"list[int]"` 같은 python 표기였으나, `contract_export.ts_type` (union/list/nested/enum + name-conflict 해소 이미 처리) 를 그대로 재사용 → `"number[]"` 같은 TS 표기. python-type 렌더러를 따로 짜는 중복 회피. 뷰어가 TS 프론트라 방언도 자연스러움.
2. **frontend fetch = `useResource` 재사용** — §6.1 의 별도 `api.ts` 대신 framework HTTP consumer(`useRobots` 가 쓰는 `useResource`)를 얇게 감싼 `useContractGraph.ts`. fetch 코드 새로 안 짬 (§8 "backend 접근 0, HTTP only" 경계는 그대로).
3. **empty module 처리 = 2-layer** — framework `build_module_contracts` 는 contract 0 module(Bridge)도 정직하게 열거(editorialize X), apps `build_contract_graph` 가 graph node 에서 제외. "무엇이 node 인가"는 그래프 빌더 관심사.

**핵심 불변 (설계대로 지켜짐)**: unfiltered (FRONTEND_EXPOSED 무시 — 내부 `stream/motor/{robot_id}/command` + `JointCommand` 그래프에 포함, `test_graph_is_unfiltered` 가 못박음) / service 엣지 없음 (owner-attach, §2) / stream·event 만 publisher→subscriber 방향 엣지 / 앱 generated `contract.ts` import 0 (뷰어 전용 `types.ts`) / React Flow lazy code-split.

**파일**: backend = [snapshot.py](../backend_v2/framework/runtime/snapshot.py) (`ModuleContract`/`build_module_contracts`) + [app.py](../backend_v2/framework/runtime/app.py) (`Runtime.module_contracts()`) + [contract_export.py](../backend_v2/apps/contract_export.py) (`build_contract_graph`) + [resolve.py](../backend_v2/apps/resolve.py) (`_graph_provider`) + [bridge/module.py](../backend_v2/modules/bridge/module.py) (`GET /contract/graph`). frontend = `frontend_v2/src/features/contract-viewer/` (types / useContractGraph / toReactFlow / nodes/ModuleNode / ContractGraphPage) + `App.tsx` lazy route + `Sidebar.tsx` Dev 링크.

## 10. 인접 문서
- [backend_v2_modules.md §8](backend_v2_modules.md) — "두 generator 의 SSOT". 본 뷰어 = 그 두 번째 소비자(contract viewer)의 구현 결정. §8.1 의 CLI 원안은 본 문서가 runtime-served frontend 페이지로 정정.
- [frontend_v2.md §2.1](frontend_v2.md) — frontend gen (첫 번째 소비자) + backend EXPORT / frontend CONSUME 경계. 본 뷰어가 그 경계를 그대로 따름.
- [backend_v2.md](backend_v2.md) — framework spec (`@service`/`@publishes`/`@subscriber` origin).
