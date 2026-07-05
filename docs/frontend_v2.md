# frontend_v2 — Frontend 구현 + 진행 status

> backend_v2 ([backend_v2.md](backend_v2.md)) 와 1:1 정합되는 React frontend. **frontend_v2 의 일 = 옛 `frontend/` 의 검증된 아키텍처를 backend_v2 wire 에 맞게 포팅하는 것** — "뭘 지을지 새로 설계" 가 아니다.
>
> ⚠️ **옛 `frontend/` 는 지우지 않고 의도적으로 남겨둔 carry-over reference 다** (단순 "옛 backend 호환" 보존이 아님). dockview 패널 조립 / RobotsLayout / sidebar / 패널 레지스트리 / 라우팅 / 각종 race fix — "페이지가 어떻게 구성·확장되는가" 의 답이 거기 이미 다 풀려 있다. **default = 그 구조를 가져온다. defer 는 backend module 이 아직 없어서 *지을 수 없는* feature 페이지만.** "최소로 짓고 shell/dockview 는 나중에" 로 판단하면 안 된다 (2026-07-01 그 실수 발생 — §2.3, anchor 1).
>
> **현재 status**: Step F1-F6 (jog + dockview shell) + **RobotCalibrateMode 완료 (2026-07-02)**. jog vertical slice + dockview shell foundation + `CalibrationPanel`(preview traffic light + capture 세션 + active bundle + run history) + registry/sidebar/route. `pnpm lint` 0 + tsc clean + 31 vitest PASS + **Playwright headed calibrate e2e 4/4** (mock+sim: WS연결·bundle / preview toggle / start_run→history / preview 검출(green)+capture accepted). 이후 feature = mode 파일 PANELS + registry snap-in. 구현 상세 = §1-§12, status + 다음 = §15.
>
> **⚠️ calibration = `useService` (boot-query), `useMirror` 아님** — backend 가 Calibration Bundle 을 boot-time configuration 으로 결정 (Mirror 안 씀, [calibration_module_boundary.md §6](calibration_module_boundary.md)). 아래 §7/§11.1 의 "useMirror(CalibrationBundle)" 예시는 **superseded** — CalibrationPanel 은 `useService(CALIBRATION_SNAPSHOT_BUNDLE)` 로 조회. `useMirror` hook 자체는 정의만 유지 (현재 도메인 consumer 0, backend Mirror deferred 와 동일).

## 1. 개요

frontend_v2 = **backend_v2 의 4 primitive (Service / Stream / Event / Mirror) + Capability + HTTP resource 를 React hook 으로 노출**.

새로 짠 게 아니라 **carry over + 새 hook 추가** — 옛 frontend 의 framework layer (`useTopic` / `useService` / `useResource` / `bootstrap`) 는 검증된 패턴이라 그대로 가져왔고, backend_v2 가 도입한 어휘 (Mirror invalidate+refetch / Stream seq invariant / Capability boot-1회 snapshot) 만 새 hook 3개로 추가했다.

## 2. 핵심 원칙

### 2.1 backend_v2 = frontend_v2 SSOT + 계약 타입 생성 (contract.ts)

backend_v2 의 `modules/*/contract.py` (StrEnum 키 + Pydantic 모델) 가 진실 source. frontend 의 [`src/api/generated/contract.ts`](../frontend_v2/src/api/generated/contract.ts) (Topic / ServiceKey / payload interface) 는 여기서 자동 생성 — 손작업 동기화 0.

**경계 (2026-07-01 구현·검증 완료): backend = 계약 EXPORT, frontend = 계약 CONSUME.** `openapi-typescript http://.../openapi.json` 이 떠 있는 서버의 OpenAPI 를 HTTP 로 소비하던 것과 동형 — frontend 빌드 도구는 backend 코드/폴더/python 환경을 **일절 안 건드리고** 떠 있는 서버의 계약을 HTTP fetch 만 한다.

```
backend runtime (전 module 로드 → import/dep 이미 다 해결된 상태)
  │ framework: Runtime.contract_snapshot()  = 로드된 module 의 @service/@subscriber/
  │            @publishes spec 열거 → wire_key→payload (module.py 재import 0)
  │ apps/contract_export.py: build_contract_json(snapshot)
  │            = FRONTEND_EXPOSED 필터 + reachability + name-conflict + ts_type 해소
  │ bridge: GET /contract.json  = 위 JSON serve (relay only, 도메인 로직 0)
  ▼
contract.json  { enums, interfaces, topics, services }   (type 은 이미 TS 문자열로 해소)
  │ HTTP fetch  (frontend Node, backend 접근 0)
  ▼
frontend_v2/scripts/gen-contract.mjs  →  src/api/generated/contract.ts
    (node, 의존성 0. render = 순수 문자열 조립 — JSON schema→TS 재해석 X)
```

`package.json::gen:types = "node scripts/gen-contract.mjs"`. **전제조건: gen 전에 backend 를 전 module 로드하는 host 로 먼저 띄워야 함** (`cd backend_v2 && uv run python -m apps.main --host mock` → bridge :8000). gen 은 그 서버의 `/contract.json` 을 fetch — 안 떠 있으면 스크립트가 명확한 에러 + mock 부팅 안내 후 exit. (이 "backend 를 띄워야 함" 은 running-server 방식의 정직한 대가 — 옛 source-introspection 은 backend 없이 돌았지만 heavy dep 를 gen 머신이 요구했다.)

**백엔드 개발자가 프론트 노출 계약을 적는 유일한 곳 = [`apps/contract_export.py::FRONTEND_EXPOSED`](../backend_v2/apps/contract_export.py)** (opt-in allowlist):
- 노출할 key(enum 멤버)만 나열. **req/res 타입은 안 적음** — module.py `@service` 시그니처에 이미 있고 snapshot 이 제공.
- **contract.py / module.py 는 안 건드림** — 순수 유지 (노출 개념 모름).
- 워크플로우: 서비스는 `@service(key)` 만 짜고, 나중에 "프론트가 명령/구독한다" 결정하는 순간 `FRONTEND_EXPOSED` 에 그 enum 멤버 한 줄 추가.
- 가드: `check_exposed` (EXPOSED ⊆ discovered → 오타/삭제 fail-fast) + incomplete-host (노출 키인데 running runtime 에 payload 없음 → "전 module 로드하는 mock/dev 로 gen 하라" fail-fast).

현재 노출 = **9 key (topic 5 + service 4), interface 18 (도달 14 + HTTP resource 4), enum 2** (참조된 것만). 내부 wire (`Motor.Stream.COMMAND` = Motion→Motor 100Hz 위치명령 등) 는 안 적혀 자동 미노출.

**왜 이 구조인가 (기각된 대안 — 재제안 금지):**
- ❌ **gen 이 module.py import** → torch/open3d/pybullet 등 heavy dep 를 gen 머신이 요구 (분산/스케일 취약). ✅ 떠 있는 서버가 이미 import 끝냈으니 그 결과만 HTTP 로 fetch.
- ❌ **frontend gen 이 `cd ../backend_v2 && uv run python`** → 프론트가 backend 코드/환경에 직접 커플링. ✅ frontend Node 가 HTTP 만.
- ❌ **노출 플래그를 contract.py/module.py 에** (`@service(..., frontend=True)`) → 계약/구현이 프론트 관심사에 오염. ✅ 서버측 allowlist 한 곳.
- ❌ **OpenAPI 그대로** → v2 서비스는 Zenoh RPC 라 HTTP route 없음 + Stream/Event 도 있어 OpenAPI schema 에 안 맞음. ✅ `/contract.json` 커스텀 (services + streams + events).

**구현 파일 + 테스트**:
| 쪽 | 파일 | 검증 |
|---|---|---|
| backend EXPORT | [framework/runtime/snapshot.py](../backend_v2/framework/runtime/snapshot.py) (`ContractSnapshot` + `Runtime.contract_snapshot()`), [apps/contract_export.py](../backend_v2/apps/contract_export.py) (`FRONTEND_EXPOSED` + `build_contract_json`), bridge `GET /contract.json` (+ apps 가 runtime capture 하는 provider closure) | [tests/apps/test_contract_export.py](../backend_v2/tests/apps/test_contract_export.py) — snapshot 열거 / build 필터·reachability·name-conflict / provider wiring / guards / HTTP serve |
| frontend CONSUME | [frontend_v2/scripts/gen-contract.mjs](../frontend_v2/scripts/gen-contract.mjs) (node, 의존성 0) | [src/api/gen-contract.test.ts](../frontend_v2/src/api/gen-contract.test.ts) — render golden (fixture → contract.ts byte-identical) + 구조 조립 |

두 쪽 테스트는 self-contained (서로 참조 0). backend↔frontend end-to-end 정합(backend JSON → contract.ts)은 "mock 띄우고 `pnpm gen:types` → `contract.ts` 안 바뀜" verify 단계가 담당 — 런타임/테스트 커플링 아님.

### 2.2 Carry over > Rewrite (지배 원칙)

옛 frontend 는 **검증된 아키텍처의 살아있는 reference**. frontend_v2 의 default 동작 = "이 구조를 backend_v2 어휘로 포팅". 새로 최소 설계하지 않는다.

carry-over 대상은 hook 뿐 아니라 **UI 구조 전체**:
- **framework layer** — framework/*, bridge.ts, useResource cache
- **app shell** — RobotsLayout (R3F once + Outlet), ModeDockview (generic dockview wrapper + 패널 레지스트리 + 레이아웃 영속화), Sidebar, 라우팅 (`/robots/:id/<mode>`), lib/workspaceLayout
- **panel / scene** — RobotModel ref-stash, Scene/Container, JogJ/JogTcp, RobotStatePanel

**defer 는 "지을 수 없는 것" 만** — backend module 이 아직 없는 feature 페이지/패널 (calibrate/scan/tasks 페이지, scene3d/detection/task 패널). 구조(shell)는 backend 무관이라 지금 가져온다. 재구현/최소설계 = 시간 낭비 + 자산 손실 + 나중에 jog 페이지 rework ([[feedback-developer-focus-business-logic]], [[feedback-port-keep-v2-arch]]).

### 2.3 dockview = 조립 메커니즘(foundation), per-page 사치 아님

⚠️ **2026-07-01 실수 박제 — 다른 세션 주의.** 처음 jog 를 dockview 없이 CSS-grid 단발 MovePage 로 짜고 "dockview = floating/resize UX = 1페이지엔 over-engineering, defer" 로 판단했다. **틀렸다.** 옛 frontend 코드를 읽으면 dockview 는 UX 기능이 아니라 **페이지/패널을 조립하는 메커니즘 그 자체**다:

- [RobotsLayout](../frontend/src/pages/RobotsLayout.tsx) — R3F Canvas 를 한 번만 마운트 + `<Outlet>`.
- 각 mode = [RobotMoveMode](../frontend/src/pages/robotModes/RobotMoveMode.tsx) 처럼 **`PANELS: PanelSpec[]` 배열 하나** (`{id, component, title, w, h}`).
- [ModeDockview](../frontend/src/pages/robotModes/ModeDockview.tsx) — generic wrapper: `PANEL_COMPONENTS` 레지스트리 + floating 배치 + per-mode 레이아웃 영속화 + reset.

즉 **기능 추가 = 패널 컴포넌트 등록 + mode 파일에 PANELS 한 줄.** jog 는 그 메커니즘의 첫 패널(`motion`)이었다. dockview 를 빼면 jog 만 구조적 one-off 가 되고, 나중에 calibration 붙일 때 jog 페이지를 패널로 다시 뜯어야 한다. **dockview shell 은 backend 무관 + 이미 carry-over 가능 (race fix f15a20b 포함) → 지금 가져온다.**

**defer 는 backend module 부재로 지을 수 없는 feature 만** — RobotCalibrateMode/scan/tasks 페이지, Scene3D/Detection/Task 패널, focus-dim/multi-robot 풍부함은 해당 backend 가 박히는 Step E+ 에서 (§10, §11, §15).

### 2.4 Module-based store (방향 — 현재는 미구현)

옛 frontend 의 `domain/stores/*` 는 도메인 별 (calibration / detector / scene3D / system / taskResult) — 옛 backend ApplicationNode 정합. frontend_v2 의 방향은 **backend_v2 module 1:1** (`stores/motor.ts` 등) + cross-module read 는 `useMirror`. **단 first cut 은 store 분리가 불필요** — Move 1 페이지는 `framework/store.ts` 단일 store (topic/service cache) + hook 직접 read 로 충분. 별도 `stores/<module>.ts` 는 Step E+ 에서 cross-module read 가 실제로 생길 때 박는다 (§8).

### 2.5 wire 어휘 1:1

backend_v2 의 `srv/` / `event/` / `stream/` 3 prefix + `Module.Service` / `Module.Event` / `Module.Stream` nested StrEnum 이 frontend 의 `Topic` (stream + event 합침) + `ServiceKey` 두 카테고리에 매핑. 옛 frontend 와 동일 형태.

## 3. 6 hook (Hook layer)

backend_v2 4 primitive (Service / Stream / Event / Mirror) + Capability + HTTP resource = **6 hook**. 진입점 [`framework/index.ts`](../frontend_v2/src/framework/index.ts).

### 3.1 `useService` — RPC call + auto cache

옛 `framework/service.ts` carry over. backend_v2 의 exception model 은 bridge.ts 가 `{success, message, data}` shape 로 shim.

```tsx
const moveJ = useService(ServiceKey.MOTION_MOVE_J);
await moveJ.call({ target_joints: [0, 0, 0, 0, 0, 0] });
```

### 3.2 `useTopic` + `onTopic` — generic latest cache

옛 `framework/topic.ts` carry over. backend_v2 의 stream / event 둘 다 처리, payload `<T>` 자동 typed (`TopicPayloadMap[K]`).

```tsx
const tcp = useTopic(Topic.MOTION_TCP_STATE);
```

단 *generic latest read* — Stream invariant (seq / lag) 검사 X. 그건 `useStream`.

### 3.3 `useStream` — Stream + seq + timestamp invariant ✨ 신규

backend_v2 §8.5 의 Stream payload invariant (`seq: int`, `timestamp_unix: float`) 활용. 구현 = [`framework/stream.ts`](../frontend_v2/src/framework/stream.ts), 상세 §6.

```tsx
const s = useStream(Topic.MOTION_TCP_STATE, { robotId });
// s.value / s.seq / s.lagMs / s.stale / s.outOfOrderCount
```

### 3.4 `useMirror` — Snapshot + change event auto-refetch ✨ 신규

backend_v2 §3.3 Mirror[T] 의 frontend 등가. snapshot service 호출 + change event 도착 시 *payload 안 보고* refetch. 구현 = [`framework/mirror.ts`](../frontend_v2/src/framework/mirror.ts), 상세 §7. **Step E (Calibration backend) 박힐 때 활성** — first cut 에선 hook 만 박고 검증 (§11.1).

```tsx
const cal = useMirror({
  snapshotService: ServiceKey.CALIBRATION_SNAPSHOT_BUNDLE, // Step E+
  changeTopic: Topic.CALIBRATION_ACTIVATED,
  robotId,
});
```

### 3.5 `useCapability` — Boot-1회 snapshot (Mirror 박지 X)

backend_v2 §7 Capability = static fact. boot 1회 read + module-scoped cache (영구). 구현 = [`framework/capability.ts`](../frontend_v2/src/framework/capability.ts).

```tsx
const cap = useCapability(ServiceKey.MOTOR_GET_TOPOLOGY, { robotId });
const armMotors = cap.value?.motors.filter((m) => m.kind === MotorKind.JOINT);
```

### 3.6 `useResource` — HTTP fetch + cache + poll

옛 `framework/resource.ts` carry over. backend_v2 의 `/robots`, `/system` HTTP endpoint 처리.

```tsx
const { data: robots } = useResource<RobotsResponse>("/robots");
```

## 4. 폴더 구조 (F6 후 실제)

**조직 기준 = "새 기능 추가하는 사람의 사고 흐름"** (§4.1). `✓` = 구현됨. `(Step E+)` = backend 부재로 defer.

```
frontend_v2/src/
├── main.tsx, App.tsx              # ✓ App: Sidebar + main + nested route (/robots/:id → RobotsLayout + mode Outlet)
├── api/{bridge.ts ✓, generated/contract.ts ✓}   # backend_v2 wire (gen emit, lint 제외)
├── types/bridge.ts ✓
├── framework/                     # ✓ 6 hook + 인프라
│   ├── store.ts bootstrap.ts service.ts topic.ts resource.ts
│   └── stream.ts mirror.ts capability.ts index.ts
├── hooks/useRobots.ts ✓
├── pages/                          # ★ 라우트 element 만 (App.tsx <Route>). 접미사=타입
│   ├── RobotsLayout.tsx ✓          #   *Layout — /robots/:id (R3F once + meta + <Outlet>)
│   ├── TasksPage.tsx ✓             #   *Page   — 최상위 /tasks (R3F focus=null + dockview)
│   └── robotModes/                 #   *Mode   — /robots/:id/{mode} Outlet 뷰만
│       ├── RobotMoveMode.tsx ✓     #     PANELS=[robotState, motion]
│       ├── RobotCalibrateMode/RobotScanMode/RobotAssetsMode.tsx ✓
│       └── RobotModeRedirect.tsx ✓ #     /robots/:id → 첫 mode redirect
├── components/
│   ├── ui/{button,tabs,slider}.tsx ✓   # shadcn primitive (radix, CLI 생성·regenerate — lint override §12.6)
│   ├── panels/                     # ★ 확장 단위. 새 기능 = 여기 패널 추가 (§4.1)
│   │   ├── registry.ts ✓           # 패널만 import 하는 순수 key→component map
│   │   ├── RobotStatePanel/        # 모든 패널 = 폴더 (§4.1). index.tsx = 등록 패널
│   │   │   └── index.tsx ✓         # useParams self-read + shadcn Button
│   │   └── MotionPanel/            # 복잡 패널 = 서브컴포넌트를 폴더 안 캡슐화
│   │       ├── index.tsx ✓         # useParams → Tabs → control 에 robotId props
│   │       ├── JogJControl.tsx ✓   # 순수 (robotId props) — 단위테스트 대상
│   │       └── JogTcpControl.tsx ✓
│   ├── scene/{Scene,Container,RobotLayer,RobotModel,AxisFrame,sceneOptions}.tsx ✓  # R3F 레이어
│   └── shared/                     # app-shell (라우트 아님, 재사용)
│       ├── Sidebar.tsx ✓           # 앱 nav
│       └── ModeDockview.tsx ✓      # generic dockview shell (registry + 레이아웃 persist) — robot modes + TasksPage 공유
├── constants/index.ts ✓           # WS_URL / BASE_URL / DEFAULT_ROBOT_ID
├── lib/{utils.ts ✓, workspaceLayout.ts ✓}   # cn() + dockview 레이아웃 persist (collapse 는 PanelShell 도입 시)
└── e2e/jog.spec.ts ✓              # L4 Playwright headed (2 PASS)
```

### 4.1 조직 원칙 — panel = 확장 단위 (extension unit)

폴더 기준을 **"새 기능 추가하는 사람이 가장 먼저 찾는 위치"** 로 잡는다.

- **`pages/` = 라우트 element 만** (App.tsx `<Route>` 에 직접 붙음). 접미사로 타입 자명:
  `*Layout`(sub-route 감쌈) / `*Page`(최상위) / `*Mode`(/robots/:id/{mode}). **라우트에 안
  붙는 공유 shell 은 `components/shared`** (ModeDockview·Sidebar), **page+로직+전용 컴포넌트
  완결 feature 는 `features/`** (예: contract-viewer). 판단: "라우트에 붙나 / 재사용 조각인가
  / 완결 feature 인가" → 페이지 늘어도(world/settings…) 흔들림 없음.

패널은 `panels/` 단위로 캡슐화:
- **`panels/` = 확장 단위.** 새 기능 = ① 패널 컴포넌트 만들고 ② registry 등록 ③ mode 파일 PANELS 배치. 끝. (`motor/`·`jog/` 같은 도메인 폴더를 최상위로 두면 "패널인가/재사용인가/어디 넣지" 인지 비용이 먼저 발생 → 안 함.)
- **모든 패널 = 폴더** (`index.tsx` = 등록 패널; 내부 Control/Status 서브컴포넌트는 그 패널만 쓰는 구현 세부라 같은 폴더에 캡슐화). 단일/복잡 무관 **폴더-per-panel 통일** — "파일인가 폴더인가/언제 승격하나" 판단 비용 제거 + 구조 일관 (2026-07-04 결정, 옛 "단순=단일파일, 쪼개질 때 승격" 폐기 — RobotStatePanel 도 폴더화).
- **registry 는 패널만 import 하는 순수 map** — 패널이 useParams(router 의존)를 자체 흡수하므로 wrapper 불필요.
- **router 의존은 패널에서 끝. Control 은 순수 props** — 옛 frontend 는 control 이 useParams 직접 읽어 테스트가 어려웠음. v2 는 패널만 useParams, control 은 robotId props → 단위테스트 쉬움 (옛 구조 복사 X, 문제만 이해해 재해석).
- **`shared/` 는 실제 재사용 생길 때만.** **PanelShell / shadcn 확대는 공통 chrome·디자인 요구가 충분히 쌓일 때** (지금 패널 2개엔 추상화가 구현보다 큼).

설정: `package.json` (+ `dockview` 6.6.1, `lucide-react`, `radix-ui`, shadcn 스택), `components.json` (shadcn, tailwind v4/vite), `<html class="dark">` (shadcn 다크 토큰), `playwright.config.ts` (`headless:false` — §8), `eslint.config.js` (`src/api/generated` + `src/components/ui` 규칙 override).

## 5. 데이터 흐름

### 5.1 Stream subscribe (motor raw state)

```
backend MotorModule.publish(Motor.Stream.RAW_STATE, JointState{robot_id, seq, ts, positions_raw})
  → Zenoh → BridgeModule (WS binary frame type=1, key=stream/motor/<id>/raw_state, msgpack payload)
  → bridge.ts._handleBinary (frame parse + msgpack decode) → topicListeners[wire]
  → bootstrap.ts subscribe loop → useFrameworkStore.setTopicData(wire, jointState)
  → useStream(Topic.MOTOR_RAW_STATE) { value, seq, lagMs, stale, outOfOrderCount }
  → RobotStatePanel (joint table) / RobotModel (URDF joint apply)
```

### 5.2 Service call (MoveJ)

```
useService(ServiceKey.MOTION_MOVE_J).call({ target_joints })
  → bridge.callService (WS JSON {op:"service", key, request_id, data, robot_id})
  → BridgeModule._service (msgpack envelope → transport.call) → Zenoh queryable
  → MotionModule.move_j(req) → resp
  → WS binary frame type=2 (response) / type=3 (error)
  → bridge.ts._handleBinary → pendingServices[request_id] resolve → useService().data 갱신
```

### 5.3 Jog stream publish (50Hz)

```
JogJ hold "+" 버튼 → 50Hz interval → bridge.publish(Topic.MOTION_JOG_J, {robot_id, velocities}, robotId)
  → WS JSON {op:"publish", topic, data} → BridgeModule._publish (msgpack → transport.publish) → Zenoh
  → MotionModule.on_jog_j(JogJInput) → ref latch + dt 적분 + IK → Motor.Stream.COMMAND publish
```

## 6. `useStream` 구현 (seq + lag invariant)

[`framework/stream.ts`](../frontend_v2/src/framework/stream.ts) — backend_v2 §8.5 의 `seq` / `timestamp_unix` 활용:

- **`seq`** — 최신 `value` 에서 *derive* (별도 state X). state 두고 effect 에서 setState 하면 cascading render (react-hooks/set-state-in-effect).
- **`outOfOrderCount`** — effect 가 `lastSeqRef` 와 비교, 역행 시 functional update 로 누적 + `console.warn`. accumulator 라 effect 가 정당.
- **`lagMs` / `stale`** — `Date.now() - timestamp_unix*1000` 을 render 에서 계산. wall-clock 의존이 *의도* (시간 경과 자체가 staleness) 라 `react-hooks/purity` 는 의도적 disable.

invariant 위반 시 fail-fast X — 경고 + state 노출. UI 가 "🟡 lag 1.2s" badge 자연 표시.

## 7. `useMirror` 구현 (invalidate+refetch)

[`framework/mirror.ts`](../frontend_v2/src/framework/mirror.ts) — backend_v2 §3.3.1 (Startup ordering) + §3.3.5 (invalidate+refetch only) 정합:

- **`useBridgeConnected` dep 필수** — WS 미연결 시 callService 가 drop → timeout. `connected=true` 박힌 후 fetch.
- **① mount effect** — connected 면 snapshot fetch. Owner 안 떠 있으면 graceful (cache=null 유지, fail-fast X).
- **② change event effect** — event 도착 시 *payload 안 보고* snapshot 재호출 (invalidate+refetch only).
- `snapshotReq` 는 deps 에서 의도적 제외 — caller 가 inline object 넘기면 매 render identity 바뀌어 refetch loop. snapshot 은 mount/event 시점만.

**Step E (Calibration backend) 박힐 때 실 동작 검증** — first cut 에선 hook + L2 test 만 (§11.1).

## 8. Module store — Step E+ 계획

방향 (§2.4) 은 backend_v2 module 1:1 store + cross-module read = `useMirror`. **first cut 미구현** — Move 페이지는 `framework/store.ts` 단일 cache + `useStream`/`useTopic` 직접 read 로 충분 (motor state 외 cross-module 의존 없음). Step E+ 에서 calibration/scene3d 등 cross-module read 가 생기면 아래 패턴으로 박는다:

```ts
// stores/motor.ts (Step E+ 예시)
export const useMotorStore = create<{ jointStates: Record<string, JointState>; ... }>(...);
onTopic(Topic.MOTOR_RAW_STATE, (s) => useMotorStore.getState().setJointState(s));
```

다른 module store 직접 접근 X — cross-module 은 `useMirror` (backend_v2 §2.4 Database-per-Module 의 frontend 등가).

## 9. 구현 경로

**Step F1-F5 (jog vertical slice) — 완료.** 목적 = hook→wire→mock motor 데이터 경로를 끝까지 한 줄로 검증 (shell 보다 데이터 경로 de-risk 우선). 산출물/검증 §15.1, step 별 test §12.5.

- **F1 Scaffold** — vite + tsconfig + tailwind + main/App + 빈 페이지.
- **F2 framework carry over** — bridge / store / topic / service / resource / bootstrap + contract.ts.
- **F3 새 hook** — stream / mirror / capability.
- **F4 UI** — scene (Scene/Container/RobotLayer/RobotModel/AxisFrame) + jog (JogJ/JogTcp) + RobotStatePanel.
- **F5 MovePage + L4 e2e** — 3-column grid + `/robots/:id/move` route + Playwright headed.

**다음 — Step F6 (dockview shell carry-over) = foundation.** 옛 RobotsLayout + ModeDockview + PANEL_COMPONENTS registry + Sidebar + RobotMoveMode + 라우팅 + lib/workspaceLayout 을 backend_v2 hook/contract 에 맞게 포팅. MovePage 의 패널들을 RobotMoveMode PANELS 로 fold (MovePage 제거). 이후 feature 페이지 = mode 파일 + 패널 등록 = snap-in (§2.3).

## 10. 옛 frontend carry over 자산 인벤토리

| file | 가치 | 변경 |
|---|---|---|
| `api/bridge.ts` | backend_v2 wire (msgpack + binary frame + service shim) | wire 정합 |
| `types/bridge.ts` | WsOp / FrameType / FRAME_VERSION | 그대로 |
| `framework/{store,bootstrap,service,topic,resource}.ts` | Zustand wrap + subscribe loop + useService/useTopic/useResource | 그대로 (bootstrap binary re-attach small fix) |
| `components/scene/RobotModel.tsx` | ref-stash pattern + urdf-loader + loadMeshCb override (cross-robot opacity bleed fix) | 그대로 (commit f15a20b race fix 보존) |
| `components/scene/{Scene,Container}.tsx` | R3F Canvas + TCP frame conversion | carry over |
| `components/panels/motion/JogJ,JogTcp` → `components/jog/{JogJ,JogTcp}.tsx` | 50Hz publish + deadman | stream key 새 contract 키로 rewire + payload `robot_id` + `setPointerCapture` |
| `components/panels/RobotStatePanel` → `components/motor/RobotStatePanel.tsx` | joint state 구독 + table | `Topic.MOTOR_RAW_STATE` rewire |
| `lib/utils.ts` | cn() classname merge | 그대로 |

**shell carry-over (Step F6 — foundation, backend 무관)**: RobotsLayout / ModeDockview / `components/panels/registry` (PANEL_COMPONENTS) / Sidebar / robotModes/{RobotMoveMode, RobotModeRedirect} / 라우팅 / lib/workspaceLayout. dockview workspace + 패널 레지스트리 + mode-based sidebar 가 여기 포함 — 이건 foundation 이지 skip 대상이 아니다 (§2.3).

**Step E+ carry over (backend module 부재로 지금 지을 수 없음)**: `components/panels/calibration/*`, `TaskProgressPanel`, `scene/{TaskResultLayer,DetectionLayer,Scene3DLayer}`, RobotCalibrateMode/scan/tasks 페이지, 도메인 store, focus-dim/multi-robot 풍부함.

## 11. 알려진 risk

### 11.1 Mirror 활성화는 Step E+ 부터

first cut 시점 backend 에 Calibration / Scan / Task module 미박 → Mirror 의 진짜 use case (CalibrationBundle cross-module read) 없음. `useMirror` 는 hook + L2 test 만 박힌 상태 — 실 동작은 Step E backend 시점 검증.

### 11.2 jog publish rate ↔ backend IDLE_RESET 의존 (→ §15.5 robustness)

jog 정확성이 frontend publish rate < 200ms (`_JOG_IDLE_RESET_S`) 에 의존. 메인스레드 stall (느린 머신 / GC / 순간 부하 / headless 렌더) 시 50Hz 가 밀려 backend 가 latch 를 reset → jog 가 조용히 멈춤. L4 e2e 가 headless 에서 이 현상을 노출 (§8). robustness 검토 = §15.5.

### 11.3 dockview shell = Step F6 foundation (완료)

~~옛 dockview 는 over-engineering 이라 defer~~ → **틀린 판단이었음 (§2.3).** dockview = 패널 조립 메커니즘 = foundation. **Step F6 에서 carry-over 완료** — MovePage(CSS-grid 단발) 의 RobotStatePanel/JogJ/JogTcp 를 RobotMoveMode PANELS(robotState, motion) 로 fold, MovePage 제거. 패널 본체는 그대로, 배선만 dockview 로 (fold 로 생긴 rework 는 예상대로 작았음).

### 11.4 옛 domain store lifecycle race

옛 `scene3D.ts` / `calibration.ts` 의 bootstrap/dispose 가 fragile. carry over 시 `useMirror` 로 simplify — mount/unmount 자동 처리 → manual lifecycle X.

## 12. Test 정책 — meaningful tests only

backend_v2 [[feedback-meaningful-tests]] 정합. **모든 test = "spec 의 어느 invariant 검증" 을 docstring 에 `spec frontend_v2.md §X — invariant Y` 로 명시.** 단순 PASS / snapshot / happy-path-only / mock-only-expect 박지 X.

### 12.1 4 계층 검증

| 계층 | tool | 대상 | 진입점 |
|---|---|---|---|
| **L1 lint + type** | `pnpm lint` + `tsc -b` | 모든 file | 매 commit |
| **L2 unit** | Vitest + happy-dom + RTL + mock WebSocket | bridge / hook / panel | 매 PR |
| **L3 single-process e2e** | Vitest + mock bridge | full data flow (publish → store → component) | feature 박을 때 |
| **L4 cross-process e2e** | **Playwright (headed)** + mock backend — `page.mouse` 가 button hold 정상 (pointercancel 없이 800ms hold + full wire raw 변화). `headless:false` 필수 (이유 = §8) | 실 vite dev + 실 backend mock | Step F5 + release |

L1-L3 는 매 PR. L4 는 무거움 — Step F5 + 새 page 추가 시.

### 12.2 진짜 invariant — 박은 test 항목

**bridge.ts**: binary frame parse (type 1/2/3) / msgpack round-trip / service shim type=2 → success / type=3 → error / 5s timeout safety net / reconnect `_resubscribeAll` 재구독.

**useStream**: seq monotonic → outOfOrderCount=0 / seq 역행 → count 증가 + console.warn / timestamp_unix lag detect / stale = lag > staleMs / seq field 없음 → graceful (warn X).

**useMirror**: mount 시 snapshot 1회 / change event → 재호출 (payload 안 봄) / isReady snapshot 후 true / Owner 안 떠 있음 (fail) → cache=null 유지 / unmount cleanup.

**useService**: call 후 cache reactive 갱신 / pending flag timeline.

**useCapability**: boot 1회 fetch / module-cache (re-mount 시 fetch 안 함).

**JogJ / JogTcp**: 50Hz interval publish (hold) / release → interval clear + stop / deadman (blur → stop) / payload `robot_id` 박힘.

### 12.3 박지 말 패턴

snapshot test 만 / mock 자체만 expect / happy path 만 / docstring 없는 단순 PASS.

### 12.4 deps + setup

```
pnpm add -D vitest @testing-library/react @testing-library/dom \
  @testing-library/jest-dom happy-dom mock-socket @playwright/test
```

- `vitest.setup.ts` — `@testing-library/jest-dom`
- `src/**/*.test.{ts,tsx}` — L2 vitest (11 file)
- `e2e/jog.spec.ts` — L4 Playwright
- `playwright.config.ts` — `headless: false` (실 GPU, §8), port 5174
- script: `"test": "vitest run"`, `"test:e2e": "playwright test"`

### 12.5 각 Step 박은 test (F2-F5)

| Step | test |
|---|---|
| F2 | bridge.ts (6), useService (2), useTopic + bootstrap, useResource (cache + poll) |
| F3 | useStream (5), useMirror (5), useCapability (2) |
| F4 | RobotModel ref-stash, JogJ (3), RobotStatePanel (2) |
| F5 | L4 Playwright headed 2 test: ① wait (WS+URDF+RAW_STATE 도착) ② plain `page.mouse` 800ms hold → 50Hz publish (WS send > 20) → backend Motion → mock motor cmd → raw Δ > 20 |

## 13. 인접 문서

- [backend_v2.md](backend_v2.md) — backend SSOT. frontend 어휘 (Mirror / Stream / Capability / seq invariant) origin + Module catalog (§16 — store / component 1:1 매핑)
- [backend_v2_status.md](backend_v2_status.md) — backend 진행 status
- [testing_strategy.md](testing_strategy.md) — backend 4 계층 검증. §12 정합

## 14. 핵심 결정 anchor

| # | 결정 | 위치 | 근거 |
|---|---|---|---|
| 1 | **옛 frontend/ = 의도적 carry-over reference. default = 검증된 구조(shell 포함) 포팅, 새 최소설계 X. defer 는 backend 부재 feature 만** | §2.2 | 지우지 않고 남긴 이유. "최소로 짓고 shell 나중에" 판단 금지 |
| 2 | **contract.py = SSOT. backend `/contract.json` EXPORT + frontend Node gen fetch (backend 접근 0). 노출 = `apps/contract_export.py::FRONTEND_EXPOSED` 한 곳** | §2.1 | [[feedback-ssot-first]] + 프론트↔백 커플링 차단 (2026-07-01 구현) |
| 3 | Carry over = framework + **app shell (RobotsLayout/ModeDockview/registry/Sidebar/라우팅)** + panel/scene — full rewrite X | §2.2 | 검증된 자산 재구현 = 손실 |
| 4 | **dockview = 패널 조립 메커니즘(foundation), per-page 사치 X — Step F6 carry-over** | §2.3 + §11.3 | 2026-07-01 "dockview=사치 defer" 오진 정정 (코드 안 읽고 단정) |
| 5 | Module store 는 Step E+ — first cut framework/store.ts 단일 | §2.4 + §8 | cross-module read 없으면 불필요 |
| 6 | `useStream` 신규 — seq derive + lag invariant | §3.3 + §6 | backend_v2 §8.5 활용 |
| 7 | `useMirror` 신규 — snapshot + invalidate+refetch | §3.4 + §7 | backend_v2 §3.3 등가. Step E 활성 |
| 8 | `useCapability` 신규 — boot 1회 cache | §3.5 | backend_v2 §7 static fact |
| 9 | `useService` 그대로 — exception model 은 bridge.ts shim | §3.1 | 검증된 자산 |
| 10 | DEFAULT_ROBOT_ID = `so101_6dof_0` | §4 | [[project-active-robot-so101-d405]] |
| 11 | RobotModel ref-stash + loadMeshCb override 보존 | §10 | commit f15a20b race fix |
| 12 | meaningful tests only — docstring 에 spec §X 명시 | §12 | [[feedback-meaningful-tests]] |
| 13 | **L4 e2e = Playwright headed 단일** (WebdriverIO 제거) | §8 | "Playwright hold 못 함" 오진 정정 — 측정으로 뒤집힘 |

### 작업 원칙

- 본 문서 = frontend_v2 SSOT. 박힌 결정 의심하지 말고 따를 것.
- backend_v2 어휘 (Mirror / Stream / Event / Capability / seq invariant) hook 1:1 — 새 어휘 X.
- carry over 시 옛 file path + line 추적 (원본 비교).
- 박지 말 패턴: full rewrite from scratch, generator hand-write ([[feedback-ssot-first]] 위반), 한 src 에 옛+새 혼재.

## 15. 현재 status + 다음 세션 handoff

### 15.1 현재 상태 (2026-07-01)

**Step F1-F6 완료 — jog vertical slice + dockview shell + panels 확장단위 구조(§4.1) + shadcn ui/.** `pnpm lint` 0 + `pnpm build` clean + **31 vitest PASS** + **L4 e2e 2 PASS (headed, mock backend + dev server 로 실측)**. F6 에서 **collapse dead-code 버그 발견·수정** (§15.7).

| Step | 산출물 | 검증 |
|---|---|---|
| F1 scaffold | build PASS, 233KB JS (옛 frontend 1.9MB 의 12%) | ✅ |
| F2 framework carry over | bridge/store/topic/service/resource/bootstrap | L2 13 PASS |
| F3 새 hook | stream/capability/mirror | L2 12 PASS |
| F4 UI | scene 5 + jog 2 + RobotStatePanel | L2 5 PASS |
| F5 MovePage + route + L4 e2e | 3-column grid + Playwright headed | 2 PASS |
| F6 dockview shell + 구조 정리 | RobotsLayout + ModeDockview + Sidebar + RobotMoveMode + RobotModeRedirect + workspaceLayout. **panels/ = 확장 단위 재구조화** (§4.1): MotionPanel/ 폴더(index+JogJControl+JogTcpControl) + RobotStatePanel.tsx + 순수 registry.ts. **shadcn 도입** (ui/ = button/tabs/slider, 패널 UI 재작성). MovePage 제거 | **L1 0 + build + L2 31 + L4 2 PASS (headed, 실측)** |

**검증** (cwd `frontend_v2/`):
```powershell
pnpm install
pnpm lint        # 0 problems
pnpm build       # tsc -b && vite build
pnpm test        # 31 PASS
```

**L4 e2e** (mock backend + dev server 사전 띄움):
```powershell
# T1: cd backend_v2; uv run --no-sync python -m apps.main --host mock
# T2: cd frontend_v2; pnpm dev
# T3: cd frontend_v2; pnpm test:e2e   # Playwright headed — 2 PASS
```

### 15.2 L4 e2e 도구 결정 (2026-07-01 — "Playwright hold 못 함" 오진 정정)

이전 세션이 WebdriverIO 로 갈아탄 근거 ("W3C Actions 만 button hold 가능, Playwright CDP Mouse 는 100ms 시점 pointerup auto fire") 는 **측정으로 뒤집힌 오진**.

| | headless (옛 가정) | headed (`headless:false`) |
|---|---|---|
| bare `setInterval(20ms)` (800ms 중) | ~8 (10Hz) | ~54 (50Hz) |
| jog publish 실측 | 7Hz | 47Hz |
| pointercancel | **0** | **0** |
| 모터 raw Δ | 0 (안 움직임) | 88 (움직임) |

- plain `page.mouse.down/up` 은 headless/headed **둘 다 pointercancel 0 + 깨끗한 800ms hold** — 입력 도구 문제 아님.
- 진짜 원인 = headless SwiftShader (소프트웨어 렌더) 가 R3F 3D 씬으로 메인스레드를 굶겨 50Hz jog 가 ~10Hz 로 밀림 → backend IDLE_RESET(0.2s) 아래로 떨어져 적분 안 됨 → 모터 안 움직임.
- WebdriverIO 가 통과한 건 chromedriver default **headed (실 GPU)** 라 메인스레드가 안 굶었기 때문 (W3C Actions 덕 아님).
- **결정: WebdriverIO 제거 (config/test/deps), Playwright 단일 + `playwright.config.ts::headless:false`.**

### 15.3 운영 불변식 (어기지 말 것)

- **L4 e2e = Playwright headed** (`headless:false` 필수 — §15.2).
- **useCapability / useMirror 의 `useBridgeConnected` dep 필수** — WS 미연결 시 callService drop → timeout.
- **JogJ button `setPointerCapture`** — 실 hardware 빠른 손가락 / 누른 채 드래그 시 hit-target 변동 → pointercancel 방어 (오진과 무관한 정당 safeguard).
- **useStream** — seq derive (state X), lag 은 render-시점 wall-clock 의도 (§6).
- eslint 는 `src/api/generated` 제외 (generator = SSOT).

### 15.4 다음 진입점

0. **✅ 계약 타입 생성 재설계 — 구현·검증 완료 (2026-07-01, §2.1).** backend `/contract.json` EXPORT + frontend Node gen (`gen-contract.mjs`) fetch. 옛 source-introspection `gen_contract.py` 삭제, 노출 = `apps/contract_export.py::FRONTEND_EXPOSED`. 상세·기각대안·가드 = **§2.1** (옛 `frontend_contract_gen.md` 는 여기로 통합·삭제). backend 130 test + frontend render golden PASS.
1. **backend bridge ws panic (latent, mid)** — frontend reload/close 시 `backend_v2/modules/bridge/ws.py` 의 `_drain_helper` `AssertionError` (websockets legacy). browser refresh 시 backend restart 필요. fix = ws.py connection cleanup 또는 새 websockets API migrate. (L4 e2e 는 test 당 fresh context 라 이번엔 안 걸렸음 — 실 사용 reload 시 재현.)
2. **✅ 집: 실 SO-101 jog — TCP jog 검증 완료 (2026-07-02, §15.6).** frontend_v2 화면에서 TCP jog → 실 모터 회전 확인. (joint jog / 3D 실데이터는 미확인.)
3. **✅ RobotCalibrateMode 완료 (2026-07-02)** — `CalibrationPanel` (preview traffic light + start/capture/undo/finalize 세션 + active bundle + run history/rollback) + registry/sidebar/route. calibration = `useService` boot-query (Mirror 아님). `e2e/calibrate.spec.ts` Playwright 4/4 (mock+sim, capture-success over-wire 포함).
4. **남은 Step E+ feature 페이지** — Detector / Scene3D / Scan / Reconstruction / Task / Gamepad 박힌 후 그 패널 carry over. Hand-Eye/Intrinsic 세부 패널은 capture UX 심화 시. 새 패널 = `panels/` + registry 한 줄 + mode PANELS 한 줄 snap-in (§4.1, §10).

### 15.5 jog robustness — IDLE_RESET 검토 거리

jog 정확성이 publish rate < 200ms (`_JOG_IDLE_RESET_S`) 에 의존 (§11.2). 메인스레드 stall 시 jog 가 조용히 멈춤 — 실 GPU 머신은 50Hz 라 정상이지만 robustness 결정 거리: backend IDLE_RESET tolerance 완화 vs frontend rate 보장. (지금 fix X — 별도 검토.)

### 15.6 hardware 검증 (집에서) — TCP jog ✅ (2026-07-02)

**검증 완료**: 집에서 frontend_v2 화면 → **TCP jog → 실 SO-101 모터 회전 확인**. C2 wire (frontend→bridge) + Motion JogTcp→IK→feetech command + 토크 enable 이 실 하드웨어에서 동작. Feetech driver 실 통신(register map / sync / signed / clamp)이 TCP jog 경로에서 살아있음 확인됨.

절차:
1. `cd backend_v2; uv run python -m apps.main --host pc` (또는 pi_motor + pi_camera 분산)
2. `cd frontend_v2; pnpm dev`
3. 브라우저 `http://localhost:5174/robots/so101_6dof_0/move`
4. TCP jog hold → 실 SO-101 motor 회전 확인 ✅

미확인/잠재 issue: joint jog / 3D 실데이터 렌더, motors.yaml pid/profile 미적용 (EEPROM default), backend bridge ws panic (reload 시 restart), camera(realsense) 통신.

### 15.7 F6 에서 L4 e2e 로 잡은 버그 (collapse dead-code)

**증상**: dockview 패널이 헤더바(36px)만 남고 접힌 채 렌더 → 콘텐츠 clip → jog "+" 버튼 좌표가 클립 영역이라 hold 가 안 먹음 (L4 test 2: jogSent=2, 기대 >20). L2/build/lint 는 다 통과 → **실제 e2e 를 돌려야 잡히는 버그** (테스트 통과 ≠ 동작).

**root cause**: `workspaceLayout.loadCollapsed` 의 fallback 이 `true` (옛 frontend 는 PanelShell 이 펼치기 토글 제공). F6 에서 collapse helper 만 carry 하고 PanelShell 은 안 가져와 → 패널이 collapsed(header-only)로 생성되는데 펼칠 UI 가 없음 = 반쯤 배선된 dead code.

**fix**: PanelShell 을 지금 안 넣기로 했으니(§4.1) collapse 기계를 제거 — `workspaceLayout` 에서 `loadCollapsed`/`saveCollapsed`/`PANEL_HEADER_HEIGHT`/`COLLAPSED_KEY` 삭제, `ModeDockview` 는 패널을 `p.height` full 로 생성. collapse 는 PanelShell 도입 시 함께. → L4 2 PASS.

**교훈**: carry-over 시 "그 helper 를 구동하는 컴포넌트" 까지 세트로 안 가져오면 dead code 가 남고, 그게 런타임 UI 를 깨뜨린다. 부분 carry 는 그 자체로 검증 대상.
