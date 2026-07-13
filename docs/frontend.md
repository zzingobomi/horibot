# frontend — Frontend 구현 + 진행 status

> backend ([backend.md](backend.md)) 와 1:1 정합되는 React frontend. **frontend 의 일 = 옛 `frontend/` 의 검증된 아키텍처를 backend wire 에 맞게 포팅하는 것** — "뭘 지을지 새로 설계" 가 아니다.
>
> ⚠️ **옛 `frontend/` 는 지우지 않고 의도적으로 남겨둔 carry-over reference 다** (단순 "옛 backend 호환" 보존이 아님). dockview 패널 조립 / RobotsLayout / sidebar / 패널 레지스트리 / 라우팅 / 각종 race fix — "페이지가 어떻게 구성·확장되는가" 의 답이 거기 이미 다 풀려 있다. **default = 그 구조를 가져온다. defer 는 backend module 이 아직 없어서 *지을 수 없는* feature 페이지만.** "최소로 짓고 shell/dockview 는 나중에" 로 판단하면 안 된다 (2026-07-01 그 실수 발생 — §2.3, anchor 1).
>
> **현재 status**: Step F1-F6 (jog + dockview shell) + **RobotCalibrateMode 완료 (2026-07-02)**. jog vertical slice + dockview shell foundation + `CalibrationPanel`(preview traffic light + capture 세션 + active bundle + run history) + registry/sidebar/route. `pnpm lint` 0 + tsc clean + 31 vitest PASS + **Playwright headed calibrate e2e 4/4** (mock+sim: WS연결·bundle / preview toggle / start_run→history / preview 검출(green)+capture accepted). 이후 feature = mode 파일 PANELS + registry snap-in. 구현 상세 = §1-§12, status + 다음 = §15.
>
> **⚠️ calibration = `useService` (boot-query), `useMirror` 아님** — backend 가 Calibration Bundle 을 boot-time configuration 으로 결정 (Mirror 안 씀, [calibration.md §6](calibration.md)). 아래 §7/§11.1 의 "useMirror(CalibrationBundle)" 예시는 **superseded** — CalibrationPanel 은 `useService(CALIBRATION_SNAPSHOT_BUNDLE)` 로 조회. `useMirror` hook 자체는 정의만 유지 (현재 도메인 consumer 0, backend Mirror deferred 와 동일).

## 1. 개요

frontend = **backend 의 4 primitive (Service / Stream / Event / Mirror) + Capability + HTTP resource 를 React hook 으로 노출**.

새로 짠 게 아니라 **carry over + 새 hook 추가** — 옛 frontend 의 framework layer (`useTopic` / `useService` / `useResource` / `bootstrap`) 는 검증된 패턴이라 그대로 가져왔고, backend 가 도입한 어휘 (Mirror invalidate+refetch / Stream seq invariant / Capability boot-1회 snapshot) 만 새 hook 3개로 추가했다.

## 2. 핵심 원칙

### 2.1 backend = frontend SSOT + 계약 타입 생성 (contract.ts)

backend 의 `modules/*/contract.py` (StrEnum 키 + Pydantic 모델) 가 진실 source. frontend 의 [`src/api/generated/contract.ts`](../frontend/src/api/generated/contract.ts) (Topic / ServiceKey / payload interface) 는 여기서 자동 생성 — 손작업 동기화 0.

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
frontend/scripts/gen-contract.mjs  →  src/api/generated/contract.ts
    (node, 의존성 0. render = 순수 문자열 조립 — JSON schema→TS 재해석 X)
```

`package.json::gen:types = "node scripts/gen-contract.mjs"`. **전제조건: gen 전에 backend 를 전 module 로드하는 host 로 먼저 띄워야 함** (`cd backend && uv run python -m apps.main --host mock` → bridge :8000). gen 은 그 서버의 `/contract.json` 을 fetch — 안 떠 있으면 스크립트가 명확한 에러 + mock 부팅 안내 후 exit. (이 "backend 를 띄워야 함" 은 running-server 방식의 정직한 대가 — 옛 source-introspection 은 backend 없이 돌았지만 heavy dep 를 gen 머신이 요구했다.)

**백엔드 개발자가 프론트 노출 계약을 적는 유일한 곳 = [`apps/contract_export.py::FRONTEND_EXPOSED`](../backend/apps/contract_export.py)** (opt-in allowlist):
- 노출할 key(enum 멤버)만 나열. **req/res 타입은 안 적음** — module.py `@service` 시그니처에 이미 있고 snapshot 이 제공.
- **contract.py / module.py 는 안 건드림** — 순수 유지 (노출 개념 모름).
- 워크플로우: 서비스는 `@service(key)` 만 짜고, 나중에 "프론트가 명령/구독한다" 결정하는 순간 `FRONTEND_EXPOSED` 에 그 enum 멤버 한 줄 추가.
- 가드: `check_exposed` (EXPOSED ⊆ discovered → 오타/삭제 fail-fast) + incomplete-host (노출 키인데 running runtime 에 payload 없음 → "전 module 로드하는 mock/dev 로 gen 하라" fail-fast).

현재 노출 = **9 key (topic 5 + service 4), interface 18 (도달 14 + HTTP resource 4), enum 2** (참조된 것만). 내부 wire (`Motor.Stream.COMMAND` = Motion→Motor 100Hz 위치명령 등) 는 안 적혀 자동 미노출.

**왜 이 구조인가 (기각된 대안 — 재제안 금지):**
- ❌ **gen 이 module.py import** → torch/open3d/pybullet 등 heavy dep 를 gen 머신이 요구 (분산/스케일 취약). ✅ 떠 있는 서버가 이미 import 끝냈으니 그 결과만 HTTP 로 fetch.
- ❌ **frontend gen 이 `cd ../backend && uv run python`** → 프론트가 backend 코드/환경에 직접 커플링. ✅ frontend Node 가 HTTP 만.
- ❌ **노출 플래그를 contract.py/module.py 에** (`@service(..., frontend=True)`) → 계약/구현이 프론트 관심사에 오염. ✅ 서버측 allowlist 한 곳.
- ❌ **OpenAPI 그대로** → v2 서비스는 Zenoh RPC 라 HTTP route 없음 + Stream/Event 도 있어 OpenAPI schema 에 안 맞음. ✅ `/contract.json` 커스텀 (services + streams + events).

**구현 파일 + 테스트**:
| 쪽 | 파일 | 검증 |
|---|---|---|
| backend EXPORT | [framework/runtime/snapshot.py](../backend/framework/runtime/snapshot.py) (`ContractSnapshot` + `Runtime.contract_snapshot()`), [apps/contract_export.py](../backend/apps/contract_export.py) (`FRONTEND_EXPOSED` + `build_contract_json`), bridge `GET /contract.json` (+ apps 가 runtime capture 하는 provider closure) | [tests/apps/test_contract_export.py](../backend/tests/apps/test_contract_export.py) — snapshot 열거 / build 필터·reachability·name-conflict / provider wiring / guards / HTTP serve |
| frontend CONSUME | [frontend/scripts/gen-contract.mjs](../frontend/scripts/gen-contract.mjs) (node, 의존성 0) | [src/api/gen-contract.test.ts](../frontend/src/api/gen-contract.test.ts) — render golden (fixture → contract.ts byte-identical) + 구조 조립 |

두 쪽 테스트는 self-contained (서로 참조 0). backend↔frontend end-to-end 정합(backend JSON → contract.ts)은 "mock 띄우고 `pnpm gen:types` → `contract.ts` 안 바뀜" verify 단계가 담당 — 런타임/테스트 커플링 아님.

### 2.2 Carry over > Rewrite (지배 원칙)

옛 frontend 는 **검증된 아키텍처의 살아있는 reference**. frontend 의 default 동작 = "이 구조를 backend 어휘로 포팅". 새로 최소 설계하지 않는다.

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

옛 frontend 의 `domain/stores/*` 는 도메인 별 (calibration / detector / scene3D / system / taskResult) — 옛 backend ApplicationNode 정합. frontend 의 방향은 **backend module 1:1** (`stores/motor.ts` 등) + cross-module read 는 `useMirror`. **단 first cut 은 store 분리가 불필요** — Move 1 페이지는 `framework/store.ts` 단일 store (topic/service cache) + hook 직접 read 로 충분. 별도 `stores/<module>.ts` 는 Step E+ 에서 cross-module read 가 실제로 생길 때 박는다 (§8).

### 2.5 wire 어휘 1:1

backend 의 `srv/` / `event/` / `stream/` 3 prefix + `Module.Service` / `Module.Event` / `Module.Stream` nested StrEnum 이 frontend 의 `Topic` (stream + event 합침) + `ServiceKey` 두 카테고리에 매핑. 옛 frontend 와 동일 형태.

## 3. 6 hook (Hook layer)

backend 4 primitive (Service / Stream / Event / Mirror) + Capability + HTTP resource = **6 hook**. 진입점 [`framework/index.ts`](../frontend/src/framework/index.ts).

### 3.1 `useService` — RPC call + auto cache

옛 `framework/service.ts` carry over. backend 의 exception model 은 bridge.ts 가 `{success, message, data}` shape 로 shim.

```tsx
const moveJ = useService(ServiceKey.MOTION_MOVE_J);
await moveJ.call({ target_joints: [0, 0, 0, 0, 0, 0] });
```

### 3.2 `useTopic` + `onTopic` — generic latest cache

옛 `framework/topic.ts` carry over. backend 의 stream / event 둘 다 처리, payload `<T>` 자동 typed (`TopicPayloadMap[K]`).

```tsx
const tcp = useTopic(Topic.MOTION_TCP_STATE);
```

단 *generic latest read* — Stream invariant (seq / lag) 검사 X. 그건 `useStream`.

### 3.3 `useStream` — Stream + seq + timestamp invariant ✨ 신규

backend §8.5 의 Stream payload invariant (`seq: int`, `timestamp_unix: float`) 활용. 구현 = [`framework/stream.ts`](../frontend/src/framework/stream.ts), 상세 §6.

```tsx
const s = useStream(Topic.MOTION_TCP_STATE, { robotId });
// s.value / s.seq / s.lagMs / s.stale / s.outOfOrderCount
```

### 3.4 `useMirror` — Snapshot + change event auto-refetch ✨ 신규

backend §3.3 Mirror[T] 의 frontend 등가. snapshot service 호출 + change event 도착 시 *payload 안 보고* refetch. 구현 = [`framework/mirror.ts`](../frontend/src/framework/mirror.ts), 상세 §7. **Step E (Calibration backend) 박힐 때 활성** — first cut 에선 hook 만 박고 검증 (§11.1).

```tsx
const cal = useMirror({
  snapshotService: ServiceKey.CALIBRATION_SNAPSHOT_BUNDLE, // Step E+
  changeTopic: Topic.CALIBRATION_ACTIVATED,
  robotId,
});
```

### 3.5 `useCapability` — Boot-1회 snapshot (Mirror 박지 X)

backend §7 Capability = static fact. boot 1회 read + module-scoped cache (영구). 구현 = [`framework/capability.ts`](../frontend/src/framework/capability.ts).

```tsx
const cap = useCapability(ServiceKey.MOTOR_GET_TOPOLOGY, { robotId });
const armMotors = cap.value?.motors.filter((m) => m.kind === MotorKind.JOINT);
```

### 3.6 `useResource` — HTTP fetch + cache + poll

옛 `framework/resource.ts` carry over. backend 의 `/robots`, `/system` HTTP endpoint 처리.

```tsx
const { data: robots } = useResource<RobotsResponse>("/robots");
```

## 4. 폴더 구조 (F6 후 실제)

**조직 기준 = "새 기능 추가하는 사람의 사고 흐름"** (§4.1). `✓` = 구현됨. `(Step E+)` = backend 부재로 defer.

```
frontend/src/
├── main.tsx, App.tsx              # ✓ App: Sidebar + main + nested route (/robots/:id → RobotsLayout + mode Outlet)
├── api/{bridge.ts ✓, generated/contract.ts ✓}   # backend wire (gen emit, lint 제외)
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

[`framework/stream.ts`](../frontend/src/framework/stream.ts) — backend §8.5 의 `seq` / `timestamp_unix` 활용:

- **`seq`** — 최신 `value` 에서 *derive* (별도 state X). state 두고 effect 에서 setState 하면 cascading render (react-hooks/set-state-in-effect).
- **`outOfOrderCount`** — effect 가 `lastSeqRef` 와 비교, 역행 시 functional update 로 누적 + `console.warn`. accumulator 라 effect 가 정당.
- **`lagMs` / `stale`** — `Date.now() - timestamp_unix*1000` 을 render 에서 계산. wall-clock 의존이 *의도* (시간 경과 자체가 staleness) 라 `react-hooks/purity` 는 의도적 disable.

invariant 위반 시 fail-fast X — 경고 + state 노출. UI 가 "🟡 lag 1.2s" badge 자연 표시.

## 7. `useMirror` 구현 (invalidate+refetch)

[`framework/mirror.ts`](../frontend/src/framework/mirror.ts) — backend §3.3.1 (Startup ordering) + §3.3.5 (invalidate+refetch only) 정합:

- **`useBridgeConnected` dep 필수** — WS 미연결 시 callService 가 drop → timeout. `connected=true` 박힌 후 fetch.
- **① mount effect** — connected 면 snapshot fetch. Owner 안 떠 있으면 graceful (cache=null 유지, fail-fast X).
- **② change event effect** — event 도착 시 *payload 안 보고* snapshot 재호출 (invalidate+refetch only).
- `snapshotReq` 는 deps 에서 의도적 제외 — caller 가 inline object 넘기면 매 render identity 바뀌어 refetch loop. snapshot 은 mount/event 시점만.

**Step E (Calibration backend) 박힐 때 실 동작 검증** — first cut 에선 hook + L2 test 만 (§11.1).

## 8. Module store — Step E+ 계획

방향 (§2.4) 은 backend module 1:1 store + cross-module read = `useMirror`. **first cut 미구현** — Move 페이지는 `framework/store.ts` 단일 cache + `useStream`/`useTopic` 직접 read 로 충분 (motor state 외 cross-module 의존 없음). Step E+ 에서 calibration/scene3d 등 cross-module read 가 생기면 아래 패턴으로 박는다:

```ts
// stores/motor.ts (Step E+ 예시)
export const useMotorStore = create<{ jointStates: Record<string, JointState>; ... }>(...);
onTopic(Topic.MOTOR_RAW_STATE, (s) => useMotorStore.getState().setJointState(s));
```

다른 module store 직접 접근 X — cross-module 은 `useMirror` (backend §2.4 Database-per-Module 의 frontend 등가).

## 9. 구현 경로

**Step F1-F5 (jog vertical slice) — 완료.** 목적 = hook→wire→mock motor 데이터 경로를 끝까지 한 줄로 검증 (shell 보다 데이터 경로 de-risk 우선). 산출물/검증 §15.1, step 별 test §12.5.

- **F1 Scaffold** — vite + tsconfig + tailwind + main/App + 빈 페이지.
- **F2 framework carry over** — bridge / store / topic / service / resource / bootstrap + contract.ts.
- **F3 새 hook** — stream / mirror / capability.
- **F4 UI** — scene (Scene/Container/RobotLayer/RobotModel/AxisFrame) + jog (JogJ/JogTcp) + RobotStatePanel.
- **F5 MovePage + L4 e2e** — 3-column grid + `/robots/:id/move` route + Playwright headed.

**다음 — Step F6 (dockview shell carry-over) = foundation.** 옛 RobotsLayout + ModeDockview + PANEL_COMPONENTS registry + Sidebar + RobotMoveMode + 라우팅 + lib/workspaceLayout 을 backend hook/contract 에 맞게 포팅. MovePage 의 패널들을 RobotMoveMode PANELS 로 fold (MovePage 제거). 이후 feature 페이지 = mode 파일 + 패널 등록 = snap-in (§2.3).

## 10. 옛 frontend carry over 자산 인벤토리

| file | 가치 | 변경 |
|---|---|---|
| `api/bridge.ts` | backend wire (msgpack + binary frame + service shim) | wire 정합 |
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

backend [[feedback-meaningful-tests]] 정합. **모든 test = "spec 의 어느 invariant 검증" 을 docstring 에 `spec frontend.md §X — invariant Y` 로 명시.** 단순 PASS / snapshot / happy-path-only / mock-only-expect 박지 X.

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

- [backend.md](backend.md) — backend SSOT. frontend 어휘 (Mirror / Stream / Capability / seq invariant) origin + Module catalog (§16 — store / component 1:1 매핑)
- [backend.md](backend.md) — backend 진행 status
- [dev_reference.md](dev_reference.md) — backend 4 계층 검증. §12 정합

## 14. 핵심 결정 anchor

| # | 결정 | 위치 | 근거 |
|---|---|---|---|
| 1 | **옛 frontend/ = 의도적 carry-over reference. default = 검증된 구조(shell 포함) 포팅, 새 최소설계 X. defer 는 backend 부재 feature 만** | §2.2 | 지우지 않고 남긴 이유. "최소로 짓고 shell 나중에" 판단 금지 |
| 2 | **contract.py = SSOT. backend `/contract.json` EXPORT + frontend Node gen fetch (backend 접근 0). 노출 = `apps/contract_export.py::FRONTEND_EXPOSED` 한 곳** | §2.1 | [[feedback-ssot-first]] + 프론트↔백 커플링 차단 (2026-07-01 구현) |
| 3 | Carry over = framework + **app shell (RobotsLayout/ModeDockview/registry/Sidebar/라우팅)** + panel/scene — full rewrite X | §2.2 | 검증된 자산 재구현 = 손실 |
| 4 | **dockview = 패널 조립 메커니즘(foundation), per-page 사치 X — Step F6 carry-over** | §2.3 + §11.3 | 2026-07-01 "dockview=사치 defer" 오진 정정 (코드 안 읽고 단정) |
| 5 | Module store 는 Step E+ — first cut framework/store.ts 단일 | §2.4 + §8 | cross-module read 없으면 불필요 |
| 6 | `useStream` 신규 — seq derive + lag invariant | §3.3 + §6 | backend §8.5 활용 |
| 7 | `useMirror` 신규 — snapshot + invalidate+refetch | §3.4 + §7 | backend §3.3 등가. Step E 활성 |
| 8 | `useCapability` 신규 — boot 1회 cache | §3.5 | backend §7 static fact |
| 9 | `useService` 그대로 — exception model 은 bridge.ts shim | §3.1 | 검증된 자산 |
| 10 | DEFAULT_ROBOT_ID = `so101_6dof_0` | §4 | [[project-active-robot-so101-d405]] |
| 11 | RobotModel ref-stash + loadMeshCb override 보존 | §10 | commit f15a20b race fix |
| 12 | meaningful tests only — docstring 에 spec §X 명시 | §12 | [[feedback-meaningful-tests]] |
| 13 | **L4 e2e = Playwright headed 단일** (WebdriverIO 제거) | §8 | "Playwright hold 못 함" 오진 정정 — 측정으로 뒤집힘 |

### 작업 원칙

- 본 문서 = frontend SSOT. 박힌 결정 의심하지 말고 따를 것.
- backend 어휘 (Mirror / Stream / Event / Capability / seq invariant) hook 1:1 — 새 어휘 X.
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

**검증** (cwd `frontend/`):
```powershell
pnpm install
pnpm lint        # 0 problems
pnpm build       # tsc -b && vite build
pnpm test        # 31 PASS
```

**L4 e2e** (mock backend + dev server 사전 띄움):
```powershell
# T1: cd backend; uv run --no-sync python -m apps.main --host mock
# T2: cd frontend; pnpm dev
# T3: cd frontend; pnpm test:e2e   # Playwright headed — 2 PASS
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
1. **backend bridge ws panic (latent, mid)** — frontend reload/close 시 `backend/modules/bridge/ws.py` 의 `_drain_helper` `AssertionError` (websockets legacy). browser refresh 시 backend restart 필요. fix = ws.py connection cleanup 또는 새 websockets API migrate. (L4 e2e 는 test 당 fresh context 라 이번엔 안 걸렸음 — 실 사용 reload 시 재현.)
2. **✅ 집: 실 SO-101 jog — TCP jog 검증 완료 (2026-07-02, §15.6).** frontend 화면에서 TCP jog → 실 모터 회전 확인. (joint jog / 3D 실데이터는 미확인.)
3. **✅ RobotCalibrateMode 완료 (2026-07-02)** — `CalibrationPanel` (preview traffic light + start/capture/undo/finalize 세션 + active bundle + run history/rollback) + registry/sidebar/route. calibration = `useService` boot-query (Mirror 아님). `e2e/calibrate.spec.ts` Playwright 4/4 (mock+sim, capture-success over-wire 포함).
4. **남은 Step E+ feature 페이지** — Detector / Scene3D / Scan / Reconstruction / Task / Gamepad 박힌 후 그 패널 carry over. Hand-Eye/Intrinsic 세부 패널은 capture UX 심화 시. 새 패널 = `panels/` + registry 한 줄 + mode PANELS 한 줄 snap-in (§4.1, §10).

### 15.5 jog robustness — IDLE_RESET 검토 거리

jog 정확성이 publish rate < 200ms (`_JOG_IDLE_RESET_S`) 에 의존 (§11.2). 메인스레드 stall 시 jog 가 조용히 멈춤 — 실 GPU 머신은 50Hz 라 정상이지만 robustness 결정 거리: backend IDLE_RESET tolerance 완화 vs frontend rate 보장. (지금 fix X — 별도 검토.)

### 15.6 hardware 검증 (집에서) — TCP jog ✅ (2026-07-02)

**검증 완료**: 집에서 frontend 화면 → **TCP jog → 실 SO-101 모터 회전 확인**. C2 wire (frontend→bridge) + Motion JogTcp→IK→feetech command + 토크 enable 이 실 하드웨어에서 동작. Feetech driver 실 통신(register map / sync / signed / clamp)이 TCP jog 경로에서 살아있음 확인됨.

절차:
1. `cd backend; uv run python -m apps.main --host pc` (또는 pi_motor + pi_camera 분산)
2. `cd frontend; pnpm dev`
3. 브라우저 `http://localhost:5174/robots/so101_6dof_0/move`
4. TCP jog hold → 실 SO-101 motor 회전 확인 ✅

미확인/잠재 issue: joint jog / 3D 실데이터 렌더, motors.yaml pid/profile 미적용 (EEPROM default), backend bridge ws panic (reload 시 restart), camera(realsense) 통신.

### 15.7 F6 에서 L4 e2e 로 잡은 버그 (collapse dead-code)

**증상**: dockview 패널이 헤더바(36px)만 남고 접힌 채 렌더 → 콘텐츠 clip → jog "+" 버튼 좌표가 클립 영역이라 hold 가 안 먹음 (L4 test 2: jogSent=2, 기대 >20). L2/build/lint 는 다 통과 → **실제 e2e 를 돌려야 잡히는 버그** (테스트 통과 ≠ 동작).

**root cause**: `workspaceLayout.loadCollapsed` 의 fallback 이 `true` (옛 frontend 는 PanelShell 이 펼치기 토글 제공). F6 에서 collapse helper 만 carry 하고 PanelShell 은 안 가져와 → 패널이 collapsed(header-only)로 생성되는데 펼칠 UI 가 없음 = 반쯤 배선된 dead code.

**fix**: PanelShell 을 지금 안 넣기로 했으니(§4.1) collapse 기계를 제거 — `workspaceLayout` 에서 `loadCollapsed`/`saveCollapsed`/`PANEL_HEADER_HEIGHT`/`COLLAPSED_KEY` 삭제, `ModeDockview` 는 패널을 `p.height` full 로 생성. collapse 는 PanelShell 도입 시 함께. → L4 2 PASS.

**교훈**: carry-over 시 "그 helper 를 구동하는 컴포넌트" 까지 세트로 안 가져오면 dead code 가 남고, 그게 런타임 UI 를 깨뜨린다. 부분 carry 는 그 자체로 검증 대상.


### 15.8 Scene per-robot 재구성 + LivePointCloud 패널 (2026-07-06)

**계기**: 실 hardware 분산 테스트에서 (a) Tasks 페이지 URDF 가 안 움직임, (b) 라이브 cloud 가 공중에 뜸(사선), (c) cloud 가 v1 보다 엉성함.

**root cause 3건 + fix**:

1. **backend `/robots` 가 robots.yaml spec 위반** — enabled 필터 없이 전 robot + `default = robots[0]` = 비활성 omx_f_0. Tasks(focus=null) 가 죽은 robot 의 stream 을 구독해 전원 freeze. fix = resolve 에서 enabled 필터 + `default: true` flag(생략 시 첫 enabled). `robot/robots.yaml` 에 so101 `default: true` 명시 (omx 재활성화 시 default 안 뒤집힘 — 재활성화 시나리오 test 로 박음).
2. **scene 이 "focus robot 1대 stream 을 전원에 적용" (옛 §4 결정 3 임시 호환)** — N=2 협동 자리 도래로 제거. **RobotLayer 가 robot 마다 자기 `MOTION_TCP_STATE` 구독** (RobotItem), TCP AxisFrame 도 per-robot. Scene3DLayer 는 robotId 만 받아 자립 (자기 tcp stream + hand_eye). base 배치 수학은 `scene/transforms.ts` 로 SSOT 화 (RobotLayer/Scene3DLayer/Container 3 소비자). Container 에서 "특정 robot 의 stream" 개념 소멸 — robot 추가 = yaml 만. 회귀 가드 = `RobotLayer.test.tsx` (두 robot 각자 joints, 단일 stream 공유로 되돌리면 즉시 깨짐).
3. **hand_eye "토글 시 1회 fetch + silent identity fallback"** — 타임아웃(분산 stale queryable 등) 시 identity 로 굳어 cloud 가 TCP 에 매달림. fix = `useMirror` (mount/재연결/CALIBRATION_ACTIVATED 시 자동 refetch).

**LivePointCloudPanel 포팅 (v1 → v2)** — Density radio 1/2/5mm (default 2mm, 사용자 정책 2026-06-21) → `SET_STREAM.voxel_size` 동봉, Point Size slider (mm → material size m). v2 backend `_DEFAULT_VOXEL=5mm` 만 믿던 자리가 엉성함의 원인. live 토글은 ScanPanel 에서 이 패널로 이동 (컨트롤 SSOT 1곳). scanStore 에 voxelSize/pointSize 추가 (현재 single-view — N robot 동시 라이브 필요 시 dict 화 주석).

**같은 날 backend**: graceful shutdown 행 fix — uvicorn `timeout_graceful_shutdown=2` (WS/MJPEG 는 설계상 무한 연결이라 상한 없으면 "Waiting for connections to close" 무한 대기 + 임베딩이라 force_exit 시그널 경로 없음). 회귀 가드 = `test_bridge_shutdown.py` (WS+MJPEG 열어둔 채 stop bounded — inversion 으로 실제 잡힘 증명). main.py "Runtime stopped" 로그 추가.

**검증**: backend tests/apps 48 PASS + frontend vitest 69 PASS + lint/pyright/tsc 0. **hardware 재검증 대기**: 분산 3-머신 재시작 후 (a) Tasks URDF 실시간, (b) cloud 가 책상 면에 앉는지(hand_eye), (c) 2mm density 촘촘함, (d) Ctrl+C 즉시 종료.


---
---

# 부록 — 통합 원문 (2026-07-11 문서 다이어트)

> 아래 문서들을 본 문서 부록으로 병합 (원문 그대로):
> - `frontend.md`
> - `frontend.md`
> - `frontend.md`


---
---

<!-- ═══════════ [통합 원문] frontend.md ═══════════ -->

# frontend.md

frontend 워크스페이스에서 **3D 씬에 시각 표현이 들어가는 방법**의 아키텍처 —
**구현 완료 (2026-07-10)**. 패널 capability 게이팅
([frontend.md](frontend.md) §7) 직후 같은 날
설계·구현·재설계(소유권 모델로 1회 정정)까지.

> 진입 톤: "패널에서 3D에 마커/기하 표시" / "scenePart" / "씬 객체" / "RobotFrame" /
> "frustum 어디서 그리나" / "ghost 미리보기" / "씬 기여 DX" 나오면 본 문서.

**anchor 문장**:

> 프레임워크는 "Scene에 어떻게 참여하는가"를 책임지고, 개발자는 "무엇을 어떻게
> 그릴 것인가"를 책임진다. 좌표계 선택은 렌더링 내용의 일부이므로 프레임워크가
> 숨기지 않는다.

---

## 1. 왜 (문제)

패널(dockview overlay, z-10)과 3D 씬(R3F Canvas, z-0)은 **별도 React 트리**.
기능 하나가 3D 표현을 가지려면 종래엔 4중 편집이 반복됐다: Layer 컴포넌트 신규 +
Scene.tsx 하드코딩 JSX 수정 + store 신규 + `robotBaseMatrix` decompose ~20줄 복붙.
backend framework 원칙과 같은 판정: **반복 보일러플레이트를 제거하는가**
([task.md](task.md) §1) → 승격 대상.

## 2. 소유권 모델 (1차 판정 기준)

씬에 보이는 모든 것은 **"누가 소유하나"** 로 분류된다. 이것이 1차 기준 — "무겁나 /
독립 수명이냐"는 부차적 결과다.

| 부류 | 소유자 | 예 | 폴더 | 등록 |
|---|---|---|---|---|
| **Core chrome** | 씬 자체 | 조명/grid/BASE 축/OrbitControls | [Scene.tsx](../frontend/src/components/scene/Scene.tsx) | 하드코딩 |
| **Scene object** | 세계(하드웨어/산출물) | Robot / **Camera(frustum+cloud)** / ScanMesh | [scene/objects/](../frontend/src/components/scene/objects/) | Scene.tsx 한 줄 (드문 아키텍처 사건) |
| **Feature overlay** | 기능 | TaskMarkersOverlay(topic 수명) / **scenePart**(패널 수명) | [scene/overlays/](../frontend/src/components/scene/overlays/) + 패널 폴더 | registry `scenePart:` 한 줄 |

공용 primitive(RobotFrame/AxisFrame/primitives/transforms)는
[scene/shared/](../frontend/src/components/scene/shared/) — 폴더 구조가 이 표를
그대로 반영.

**판별 질문 (개발자 가이드)**:

> **"패널을 닫으면 이게 사라져야 하나?"**
> - 사라져야 함 (내 기능이 보여주는 것 — ghost 미리보기, 체커보드) → **scenePart**
> - 남아야 함 / 여러 패널이 같은 걸 원함 (세계에 있는 것 — 카메라 frustum) →
>   **씬 객체의 속성** — 패널은 그리지 않고 store 토글만

**씬 객체는 자기 시각 요소를 자기 안에서 그린다.** Camera 가 pose(tcp·hand_eye)를
한 번 계산해 frustum + live cloud 를 자식으로 렌더 — 어느 패널이 몇 개 열리든
렌더는 카메라당 한 번 (중복이 구조적으로 불가). 패널은 `cameraStore.showFrustum` /
`scanStore.liveEnabled` 토글만. 새 객체 종류(미래: world state 의 Box/Conveyor —
backend stream 이 생기면 `<WorldObjects>` 가 data-driven 으로 N 개)는 Scene.tsx 에
한 줄 추가되는 게 정직하다 — **"Scene.tsx diff 0" 계약의 대상은 기능/패널 기여**
(scenePart/토글)이지 객체 종류 추가가 아님.

## 3. scenePart 메커니즘 (기능 오버레이의 패널 수명 형태)

개발자가 쓰는 것 — 패널 폴더에 R3F 조각 + registry 한 줄:

```
panels/WaypointPanel/
  index.tsx      ← React UI ([보기] 토글 버튼)
  scenePart.tsx  ← R3F 조각 (제약 없음 — useFrame/shader/drei 자유)
```

```tsx
// scenePart.tsx — 패널 코드와 같은 멘탈모델
export function WaypointScenePart() {
  const robotId = useRobotId();               // 패널에서 쓰던 그 훅
  const preview = useWaypointStore((s) => s.previews[robotId]);
  if (!preview) return null;
  return <RobotModel jointAngles={preview.jointAngles} opacity={0.35} tint="#34d399" ... />;
}
// registry.ts:  waypoints: { title, ..., scenePart: WaypointScenePart }
```

프레임워크가 해주는 것 (배선 전부):

1. **인스턴스 추적** — [withRobotOwnership](../frontend/src/components/shared/robotOwnership.tsx)
   HOC(chokepoint)가 [panelInstanceStore](../frontend/src/stores/panelInstanceStore.ts)
   에 `(useId, panelKind, robotId)` 등록/해제. **바인딩 + capability OK 일 때만**
   → unsupported robot 이면 scenePart 자동 미표시.
2. **마운트** — Canvas 의 [ScenePartHost](../frontend/src/components/scene/overlays/ScenePartHost.tsx)
   가 인스턴스 × `PANEL_CATALOG.scenePart` 교집합을 인스턴스별 렌더. 같은 패널
   2개(robot A/B)면 조각 2개, 각자 자기 robot.
3. **robot 공급** — 각 조각을 `<RobotProvider>` 로 감쌈 → `useRobotId()`/`useStream`
   패널 그대로.
4. **좌표 primitive** — [RobotFrame](../frontend/src/components/scene/shared/RobotFrame.tsx)
   (robotId 생략 = context 의 자기 robot) + [primitives](../frontend/src/components/scene/shared/primitives.tsx)
   (`<Frame>`/`<Marker>`/`<BoxOutline>`/`<PolyLine>` — 안 쓰면 그만).

**경계를 runtime 에 넘는 것은 인스턴스 목록(순수 데이터)뿐** — scene 컴포넌트는
registry 정적 등록 (identity 안정).

### 데이터 공유 (알고 쓸 것)

scenePart 는 Canvas 트리 렌더 → 패널 로컬 useState 는 안 넘어감. 경로 둘:
framework hook 재구독(useStream/useMirror — module cache, 대부분) / **feature
store**(패널 선택값 — waypointStore/scanStore 패턴). "UI 로 3D 를 제어"(토글류)도
같은 store 경로. 인스턴스-스코프 공유 슬롯은 실 수요 생길 때 도입.

## 4. 좌표계 — 명시적 `<RobotFrame>` (auto-wrap 기각)

scenePart/씬 객체는 world frame 에서 시작, robot base frame 좌표(백엔드 숫자)는
`<RobotFrame>` 으로 명시적으로 감싼다. z-up→y-up + base_pose 수학은
RobotFrame/transforms.ts SSOT. R3F 에서 transform 은 원래 트리에서 명시하는 것
(`<group position>`) — RobotFrame 은 그 문법의 robot 좌표계 버전일 뿐, DSL 아님.

## 5. 기각 결정 (재론 방지 — 근거 포함)

- **descriptor DSL** (`overlay.set([{kind:"frame",...}])`) — 기각. 새로 배울 어휘 +
  표현력 인위적 천장. backend Step/Slot DSL 폐기와 동일 판정.
- **runtime JSX 주입** (tunnel-rat 식) — 기각. Canvas 트리 렌더 시 RobotContext
  단절 footgun + 엘리먼트 identity 를 작성자 규율에 맡김(emitTCP 무한루프 전력,
  commit f15a20b).
- **자동 RobotFrame 래핑** — 기각. mixed-frame(robot 기하 + world 기하 형제)이
  scenePart 단위 opt-out 으로 불가능(표현력 천장) + opt-out flag 는 config DSL
  재발 + 좌표계는 wiring 이 아니라 content 의미론.
- **"여러 패널이 같은 걸 원하면 Tier 1 layer" 규칙** — 기각 (2026-07-10 정정).
  증상(중복 렌더) 기반 규칙 — Robot/Grid 도 여러 패널에서 의미 있으니 Layer 가
  무한증식하는 함정. 올바른 기준은 §2 소유권. **"Layer" 어휘를 도메인 단위로 쓰지
  않는다** — 씬 객체가 조직 단위.
- **render-pass Layer 체계** (Opaque/Transparent/Gizmo) — defer. 엔진 표준이지만
  현재 소비자 0 (렌더 패스/소팅 문제 없음) — 필요가 생길 때.
- **파생 pose 의 DB 저장** (waypoint teach 시점 tcp pose 컬럼) — 기각·revert
  (2026-07-10). pose = joints + 현재 캘의 파생값 — 저장하면 캘 재커밋 시 silent
  stale. scan 의 "raw 만 저장, 파생은 fresh 계산" 원칙과 동일. waypoint 3D 는
  joint-space 데이터에 정직한 **ghost 미리보기**(URDF 재사용, backend 무변경)로.
  cartesian 마커/그룹 polyline 이 필요해지면 그때 motion FK 서비스(fresh 계산) 검토.
- **runtime `registerSceneLayer()` 호출식** — 기각. 정적 선언이 패널 registry 와
  동형, import-side-effect 등록은 의존이 숨음.

## 6. 성능 한계 (명시)

scenePart/primitive 는 React state 경로 — **소량 + 중저빈도** 기하용. 고빈도
대용량(포인트클라우드)은 씬 객체가 dynamic buffer 직접 관리 (Cameras 의
CameraCloud 패턴 — 옛 Scene3DLayer 로직 그대로, Mirror hand_eye 는 "토글 시 1회
fetch 가 identity 로 굳던" 사고의 fix 라 불변).

## 7. 기존 구현의 의도 보존 (이사 기록)

- **Scene3DLayer → Cameras 로 흡수** — cloud 는 "카메라가 보는 센서 데이터"라
  Camera 씬 객체의 자식. base·tcp·handEye pose 계산이 frustum 과 한 곳으로 수렴
  (옛 2중 계산 제거). buffer/구독 로직 불변.
- **scanStore / cameraStore / waypointStore** — 패널 ↔ Canvas 브리지 store 패턴.
  scanStore(liveEnabled/voxel/pointSize/mesh) 유지, cameraStore(showFrustum) 신규,
  waypointStore(ghost preview) 신규.
- **TaskResultLayer → TaskResultsOverlay** — task 기능 소유, topic 수명 (결과는
  패널보다 오래 사는 진단 도구). extractMarkers/렌더 불변.
- **Container 의 scanRobotId/scanBaseMatrix** — 각 객체 안으로 이동 ("대상 robot =
  focus ?? 첫 robot" 은 ScanMesh/TaskResultsOverlay 에, 카메라는 rgbd capability
  파생). N robot 동시 라이브 시 store dict 화 경로는 각 store 주석.
- **sceneOptions** — Core 전용 토글 유지.
- 컴포넌트 rename: RobotLayer→Robots / MeshLayer→ScanMesh /
  TaskResultLayer→TaskResultsOverlay (Layer 어휘 제거).

## 8. 첫 소비자들

- **Camera 씬 객체** ([objects/Cameras.tsx](../frontend/src/components/scene/objects/Cameras.tsx))
  — D405 frustum + live cloud. 처음엔 LivePointCloudPanel 의 scenePart 로 지었다가
  "캘 패널에서도 frustum 보고 싶다"(여러 패널이 같은 것) 요구가 소유권 오류를
  드러내 씬 객체로 승격 — §2 판별 질문의 실전 사례. 캘/라클 패널에 `시야` 토글.
- **WaypointScenePart** ([panels/WaypointPanel/scenePart.tsx](../frontend/src/components/panels/WaypointPanel/scenePart.tsx))
  — scenePart 레퍼런스. waypoint [보기] 버튼(명시 토글, hover X) → 그 joint 자세의
  **반투명 emerald ghost** (RobotModel 재사용 — tint prop). "MoveJ 하면 어떤
  자세가 되나"를 실행 전에 봄. waypoint 는 joint 구성이라 점 마커가 아니라 ghost 가
  정직한 시각화 (팔꿈치 configuration 까지 보임). cartesian 지도/그룹 polyline 은
  후속 (§5 파생 pose 기각 참조).

## 9. 색 시스템 (시각적 의미 SSOT)

씬 색은 **역할(의미)** 로 고정 — hex 를 파일마다 새로 고르면 체계가 조용히
무너지므로 [scene/theme/visualizationColors.ts](../frontend/src/components/scene/theme/visualizationColors.ts)
의 `VizColor` 토큰 한 곳이 SSOT. 새 시각화는 hex 가 아니라 역할을 고른다.

| 토큰 | hex | 의미 | 소비자 |
|---|---|---|---|
| `PREVIEW` | violet `#8b5cf6` | 가상·예측 (command preview / ghost / simulation) | waypoint ghost |
| `SENSOR` | blue `#66ccff` | 센서 계열 | camera frustum, PolyLine default |
| `DETECTION` | emerald `#34d399` | 인식 결과 / attention | task 검출 마커, Marker default |
| `TARGET` | orange `#f59e0b` | 작업 목표 "여기로 가야 한다" | grasp/place 마커, BoxOutline default |
| `TCP` | amber `#ffcc44` | 로봇 기준 프레임 "현재 손 끝" | TCP frame label |
| `CANDIDATE` | gray `#71717a` | 후보 / 비활성 | 검출 후보 |
| (실물) | tint 없음 | real world object | RobotModel 원본 material |

**TARGET ≠ TCP (분리 불변식).** 둘 다 주황 계열이지만 "가야 할 목표(Task/Planning
결과)"와 "현재 로봇 기준점(항상 존재)"은 **동시에 화면에 뜨는 다른 개념** — 같은
색이면 순간 구분 불가라 색으로 갈린다. 합치지 말 것.

**축 색은 이 체계가 아니다.** AxisFrame 의 X=red/Y=green/Z=blue 는 좌표축 관례
(RGB=XYZ)지 의미 색이 아님 — VizColor 에 넣지 않는다. 축에 red 를 썼다고 "warning"
토큰을 끌어오면 두 체계가 무너진다.

**미래 팔레트 (소비자 생기면 토큰 추가 — 지금 코드엔 없음).** 디지털 트윈 확장 시
같은 체계를 그대로: box ghost = PREVIEW, 센서 영역 = SENSOR, collision/constraint =
red/orange(warning), trajectory preview = PREVIEW. 소비자 없는 상수 선제작은 안 함
(§5 원칙) — 문서에 방향만 박고 그때 `VizColor` 에 한 줄.

## 10. 검증

- vitest **144/144** (34 파일) / lint 0 error / `tsc -b` green (2026-07-10):
  panelInstanceStore(5)/RobotFrame(4)/ScenePartHost(4)/Cameras(4)/cameraPose(3)/
  HOC 등록(5)/waypointStore(2)/WaypointScenePart(3)/패널 ghost wire(2) + 기존.
- 픽셀(ghost 색/frustum 위치)은 headed 검증 몫 (jsdom 은 Canvas 를 못 그림).


---
---

<!-- ═══════════ [통합 원문] frontend.md ═══════════ -->

# frontend.md

frontend workspace에서 **robot이 누구의 소유인가** 를 정하는 아키텍처 규칙.
"패널이 robot을 소유한다" 를 SSOT로 하는 불변식(invariant) 문서 — 구현 방법이
아니라 **무엇이 참이어야 하는가** 만 기술한다. 구현체(어떤 hook/직렬화/컴포넌트를
쓸지)는 이 불변식을 만족하는 한 자유롭게 진화할 수 있고, 갈아엎어도 이 문서는
그대로 살아 있어야 한다.

> 진입 톤: "패널이 robot 어떻게 아나" / "route 밖에서 robot-scoped 패널" /
> "robot 셀렉터" / "어느 페이지서든 아무 패널" / "ambient robot" 나오면 본 문서.
> UI 기능(패널 추가/삭제 헤더)은 [frontend.md](frontend.md).

---

## 1. 왜 Route 기반 모델이 한계인가

현재 robot id는 **route에서 ambient로 주입** 된다 — robot-scoped 패널은
`/robots/:id` 라우트 param을 읽어 대상 robot을 안다 (route 밖이면 명시적 throw,
ambient default 없음). 이건 "한 페이지 = 한 robot" 전제 위에선 깔끔했다.

그런데 workspace의 목표가 **"어느 페이지에서든 사용자가 원하는 패널을 띄운다"** 로
올라가면 이 전제가 깨진다:

1. **route가 robot을 안 주는 페이지** (task 페이지, world 등)에 robot-scoped 패널을
   띄우면 대상 robot을 해석할 근거가 없다.
2. `/robots/:id` 안에서도 **모든 패널이 route robot에 강제로 묶여**, 서로 다른
   robot을 나란히 볼 수 없다.

근본 원인: **robot을 navigation 위치(route)에 묶은 것.** 패널이 어느 robot의
데이터를 보여줄지는 navigation 관심사가 아니다. 소유권을 잘못 둔 것이다.

---

## 2. 설계 목표

- 어느 페이지에서든 어떤 패널이든 띄울 수 있다.
- 한 workspace 안에서 서로 다른 robot을 대상으로 하는 패널이 공존할 수 있다.
- robot 바인딩이 **환경 상태(route, 현재 robot 목록, robot 개수)에 흔들리지
  않는다** — 한 번 정해진 패널의 대상은 환경이 바뀌어도 그대로다.

---

## 3. Ownership (소유권)

- **robot은 패널이 소유한다.** 각 패널은 `robot = <id>` 또는 `robot = None` 을
  자기 상태로 가진다.
- **Workspace는 robot을 소유하지 않는다.** Workspace의 책임은 패널 배치·레이아웃·
  저장뿐이다. "현재 robot" 이라는 개념이 Workspace에는 존재하지 않는다.
- **Route는 robot을 소유하지 않는다.** Route는 패널 **생성 시점의 초기값** 만
  제공하고, 그 이후로는 패널의 robot에 관여하지 않는다.

---

## 4. Binding Rules (바인딩)

핵심 불변식:

> **패널이 어느 robot의 데이터를 보여줄지는 오직 그 패널의 `robot` 상태에서
> 결정된다. 환경(route, 현재 robot 목록, robot 개수)은 바인딩에 영향을 주지
> 않는다.**

파생:

- 패널은 **생성 이후 "현재 robot 목록" 을 바인딩 목적으로 다시 참조하지 않는다.**
  자기 `robot` 상태만 본다. robot 목록이 늘거나 줄어도 패널의 대상은 불변.
- **생성 시 초기값 결정** (환경을 읽는 유일한 순간, 그것도 "지금 읽어 고정" 이지
  "매번 읽음" 이 아니다):
  - robot 후보 **0개** → `robot = None`
  - robot 후보 **1개** → 그 robot을 패널 상태에 **기록** (자동 바인딩이 아니라
    route가 초기값을 넣는 것과 동일한 "생성 시 초기값" — 기록 이후엔 환경 무관)
  - robot 후보 **2개 이상** → `robot = None`
- 후보 robot이 있으면 route robot을 우선 초기값으로 쓸 수 있으나, 이는 초기값
  선택 규칙일 뿐이며 바인딩 규칙이 아니다.

---

## 5. Workspace Rules

- **Add = 항상 Spawn.** "추가" 의 의미는 언제나 "새 패널 생성" 이다. 기존 패널을
  앞으로 끌어오는 toggle이 아니다. 이는 robot과 무관한 **Workspace의 공리** —
  robot이 전혀 없는 패널(로그/콘솔 등)도 "추가" 는 곧 "생성" 이어야 자연스럽다.
  robot 소유 모델은 이 공리 위에 얹히는 속성일 뿐이다.
- **레이아웃 저장 시 각 패널의 `robot` 도 함께 저장** 된다. 다시 열면 패널 배치와
  대상 robot이 그대로 복원된다. (환경을 다시 읽어 재계산하지 않는다 — §4 불변식.)

---

## 6. UI Rules

UI는 패널의 `robot` 상태의 **함수** 다 (상태를 바꾸는 원인이 아니라 결과):

- `robot = None` → 패널은 **"Select Robot"** 빈 상태를 보여준다. 임의의 default
  robot 데이터를 조용히 보여주지 않는다.
- `robot = <id>` → 해당 robot의 데이터 뷰.
- **robot 셀렉터는 패널 헤더에 둔다** (본문 아님). robot은 그 패널의 설정이므로
  헤더가 맞고, 여러 패널을 나란히 놨을 때 각 패널이 어느 robot인지 헤더에서 한눈에
  식별된다.
- **robot이 하나뿐이면 셀렉터를 숨긴다** (선택지 하나짜리 picker는 노이즈).

불변식과 UI의 경계 (혼동 방지):

- §4의 "환경을 읽지 않는다" 는 **바인딩에 거는 규칙** 이다.
- **셀렉터(picker)가 옵션을 채우려고 robot 목록을 읽는 것, "N=1이면 숨김" 이
  robot 개수를 읽는 것은 위반이 아니다** — 이는 picker가 제 일을 하는 것이고
  cosmetic affordance일 뿐, 패널의 바인딩(`robot` 상태)을 바꾸지 않는다.

---

## 7. Exception — Task Panel

task 실행에 종속된 패널(프롬프트/진행상황 등)은 robot을 **task 바인딩** 에서
얻는다 — 사용자가 패널마다 고르는 것이 아니다. 한 task의 프롬프트와 진행상황이
같은 robot을 봐야 하기 때문 (패널별로 갈라지면 안 됨). 따라서 **§3~§6의 소유권/
셀렉터 규칙은 task 패널에 적용되지 않는다.** task 패널의 바인딩 단위는 "패널" 이
아니라 "task" 다.

---

## 8. Open Questions

- **Workspace 레이아웃의 저장 범위 — route별 저장 vs 전역 저장.** 현재는 route별
  분리 저장(robot route마다 자기 배치를 기억). 이건 robot 소유 모델과 **직교** —
  두 방식 다 "패널이 robot을 소유" 와 충돌하지 않으므로 여기서 결정하지 않는다.
  Workspace scope를 별도로 설계할 때 결정한다. **그때까지는 현행(route별) 유지.**


---
---

<!-- ═══════════ [통합 원문] frontend.md ═══════════ -->

# frontend.md

frontend 워크스페이스(3D 씬 + dockview 플로팅 패널)의 **패널 관리 UI를
auto-hide 헤더로 재설계** — **구현 완료 (2026-07-09)**. §2/§3 설계 그대로
`AutoHideHeader.tsx` + ModeDockview/robotOwnership/RobotsLayout 반영, Playwright
headed 로 reveal→닫기→추가 검증. §6 튜닝값(threshold/delay/힌트)은 기본값으로
박음 — 실사용하며 손끝 조정 자리. (2026-07-07 논의 → 07-09 구현)

> 진입 톤: "reset layout 버튼 거슬림" / "패널 추가·삭제 안 됨" / "robot id 박스
> 거슬림" / "auto-hide 헤더" / "패널 관리 UI" / "몰입 캔버스" 나오면 본 문서.

---

## 1. 왜 (문제 정의)

현재 dockview workspace에 **패널 관리 계층이 없어서**, 필요한 조작이 전부 3D 씬
위에 떠다니는 *땜빵 플로팅 요소*로 흩어져 있다. 세 불만이 사실 한 뿌리:

1. **Reset layout 버튼** — [ModeDockview.tsx:136-144](../frontend/src/components/shared/ModeDockview.tsx#L136-L144)
   에서 `right-[180px]` 매직넘버로 좌표를 손으로 박은 플로팅 버튼.
2. **robot id/type 박스** — [RobotsLayout.tsx:49-55](../frontend/src/pages/RobotsLayout.tsx#L49-L55)
   의 우상단 플로팅 박스. 씬 위에 상시 떠 있음.
3. **패널 추가/삭제 불가** — [ModeDockview.tsx:51-53](../frontend/src/components/shared/ModeDockview.tsx#L51-L53)
   의 `LockedTab` 이 `hideClose` 로 닫기를 **일부러 막음**. 주석에 이유가 적혀
   있음 — "panel close 후 다시 살리는 UI 가 없어서". 즉 "다시 추가"가 없으니
   "삭제"도 막았고, 그래서 Reset layout 이 유일한 탈출구가 됨.

reference = Grafana/HomeAssistant/k8s dashboard 류 inhouse 웹
([[project_horibot_is_inhouse_web]]). 그런 도구는 예외 없이 workspace 상단
관리 UI + Add Panel + 패널별 메뉴로 이걸 해결.

---

## 2. 무엇 (확정 설계)

**핵심 원칙: 이 UI 의 주인공은 3D 씬. 관리 UI 는 평소 0px, 부를 때만 나타난다.**
"숨기는 게 목적"이 아니라 "필요할 때(마우스가 위로 향하는 의도의 순간) 자연스럽게
나타난다"가 목적.

### 2.1 동작

```
평소 (view 모드)
┌────────────────────────────────────┐
│              ─────────              │  ← 상단 중앙 옅은 힌트(얇은 라인/그라데이션)
│                                    │
│             3D Scene               │
└────────────────────────────────────┘

마우스를 상단으로 → 헤더 슬라이드 다운
┌────────────────────────────────────┐
│                    [+ 패널]   [⋯]  │  ← 우측 정렬 액션
├────────────────────────────────────┤
│             3D Scene               │
└────────────────────────────────────┘

마우스 떠나면 200~500ms 후 다시 사라짐
```

- **트리거 = 상단 전체 (규칙)**, 단 **패널이 차지한 영역에서는 발동 안 함 (예외)**.
  "상단으로 가면 헤더가 나온다"는 자연스러운 규칙을 유지하고, 문제(패널이 상단을
  덮음)는 규칙 변경이 아니라 예외로 처리. (코너-only 트리거는 "왜 오른쪽 끝에만?"
  이라는 학습 규칙을 새로 만들어서 기각 — §5)
- **발견성**: 상단 중앙에 옅은 힌트(얇은 라인 또는 옅은 그라데이션) 하나. 사용자가
  "위에 뭔가 있네" 느끼고 올리면 내려옴.
- **편집중 pin**: 패널 드래그 중 / `+ 패널` 드롭다운·`⋯` 메뉴가 열려 있는 동안은
  헤더를 상시 표시(out-timer 정지). view 모드에서만 auto-hide.

### 2.2 헤더 내용

- `+ 패널 추가 ▾` — 현재 안 떠 있는 등록 패널 목록 드롭다운. 클릭 시 추가.
  목록 소스 = [registry.ts](../frontend/src/components/panels/registry.ts) 의
  `PANEL_COMPONENTS` 에서 "현재 mode 의 PanelSpec 후보 중 미배치" 필터. (mode 별
  후보는 각 robotModes 파일 / TasksPage 의 PANELS 선언 참조)
- `⋯` 메뉴 — "레이아웃 초기화"(현 handleReset). Reset 은 이제 비상용 강등.
- **robot id/type 은 헤더에 넣지 않음 = 완전 제거**. 사이드바에 이미
  `so101_6dof_0` 이 있어 순수 중복 ([RobotsLayout.tsx:49-55](../frontend/src/pages/RobotsLayout.tsx#L49-L55)
  플로팅 박스 삭제).

### 2.3 패널 닫기 활성화

- `LockedTab` 의 `hideClose` **제거** → 패널별 X 로 닫기 가능.
- `+ 패널 추가` 와 **반드시 세트** (닫아도 다시 추가 가능해야 실수 복구됨).

---

## 3. 어떻게 (구현 — 좌표 수학 0)

### 3.1 "패널 위 제외"는 이미 공짜

dockview rect 를 뽑아 매 프레임 비교할 필요 **없음**. 이 workspace 의
pointer-events 정책이 이미 "패널 위 vs 빈 영역"을 브라우저 히트테스트로 구분함
([workspace-dockview.css:3-12](../frontend/src/styles/workspace-dockview.css#L3-L12)):

- dockview 래퍼([ModeDockview.tsx:126](../frontend/src/components/shared/ModeDockview.tsx#L126))
  = `pointer-events: none` → 빈 영역 마우스는 z-0 R3F 캔버스(OrbitControls)로 통과.
- 플로팅 패널만 = `pointer-events: auto` (dockview default `.dv-floating-overlay-host
  > .dv-resize-container`) → 패널 위 마우스는 패널이 잡음.

### 3.2 reveal 로직 (mousemove + elementFromPoint)

show/hide 타이머·pin 상태 때문에 어차피 JS 필요 → 거기에 `elementFromPoint` 한 줄:

```ts
// throttle (rAF 또는 ~16ms)
function onMouseMove(x: number, y: number) {
  if (y < REVEAL_THRESHOLD_PX) {
    const el = document.elementFromPoint(x, y);
    const overPanel = el?.closest(".dv-resize-container"); // 패널 위면 제외
    if (!overPanel) revealHeader();  // 빈 상단 → 헤더 표시
  }
  // 헤더 영역 밖 + 편집중 아님 → out-timer (200~500ms) 로 hide
}
```

- `elementFromPoint` 는 `pointer-events:none` 요소를 자동으로 건너뜀 → 빈 상단이면
  반환값이 캔버스, 패널 위면 `.dv-resize-container`. `closest()` 유무 하나로
  규칙/예외가 갈림.
- **별도 캡처 strip 을 안 만들고 관찰만** → 상단 orbit-drag 밴드를 안 뺏김
  (OrbitControls 그대로 삼).
- lib 커플링은 `.dv-resize-container` 클래스명뿐 — 이미 css 전체가 `.dv-*` 에
  의존 중이라 새 커플링 아님. dockview 버전 업 시 확인 지점 = 이 클래스명.

### 3.3 편집중 pin 배선

- 패널 드래그: dockview `event.api.onDidLayoutChange` / 드래그 이벤트로 감지 →
  드래그 동안 pin.
- 메뉴 열림: `+ 패널` 드롭다운 / `⋯` 팝오버 open state 를 헤더가 들고 있다가,
  열려 있으면 out-timer 정지. (안 하면 드롭다운 항목으로 마우스 내릴 때 헤더가
  사라져 메뉴까지 죽음 — 필수)

---

## 4. 착지 파일 (다른 세션 PnP 영역과 분리)

헤더 작업을 아래로 국한하면 NL PnP 세션(`TasksPage.tsx` / `PromptPanel` /
backend motion·task 편집 중)과 안 부딪힘:

| 파일 | 작업 |
|---|---|
| [ModeDockview.tsx](../frontend/src/components/shared/ModeDockview.tsx) | Reset 플로팅 버튼 제거 → 헤더로 이동, `LockedTab` 의 `hideClose` 제거, 새 헤더 컴포넌트 마운트, `+ 패널 추가` 로직(api.addPanel + 미배치 필터) |
| 새 `AutoHideHeader` 컴포넌트 | reveal/hide 타이머, elementFromPoint 트리거, 힌트, 드롭다운/메뉴, pin |
| [workspace-dockview.css](../frontend/src/styles/workspace-dockview.css) | 헤더/힌트 스타일 |
| [RobotsLayout.tsx](../frontend/src/pages/RobotsLayout.tsx) | 우상단 meta box(49-55) 제거 |

- **`TasksPage.tsx` / `PromptPanel` 은 건드리지 않음** (다른 세션 영역). 헤더는
  `ModeDockview` 공유 wrapper 에 얹으므로 robot mode + tasks 양쪽에 자동 적용됨 —
  `TasksPage.tsx` 수정 불필요.
- git commit/branch 는 다른 세션과 조율 (working tree 엉킴 방지).

---

## 5. 기각·보류 결정 (재론 방지)

- **툴바 막대(full-width)** — 기각. robot id/type 제거 후 헤더에 들어갈 게
  `+ 패널`/`⋯` 2개뿐 → 폭 전체 막대는 chrome 낭비 + 40px 상시 점유. 막대는
  "workspace 액션이 3~4개 이상 상시 필요"할 때만 정당. auto-hide 헤더가 그
  전제를 아예 없앰.
- **우상단 코너-only 트리거** — 기각. 패널 충돌은 줄지만 "왜 오른쪽 끝에만 나오지?"
  라는 학습 규칙을 새로 만듦. "상단=reveal, 패널위=예외"가 더 일관됨. (§3.1 이
  코너 없이도 충돌을 공짜로 해결하므로 차선책 자체가 불필요)
- **최초 1회 헤더 peek(로드시 보여줬다 접기)** — 채택 안 함. 발견성엔 도움 되나
  "UI 가 제멋대로 움직이는" 느낌. 힌트 인디케이터로 발견성 확보하고 peek 는 기본값
  제외. (제품 철학 문제 — 정답 없음, 기본값에서 뺌)
- **command palette(⌘K) / edge dock 트레이** — 후보였으나 auto-hide 헤더로 수렴.
  나중에 워크스페이스 액션이 폭증하면 ⌘K 를 보조로 얹는 건 여전히 열려 있음.
- **도킹 레이아웃(패널이 씬 안 가림) 전환** — 기각. 불만의 본질이 "패널을 내 맘대로
  못 함"(자유도 부족)이라 자유 플로팅 유지가 맞음. 도킹은 자유도를 틀에 가둠.

---

## 6. 튜닝값 (프로토타입에서 손끝으로)

말로 못 정하는 값 — 프로토타입 띄우고 실제 사용 감으로 결정:

- `REVEAL_THRESHOLD_PX` (상단 몇 px 진입 시 reveal): 10~20 후보
- out-delay: 200~500ms 후보
- in-delay(히스테리시스, 스쳐 지나갈 때 안 뜨게): ~120ms 후보 (0 도 시도)
- 힌트 형태: 얇은 라인 vs 옅은 그라데이션

검증 철학: 이 종류 UI 는 말보다 직접 써봐야 답이 남 (프로젝트 L4 headed 검증과
동일 이유). 프로토타입 → 하루 사용 → 확정.

---

## 7. capability 게이팅 (2026-07-10 구현)

"어떤 robot 이 못 여는 패널(예: OMX 는 rgbd 없음 → Scan/Live PointCloud)" 처리.
**숨기지 않고 disabled + 이유**, 열린 패널은 **empty state**, 최종 권한은 백엔드.

### 7.1 SSOT — registry 의 `requiredCapabilities`

패널 → 요구 capability 선언은 [registry.ts](../frontend/src/components/panels/registry.ts)
의 `PANEL_CATALOG` 한 곳 (`scan`/`livePointCloud` → `["rgbd"]`). 선언 없으면 요구
없음 = 항상 활성. capability 어휘는 **robots.yaml SSOT** (프론트 `useRobots()` 노출)
— 프론트에 별도 `RobotCapability` union 을 만들지 않음(어휘 이중화 = drift). 값은
`string[]`; 컴파일타임 안전이 필요하면 백엔드 Pydantic `Literal` 승격 → contract
regen 이 정석.

### 7.2 부족 사유 = capability 에서 파생

문구는 패널마다 손으로 쓰지 않고 [lib/capabilities.ts](../frontend/src/lib/capabilities.ts)
의 `CAPABILITY_LABELS` 에서 조립(`describeMissing`) → `requiredCapabilities` 와 drift
불가 + 다중 요구 자동 조립 + 부족한 그것을 정확히 지목. `unavailableReason` 은
예외적 UX override 자리만 (기본은 파생). registry ↔ robotOwnership 순환 import 를
피하려 helper 는 lib 모듈.

### 7.3 두 소비자 — 헤더 disabled(조건부) + HOC empty state(항상)

- **AutoHideHeader `+ 패널 추가`** — 부족 항목을 🔒 + 사유로 disabled.
  단 **ambient robot(route `:id`)이 있을 때만** 판정. `/tasks`·`/world` 는 focus=null
  이라 "현재 robot" 자체가 없음(robot 은 패널이 소유, [[robot_ownership_model]]) →
  `ambientCapabilities=null` → 아무것도 disable 안 함.
- **`withRobotOwnership`** — 패널이 실제 바인딩한 robot 을 검사하는 **항상-정확한
  1차 방어**. capability 부족이면 "이 robot 에서는 지원하지 않습니다" empty state
  (레이아웃 유지 — 저장된 SO-101 layout 을 OMX 로 열어도 패널을 강제로 없애지 않음).
  `params.robotId` reactive → 탭 셀렉터로 robot 바꾸면 판정도 자동 재계산.
- `requiredCapabilities` 는 **wrap 시점 클로저**로 HOC 에 주입(dockview params 아님)
  — static registry 사실이 localStorage layout 에 영속돼 stale 되는 것 방지.

### 7.4 registry = UI 힌트, 백엔드 = 권한의 원천

`requiredCapabilities` 는 "capability 상 명백히 불가능"(OMX 엔 rgbd 자체가 없음)만
선제 차단하는 **힌트**. robot 이 capability 를 가졌다고 반드시 성공하는 건 아니며
(detector 미실행 / calibration 미로드 등 동적 조건), 최종 판정은 백엔드가 계속 수행
→ 실패는 기존 서비스 에러 메시지 경로로 표면화. 그래서 §2 의 "전체 카탈로그" 철학은
유지된다 — 카탈로그는 전체를 보여주고, **선택 가능 여부만** robot context 가 결정.
