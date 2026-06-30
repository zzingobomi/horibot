# frontend_v2 — Frontend 구현 spec

> backend_v2 ([backend_v2.md](backend_v2.md)) 와 1:1 정합되는 React frontend. 옛 `frontend/` 는 옛 backend 와 호환되도록 두고, `frontend_v2/` 신규 디렉토리에 backend_v2 어휘 (Mirror / Stream / Event / Capability / Stream invariant) 정합 박는다.
>
> 다음 세션 진입 시 frontend 작업 시작점 = 본 문서.

## 1. 개요

frontend_v2 의 목표 = **"backend_v2 의 4 primitive (Service / Stream / Event / Mirror) 를 React hook 으로 그대로 노출 + 옛 frontend 자산 carry over"**.

새로 짜는 것이 아니라 **carry over + 새 hook 추가** — 옛 frontend 의 framework layer (`useTopic`, `useService`, `useResource`, `bootstrap`) 는 검증된 패턴, 그대로 가져온다. backend_v2 가 도입한 어휘 (Mirror invalidate+refetch, Stream seq invariant, Capability boot-1회 snapshot) 만 새 hook 으로 추가.

## 2. 핵심 원칙

### 2.1 backend_v2 = frontend_v2 SSOT

backend_v2 의 `modules/*/contract.py` 가 진실 source. `pnpm gen:types` 가 [`backend_v2/scripts/gen_contract.py`](../backend_v2/scripts/gen_contract.py) 호출 → `frontend_v2/src/api/generated/contract.ts` 자동 emit. 손작업 동기화 0.

### 2.2 Carry over > Rewrite

옛 frontend 의 잘 박힌 코드 (framework/*, bridge.ts 변형, RobotModel ref-stash, urdf-loader 통합, JogJ/JogTcp, useResource cache pattern) 는 **그대로 가져온다**. 재구현 = 시간 낭비 + 자산 손실 ([[feedback-developer-focus-business-logic]]).

### 2.3 First cut = Move 페이지 1개

dockview workspace / focus mode / capability-based mode UI / Calibration / Task / Scene3D UI 다 **first cut 에서 박지 않는다**. Step E+ (Calibration / Scan / Task / Scene3D backend 박힐 때) carry over.

### 2.4 Module-based store

옛 frontend 의 domain/stores/* 는 도메인 별 (calibration / detector / scene3D / system / taskResult) — 옛 backend 의 ApplicationNode 정합. frontend_v2 는 **backend_v2 module 1:1** 정합 — `stores/motor.ts`, `stores/motion.ts`, `stores/camera.ts`, `stores/robot.ts`. cross-module read 는 `useMirror`.

### 2.5 wire 어휘 1:1

backend_v2 의 `srv/` / `event/` / `stream/` 3 prefix 와 `Module.Service` / `Module.Event` / `Module.Stream` nested StrEnum 이 frontend 의 `Topic` (stream + event 합침) + `ServiceKey` 두 카테고리에 매핑. 옛 frontend 와 동일 형태.

## 3. 4 primitive (Hook layer)

backend_v2 의 4 primitive (Service / Stream / Event / Mirror) + Capability + HTTP resource = **6 hook**.

### 3.1 `useService` — RPC call + auto cache

옛 [`framework/service.ts`](../frontend/src/framework/service.ts) 그대로 carry over.

```tsx
const moveJ = useService(ServiceKey.MOTION_MOVE_J);
await moveJ.call({ target_joints: [0, 0, 0, 0, 0, 0] });

const cap = useService(ServiceKey.MOTOR_CAPABILITIES);
const torqueToggle = cap.data?.flags.includes("torque_toggle");
```

backend_v2 의 exception model 은 bridge.ts 가 `{success, message, data}` shape 로 shim (C2b-1).

### 3.2 `useTopic` + `onTopic` — generic topic latest cache

옛 [`framework/topic.ts`](../frontend/src/framework/topic.ts) carry over. backend_v2 의 stream / event 둘 다 처리. payload `<T>` 자동 typed (`TopicPayloadMap[K]`).

```tsx
const tcp = useTopic(Topic.MOTION_TCP_STATE);
```

단 `useTopic` 은 *generic latest read* — backend_v2 의 Stream invariant (seq monotonic / timestamp_unix lag) 검사 X. 그건 `useStream` 자리.

### 3.3 `useStream` — Stream with seq + timestamp invariant ✨ 신규

backend_v2 §8.5 의 Stream payload invariant (`seq: int`, `timestamp_unix: float`) 활용. reconnect / lag / out-of-order detection.

```tsx
const stream = useStream(Topic.MOTOR_RAW_STATE);
// stream.value: JointState | null
// stream.seq: number — monotonic 검증된 seq
// stream.lagMs: number — Date.now() - timestamp_unix*1000
// stream.outOfOrderCount: number — seq 역행 누적
// stream.stale: boolean — lag > threshold (default 500ms)
```

invariant 위반 시 (seq 역행 / lag 임계 초과) `console.warn` + state 노출. UI 가 "🟡 lag 1.2s" badge 자연 표시.

### 3.4 `useMirror` — Snapshot + change event auto-refetch ✨ 신규

backend_v2 §3.3 의 Mirror[T] frontend 등가. snapshot service 호출 + change event 받으면 refetch.

```tsx
const cal = useMirror({
  snapshotService: ServiceKey.CALIBRATION_SNAPSHOT_BUNDLE,
  changeTopic: Topic.CALIBRATION_ACTIVATED,
  robotId: "so101_6dof_0",
});
// cal.value: CalibrationBundle | null
// cal.isReady: boolean
```

backend_v2 invariant (§3.3.5 invalidate+refetch only) 정합 — change event 도착 시 *payload 사용 X*, snapshot 재호출. Step E (Calibration backend) 박힐 때 활성.

### 3.5 `useCapability` — Boot-1회 snapshot (Mirror 박지 X)

backend_v2 §7 의 Capability snapshot. boot 1회 read, 변경 X.

```tsx
const motorCap = useCapability(ServiceKey.MOTOR_CAPABILITIES);
const cameraCap = useCapability(ServiceKey.CAMERA_CAPABILITIES);
if (cameraCap.value?.flags.includes("depth")) showPointCloudPanel();
```

내부 = useService 호출 1회 + module-scoped cache (영구). re-mount 시 cache 활용.

### 3.6 `useResource` — HTTP fetch + cache + poll (carry over)

옛 [`framework/resource.ts`](../frontend/src/framework/resource.ts) 그대로. backend_v2 의 `/robots`, `/system` HTTP endpoint 처리.

```tsx
const { robots } = useResource<RobotsResponse>("/robots").data ?? {};
const { data: metrics } = useResource<SystemMetrics>("/system", { poll: 5000 });
```

## 4. 폴더 구조

```
frontend_v2/
├── package.json              # deps: react 19 + vite + zustand + R3F + urdf-loader + msgpack + dockview (step E+)
├── tsconfig.{json,app,node}.json
├── vite.config.ts            # @/ alias, tailwind plugin
├── index.html
├── src/
│   ├── main.tsx              # React root
│   ├── App.tsx               # BrowserRouter + useFrameworkBootstrap
│   ├── index.css             # tailwind 4
│   ├── api/
│   │   ├── bridge.ts         # backend_v2 wire (C2b-1 결과 그대로)
│   │   └── generated/
│   │       └── contract.ts   # gen_contract.py emit
│   ├── types/
│   │   └── bridge.ts         # WsOp / FrameType / FRAME_VERSION
│   ├── framework/            # 6 hook 자리
│   │   ├── store.ts          # carry over
│   │   ├── bootstrap.ts      # carry over (binary topic re-attach generic 박음)
│   │   ├── service.ts        # carry over (useService)
│   │   ├── topic.ts          # carry over (useTopic / onTopic)
│   │   ├── resource.ts       # carry over (useResource)
│   │   ├── stream.ts         # 신규 (useStream — seq + lag invariant)
│   │   ├── mirror.ts         # 신규 (useMirror — invalidate+refetch)
│   │   ├── capability.ts     # 신규 (useCapability — boot-1회)
│   │   └── index.ts
│   ├── stores/               # backend_v2 module 1:1
│   │   ├── motor.ts          # 신규
│   │   ├── motion.ts         # 신규
│   │   └── robot.ts          # 신규 (defaultRobotId)
│   ├── components/
│   │   ├── jog/
│   │   │   ├── JogJPanel.tsx     # carry over (옛 JogJ.tsx) + 새 stream key + payload.robot_id
│   │   │   └── JogTcpPanel.tsx   # carry over (옛 JogTcp.tsx) + 새 stream key
│   │   ├── motor/
│   │   │   └── RobotStatePanel.tsx   # carry over + 새 Topic.MOTOR_RAW_STATE
│   │   └── scene/
│   │       ├── RobotModel.tsx    # carry over (ref-stash 그대로)
│   │       ├── Scene.tsx         # carry over (R3F + Canvas)
│   │       └── Container.tsx     # carry over (TCP frame conversion)
│   ├── pages/
│   │   └── MovePage.tsx      # CSS grid (dockview skip first cut)
│   ├── constants/
│   │   └── index.ts          # WS_URL / BASE_URL / DEFAULT_ROBOT_ID = "so101_6dof_0"
│   └── lib/                  # utils carry over
└── pnpm-lock.yaml
```

**왜 옛 frontend 와 폴더 형태가 다른가**:
- `domain/stores/` → `stores/` — Module 1:1. *domain* 어휘는 옛 layered architecture 잔재
- `components/panels/*` → `components/<module>/` — Module 별 분리 (Step E+ 박힐 때 `components/calibration/` 등 추가)
- `pages/robotModes/` → 첫 cut 박지 X (Move 1 페이지)

## 5. 데이터 흐름

### 5.1 Stream subscribe (motor raw state)

```
backend_v2 MotorModule
   publish(Motor.Stream.RAW_STATE, JointState(robot_id, seq, ts, positions_raw))
      ↓ Zenoh
   ZenohTransport (Pi → PC)
      ↓ msgpack bytes
   BridgeModule (WS frame, type=1, key=stream/motor/so101_6dof_0/raw_state, payload=msgpack)
      ↓ WS binary
frontend_v2 bridge.ts._handleBinary
   → frame parse + msgpack decode
   → topicListeners[wire].forEach(cb)
      ↓
bootstrap.ts loop (자동 박힌 subscribe)
   → useFrameworkStore.setTopicData(wire, jointState)
      ↓ reactive
useStream(Topic.MOTOR_RAW_STATE)
   → seq monotonic 검사 (역행 시 warn)
   → lag = Date.now() - timestamp_unix*1000
   → { value, seq, lagMs, stale, outOfOrderCount }
      ↓
RobotStatePanel — joint angle 시각화
RobotModel — URDF joint apply
```

### 5.2 Service call (MoveJ)

```
JogJPanel "Home" 클릭
   ↓
useService(ServiceKey.MOTION_MOVE_J).call({ target_joints: home })
   ↓
bridge.callService — WS JSON {op:"service", key, request_id, data, robot_id}
   ↓
BridgeModule._service — msgpack encode envelope {timestamp, data} → transport.call
   ↓ Zenoh queryable
MotionModule.move_j(req: MoveJRequest) → MoveJResponse
   ↓
bridge → WS binary frame type=2 (response) or type=3 (error)
   ↓
bridge.ts._handleBinary
   → pendingServices[request_id] resolve
   → useFrameworkStore.setServiceData(wire, entry)
      ↓ reactive
useService(...).data 갱신
```

### 5.3 Jog stream publish (50Hz)

```
JogJPanel hold "↑" 버튼
   ↓ 50Hz interval
bridge.publish(Topic.MOTION_JOG_J, { robot_id, velocities }, robotId)
   ↓
WS JSON {op:"publish", topic, data}
   ↓
BridgeModule._publish — msgpack encode → transport.publish
   ↓ Zenoh
MotionModule._on_jog_j(event: JogJInput)
   → SE(3) 적분 + IK + Motor.Stream.COMMAND publish
```

## 6. Stream invariant 검사 (`useStream`)

backend_v2 §8.5 의 `seq: int` + `timestamp_unix: float` 활용:

```ts
// framework/stream.ts (sketch)
export function useStream<K extends keyof TopicPayloadMap>(
  topic: K,
  options?: { staleMs?: number },
): {
  value: TopicPayloadMap[K] | null;
  seq: number;
  lagMs: number;
  stale: boolean;
  outOfOrderCount: number;
} {
  const payload = useTopic(topic);
  const stateRef = useRef({ lastSeq: -1, outOfOrder: 0 });

  useEffect(() => {
    if (!payload) return;
    const next = (payload as { seq?: number }).seq;
    if (typeof next !== "number") return;
    if (next < stateRef.current.lastSeq) {
      stateRef.current.outOfOrder++;
      console.warn(`[useStream] ${topic} seq 역행: ${stateRef.current.lastSeq} → ${next}`);
    }
    stateRef.current.lastSeq = next;
  }, [payload, topic]);

  const ts = (payload as { timestamp_unix?: number })?.timestamp_unix;
  const lagMs = ts ? Date.now() - ts * 1000 : 0;
  const stale = lagMs > (options?.staleMs ?? 500);

  return {
    value: payload,
    seq: stateRef.current.lastSeq,
    lagMs,
    stale,
    outOfOrderCount: stateRef.current.outOfOrder,
  };
}
```

invariant 위반 시 *fail-fast 박지 X* — 첫 박을 때 *경고만*. UI 가 stale badge 자연 표시.

## 7. Mirror lifecycle (`useMirror`)

backend_v2 §3.3.1 (Startup ordering) + §3.3.5 (invalidate+refetch only) 정합:

```ts
// framework/mirror.ts (sketch)
export function useMirror<TSnap, TEvent>({
  snapshotService,
  snapshotReq,
  changeTopic,
  robotId,
}: {
  snapshotService: keyof ServiceMap;
  snapshotReq?: object;
  changeTopic: keyof TopicPayloadMap;
  robotId?: string;
}): { value: TSnap | null; isReady: boolean } {
  const [value, setValue] = useState<TSnap | null>(null);
  const event = useTopic(changeTopic, robotId);

  // ① mount 시 snapshot fetch (event 와 무관)
  useEffect(() => {
    fetchSnapshot();
  }, [snapshotService, robotId]);

  // ② event 도착 시 fresh refetch (payload 안 봄)
  useEffect(() => {
    if (event === null) return;
    fetchSnapshot();
  }, [event]);

  async function fetchSnapshot() {
    const res = await bridge.callService(snapshotService, snapshotReq ?? {}, { robotId });
    if (res.success) setValue(res.data as TSnap);
  }

  return { value, isReady: value !== null };
}
```

Step E (Calibration backend) 박힐 때 활성. first cut 박지 X (motion module 자체 Mirror 사용 X — Step F1 시점 calibration 미박).

## 8. Module-based store

backend_v2 module 1:1 정합:

```ts
// stores/motor.ts
export const useMotorStore = create<{
  jointStates: Record<string, JointState>;  // robot_id → state
  setJointState: (state: JointState) => void;
}>((set) => ({
  jointStates: {},
  setJointState: (s) => set((p) => ({
    jointStates: { ...p.jointStates, [s.robot_id]: s },
  })),
}));

// handlers.ts (carry over pattern)
onTopic(Topic.MOTOR_RAW_STATE, (state) => {
  useMotorStore.getState().setJointState(state);
});
```

**다른 module store 접근 X** — cross-module 은 `useMirror`. backend_v2 §2.4 Database-per-Module 의 frontend 등가.

## 9. 도입 단계

각 step 끝 = 검증 가능한 산출물.

### Step F1 — Scaffold (반나절)

`frontend_v2/` 디렉토리 + package.json + tsconfig + vite + tailwind + index.html + main/App.

검증:
- `cd frontend_v2; pnpm install && pnpm build` PASS
- 빈 페이지 띄움 ("Horibot v2")

### Step F2 — Carry over framework + bridge + contract (반나절)

옛 frontend 에서 그대로 가져오기:
- `api/bridge.ts` (C2b-1 결과 그대로)
- `api/generated/contract.ts` (gen_contract.py 호출 path 자리 update)
- `types/bridge.ts`
- `framework/{store,bootstrap,service,topic,resource,index}.ts`
- `constants/index.ts` (DEFAULT_ROBOT_ID = "so101_6dof_0")

backend_v2 mock 띄운 후 — `useResource("/robots")` 호출 → robot list 표시 검증.

### Step F3 — 새 hook (useStream + useMirror + useCapability) (반나절)

`framework/stream.ts` + `framework/mirror.ts` + `framework/capability.ts` 박음. unit-style 검증 (`stores/motor` 의 useStream 으로 seq 검사 박는지).

### Step F4 — Carry over UI (1 day)

옛 frontend 에서:
- `components/scene/RobotModel.tsx` (ref-stash pattern 그대로)
- `components/scene/Scene.tsx` + `Container.tsx`
- `components/panels/motion/JogJ.tsx` → `components/jog/JogJPanel.tsx` (새 Topic.MOTION_JOG_J + payload.robot_id)
- `components/panels/motion/JogTcp.tsx` → `components/jog/JogTcpPanel.tsx`
- `components/panels/RobotStatePanel.tsx` → `components/motor/RobotStatePanel.tsx`
- 옛 `lib/robot/*` carry over (URDF path / FK helper)

### Step F5 — MovePage + 검증 (반나절)

`pages/MovePage.tsx` — CSS grid 3 column (RobotStatePanel | RobotScene3D | JogJ/JogTcp tab). 라우트 `/move` 또는 `/robots/:id/move`.

검증:
- `cd backend_v2; uv run python -m apps.main --host mock`
- `cd frontend_v2; pnpm dev`
- 브라우저 `:5173/move` — mock motor 의 joint state stream 보이고 jog 누르면 mock motor 움직임
- jog 명령 정합 (`MOTION_JOG_J` stream 50Hz publish → mock motion subscribe → mock motor command publish)

**Total ≈ 2-3 day** (옛 frontend 의 한 주 vs framework rewrite from scratch 의 한 주 사이).

## 10. 옛 frontend 에서 carry over 자산 인벤토리

| file | 가치 | 변경 |
|---|---|---|
| `api/bridge.ts` | backend_v2 wire 정합 박힌 결과 | 그대로 |
| `types/bridge.ts` | WsOp / FrameType / FRAME_VERSION | 그대로 |
| `framework/store.ts` | Zustand wrap 최소 surface | 그대로 |
| `framework/bootstrap.ts` | robot-scoped subscribe loop + binary re-attach race fix | 그대로 (binary re-attach generic 박는 자리 small fix) |
| `framework/service.ts` | useService + auto cache | 그대로 |
| `framework/topic.ts` | useTopic + onTopic | 그대로 |
| `framework/resource.ts` | HTTP fetch + module cache + poll + select | 그대로 |
| `components/scene/RobotModel.tsx` | ref-stash pattern + urdf-loader integration + loadMeshCb override (cross-robot opacity bleed fix) | 그대로 (commit f15a20b 의 race fix 보존) |
| `components/scene/Scene.tsx` + `Container.tsx` | R3F Canvas + TCP frame conversion | 그대로 |
| `components/panels/motion/JogJ.tsx` + `JogTcp.tsx` | 50Hz interval publish + deadman pattern | stream key 자리 새 contract.ts 키로 rewire + payload 에 `robot_id` 박음 |
| `components/panels/RobotStatePanel.tsx` | joint state subscribe + table | `Topic.MOTOR_RAW_STATE` 로 rewire |
| `lib/robot/utils.ts` | raw↔rad helper | 그대로 |
| `lib/utils.ts` | cn() classname merge | 그대로 |

**Skip first cut (Step E+ 박힐 때 carry over)**:
- `components/panels/calibration/*` (CaptureGuideOverlay / PoseList / CalibrationPanel / IntrinsicPanel / CameraPanel)
- `components/panels/TaskProgressPanel.tsx`
- `components/scene/TaskResultLayer.tsx` (Detection / Position3 auto-dispatch)
- `components/scene/DetectionLayer.tsx`
- `components/scene/Scene3DLayer.tsx`
- `domain/stores/*` 의 도메인 별 store 자리 — 새 backend_v2 module 1:1 store 로 재배치
- `dockview` workspace + focus mode + mode-based sidebar — Step E+ 박힐 때

## 11. 알려진 risk

### 11.1 backend_v2 의 Mirror 활성화는 Step E+ 부터

frontend_v2 첫 cut (Step F1-F5) 박을 때 backend_v2 에는 *Calibration / Scan / Task module 미박*. Mirror 의 진짜 use case (CalibrationBundle 의 cross-module read) 자체 없음. `useMirror` 박은 후 *Step E backend* 시점 검증 — 박은 hook 의 실 동작 그때 확인.

### 11.2 JogJ.tsx 의 idle-reset race

옛 [`JogJ.tsx:53-61`](../frontend/src/components/panels/motion/JogJ.tsx) 의 stopJog 가 50Hz interval clear 박지만 backend `IDLE_RESET_S=0.2s` 의 fresh latch 메커니즘 frontend 가 모름. 재연결 mid-jog 시 옛 velocity 계속 publish 가능. 검증 시 reconnect 시나리오 확인 — issue 박혔으면 `useStream` 의 stale flag 활용 가능.

### 11.3 dockview 폐기 X — Step E+ carry over

옛 dockview workspace 동작 잘 박음 (사용자 자유 drag/resize). first cut 박지 않는 이유 = *over-engineering 회피*. Step E+ 박힐 때 carry over.

### 11.4 옛 frontend domain store 의 lifecycle race

옛 [`scene3D.ts`](../frontend/src/domain/stores/scene3D.ts) + [`calibration.ts`](../frontend/src/domain/stores/calibration.ts) 의 bootstrap/dispose pattern 이 fragile. carry over 시 *Mirror 정합으로 simplify* — `useMirror` 가 mount/unmount 자동 처리 → manual lifecycle X.

## 12. Test 정책 — meaningful tests only

backend_v2 의 [[feedback-meaningful-tests]] 정합. **모든 test = "spec 의 어느 invariant 검증" 명시** — docstring 에 `// spec frontend_v2.md §X — invariant Y` 박음. 단순 PASS 박는 test 박지 X.

### 12.1 4 계층 검증

[testing_strategy.md](testing_strategy.md) (backend) 와 정합:

| 계층 | tool | 대상 | 진입점 |
|---|---|---|---|
| **L1 lint + type** | `pnpm lint` + `tsc -b` | 모든 file | 매 commit |
| **L2 unit + 합성 회귀 차단** | Vitest + happy-dom + RTL + mock WebSocket | bridge / hook / panel | 매 PR |
| **L3 single-process e2e** | Vitest + mock bridge | full data flow (publish → store → component reactive) | feature 박을 때 |
| **L4 cross-process e2e** | **WebdriverIO** + mock backend (host=mock) — W3C Actions API 가 진짜 button hold 지원. Playwright 는 CDP Mouse cascade 때문에 ~100ms 시점 pointerup auto fire → button hold 시나리오 안 됨 (2026-07-01 confirmed). Playwright 는 단순 wait 시나리오 (WS+URDF+state 도착) 만. | 실 frontend vite dev + 실 backend_v2 mock | Step F5 + release |

L1-L3 는 매 PR. L4 는 무거움 — Step F5 (jog mock backend 검증) 박을 때 + 새 page 추가 시.

### 12.2 진짜 invariant — 박을 test 항목

각 항목 = 단일 PASS 박는 자리 X. *spec 의 어느 invariant* 박는지 docstring 명시.

**bridge.ts** (transport layer):

| invariant | 검증 |
|---|---|
| binary frame parse (type 1/2/3) | mock `[u8 v=1][u8 t][u16 BE key_len][key][payload]` 박은 후 `_handleBinary` 의 dispatch 확인 |
| msgpack encode-decode round-trip | publish 박은 `data` 가 server-side decode 결과 동일 (`{ robot_id, seq, timestamp_unix }`) |
| service shim — type=2 → `{success:true, data:env.data}` | mock service response 도착 시 callService Promise resolve |
| service shim — type=3 → `{success:false, message:type:msg}` | mock service error 도착 시 callService Promise resolve with error |
| timeout safety net — backend default 박혀있어도 frontend 5s timeout | timeout exceed 시 callService Promise resolve fail |
| reconnect — `_resubscribeAll` 박는 자리 모든 topicListeners + binaryTopicListeners 재구독 | close → open 박은 mock 자리 subscribe send 두 번 박힘 |

**useStream** (seq + lag invariant):

| invariant | 검증 |
|---|---|
| seq monotonic 정상 | seq 1, 2, 3 도착 후 outOfOrderCount=0 |
| seq 역행 → outOfOrderCount 증가 + console.warn | seq 1, 2, 1 도착 후 outOfOrderCount=1 |
| timestamp_unix lag detect | timestamp_unix = now - 1.0 도착 시 lagMs ≈ 1000 |
| stale flag — lagMs > staleMs (default 500) | timestamp_unix = now - 0.7 도착 시 stale=true |
| seq field 박지 X — graceful (warn X, skip) | payload `{a, b}` 도착해도 outOfOrderCount=0, lagMs=0 |

**useMirror** (invalidate+refetch invariant — backend_v2 §3.3.5):

| invariant | 검증 |
|---|---|
| mount 시 snapshot fetch | mock service spy 가 mount 후 1회 호출 |
| change event 도착 → snapshot 재호출 (payload 박지 X) | mock event publish 후 service spy 2회 호출, event payload 사용 안 함 |
| isReady — snapshot 받은 후 true | mount 직후 isReady=false → service resolve 후 isReady=true |
| Owner 안 떠 있음 (snapshot fail) → cache=null + isReady=false 유지 | mock service reject 후 isReady=false 유지 |
| unmount — subscription cleanup | unmount 후 mock listener count=0 |

**useService**:

| invariant | 검증 |
|---|---|
| call 후 cache reactive 갱신 | call 후 reactive view (data / success / pending) 갱신 |
| pending flag — call 호출 직후 true, response 후 false | RTL spy 로 timeline 확인 |

**useCapability**:

| invariant | 검증 |
|---|---|
| boot 1회 fetch | mount 시 service spy 1회 |
| module-cache — re-mount 시 fetch 안 함 | unmount + 다시 mount 후 service spy 1회 유지 |

**JogJPanel / JogTcpPanel**:

| invariant | 검증 |
|---|---|
| 50Hz interval publish (button hold) | fake timer + RTL pointerDown 후 100ms 안 5회 publish |
| release (pointerUp) — interval clear + stop 명령 | fake timer 로 pointerUp 후 publish 안 함 검증 |
| deadman — focus lost (blur) → stop 자동 | window blur 후 jog publish 안 함 |
| payload `robot_id` 박힘 | publish spy 의 args[1].robot_id === DEFAULT_ROBOT_ID |

### 12.3 박지 말 패턴

- **snapshot test 만 박는 자리** — UI 의 markup 자리 lock — invariant 박지 X
- **mock 자체만 검증** — mock 의 expect 만 박은 test — 실 코드 동작 안 잡음
- **happy path 만 박는 test** — error / race / out-of-order 박는 게 진짜 가치
- **단순 PASS 박는 test** — docstring 없이 PASS 만 박혀있으면 spec 의 어느 invariant 검증인지 모름

### 12.4 박을 deps + 첫 setup

```
pnpm add -D vitest @testing-library/react @testing-library/dom \
  @testing-library/jest-dom happy-dom mock-socket \
  @playwright/test \
  @wdio/cli @wdio/local-runner @wdio/mocha-framework @wdio/spec-reporter \
  @wdio/types webdriverio @types/mocha ts-node
```

- `vitest.config.ts` — happy-dom env + setup file
- `vitest.setup.ts` — `@testing-library/jest-dom` import
- `src/__tests__/` — test file (L2 vitest)
- `e2e/` — Playwright spec (L4, 단순 wait 시나리오 만)
- `e2e_wdio/` — WebdriverIO test (L4, button hold 진짜 검증)
- `playwright.config.ts` — hasTouch:true, port 5174
- `wdio.conf.ts` — chromedriver + W3C Actions
- `tsconfig.wdio.json` — wdio + mocha types
- `package.json` script:
  - `"test": "vitest run"`, `"test:watch": "vitest"`
  - `"test:e2e": "playwright test"` (simple flows)
  - `"test:e2e:wdio": "wdio run wdio.conf.ts"` (real button hold)

### 12.5 각 Step 박을 test (Step F2-F5)

| Step | 박을 test |
|---|---|
| F2 (carry over framework + bridge) | bridge.ts (6 invariant), useService (2), useTopic + bootstrap (subscribe-on-mount 자리), useResource (cache + poll) |
| F3 (새 hook) | useStream (5), useMirror (5), useCapability (2) |
| F4 (carry over UI) | RobotModel ref-stash (race fix invariant), JogJPanel (4), JogTcpPanel (4), RobotStatePanel (joint state 표시) |
| F5 (MovePage + e2e) | L4 — Playwright (단순 wait — WS+URDF+state stream 도착) + WebdriverIO (W3C Actions 가 진짜 button 800ms hold → 50Hz publish → backend Motion → mock motor cmd → raw 변화) |

## 13. 인접 문서

- [backend_v2.md](backend_v2.md) — backend SSOT. frontend_v2 의 모든 어휘 (Mirror / Stream / Capability / seq invariant) 의 origin
- [backend_v2_modules.md](backend_v2_modules.md) — Module catalog. frontend_v2 의 store / component 1:1 매핑
- [backend_v2_status.md](backend_v2_status.md) — backend 진행 status. frontend 의 Step E+ 시점 결정에 활용
- [frontend_v2_status.md](frontend_v2_status.md) — frontend_v2 현재 진행 status + 다음 세션 handoff
- [testing_strategy.md](testing_strategy.md) — backend 4 계층 검증 정책. frontend §12 정합

## 14. 핵심 결정 anchor

| # | 결정 | 위치 | 근거 |
|---|---|---|---|
| 1 | frontend_v2/ 신규 (frontend/ 그대로 — backend/ + backend_v2/ 와 같은 pattern) | §1 | 옛 frontend = 옛 backend 호환 보존, 진화 명확 |
| 2 | gen_contract.py = SSOT (Python contract.py introspect → TS emit) | §2.1 | [[feedback-ssot-first]] + [[feedback-developer-focus-business-logic]] |
| 3 | Carry over framework/* (useTopic/useService/useResource) — full rewrite 박지 X | §2.2 | 옛 코드 검증된 자산 — 재구현 = 시간 낭비 + 자산 손실 |
| 4 | first cut = Move 페이지 1개 (dockview / focus mode / Calibration UI 박지 X) | §2.3 | over-engineering 회피, Step E+ 박힐 때 carry over |
| 5 | Module-based store (`stores/motor.ts` / `stores/motion.ts` 등) — backend_v2 module 1:1 | §2.4 + §8 | backend_v2 §2.4 Database-per-Module 정합 |
| 6 | `useStream` 신규 — seq monotonic + timestamp_unix lag invariant 검사 | §3.3 + §6 | backend_v2 §8.5 stream payload invariant 활용. reconnect/lag/out-of-order detection unbuilt 부분 fix |
| 7 | `useMirror` 신규 — snapshot service + change event invalidate+refetch | §3.4 + §7 | backend_v2 §3.3 Mirror[T] frontend 등가. Step E (Calibration) 박힐 때 활성 |
| 8 | `useCapability` 신규 — boot 1회 snapshot cache (Mirror 박지 X) | §3.5 | backend_v2 §7 capability = static fact. invalidation cycle 박지 X |
| 9 | `useService` 그대로 — backend_v2 의 exception model 은 bridge.ts shim | §3.1 | 옛 framework/service.ts 잘 박힌 자리. C2b-1 의 shim 으로 호환 |
| 10 | dockview = Step E+ (first cut CSS grid) | §2.3 + §11.3 | jog 1 페이지 박는 자리 over-engineering. Step E+ 박힐 때 carry over |
| 11 | DEFAULT_ROBOT_ID = `so101_6dof_0` (옛 omx_f_0 X) | §4 | [[project-active-robot-so101-d405]] 정합 |
| 12 | RobotModel 의 ref-stash pattern + loadMeshCb override 보존 | §10 | commit f15a20b 의 race fix + cross-robot opacity bleed fix. 재구현 = 2일 손실 |
| 13 | meaningful tests only — 모든 test = spec invariant 검증, docstring 명시 | §12 | [[feedback-meaningful-tests]] 정합. backend_v2 의 67 PASS 가 모두 spec ref + invariant 명시 — 단순 PASS 박는 test 박지 않음. snapshot test / happy path 만 / mock 만 expect 박지 않음 |

### 작업 원칙

- 본 문서 = frontend_v2 spec SSOT. 박힌 결정 (위 12개) 의심하지 말고 따를 것.
- backend_v2 의 어휘 (Mirror / Stream / Event / Capability / seq invariant) frontend hook 으로 1:1 노출 — 새 어휘 박지 X.
- 옛 frontend 에서 carry over 박을 때 *원본 비교* 자리 박을 것 — 옛 file path + line 추적.
- Step F1-F5 순차. 점프 X. 각 step 끝 = build clean + 검증 가능 산출물.
- 박지 말 패턴: full rewrite from scratch ("framework 250 줄 자산 폐기"), generator hand-write ([[feedback-ssot-first]] 위반), legacy 이동 hybrid (한 src 안 옛+새 혼재).
