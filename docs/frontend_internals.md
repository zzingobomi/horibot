# Frontend internals — 데이터 흐름 + 구현 anchor

[frontend_walkthrough.md](frontend_walkthrough.md) 는 *사용처 가이드* (어떻게 쓰나). 본 문서는 *내부 구현 anchor* (어떻게 짰나, 왜 이렇게).

문서만 읽어도 그림 잡히게 ASCII 다이어그램 + 수도코드 + 진짜 코드 path. 디테일은 직접 코드 클릭해서 확인.

---

## §1. 토픽 도착 → component re-render 끝까지

대상: `motor_node` 가 `MOTOR_STATE_JOINT` publish 했을 때, [JointPanel](../frontend/src/components/panels/JointPanel.tsx) 의 joints UI 가 갱신되기까지.

### 흐름 도식

```
┌─────────────────┐    Zenoh    ┌────────────────┐
│  motor_node     │ ──────────▶ │  zenoh_bridge  │
│  publish(MJS)   │  (JSON)     │  (백엔드)        │
└─────────────────┘             └────────┬───────┘
                                         │ WS 메시지
                                         │ {type:topic_data, topic, data}
                                         ▼
                              ┌────────────────────┐
                              │  bridge.ts         │
                              │  onmessage()       │
                              │  subscribers.get() │
                              └────────┬───────────┘
                                       │ callback(data)
                                       ▼
                              ┌──────────────────────────┐
                              │  framework/bootstrap.ts  │
                              │  (callback 안 두 자리)   │
                              │                          │
                              │  1) store.setTopicData() │ ◀── reactive 자리
                              │  2) topicHandlers 호출    │ ◀── 비즈니스 자리
                              └────────┬─────────────────┘
                                       │
                  ┌────────────────────┼────────────────────┐
                  ▼                                         ▼
       ┌─────────────────────┐                  ┌──────────────────────┐
       │  framework/store    │                  │  domain/handlers.ts  │
       │  topicData[wire]=…  │                  │  onTopic(...) 등록   │
       └────────┬────────────┘                  │  systemStore.upd...  │
                │ selector change                └──────────────────────┘
                ▼
       ┌────────────────────┐
       │  JointPanel        │
       │  useTopic(MJS)     │
       │  re-render         │
       └────────────────────┘
```

### 단계별 수도코드

**(a) backend publish** — [motor_node.py](../backend/nodes/device/motor_node.py)
```python
# 20Hz loop 안
state = MotorJointState(timestamp=now, joints=[MotorJoint(...) for ... ])
self.publish(Topic.MOTOR_STATE_JOINT, state)
# → ZenohSession.put("horibot/omx_f_0/motor/state/joint", json_bytes)
```

**(b) bridge fanout** — [zenoh_bridge.py](../backend/bridge/zenoh_bridge.py)
```
on Zenoh subscribe of `horibot/.../motor/state/joint`:
    for each WS client:
        ClientStream(client, topic).enqueue(data)   # LATEST_WINS 큐 (크기 1)
    sender task: ws.send_text(json.dumps({
        "type": "topic_data",
        "topic": "horibot/omx_f_0/motor/state/joint",
        "data": {...}
    }))
```

**(c) WS 수신** — [bridge.ts](../frontend/src/api/bridge.ts)
```
ws.onmessage = (msg):
    parsed = JSON.parse(msg.data)
    switch parsed.type:
      case "topic_data":
          callbacks = this.subscribers.get(parsed.topic)
          for cb in callbacks: cb(parsed.data)
      case "service_response":
          resolver = this.pendingServices.get(parsed.request_id)
          resolver({success, message, data})    # §2 에서 자세히
      case "log": ...
```

`subscribers` 는 `Map<wire_topic, Set<callback>>`. 같은 토픽 다중 sub 가능.

**(d) framework bootstrap 의 callback** — [bootstrap.ts](../frontend/src/framework/bootstrap.ts)
```
on app mount (useEffect):
    bridge.connect((connected) =>
        store.setBridgeConnected(connected)
        if connected: for h in connectHandlers: h())
    
    for each Topic.X (Object.values(Topic)):
        if X in BINARY_TOPICS: continue       # binary 는 store 자체 attach
        wire = topicFor(X)
        bridge.subscribe(wire, (data) =>
            # 두 가지 책임 ─────────────────────
            store.setTopicData(wire, data)              # ① reactive
            for h in topicHandlers.get(wire) ?? []:     # ② 비즈니스
                h(data, null))
```

두 자리:
- **① store 갱신** — `useTopic` 으로 read 하는 panel 들 즉시 reactive
- **② handler 호출** — `onTopic(...)` 으로 등록된 비즈니스 (heartbeat → systemStore.updateNode 등)

**(e) framework store** — [store.ts](../frontend/src/framework/store.ts)
```
setTopicData(k, v):
    state.topicData = { ...state.topicData, [k]: v }    # 새 reference (zustand immutable)
```

새 reference 가 핵심 — selector 가 reference 비교로 변경 detect.

**(f) useTopic selector** — [topic.ts](../frontend/src/framework/topic.ts)
```
useTopic<K>(topic, robotId?):
    wire = topicFor(topic, robotId)
    return useFrameworkStore(s => s.topicData[wire] ?? null)
    # zustand 가 selector 결과 reference 변경 시 component re-render
```

return type 은 `TopicPayloadMap[K]` — generated 의 schema 가 K 에 매핑돼 *자동 typed*. cast 0.

**(g) JointPanel re-render**
```
function JointPanel():
    joints = useTopic(Topic.MOTOR_STATE_JOINT)?.joints ?? EMPTY_JOINTS
    ...
    return <div>{joints.map(j => <JointSlider .../>)}</div>
```

### 왜 이렇게

| 자리 | 왜 |
|---|---|
| **callback 안 두 책임 분리** | reactive (`store.setTopicData`) 는 *모든 토픽 자동*. 비즈니스 (`topicHandlers`) 는 *등록한 자리만*. 사용처 panel 은 boilerplate 0 — 토픽 추가해도 자동 cache. |
| **BINARY_TOPICS skip** | binary stream (pointcloud) 은 ArrayBuffer 라 store 의 JSON cache 와 모양 다름. 도메인 store (`pointCloudStore._attach()`) 가 자체 sub + Float32Array 디코딩. |
| **`EMPTY_JOINTS` 모듈-scope const** | `?? []` 는 매 render 새 array reference → useCallback dep cascade. 모듈-scope const 면 reference stable. lint 가 잡음. |

---

## §2. 서비스 호출 — `useService(K).call(req)`

대상: [MoveJ.tsx](../frontend/src/components/panels/MotionPanel/MoveJ.tsx) 의 `await moveJ.call({ joints })`.

### 흐름 도식

```
   panel                  bridge.ts                   backend
   ─────                  ─────────                   ───────
                           
   useService(K)
   .call(req)
       │
       ▼
   bridge.callService ──▶  pendingServices Map
       │                   { request_id: cacheAndResolve }
       │                                                    
       │              ┌─── store.setServiceData              
       │              │     {pending:true}                   
       │              │                                      
       │              └─▶ ws.send(Service, key, req_id, req)
       │                                                ──▶ Zenoh queryable
       │                                                    handler 실행
       │                                                    response 생성
       │                                                ◀── service_response
       │                                                    {req_id, success, message, data}
       │                  ws.onmessage                       
       │                  resolver = pendingServices.get(req_id)
       │                  pendingServices.delete()           
       │                  resolver(res)                      
       │                      │                              
       │                      ▼                              
       │              ┌── store.setServiceData               
       │              │     {pending:false, data, ...}       
       │              │                                      
       │              └── promise.resolve(res)               
       ▼                                                     
   await 풀림
```

### 핵심 — cacheAndResolve closure

```
bridge.callService(key, req, opts):
    expanded = expand(key, robotId)              # robot_id placeholder 채움
    
    # ① 즉시 pending mark — UI 의 disabled state 즉시 반영
    prev = store.serviceData[expanded]
    store.setServiceData(expanded, {
        ...prev,
        pending: true,
    })
    
    return new Promise((resolve) =>
        request_id = makeRequestId()
        
        # ② closure 가 expanded(wire key) + resolve(Promise) 둘 다 capture
        cacheAndResolve = (res) =>
            store.setServiceData(expanded, {
                success: res.success,
                message: res.message,
                data: res.data,
                timestamp: now,
                pending: false,
            })
            resolve(res)
        
        # ③ request_id 로 매핑 — 동시 호출 안전
        pendingServices.set(request_id, cacheAndResolve)
        
        ws.send({type:Service, key:expanded, request_id, data:req})
        
        # ④ timeout 도 같은 closure — cache 갱신 + resolve 둘 다
        setTimeout(() =>
            if pendingServices.has(request_id):
                pendingServices.delete(request_id)
                cacheAndResolve({success:false, message:"타임아웃", data:{}})
        , 5000)
    )
```

### 응답 받기

```
ws.onmessage on service_response(req_id, success, message, data):
    resolver = pendingServices.get(req_id)
    if resolver:
        pendingServices.delete(req_id)
        resolver({success, message, data})
        # → cacheAndResolve 실행
        # → store.setServiceData 갱신 + promise resolve
```

### useService 의 view 부분

[service.ts](../frontend/src/framework/service.ts):
```
useService<K>(key, robotId?):
    wireKey = bridge.expand(key, robotId)
    entry = useFrameworkStore(s => s.serviceData[wireKey])    # reactive
    
    call = useCallback((req, opts) =>
        bridge.callService(key, req, opts)
        return store.serviceData[wireKey])
    
    return { call, data:entry?.data, success, message, pending, timestamp }
```

panel 자리에서:
```tsx
const moveJ = useService(ServiceKey.MOTION_MOVE_J);
// moveJ.pending — disabled state. reactive.
// moveJ.data    — 마지막 응답. cross-component 공유.
await moveJ.call({ joints });
```

### 왜 이렇게

| 자리 | 왜 |
|---|---|
| **즉시 `pending:true` mark** | UI 의 `disabled={moveJ.pending}` 즉시 반영. 사용자가 "전송 중..." 즉시 봄. await 끝나야 disabled 면 button 두 번 눌릴 가능성. |
| **request_id Map (closure)** | 여러 panel 동시 서비스 호출. response 가 *어느 호출에 대한 응답인지* request_id 로 매칭. closure 가 *그 호출의* expanded wire key + resolve 함수 capture. |
| **cache 책임이 bridge layer** | useService 안에서만이 아니라 *bridge.callService 직접 호출* (예: `domain/handlers.ts` 의 `onConnect` 안 motor config fetch) 도 cache 됨. 즉 *callService 한 번 호출 = framework store cache 한 번 갱신* invariant. |

---

## §3. `useResource` — HTTP module cache + listener

대상: [Container.tsx](../frontend/src/components/scene/Container.tsx) + [RobotStatePanel.tsx](../frontend/src/components/panels/RobotStatePanel.tsx) 둘 다 `/calibration/results` 호출 시.

### 흐름 도식

```
   첫 panel mount                두번째 panel mount             refetch()
   ───────────────              ───────────────────             ────────
                                                                
   useResource("/X")            useResource("/X")              entry.refetch()
       │                            │                              │
       ▼                            ▼                              ▼
   getEntry("/X")               getEntry("/X")                fetchResource(force:true)
   (없으니 새로 생성)            (이미 있음)                       │
       │                            │                              ▼
   listeners.add(me)            listeners.add(me)             new fetch 시작
       │                            │                              │
   entry.data == null?         entry.data != null            entry.data 갱신
       YES                         (cache hit)                     │
       │                            │                              ▼
   fetchResource("/X")          그대로 return                  notify(entry)
       │                            (no fetch)                    │
   entry.pending = fetch                                         ┌─┴─┐
       │                                                         ▼   ▼
   notify(entry)                                            listener1  listener2
   (모든 listener 호출 → setVersion++)                       force-update
                                                                  │
                                                                  ▼
                                                            cross-component
                                                            동시 갱신
```

### 핵심 — module cache + Set<listener>

```
module-scope:
    cache: Map<path, Entry>
    
Entry: {
    data: T | null
    error: string | null
    loading: bool
    pending: Promise<T> | null
    listeners: Set<() => void>     # re-render trigger 함수들
}

useResource<T, S>(path, options):
    [, setVersion] = useState(0)
    
    useEffect:
        entry = getEntry(path)
        listener = () => setVersion(v => v + 1)
        entry.listeners.add(listener)
        
        if entry.data == null and not entry.pending and not entry.loading:
            fetchResource(path)   # 첫 호출만 fetch
        
        if options.poll:
            timer = setInterval(() => fetchResource(path, force=true), poll)
        
        cleanup:
            entry.listeners.delete(listener)
            clearInterval(timer)
    
    entry = getEntry(path)
    selected = useMemo(() =>
        if entry.data == null: return null
        if options.select: return options.select(entry.data)
        return entry.data,
    [entry.data, options.select])
    
    return { data: selected, loading, error, refetch }
```

### `fetchResource` 의 *cache hit + race 방지*

```
fetchResource(path, force=false):
    entry = getEntry(path)
    
    # ① cache hit / 동시 호출 진행 중 — skip
    if not force and (entry.data != null or entry.pending):
        if entry.pending: await entry.pending   # 진행 중이면 그 결과 기다림
        return
    
    # ② fetch 시작 — loading mark + notify
    entry.loading = true
    notify(entry)        # listener 호출 → "Loading..." UI 표시
    
    entry.pending = fetch(BASE_URL + path)
        .then(r => r.json() as T)
    
    try:
        entry.data = await entry.pending
        entry.error = null
    except as e:
        entry.error = e.message
    finally:
        entry.loading = false
        entry.pending = null
        notify(entry)    # listener 호출 → data 채워진 UI
```

### `select` 변환 — `useJointOffsetsRad`

```ts
// hooks/useCalibrationResults.ts
useJointOffsetsRad():
    return useResource("/calibration/results", {
        select: (d) => Object.fromEntries(
            d.joint_offsets.map((e) => [e.motor_id, e.offset_rad])
        ),
    }).data
```

같은 path cache 를 공유하면서 *변환 결과* 만 component 별 다르게. useMemo 가 select 결과 cache.

### 왜 이렇게

| 자리 | 왜 |
|---|---|
| **module-scope Map** | path 가 key — 동적이라 zustand store schema 가 헷갈림. `cache.get(path)` 단순. |
| **listeners Set + setVersion** | zustand 의 selector 안 씀. selector 는 *state 자체 변경* 만 detect — 우리 cache 는 Entry 안의 data field 갱신이라 React 가 자체 detect 안 함. *수동 force-update* 필요. |
| **`entry.pending` 으로 race 방지** | 두 component 가 같은 path 동시 mount → fetch 두 번? — pending 체크로 *한 번만* fetch. 두번째 호출은 *진행 중 promise 기다림*. |
| **select 결과를 useMemo** | data reference 가 같으면 select 안 다시 호출. derived value cache. useResource 사용처가 매번 `Object.fromEntries(...)` 안 돌아도 됨. |

---

## §4. register 패턴 — `onTopic` / `onConnect`

### 모양

```
// framework/topic.ts
module-scope:
    topicHandlers: Map<wire, GenericHandler[]>

onTopic<K>(topic, handler, robotId?):
    wire = topicFor(topic, robotId)
    arr = topicHandlers.get(wire) ?? []
    arr.push(handler)
    topicHandlers.set(wire, arr)


// framework/bootstrap.ts
module-scope:
    connectHandlers: Array<() => void>

onConnect(handler):
    connectHandlers.push(handler)
```

### 사용 자리 — module import 시점에 등록

```
// App.tsx
import "@/domain/handlers"   # ① 모듈 평가 = onTopic / onConnect 호출됨
useFrameworkBootstrap()      # ② 그 다음에 mount → bridge sub
```

```
// domain/handlers.ts (top-level)
onConnect(() => bridge.callService(MOTOR_GET_CONFIG, {}))
onTopic(Topic.SYSTEM_HEARTBEAT, hb => systemStore.updateNode(...))
onTopic(Topic.SYSTEM_LOG,        log => systemStore.addLog(log))
onTopic(Topic.TASK_TREE,         () => taskResultStore.clearAll())
onTopic(Topic.TASK_STEP_RESULT,  r  => taskResultStore.setStepResult(r))
```

### 흐름 그림

```
App import 순서:
    1. "@/domain/handlers"    ──▶  topicHandlers / connectHandlers 등록 완료
    2. useFrameworkBootstrap  ──▶  bridge sub 시작 → 토픽 도착 시 handler 호출

만약 순서 뒤집으면:
    1. useFrameworkBootstrap  ──▶  bridge sub. 토픽 도착해도 topicHandlers Map 빔
    2. "@/domain/handlers"    ──▶  등록. but 첫 토픽 이미 놓침
```

### 왜 이렇게

| 자리 | 왜 |
|---|---|
| **module-scope Map / Array** | React lifecycle 과 분리. App mount/unmount 와 무관하게 *모듈이 살아있는 동안* 등록 유지. |
| **register 패턴 (direct import X)** | framework 가 `useSystemStore` import 하면 *generic framework 가 우리 앱 store 에 묶임* — reusability 망함. callback 받는 API 만 노출, 등록 책임은 domain 쪽. 의존 방향 `framework ← domain` 단방향. |
| **App.tsx 의 import 순서 중요** | handlers 가 bootstrap 보다 앞서 평가되어야 첫 토픽 받을 때 handler 가 등록된 상태. ESM 정적 import 순서 = 코드 순서. |

---

## §5. SSOT pipeline — backend → generated → frontend

새 토픽 / 서비스 / HTTP 추가 시 자동 propagate.

### 파이프라인

```
   backend                                                frontend
   ───────                                                ────────
   
   pydantic schema 정의
   (core/transport/messages/*.py)
        │
        ▼
   api_contract.py 등재
   PUBLIC_TOPICS["X"] = (Topic.X, MyModel)
        │
        ▼
   bridge.custom_openapi()
   → /openapi.json 의 "x-contract" 필드에 인라인
        │
        │  ◀──  pnpm gen:types  ◀──
        ▼
                                          ┌────────────────────────────┐
                                          │ generated/types.ts         │
                                          │ (openapi-typescript)       │
                                          │ MyModel → TS type 자동      │
                                          └────────────────────────────┘
                                          ┌────────────────────────────┐
                                          │ generated/contract.ts      │
                                          │ (gen-contract.mjs 자체)     │
                                          │ Topic = { X: "..." }       │
                                          │ TopicPayloadMap = { ... }  │
                                          │ ServiceKey / ServiceMap    │
                                          └────────────┬───────────────┘
                                                       │
                                                       ▼
                                          ┌────────────────────────────┐
                                          │  panel.tsx                 │
                                          │  useTopic(Topic.X)         │
                                          │  → MyModel typed 자동       │
                                          └────────────────────────────┘
```

### 새 토픽 1줄 자리 예시

`LASER_STATE` 추가 가정:

```python
# backend/core/transport/messages/laser.py
class LaserState(StrictModel):
    distance_m: float
    timestamp: float

# backend/api_contract.py
PUBLIC_TOPICS = {
    ...
    "LASER_STATE": (Topic.LASER_STATE, LaserState),
}
```

```bash
cd frontend && pnpm gen:types
```

```tsx
// panel.tsx
const laser = useTopic(Topic.LASER_STATE)
// laser?.distance_m  ← typed, autocomplete
```

frontend 자체 작업 *0줄*. backend 만 변경.

---

## §6. 자질구레한 디테일

### zustand getState() vs selector

```ts
// selector — component re-render trigger. render 안에서.
const joints = useFrameworkStore((s) => s.topicData[wire]?.joints ?? [])

// getState() — non-reactive read. callback/handler 안에서.
onTopic(Topic.X, () => {
  useSystemStore.getState().updateNode(...)  // setter 호출만, re-render 트리거 X
})
```

handler 는 *render 밖* — selector 쓰면 hook rule 위반. getState() 가 정공.

### `useDetectorOverride.maskBefore` — frontend-local hack

```ts
// 데이터 흐름:
backend publish PERCEPTION_GROUNDED_STATE
   ↓ (latest-wins 토픽 — backend 가 *clear publish* 안 함)
useTopic(...) 가 마지막 publish 영구 보관
   ↓
PromptPanel 의 Run 버튼 → useDetectorOverride.hide() → maskBefore = now
   ↓
DetectionLayer/CameraFeedPanel:
    if topic.timestamp > maskBefore: 표시
    else: 가림
   ↓
backend 가 새 detection publish → timestamp > maskBefore → 자동 노출
```

**왜 hack**: 정공법 = backend 에 `PERCEPTION_CLEAR` service 추가 + handler 가 *empty publish*. 그러나 backend 변경 자리라 frontend rewrite scope 안 에선 hack. 다음 reorg 자리.

### bridge 재연결 시점 — `onConnect` 가 매번 호출됨

```
bridge 연결 끊김 → ReconnectingWebSocket 자동 재연결 시도
재연결 성공 → bridge.connect callback(connected=true)
    → connectHandlers 다 호출
    → domain/handlers.ts 의 onConnect 안:
        - bridge.callService(MOTOR_GET_CONFIG, {})    # 다시 fetch
        - if unsubPointCloud: unsubPointCloud()       # 이전 sub 정리
          unsubPointCloud = pointCloudStore._attach() # 다시 attach
```

재연결 시 *멱등* — 이전 unsub 호출 후 재 attach.

---

## §7. 직접 뜯어볼 anchor — 순서

1. [framework/index.ts](../frontend/src/framework/index.ts) — 공개 API 모음 (5분)
2. [framework/store.ts](../frontend/src/framework/store.ts) — 단순 zustand store (5분)
3. [framework/topic.ts](../frontend/src/framework/topic.ts) — `useTopic` + `onTopic` (15분 — §1 자리)
4. [framework/service.ts](../frontend/src/framework/service.ts) + [api/bridge.ts](../frontend/src/api/bridge.ts) `callService` (30분 — §2 자리)
5. [framework/resource.ts](../frontend/src/framework/resource.ts) — listener 패턴 (20분 — §3 자리)
6. [framework/bootstrap.ts](../frontend/src/framework/bootstrap.ts) — useEffect 안 흐름 (15분)
7. [domain/handlers.ts](../frontend/src/domain/handlers.ts) — 진짜 사용 예 (10분)
8. [components/panels/MotionPanel/MoveJ.tsx](../frontend/src/components/panels/MotionPanel/MoveJ.tsx) — `useTopic` + `useService` 합쳐진 *대표* panel (20분)

각 자리 직접 뜯고, 막힌 자리 질문하면 답하거나 본 문서 갱신.
