# Frontend walkthrough

코드 따라가면서 공부할 때 *anchor*. 패턴 용어 (decorator/observer/...) 안 씀 — 진짜 코드 + "왜 이렇게 짰나" 우선.

전제: backend api_contract / 토픽 / 서비스 SSOT 가 frontend 모든 자리에 *자동 propagate*. 새 기능 추가 시 사용처 1줄.

## §0. 시작점 — App.tsx 한 자리

```tsx
// frontend/src/App.tsx
import { useFrameworkBootstrap } from "@/framework";
import "@/domain/handlers"; // module-top side-effect — onTopic 등록

function AppContent() {
  useFrameworkBootstrap();
  return <... />;
}
```

이 두 줄이 전부:
- `useFrameworkBootstrap()` — bridge connect + 모든 토픽 auto-subscribe + handler dispatch
- `import "@/domain/handlers"` — module 평가 시 `onTopic(...)` 들이 등록됨. 등록 시점이 bootstrap 의 sub 보다 *앞서야* 첫 토픽 받을 때 handler 가 호출되니까 import 가 위쪽

## §1. 데이터 흐름 한 가지 따라가기 — heartbeat 토픽

backend 가 `horibot/system/heartbeat` 에 `Heartbeat` pydantic 으로 publish.

**1단계 — backend SSOT 등재** ([backend/api_contract.py](../backend/api_contract.py)):

```python
PUBLIC_TOPICS = {
  "SYSTEM_HEARTBEAT": (Topic.SYSTEM_HEARTBEAT, Heartbeat),
  ...
}
```

**2단계 — gen:types** ([frontend/src/api/generated/contract.ts](../frontend/src/api/generated/contract.ts) 자동 emit):

```ts
export const Topic = {
  SYSTEM_HEARTBEAT: "horibot/system/heartbeat",
  ...
};

export type TopicPayloadMap = {
  "horibot/system/heartbeat": components["schemas"]["Heartbeat"];
  ...
};
```

**3단계 — bootstrap 가 자동 sub** ([frontend/src/framework/bootstrap.ts](../frontend/src/framework/bootstrap.ts)):

```ts
for (const tpl of Object.values(Topic)) {
  if (BINARY_TOPICS.has(tpl)) continue;
  const wire = topicFor(tpl);
  bridge.subscribe(wire, (data) => {
    useFrameworkStore.getState().setTopicData(wire, data);
    topicHandlers.get(wire)?.forEach((h) => h(data, null));
  });
}
```

토픽 도착 → framework store 의 latest 갱신 + 등록된 onTopic handler 들 호출.

**4단계 — domain 비즈니스 등록** ([frontend/src/domain/handlers.ts](../frontend/src/domain/handlers.ts)):

```ts
onTopic(Topic.SYSTEM_HEARTBEAT, (hb) => {
  useSystemStore.getState().updateNode(
    hb.node,
    hb.status === "ok" ? "running" : "error",
    hb.timestamp,
    hb.robot_id ?? null,
  );
});
```

`hb` 는 `Heartbeat` 로 *자동 typed* — `TopicPayloadMap[K]` 가 `K` 에 맞춰 emit. cast / `as` 자리 0.

**5단계 — UI 가 read** — 둘 중 하나:
- 누적 자리: `useSystemStore((s) => s.nodes)` (heartbeat 별 node 누적)
- latest 자리: `useTopic(Topic.SYSTEM_HEARTBEAT)` (단순 마지막 publish)

`SYSTEM_HEARTBEAT` 는 *누적* 자리라 store. `MOTOR_STATE_JOINT` 같은 *단순 latest* 자리는 store 없이 `useTopic` 직접.

## §2. framework/ — generic transport ↔ UI 변환

backend SSOT 에서 자동 emit 된 토픽 / 서비스 / HTTP 가 frontend UI 자리에 *1줄로* 들어오게 하는 자리. *우리 horibot 와 무관*한 generic.

### [framework/topic.ts](../frontend/src/framework/topic.ts)

```ts
// declarative read
const joints = useTopic(Topic.MOTOR_STATE_JOINT)?.joints ?? [];

// 비즈니스 등록 (domain/handlers.ts 에서)
onTopic(Topic.SYSTEM_LOG, (log) => {
  useSystemStore.getState().addLog(log);
});
```

generated `TopicPayloadMap[K]` 가 `K` 에 따라 자동 — `log` / `joints` 안에 cast 0.

### [framework/service.ts](../frontend/src/framework/service.ts)

```ts
const moveJ = useService(ServiceKey.MOTION_MOVE_J);
await moveJ.call({ joints: [...] });

// pending state 자동
disabled={moveJ.pending}
```

`bridge.callService` 가 framework store 에 *pending → response 자동 cache* ([frontend/src/api/bridge.ts](../frontend/src/api/bridge.ts) 의 `cacheAndResolve`). 같은 서비스를 다른 panel 에서도 `useService(Key.X)` 로 호출하면 *마지막 응답 공유* — `data` / `success` 가 reactive.

`MOTOR_GET_CONFIG` 가 좋은 예: `domain/handlers.ts` 의 onConnect 에서 1회 fetch → 그 응답이 framework store cache → JointPanel / Settings / Sidebar 가 `useService(MOTOR_GET_CONFIG).data` 로 *같은 cache* read.

### [framework/resource.ts](../frontend/src/framework/resource.ts)

backend HTTP endpoint (`/robots`, `/tasks`, `/system`, `/calibration/results`) 호출용. module cache + cross-component sync + poll + select.

```ts
const { robots } = useResource<RobotsListResponse>("/robots").data ?? {};

// select 로 derived (memo 불필요)
const offsets = useResource<CalibrationResults, Record<number, number>>(
  "/calibration/results",
  { select: (d) => Object.fromEntries((d.joint_offsets ?? []).map((e) => [e.motor_id, e.offset_rad])) },
).data ?? {};

// 주기 fetch
const { data } = useResource<SystemMetrics>("/system", { poll: 5000 });
```

같은 path 호출 자리는 *같은 module cache* 공유 — 여러 컴포넌트가 호출해도 fetch 1번. `refetch()` 호출 시 모든 사용처 동시 reactive.

### [framework/bootstrap.ts](../frontend/src/framework/bootstrap.ts)

`useFrameworkBootstrap()` — App.tsx 1회 마운트. bridge connect + 모든 토픽 auto-sub.

`onConnect(...)` — bridge 연결 시점에 호출되는 callback. domain/handlers.ts 가 *연결 후 1회 비즈니스* (motor config fetch, PointCloud binary stream attach) 등재.

### [framework/store.ts](../frontend/src/framework/store.ts)

internal cache (topic latest + service response + bridge connected). 외부에서 *직접* 안 만짐 — `useTopic / useService / useBridgeConnected` 가 view.

## §3. domain/ — 우리 앱 비즈니스

### [domain/handlers.ts](../frontend/src/domain/handlers.ts)

토픽 도착 시 *진짜 누적 / side-effect* 만. 단순 latest cache 토픽 (motor/camera/motion/task/detector state) 은 framework store 가 자동 흡수 — 본 파일에 등재 X. 사용처가 `useTopic(Topic.X)` 직접.

남은 자리: heartbeat tracker / log accumulator / task tree clear / step result accumulator / motor config fetch on connect.

### [domain/stores/](../frontend/src/domain/stores/)

진짜 *누적 / cross-component / 도메인 액션* 만:
- [`system.ts`](../frontend/src/domain/stores/system.ts) — nodes (heartbeat 누적) + logs (bounded FIFO)
- [`taskResult.ts`](../frontend/src/domain/stores/taskResult.ts) — `Record<step_id, payload>` 누적
- [`pointCloud.ts`](../frontend/src/domain/stores/pointCloud.ts) — binary stream + 도메인 액션 (capture/buildMesh/...)
- [`scene.ts`](../frontend/src/domain/stores/scene.ts) — UI state (옵션 toggle, link visibility)
- [`detector.ts`](../frontend/src/domain/stores/detector.ts) — `maskBefore` (frontend-local hide. 정공법은 backend 에 `PERCEPTION_CLEAR` service 추가)

옛 store 9개 중 4개 (camera/motion/task/robot) 는 *단순 latest cache* 라 framework 가 흡수 — 삭제됨. 사용처는 `useTopic` / `useService` 직접.

## §4. 새 자리 추가 시 — 사용처 1줄

| 추가 | frontend 자리 |
|---|---|
| 새 토픽 *read* | `useTopic(Topic.X)` (panel 안 1줄) |
| 새 토픽 *비즈니스* | `onTopic(Topic.X, handler)` ([domain/handlers.ts](../frontend/src/domain/handlers.ts) 1줄) |
| 새 서비스 | `const svc = useService(Key.X); await svc.call(req)` |
| 새 HTTP endpoint | `const { data } = useResource<T>("/path")` |
| 새 로봇 | [robots.yaml](../robot/robots.yaml) 1줄. frontend 코드 0 |
| 새 panel | [panels/X.tsx](../frontend/src/components/panels/) + [registry.ts](../frontend/src/components/panels/registry.ts) 1줄 |

backend pydantic / api_contract.py 자리 1줄 추가 → `pnpm gen:types` → frontend 가 자동 typed. hand-sync 0.

## §5. 자주 막힐 자리

### "토픽이 안 와요"

- `domain/handlers.ts` 에 `onTopic` 등록했나 (필요 시) — 등록 자리는 *module-top* 이라 `App.tsx` 의 `import "@/domain/handlers"` 가 살아있어야
- bootstrap 의 `BINARY_TOPICS.has(tpl)` skip 자리 — binary 토픽 (pointcloud stream) 은 store 자체 `_attach()` 로 sub
- backend 가 진짜 publish 하는지 — Zenoh CLI 또는 백엔드 로그

### "useService.call() 의 응답이 cache 안 됐어요"

`bridge.callService` 의 `cacheAndResolve` 가 *resolve 직전* framework store 에 set. timeout 자리도 cache 됨. 다만 *재호출* 시 새 응답이 옛 응답 덮어씀.

### "MOTOR_GET_CONFIG 가 두 번 호출됨"

`domain/handlers.ts` 의 `onConnect` 가 *재연결마다* 호출. 정상. 응답 cache 는 동일 wire key 라 *최신 응답* 만 보임.

### "type 이 unknown 으로 나와요"

`generated/contract.ts` 의 `TopicPayloadMap` / `ServiceMap` 에 *unknown* 으로 emit 된 자리 (예: `TASK_STATE`). backend api_contract.py 의 schema 가 generic dict 인 자리. *pydantic 으로 정의 + 등재* → 다음 `pnpm gen:types` 후 typed.

### "옛 store 호출이 panel 에 남아있어요"

다 정리됐어야 — `@/store/` import 자리 0. 만약 남아있으면 tsc 가 잡음.

### "panel 추가했는데 dockview 가 안 띄움"

[panels/registry.ts](../frontend/src/components/panels/registry.ts) 의 `PANEL_COMPONENTS` 에 등재됐는지 + 페이지의 `PANELS` spec 에 `id/component` 짝 추가됐는지.

## 참조

- [framework/index.ts](../frontend/src/framework/index.ts) — 공개 API 한 자리
- [domain/handlers.ts](../frontend/src/domain/handlers.ts) — onTopic / onConnect 사용 예
- [components/panels/MotionPanel/MoveJ.tsx](../frontend/src/components/panels/MotionPanel/MoveJ.tsx) — useTopic + useService 가 합쳐진 *대표 panel* (joint state + motor config + trajectory + move_j + stop 한 자리)
- [components/panels/JointPanel.tsx](../frontend/src/components/panels/JointPanel.tsx) — service refresh 패턴 (`enableSvc.call(...)` 후 `cfgSvc.call({})` 로 config cache 갱신)
- [components/scene/Container.tsx](../frontend/src/components/scene/Container.tsx) — `useTopic` (joints) + `useResource` (calibration) + `useSceneStore` (UI) 가 한 자리에서 *각자 자리* 가짐
