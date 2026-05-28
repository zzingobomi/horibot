# Task 디버거 UI

> Task 실행을 VSCode 디버거처럼 **step 단위로 멈추고 굴리는** UI. 기존 "Run 한 번 누르면 끝까지" 모드 옆에 **Run / Step / Run-to-here / Breakpoint** 4종 조작을 얹음. 미래 LLM orchestrator 의 ForEach / If / Loop 같은 control flow 흡수를 데이터 모델 차원에서 미리 깔아둠.

## 왜 이 모양인가 — 핵심 결정 4개

학습 차원에서 이 결정들이 코드 곳곳에 어떻게 박혀있는지가 핵심이니까 먼저 정리.

### 1. mental model: VSCode 디버거

처음 후보였던 "auto / manual 2-mode 분리" 는 거부함. 이유:

- "한 스텝씩만" 모드와 "끝까지 자동" 모드가 사실 **같은 매커니즘** — step 경계마다 멈출지 말지의 차이
- 두 모드 분리하면 "manual 모드에서 도중 auto 로 전환" 같은 표현이 어색해짐
- VSCode 디버거 메타포는 이미 모두가 익숙

→ 모드 개념 제거. **4종 조작** 으로 통합:

| 조작 | 의미 |
|---|---|
| ▶ Run / Continue | 다음 breakpoint 또는 끝까지 자동 진행 |
| ⤵ Step | 1 step 만 실행 후 다시 pause |
| 🎯 Run to here | 지정한 step **직전**까지 자동 진행 후 pause (우클릭 메뉴) |
| ● Breakpoint | step 옆 점 — 자동 진행 중 거기서 멈춤 |

VSCode 의 "Run to cursor" 와 동일하게 **선택한 step 직전까지** 실행 (그 step 에서 멈춤, 그 step 은 다음 진행 대상). [task_runner.py:_should_pause_before](../backend/modules/task/task_runner.py#L218-L242) 의 분기와 정확히 일치.

### 2. 데이터 모델: 평면 list → 트리 (미래 ForEach 흡수)

현 pick_and_place 는 **평면 list** 면 충분하지만, 미래 LLM orchestrator ([ideas.md](ideas.md#llm-task-orchestrator-) §4축) 에서 ForEach / If / Loop primitive 가 들어오면 step 들이 **중첩** 됨. 그때 가서 데이터 구조 갈아엎기 싫음.

→ 지금부터 **트리 구조 가능한 모양** 으로 박음:

- 모든 `Step` 에 `id: str` 필드 (안정적 식별자 — breakpoint 박을 때 사용)
- `StepNode` (frontend) 에 optional `children?: StepNode[]` — 지금은 모든 step 이 leaf, 미래 ForEach 가 추가될 때 자식으로 채움
- 평면 list 는 "깊이 1 트리" 로 자연스럽게 표현됨 → UI 도 동일 컴포넌트로 재귀 렌더 가능 (현재는 재귀 없이 1-depth 만 렌더)

### 3. 토픽 분리: TASK_TREE (정적) vs TASK_STATE (동적)

원래 `TASK_STATE` 토픽에 모든 정보 넣으면 단순하지만 — step 트리 구조는 **task 시작 시 1회만** 변함 (실행 중에는 안 변함), 그 외 정보 (status, current step, step statuses) 는 **매 step 마다** 변함. 둘이 변화 빈도가 1000배 다름.

→ 두 토픽 분리:
- `TASK_TREE` (`omx/task/tree`) — task 시작 또는 preview 시 1회 publish. latest-wins 큐로 늦게 붙은 클라이언트도 마지막 트리를 받음.
- `TASK_STATE` (`omx/task/state`) — 자주 publish. status / current_step_id / step_statuses dict / breakpoints list 포함.

### 4. Run 전에도 트리 보이게 — Preview 서비스

"Run 누른 후에야 step 트리 보이면 사전에 breakpoint 못 박는다" 문제. 해결: **`TASK_PREVIEW` 서비스** — task factory 만 실행해서 트리 만들고 `TASK_TREE` 토픽에 broadcast. 실행은 X.

자동 호출은 **앱 마운트 시 1회만** ([PromptPanel](../frontend/src/components/panels/PromptPanel.tsx) 의 `bridgeConnected` 의존 useEffect). 이후 prompt 수정 시 자동 갱신 안 함 — 사전에 박은 breakpoint 가 새 트리에서 dangling 되는 혼란을 피함. 사용자가 명시적으로 "Preview" 버튼 눌러야 갱신.

---

## 데이터 흐름 (시간 순)

```
[앱 시작]
  PromptPanel mount
    └─ bridgeConnected=true 보고
       └─ TASK_PREVIEW 호출 (task=pick_and_place, prompt=default)
          └─ Backend: factory(prompt) → Task 빌드 → task_tree(task) → publish TASK_TREE
             └─ Frontend: useBridge 가 TASK_TREE 받음 → setTaskTree(tree)
                └─ TaskProgressPanel 자동 리렌더 — step 리스트 표시

[사용자가 step 옆 점 영역 클릭]
  StepRow gutter 클릭
    └─ toggleBreakpoint(stepId) 서비스 호출
       └─ Backend: TaskRunner._breakpoints set 토글
          └─ TaskRunner._publish_state() → TASK_STATE 토픽
             └─ Frontend: setTaskState(state) — breakpoints list 갱신
                └─ TaskProgressPanel 리렌더 — 점 빨갛게

[사용자가 PromptPanel 의 Run 누름]
  PromptPanel run() → TASK_RUN 서비스
    └─ Backend: factory(prompt) → Task → publish TASK_TREE (한번 더, 동기화)
       └─ TaskRunner.run(task) — thread 시작, _state.step_statuses 초기화 (모두 pending)
          └─ _run_task 루프:
             각 step 전에:
                ├─ _should_pause_before(step) → breakpoint? STEP_ONCE? RUN_TO target?
                │    └─ 멈춰야 하면 TaskStatus.PAUSED + _pause_event.clear()
                │       └─ TASK_STATE publish (current_step_id = 다음 step.id)
                ├─ _pause_event.wait() — 사용자 입력 대기
                ├─ step 실행 → _set_step_status(id, RUNNING) → publish
                └─ 성공 → COMPLETED publish / 실패 → FAILED publish + task FAILED

[사용자가 우클릭 → "Run to here"]
  StepRow onContextMenu → ContextMenu state set
    └─ MenuItem 클릭 → runTo(stepId) 서비스
       └─ Backend: _mode = RUN_TO, _run_to_target = stepId, _pause_event.set()
          └─ _run_task 가 다음 step.id == target 일 때 _should_pause_before True 반환 → 다시 PAUSED
```

---

## Backend 구현

### Step DSL — id 자동 부여

[step_types.py](../backend/modules/task/step_types.py) — 모든 step dataclass 에 `id: str = ""` 추가. `label` 다음, `type` (Literal, init=False) 직전 위치:

```python
@dataclass
class MoveTCPStep:
    position: Position3 | None = None
    position_key: str | None = None
    offset: Position3 = (0.0, 0.0, 0.0)
    label: str = ""
    id: str = ""    # task 빌드 시 Task.__post_init__ 에서 자동 부여
    type: Literal["move_tcp"] = field(default="move_tcp", init=False, repr=False)
```

`id` 부여는 **Task dataclass 의 `__post_init__`** 에서. dataclass 의 default 라이프사이클을 활용해서 모든 Task 생성 경로 (factory 함수들) 가 자동 거침:

```python
@dataclass
class Task:
    name: str
    steps: list[Step]
    description: str = ""

    def __post_init__(self) -> None:
        # 이미 id 가 있는 step 은 보존 — 미래 LLM 이 명시적 id 줄 수 있음.
        for i, step in enumerate(self.steps):
            if not getattr(step, "id", ""):
                step.id = f"step-{i}"
```

미래 ForEach 가 자식 step 들을 가질 때는 여기서 재귀적으로 `"step-3.0"`, `"step-3.1"` 같은 path-based id 부여로 확장. 지금은 평면이라 단순 인덱스.

### Tree serialization

`task_tree(task)` — frontend 가 받는 dict 모양. dataclasses.asdict 로 step 직렬화:

```python
def task_tree(task: Task) -> dict:
    return {
        "task_name": task.name,
        "description": task.description,
        "steps": [step_to_dict(s) for s in task.steps],
    }
```

미래 ForEach step 의 children 필드도 dataclass field 면 asdict 가 자동 재귀 — 추가 코드 X.

### TaskRunner — 디버거 모드

[task_runner.py](../backend/modules/task/task_runner.py) 의 핵심:

**상태 분리:**

```python
class DebugMode(str, Enum):
    AUTO = "auto"        # 다음 breakpoint 까지
    STEP_ONCE = "step"   # 1 step 만 실행 후 pause
    RUN_TO = "run_to"    # 특정 step.id 직전까지

# TaskRunner 인스턴스 필드:
self._mode: DebugMode = DebugMode.AUTO
self._run_to_target: str | None = None
self._breakpoints: set[str] = set()
```

`_mode` 와 `_run_to_target` 은 **외부 노출 안 함**. 사용자가 알 필요 없음 — Run / Step / Run-to-here 버튼이 내부적으로 mode set + `_pause_event.set()` 두 동작을 묶을 뿐.

`_breakpoints` 는 `TaskState.breakpoints` 로 노출 (frontend 가 시각화에 필요).

**핵심 게이트 — `_should_pause_before`:**

매 step 실행 직전에 호출. 세 조건 중 하나라도 만족하면 멈춤:

```python
def _should_pause_before(self, step) -> bool:
    with self._state_lock:
        mode = self._mode
        target = self._run_to_target
        is_breakpoint = step.id in self._breakpoints

    if mode == DebugMode.STEP_ONCE:
        return True
    if mode == DebugMode.RUN_TO and target == step.id:
        return True
    if is_breakpoint:
        return True
    return False
```

여기서 mode 를 reset 안 함 — `step_once()` 와 `run_to()` 가 호출될 때마다 다시 set 함. 즉 "한 번 동작 후 자동으로 AUTO 복귀" 가 아니라 "다음 사용자 입력이 mode 를 갱신" 패턴.

**외부 API 4종:**

```python
def resume(self) -> bool:       # mode → AUTO, pause_event.set()
def step_once(self) -> bool:    # mode → STEP_ONCE, pause_event.set()
def run_to(self, target_step_id: str) -> bool:   # mode → RUN_TO, target set, set()
def toggle_breakpoint(self, step_id: str) -> bool:  # set 토글 + publish
```

각각 PAUSED 상태에서만 의미 있음 (breakpoint 토글만은 IDLE 에서도 가능 — 사전 박기).

**run loop 의 구조:**

```python
for i, step in enumerate(task.steps):
    if self._stop_event.is_set(): return STOPPED

    should_pause = self._should_pause_before(step)
    if should_pause:
        # current_step_id 를 다음 step 으로 미리 세팅 — UI 가 "지금 여기" 표시
        self._update_state(status=PAUSED, current_step=i, current_step_id=step.id, ...)
        self._pause_event.clear()

    self._pause_event.wait()  # 외부 pause() 또는 위 게이트 둘 다 처리

    if self._stop_event.is_set(): return STOPPED

    # 실행
    self._set_step_status(step.id, STEP_RUNNING)
    ok = self._executor.execute(step, context, self._stop_event)
    self._set_step_status(step.id, STEP_COMPLETED if ok else STEP_FAILED)
    if not ok: return FAILED
```

PAUSED 시 `current_step` 을 i (0-based, "다음" 의미) 로 세팅하는 점 주목 — RUNNING 시 i+1 (1-based, "지금" 의미) 와 의미가 다름. UI 에서 PAUSED 일 때 "다음 실행될 step" 을 강조하는 근거.

### 토픽 / 서비스

[topic_map.py](../backend/core/topic_map.py) 추가분:

```python
# Topic
TASK_TREE = "omx/task/tree"   # task 시작 또는 preview 시 1회 publish (latest-wins)

# Service
TASK_STEP = "omx/task/srv/step"
TASK_RUN_TO = "omx/task/srv/run_to"
TASK_TOGGLE_BREAKPOINT = "omx/task/srv/toggle_breakpoint"
TASK_PREVIEW = "omx/task/srv/preview"
```

`TASK_TREE` 는 **bridge `_ALWAYS_SUBSCRIBE`** 에 추가 ([zenoh_bridge.py](../backend/bridge/zenoh_bridge.py#L137-L148)) — frontend 가 어느 시점에 connect 해도 latest-wins 큐를 통해 마지막 tree 수신.

[task_node.py](../backend/nodes/task_node.py) 의 서비스 핸들러 — 단순한 위임 패턴:

```python
def _handle_step(self, _req: dict) -> dict:
    ok = self._runner.step_once()
    return {"success": ok, "message": "ok" if ok else "PAUSED 상태 아님", "data": {}}

def _handle_run_to(self, req: dict) -> dict:
    step_id = str(req.get("data", {}).get("step_id") or "").strip()
    if not step_id:
        return {"success": False, "message": "step_id 필요", "data": {}}
    ok = self._runner.run_to(step_id)
    return {"success": ok, ...}

def _handle_toggle_breakpoint(self, req: dict) -> dict:
    step_id = str(req.get("data", {}).get("step_id") or "").strip()
    if not step_id: ...
    self._runner.toggle_breakpoint(step_id)  # 항상 success
    return {"success": True, "message": "ok", "data": {}}
```

`_handle_preview` 는 살짝 다름 — `_runner.is_running()` 가드 (실행 중인 task 의 tree 를 preview 가 덮어쓰지 않게):

```python
def _handle_preview(self, req: dict) -> dict:
    if self._runner.is_running():
        return {"success": False, "message": "실행 중 — 종료 후 preview", "data": {}}
    # ... factory(data) → tree → publish + 응답 양쪽
    self.publish(Topic.TASK_TREE, tree)
    return {"success": True, "message": "ok", "data": tree}
```

응답으로 tree 도 같이 반환 (직접 사용 가능) + 토픽으로 broadcast (다른 클라이언트 동기화). 현재는 1 client 전제라 사실 토픽만으로 충분하지만 broadcast 패턴이 일관성.

`_handle_run` 은 tree publish + runner.run 두 단계:

```python
def _handle_run(self, req: dict) -> dict:
    # ... factory 검증 ...
    self.publish(Topic.TASK_TREE, task_tree(task))   # frontend 가 먼저 tree 알게
    if not self._runner.run(task): ...               # 그 다음 실행 시작
```

순서 중요 — Zenoh put 은 sync 이므로 tree publish 가 thread 시작보다 먼저 끝남. frontend 가 첫 RUNNING state 받기 전에 tree 알고 있어야 step_id 매칭 가능.

---

## Frontend 구현

### Types & Store

[types/task.ts](../frontend/src/types/task.ts) — `StepNode` 가 핵심:

```ts
export interface StepNode {
  id: string;
  type: string;
  label: string;
  children?: StepNode[];   // 미래 ForEach/If 의 자식 step
  [key: string]: unknown;  // step type 별 파라미터 — 디버그 표시 외엔 직접 접근 안 함
}

export interface TaskTree {
  task_name: string;
  description: string;
  steps: StepNode[];
}

export interface TaskState {
  status: TaskStatus;
  task_name: string;
  current_step: number;
  total_steps: number;
  current_label: string;
  current_step_id: string;
  error: string | null;
  step_statuses: Record<string, StepStatus>;
  breakpoints: string[];
}
```

`StepNode` 의 `[key: string]: unknown` 인덱스 시그니처가 디테일 패널의 핵심 — step type 마다 다른 파라미터 (move_tcp 는 position/offset, gripper 는 action/current 등) 를 type 별 클래스 만들지 않고도 표시 가능. 백엔드 dataclass → asdict → JSON → 그대로 표시.

[taskStore.ts](../frontend/src/store/taskStore.ts) — `taskTree` 가 추가됨. 두 store 가 분리되었지만 한 Zustand store 안에 같이 보관 (한 도메인).

### 토픽 구독 — useBridge

[useBridge.ts](../frontend/src/hooks/useBridge.ts) — `TASK_TREE` 구독은 다른 토픽과 같은 패턴:

```ts
const unsubTaskTree = bridge.subscribe(Topic.TASK_TREE, (data) => {
  setTaskTree(data as unknown as TaskTree);
});
```

latest-wins 큐 덕에 늦게 mount 된 컴포넌트도 마지막 tree 를 받음. 즉 `TaskProgressPanel` 이 dockview 의 collapsed 상태로 시작했다가 사용자가 펼치는 순간 tree 데이터가 이미 store 에 있음.

### useTask hook 확장

[useTask.ts](../frontend/src/hooks/useTask.ts) 가 backend 4개 서비스를 1:1 노출:

```ts
const step = useCallback(async (): Promise<boolean> => {
  const res = await bridge.callService(ServiceKey.TASK_STEP, {});
  return res.success;
}, []);

const runTo = useCallback(async (stepId: string): Promise<boolean> => {
  const res = await bridge.callService(ServiceKey.TASK_RUN_TO, { step_id: stepId });
  return res.success;
}, []);

const toggleBreakpoint = useCallback(async (stepId: string): Promise<boolean> => {
  const res = await bridge.callService(ServiceKey.TASK_TOGGLE_BREAKPOINT, { step_id: stepId });
  return res.success;
}, []);
```

`taskTree` 도 같이 반환해서 panel 이 한 hook 으로 모든 정보 / 조작 접근.

### TaskProgressPanel — 트리 렌더 + breakpoint UX

[TaskProgressPanel.tsx](../frontend/src/components/panels/TaskProgressPanel.tsx) — 한 파일에 다 들어있음 (panel + StepRow + StatusIcon + ControlBtn + MenuItem).

**구조:**

```
PanelShell
├─ Status + 컨트롤 바 (Run/Step/Stop)
├─ Step 리스트 (taskTree.steps.map → StepRow)
├─ Error 영역 (taskState.error 가 있을 때만)
└─ Context Menu (createPortal → document.body)
```

**StepRow 의 행 구조:**

```
[breakpoint gutter] [chevron] [status icon] [label]
        ↓                              디테일 (expanded 시)
   호버 prefill /                      ┌─ type    move_tcp
   클릭 토글                            ├─ params  ...
                                       └─ ...
```

**Breakpoint gutter — 호버 prefill 패턴 (VSCode 같이):**

```tsx
<button
  onClick={(e) => {
    e.stopPropagation();   // 행 클릭 (expand) 와 분리
    onToggleBreakpoint();
  }}
  className="w-3 h-3 ..."
>
  {isBreakpoint ? (
    <span className="w-2.5 h-2.5 rounded-full bg-red-500" />
  ) : hover ? (
    <span className="w-2 h-2 rounded-full bg-red-500/30" />   // 옅은 prefill
  ) : null}
</button>
```

`hover` state 는 StepRow 자체의 `onMouseEnter/Leave`. gutter 만 호버해도 행 자체가 호버되니 동일 영역.

**expand / collapse:**

```ts
const [expanded, setExpanded] = useState<Set<string>>(() => new Set());

const toggleExpand = useCallback((stepId: string) => {
  setExpanded((prev) => {
    const next = new Set(prev);
    if (next.has(stepId)) next.delete(stepId);
    else next.add(stepId);
    return next;
  });
}, []);
```

행 자체 onClick = `toggleExpand`. gutter 의 onClick 은 `e.stopPropagation()` 으로 expand 안 일어나게 분리.

**디테일 표시 — 인덱스 시그니처 활용:**

```tsx
const HIDDEN_PARAM_KEYS = new Set(["id", "type", "label", "children"]);

const paramEntries = useMemo(
  () => Object.entries(node).filter(([k]) => !HIDDEN_PARAM_KEYS.has(k)),
  [node],
);

{isExpanded && (
  <div className="px-2 pb-1.5 pl-10 ...">
    <div className="grid grid-cols-[auto_1fr] gap-x-3 gap-y-0.5 ...">
      <span>type</span><span>{node.type}</span>
      {paramEntries.map(([k, v]) => (
        <div key={k} className="contents">    {/* grid item 평탄화 */}
          <span>{k}</span>
          <span>{formatParamValue(v)}</span>
        </div>
      ))}
    </div>
  </div>
)}
```

`className="contents"` 트릭 — div 가 grid layout 에서 자기 자리 차지 안 하고 자식만 grid item 처럼 동작. key prop 을 가지면서도 layout 영향 X.

### 우클릭 메뉴 — Portal 트릭

`position: fixed` 를 그냥 쓰면 dockview 패널이 `transform` 으로 위치를 잡고 있어서 **containing block 이 viewport 가 아닌 패널 박스가 됨** (CSS 명세). 그래서 메뉴가 패널 우하단으로 짓밀림.

해결: **`createPortal` 로 `document.body` 에 직접 렌더**.

```tsx
{menu && createPortal(
  <div
    className="fixed z-50 ..."
    style={{ left: menu.x, top: menu.y }}
    onMouseDown={(e) => e.stopPropagation()}   // 외부 클릭 닫기와 분리
  >
    <MenuItem label="Run to here" onClick={onMenuRunTo} disabled={!isPaused} />
    <MenuItem label={breakpointSet.has(menu.stepId) ? "Remove breakpoint" : "Add breakpoint"} ... />
  </div>,
  document.body,
)}
```

`onMouseDown stopPropagation` 가 없으면 메뉴 자체를 클릭해도 외부 mousedown 리스너가 닫아버림. 메뉴 자체의 mousedown 은 close 트리거에서 제외해야 함.

외부 닫기 useEffect:

```ts
useEffect(() => {
  if (!menu) return;
  const close = () => setMenu(null);
  const onKey = (e: KeyboardEvent) => { if (e.key === "Escape") setMenu(null); };
  window.addEventListener("mousedown", close);
  window.addEventListener("keydown", onKey);
  return () => { /* cleanup */ };
}, [menu]);
```

### Preview 흐름 — PromptPanel

[PromptPanel.tsx](../frontend/src/components/panels/PromptPanel.tsx) 의 마운트 시 1회 자동 preview:

```ts
const bridgeConnected = useSystemStore((s) => s.bridgeConnected);
const didInitialPreview = useRef(false);

useEffect(() => {
  if (!bridgeConnected) return;            // ★ 핵심: WebSocket 연결 후
  if (didInitialPreview.current) return;
  didInitialPreview.current = true;
  handlePreview();
}, [bridgeConnected, handlePreview]);
```

처음에 `bridgeConnected` 무시하고 mount 즉시 호출했었는데 — `bridge.callService` 가 WebSocket 안 붙은 상태에서 호출되면 silent fail. `bridgeConnected=true` 가 된 후 1회만 발화하는 패턴이 안전.

이후 prompt 변경 시 자동 갱신 안 함 — 사전에 박은 breakpoint 가 새 트리에서 dangling 되는 혼란을 피함. 사용자가 명시적으로 "Preview" 버튼 (Detect 옆) 눌러야 갱신:

```tsx
<button onClick={handlePreview} disabled={isActive || previewing || !prompt.trim()}>
  <Eye className="w-3 h-3" />
  {previewing ? "..." : "Preview"}
</button>
```

---

## 미래 확장 — ForEach / If 흡수

[ideas.md §LLM orchestrator](ideas.md#llm-task-orchestrator-) 의 primitive 도입 시점에 무엇이 필요한가:

### Backend

- `ForEachStep`, `IfStep` 같은 control flow dataclass 추가 — `children: list[Step]` (또는 `then` / `else`) 필드
- `Task.__post_init__` 가 자식 step 들에 재귀적으로 `"step-3.0"`, `"step-3.1"` path-based id 부여
- `step_to_dict` / `task_tree` 가 자식 직렬화 (dataclass field 면 asdict 자동)
- `StepExecutor` 에 control flow 핸들러 — `_for_each(step, context)` 가 `step.children` 을 N 번 순회
- `TaskRunner._run_task` 가 평면 list 가 아닌 트리 순회로 변경 — pre-order traversal + ForEach 의 iteration 추적

가장 어려운 부분은 `TaskRunner` 의 트리 순회. 지금은 단순 for loop. 트리가 되면:
- 어떤 step.id 가 "현재 실행 중" 인지 (path 어디까지 들어가있는지)
- breakpoint 가 ForEach 안 step 에 박힐 때 — 매 iteration 마다 멈출지, 첫 iteration 만 멈출지

이건 그때 가서 풀 문제. **지금 데이터 모델은 그걸 받아들일 수 있는 모양** 으로 박혀있음 — 이게 핵심.

### Frontend

- `StepRow` 가 `node.children` 있으면 자식들 재귀 렌더 + 들여쓰기 padding 증가. 지금 코드를 약간만 손보면 됨:

```tsx
{taskTree.steps.map((node) => <StepRow node={node} depth={0} ... />)}

// StepRow 안:
<div style={{ paddingLeft: depth * 12 }}> ... </div>
{node.children?.map((child) => <StepRow node={child} depth={depth+1} ... />)}
```

- 디테일 표시 (`paramEntries`) 는 인덱스 시그니처 덕에 ForEach 의 `iter_var`, `iter_count` 같은 새 필드도 자동 표시.

- 컨텍스트 메뉴는 step id 기반이라 트리 깊이와 무관 — 그대로 동작.

---

## 코드 anchor 정리 — 학습 진입점

| 학습 대상 | 위치 |
|---|---|
| Step DSL 의 id 필드 + 자동 부여 | [step_types.py](../backend/modules/task/step_types.py) |
| TaskRunner 의 디버거 게이트 | [task_runner.py:_should_pause_before](../backend/modules/task/task_runner.py#L218-L242) |
| TaskRunner 의 외부 API (resume/step_once/run_to/toggle_breakpoint) | [task_runner.py](../backend/modules/task/task_runner.py#L116-L160) |
| 토픽 / 서비스 키 (백엔드) | [topic_map.py](../backend/core/topic_map.py) |
| 토픽 / 서비스 키 (프론트) | [topics.ts](../frontend/src/constants/topics.ts) |
| Service 핸들러 (위임 패턴) | [task_node.py:_handle_step](../backend/nodes/task_node.py) |
| Tree publish 타이밍 | [task_node.py:_handle_run](../backend/nodes/task_node.py) |
| Preview 가드 (실행 중 거절) | [task_node.py:_handle_preview](../backend/nodes/task_node.py) |
| Frontend types (`StepNode`, `TaskTree`) | [types/task.ts](../frontend/src/types/task.ts) |
| `TASK_TREE` 구독 | [useBridge.ts](../frontend/src/hooks/useBridge.ts) |
| 서비스 1:1 노출 hook | [useTask.ts](../frontend/src/hooks/useTask.ts) |
| TaskPanel 전체 | [TaskProgressPanel.tsx](../frontend/src/components/panels/TaskProgressPanel.tsx) |
| 호버 prefill breakpoint gutter | [TaskProgressPanel.tsx](../frontend/src/components/panels/TaskProgressPanel.tsx) StepRow |
| Portal 우클릭 메뉴 | [TaskProgressPanel.tsx](../frontend/src/components/panels/TaskProgressPanel.tsx) 의 createPortal 부분 |
| 마운트 시 1회 자동 preview | [PromptPanel.tsx](../frontend/src/components/panels/PromptPanel.tsx) 의 `bridgeConnected` useEffect |

---

## 알려진 한계 / 미해결

- **breakpoint 가 task 종료 후에도 잔존** — TaskRunner._breakpoints 는 `run()` 시점에 초기화 안 함. 의도적 (사전 박기 지원) 이지만 사용자가 "다음 cycle 부터 깨끗하게" 시작하고 싶을 땐 직접 토글해서 풀어야 함.
- **preview 와 실제 run 결과가 다를 수 있음** — preview 시 LLM prompt_parser 가 한 번, run 시 또 한 번 호출. parser 가 비결정적이면 두 결과가 다를 수 있음. picky 의 단순 2-슬롯 추출은 영향 작지만 미래 복잡한 task 에선 캐시 필요할 수도.
- **트리가 평면일 때만 검증됨** — 깊이 1 트리에 대한 동작만 실제 테스트. ForEach 등 control flow 가 들어왔을 때 TaskRunner 의 트리 순회는 추가 설계 + 구현 필요.
- **step 실패 시 retry 불가** — FAILED step 에서 task 가 끝남. retry / skip step 같은 IDE 디버거 의 "이 줄 다시 실행" 패턴은 미구현.
