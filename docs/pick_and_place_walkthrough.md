# Pick and Place 따라가기 — 수학 약해도 OK

> **이 문서가 누구를 위한 건가**: 프론트엔드의 "Run" 버튼이 눌리는 순간부터 실제 모터가 돌고, 상태가 다시 UI로 돌아오기까지의 *왕복 흐름*을 코드 한 줄 → 알고리즘 → 수학까지 위에서 아래로 내려가며 이해하고 싶은 사람.
>
> [tsdf_walkthrough.md](tsdf_walkthrough.md)의 자매편. 수학 표기 최소화, 직관 + 그림 + 구체적 숫자 → 그다음 수식. 매 챕터 끝에 정확한 코드 줄 번호.
>
> **읽는 법**: 위에서부터 순서대로. 좌표계·핀홀 모델 같은 공통 챕터는 TSDF 문서로 짧게 넘김.
>
> **사전 지식**: TSDF 워크스루의 §1 (좌표계 + 변환행렬), §3 (핀홀 모델) 정도. 행렬 곱이 무엇인지만 알면 충분.

---

## 0. 전체 한눈에

OMX_F의 pick-and-place는 한 마디로 *프론트엔드의 버튼 → 백엔드의 12개 step → 모터 → 다시 상태 환류*의 왕복. 다음 표를 머리에 박고 시작하자.

| # | 어디서 | 한 줄 요약 | 챕터 |
|---|--------|----------|------|
| 1 | Frontend | "Run" 버튼 → `bridge.callService(TASK_RUN, {task, prompt})` → WebSocket | §1 |
| 2 | Bridge   | WS message → Zenoh queryable 호출 → TaskNode | §1 |
| 3 | TaskNode | TASK_REGISTRY에서 task factory 호출 → `TaskRunner.run(task)` | §3 |
| 4 | TaskRunner | step 리스트를 한 개씩 `StepExecutor.execute(step)` | §4 §5 |
| 5 | DetectorNode | Grounding DINO + depth → 객체 위치 + **height** | §6 |
| 6 | StepExecutor | `_grasp_policy`: height 보고 grasp z 결정 | §7 |
| 7 | MotionNode + Solver | IK가 조인트각 5개를 풀어냄 (DLS) | §8 |
| 8 | TrajectoryRunner | Ruckig으로 50Hz 부드러운 명령 stream | §9 |
| 9 | MotorNode → Dynamixel | 모터 회전 (그리퍼는 부드러운 profile) | §10 |
| 10 | Backend → Frontend | `TASK_STATE`/`MOTION_STATE_TRAJ` 토픽으로 UI 갱신 | §2 |

§1과 §2가 *프론트엔드 양끝*, §3~§10이 *백엔드 본체*. **§7 GraspPolicy**가 detect 결과를 받아 grasp z를 동적으로 결정하는 *새로 추가된 정책 step*.

---

## 1. Frontend → Backend: "Run" 버튼이 누른 한 줄

`PromptPanel.tsx` ([frontend/src/components/panels/PromptPanel.tsx:30-36](frontend/src/components/panels/PromptPanel.tsx#L30-L36))에서 사용자가 Run을 누르면:

```typescript
await run({ task: "pick_and_place", prompt: trimmed });
```

> *현재 PromptPanel은 `pick_named_object`를 부르지만 — 이 문서는 GroundedDetect+grasp policy로 전환된 `pick_and_place`를 기준으로 설명.*

`run`은 `useTask` 훅 ([frontend/src/hooks/useTask.ts:20-34](frontend/src/hooks/useTask.ts#L20-L34))의 한 줄:

```typescript
const res = await bridge.callService(
  ServiceKey.TASK_RUN,
  req as unknown as Record<string, unknown>,
);
```

여기서 `bridge`가 무엇을 하는지가 이 챕터의 핵심.

### 1.1 BridgeClient — WebSocket을 RPC로 감싼 얇은 층

`bridge`는 싱글톤 ([frontend/src/api/bridge.ts:245](frontend/src/api/bridge.ts#L245)). 브라우저는 Zenoh를 직접 못 쓰니까, `BridgeClient`가 WebSocket으로 백엔드 bridge에 RPC를 보내는 역할.

`callService`의 본체 ([frontend/src/api/bridge.ts:197-229](frontend/src/api/bridge.ts#L197-L229)):

```typescript
callService(key, data, options): Promise<{success, message, data}> {
  return new Promise((resolve) => {
    const request_id = makeRequestId();              // UUID
    this.pendingServices.set(request_id, resolve);   // promise resolver 저장
    this._send({                                     // WS 텍스트 메시지
      type: "service",
      key,                                            // "omx/task/srv/run"
      request_id,
      data,
      timeout: timeoutMs / 1000,
    });

    setTimeout(() => {                                // 타임아웃 후 fail resolve
      if (this.pendingServices.has(request_id)) {
        resolve({success: false, message: "타임아웃", data: {}});
      }
    }, timeoutMs);
  });
}
```

핵심 아이디어:
1. **request_id로 promise를 보관**해서, 응답이 비동기로 와도 정확히 그 `await`에 매칭.
2. WebSocket 한 줄에 모든 service를 다중화 — 별도 HTTP 연결 안 씀. 응답 메시지가 도착하면 `_handleIncoming`이 `request_id`로 resolver를 찾아 promise를 깨움.

### 1.2 Bridge 서버 (FastAPI) — WS ↔ Zenoh 변환

WebSocket으로 들어온 `{type: "service", key, ...}` 메시지는 [backend/bridge/zenoh_bridge.py](backend/bridge/zenoh_bridge.py)가 받아 *Zenoh queryable*에 동일한 호출로 변환:

```
브라우저                  bridge (FastAPI)              Zenoh                TaskNode
   │                          │                          │                     │
   │ "service" WS msg         │                          │                     │
   ├─────────────────────────>│                          │                     │
   │                          │  session.get(            │                     │
   │                          │     "omx/task/srv/run")  │                     │
   │                          ├─────────────────────────>│                     │
   │                          │                          │    handler 호출     │
   │                          │                          ├────────────────────>│
   │                          │                          │<────────────────────┤
   │                          │<─────────────────────────┤                     │
   │ "service_response"       │                          │                     │
   │<─────────────────────────┤                          │                     │
```

같은 패턴이 **모든 service**에 적용. 브라우저에선 *그냥 RPC 한 번*, 백엔드에선 *queryable 호출 한 번*.

### 1.3 코드에서 어디?

- Run 버튼 핸들러: [PromptPanel.tsx:30-36](frontend/src/components/panels/PromptPanel.tsx#L30-L36)
- useTask hook: [hooks/useTask.ts:20-34](frontend/src/hooks/useTask.ts#L20-L34)
- callService: [api/bridge.ts:197-229](frontend/src/api/bridge.ts#L197-L229)
- 서버 측 WS↔Zenoh: [backend/bridge/zenoh_bridge.py](backend/bridge/zenoh_bridge.py)

---

## 2. Backend → Frontend: 상태 환류

`TASK_RUN`은 즉시 `{success: true}` 응답 후 비동기로 task가 시작됨. 그동안 *현재 어느 step인지, 어디까지 진행됐는지*를 UI에 어떻게 보여줄까? — **토픽 구독**.

### 2.1 useBridge 한 곳에서 모든 토픽 라우팅

`App.tsx`가 마운트될 때 [useBridge](frontend/src/hooks/useBridge.ts)가 한 번 호출돼 *모든 백엔드 토픽을 zustand store로 라우팅*하는 구독을 건다.

pick-and-place에 관련된 핵심 구독 4개:

```typescript
// Task 상태 (idle/running/paused/success/failed/stopped + step 번호)
bridge.subscribe(Topic.TASK_STATE, (data) => setTaskState(data));

// Trajectory 진행률 (각 MoveL/MoveJ의 0~1 progress)
bridge.subscribe(Topic.MOTION_STATE_TRAJ, (data) => setTrajectoryState(data));

// Detector 라이브 (5fps YOLO 결과 — 화면 위 bbox 표시용)
bridge.subscribe(Topic.DETECTOR_STATE, (data) => setDetections(...));

// Grounded detect 결과 (text prompt 매칭 결과 — 3D 마커용)
bridge.subscribe(Topic.PERCEPTION_GROUNDED_STATE, (data) => setGroundedResult(...));
```

토픽이 도착할 때마다 *해당 store만 갱신*. React가 store 구독해서 자동 리렌더.

### 2.2 양방향 흐름 한눈에

```
                Frontend                          Backend
┌─────────────────────────────┐    ┌──────────────────────────────────┐
│ PromptPanel (Run 버튼)      │    │                                  │
│      │                      │    │                                  │
│      ▼                      │    │                                  │
│  bridge.callService(        ├───►│  bridge/zenoh_bridge.py          │
│    TASK_RUN,                │    │  ─► Zenoh.get("omx/task/srv/run")│
│    {task, prompt})          │    │     ─► TaskNode._handle_run      │
│                             │    │        ─► TaskRunner.run(task)   │
│                             │◄───┤   {success: true}                │
│                             │    │                                  │
│  (이제부터 상태가 흘러옴)   │    │  TaskRunner 매 step:             │
│  useBridge.subscribe(       │◄───┤    publish TASK_STATE             │
│    TASK_STATE / TRAJ /      │    │                                  │
│    PERCEPTION_GROUNDED)     │    │  MotionNode 매 50Hz tick:        │
│   ─► zustand store          │◄───┤    publish MOTION_STATE_TRAJ     │
│      ─► 패널 자동 리렌더    │    │                                  │
└─────────────────────────────┘    └──────────────────────────────────┘
```

- *명령*: HTTP-스타일 RPC (요청 → 응답). callService.
- *상태*: pub/sub stream. subscribe + store 갱신.

---

## 3. TaskNode + TASK_REGISTRY — task 이름이 factory가 되는 곳

[task_node.py](backend/nodes/task_node.py)의 `_handle_run`이 받아 처리:

```python
def _handle_run(self, req: dict) -> dict:
    data = req.get("data", {})
    task_name = data.get("task", "pick_and_place")

    factory = TASK_REGISTRY.get(task_name)            # _factory_pick_and_place
    task = factory(data)                              # Task 인스턴스 생성
    self._runner.run(task)                            # TaskRunner 스레드 시작
    return {"success": True, "message": "ok", "data": {}}
```

factory ([task_node.py:24-32](backend/nodes/task_node.py#L24-L32)):

```python
def _factory_pick_and_place(data: dict) -> Task:
    prompt = str(data.get("prompt", "")).strip()
    if not prompt:
        raise ValueError("prompt 필요")
    place = data.get("place_position", DEFAULT_PLACE_POSITION)
    return create_pick_and_place_task(
        prompt=prompt,
        place_position=Position3(place),
    )
```

frontend가 보낸 `prompt` / `place_position` 인자를 받아 `create_pick_and_place_task`를 호출. 새 task를 추가하려면 (1) factory 작성, (2) TASK_REGISTRY 등록.

---

## 4. Task = 선언형 step 리스트

`create_pick_and_place_task` ([pick_and_place.py:23-71](backend/modules/task/tasks/pick_and_place.py#L23-L71))를 보면 사실상 **데이터 구조** 하나를 반환할 뿐:

```python
return Task(
    name="pick_and_place",
    description="...",
    steps=[
        GripperStep(action="open", label="open_gripper"),
        GroundedDetectStep(prompt=prompt, output_key="object_pos", label=...),
        GraspPolicyStep(input_key="object_pos", output_key="grasp_xyz", label=...),
        MoveTCPStep(position_key="grasp_xyz", offset=(0,0,PRE_GRASP_DZ), label="pre_grasp"),
        MoveTCPStep(position_key="grasp_xyz", offset=(0,0,0), label="grasp"),
        GripperStep(action="close", verify_grasp=True, label="close_gripper"),
        WaitStep(duration_sec=0.5, label="grip_settle"),
        MoveTCPStep(position_key="grasp_xyz", offset=(0,0,LIFT_DZ), label="lift"),
        MoveTCPStep(position=place_position, label="move_to_place"),
        GripperStep(action="open", label="release"),
        WaitStep(duration_sec=0.3, label="release_settle"),
        HomeStep(label="return_home"),
    ],
)
```

전부 [step_types.py](backend/modules/task/step_types.py)의 **dataclass**. 단지 *무엇을 어떤 인자로 할지 적은 종이*. 직접 모터를 건드리는 코드는 한 줄도 없다.

### 4.1 이게 왜 좋은가

1. **테스트 / 재사용 쉬움**: Task 객체는 dataclass 묶음이라 직렬화/저장/재생산 가능.
2. **실행 방식 분리**: "어떻게" 실행할지는 `TaskRunner`가 결정. Task 정의는 그 결정과 무관.

이런 패턴이 **declarative DSL** (선언형 도메인 특화 언어). SQL이나 HTML도 같은 맛.

### 4.2 step의 세 종류 — 입력 / 출력 / 변환

이번 task의 흐름을 따라가면 세 종류로 나뉜다:

- **출력 step (생산자)**: `GroundedDetectStep(output_key="object_pos", ...)` — context에 *객체 위치* 저장
- **변환 step**: `GraspPolicyStep(input_key="object_pos", output_key="grasp_xyz", ...)` — 입력을 읽어 *grasp 위치*로 변환해 저장
- **입력 step (소비자)**: `MoveTCPStep(position_key="grasp_xyz", offset=..., ...)` — 변환된 위치에서 꺼내 씀

이 *세 종류가 context dict를 통해 연결*되는 게 pick&place의 데이터 흐름 골격. detect가 본 raw 위치를 정책이 grasp 위치로 가공하고, 후속 move 4개가 같은 grasp 위치에 offset만 변주.

### 4.3 offset의 의미

같은 `grasp_xyz`를 4번 재사용하면서 Z만 바꿔 *pre_grasp(+6cm) → grasp(0) → lift(+8cm)*. 한 번 결정한 grasp 위치를 여러 깊이로 변주.

```python
position = [b + o for b, o in zip(base_pos, step.offset)]    # step_executor.py:99
```

### 4.4 코드에서 어디?

- Task 정의: [pick_and_place.py](backend/modules/task/tasks/pick_and_place.py)
- step dataclass: [step_types.py](backend/modules/task/step_types.py)

---

## 5. TaskRunner + StepExecutor — 메인 루프와 디스패치

`TaskRunner.run(task)`는 *백그라운드 스레드를 띄워* `_run_task`를 실행 ([task_runner.py:74-89](backend/modules/task/task_runner.py#L74-L89)). 즉시 반환하니까 `_handle_run`이 곧바로 `{success: true}`를 응답.

`_run_task` 본체:

```python
def _run_task(self, task: Task) -> None:
    context = TaskContext()       # 빈 dict 컨테이너

    for i, step in enumerate(task.steps):
        if self._stop_event.is_set(): ...   # stop 체크
        self._pause_event.wait()             # pause 대기

        ok = self._executor.execute(step, context, self._stop_event)
        if not ok: return FAILED
```

### 5.1 두 개의 Event — pause/stop

`threading.Event`는 *스레드 간 신호 깃발*. `set()`이면 통과, `clear()`면 대기.

- `_pause_event`: 기본 set (통과). `pause()`가 clear, `resume()`이 다시 set.
- `_stop_event`: 기본 clear. `stop()`이 set하면 *다음 step 직전* 감지돼 break.

이 *순서가 보장하는 의미*: pick-and-place 한 step 중간에 멈추진 않는다. 안전 디자인.

### 5.2 StepExecutor — step type별 디스패치

`TaskRunner`는 "다음 step 실행해" 단계만 결정. *어떻게*는 [step_executor.py:69-86](backend/modules/task/step_executor.py#L69-L86)의 `execute`가 type별 분기:

```python
def execute(self, step, context, stop_event) -> bool:
    match step.type:
        case "move_tcp":      return self._move_tcp(step, context)
        case "gripper":       return self._gripper(step)
        case "detect":        return self._detect(step, context)
        case "grounded_detect": return self._grounded_detect(step, context)
        case "grasp_policy":  return self._grasp_policy(step, context)
        case "wait":          return self._wait(step)
        case "home":          return self._home(step)
```

각 핸들러는 *얇은 어댑터* — `call_service`로 motion/motor/detector 노드에 RPC를 던지고 결과로 bool을 돌려줌.

```
GroundedDetectStep  →  PERCEPTION_GROUNDED_DETECT  + context.set(output_key, ...)
GraspPolicyStep     →  (서비스 호출 없음, context 계산)
MoveTCPStep         →  MOTION_MOVE_L              + traj 완료 대기
GripperStep         →  MOTOR_GRIPPER
HomeStep            →  MOTION_MOVE_J              + traj 완료 대기
WaitStep            →  time.sleep
```

### 5.3 traj 완료 대기 — Event 두 번째 등장

MoveTCP / Home 같은 *시간이 걸리는* step은 서비스를 호출한 후 **트래젝토리 완료 신호**를 기다림.

```python
self._traj_event = threading.Event()
self._node.create_subscriber(Topic.MOTION_STATE_TRAJ, self._on_traj_state)

# _move_tcp에서
self._traj_event.clear()
self._node.call_service(Service.MOTION_MOVE_L, {"position": position})
return self._wait_for_traj()       # event가 set될 때까지 최대 30s

# 콜백
def _on_traj_state(self, data):
    if data["status"] in (DONE, FAILED, STOPPED):
        self._traj_event.set()
```

같은 `MOTION_STATE_TRAJ` 토픽이 *backend 동기화*와 *frontend progress bar*에 동시 흐름. 다대다 pub/sub의 장점.

### 5.4 단순 step — Gripper / Wait / Home

본격 챕터들 전에 짧게.

**GripperStep** ([step_executor.py](backend/modules/task/step_executor.py)):

```python
res = self._node.call_service(Service.MOTOR_GRIPPER,
                              {"action": step.action, "current": step.current})
time.sleep(GRIPPER_SETTLE)

# verify_grasp=True 면 close 직후 Present_Position 확인
if step.action == "close" and step.verify_grasp:
    pos = self._joint_cache.get_raw(GRIPPER_ID)
    if pos < GRIPPER_HELD_THRESHOLD:   # 1900
        return False                   # 빈손 → step fail → task fail
```

`MOTOR_GRIPPER` 서비스가 motor_node에서 ID 6번 그리퍼 모터를 *current control mode*로 토크 명령. `current` (mA)이 *파지력*.

**잡힘 검증 (`verify_grasp`)**: Current-based Position mode 라 close 명령에 큐브가 막히면 그 자리에서 멈추고, 빈손이면 `GRIPPER_CLOSE_RAW`(1800) 까지 끝까지 닫힘. 그래서 close 직후 `Present_Position` 이 `GRIPPER_HELD_THRESHOLD`(1900) 미만이면 빈손으로 판정 → step fail → task fail. 이게 없으면 빈손이어도 step service call 은 success 라서 task 가 그대로 place 까지 가서 "성공" 으로 끝남.

**그리퍼 부드러운 동작**: 이전엔 `set_goal_position`만 호출해서 *Dynamixel 기본값 = 0 = 최대 속도*로 휙 움직였음. [motor_node.py:74-87](backend/nodes/motor_node.py#L74-L87)의 `_apply_gripper_smooth_profile()`이 시작 시 한 번 `profile_velocity = 80`, `profile_acceleration = 30`을 설정 → 이후 모든 goal_position 명령에 *사다리꼴 ramp* 적용:

```
position
   ▲                     ┌──── 목표
   │                    ╱
   │                   ╱      ← acc phase (profile_acc)
   │       ─────────/         ← constant velocity (profile_vel)
   │             ╱
   │   ──────╱                ← dec phase
   └────────────────────────► time
```

`profile_velocity` 단위가 0.229 rpm이라 `80 ≈ 18 rpm` → full stroke 약 1.5초. 부드럽게 잡고 부드럽게 놓음. reboot 시에도 `_srv_reboot`가 재적용 ([motor_node.py:198-203](backend/nodes/motor_node.py#L198-L203)).

**WaitStep**: 진짜로 그냥 sleep. `grip_settle`(0.5s) / `release_settle`(0.3s).

**HomeStep** ([step_executor.py:235-249](backend/modules/task/step_executor.py#L235-L249)): 미리 저장된 "home" pose를 `MOTION_MOVE_J`로 — *조인트 공간 보간*. MoveL과 달리 TCP가 직선으로 안 감.

---

## 6. GroundedDetectStep — 텍스트 prompt → 객체 3D 위치 + height

이 챕터가 *기존 YOLO+평면 가정* 경로를 대체. 핵심 차이:

| | (예전) `_handle_detect` | (현재) `_handle_grounded_detect` |
|--|--|--|
| 모델 | YOLO (객체 class) | Grounding DINO (텍스트 매칭) |
| 입력 | (없음, 화면 전체 스캔) | `prompt: str` |
| 카메라 | color만 | color + **depth** |
| Z 회수 | base Z=0 평면 가정 | bbox 영역 depth median (실측) |
| 출력 | 객체 위치 | 위치 + base_z + **height** |

`DetectorNode._handle_grounded_detect` ([detector_node.py:211-378](backend/nodes/detector_node.py#L211-L378))가 푸는 게 6단계.

### 6.1 단계 1: depth stream 확보 (on-demand)

D405는 color stream은 항상 켜놓지만 *depth는 별도 enable* 필요. 이게 이 챕터에서 *유일하게 복잡한 코디네이션*:

```python
need_enable = True
with self._depth_lock:
    f = self._latest_depth_frame
    if f is not None and (time.time() - f.timestamp) < 1.0:
        need_enable = False   # 다른 노드가 이미 켜놨음

if need_enable:
    self.call_service(CAMERA_SET_DEPTH_STREAM, {"enabled": True})
    # 새 frame 한 장 polling 대기 (max 5s)
```

**disable은 절대 안 함**. PointCloudNode 등 다른 소비자가 동시에 쓸 수 있어서. 패턴: *공용 전등 — 켜는 건 idempotent, 끄는 건 아무도 안 함*.

### 6.2 단계 2: Grounding DINO inference

```python
det = self._grounded.detect(depth_frame.color_bgr, prompt)    # color만 들어감
(x1, y1, x2, y2), score = det
```

**Grounding DINO는 *color만 보고* bbox를 찾음**. depth는 아직 안 쓰임. text prompt + image → text가 매칭되는 bbox 1개.

### 6.3 단계 3: bbox 영역 depth → Z_cam

```python
roi = depth_frame.depth_z16[iy1:iy2, ix1:ix2]
valid = roi[roi > 0]
top_raw = float(np.percentile(valid, 25))     # 카메라 가까운 25% = 객체 윗면
Z_cam = top_raw * depth_frame.depth_scale
```

핀홀 모델 +평면 가정 *없음*. depth가 *그 픽셀에서 실제로 카메라까지 거리*를 직접 알려줌. percentile 25를 쓰는 이유: bbox 안에는 *객체 표면 + 배경(책상)* depth가 섞여 있는데, *카메라에 가까운 부분*(낮은 depth값 = 윗면)을 robust하게 뽑기 위해.

### 6.4 단계 4: 객체 height 추정 (이 챕터의 핵심)

depth가 있으니까 가능한 *추가 정보*:

```python
# bbox 외곽 ring (30px PAD - 내부 제거) = 책상 표면
PAD = 30
ext_roi = depth_frame.depth_z16[ey1:ey2, ex1:ex2].copy()
ext_roi[bbox 내부] = 0
base_raw = float(np.percentile(ext_valid, 75))    # 카메라 먼 75% = 책상

height_cam = (base_raw - top_raw) * depth_frame.depth_scale
```

직관:
```
       카메라 (위)
         │
         ▼
    ╔════╗       ← top_raw (bbox 내부, depth 작음)
    ║obj ║
════╝    ╚════   ← base_raw (외곽 ring, depth 큼)
   책상
       
height = base_raw - top_raw
```

이 height가 §7 GraspPolicy의 핵심 입력. height 없이는 정책 자체가 불가능.

### 6.5 단계 5: unproject + hand_eye → base 좌표

```python
u, v = (x1 + x2)/2.0, (y1 + y2)/2.0           # bbox 중심
X_cam = (u - cx) / fx * Z_cam                  # 핀홀 모델 역 (TSDF 워크스루 §3.2)
Y_cam = (v - cy) / fy * Z_cam
obj_in_cam = np.array([X_cam, Y_cam, Z_cam])

# TCP pose (FK)
R_be, t_be = ...

# hand_eye + FK 합성 (TSDF 워크스루 §1.5)
obj_in_ee  = R_ce @ obj_in_cam + t_ce
obj_in_base = R_be @ obj_in_ee + t_be
```

핀홀 역모델은 TSDF 워크스루 §3.2와 동일. 다른 점: *Z를 가정으로 푸는 게 아니라 측정값으로* 받는 것.

### 6.6 단계 6: payload 구성 + broadcast

```python
result_payload = {
    "prompt": prompt,
    "position": obj_in_base.tolist(),  # 객체 윗면 base xyz
    "bbox2d": {...},
    "confidence": score,
    "base_z": base_z,        # 책상 z (= 객체 윗면 z - height)
    "height": height,
    "timestamp": ...,
}
self.publish(Topic.PERCEPTION_GROUNDED_STATE, result_payload)
return {"success": True, "message": "ok", "data": result_payload}
```

**같은 데이터를 두 군데로**:
1. `publish` → 토픽 broadcast (frontend 시각화)
2. `return` → RPC 호출자 (StepExecutor)

이 분리 디자인의 의미: *호출자가 누구든* (PromptPanel의 Detect 버튼, GroundedDetectStep, self-play 등) frontend가 일관되게 시각화. 호출자별 보일러플레이트 없음.

### 6.7 step_executor의 context.set

StepExecutor `_grounded_detect` ([step_executor.py:148-187](backend/modules/task/step_executor.py#L148-L187))가 결과를 context dict에 저장:

```python
context.set(step.output_key, position)                       # "object_pos" → [x, y, z]
context.set(f"{step.output_key}_meta", {
    "base_z": float(data.get("base_z", 0.0)),
    "height": float(data.get("height", 0.0)),
})                                                            # "object_pos_meta" → {base_z, height}
```

position과 별도로 `_meta` suffix 키에 base_z/height 저장. 다음 챕터 §7의 GraspPolicy가 이 둘을 읽음.

> *YOLO + 평면 가정 경로 `_handle_detect`는 코드에 남아 있고 `DetectStep`이 호출함. 가벼운 디버깅이나 depth stream 비용을 피하고 싶을 때 유용. 현재 pick_and_place는 GroundedDetect를 씀.*

---

## 7. GraspPolicyStep — height 보고 grasp z 결정

이 챕터는 **수학 없음, 1줄짜리 정책**. 그러나 디자인적으로 중요 — *detect 결과를 어떻게 동적으로 변환*해서 후속 MoveTCP에 넘기느냐.

### 7.1 왜 정책이 필요한가

GroundedDetect의 출력 `position`은 *객체 윗면* base xyz. 그러면 grasp는 어디서? 옆면 중간:

```
        ┌──────┐
        │      │
        │ ←──── grasp 여기 (옆면 중간 = base_z + height * 0.5)
        │      │
        │      │
     ═══└──────┘═══
           ↑ base_z
```

정책 한 줄: `grasp_z = base_z + height * 0.5`.

> *이전엔 `height < 4cm` 면 "얇음" 으로 분류해서 윗면 누르기(top_z - 5mm) 로 갔는데, 20mm 큐브가 thin 분기로 빠져 손가락이 큐브 위 공중에서 close 되는 문제가 있었음. 그리퍼 손가락 두께상 위에서 누르기는 카드/시트가 아닌 한 거의 안 됨 → thin 분기 폐기.*

### 7.2 GraspPolicyStep dataclass

[step_types.py](backend/modules/task/step_types.py):

```python
@dataclass
class GraspPolicyStep:
    input_key: str = "detected_position"
    output_key: str = "grasp_xyz"
    grasp_ratio: float = 0.5       # height의 절반 (옆면 중간)
    label: str = ""
    type: Literal["grasp_policy"] = field(default="grasp_policy", ...)
```

`grasp_ratio` 만 노출. 객체별로 *위쪽 잡기* 하고 싶으면 `grasp_ratio=0.7`, *아래쪽 잡기* 면 `0.3` 같은 변주.

### 7.3 _grasp_policy 핸들러

[step_executor.py](backend/modules/task/step_executor.py):

```python
def _grasp_policy(self, step: GraspPolicyStep, context: TaskContext) -> bool:
    pos = context.get(step.input_key)
    if not isinstance(pos, (list, tuple)) or len(pos) < 3: return False

    meta_raw = context.get(f"{step.input_key}_meta")
    meta: dict = meta_raw if isinstance(meta_raw, dict) else {}
    base_z = float(meta.get("base_z", 0.0))
    height = float(meta.get("height", 0.0))

    x, y, _top_z = float(pos[0]), float(pos[1]), float(pos[2])

    grasp_z = base_z + height * step.grasp_ratio   # 옆면 그립

    context.set(step.output_key, [x, y, grasp_z])
    return True
```

`context.get(input_key)`로 GroundedDetect가 저장한 *위치*를, `context.get(input_key + "_meta")`로 *meta 정보(base_z/height)*를 꺼냄. 정책 1줄 실행. 결과 [x, y, grasp_z]를 *output_key*로 저장 → 후속 MoveTCPStep이 `position_key="grasp_xyz"`로 받음.

### 7.4 task 안에서의 흐름

```python
GroundedDetectStep(prompt="cube", output_key="object_pos")
# → context["object_pos"]      = [x, y, top_z]
# → context["object_pos_meta"] = {"base_z": ..., "height": ...}

GraspPolicyStep(input_key="object_pos", output_key="grasp_xyz")
# → context["grasp_xyz"] = [x, y, grasp_z]   (정책 적용)

MoveTCPStep(position_key="grasp_xyz", offset=(0, 0, 0.06))   # pre_grasp
MoveTCPStep(position_key="grasp_xyz", offset=(0, 0, 0.0))    # grasp
MoveTCPStep(position_key="grasp_xyz", offset=(0, 0, 0.08))   # lift
```

`grasp_xyz`가 *세 MoveTCP의 공통 base*. offset만 다르게.

### 7.5 코드에서 어디?

- 정책 dataclass: [step_types.py](backend/modules/task/step_types.py) `GraspPolicyStep`
- 정책 핸들러: [step_executor.py:189-217](backend/modules/task/step_executor.py#L189-L217)
- task에서 사용: [pick_and_place.py](backend/modules/task/tasks/pick_and_place.py)

---

## 8. MoveTCPStep — IK가 풀어내는 것

`MoveTCPStep`은 *base 좌표계의 목표 XYZ에 EE를 옮긴다*. step_executor가 하는 일은 단순:

```python
position = [b + o for b, o in zip(base_pos, step.offset)]
res = self._node.call_service(Service.MOTION_MOVE_L, {"position": position})
return self._wait_for_traj()
```

MoveL 서비스 안에서:

1. **MotionCommand**: 목표 검증 → `LinearPath(start, end)` 만들어 `TrajectoryRunner.run_cartesian()`에 넘김
2. **TrajectoryRunner**: 50Hz 루프로 시간에 따라 LinearPath를 따라가며 IK → MOTOR_CMD_JOINT publish (§9)

이 챕터는 IK가 푸는 문제에 집중.

### 8.1 IK가 하는 일

문제: *base 좌표계의 (x, y, z)가 주어졌을 때, EE가 정확히 거기에 가도록 조인트 각도 5개를 찾아라.*

```
주어진:   p_target ∈ ℝ³
찾기:    θ = (θ₁, θ₂, θ₃, θ₄, θ₅)  such that  FK(θ) = p_target
```

OMX_F는 5DOF — 5개 변수로 3차원을 맞추니까 *2개의 여유 자유도*. IK는 "유일한 해"가 아니라 "*그럴듯한 해 하나*"를 고름.

### 8.2 왜 어려운가

`FK(θ)`는 *비선형*. 회전행렬에 sin/cos. 일반적 방법은 **반복 최적화** — 잔차 `r(θ) = FK(θ) - p_target`을 줄이는 방향으로 θ를 갱신.

### 8.3 Jacobian — 비선형 산을 한 걸음 내려가는 도구

*θ를 살짝 바꾸면 EE 위치가 얼마나 바뀌는가*를 알려주는 행렬이 **Jacobian** J. 크기 3×5:

```
J[i, j] = ∂(FK_i) / ∂θ_j
```

J의 j번째 열은 "*j번째 조인트만 살짝 돌렸을 때 EE가 어느 방향으로 얼마나 움직이는가*".

```
Δp ≈ J · Δθ      (작은 변화)
```

Δp = (p_target − 현재_p). pseudo-inverse:

```
Δθ = J⁺ · Δp
```

반복. pseudo-inverse가 *현재 θ에서 가장 가까운* 해로 자연스럽게 끌어줌.

### 8.4 DLS — 특이점 근처에서 안정화

순수 pseudo-inverse는 *특이점*에서 폭주 (J 행이 선형 종속). 해결: **Damped Least Squares**:

```
Δθ = Jᵀ · (J·Jᵀ + λ²·I)⁻¹ · Δp
```

`λ` (damping)이 *어떤 자세에서도 안전*하게. PyBullet의 `calculateInverseKinematics`가 DLS 변형.

### 8.5 PybulletSolver.ik의 실제 흐름

[solver.py:171-234](backend/modules/kinematics/solver.py#L171-L234):

```python
def ik(self, target_position, target_quaternion, current_joint_angles):
    # 1. commanded → actual (sag 역보정)
    current_actual = self._commanded_to_actual(current_joint_angles)

    # 2. seed 설정
    rest = list(current_actual)
    self._set_joint_positions(current_actual)

    # 3. PyBullet IK (DLS)
    result = p.calculateInverseKinematics(
        bodyUniqueId=self._robot,
        endEffectorLinkIndex=self._ee_index,
        targetPosition=target_position,
        restPoses=rest, maxNumIterations=100, residualThreshold=1e-4,
    )

    # 4. 수렴 검증
    if error > 0.01: return None

    # 5. actual → commanded (sag 보정)
    return self._actual_to_commanded(actual_angles)
```

- **target_quaternion = None** — 5DOF라 position-only IK.
- **sag 보정 양방향** — commanded↔actual 매핑. [docs/calibration_apply_flow.md](docs/calibration_apply_flow.md) 참고.

### 8.6 코드에서 어디?

- IK 호출: [solver.py:171-234](backend/modules/kinematics/solver.py#L171-L234)
- MoveTCP entry: [motion_modes.py:25-35](backend/modules/kinematics/motion_modes.py#L25-L35)

---

## 9. Ruckig Trajectory — 50Hz 부드러운 모션

`MoveTCPStep`이 IK를 *한 번* 풀어 끝나는 게 아니다. *시작점에서 목표점까지 직선 위 점들을 50Hz로 샘플링하면서, 각 점마다 IK를 풀어 모터에 보낸다*.

### 9.1 왜 한 번이 아닌가

조잡한 접근: 목표 조인트각을 한 번만 publish. 모터가 알아서.

문제 — Dynamixel의 *내부 trapezoidal profile*은:
1. 조인트별 도착 시간이 어긋남 → EE가 곡선
2. 가속도/jerk 제어 거침 → 진동
3. 캘 정밀도 mm인데 경로가 거치면 의미 없음

진짜 답: *PC가 50Hz로 경로 한 점씩 publish, 모터는 작은 보간만*.

### 9.2 Jerk-limited motion

가속도의 변화율이 *jerk* (3차 도함수). 큰 jerk:
- 기어 충격
- 진동 → 카메라/그리퍼
- 바닥 흔들림

**jerk-limited profile** = 시간축에서 *jerk까지 bounded*. Ruckig은 *7-구간 S-curve*를 즉시 계산.

### 9.3 7-구간 S-curve

```
acceleration
    ▲
    │      ┌───────┐
    │     ╱         ╲
────┼────┘           └────────────────  → time
    │                ╲                 ╱
    │                 ╲               ╱
    │                  └─────────────┘
```

jerk + / 0 / − / 0 / − / 0 / +.

### 9.4 Cartesian path 파라미터화

MoveL은 *base 직선*. 길이 `L`을 매개변수 `s ∈ [0, L]`로:

```
position(s) = start + (s/L) · (end − start)
```

`LinearPath` ([trajectory_runner.py:61-77](backend/modules/kinematics/trajectory_runner.py#L61-L77)). Ruckig이 1차원 변수 `s`에 jerk-limited profile 적용.

### 9.5 50Hz 루프

[trajectory_runner.py:230-280](backend/modules/kinematics/trajectory_runner.py#L230-L280):

```python
while True:
    if self._stop_ev.is_set(): break
    result = otg.update(inp, out)                  # Ruckig: 다음 sample
    wp = path.position_at(out.new_position[0])     # s → [x, y, z]
    angles = self._move_tcp(wp, current_angles)    # IK
    if angles is None:
        self._publish_state(FAILED, progress); return
    current_angles = angles
    self._publish_cmd(angles)                       # → MOTOR_CMD_JOINT
    self._publish_state(RUNNING, progress)
    sleep_until(next_t)
```

*Ruckig은 시간*, *path는 공간*, *IK는 매 sample*을 조인트각으로, *publish는 모터*. 4개가 50Hz로 도는 동안 EE는 부드러운 S-curve로 미끄러진다.

### 9.6 두 소비자 — 백엔드 동기화 + UI

매 sample `MOTION_STATE_TRAJ` 발행. 두 소비자:
1. **StepExecutor**가 듣고 traj 완료 Event를 set (§5.3)
2. **Frontend useBridge**가 듣고 motionStore 갱신 (§2.1)

같은 stream을 두 곳이 활용.

---

## 10. MOTOR_CMD_JOINT → 실제 모터까지

`_publish_cmd(angles)`가 토픽에 publish하면 [motion_node.py:130-146](backend/nodes/motion_node.py#L130-L146)에서 rad → raw 정수 변환:

```python
self.publish(Topic.MOTOR_CMD_JOINT, {
    "joints": [
        {"id": cfg.id,
         "position": coords.urdf_to_motor(angle, cfg, ...)}
        for cfg, angle in zip(self._arm_cfgs, angles_rad)
    ],
})
```

`urdf_to_motor`:
1. URDF rad → raw 정수 (0~4095, 중심 2048 = 0 rad)
2. **joint_offset 차감** — 캘 결과의 모터 zero 오차 보정
3. limit clamp

MotorNode가 *Dynamixel SyncWrite* — 한 패킷으로 5개 모터 동시. 그리퍼는 §5.4에서 본 *부드러운 profile*이 별도 적용돼 있음.

---

## 11. context dict — step 간 데이터 다리

pick-and-place 핵심 데이터 흐름:

```python
GroundedDetectStep(output_key="object_pos")
# → context["object_pos"]      = [x, y, top_z]
# → context["object_pos_meta"] = {"base_z", "height"}

GraspPolicyStep(input_key="object_pos", output_key="grasp_xyz")
# → context["grasp_xyz"]       = [x, y, grasp_z]

MoveTCPStep(position_key="grasp_xyz", offset=(0,0,0.06))   # pre_grasp
MoveTCPStep(position_key="grasp_xyz", offset=(0,0,0))      # grasp
MoveTCPStep(position_key="grasp_xyz", offset=(0,0,0.08))   # lift
```

세 종류의 키:
- `object_pos` — 원본 detect 결과 (위치)
- `object_pos_meta` — 부수 정보 (base_z, height)
- `grasp_xyz` — 정책이 가공한 결과

이 셋이 *한 task 동안 살아 있는* `TaskContext` dict 안에 들어감. task 끝나면 GC.

### 11.1 왜 detect를 한 번만 하는가

1. **속도**: GroundedDetect는 *수 초~수십 초* (CPU 추론). 매 step마다 다시 하면 비효율.
2. **일관성**: 한 번 결정한 grasp_xyz를 끝까지 *같은 위치*로 유지.

가정: *물체가 detect~lift 사이 안 움직임*. 외부 간섭 시나리오면 매 step detect로 바꿔야 함.

---

## 12. 정리 — 전체 흐름 다시 한번

```
[Frontend]
  PromptPanel "Run" 클릭
    → bridge.callService(TASK_RUN, {task: "pick_and_place", prompt})    (§1)

[Bridge (FastAPI)]
    → Zenoh queryable "omx/task/srv/run"
    → TaskNode._handle_run                                              (§3)
        ← {success: true}  (즉시 반환)

[TaskNode 백그라운드 스레드]
    → factory(data) → Task 객체
    → TaskRunner.run(task) → 새 스레드 _run_task                        (§5)
        for step in task.steps:
            → publish TASK_STATE
            → StepExecutor.execute(step)
                ┌────────────────────────────────────────────────────┐
                │ GroundedDetectStep:                                │
                │   call_service(PERCEPTION_GROUNDED_DETECT, prompt) │
                │     → DetectorNode._handle_grounded_detect (§6)    │
                │       depth enable + Grounding DINO bbox           │
                │       + depth median(top/base) → Z_cam + height    │
                │       + hand_eye → base xyz                        │
                │   context.set("object_pos", [x,y,top_z])           │
                │   context.set("object_pos_meta", {base_z, height}) │
                │   ─ 동시에 PERCEPTION_GROUNDED_STATE broadcast      │
                │                                                    │
                │ GraspPolicyStep:                              (§7) │
                │   height < 4cm → grasp_z = top_z - 5mm             │
                │   else         → grasp_z = base_z + h * 0.5         │
                │   context.set("grasp_xyz", [x, y, grasp_z])        │
                │                                                    │
                │ MoveTCPStep × 3:                                   │
                │   position = grasp_xyz + offset(0,0,dz)            │
                │   call_service(MOTION_MOVE_L, {position})          │
                │     → MotionNode → LinearPath                      │
                │     → TrajectoryRunner (별도 스레드, 50Hz)        │
                │         Ruckig.update → s                          │
                │         path.position_at(s) → xyz                  │
                │         IK(xyz) (PyBullet DLS) → 5 angles  (§8)    │
                │         → publish MOTOR_CMD_JOINT                  │
                │         → publish MOTION_STATE_TRAJ (progress)     │
                │   wait MOTION_STATE_TRAJ.status == DONE    (§5.3)  │
                │                                                    │
                │ GripperStep / WaitStep / HomeStep         (§5.4)   │
                │   gripper는 부드러운 profile_velocity로 ramp        │
                └────────────────────────────────────────────────────┘
    → MOTOR_CMD_JOINT → MotorNode → Dynamixel SyncWrite → 모터  (§10)

[Frontend 환류]                                                  (§2)
    useBridge가 토픽 구독 → store 갱신 → 패널 자동 리렌더
        TASK_STATE          → taskStore       → TaskProgressPanel
        MOTION_STATE_TRAJ   → motionStore     → progress bar
        PERCEPTION_GROUNDED_STATE → detectorStore → 3D 마커
```

---

## 13. 더 공부하고 싶을 때

1. **로봇 운동학 일반** — Steven LaValle, *Planning Algorithms*, Ch. 3 (online): http://lavalle.pl/planning/.
2. **DH parameter + URDF** — Kevin Lynch & Frank Park, *Modern Robotics*.
3. **IK / Jacobian / DLS** — Samuel Buss, "Introduction to Inverse Kinematics with Jacobian Transpose, Pseudoinverse and Damped Least Squares methods".
4. **Jerk-limited motion planning** — Lars Berscheid & Torsten Kröger, "Jerk-limited Real-time Trajectory Generation" (RSS 2021) — Ruckig 원 논문.
5. **Open-vocabulary detection** — Shilong Liu et al., "Grounding DINO: Marrying DINO with Grounded Pre-Training" (ECCV 2024).
6. **단안/depth 깊이 회수** — Hartley & Zisserman, *Multiple View Geometry in Computer Vision*, Ch. 6.
7. **dataclass + DSL 패턴** — Python `dataclasses` 공식 문서.
8. **pub/sub 아키텍처 일반** — Martin Kleppmann, *Designing Data-Intensive Applications*, Ch. 11.

---

## 14. 자주 하는 헷갈림 (체크리스트)

- [ ] *명령*은 RPC (callService), *상태*는 pub/sub (subscribe). 두 통로가 분리.
- [ ] `callService`의 `await`이 동기처럼 보이는 트릭은 *request_id ↔ promise resolver 매핑*.
- [ ] Bridge는 WS ↔ Zenoh 변환 층. `{success, message, data}` 봉투가 두 층 모두 동일.
- [ ] `useBridge`가 모든 토픽 구독을 *한 곳에서* 등록 → store로 라우팅.
- [ ] Task 정의는 *데이터*, 실행은 *TaskRunner + StepExecutor*. 두 층 분리.
- [ ] step 세 종류: 생산자 (Detect) / 변환 (GraspPolicy) / 소비자 (MoveTCP). context dict로 연결.
- [ ] GroundedDetect는 *color로 bbox* 찾고 *depth로 Z + height* 측정 — 두 신호 결합.
- [ ] depth stream은 *공용 자원* — 켜는 건 idempotent, 끄는 건 안 함.
- [ ] GraspPolicy는 5줄 정책 — 얇음/두꺼움 분기로 grasp_z 결정. 수학 없음, 디자인적으로 중요.
- [ ] IK는 *비선형 잔차 최적화*. Jacobian이 선형근사, DLS가 특이점에서 안정.
- [ ] 5DOF 아암 → position-only IK (orientation 자유). target_quaternion=None.
- [ ] MoveL = *base 직선*, MoveJ = *조인트 직선* (TCP는 곡선).
- [ ] Ruckig은 1D 변수 s ∈ [0, L]을 jerk-limited로. path가 s → xyz로 변환.
- [ ] 매 50Hz tick마다 IK *새로 풀어* MOTOR_CMD_JOINT publish.
- [ ] `MOTION_STATE_TRAJ` 한 stream이 *backend 동기화 + frontend progress* 동시 공급.
- [ ] 그리퍼는 *current control mode*(파지력) + *position control with profile_velocity*(부드러움) 결합.
- [ ] commanded↔actual = sag 보정. FK/IK는 commanded(모터 명령) ↔ actual(실제 링크 끝) 매핑.

---

*이 문서가 부족하면 표시해 두고 다음 round 때 보강.*
