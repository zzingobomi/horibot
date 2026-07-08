# Task Framework — 명령형 async 함수 + 보일러플레이트 흡수 프레임워크 (설계 확정)

> **status: 설계 확정 (2026-07-08 논의 수렴). 구현 대기. backend_v2.md §17.1④/§17.4 는 본 문서로 개정 대상 (§10).**

## 1. 목표 — 단일 원칙

**Task Framework 는 FastAPI/NestJS 와 동일한 철학을 따른다. 개발자는 로봇 시나리오의
의도만 작성한다. 실행, 바인딩, 예외 처리, 상태 관리, 진행 보고, 취소, 추적, 타입 변환,
로깅 — 모든 공통 관심사는 프레임워크가 담당한다.**

기능 추가 기준은 하나: **"이 기능이 개발자가 반복해서 작성하는 보일러플레이트를
제거하는가?"** 반복이 관찰되면 프레임워크 후보, 아니면 그냥 Python 코드다.
(rule of three 는 이 기준의 한 사례. ctx 에 뭘 넣나 / helper 로 빼나 류의 분류 논쟁도
이 기준으로 절차화 — 미리 taxonomy 를 만들지 않고 반복을 관찰한다.)

이건 이 프로젝트가 이미 검증한 수다 — module framework 의 `@service`/`@subscriber` 가
정확히 같은 철학 (Zenoh 배관/직렬화/에러 봉투를 프레임워크가 흡수, 개발자는 핸들러
본문만). Task Framework 는 같은 수를 시나리오 층에서 반복한다.

| FastAPI/NestJS | Task Framework |
|---|---|
| 라우팅 (`@app.get`) | `@task(name=...)` → registry |
| param 바인딩 + validation | `RunRequest.params` → signature 기반 타입 변환·필수 검증 |
| exception handler → HTTP 상태 | typed 예외 → FAILED + STATE + Motion.STOP + cleanup (§5) |
| DI (`Depends`) | ctx 주입 — robot_id 를 작성자가 한 번도 안 적음 |
| middleware | primitive 경계 게이트/trace/로깅 (작성자 비가시) |
| OpenAPI 자동 생성 | signature → task 스펙 자동 노출 (§6) |

FastAPI 와의 차이는 흡수량이 **더 많다**는 것 — task 는 물리 세계의 long-running
프로세스라 취소·일시정지·모션 정지·진행 추적까지 전부 공통 관심사다.

**폐기 결정**: Step/Slot 선언형 DSL. 선언형이 밥값하는 조건 (반응성 / 비개발자 GUI
편집 / 정적 분석) 이 우리 task 에 없고, 작성자가 도메인 문제보다 프레임워크 문제
(Step 연결, Slot 타입, resolve, factory) 를 더 많이 보게 만들었다. 디버거 UI (tree
preview) 는 표면 설계에 역방향 요구를 못 낸다 — 시나리오 작성성이 결정하고 디버거는
적응한다.

## 2. 작성자가 쓰는 것 / 프레임워크가 가져가는 것

| 작성자가 쓴다 (= 로직 그 자체) | 프레임워크가 가져간다 (작성자 코드에 등장 금지) |
|---|---|
| task 함수 시그니처 (typed params) | params 파싱·타입 변환·기본값·필수 검증 |
| ctx primitive 호출 순서 | robot_id 배관 (ctx 바인딩) |
| `if`/`for`/변수 — 언어 그대로 | wire 계약 (Request/Response 클래스, 서비스 키, timeout 상수) |
| 순수 계산 함수 (grasp 위치 등) | 예외 → FAILED 전파·메시지 조립 (§5) |
| 복구할 때만 typed 예외 catch | 취소 (cancel + in-flight Motion.STOP + cleanup) |
| 보이고 싶은 중간값 `ctx.record` | STATE/TRACE/STEP_RESULT 발행 전부 |
| — | pause/step 게이트 (primitive 경계 자동) |
| — | primitive 호출/완료/실패 로깅 |

현행 Step DSL 이 오른쪽 칸을 작성자에게 새게 하는 실측 지점 (steps.py):
`ctx.resolve` + isinstance 8곳, `timeout=60.0` 반복 5곳, wire 클래스 직접 조립,
`.out`/Slot 배관, dataclass 껍데기 4개. 전부 오른쪽으로 넘어간다.

## 3. 작성 표면

### 3.1 `@task` — 시그니처가 스펙

```python
@task(name="pick_and_place", robot_ids=["so101_6dof_0"])
async def pick_and_place(
    ctx: TaskContext,
    pick_object: str,                # 필수 — 누락 시 실행 전 거부
    place_object: str = "",
    search_group: str = "search",
) -> None:
    ...
```

- `@task` 가 TASK_REGISTRY 등록. 수동 params 파싱 함수 (`_pick_and_place_task`) 소멸.
- wire 는 `RunRequest.params: dict[str, str]` 유지 (LLM/PromptPanel 호환) — 프레임워크가
  signature 로 str/int/float/bool 변환 + 검증. 타입 확장은 실요구 시.

### 3.2 TaskContext — Day-1 primitive + escape hatch

```python
class TaskContext:
    robot_id: str
    # Day-1 primitive — timeout/wire/재시도 정책은 메서드가 소유 (호출부 고민 X)
    async def move_j_waypoint(self, name: str, *, label: str = "") -> None
    async def move_to_pose(self, pos: Position3, *, offset=..., tool_offset=None, label="") -> None
    async def move_l(self, pos: Position3, *, offset=..., orientation="keep", label="") -> None
    async def gripper(self, action: Literal["open", "close"], *, label: str = "") -> None
    async def verify_grasp(self, *, label: str = "") -> None
    async def detect(self, prompt: str, *, top_k: int = 5, label: str = "") -> list[Detection]
    async def wait(self, sec: float, *, label: str = "") -> None
    async def waypoint_group(self, group: str) -> list[Waypoint]
    # escape hatch — 아직 흡수 안 된 서비스 직접 호출
    async def call(self, key, req, res_cls, *, timeout=5.0) -> TRes
    runtime: ModuleRuntime
    # 중간값 노출 (STEP_RESULT → TaskResultLayer, 옛 step 자동 발행의 명시 대체)
    def record(self, label: str, value: BaseModel | None) -> None
```

- 메서드 본문 = 옛 steps.py execute 이식. **의미 단위 경계** — detect 는
  `list[Detection]` 반환 (`DetectResponse` 아님). timeout/wire/실패→예외 변환이 안으로.
- `verify_grasp` 의 TaskRobotSpec/gripper raw 캐시는 ctx 내부 (현행 주입 경로 유지).
- 작성자가 만나는 값 타입은 소수 유지 (`Detection`, `Position3`) — 타입 은닉이 아니라
  typed signature 가 "지금 이 타입 뭐지" 를 해결. 변환 지점 (`grasp_position(det)`) 은
  숨길 게 아니라 가장 잘 보여야 할 도메인 결정 (집 튜닝 knob 이 전부 거기 있음).

### 3.3 도메인 어휘 — helper 함수로 자라고, 반복되면 흡수

```python
async def search_group_detect(ctx, prompt: str, group: str) -> list[Detection]:
    """task 로컬 helper — group 자세 순회 + Top-K 후보 누적 (§17.5)."""
    candidates = []
    for wp in await ctx.waypoint_group(group):
        await ctx.move_j_waypoint(wp.name)
        await ctx.wait(0.3)
        candidates += await ctx.detect(prompt)
    return candidates
```

- 규약: `async def f(ctx, ...)` 자유 함수. task-로컬로 시작 → 두 번째 task 가 반복하면
  `modules/task/helpers.py` 로 이동 (import 한 줄 = 승격 비용 ~0).
- `ctx.pick`/`ctx.detect_best` 선제작 금지 — §1 기준 그대로: 아직 반복 안 됐고, grasp
  전략/best 판정은 task 가 튜닝하는 도메인 결정. **반복이 관찰되는 순간 흡수 후보**
  (search 순회는 유력한 첫 후보 — task #2 가 쓰면 helpers.py 로).
- helper 내부의 move/detect 는 각각 게이트/trace 를 탐 — 관측 무손실.

### 3.4 pick_and_place 재작성 (before: 329줄 Slot 배관 / after: 로직만)

```python
@task(name="pick_and_place", robot_ids=["so101_6dof_0"])
async def pick_and_place(ctx: TaskContext, pick_object: str,
                         place_object: str = "", search_group: str = "search"):
    await ctx.gripper("open")

    pick = select_target(await search_group_detect(ctx, pick_object, search_group))
    place = None
    if place_object:                                   # 그냥 if
        place = select_target(await search_group_detect(ctx, place_object, search_group))

    grasp = grasp_position(pick)                       # 순수 함수 — Slot 없음
    ctx.record("grasp", grasp)
    await ctx.move_to_pose(grasp, offset=Z(PRE_GRASP_DZ), label="pre_grasp")
    await ctx.move_to_pose(grasp, tool_offset=PINCH_OFFSET, label="grasp")
    await ctx.gripper("close")
    await ctx.verify_grasp()
    await ctx.wait(0.5)
    await ctx.move_to_pose(grasp, offset=Z(LIFT_DZ), label="lift")
    await ctx.verify_grasp()
    if place is not None:
        p = place_position(place)
        await ctx.move_to_pose(p, offset=Z(PLACE_HOVER_DZ), label="pre_place")
        await ctx.move_to_pose(p, tool_offset=PINCH_OFFSET, label="place")
        await ctx.verify_grasp()
        await ctx.gripper("open")
        await ctx.wait(0.3)
        await ctx.move_to_pose(p, offset=Z(PLACE_HOVER_DZ), label="retreat")
    await ctx.move_j_waypoint("home")
```

select_target / grasp_position / place_position / GeometricPrior = ctx 무관 순수 함수.

## 4. 실행 규약 (프레임워크 책임)

| 항목 | 규약 |
|---|---|
| 실행 | RUN → `asyncio.create_task(fn(ctx, **kwargs))`. per-robot 동시 1 task |
| 성공 | 정상 반환 → SUCCESS |
| 실패 | 예외 → FAILED + error (§5 실패 모델) |
| 중단 | STOP → `task.cancel()` → CancelledError → STOPPED + **finally 에서 Motion.STOP** |
| 일시정지 | primitive 진입 직전 게이트 hold. 모션 중 급정지는 STOP 의 몫 |
| 관측 | STATE (status/current_label/error/breakpoints) / TRACE (신설 — primitive 호출 누적, TREE 폐기 자리) / STEP_RESULT (값 primitive 자동 + ctx.record) |

**현행 결함 해소**: 지금 runner 의 stop 은 step 경계에서만 검사돼
([runner.py:193](../backend_v2/modules/task/runner.py#L193)) 60s MoveJ 비행 중 정지가
안 먹힌다. `task.cancel()` 은 in-flight await 를 즉시 끊는다.

게이트/trace: 게이트 대상 primitive 진입 시 TraceEntry (id = label 또는 자동
`"primitive#n"`) → pause/step_once/run_to/breakpoint 판정 → 게이트 → 실행 → 완료
마킹. **작성자에겐 존재하지 않는 층** (label 인자 하나만, 그마저 선택).

## 5. 실패 모델 — "try 를 안 쓰는 게 기본"

primitive 는 **typed 도메인 예외**를 던진다 (현행 `RuntimeError(f"...")` 전면 대체):

```python
class TaskError(Exception): ...          # 공통 베이스 — 프레임워크 exception filter 대상
class DetectionNotFound(TaskError): ...  # prompt, 시도 자세 수
class WaypointMissing(TaskError): ...    # name, robot_id
class GraspVerifyFailed(TaskError): ...  # raw, threshold
# MotionFailed 는 motion contract 기존 것 재사용
```

- **기본형: try 없음.** 예외는 그대로 위로 → 프레임워크가 FAILED 전환 + 어디서/어떤
  입력으로 실패했는지 trace 로 메시지 조립 + Motion 정지 + STATE 발행 (NestJS
  exception filter 등가).
- **복구는 예외적으로, 잡고 싶은 타입만**:

```python
try:
    await ctx.verify_grasp()
except GraspVerifyFailed:
    await ctx.gripper("open")      # 놓고 한 번 재시도
    ...
```

## 6. task 스펙 자동 노출 (OpenAPI 등가)

`@task` signature 에서 param 스펙 (이름/타입/기본값/필수/docstring) 을 추출해
`GET /tasks` 에 노출:

```json
{"name": "pick_and_place", "robot_ids": ["so101_6dof_0"],
 "params": [{"name": "pick_object", "type": "str", "required": true}, ...]}
```

- LLM 의 한국어→params 파싱이 task 스펙을 자동으로 앎 (현재 PromptPanel 의
  pick_and_place 하드코딩 제거 경로).
- frontend task 실행 폼 자동 생성 가능 (후속).
- 시그니처 = param SSOT 의 자연 연장 — 스펙 문서/파싱 코드/검증 코드 hand-sync 소멸.

## 7. 로직 검증 경로 — 하드웨어도 frontend 도 없이

1. **순수 함수 pytest** — select_target/grasp_position/prior: 입력→출력 검증 끝. ctx 조차 없음.
2. **FakeContext** — 프레임워크가 정식 제공 (표면의 일부). wire 수준이 아니라 의미
   수준 fake:

```python
async def test_place_생략시_pick만():
    ctx = FakeContext(
        waypoint_groups={"search": [wp("a"), wp("b")]},
        detect_script={"red cube": [[], [det(score=0.9)]]},   # 자세별 결과
    )
    await pick_and_place(ctx, pick_object="red cube")
    assert ctx.calls("detect") == ["red cube", "red cube"]    # 두 자세 순회
    assert "pre_place" not in ctx.labels()                    # place 분기 안 탐
```

   시나리오 로직 (분기/순회/누적/실패 경로) 을 프로세스 하나로 검증. 실 ctx 와 같은
   Protocol 구현 — 드리프트는 pyright 가 잡음.
3. **CLI 직접 실행** — `uv run python scripts/run_task.py pick_and_place --param
   pick_object="red cube" --deploy mock`. host_mock 스택 부팅 → RUN → STATE/TRACE
   stdout 실시간 → 종료 코드 = 결과. `--paused` = 첫 primitive 앞 정지 + Enter 한
   스텝 (터미널 디버거 — frontend 없이 작성 루프 완결).
4. **실 hardware** — 집. 1–3 이 로직 오류를 소진하고 hardware burn 은 캘/검출
   정확도에만.

## 8. 통합 diff (기존 아키텍처)

- **framework 코어 무변경** — TaskContext = `runtime.call` wrapper + 게이트. task 는
  여전히 순수 consumer 모듈. robot-agnostic §2.7 / Motion §17.3 계약 / Waypoint/
  Detector/Motor 계약 전부 무변. llm 모듈 무변 (RunRequest wire 유지).
- **contract diff**: RUN + `start_paused: bool` / PREVIEW·TREE 폐기 / TRACE 신설 /
  RUN_TO·TOGGLE_BREAKPOINT step_id → label / GET /tasks 에 param 스펙 (§6).
  `FRONTEND_EXPOSED` 갱신 → `pnpm gen:types`.
- **처분**:

| 자산 | 처분 |
|---|---|
| TaskModule (8 service) | 유지·수정 (위 diff) |
| TaskRunner | 재작성 — coroutine 감독 + 게이트 + exception filter |
| step.py / schema.py 의 Slot·SlotOr·StepResult | 폐기 (값 타입 Position3 등만 잔류) |
| steps.py Day-1 6종 | ctx 메서드로 이식 + typed 예외 전환 |
| tasks/pick_and_place.py | §3.4 로 재작성 |
| TASK_REGISTRY / robot_ids 바인딩 | 유지 (`@task` 가 등록 + param 스펙 추출) |

- **frontend (후속, 표면에 역방향 요구 금지)**: TaskProgressPanel steps 소스
  TREE→TRACE, bp/run-to = label. PromptPanel 의 실행 전 확인 = `start_paused`
  (CLI `--paused` 의 UI 소비) + 직전 run TRACE 보존. TaskResultLayer 무변.
  dry-run 은 기각 (detect 가 분기를 좌우하면 trace 가 거짓).

## 9. 구현 순서 (착수 지시 대기)

1. **코어**: `@task` + param 바인딩 + TaskContext (primitive 이식 + typed 예외) +
   runner 재작성 (cancel/게이트/exception filter/STATE·TRACE 발행) — noop task 로
   runner e2e (pytest).
2. **FakeContext + 순수 함수 분리** — pick_and_place 를 §3.4 로 재작성 + FakeContext
   테스트 (분기/순회/실패 경로).
3. **CLI runner** (`scripts/run_task.py`) — host_mock 으로 터미널 실행 검증.
4. **contract 정리** — PREVIEW/TREE 제거, TRACE/start_paused/GET /tasks 스펙 추가,
   gen:types 재생성.
5. **frontend 적응** — TaskProgressPanel/PromptPanel (별도 세션).
6. **backend_v2.md §17 개정 + step_dsl.md reference 격하** (§10).

각 단계 = 독립 검증 가능 (testing_strategy L2→L3 정렬). 1–4 는 hardware 무관.

## 10. 확정 시 개정 대상 (옛 문구)

- backend_v2.md §17.4 "Slot/StepResult/Step/StepContext/task_tree 거의 그대로" — 폐기
  (runner/디버거/stream 3종 포팅 자체는 유효했음, 작성 표면만 본 문서로 대체).
- §17.1 ④ "task #2 에서 반복 보이면 Step/Slot 추출" — "반복 보이면 helper 승격 /
  프레임워크 흡수" 로 교체.
- §17.1 "비주얼 에디터 — 직렬화 가능한 스펙 유지" — 문 닫음. task 정본 = Python 함수.
  에디터가 진짜 필요해지면 그때 IR 별도 설계.
- step_dsl.md (옛 backend) — reference 격하.
