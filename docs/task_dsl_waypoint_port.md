# Task DSL 재이주 + Waypoint 자산 계층 + PnP (설계, 2026-07-03)

> backend_v2 로 **Task DSL / 디버거 / pick_and_place 재이주** + 신규 **Waypoint(로봇
> 자산) 계층** 설계. **구현 전 단계** — 방향·이름·스키마·소유권 결정 완료(§2 결정
> 로그), 구현은 이후 세션. 본 문서로 논의 이어가기.
>
> 관련: [backend_v2.md](backend_v2.md)(framework SSOT) · [framework_async_call_contract.md](framework_async_call_contract.md)(async-uniform call — 이 포팅의 전제) · [step_dsl.md](step_dsl.md)(옛 DSL 설계) · [naming_conventions.md](naming_conventions.md)(wire 클래스 이름).

## 1. 배경 — 신규 설계가 아니라 "재이주 + 개선"

옛 `backend/` 엔 이미 **성숙한 Task DSL(typed Slot lego) + pick_and_place + VSCode 식
디버거 UI** 가 다 있다. v2 로 **아키텍처(배선)가 바뀌었을 뿐**, DSL/디버거를 뭘로
설계할지는 답이 나 있다. 따라서 본 작업 = **옛 조각을 v2 primitive(module / @service /
@subscriber / @publishes / async `runtime.call` / contract-gen / framework hook)에
매핑 + 동작 보존** ([feedback_port_keep_v2_arch] — 기계적 복사 X, v2 경계에 맞게 적응).

동시에 논의 중 옛 backend 의 **두 실제 결함**을 설계로 고치기로 했다:
- search 자세가 `robot_poses.yaml` 하드코딩 → **Waypoint 자산 계층**으로.
- detection first-match-wins 오검출(흰 큐브 옆, 가운데 흰 안경닦이를 큐브로 오인) →
  **Top-K + 기하 prior**(멀티뷰는 후속).

## 2. 결정 로그 (2026-07-03 잠금)

| # | 항목 | 결정 | 근거 |
|---|---|---|---|
| D1 | 계층 | **Motion → Robot Asset Layer → Consumer(PnP/Scan/Inspection)** | Waypoint 는 Motion 위에 얹혀 여러 task 가 공유하는 자산 |
| D2 | 실행 모델 | **async runner** — step `async def execute(ctx)` + `await runtime.call` | thread 기반 유지 시 방금 없앤 `run_coroutine_threadsafe` 브리지 부활 (§3.1) |
| D3 | 자산 이름 | **Waypoint / WaypointGroup / WaypointGroupMember** | "Pose" 는 TCP pose 연상. Waypoint=티칭 위치 재사용 자산 (UR 어휘). DSL·UI 가독 |
| D4 | 저장 내용 | **joint-only** 저장, 이름은 umbrella(Waypoint) | 티칭 자산은 항상 joint config. cartesian 좌표(grasp/place)는 런타임 파생이라 자산 X. `kind` 다형성은 필요 시 추가, 이름은 그 migration 생존 |
| D5 | Group 저장 | **Waypoint / WaypointGroup / WaypointGroupMember(order)** 3-테이블 | 축은 정규화 아니라 **변경 패턴** — reorder/add/remove 가 행 단위 + `order` 컬럼이 드래그 UI 와 1:1. SQLAlchemy 표준 |
| D6 | 소유권 | **Robot Instance** (`Waypoint.robot_id` = instance id) | 설치 위치/base frame/tool/joint-zero 캘이 instance 마다 다름. v2 per-instance 데이터 관례 일관. model 공유는 나중 옵션 |
| D7 | 티칭 | **jog → 현재 joint 저장** | joint config 는 같은 instance 에서 항상 정확 재현 + IK 재계산 X. jog 이미 있음(Motion D3) |
| D8 | DSL 표면 | **`MoveJ(waypoint=<ref>)`** — "ByName" 구현 디테일 숨김 | waypoint 참조(이름/ID/Slot) resolve 는 runtime. 식별 방식과 DSL 분리 |
| D9 | Calibration pose | **별도 개념 유지** | 캘 pose 는 알고리즘 생성/추천(JointPerturbationStrategy), 티칭 자산과 출처·생명주기 다름 |
| D10 | detection | **Top-K + 기하 prior 우선**, multi-view 3D 합의는 후속 확장 | (1)+(3)이 실패의 급소, (2)는 일반해 (§5.2) |
| D11 | 자동 scan | **defer** (사용자 UI/UX 결정 필요) | scan 자동 순회는 Waypoint + DSL 소비자로 나중 흡수 |

> **모듈 이름 향후 여지**: `WaypointModule` 단일로 시작하되, 장기적으로 Robot Asset
> Layer 에 tool/frame 등이 붙으면 `robot_assets/{waypoint,tool,frame}/` 하위 구조 가능.
> **Waypoint = Robot Asset Layer 의 첫 자산.**

## 3. Substrate A — DSL 코어 재이주

### 3.1 핵심 피벗: thread 기반 → async runner

옛 `TaskRunner` 는 daemon 스레드 + `threading.Event`(pause/resume) + 블로킹
`wait_for_traj` 로 돈다. step 은 sync `execute(ctx)` → `ctx.call_motion()`(블로킹).

**v2 로 그대로 옮기면 안 됨** — sync step 이 v2 async `runtime.call` 을 부르려면
`run_coroutine_threadsafe` 브리지 필요 = [framework_async_call_contract.md] 에서 scan
`_call` 로 없앤 그것 부활. 그래서 **runner 를 async 로**:

- step → `async def execute(ctx)`, 안에서 `await ctx.call(...)`(= `await runtime.call`)
- 태스크 = asyncio task. pause/resume = `asyncio.Event`, 디버거 게이트 =
  `await self._pause_event.wait()` (loop 안 막음). `Wait` 의 `time.sleep` → `await asyncio.sleep`
- 태스크 전체가 loop 위 코루틴 — I/O 마다 await, loop 는 다른 service 위해 자유
- **이번 async-uniform 리팩터가 이 포팅을 자연스럽게 만든 것** — step 이 `await runtime.call` 하나로 끝

### 3.2 옛 → v2 매핑

| 옛 backend | → v2 |
|---|---|
| `TaskNode`(ApplicationNode) | `TaskModule` plain class, PC 배치 |
| `ctx.call_motion`/`call_service`(sync + traj Event) | `await self.runtime.call(...)` |
| `ctx.node.publish(TASK_STATE/TREE/STEP_RESULT)` | `@publishes` streams |
| run/preview/stop/resume/step/run_to/toggle_breakpoint 서비스 | `@service async def` |
| `TaskRunner`(threading) | async runner (asyncio.Event 게이트) |
| `Slot`/`StepResult`/`Step`/`StepContext`/`task_tree`/`collect_step_ids` | **거의 그대로** — StepContext 가 `node` 대신 `runtime` 보유, resolve/store/run_child 동일 |
| `core/values.py`(Position3/Pose6/Detection) | v2 공유 값 타입 (task+detector 공유) — 위치 §6 open |

### 3.3 디버거 (동작 보존)

옛 `_execute_one_step` 의 디버거 게이트 그대로: `_should_pause_before`(STEP_ONCE /
RUN_TO / breakpoint) → pause → `await pause_event.wait()` → 실행 → status/step_result
publish. control flow(ForEach/Try) 가 `ctx.run_child(child)` 로 재진입 → nested step 도
동일 게이트 자동 적용 (옛 설계 유지).

서비스/토픽(옛과 동형, v2 contract 로 gen):
- 서비스: run / preview / stop / resume / step / run_to / toggle_breakpoint
- 토픽: task/tree · task/state · task/step_result

### 3.4 검증 — trivial test task

도메인 step 0개로 **Wait + no-op step 2개짜리 test task** 를 만들어 runner + 디버거
end-to-end(pause/resume/step/breakpoint/run_to) 를 host_mock 단일 프로세스에서 검증
(옛 lego acceptance test 방식). PnP 전에 코어부터 안정화.

### 3.5 프론트 디버거 UI 포팅

옛 `frontend/` 는 이미 `useTopic`/`useService` 로 backend 와 decoupled — v2 framework
hook(`useStream`/`useService`) + contract-gen 으로 재배선:
- **TaskProgressPanel**(디버거: status dot + step status icon + 재귀 StepRow + breakpoint
  context menu + Resume/Step/Stop/RunTo)
- **PromptPanel**(run/preview/stop 트리거)
- **TaskResultLayer**(task/step_result → type dispatch: Detection→sphere, Position3→marker;
  새 task tree 도착 시 clear)
- stores: taskResult 누적

## 4. Substrate B — Waypoint 자산 계층

### 4.1 스키마 (Database-per-Module — WaypointModule 소유)

```
Waypoint
  id            PK
  robot_id      instance id (D6)
  name          사용자 이름 (home / search_left ...)
  joint_values  joint config (D4 — joint-only)
  created_at    UTC-aware

WaypointGroup
  id            PK
  robot_id      instance id
  name          Search / Scan / Inspection ...

WaypointGroupMember           (D5 — order 있는 join)
  group_id      FK → waypoint_groups (CASCADE)
  waypoint_id   FK → waypoints (CASCADE)
  order         int
```

루트 alembic 에 migration, 공유 Base 등록 (다른 모듈은 이 테이블 모름).

### 4.2 서비스 (WaypointModule, plain class)

- **티칭** = "현재 joint 로 create" — 별도 service 클래스 안 쪼갬 (과분리 경계). jog 로
  자세 잡고 → 이 서비스가 최신 joint state 읽어 Waypoint row 생성.
- CRUD: create(teach) / list / rename / delete
- Group: create_group / add_member / remove_member / reorder / list_groups
- 조회: get_by_ref(name|id) — DSL `MoveJ(waypoint=...)` resolve 가 사용 (D8)
- wire 클래스 이름은 [naming_conventions.md] 준수 (`CreateWaypointRequest` 등)

### 4.3 UI (Robot Assets)

탭 분리: **Waypoint Library**(티칭·이름 수정) / **Waypoint Groups**(기존 waypoint 조합).
Group 화면 = 좌 라이브러리 / 우 선택 group 구성(+Add / 드래그 reorder / remove). 드래그
= `order` update, add = member 행 1개, delete = member 행 1개 (D5 가 UI 와 1:1).

## 5. Consumer — pick_and_place

### 5.1 step 매핑 (옛 → v2)

| step | v2 |
|---|---|
| `MoveJByName(pose_name)` | **`MoveJ(waypoint=<ref>)`** (D8) — waypoint resolve 후 `MOTION_MOVE_J` |
| `MoveTCP(target, offset)` | `MOTION_MOVE_L` (Position3/Detection resolve) |
| `Gripper` / `VerifyGrasp` | `MOTOR_GRIPPER` + gripper Present_Position 검증 |
| `GraspPolicy` / `PlacePolicy` | 그대로 (Detection→Position3 순수 계산) |
| `GroundedDetect` | **detector 모듈 이주 필요** (§5.3) + Top-K 반환 (§5.2) |
| `ForEach`/`Try`/`BreakIf` | 그대로 (async 화) |
| recipe `home()`/`search_and_detect()` | `search_and_detect` 재설계 (§5.2) |

### 5.2 detection 개선 (D10)

**문제**: 옛 `search_and_detect` = ForEach + Try(GroundedDetect top-1) + BreakIf(첫 hit).
첫 포즈에서 뭐든 잡히면 믿고 break → 흰 안경닦이를 흰 큐브로 오인.

**개선 (우선순위)**:
1. **Top-K** (필수) — `GroundedDetect` → `Step[list[Detection]]`. 진짜 물체가 2등이면
   top-1 만으로는 영원히 후보에서 누락.
2. **기하 prior** (급소) — depth 로 얻은 `Detection.height`/`base_z` 로 예상 크기/높이
   범위 밖 후보 reject. 큐브(높이 有) vs 안경닦이(납작) 를 confidence 무관 구분.
3. **multi-view 3D 합의** (후속 확장) — 후보를 hand_eye+FK 로 base 3D 변환 → 여러 포즈
   반복 검출 클러스터가 진짜. 구현량 커서 **후보 누적 구조만 먼저 만들고 스코어링은 실
   데이터 보며**.

**recipe 재설계**: BreakIf 제거 → 모든 group member(waypoint) 순회하며 후보 누적 → 새
step `SelectTarget(candidates, prompt, priors)` 가 (뷰 지지 + confidence + 기하 prior)
스코어 → 최종 Detection (신뢰 후보 없으면 fail).

search 자세는 `search_and_detect(waypoint_group="...")` 로 **Waypoint Group 소비** (D11
자산 계층).

### 5.3 detector 모듈 이주 (PnP 최대 덩어리)

v2 엔 detector 모듈 **없음**. PnP 는 이걸 끌고 옴:
- 검출 모델(Grounding DINO open-vocab / YOLO / 고정클래스 — §6 open) + FrameCache 등가
- intrinsic `undistortPoints` + `MOTION_GET_TCP` + base 평면 Z=0 → base 좌표 → hand_eye
- `GROUNDED_DETECT` 서비스 (Top-K 반환)

### 5.4 검증

lego acceptance test(pick_and_place 가 primitive+recipe 조합으로 동등 동작). **detection
정확도 = 실 하드웨어(집)만** — 회사에선 구조/계약/mock e2e 까지.

## 6. 구현 전 확인할 open item

1. **motion trajectory 완료 대기** — v2 `MOVE_L`/`MOVE_J` 서비스가 (a) trajectory 끝날
   때 **완료 후 반환**(async 서비스, step 은 `await runtime.call` 하나) 인지 (b) 즉시
   반환 + step 이 `MOTION_STATE_TRAJ` 를 asyncio.Event 로 대기 인지. **(a) 권장**(async-
   uniform 정신). motion 모듈 계약 확인 필요.
2. **detector 모델 범위** — Grounding DINO(무거움) vs YOLO vs 첫 PnP 고정클래스.
3. **공유 값 타입 위치** — Position3/Pose6/Detection 을 어느 v2 모듈/contract 에 둘지
   (task + detector 공유).

## 7. 단계 순서 + 검증

```
Motion(기존)
   └─ Substrate A: DSL 코어 ─┐   Substrate B: Waypoint ─┐
                            └──────────┬────────────────┘
                          Consumer: PnP (A+B 필요)
                          Consumer: 자동 scan (defer, A+B 필요)
```

- **Phase A — DSL 코어**: schema/step/async runner/TaskModule + 디버거 서비스·토픽 +
  test task + 프론트 디버거 UI. → 회사에서 unit + host_mock e2e 검증.
- **Phase B — Waypoint**: 모듈(스키마+티칭+group 서비스) + Robot Assets UI. A 와 **독립**
  (병렬 가능). → 회사에서 CRUD/group e2e 검증.
  - **backend 완료 (2026-07-03)** — [modules/waypoint/](../backend_v2/modules/waypoint/)
    (contract/orm/repository/module) + `MODULE_REGISTRY`/`resolve_deps`/`alembic env`
    배선 + mock·pc.yaml. (개발 단계 — waypoint 3 테이블은 initial migration
    `236606bbc6f5` 에 통합, 별도 migration 없음.) joint 는 **rad 저장**
    (Motion.Stream.TCP_STATE.joints 구독 — raw encoder 안 봄, D4 확정), 티칭 = 현재
    joint create. 9 test + 전체 189 PASS, ruff/pyright clean. `Waypoint.Service` 11개
    (teach/list/rename/delete + group CRUD + member add/remove/reorder/list).
  - **frontend 완료 (2026-07-03)** — `FRONTEND_EXPOSED` 에 11 서비스 추가 → `gen:types`
    (35 services). [WaypointPanel](../frontend_v2/src/components/panels/WaypointPanel/index.tsx)
    (Library / Groups 탭 — teach/rename/delete + group CRUD + member add/remove/reorder는
    up·down 버튼) + `RobotAssetsMode`(`/robots/:id/assets`) + Sidebar "Assets" 상시 노출 +
    registry. 2 vitest(렌더+teach wire) + **3 Playwright e2e**(WS+렌더 / 티칭→목록 /
    group+멤버) PASS, 전체 47 vitest PASS, lint clean. tsc 는 pre-existing jest-dom
    matcher red 만 (내 코드는 toBeTruthy 로 회피, 신규 red 0). fixture(contract.json)
    regen invariant 갱신.
  - **Phase B 완료.** (drag reorder 는 up/down 버튼으로 대체 — 실 드래그는 후속 polish.)
- **Phase C — PnP**: detector 이주 + motion/gripper step + Top-K/기하 prior +
  search_and_detect 재설계 + pick_and_place + lego test. A+B 필요. → 구조·계약은 회사,
  **detection 정확도는 집(하드웨어)**.
- **Deferred — 자동 scan**: A+B 위에 Waypoint Group 순회. UI/UX 는 사용자 결정 후.

각 Phase 는 독립 shippable — 한 번에 다 안 짓는다.
