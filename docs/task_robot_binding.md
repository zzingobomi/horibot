# task ↔ robot 바인딩 — 열린 설계 논의 (미결, 2026-07-13)

> **상태: 논의 중 / 미구현.** 다음 세션에서 이어서 결정한다. 정해지면
> [task.md](task.md) 로 접고 이 파일은 지운다. 아직 아무것도 구현/커밋 안 함.
>
> **읽는 법: 아래 "다음 세션 접근법" 을 먼저 읽어라.** 이번 세션이 헤맨 이유가
> 거기 있고, 같은 덫을 피해야 한다.

---

## 0. 무엇을 정하려는가 (한 줄)

**task 페이지의 프론트엔드가 robot 을 어떻게 정하는가** — 지금은 하드코딩 상수로
박혀 있고, 그걸 걷어내되 "차근차근 first-principles" 로 옳은 구조를 잡는다.

## 1. 문제 (증상)

- `frontend/src/pages/pickAndPlaceTask.ts` 에 **`TASK_ROBOT_ID = "so101_6dof_0"`
  하드코딩**. backend `TASK_ROBOTS=("so101_6dof_0",)` + scenario 리터럴과 **손 복제**
  = SSOT 위반 (같은 값 3곳).
- 같은 파일의 `TASK_NAME` = **소비처 0인 죽은 export**.
- task 패널(detectionCamera / pickAndPlace / taskProgress)이 프론트의 robot-ownership
  모델에서 **carve-out** 되어 이 상수를 읽음 (registry.ts `ROBOT_OWNED_PANELS` 제외 +
  주석 "§7 carve-out").
- 사용자 지적: **예전에 default robot id 상수 박았다가 걷어내느라 고생한 클래스의 재발.**
  (memory: `feedback_no_hardcoded_robot_id`, `feedback_no_dead_scaffolding`)

## 2. 뿌리 (왜 생겼나)

backend 가 "task→robot" 의 SSOT(`TASK_ROBOTS`)인데 **프론트가 그걸 얻을 채널이 없음**:
- `GET /tasks` 폐지 + 계약 스트림 키가 `{robot_id}` **placeholder** 라 계약이
  "pick_and_place = so101" 을 **표현하지 못함**.
- → 그 공백을 **하드코딩 복제**로 메꾼 것. **상수는 증상, 뿌리는 "task 바인딩을
  표현할 채널의 부재".**

## 3. 대전제 (사용자 확정 — 여기서 출발)

**task 는 특정 robot 에 국한된 게 아니다 → task 패널은 robot 을 지정하지 않는다.**
- 근거: calibration/motion 은 "이 **robot** 의 캘/모션" 이라 robot 이 대상 자체지만,
  task 는 "이 robot 의 무엇" 이 아니라 "**무엇을 한다**" 라서 robot 이 대상이 아님.

## 4. 추론 체인 (차근차근 — 대전제를 깔고 하나씩)

### 걸림 1 (도달) — task 스트림이 robot-scoped 키다
`stream/pick_and_place/{robot_id}/state|trace|markers` — 키에 `{robot_id}` 박힘(✓).
패널이 robot 을 안 정하면 이 자리를 못 채움 → **"task 는 robot 무관"인데 "task 스트림
키는 robot 에 묶임" = 정면 모순.**

- **방향(판단)**: task **자신이 내는** 스트림은 **host-level 키**로 옮긴다 →
  `stream/pick_and_place/state`, `robot_id` 는 **payload** 로. "키 = 어느 task /
  payload = 그 run 안 어느 robot". 패널은 robot 몰라도 구독 가능.
  - 정합 근거: runner 가 지금도 **같은 RunState 를 robot 별로 중복 fan-out** 함(✓) →
    개념상 run 은 하나.
  - 구분: **task-소유 스트림 = task 키 / robot-소유 스트림(detector/camera) = robot 키
    유지** (카메라는 진짜 robot 하나에 묶이니까).
  - 미해결: 멀티robot 협동 시 host-level latest-wins 충돌 → **멀티robot run 상태를
    어떻게 표현할지**(TaskState.robot_id 단일 필드)가 걸림. 지금 정할지 defer 할지 논의.

### 걸림 2 (다음, 미논의) — 검출 카메라가 대전제에 저항
카메라 피드는 **본질적으로 robot 하나**에 묶임. 그래서 질문: **DetectionCameraPanel 은
"task 패널"인가, "task 페이지에 얹힌 robot 패널"인가?**
- 가설(판단): 검출 카메라는 task 패널이 아니라 **robot-scoped 패널**이다 → 기존
  robot-ownership(picker + capability gate)을 그대로 쓴다. capability = `rgbd`
  (검출이 depth 필요 ✓, robots.yaml 에서 so101=rgbd / omx=없음 ✓ → 자연히 so101 만).
- 여기서 갈림: 그럼 "task 패널은 robot 무관"(대전제)과 "카메라는 robot 있음"이 공존 →
  **task 페이지 = task-무관 패널 + robot-scoped 패널의 혼합**으로 보는 게 맞는지 확인 필요.

### 이후 예상 걸림 (아직 안 짚음 — 다음 세션 목록)
- RUN/STOP/PAUSE 등 제어 서비스는 이미 **robot-agnostic 키**(✓) → robot 불요. 대전제와 정합.
- 씬 **focus**: task 페이지가 카메라를 어디로 둘지 (Container 는 `focusId=null` 이면
  centroid ✓). focus 를 아예 안 쓸지, run 의 robot 으로 런타임 파생할지.
- **TaskMarkersOverlay**(씬 오버레이, 패널 아님): 마커 스트림을 어떻게 구독할지 —
  host-level 로 바뀌면 robot 몰라도 구독 + payload robot_id 프레임에 렌더.
- **무robot / 멀티robot task 일반화**: 위 구조가 0·1·N robot 다 커버하는지 최종 점검.

## 5. 확인된 사실 vs 판단 (다음 세션 — 섞지 말 것)

**✓ 코드로 확인:**
- pick_and_place 서비스 = agnostic 키(`srv/pick_and_place/run` 등, `{robot_id}` 없음) /
  스트림 = robot-scoped 키 + payload robot_id. (contract.py)
- backend 바인딩 SSOT = `TASK_ROBOTS` + scenario 리터럴. (module.py)
- runner 가 RunState 를 robot_ids 별로 fan-out 발행. (module.py `_publish_state`)
- 프론트 robot-ownership = `withRobotOwnership`/`useRobotId`/RobotContext + capability
  gate + `initialRobotId`(route:id / 단일robot / else picker). (robotOwnership.tsx,
  registry.ts, ModeDockview.tsx)
- task 패널 carve-out + 하드코딩 상수. (registry.ts, pickAndPlaceTask.ts)
- 검출은 depth 필요(depth 스냅샷 투영, depth 없는 후보 누락). (detector/module.py)
- capabilities: so101=[move,calibrate,gamepad,rgbd] / omx=[move,calibrate]. (robots.yaml)
- 제어 서비스 agnostic 이라 RUN/STOP 은 robot 불요.

**판단(미확정 — 반박 가능):**
- task-소유 스트림을 host-level 키로 옮기는 게 옳다.
- 검출 카메라는 task 패널이 아니라 robot-scoped 패널이다 (capability=rgbd).
- 방향 = "프론트는 task 에 대해 robot-agnostic, robot 필요한 자리만 패널이 소유".

**미검증(주의 — 확인 필요):**
- 브라우저 bridge 왕복 wildcard/host-level 구독: zenoh/bridge 코드상 릴레이됨(ws.py 가
  topic 을 transport.subscribe 로 그대로 넘김, zenoh `*` 네이티브 ✓, run_task 가
  zenoh 레벨서 사용 ✓) — 그러나 **브라우저 end-to-end 는 미검증**. host-level 로 가면
  이건 그냥 concrete 키라 문제 없음(오히려 단순).

## 6. 다음 세션 접근법 (이번에 헤맨 교훈 — 먼저 읽어라)

이번 세션이 계속 헤맨 근본 원인 두 개:
1. **답을 먼저 뱉고 검증을 나중에** 함 — 부분만 읽거나 기억으로 추측한 걸 "결론"인 척
   제시 → 매번 교정당하고 되돌림.
2. **기존 코드/주석의 결정(carve-out, "GET /tasks 폐지", 하드코딩)을 "확정 전제"로
   깔고 거기서 추론** — 근데 그게 바로 재검토 대상. 기존 설계를 변호하지 말 것.

그래서 다음 세션 규칙:
- **전체 서브시스템(계약·스트림·robotOwnership·registry·ModeDockview·detector) 을 먼저
  다 읽고 모델을 세운 뒤** 논의. 조각 반응 금지.
- **first-principles**: "코드가 왜 이렇게 돼 있나" 가 아니라 "이게 옳은가" 를 묻는다.
- **"확인한 사실" 과 "추측" 을 매번 명시 분리.**
- **구현 급하게 금지** — 대전제 → 걸림 하나씩 차근차근 합의하고 나서 구현.

## 7. 관련 파일 (진입점)

- `frontend/src/pages/pickAndPlaceTask.ts` (삭제 대상), `PickAndPlacePage.tsx`
- `frontend/src/components/panels/registry.ts` (`ROBOT_OWNED_PANELS` / carve-out / PANEL_CATALOG)
- `frontend/src/components/shared/robotOwnership.tsx` (`withRobotOwnership` / `RobotProvider`)
- `frontend/src/components/shared/ModeDockview.tsx` (`initialRobotId`)
- `frontend/src/hooks/useRobotId.ts`
- 패널: `panels/PickAndPlacePanel` / `TaskProgressPanel` / `DetectionCameraPanel`
- 오버레이: `components/scene/overlays/TaskMarkersOverlay.tsx`
- backend: `modules/tasks/pick_and_place/{contract,module}.py`, `modules/detector/module.py`
- `robot/robots.yaml`

## 8. 이번 세션 커밋 대기 (별개 — 이 논의와 무관하게 완료·검증됨)

되돌리지 말 것. 이 robot 바인딩 건과 독립:
- STEP_RESULT/step_note/ctx.record 제거 + task-owned 마커 통로(B, `TaskMarkers` 스트림).
- MoveJ 통합 (MoveJ_pose 흡수, `MoveTarget = JointTarget | PoseTarget` discriminated
  union, `tool_offset`→`tcp_offset` on PoseTarget).
