# workspace_autohide_header.md

frontend_v2 워크스페이스(3D 씬 + dockview 플로팅 패널)의 **패널 관리 UI를
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

1. **Reset layout 버튼** — [ModeDockview.tsx:136-144](../frontend_v2/src/components/shared/ModeDockview.tsx#L136-L144)
   에서 `right-[180px]` 매직넘버로 좌표를 손으로 박은 플로팅 버튼.
2. **robot id/type 박스** — [RobotsLayout.tsx:49-55](../frontend_v2/src/pages/RobotsLayout.tsx#L49-L55)
   의 우상단 플로팅 박스. 씬 위에 상시 떠 있음.
3. **패널 추가/삭제 불가** — [ModeDockview.tsx:51-53](../frontend_v2/src/components/shared/ModeDockview.tsx#L51-L53)
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
  목록 소스 = [registry.ts](../frontend_v2/src/components/panels/registry.ts) 의
  `PANEL_COMPONENTS` 에서 "현재 mode 의 PanelSpec 후보 중 미배치" 필터. (mode 별
  후보는 각 robotModes 파일 / TasksPage 의 PANELS 선언 참조)
- `⋯` 메뉴 — "레이아웃 초기화"(현 handleReset). Reset 은 이제 비상용 강등.
- **robot id/type 은 헤더에 넣지 않음 = 완전 제거**. 사이드바에 이미
  `so101_6dof_0` 이 있어 순수 중복 ([RobotsLayout.tsx:49-55](../frontend_v2/src/pages/RobotsLayout.tsx#L49-L55)
  플로팅 박스 삭제).

### 2.3 패널 닫기 활성화

- `LockedTab` 의 `hideClose` **제거** → 패널별 X 로 닫기 가능.
- `+ 패널 추가` 와 **반드시 세트** (닫아도 다시 추가 가능해야 실수 복구됨).

---

## 3. 어떻게 (구현 — 좌표 수학 0)

### 3.1 "패널 위 제외"는 이미 공짜

dockview rect 를 뽑아 매 프레임 비교할 필요 **없음**. 이 workspace 의
pointer-events 정책이 이미 "패널 위 vs 빈 영역"을 브라우저 히트테스트로 구분함
([workspace-dockview.css:3-12](../frontend_v2/src/styles/workspace-dockview.css#L3-L12)):

- dockview 래퍼([ModeDockview.tsx:126](../frontend_v2/src/components/shared/ModeDockview.tsx#L126))
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
| [ModeDockview.tsx](../frontend_v2/src/components/shared/ModeDockview.tsx) | Reset 플로팅 버튼 제거 → 헤더로 이동, `LockedTab` 의 `hideClose` 제거, 새 헤더 컴포넌트 마운트, `+ 패널 추가` 로직(api.addPanel + 미배치 필터) |
| 새 `AutoHideHeader` 컴포넌트 | reveal/hide 타이머, elementFromPoint 트리거, 힌트, 드롭다운/메뉴, pin |
| [workspace-dockview.css](../frontend_v2/src/styles/workspace-dockview.css) | 헤더/힌트 스타일 |
| [RobotsLayout.tsx](../frontend_v2/src/pages/RobotsLayout.tsx) | 우상단 meta box(49-55) 제거 |

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

패널 → 요구 capability 선언은 [registry.ts](../frontend_v2/src/components/panels/registry.ts)
의 `PANEL_CATALOG` 한 곳 (`scan`/`livePointCloud` → `["rgbd"]`). 선언 없으면 요구
없음 = 항상 활성. capability 어휘는 **robots.yaml SSOT** (프론트 `useRobots()` 노출)
— 프론트에 별도 `RobotCapability` union 을 만들지 않음(어휘 이중화 = drift). 값은
`string[]`; 컴파일타임 안전이 필요하면 백엔드 Pydantic `Literal` 승격 → contract
regen 이 정석.

### 7.2 부족 사유 = capability 에서 파생

문구는 패널마다 손으로 쓰지 않고 [lib/capabilities.ts](../frontend_v2/src/lib/capabilities.ts)
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
