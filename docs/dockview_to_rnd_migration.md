# dockview → react-rnd migration plan

> **이 문서는 다음 세션 (신선한 context) 의 Claude / 사용자가 react-rnd 도입을 한 번에 깨끗하게 진행할 수 있도록 작성된 작업 지시서다.** 이전 세션의 진단 history + 실제 요구사항 + 설계 + fallback 절차를 self-contained 하게 담는다.

## 1. 배경 — 왜 dockview 를 버리려 하는가

[frontend/src/pages/RobotsPage.tsx](../frontend/src/pages/RobotsPage.tsx) / [TasksPage.tsx](../frontend/src/pages/TasksPage.tsx) 에서 `dockview-react` 의 `DockviewReact` 를 사용. 사이드바 NavLink 로 `/robots/:id` ↔ `/tasks/:name` ↔ `/settings` 등 라우팅 시 **URL 은 바뀌는데 화면이 안 바뀌는** 증상이 재현됨 (브라우저 새로고침은 정상, SPA 라우팅만 막힘).

진단 결과 — **RobotsPage 가 unmount 트리거를 받지만 실제로 unmount commit 까지 가지 못함**. 콘솔 깨끗, React Router 정상. dockview 가 mount 된 상태에서만 발생 (Canvas 단독 / panel 0개 stub 둘 다 실험 — dockview 자체가 막는 게 확정).

### 시도된 fix 와 결과 (이전 세션)

| 시도 | 결과 |
|---|---|
| `useEffect` cleanup 에서 `apiRef.clear()` + listener `dispose()` 명시 호출 | 첫 사이클 OK, 누적 leak 으로 다시 막힘 |
| `<DockviewReact key={layoutKey}>` 제거 (v1 형태) | 효과 없음 |
| `<StrictMode>` 제거 | 효과 없음 (사용자 명시로 다시 켬) |
| `dockview` 6.0.3 → 6.6.1 업그레이드 | 효과 없음 |
| wrapper 의 `pointer-events-none` 제거 + CSS 로 빈 dock 영역만 click 통과 | **부분 fix** — 첫 1-2 사이클 OK, 여러 번 라우팅 후 다시 막힘 |

→ 단일 fix 로는 누적 leak 못 풀음. **우리 사용 패턴 자체가 dockview 의도와 안 맞음** 이 root cause.

### dockview 의 의도 vs 우리 사용

- **dockview 의도**: docking layout manager — main viewport 를 점유하고 multi-panel split + tab groups + popout window + floating overlay 를 종합 관리
- **우리 사용**: 3D 씬이 base (R3F Canvas, `absolute inset-0 z-0`), dockview 는 그 위 `absolute inset-0 z-10` overlay 로 **floating panel 만** 사용

우리는 dockview 의 dock area / tab group / popout / drop target 다 안 씀 — floating overlay 1개 기능만. dockview 입장에서 비표준 사용. dockview 의 mount/unmount cycle 시 dock area / watermark / floating group cleanup race 가 누적 leak 의 원인으로 추정.

## 2. 실제 요구사항 (재정의)

UI 적으로 필요한 것만:

1. **floating panel** — 3D 씬 위에 떠 있는 control panel (Robot State, Motion, Calibration, Calibration Actions, Point Cloud, Scene Controls — RobotsPage / Prompt, Task Progress, Camera Feed, Robot State — TasksPage)
2. **drag** — panel 의 헤더(탭바) 잡고 위치 이동
3. **resize** — panel 의 모서리/엣지로 크기 조절
4. **collapse** — panel 헤더만 보이게 접기/펴기 (`PanelShell` 이 이미 구현)
5. **layout 저장** — drag/resize 결과를 localStorage 에 저장, 다음 mount 때 복원
6. **per-page layout 분리** — RobotsPage 는 `workspace3d.<robot_id>`, TasksPage 는 `tasks.<task_name>` 키

dockview 의 dock area / tab group / popout / drop target / multi-split → **다 안 씀, 안 쓸 예정**.

## 3. react-rnd 도입 설계

### 3.1 라이브러리 선정

- **react-rnd** (https://github.com/bokuweb/react-rnd) — draggable + resizable. ~30KB. 단순, 검증됨, mount/unmount cleanup 안정적.
- 우리 요구사항 1-3 을 그대로 커버. 4-6 은 우리가 wrapper 에서 구현 (이미 [workspaceLayout.ts](../frontend/src/lib/workspaceLayout.ts) 에 localStorage 유틸 있음).

### 3.2 신규 / 변경 파일

**신규**:
- `frontend/src/components/canvas/rnd/FloatingPanel.tsx` — `<Rnd>` 를 wrapping. `PanelShell` 의 collapse 동작 호환. drag/resize 시 onDragStop / onResizeStop 콜백으로 layout 저장.
- `frontend/src/components/canvas/rnd/FloatingWorkspace.tsx` — RobotsPage / TasksPage 의 공통 layout container. `PANELS` 배열을 prop 으로 받아 N 개의 `FloatingPanel` 을 렌더.
- `frontend/src/lib/rndLayout.ts` — drag/resize 결과의 layout serialization (dockview 의 fromJSON/toJSON 대체).

**변경**:
- [PanelShell.tsx](../frontend/src/components/canvas/ui/PanelShell.tsx) — dockview 의 `props.api.group.api.setSize({ height })` 호출하던 부분을 wrapper 가 주입하는 callback (`onSetSize(height)`) 으로 변경. PanelShell 의 dockview 의존 제거.
- [RobotsPage.tsx](../frontend/src/pages/RobotsPage.tsx) / [TasksPage.tsx](../frontend/src/pages/TasksPage.tsx) — `DockviewReact` 자리에 `FloatingWorkspace` 사용. `addPanel` / `onReady` / `onDidLayoutChange` 등 dockview API 호출부 제거. `PANELS` 배열은 그대로 (component key 가 PanelComponentKey type 인 곳을 `keyof typeof PANELS_MAP` 같은 형태로 약간 수정).
- [panelComponents.ts](../frontend/src/components/canvas/dockview/panelComponents.ts) — `IDockviewPanelProps` 의존 제거. panel 컴포넌트들이 props 받는 surface 를 단순 React props 로 통일.
- 각 panel 파일들 (MotionPanel, CalibrationActionsPanel, RobotStatePanel, …) — `IDockviewPanelProps<object>` 타입을 단순 React.FC 또는 `FloatingPanelProps` 로 변경. `props.api.id` 사용하는 부분은 panelId 를 직접 prop 으로 받기.

**제거 (또는 보존)**:
- `frontend/src/components/canvas/dockview/panelComponents.ts` 의 dockview 의존 부분 — 단순 component map 으로 단순화
- `frontend/src/styles/workspace-dockview.css` — react-rnd 용 CSS 로 대체 또는 import 제거

### 3.3 FloatingPanel surface (제안 API)

```tsx
interface FloatingPanelProps {
  id: string;                            // panel 식별자
  title: string;
  icon?: React.ReactNode;
  position: { x: number; y: number };    // 초기 위치
  size: { width: number; height: number };
  minSize?: { width: number; height: number };
  collapsedHeight?: number;              // default 36
  onMove?: (pos: {x: number; y: number}) => void;
  onResize?: (size: {width: number; height: number}) => void;
  children: React.ReactNode;
}
```

PanelShell 이 collapse 토글 시 onResize 콜백으로 높이 변경 알림 → FloatingWorkspace 가 받아서 react-rnd 의 size 갱신 + localStorage 저장.

### 3.4 layout 저장 포맷

dockview 의 `SerializedDockview` 대체. 단순 JSON:

```json
{
  "robot-state": { "x": 16, "y": 16, "width": 260, "height": 270 },
  "motion":      { "x": 296, "y": 16, "width": 320, "height": 360 },
  ...
}
```

[workspaceLayout.ts](../frontend/src/lib/workspaceLayout.ts) 의 `loadLayout` / `saveLayout` 그대로 재사용 가능 — value 타입만 위 dict 로 변경.

### 3.5 panel 별 default layout

현재 RobotsPage / TasksPage 의 `addDefaultLayout` 안에 있는 "왼쪽 위부터 row-wrap 으로 배치하는 로직" 을 `FloatingWorkspace` 내부 또는 `rndLayout.ts` 의 `computeDefaultLayout(panels, containerWidth)` 으로 추출.

## 4. fallback 절차 — react-rnd 가 또 깨지면

작업이 한 번에 다 박힌 후 라우팅 테스트 결과 여전히 막히면:

1. **첫 진단**: react-rnd 의 `<Rnd>` 인스턴스가 RobotsPage 와 같이 unmount 되는지 확인 (React DevTools). unmount 가 되는데도 라우팅이 막히면 dockview 가 root cause 가 아니었다는 결정적 증거 → R3F Canvas / useBridge / 다른 hook 누수 의심으로 즉시 pivot.
2. **revert**: 이 세션 끝 시점의 git commit 으로 `git reset --hard <hash>` — fallback baseline 은 **현재 dockview 6.6.1 + StrictMode + pointer-events 부분 fix 적용된 상태**.
3. **fallback baseline 동작**: dockview 라우팅 leak 잔존 (이전 세션 진단대로 누적). 하지만 react-rnd 도 안 풀면 root cause 가 다른 곳이므로 fallback 해도 어차피 buggy 동일. 그 시점에 R3F / useBridge / 다른 의심처로 진단 pivot.

**revert 가 안전한 commit hash**: 이 세션 마지막 commit (사용자가 직접 박을 commit). branch 보존 권장.

## 5. 다음 세션 first step (구체적 action)

신선한 context 의 Claude 가 이 문서 읽고 바로 진행할 수 있는 step:

1. **이 문서 + [CLAUDE.md](../CLAUDE.md) Frontend 섹션 + [RobotsPage.tsx](../frontend/src/pages/RobotsPage.tsx) / [TasksPage.tsx](../frontend/src/pages/TasksPage.tsx) / [PanelShell.tsx](../frontend/src/components/canvas/ui/PanelShell.tsx) / [panelComponents.ts](../frontend/src/components/canvas/dockview/panelComponents.ts) 을 먼저 읽어 현재 dockview 사용 surface 파악**
2. `pnpm add react-rnd` (frontend dir)
3. §3.2 의 신규 파일 3개 작성 (`FloatingPanel.tsx`, `FloatingWorkspace.tsx`, `rndLayout.ts`)
4. `PanelShell.tsx` 의 dockview 의존 제거 + `onSetSize` callback prop 추가
5. 각 panel 파일 (9개 — robotState/sceneControls/calibration/calibrationActions/pointCloud/prompt/taskProgress/cameraFeed/motion) 의 `IDockviewPanelProps` 타입 의존 제거
6. `RobotsPage.tsx` / `TasksPage.tsx` 의 `DockviewReact` → `FloatingWorkspace` 교체
7. 라우팅 왕복 50회 + drag/resize/collapse/layout 복원 테스트
8. 동작 OK 면 dockview 의존 / `workspace-dockview.css` import 제거 (라이브러리 자체는 `pnpm remove dockview` 는 보류 — 한 commit 사이클은 보존)
9. 사용자가 확인 후 dockview 의존 완전 제거 + commit

## 6. 현재 (이 세션 끝 시점) 의 코드 상태

- `main.tsx` — `<StrictMode>` 적용됨 (사용자 명시)
- `dockview` 6.6.1 (`frontend/package.json`)
- RobotsPage / TasksPage — v1 (Workspace3D 초기) 형태 + 다음 두 변경:
  - wrapper `<div>` 에서 `pointer-events-none` 제거
  - 명시적 `useEffect` cleanup / `dockviewKey` prop 없음 (v1 과 동일)
- `workspace-dockview.css` — `.dv-dockview` / `.dv-groupview` / `.dv-watermark` 에 `pointer-events: none` 박혀있음 (빈 dock 클릭 통과용)

→ **dockview docs 권장 패턴에 맞게 정통화된 상태**. 다만 우리 사용 패턴이 dockview 의도와 안 맞아 누적 leak 잔존 — react-rnd 전환이 진짜 fix.

## 7. 주의사항

- **사용자 명시 결정**: dockview 코드는 이 세션에서 *삭제 안 함*. react-rnd 도입 후 동작 확인까지 보존. 사용자 confirm 후 별도 commit 에서 제거.
- **DSL/Slot 같은 비즈니스 로직과 무관** — 이건 UI layout 라이브러리 교체. backend / task / store / 3D scene logic 무변경.
- **단계적 박지 말 것** — 사용자 명시: "하나씩 했다가 마지막에 잘 안 되면 어떻게 해" → 다음 세션은 §3.2 의 전체 변경을 한 commit 으로 박는다. 그 후 라우팅 테스트.
