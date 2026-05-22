# Task 페이지 아키텍처 패턴

> Workspace3D와 PickAndPlace 같이 **"3D 씬 + 플로팅 패널"** 구조를 가진 페이지들이 3D 씬을 어떻게 공유하는지의 결정사항. PickAndPlace에 grounded detection ([grounded_detection_design.md](grounded_detection_design.md)) 붙이기 위한 사전 리팩토링 계획.

## 목표

- PickAndPlace 페이지를 Workspace3D처럼 "3D 씬 배경 + dockview floating 패널" 형태로 전환
- 두 페이지가 3D 씬을 공유 (jointAngles wiring 등 중복 제거)
- 미래 task 페이지를 위한 framework은 **만들지 않음** — 지금 필요한 만큼만

## 핵심 결정

1. **PickAndPlace = Workspace3D-style 레이아웃**. 3D 씬이 화면 전체 배경, dockview 패널이 그 위에 떠다님.

2. **최소 리팩토링만**. 미래의 N개 task를 위한 framework 안 만듦. Rule of three — 3번째 task 페이지가 생길 때 그때 더 추출.

3. **공유 단위 = `RobotSceneContainer`**. 3D 씬 + jointAngles/TCP wiring을 한 컴포넌트로 묶어 두 페이지가 import.

4. **3D 레이어는 전부 `RobotScene` 안에 마운트**. 가시성은 store null-check로 자동 (데이터 없으면 layer가 null 렌더 → 안 보임). Three.js editor / Blender / Mapbox GL이 쓰는 표준 패턴. MeshLayer / LivePointCloudLayer가 이미 이 방식.

5. **`WorkspaceCanvas` 추출 안 함**. dockview 셋업은 각 페이지가 자기 안에 둠. ~30줄 중복은 받아들임.

6. **panel registry**(`panelComponents.ts`)는 그대로. 페이지가 자기 panel 리스트를 자체 정의.

## 명시적으로 거부된 패턴

세션 중 검토하고 거부한 패턴들. 미래에 재검토하지 않도록 사유 기록.

- **`WorkspaceCanvas` 추상화(`<WorkspaceCanvas layoutKey panels={[]} sceneExtras={...} />`)** — 페이지가 panel과 layer를 각각 prop으로 명시. **거부 사유**: panel 추가 후 짝맞춤 layer를 안 넣으면 silent fail (PromptPanel 넣고 DetectionLayer 안 넣으면 bbox 안 보임). 또한 우리 스케일에 framework은 over-engineering.
- **Module bundle (`features/<feature>.tsx`에 panel + layer + store를 한 객체로)** — VS Code extension 같은 패턴. **거부 사유**: 짝맞춤 문제는 해결하지만 새 추상화 layer 도입, React/R3F idiom 벗어남, 1년 내 예상 규모(~10 task)에 비해 과함.
- **3D 레이어 페이지별 명시(`sceneLayers={<RobotLayer /><MeshLayer />...}`)** — 페이지가 모든 layer를 children으로 명시. **거부 사유**: "어디까지 보편이고 어디부터 task 전용이냐"가 주관적이고 매 layer마다 판단 필요. 3D 엔진 진영의 표준 패턴(전부 scene에 박고 visibility 토글)이 idiom이고 우리 케이스에도 맞음.

## 폴더 구조 변경

**기존**:
```
components/workspace3d/
  3d/, dockview/, panels/, ui/
```

**변경 후**:
```
components/canvas/                ← rename: workspace3d → canvas
  3d/                             ← RobotScene, RobotModel, MeshLayer, PointCloudLayer, AxisFrame, CameraFrustum
  dockview/                       ← panelComponents.ts (panel registry)
  ui/                             ← PanelShell, Section, ToggleRow, MatrixTable
  RobotSceneContainer.tsx         ← 신규 (Phase 1에서 추가)

components/panels/                ← 신규. 모든 패널 flat. subdir 없음.
  RobotStatePanel.tsx             ← workspace3d/panels/에서 이동
  SceneControlsPanel.tsx          ← 동
  CalibrationPanel.tsx            ← 동
  PointCloudPanel.tsx             ← 동
  (Phase 2 추가) PromptPanel.tsx, TaskProgressPanel.tsx, CameraFeedPanel.tsx
```

기존 4개 패널이 `workspace3d/panels/`에서 루트 `panels/`로 이동. import 경로 전부 업데이트 필요.

## Phase 1: 리팩토링 (다음 세션 작업)

**목표**: 동작 변화 없음. Workspace3D는 그대로 작동. RobotSceneContainer가 다른 페이지에서도 import 가능한 상태.

### Step 1: 폴더 rename + 패널 이동

1. `frontend/src/components/workspace3d/` → `frontend/src/components/canvas/` (디렉토리 이름 변경)
2. `frontend/src/components/canvas/panels/*` → `frontend/src/components/panels/*` (파일 이동)
3. 전체 코드베이스의 import 경로 업데이트:
   - `@/components/workspace3d/...` → `@/components/canvas/...`
   - `@/components/canvas/panels/...` → `@/components/panels/...`
   - `panelComponents.ts`의 import도 같이 수정

### Step 2: `workspaceLayout.ts`에 `layoutKey` 인자 도입

함수 시그니처를 페이지별 분리 가능하게 변경:

```ts
// 변경 전
loadLayout()
saveLayout(data)
resetWorkspaceLayout()

// 변경 후
loadLayout(layoutKey: string)
saveLayout(layoutKey: string, data: unknown)
resetWorkspaceLayout(layoutKey: string)
```

- localStorage 키: `workspace.layout.${layoutKey}` 형태 (예: `workspace.layout.workspace3d`)
- `loadCollapsed` / `saveCollapsed`는 panel id 단위 글로벌 맵이라 그대로 유지. 페이지 간 panel id는 서로 다를 거라 충돌 가능성 낮음. (예: Workspace3D는 `robot-state`, PickAndPlace는 `prompt`)
- 만약 페이지마다 같은 panel id 쓰게 되면 그때 collapsed도 페이지별 분리 검토.
- 호출부([Workspace3D.tsx](../frontend/src/pages/Workspace3D.tsx))에 `"workspace3d"` 인자 추가

### Step 3: `RobotSceneContainer` 추출

`frontend/src/components/canvas/RobotSceneContainer.tsx` 신규. [Workspace3D.tsx](../frontend/src/pages/Workspace3D.tsx)의 다음 로직을 이 컴포넌트 안으로 이동:

- `useCalibrationResults()` 호출
- `useRobotStore`에서 `joints`, `jointOffsetsRad` 구독
- `jointAngles` 계산 (라디안 + joint offset)
- `useSceneStore`에서 `options`, `linkVisibility`, `setLinkNames`, `setTcpPos` 사용
- `handleTCPMatrix` 콜백
- `<RobotScene ... />` 렌더링

Props 없음. 두 페이지가 `<RobotSceneContainer />` 한 줄로 마운트 가능해야 함.

### Step 4: `Workspace3D.tsx` 정리

- 3D 씬 wiring 부분(`<RobotScene ... />` 까지)을 `<RobotSceneContainer />` 한 줄로 교체
- `loadLayout()` / `saveLayout(data)` / `resetWorkspaceLayout()` 호출을 각각 `loadLayout("workspace3d")` / `saveLayout("workspace3d", data)` / `resetWorkspaceLayout("workspace3d")`로 변경
- dockview 셋업, panel 리스트(`PANELS`), reset 버튼은 그대로

### Step 5: 검증

```powershell
cd frontend
pnpm lint
pnpm build
```

브라우저(개발 서버 `pnpm dev`)에서 Workspace3D 페이지 열어 확인:
- 3D 씬 정상 렌더 (로봇, 그리드, 메시, 포인트클라우드)
- panel 토글 / collapse / 드래그 / 리사이즈 동작
- reset 버튼 동작
- 새로고침해도 layout 영속

## Phase 2: PickAndPlace 재작성 (별도 세션, Phase 1 후)

이 단계는 큰 윤곽만 기록. 세부는 Phase 1 끝난 뒤 별도 세션에서 결정.

### 페이지

기존 [PickAndPlace.tsx](../frontend/src/pages/PickAndPlace.tsx)는 **완전히 재작성** (이전 세션에서 user 허락받음). Workspace3D 모양을 따름:

```tsx
// 대략적 구조
<div className="...">
  <div className="absolute inset-0 z-0"><RobotSceneContainer /></div>
  <div className="absolute inset-0 z-10 pointer-events-none">
    <DockviewReact components={PANEL_COMPONENTS} onReady={onReady} />
  </div>
  <button onClick={handleReset}>Reset</button>
</div>
```

- `layoutKey`: `"pickandplace"`
- `PANELS` 리스트는 자체 보유

### 신규 패널 (`components/panels/`)

- **`PromptPanel`** — 자연어 입력 + Run/Stop 버튼. `omx/perception/grounded_detect` 서비스 호출 → 응답을 `detectorStore.groundedResult`에 push.
- **`TaskProgressPanel`** — task step 진행 (`taskStore` 읽음). 현재 step 강조, status badge.
- **`CameraFeedPanel`** — live MJPEG (`/camera/stream`) + bbox 오버레이.
- **`RobotStatePanel`** — Phase 1에서 이동한 거 재사용.

### 신규 3D layer

`components/canvas/3d/DetectionLayer.tsx` 신규. `detectorStore.groundedResult`를 읽어 다음을 모두 렌더:

- 베이스 프레임 target marker (구/십자)
- bbox → 3D frustum cone (카메라에서 ray 투영)
- TCP → target까지의 approach path line
- prompt + confidence 텍스트 라벨 (`<Html />` 또는 sprite)

[RobotScene.tsx](../frontend/src/components/canvas/3d/RobotScene.tsx)에 `<DetectionLayer />` 마운트 한 줄 추가. store가 null이면 자동으로 안 보이므로 다른 페이지에 영향 없음.

### `detectorStore` 확장

```ts
interface GroundedResult {
  position: [number, number, number];  // 베이스 프레임
  bbox2d: { x: number; y: number; w: number; h: number };  // 이미지 px
  prompt: string;
  confidence: number;
}

// 추가 필드
groundedResult: GroundedResult | null
setGroundedResult: (r: GroundedResult | null) => void
```

### 백엔드

[grounded_detection_design.md](grounded_detection_design.md) 참고. 별도 작업.

- DetectorNode 재작성 + Grounding DINO Swin-B
- 서비스 `omx/perception/grounded_detect` 등록
- `pick_named_object` task (TASK_REGISTRY에 추가)

## Phase 2에서 결정할 것들 (지금 정하지 않음)

- Pick&Place의 default panel 배치 (좌상단부터 흐름배치? 고정 사이드바형?)
- 카메라 frustum 등 sceneStore 옵션 default를 페이지별로 다르게 줄지 (현재는 전역)
- 그리퍼 동작, place 좌표 등 task 시나리오 step 구성
- bbox 2D 오버레이 (`CameraFeedPanel` 안) vs 3D viz (`DetectionLayer`) 중복 처리

## 진행 메모

- 이전 세션에서 `workspaceLayout.ts`를 일부 수정했었지만 **원복함**. Phase 1 Step 2에서 새로 작업.
- 다음 세션에서 이 문서를 읽고 Step 1~5를 순서대로 진행하면 됨.
