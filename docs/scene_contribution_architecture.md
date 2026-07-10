# scene_contribution_architecture.md

frontend_v2 워크스페이스에서 **3D 씬에 시각 표현이 들어가는 방법**의 아키텍처 —
**구현 완료 (2026-07-10)**. 패널 capability 게이팅
([workspace_autohide_header.md](workspace_autohide_header.md) §7) 직후 같은 날
설계·구현·재설계(소유권 모델로 1회 정정)까지.

> 진입 톤: "패널에서 3D에 마커/기하 표시" / "scenePart" / "씬 객체" / "RobotFrame" /
> "frustum 어디서 그리나" / "ghost 미리보기" / "씬 기여 DX" 나오면 본 문서.

**anchor 문장**:

> 프레임워크는 "Scene에 어떻게 참여하는가"를 책임지고, 개발자는 "무엇을 어떻게
> 그릴 것인가"를 책임진다. 좌표계 선택은 렌더링 내용의 일부이므로 프레임워크가
> 숨기지 않는다.

---

## 1. 왜 (문제)

패널(dockview overlay, z-10)과 3D 씬(R3F Canvas, z-0)은 **별도 React 트리**.
기능 하나가 3D 표현을 가지려면 종래엔 4중 편집이 반복됐다: Layer 컴포넌트 신규 +
Scene.tsx 하드코딩 JSX 수정 + store 신규 + `robotBaseMatrix` decompose ~20줄 복붙.
backend framework 원칙과 같은 판정: **반복 보일러플레이트를 제거하는가**
([task_imperative_framework.md](task_imperative_framework.md) §1) → 승격 대상.

## 2. 소유권 모델 (1차 판정 기준)

씬에 보이는 모든 것은 **"누가 소유하나"** 로 분류된다. 이것이 1차 기준 — "무겁나 /
독립 수명이냐"는 부차적 결과다.

| 부류 | 소유자 | 예 | 폴더 | 등록 |
|---|---|---|---|---|
| **Core chrome** | 씬 자체 | 조명/grid/BASE 축/OrbitControls | [Scene.tsx](../frontend_v2/src/components/scene/Scene.tsx) | 하드코딩 |
| **Scene object** | 세계(하드웨어/산출물) | Robot / **Camera(frustum+cloud)** / ScanMesh | [scene/objects/](../frontend_v2/src/components/scene/objects/) | Scene.tsx 한 줄 (드문 아키텍처 사건) |
| **Feature overlay** | 기능 | TaskResultsOverlay(topic 수명) / **scenePart**(패널 수명) | [scene/overlays/](../frontend_v2/src/components/scene/overlays/) + 패널 폴더 | registry `scenePart:` 한 줄 |

공용 primitive(RobotFrame/AxisFrame/primitives/transforms)는
[scene/shared/](../frontend_v2/src/components/scene/shared/) — 폴더 구조가 이 표를
그대로 반영.

**판별 질문 (개발자 가이드)**:

> **"패널을 닫으면 이게 사라져야 하나?"**
> - 사라져야 함 (내 기능이 보여주는 것 — ghost 미리보기, 체커보드) → **scenePart**
> - 남아야 함 / 여러 패널이 같은 걸 원함 (세계에 있는 것 — 카메라 frustum) →
>   **씬 객체의 속성** — 패널은 그리지 않고 store 토글만

**씬 객체는 자기 시각 요소를 자기 안에서 그린다.** Camera 가 pose(tcp·hand_eye)를
한 번 계산해 frustum + live cloud 를 자식으로 렌더 — 어느 패널이 몇 개 열리든
렌더는 카메라당 한 번 (중복이 구조적으로 불가). 패널은 `cameraStore.showFrustum` /
`scanStore.liveEnabled` 토글만. 새 객체 종류(미래: world state 의 Box/Conveyor —
backend stream 이 생기면 `<WorldObjects>` 가 data-driven 으로 N 개)는 Scene.tsx 에
한 줄 추가되는 게 정직하다 — **"Scene.tsx diff 0" 계약의 대상은 기능/패널 기여**
(scenePart/토글)이지 객체 종류 추가가 아님.

## 3. scenePart 메커니즘 (기능 오버레이의 패널 수명 형태)

개발자가 쓰는 것 — 패널 폴더에 R3F 조각 + registry 한 줄:

```
panels/WaypointPanel/
  index.tsx      ← React UI ([보기] 토글 버튼)
  scenePart.tsx  ← R3F 조각 (제약 없음 — useFrame/shader/drei 자유)
```

```tsx
// scenePart.tsx — 패널 코드와 같은 멘탈모델
export function WaypointScenePart() {
  const robotId = useRobotId();               // 패널에서 쓰던 그 훅
  const preview = useWaypointStore((s) => s.previews[robotId]);
  if (!preview) return null;
  return <RobotModel jointAngles={preview.jointAngles} opacity={0.35} tint="#34d399" ... />;
}
// registry.ts:  waypoints: { title, ..., scenePart: WaypointScenePart }
```

프레임워크가 해주는 것 (배선 전부):

1. **인스턴스 추적** — [withRobotOwnership](../frontend_v2/src/components/shared/robotOwnership.tsx)
   HOC(chokepoint)가 [panelInstanceStore](../frontend_v2/src/stores/panelInstanceStore.ts)
   에 `(useId, panelKind, robotId)` 등록/해제. **바인딩 + capability OK 일 때만**
   → unsupported robot 이면 scenePart 자동 미표시.
2. **마운트** — Canvas 의 [ScenePartHost](../frontend_v2/src/components/scene/overlays/ScenePartHost.tsx)
   가 인스턴스 × `PANEL_CATALOG.scenePart` 교집합을 인스턴스별 렌더. 같은 패널
   2개(robot A/B)면 조각 2개, 각자 자기 robot.
3. **robot 공급** — 각 조각을 `<RobotProvider>` 로 감쌈 → `useRobotId()`/`useStream`
   패널 그대로.
4. **좌표 primitive** — [RobotFrame](../frontend_v2/src/components/scene/shared/RobotFrame.tsx)
   (robotId 생략 = context 의 자기 robot) + [primitives](../frontend_v2/src/components/scene/shared/primitives.tsx)
   (`<Frame>`/`<Marker>`/`<BoxOutline>`/`<PolyLine>` — 안 쓰면 그만).

**경계를 runtime 에 넘는 것은 인스턴스 목록(순수 데이터)뿐** — scene 컴포넌트는
registry 정적 등록 (identity 안정).

### 데이터 공유 (알고 쓸 것)

scenePart 는 Canvas 트리 렌더 → 패널 로컬 useState 는 안 넘어감. 경로 둘:
framework hook 재구독(useStream/useMirror — module cache, 대부분) / **feature
store**(패널 선택값 — waypointStore/scanStore 패턴). "UI 로 3D 를 제어"(토글류)도
같은 store 경로. 인스턴스-스코프 공유 슬롯은 rule-of-three 대기.

## 4. 좌표계 — 명시적 `<RobotFrame>` (auto-wrap 기각)

scenePart/씬 객체는 world frame 에서 시작, robot base frame 좌표(백엔드 숫자)는
`<RobotFrame>` 으로 명시적으로 감싼다. z-up→y-up + base_pose 수학은
RobotFrame/transforms.ts SSOT. R3F 에서 transform 은 원래 트리에서 명시하는 것
(`<group position>`) — RobotFrame 은 그 문법의 robot 좌표계 버전일 뿐, DSL 아님.

## 5. 기각 결정 (재론 방지 — 근거 포함)

- **descriptor DSL** (`overlay.set([{kind:"frame",...}])`) — 기각. 새로 배울 어휘 +
  표현력 인위적 천장. backend Step/Slot DSL 폐기와 동일 판정.
- **runtime JSX 주입** (tunnel-rat 식) — 기각. Canvas 트리 렌더 시 RobotContext
  단절 footgun + 엘리먼트 identity 를 작성자 규율에 맡김(emitTCP 무한루프 전력,
  commit f15a20b).
- **자동 RobotFrame 래핑** — 기각. mixed-frame(robot 기하 + world 기하 형제)이
  scenePart 단위 opt-out 으로 불가능(표현력 천장) + opt-out flag 는 config DSL
  재발 + 좌표계는 wiring 이 아니라 content 의미론.
- **"여러 패널이 같은 걸 원하면 Tier 1 layer" 규칙** — 기각 (2026-07-10 정정).
  증상(중복 렌더) 기반 규칙 — Robot/Grid 도 여러 패널에서 의미 있으니 Layer 가
  무한증식하는 함정. 올바른 기준은 §2 소유권. **"Layer" 어휘를 도메인 단위로 쓰지
  않는다** — 씬 객체가 조직 단위.
- **render-pass Layer 체계** (Opaque/Transparent/Gizmo) — defer. 엔진 표준이지만
  현재 소비자 0 (렌더 패스/소팅 문제 없음) — 필요가 생길 때.
- **파생 pose 의 DB 저장** (waypoint teach 시점 tcp pose 컬럼) — 기각·revert
  (2026-07-10). pose = joints + 현재 캘의 파생값 — 저장하면 캘 재커밋 시 silent
  stale. scan 의 "raw 만 저장, 파생은 fresh 계산" 원칙과 동일. waypoint 3D 는
  joint-space 데이터에 정직한 **ghost 미리보기**(URDF 재사용, backend 무변경)로.
  cartesian 마커/그룹 polyline 이 필요해지면 그때 motion FK 서비스(fresh 계산) 검토.
- **runtime `registerSceneLayer()` 호출식** — 기각. 정적 선언이 패널 registry 와
  동형, import-side-effect 등록은 의존이 숨음.

## 6. 성능 한계 (명시)

scenePart/primitive 는 React state 경로 — **소량 + 중저빈도** 기하용. 고빈도
대용량(포인트클라우드)은 씬 객체가 dynamic buffer 직접 관리 (Cameras 의
CameraCloud 패턴 — 옛 Scene3DLayer 로직 그대로, Mirror hand_eye 는 "토글 시 1회
fetch 가 identity 로 굳던" 사고의 fix 라 불변).

## 7. 기존 구현의 의도 보존 (이사 기록)

- **Scene3DLayer → Cameras 로 흡수** — cloud 는 "카메라가 보는 센서 데이터"라
  Camera 씬 객체의 자식. base·tcp·handEye pose 계산이 frustum 과 한 곳으로 수렴
  (옛 2중 계산 제거). buffer/구독 로직 불변.
- **scanStore / cameraStore / waypointStore** — 패널 ↔ Canvas 브리지 store 패턴.
  scanStore(liveEnabled/voxel/pointSize/mesh) 유지, cameraStore(showFrustum) 신규,
  waypointStore(ghost preview) 신규.
- **TaskResultLayer → TaskResultsOverlay** — task 기능 소유, topic 수명 (결과는
  패널보다 오래 사는 진단 도구). extractMarkers/렌더 불변.
- **Container 의 scanRobotId/scanBaseMatrix** — 각 객체 안으로 이동 ("대상 robot =
  focus ?? 첫 robot" 은 ScanMesh/TaskResultsOverlay 에, 카메라는 rgbd capability
  파생). N robot 동시 라이브 시 store dict 화 경로는 각 store 주석.
- **sceneOptions** — Core 전용 토글 유지.
- 컴포넌트 rename: RobotLayer→Robots / MeshLayer→ScanMesh /
  TaskResultLayer→TaskResultsOverlay (Layer 어휘 제거).

## 8. 첫 소비자들

- **Camera 씬 객체** ([objects/Cameras.tsx](../frontend_v2/src/components/scene/objects/Cameras.tsx))
  — D405 frustum + live cloud. 처음엔 LivePointCloudPanel 의 scenePart 로 지었다가
  "캘 패널에서도 frustum 보고 싶다"(여러 패널이 같은 것) 요구가 소유권 오류를
  드러내 씬 객체로 승격 — §2 판별 질문의 실전 사례. 캘/라클 패널에 `시야` 토글.
- **WaypointScenePart** ([panels/WaypointPanel/scenePart.tsx](../frontend_v2/src/components/panels/WaypointPanel/scenePart.tsx))
  — scenePart 레퍼런스. waypoint [보기] 버튼(명시 토글, hover X) → 그 joint 자세의
  **반투명 emerald ghost** (RobotModel 재사용 — tint prop). "MoveJ 하면 어떤
  자세가 되나"를 실행 전에 봄. waypoint 는 joint 구성이라 점 마커가 아니라 ghost 가
  정직한 시각화 (팔꿈치 configuration 까지 보임). cartesian 지도/그룹 polyline 은
  후속 (§5 파생 pose 기각 참조).

## 9. 색 시스템 (시각적 의미 SSOT)

씬 색은 **역할(의미)** 로 고정 — hex 를 파일마다 새로 고르면 체계가 조용히
무너지므로 [scene/theme/visualizationColors.ts](../frontend_v2/src/components/scene/theme/visualizationColors.ts)
의 `VizColor` 토큰 한 곳이 SSOT. 새 시각화는 hex 가 아니라 역할을 고른다.

| 토큰 | hex | 의미 | 소비자 |
|---|---|---|---|
| `PREVIEW` | violet `#8b5cf6` | 가상·예측 (command preview / ghost / simulation) | waypoint ghost |
| `SENSOR` | blue `#66ccff` | 센서 계열 | camera frustum, PolyLine default |
| `DETECTION` | emerald `#34d399` | 인식 결과 / attention | task 검출 마커, Marker default |
| `TARGET` | orange `#f59e0b` | 작업 목표 "여기로 가야 한다" | grasp/place 마커, BoxOutline default |
| `TCP` | amber `#ffcc44` | 로봇 기준 프레임 "현재 손 끝" | TCP frame label |
| `CANDIDATE` | gray `#71717a` | 후보 / 비활성 | 검출 후보 |
| (실물) | tint 없음 | real world object | RobotModel 원본 material |

**TARGET ≠ TCP (분리 불변식).** 둘 다 주황 계열이지만 "가야 할 목표(Task/Planning
결과)"와 "현재 로봇 기준점(항상 존재)"은 **동시에 화면에 뜨는 다른 개념** — 같은
색이면 순간 구분 불가라 색으로 갈린다. 합치지 말 것.

**축 색은 이 체계가 아니다.** AxisFrame 의 X=red/Y=green/Z=blue 는 좌표축 관례
(RGB=XYZ)지 의미 색이 아님 — VizColor 에 넣지 않는다. 축에 red 를 썼다고 "warning"
토큰을 끌어오면 두 체계가 무너진다.

**미래 팔레트 (소비자 생기면 토큰 추가 — 지금 코드엔 없음).** 디지털 트윈 확장 시
같은 체계를 그대로: box ghost = PREVIEW, 센서 영역 = SENSOR, collision/constraint =
red/orange(warning), trajectory preview = PREVIEW. 소비자 없는 상수 선제작은 안 함
(§5 원칙) — 문서에 방향만 박고 그때 `VizColor` 에 한 줄.

## 10. 검증

- vitest **144/144** (34 파일) / lint 0 error / `tsc -b` green (2026-07-10):
  panelInstanceStore(5)/RobotFrame(4)/ScenePartHost(4)/Cameras(4)/cameraPose(3)/
  HOC 등록(5)/waypointStore(2)/WaypointScenePart(3)/패널 ghost wire(2) + 기존.
- 픽셀(ghost 색/frustum 위치)은 headed 검증 몫 (jsdom 은 Canvas 를 못 그림).
