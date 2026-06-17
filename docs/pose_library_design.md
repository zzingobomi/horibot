# Pose Library + Plan Curation + Ghost Preview — Design

> **요약** — 현재 `robot/instances/<robot_id>/robot_poses.yaml` 은 정적 yaml 1개 + 코드 hardcoded 로 사람이 직접 편집. multi-robot 으로 가면서 + scan workflow 가 user-curated pose 시퀀스를 필요로 하면서 한계 노출. 본 문서는 (a) **runtime-mutable pose library**, (b) **ordered scan plan + skip/reorder**, (c) **ghost robot preview primitive** 세 자리를 design. prior art 는 §2, 핵심 결정은 §3, entity/스키마는 §4, ghost primitive 는 §5, 백엔드 계약은 §6, 유저 플로우 walkthrough 는 §7, 마이그레이션 phase 는 §10.
>
> **scope 외** — 본 문서는 design only. 구현은 합의 후 별도 세션. trajectory replay (animated ghost playback), frame-relative pose (Polyscope X 의 feature frame), cross-robot pose sharing 은 §11 후속.

## 1. 동기

### 1.1 현재 상태

- [robot/instances/omx_f_0/robot_poses.yaml](../robot/instances/omx_f_0/robot_poses.yaml) — yaml 1개에 `home` / `rest` / `search_1~3` hardcoded
- [backend/core/robot/robot_poses.py](../backend/core/robot/robot_poses.py) — module-level cache (`_cache`) 1회 로드 후 in-memory. **runtime 변경 mechanism 없음** (`reload()` 같은 invalidate 도 없음)
- recipe 가 이름으로 의존 — [recipes.py](../backend/modules/task/recipes.py) 의 `home()` → `load_pose("home")`, `search_and_detect()` → `list_pose_names("search_")`
- ScanTask 의 [DEFAULT_SCAN_POSES = ["home"]](../backend/modules/task/tasks/scan.py#L27) — placeholder, 실 운영 자리 hardcoded
- frontend 에 자세 관리 UI 없음 — 사용자가 yaml 직접 편집

### 1.2 발견된 한계

| 자리 | 한계 | 사용자 영향 |
|---|---|---|
| **runtime 편집** | yaml 직접 편집 → backend 재시작 | jog 으로 자세 잡고 즉시 저장 불가 |
| **scan 자세 큐레이션** | hardcoded `DEFAULT_SCAN_POSES`, 매번 코드 수정 | 책상 / 물체 위치 바뀌면 작업 ↑ |
| **자세 순서 / skip** | 없음 (list 순서만, skip 개념 없음) | 일부 자세만 빼고 실행하려면 코드 수정 |
| **다음 동작 예측** | 없음 | "MoveJ home 누르면 어디로 가지?" 사전 확인 불가, 캘 추천 자세도 숫자만 표시 |
| **multi-robot** | yaml 자리는 instance 별로 분리되어 있긴 함, 하지만 추천 strategy 등 코드도 hardcoded | so101_6dof 새 robot 추가 시 yaml 직접 만들기부터 시작 |

### 1.3 사용자 요구 (대화에서 추출)

1. 홈/rest/스캔 자세 모두 영속 저장 (yaml 또는 DB)
2. **현재 자세 → 이름 붙여 저장** (jog 으로 잡은 자세를 클릭 1회로 라이브러리에 박기)
3. 자세 **드래그 앤 드롭으로 순서 지정** (스캔 시퀀스)
4. 자세 **skip 토글** (시퀀스 안 일부만 빼고 실행)
5. **자세 preview** — UI 에서 "이 자세 누르면 로봇이 어디로 가는지" 미리 보기. MoveIt 의 orange ghost 같은 자리
6. 자세 / preview 기능은 스캔뿐 아니라 **경로계획 / 캘 / 어디서나 재사용** 가능해야 함

## 2. Prior art

자세 카운트 풀 스택의 8개 시스템 조사. 자세한 내용은 별도 리서치 노트, 본 절은 design 에 직접 영향 준 결론만.

### 2.1 시스템 매트릭스

| 시스템 | 자세 SSOT | 자세 CRUD UI | Plan ordering | Ghost preview |
|---|---|---|---|---|
| **MoveIt + RViz** | SRDF `<group_state>` (코드) + warehouse_ros (runtime, SQLite/Mongo) — *이원화* | RViz "Stored States" 탭 (save/load/delete, flat keyed) | 없음 (library 는 flat) | ✅ orange=goal / green=start, 단일 ghost + trajectory replay |
| **MoveIt Pro** | `waypoints.yaml` 단일 (runtime-mutable, ROS package 안) | "Create Waypoint" 버튼 (모든 teleop 탭 navbar) | Behavior Tree XML (별도 entity) | RViz 상속 |
| **UR Polyscope** | 프로그램 트리 안 Waypoint (별도 라이브러리 아님) | jog → "Set this waypoint" | **same name = same waypoint** (rename = 전역 update). reorder = Cut/Copy/Paste | ❌ live robot = preview (freedrive jog) |
| **Polyscope X** | + Variable Waypoints (`p[x,y,z,rx,ry,rz]` 런타임 변수) + feature frame relative | 같음 | multi-node clipboard | ❌ |
| **RoboDK** | Station Tree 의 Target item (reference frame parent), `.rdk` 바이너리 | Ctrl+T "Teach Target", joint-mode vs Cartesian-mode | program tree 의 MoveJ/L node 순서 | ✅ **hover-to-preview on surface** (마우스 hover → ghost), click=move playback, double-click=teleport |
| **Mecademic MecaPortal** | 웹 브라우저, schema 미공개 | 웹 UI | 미공개 | 미공개 |
| **Foxglove 3D Panel** | 시각화 only, pose 라이브러리 없음 | — | — | **multi-URDF in one scene, 독립 joint driver** — 우리 RobotModel 과 동형 |
| **Isaac Sim Robot Poser** | USD prim `IsaacNamedPose` (robot asset 안에 박힘) | Table view + Add/Apply | 없음 | live teleport (pause 자리) |
| **ABB RAPID** | `jointtarget` / `robtarget` named variable (코드) | FlexPendant "ModPos" | RAPID 프로그램 순서 | RobotStudio 시뮬레이션 (단일 ghost 자리 불명확) |

### 2.2 차용 결론

1. **library 는 flat keyed, 순서는 plan 자리** — 메이저 시스템 누구도 library 자체에 drag-reorder 없음. Plan/program 이 reorder 의 자연 자리. Horibot 의 typed Slot DSL 이 plan 자리.

2. **system pose vs user pose 의 이원화는 흔하지만 (MoveIt SRDF vs warehouse), 최근 trend 는 단일 mutable 자리로 collapse** (MoveIt Pro `waypoints.yaml`). 이유 — "home 자세 추가하려고 Setup Assistant 다시 돌리기" 마찰 제거. Horibot 도 단일 자리로 갈 수 있음. 코드 reference (`home` / `search_*`) 는 그냥 name 으로.

3. **yaml vs DB** — MoveIt Pro = yaml, MoveIt warehouse = SQLite/Mongo, Polyscope = 프로그램 파일 내장, Isaac = USD prim, ABB = 코드 변수. **단일 사용자 / single PC 자리 yaml 이 산업 표준.** DB 는 multi-user / history query 가 필요할 때만.

4. **ghost preview 의 4가지 패턴** — (a) static dual ghost (MoveIt), (b) hover-to-preview (RoboDK 의 surface teaching), (c) click/teleport (Isaac, Polyscope), (d) animated trajectory replay (MoveIt Show Trail). Horibot 의 첫 사이클은 **(a) 단일 static ghost + (b) hover-to-preview**, (d) 는 후속.

5. **색 컨벤션** — MoveIt orange=goal / green=start 가 가장 인지도 높음. Horibot 은 robot 본체 보통 회색이라 **orange=preview ghost** 차용.

6. **Foxglove pattern 이 우리 인프라와 동형** — 같은 URDF 두 번 마운트, 각각 다른 joint driver, transparency 다르게. 우리 [RobotModel.tsx](../frontend/src/components/scene/RobotModel.tsx) 가 이미 `opacity` / `jointAngles` instance per 받음 → ghost = 두 번째 마운트.

7. **Polyscope 의 same-name=same-waypoint** 은 강력하지만 rename 의미가 비명시적 → 첫 사이클 skip. 차용 X.

8. **MoveIt Pro 의 단일 `waypoints.yaml` + frontend 가 runtime mutate** 가 가장 깔끔한 SSOT 패턴 — 본 문서가 채택.

## 3. 핵심 design 결정

> 사용자가 명시적으로 묻지 않는 한 본 절의 결정은 **합의 후 변경 가능, 합의 전엔 default**. 진짜 갈래는 §3 마지막의 "남은 결정 1개" 만.

### 3.1 SSOT — yaml 단일 파일 per robot instance

**결정**: `robot/instances/<robot_id>/poses.yaml` (현재 `robot_poses.yaml` 에서 rename) 가 SSOT. backend service 가 atomic write (temp file + rename) 로 mutate. RDB 미사용.

**근거**:
- MoveIt Pro / Polyscope / Isaac 자리 다 *robot asset 옆 파일* — git-trackable, robot 따라 이동, 다른 PC clone 시 즉시 동작
- pose 는 blob 없음 (joint vector ~6 float) — StorageNode 의 RDB+ObjectStore 패턴 (scan workflow 의 무게) 과 비대칭, 과한 인프라
- 단일 사용자 / 단일 운영 PC — 동시 편집 race 없음. 분산 (Pi) 자리에서 pose name 해석은 PC (TaskNode) 안에서만 일어남 (Pi 의 MotionNode 는 raw joint 받음, pose name 모름)
- git diff 가 "어제 누가 home 자세 바꿨네" 의 자연 audit log

**대안**: StorageNode 의 RdbStore 에 `poses` 테이블 추가 (캘 / scan_workflow 와 같은 패턴). reject — pose 라이프사이클이 캘 (active + invalidation), scan (append-only blob) 어느 쪽과도 다름. 단순 named lookup + low write volume = yaml 적합.

### 3.2 Entity 두 개 — Pose (flat library) + Plan (ordered ref list)

**결정**: 두 entity 분리, 각각 다른 yaml.

```
robot/instances/<robot_id>/
  poses.yaml          # flat keyed library — name → joints
  plans.yaml          # named ordered plans — name → ordered pose refs + skip
```

**근거**:
- §2.2 결론 1 — library 는 flat, 순서는 plan 자리 (산업 표준)
- 두 entity 는 라이프사이클 다름 — pose 는 사람이 한 번 박고 거의 안 바뀜, plan 은 작업마다 큐레이션
- 사용자 요구 (3) (4) (드래그 reorder, skip) 가 plan 만의 속성

**대안**: 단일 파일에 두 섹션. reject — yaml 안 두 entity 보다 두 파일이 atomic write / version control diff 더 깨끗. 두 파일 모두 robot asset 옆 → 같이 이동 / 같이 git tracked.

### 3.3 Pose 종류 = joint-space only (1차)

**결정**: pose value 는 joint 각도 (degree, motor id 1-based) 만. Cartesian pose / TCP pose 미지원.

**근거**:
- RoboDK 의 joint-mode vs Cartesian-mode 구분은 reference frame 시스템 (Polyscope X 의 feature frame) 이 있을 때 의미 — Horibot 은 base frame 하나뿐
- joint-mode 는 calibration 변경 / URDF 패치에 강함 ([docs/calibration_apply_flow.md](calibration_apply_flow.md) 자리 raw joint 가 invariant 인 패턴과 정합)
- Cartesian pose 가 필요한 자리는 별도 entity (waypoint?) 후속 (§11)

**대안**: 처음부터 Cartesian + joint 양쪽 지원. reject — 첫 사이클 over-engineering. user 가 Cartesian 자리 필요한 상황 등장하면 그때 추가.

### 3.4 System pose 와 user pose 미구분 (단일 mutable layer)

**결정**: yaml 안 모든 자세는 동등하게 user-editable. 다만 **recipe 가 코드에서 참조하는 이름** (`home`, `search_*`) 은 yaml 자체에 `protected: true` flag 박아 UI 가 삭제/이름 변경 차단 (값 수정은 허용).

**근거**:
- §2.2 결론 2 — MoveIt Pro 패턴이 이쪽 (단일 layer + runtime mutable)
- code-referenced name 을 사용자가 모르고 삭제하면 recipe break — protected flag 가 hard guard
- 값 수정은 허용해야 함 (home 위치 책상에 맞게 tune)

```yaml
home:
  joints: [...]
  protected: true   # name 삭제/변경 차단
  note: "task 시작 자세"
search_1:
  joints: [...]
  protected: true   # search_* prefix 가 list_pose_names 검색 대상
```

**대안**: system pose 는 별도 yaml (또는 코드 hardcoded), user pose 는 mutable layer. reject — 이원화의 마찰 (예: 사용자가 "system home 위치가 안 맞아" 자리에서 yaml 직접 편집 vs UI 자리 갈등) 이 MoveIt 의 SRDF vs warehouse 자리에서 이미 검증됨. 단일 layer + protected flag 가 더 깔끔.

### 3.5 Ghost preview = Foxglove pattern (multi-URDF in scene, 독립 joint driver)

**결정**: 기존 [RobotModel](../frontend/src/components/scene/RobotModel.tsx) 컴포넌트를 한 번 더 마운트 (`<RobotPreviewLayer />`). 동일 URDF, opacity 0.3, orange tint. joint 는 `previewStore` 에서 받음.

**근거**:
- §2.2 결론 4, 6 — Foxglove 가 우리 인프라와 동형, RobotModel 이 이미 instance-per-opacity 지원
- 새 컴포넌트 거의 안 만들고 prop 한두 개 추가 (tint) + 두 번째 마운트 + 새 store 만
- 다양한 trigger 가 같은 primitive 위에 — pose hover, 캘 추천, MoveJ target, plan playback 다 `setPreview(joints)` 로 통일

**대안 1**: 별도 단순 시각화 컴포넌트 (joint sphere markers). reject — URDF 가 이미 있는데 단순화하면 "ghost 자리 실제 로봇 모양" 의 직관 잃음.

**대안 2**: trajectory animation (애니메이션 replay). 첫 사이클 X — primitive 만 static frame, animation 은 후속.

### 3.6 Trigger 는 다양, store 는 단일 (`previewStore`)

**결정**: `previewStore.setGhost(robotId, joints | null)` 단일 API. caller 자리 누구든 (pose list hover, plan list hover, 캘 추천 hover, MoveJ form 입력) 같은 store 호출. 한 번에 표시되는 ghost 는 최대 1개 per robot (다중 ghost 는 후속).

**근거**:
- 사용자 요구 (6) — 어디서나 재사용
- 단일 ghost 면 store 모양 단순 — `{ robotId: joints }` dict
- 동시에 두 곳에서 trigger 시 마지막 setter wins (마우스 hover 자연스러움)

**대안**: trajectory ghost (N waypoint) 동시. 후속 — first cycle 미적용, store 모양 확장 여지만 남김.

### 3.7 사용자에게 묻는 단일 분기

위 6개 결정은 본 문서가 한 안 (default). 사용자 결정 갈리는 자리는 정확히 한 자리:

**Q**: rename / migrate 시 기존 `robot_poses.yaml` → `poses.yaml` 으로 파일 이름 자체를 바꿀까, 아니면 기존 이름 유지하고 schema 만 (add protected flag) 확장할까?

- **option (i)** `poses.yaml` 로 rename — MoveIt Pro 컨벤션 일치, 이름 짧음. 코드 (`RobotConfig.robot_poses_yaml` 같은 path 자리, `robot_poses.py` 모듈명) 다 같이 rename.
- **option (ii)** `robot_poses.yaml` 유지 — git diff 가 깨끗 (rename 자체 안 보임), 기존 코드 변경 최소.

추천 **(i)** — design 새로 짜는 자리에서 이름 깨끗하게. 코드 rename 비용은 grep + replace 한 번. **반대 의견 없으면 (i) 로 진행.**

다른 디테일 (yaml 키 이름 / store API 시그니처 / panel 배치 등) 은 §4~§9 본문에서 본 문서가 1안 으로 박음 — 본문 읽으며 동의 안 되는 자리만 짚으면 변경.

## 4. Entity 스키마

### 4.1 `poses.yaml`

```yaml
# robot/instances/<robot_id>/poses.yaml
# Pose library — flat keyed (name → pose definition).
# Recipe 가 코드에서 이름으로 참조 (load_pose("home"), list_pose_names("search_")).
# protected=true 면 frontend 가 삭제/rename 차단 (값 수정은 허용).

poses:
  home:
    joints:
      - { id: 1, degree: 0 }
      - { id: 2, degree: -60 }
      - { id: 3, degree: 25 }
      - { id: 4, degree: 80 }
      - { id: 5, degree: 0 }
    protected: true
    note: "Task 시작 자세"
    created_at: "2026-06-17T18:00:00"
    updated_at: "2026-06-17T18:00:00"

  rest:
    joints: [...]
    protected: true
    note: "Park 자세 — 토크 풀어도 안전"

  search_1:
    joints: [...]
    protected: true   # search_ prefix 가 recipe 의 list_pose_names 대상
    note: "좌측 시야 — J1 +30°"

  # User 가 jog → "현재 자세 저장" 으로 만든 자리
  scan_top:
    joints: [...]
    protected: false
    note: "책상 위에서 내려다보기"
```

### 4.2 `plans.yaml`

```yaml
# robot/instances/<robot_id>/plans.yaml
# Named ordered plans — sequence of pose refs with skip flag.
# 첫 사이클 사용처: ScanTask. 후속에 MotionPlan (move sequence), CalibrationCapturePlan 등.

plans:
  table_scan_default:
    kind: scan           # 어떤 task 가 사용하는지 (선택적 필터, 강제 아님)
    note: "책상 위 물체 3-자세 스캔"
    steps:
      - { pose: scan_top, skip: false }
      - { pose: scan_left, skip: false }
      - { pose: scan_right, skip: false }
    created_at: "..."
    updated_at: "..."

  table_scan_quick:
    kind: scan
    note: "급할 때 1-자세"
    steps:
      - { pose: scan_top, skip: false }
```

`kind` 는 task 가 "본 plan 자리 본 task 용" 필터링 자리 — frontend 의 plan picker 가 task 별로 솎아내는 자리에서 활용. backend 자리 강제 아님 (plan steps 가 유효한 pose name 이기만 하면 어느 task 든 실행 가능).

### 4.3 Python 모델 (Pydantic)

```python
# backend/core/robot/pose_library_models.py
class JointAngle(StrictModel):
    id: int
    degree: float

class PoseDefinition(StrictModel):
    joints: list[JointAngle]
    protected: bool = False
    note: str | None = None
    created_at: str  # ISO 8601
    updated_at: str

class PoseLibrary(StrictModel):
    poses: dict[str, PoseDefinition]

class PlanStep(StrictModel):
    pose: str           # pose name (poses.yaml 에 존재해야 함, validate 시 체크)
    skip: bool = False

class PlanDefinition(StrictModel):
    kind: str | None = None   # "scan" 등 — task filter hint
    note: str | None = None
    steps: list[PlanStep]
    created_at: str
    updated_at: str

class PlanLibrary(StrictModel):
    plans: dict[str, PlanDefinition]
```

### 4.4 backend 모듈 위치

```
backend/core/robot/
  pose_library.py       # 기존 robot_poses.py rename + extend.
                        #   load_pose / list_pose_names / save_pose /
                        #   delete_pose / rename_pose / reorder_within_plan ...
  plan_library.py       # 신규
  pose_library_models.py
```

기존 `robot_poses.py` 의 `load_pose` / `list_pose_names` API 는 **유지** — recipe 가 호출. 새 mutate API 만 추가.

기존 module-level `_cache` 는 단순 invalidate 로 (mutate service 가 호출 후 `_cache = None`). atomic write (temp + rename) 라 mid-read race 없음.

## 5. Ghost preview primitive

### 5.1 컴포넌트 — `RobotPreviewLayer`

기존 [RobotModel.tsx](../frontend/src/components/scene/RobotModel.tsx) 컴포넌트 그대로 마운트, prop 다르게:

```tsx
// frontend/src/components/scene/RobotPreviewLayer.tsx
import { usePreviewStore } from "@/domain/stores/preview";
import { RobotModel } from "./RobotModel";

const GHOST_OPACITY = 0.35;
const GHOST_TINT = new THREE.Color("#ff8c1a");   // MoveIt orange

export function RobotPreviewLayer({ robots, focusId }: Props) {
  const ghosts = usePreviewStore((s) => s.ghosts);

  return (
    <>
      {robots.map((r) => {
        const ghost = ghosts[r.id];
        if (!ghost) return null;
        return (
          <RobotModel
            key={`ghost-${r.id}`}
            robotType={r.type}
            robotId={r.id}
            basePose={r.base_pose}
            opacity={GHOST_OPACITY}
            tint={GHOST_TINT}        // ⭐ NEW prop — material color overlay
            jointAngles={ghost.joints}
            visible={true}
          />
        );
      })}
    </>
  );
}
```

`RobotModel` 의 `tint` prop 신규 — `loadMeshCb` / `applyOpacity` 안에서 material `.color.copy(tint)` 적용. material 은 이미 clone 되어 있어 (기존 dim 처리 자리) instance-별 색 안전.

[RobotLayer.tsx](../frontend/src/components/scene/RobotLayer.tsx) 옆에 [Scene.tsx](../frontend/src/components/scene/Scene.tsx) 안에서 `<RobotPreviewLayer robots={robots} focusId={focusId} />` 추가. RobotsLayout / WorldPage / TasksPage 모든 R3F 자리에 같은 자리 마운트.

### 5.2 Store — `previewStore`

```ts
// frontend/src/domain/stores/preview.ts
import { create } from "zustand";

interface GhostState {
  joints: number[];   // URDF rad — RobotModel.jointAngles 와 같은 단위
  source?: string;    // debug — "pose:home" / "calib_recommend:3" / "moveJ_form" 등
}

interface PreviewStore {
  ghosts: Record<string, GhostState>;        // robotId → ghost
  setGhost: (robotId: string, ghost: GhostState | null) => void;
  clearAll: () => void;
}

export const usePreviewStore = create<PreviewStore>((set) => ({
  ghosts: {},
  setGhost: (robotId, ghost) =>
    set((s) => {
      const next = { ...s.ghosts };
      if (ghost === null) delete next[robotId];
      else next[robotId] = ghost;
      return { ghosts: next };
    }),
  clearAll: () => set({ ghosts: {} }),
}));
```

### 5.3 Trigger 패턴 — caller 가 hover / select 자리에서 직접

각 caller 가 자기 hover 핸들러 안에서 `setGhost` 호출. central trigger registry 없음 — 단순함이 SSOT (어디서 호출되는지 grep 으로 즉시).

```tsx
// 예: pose list 의 row hover
<PoseRow
  onMouseEnter={() =>
    setGhost(focusId, { joints: poseToRad(pose.joints), source: `pose:${pose.name}` })
  }
  onMouseLeave={() => setGhost(focusId, null)}
  ... />
```

caller examples (구현 단계에서 추가될 자리):
- Poses 패널 (§7.1) — row hover
- 캘 [HandEyePanel](../frontend/src/components/panels/calibration/HandEyePanel.tsx) 의 추천 자세 목록 row hover ([NextPoseRecommendation.joints](../backend/modules/calibration/next_pose_planner.py) 가 이미 frontend 도착)
- [MoveJ.tsx](../frontend/src/components/panels/motion/MoveJ.tsx) form — input 값 변경 시 debounce 200ms 후 ghost update
- Plan 패널 (§7.2) — step row hover = 그 step 의 pose ghost. plan 자체 hover (or active) = 시퀀스 첫 step ghost

### 5.4 Cartesian → joint IK preview (후속 고려)

MoveL / MoveTCP 자리 caller 자리 frontend 가 IK 못 풀음 (PyBullet backend) → backend service `MOTION_IK(target_pose) → joints` 호출 필요. **첫 사이클 미구현** — Cartesian target preview 는 caller 자리 (MoveL form) 추가 시 별도 follow-up. joint-space caller (poses / 캘 추천 / MoveJ) 부터.

### 5.5 색 + opacity 디테일

| 자리 | 값 | 근거 |
|---|---|---|
| **opacity** | 0.35 | MoveIt RViz 기본값 ~0.4. 0.5 는 너무 진해 live robot 과 헷갈림 |
| **tint** | `#ff8c1a` (MoveIt orange) | §2.2 결론 5 |
| **dim others (focus mode)** | 0.25 (기존 [RobotLayer](../frontend/src/components/scene/RobotLayer.tsx#L38) 그대로) | 변경 없음 — ghost 와 dim 의 가시성 충돌 시 ghost 자리 winning |

## 6. 백엔드 service / topic 계약

### 6.1 새 service

[backend/core/transport/topic_map.py](../backend/core/transport/topic_map.py) `Service` 클래스 추가:

```python
# ─── Pose Library (per-robot) ─────────────────────────
POSE_LIST = "horibot/{robot_id}/pose/srv/list"           # → {poses: PoseDefinition[]}
POSE_GET = "horibot/{robot_id}/pose/srv/get"             # name → PoseDefinition
POSE_SAVE = "horibot/{robot_id}/pose/srv/save"           # {name, joints, note?} → 신규/덮어쓰기. protected name 의 joints 수정 OK, name 자체 변경은 별도 service.
POSE_DELETE = "horibot/{robot_id}/pose/srv/delete"       # name. protected 자리 거부.
POSE_RENAME = "horibot/{robot_id}/pose/srv/rename"       # {old, new}. protected 자리 거부.

# ─── Plan Library (per-robot) ─────────────────────────
PLAN_LIST = "horibot/{robot_id}/plan/srv/list"           # kind 필터 옵션 → PlanDefinition[]
PLAN_GET = "horibot/{robot_id}/plan/srv/get"             # name → PlanDefinition (with steps)
PLAN_SAVE = "horibot/{robot_id}/plan/srv/save"           # {name, kind?, note?, steps[]} → 신규/덮어쓰기
PLAN_DELETE = "horibot/{robot_id}/plan/srv/delete"
PLAN_RENAME = "horibot/{robot_id}/plan/srv/rename"
```

namespace `pose` / `plan` 은 새 도메인. [topic_map.py](../backend/core/transport/topic_map.py) 의 기존 도메인 (motor/camera/motion/scene3d/...) 옆 자리.

robot-scoped (template `{robot_id}`) — pose 는 robot 별로 별도 yaml. multi-robot 자리 자연.

### 6.2 새 topic — invalidation

```python
POSE_LIBRARY_INVALIDATED = "horibot/{robot_id}/pose/state/invalidated"  # save/delete/rename 후 1회
PLAN_LIBRARY_INVALIDATED = "horibot/{robot_id}/plan/state/invalidated"
```

frontend store 가 구독해 refetch. backend 안 caller (recipes 의 `load_pose`) 도 module-level `_cache = None` 자리 listener 박을 수 있지만 — 첫 사이클 자리 backend 안 cache invalidation 은 service handler 자리 본인이 직접 `_cache = None` 호출 (in-process 단일 source). 분산 자리 후속 (PC 가 yaml 편집했는데 다른 PC 도 PoseLibrary 자리 쓰는 시나리오 자리 — 현재 없음).

### 6.3 [api_contract.py](../backend/api_contract.py) 등재

```python
PUBLIC_SERVICES[Service.POSE_LIST] = (PoseListReq, PoseListRes)
PUBLIC_SERVICES[Service.POSE_GET] = (PoseGetReq, PoseGetRes)
...
PUBLIC_TOPICS[Topic.POSE_LIBRARY_INVALIDATED] = PoseLibraryInvalidated
PUBLIC_TOPICS[Topic.PLAN_LIBRARY_INVALIDATED] = PlanLibraryInvalidated
```

→ `pnpm gen:types` → frontend `contract.ts` 자동 emit. 손작업 동기화 X ([feedback_ssot_first.md], [feedback_developer_focus_business_logic.md] 자리 정합).

### 6.4 어느 노드가 host 하나

**결정**: 새 ApplicationNode 만들지 않고 — **TaskNode 가 host**.

근거:
- TaskNode 가 이미 `load_pose` 호출 자리 (recipe 안에서 — `home()` / `search_and_detect()`). pose library 의 1차 consumer.
- 새 node 추가는 host config 변경 + 부팅 sequence 변경. 기존 TaskNode 가 같은 PC 자리.
- pose 는 호스트 1군데 (PC) 의 파일 — TaskNode 가 위치한 PC 가 그 파일 owner.

대안: 신규 `PoseLibraryNode`. reject — 작은 entity 자리 별도 node 과한 분리.

대안 2: StorageNode 가 host. reject — storage_layer.md 의 §3 자리 "캘 특유 패턴 / append-only blob 패턴 이 아닌 entity 는 강제 X" 정신. pose 는 단순 yaml CRUD, StorageNode 의 RdbStore/ObjectStore Protocol 자리 안 거침.

### 6.5 ScanTask 의 새 caller 자리

[scan.py](../backend/modules/task/tasks/scan.py) 의 `create_scan_task(scan_poses)` 자리는 *그대로* — 본 task 는 pose name list 받음. 새 자리 frontend 가 plan picker → pose name 시퀀스 추출 → `TASK_RUN` payload 자리 `scan_poses` 채움. plan 자체를 task argument 로 받게 변경할 수 있지만 (첫 사이클 미적용) — plan 의 skip 필터링은 frontend 가 미리 (`steps.filter(s => !s.skip).map(s => s.pose)`).

대안: ScanTask 가 plan_name 받음 + backend 에서 expand. 후속 — task argument 의 typed schema (TaskInputSchema) 가 plan name 을 enum 으로 advertize 하는 그림이 더 깔끔하지만 첫 사이클은 frontend expand 가 단순.

## 7. UI 영역 + 유저 플로우

### 7.1 새 패널: **Poses Panel** (Move mode 안)

[RobotMoveMode.tsx](../frontend/src/pages/robotModes/RobotMoveMode.tsx) 의 PANEL_SPEC 에 추가:

```ts
{ id: "poses", component: "poses", title: "Poses", width: 280, height: 360 }
```

패널 구성:

```
┌─ Poses ──────────────────────────────────┐
│ [+ Save current pose]                    │  ← 클릭 → MOTION_GET_TCP 로 현 joint 캡처 → 이름 dialog
├──────────────────────────────────────────┤
│ 🔒 home       Task 시작 자세             │  ← protected = 자물쇠 아이콘
│    rest       Park 자세                  │
│ 🔒 search_1   좌측 시야 +30°             │
│ 🔒 search_2   가운데                     │
│ 🔒 search_3   우측 -30°                  │
│    scan_top   책상 위                    │
│    scan_left  좌측 시야                  │
│                                          │
│  ↑ hover → ghost on scene                │
│  ↑ click → "Move here" / "Rename" /     │
│            "Delete" menu                 │
└──────────────────────────────────────────┘
```

세부:
- **+ Save current pose** — modal: name input + note (선택). 같은 이름 있으면 "덮어쓰기?" confirm
- **hover row** — `previewStore.setGhost(robotId, joints)` ghost 표시
- **click row** — action menu (Move here / Rename / Delete). Move here = `MOTION_MOVE_J` (joints) 호출. Rename / Delete = protected 자리 disabled
- **note 편집** — inline edit on row (double-click)
- **drag reorder** — **library 는 reorder 안 함** (§2.2 결론 1). library 는 lexical sort 또는 created_at 정렬.

### 7.2 새 패널: **Scan Plan Panel** (TasksPage 의 `/tasks/scan` 자리)

[TasksPage.tsx](../frontend/src/pages/TasksPage.tsx) 의 PANELS 에 task name 별 conditional 추가. `name === "scan"` 일 때 plan 패널 마운트. 다른 task 자리 자기 자리 plan (없으면 미마운트).

```
┌─ Scan Plan ──────────────────────────────┐
│ Plan: [▼ table_scan_default          ]   │  ← dropdown — kind=scan 필터링
│       [+ New plan]                       │
├──────────────────────────────────────────┤
│ ⋮⋮ [✓] scan_top      ← drag handle      │  ← drag-and-drop reorder
│ ⋮⋮ [✓] scan_left                         │  ← checkbox = skip toggle (체크 = 실행)
│ ⋮⋮ [ ] scan_right    (skipped)           │
│ ⋮⋮ [✓] home          (return)            │
│                                          │
│ [+ Add step]  ← pose picker dropdown    │
│                                          │
│ ┌──────────────────────────────────────┐ │
│ │  [▶ Run scan]   ← TASK_RUN(scan,    │ │
│ │                    scan_poses=[...]) │ │
│ │  [💾 Save plan changes]              │ │
│ └──────────────────────────────────────┘ │
└──────────────────────────────────────────┘
```

세부:
- **drag handle (⋮⋮)** — dnd-kit 사용. 라이브러리 추가 1개 (`@dnd-kit/core` + `@dnd-kit/sortable`)
- **skip checkbox** — 체크 = 실행, 빈 = skip (intuitive: "check the things you want")
- **hover step row** — `previewStore.setGhost(robotId, joints)` ghost. plan 자체 hover 자리 첫 active step 의 ghost
- **+ Add step** — pose dropdown (poses.yaml 의 모든 자세). 선택 시 step 추가 (default skip=false)
- **delete step** — row hover 시 ✕ 버튼
- **Save plan changes** — dirty 자리 highlight. 안 누르면 cancel
- **Run scan** — `TASK_RUN(name=scan, args={scan_poses: planSteps.filter(!skip).map(.pose)})`

### 7.3 캘 추천 자세 — 기존 패널의 hover 만 추가

[HandEyePanel.tsx](../frontend/src/components/panels/calibration/HandEyePanel.tsx) 의 추천 자세 목록 row 자리 — 변경 최소:
- hover 시 `setGhost(robotId, recommendation.joints → rad)` 추가 한 줄
- 추천 자세 자리 즉시 ghost 로 확인 가능 — "이 추천 자리 갈 만한지" 사용자가 책상 충돌 시각적 검증

### 7.4 MoveJ form preview (선택)

[MoveJ.tsx](../frontend/src/components/panels/motion/MoveJ.tsx) 의 joint input 6개 자리 — 값 변경 시 debounce 200ms 후 `setGhost(robotId, currentInputJoints)`. form 떠나면 clear.

(첫 사이클 자리 우선순위 낮음 — Poses 패널 + Scan Plan 먼저, MoveJ preview 는 nice-to-have.)

### 7.5 유저 플로우 walkthrough

#### Flow A — 새 스캔 자세 만들기

1. `/robots/so101_6dof_0/move` 진입
2. Motion 패널의 Jog 으로 책상 위 좋은 시야 자세 잡음
3. Poses 패널 `+ Save current pose` 클릭
4. modal: name `scan_table_north` + note `책상 북쪽` + Save
5. backend `POSE_SAVE` 호출 → yaml atomic write → `POSE_LIBRARY_INVALIDATED` publish
6. Poses 패널 목록에 추가됨. row hover 시 ghost preview 정상

#### Flow B — Scan plan 만들고 실행

1. `/tasks/scan` 진입
2. Scan Plan 패널: `+ New plan` → name `quick_2pose`
3. `+ Add step` → dropdown 에서 `scan_table_north` 선택 → 추가
4. `+ Add step` → `scan_table_south` 선택 → 추가
5. `+ Add step` → `home` 선택 (마지막 복귀 자리)
6. drag 으로 순서 확정. step 1 hover → ghost 가 scan_table_north 자세 보임 — 책상에 부딪힐 거 같으면 step 의 ✕ 로 삭제 또는 [✓] 해제 (skip)
7. `Save plan changes` → backend `PLAN_SAVE` → yaml write
8. `Run scan` → `TASK_RUN(scan, scan_poses=[scan_table_north, scan_table_south, home])` (skip 된 자리 자동 필터)
9. TaskNode 가 ScanTask 만들고 ForEach 순회 — 기존 흐름 그대로

#### Flow C — 캘 추천 자세 검증

1. `/robots/so101_6dof_0/calibrate` 진입, Hand-Eye 패널에서 캡처 시작
2. 5장 캡처 후 BA 자동 → σ + 추천 자세 6개 표시
3. 추천 row 1 hover → ghost 가 그 자세로 표시. **로봇 본체가 그 자세 가면 보드를 볼 수 있는지** 시각적 확인
4. ghost 가 책상에 부딪힐 거 같으면 추천 row 의 [skip] 표시 (현재 캘 UI 의 `recommendation_fail` 자리)
5. 좋은 자세 row 클릭 → `MOTION_MOVE_J` (현재 동작 자리). live robot 가 그 자세로 이동
6. 책상 / 보드 가시성 다시 확인 → [캡처]

(현재 캘 UI 의 추천 자세 row → ghost 자리 추가가 본 사이클의 *가장 즉시적 가치*. plan / pose CRUD 보다 우선 검토할 자리.)

#### Flow D — 다른 PC 에서 같은 plan 공유

1. PC1 가 plan 만들고 `Save` → `robot/instances/so101_6dof_0/plans.yaml` 변경
2. `git commit + push`
3. PC2 가 `git pull` → 부팅 시 plan_library 자리 새 yaml 자동 로드 (in-memory cache 자리 backend 재시작 자리 — 첫 사이클 hot reload 미적용. 또는 frontend 가 `PLAN_LIST` 호출 시점에 backend 가 mtime check + reload)

→ git 이 sync mechanism. multi-PC 자리 inhouse web 시나리오 ([project_horibot_is_inhouse_web]) 자연 정합.

## 8. 추후 확장 자리 (구현 X, plan only)

### 8.1 trajectory ghost — animated playback

plan 실행 중 next step ghost 미리, current step joint 실시간 (live), 다음 step ghost 등 trajectory ahead. MoveIt 의 "Show Trail" 자리. previewStore 의 `ghosts: dict[robotId, GhostState | GhostState[]]` 확장 자리.

### 8.2 Cartesian pose entity

`poses.yaml` 자리 joint 외 Cartesian (TCP base-frame pose) 자리. MoveTCP / Detection result preview 자리 필요. backend `MOTION_IK` service 가 frontend 가 호출하는 자리 자리. RoboDK 의 joint-mode vs Cartesian-mode 자리 정합.

### 8.3 Plan 의 task argument 통합

`TASK_RUN(scan, plan_name=...)` 자리 — backend 가 plan 자체 해석 (skip 자리 backend). frontend 가 plan name 만 보냄. typed task input schema (`TaskInputSchema`) 자리 plan name 을 enum 으로 advertise.

### 8.4 Plan kind 별 task binding

`plans.yaml` 의 `kind: scan` 자리가 frontend 의 plan picker 자리 필터 (해당 task 만). 후속 자리 `kind: motion` (named MoveJ sequence), `kind: calibration` (capture sequence) 등 확장.

### 8.5 다른 robot 의 pose 복사

so101 의 `home` 자리 omx 자리 복사 (motor 수 / limit 다르면 거부). cross-robot pose share 자리 — 첫 사이클 미적용.

### 8.6 변형 자세 자동 생성

캘 추천 자세 strategy ([next_pose_planner.py](../backend/modules/calibration/next_pose_planner.py)) 의 joint_perturbation 자리 같은 식으로 — "scan_top 자리 ±30° variants" 자동 생성. Plan 에 추가 후보 자리.

## 9. 실패 모드 + safety

### 9.1 pose name → recipe break

- protected flag 가 hard guard (§3.4). UI 에서 자물쇠 표시.
- pose 삭제 시 plan 의 step 자리 dangling reference — `PLAN_LIST` 의 step 자리 validate (pose 미존재 시 frontend 자리 step row 가 빨강 표시 + "이 자세 없음" 메시지). 자동 삭제 X (사용자가 명시적으로).

### 9.2 yaml 손상 (syntax error)

- backend 부팅 시 fail-fast (현재 [robot_poses.py](../backend/core/robot/robot_poses.py) 자리 자체) — yaml.safe_load 실패 시 ValueError. multi-robot 자리 *해당 robot 만* fail (다른 robot 영향 X) — RobotRegistry 의 per-robot 격리 자리 정합.

### 9.3 atomic write 실패 (디스크 가득 / 권한)

- temp file + rename 패턴 자리 — temp 실패 시 원본 보존. rename 자리 atomic (POSIX). Windows 자리 `os.replace` (atomic on same filesystem).
- 실패 시 service response `{success: false, message: ...}` — frontend toast 표시.

### 9.4 다중 frontend 동시 편집

- 첫 사이클 자리 last write wins (no optimistic locking). 인하우스 단일 사용자 시나리오 자리 충분.
- 후속 자리 yaml 의 `updated_at` 자리 ETag 등 추가 가능.

### 9.5 plan 실행 중 plan 편집

- 첫 사이클 자리 frontend 의 plan 패널 자리 "TASK RUNNING 중 disable" UI 자리. backend race X (TASK_RUN payload 자리 frontend 가 expand 된 `scan_poses` list 보냄 — 실행 중 plan yaml 변경되어도 영향 X).

## 10. 마이그레이션 phase

### Phase A — yaml schema migrate + 부팅 unblock (구현 0.5일)

1. [robot_poses.py](../backend/core/robot/robot_poses.py) 의 yaml schema 자리 §4.1 의 nested (`poses:` top key + `joints` / `protected` / `note` / `created_at` / `updated_at` per pose) 로 변경
2. 기존 [omx_f_0/robot_poses.yaml](../robot/instances/omx_f_0/robot_poses.yaml) 자리 마이그레이션 (`home: [...]` → `poses: { home: { joints: [...], protected: true, ... } }`)
3. 신규 [so101_6dof_0/poses.yaml](../robot/instances/so101_6dof_0/) 생성 — `home` / `rest` / `search_1~3` 자리 SO-101 6DOF (M1~M6, gripper M7 제외) 값으로
4. 파일명 rename (`robot_poses.yaml` → `poses.yaml`) — §3.7 의 사용자 결정 자리 option (i) 채택 가정
5. [RobotConfig.robot_poses_yaml](../backend/core/robot/robot_registry.py#L103) 자리 path field rename → `RobotConfig.poses_yaml`
6. backend 부팅 → so101 default 자리 `pick_and_place` preview 자리 IO error 해결

**deliverable**: 기존 recipe (`home()`, `search_and_detect()`) 동작 그대로 유지, multi-robot 자리 양쪽 동작.

### Phase B — pose library mutation service + Poses 패널 (구현 1.5~2일)

1. backend `POSE_SAVE` / `POSE_DELETE` / `POSE_RENAME` / `POSE_LIST` / `POSE_GET` service 자리 TaskNode 호스트 자리
2. `POSE_LIBRARY_INVALIDATED` topic
3. atomic write helper (temp file + os.replace)
4. [api_contract.py](../backend/api_contract.py) 등재 + `pnpm gen:types`
5. frontend `poseStore` (Zustand) — `LIST` 결과 자리 캐시, INVALIDATED 자리 refetch
6. `Poses` 패널 (§7.1) — list / save current / rename / delete / hover preview (ghost X 자리 아직 — Phase C)
7. **단** ghost preview primitive 미적용 자리 — hover 자리 disabled

**deliverable**: 사용자가 jog → 자세 저장 → list 확인 자리. ghost X.

### Phase C — Ghost preview primitive + 캘 추천 hover 적용 (구현 1~1.5일)

1. `RobotModel` 의 `tint` prop 추가 + material color overlay 적용
2. `RobotPreviewLayer` 컴포넌트
3. `previewStore` (Zustand)
4. [Scene.tsx](../frontend/src/components/scene/Scene.tsx) 자리 `<RobotPreviewLayer />` 마운트 (RobotsLayout / WorldPage / TasksPage 모두)
5. Poses 패널의 hover 자리 `setGhost` 호출 자리 (Phase B 의 disabled hover 활성화)
6. [HandEyePanel](../frontend/src/components/panels/calibration/HandEyePanel.tsx) 의 추천 row hover 자리 `setGhost` 추가 (한 줄)
7. (선택) [MoveJ.tsx](../frontend/src/components/panels/motion/MoveJ.tsx) form preview

**deliverable**: ghost 동작. Phase A 완료 + Phase B + Phase C 누적 시 사용자 데모 자리 *jog → 저장 → 캘 추천 hover preview* 자리 가능.

### Phase D — Plan entity + Scan Plan 패널 (구현 2~3일)

1. backend `plan_library.py` 모듈 + `PLAN_*` service 자리
2. `plans.yaml` schema (§4.2)
3. frontend `planStore` + Plan 패널 (§7.2) — dnd-kit 라이브러리 추가
4. TasksPage 의 `scan` task 자리 plan picker 통합 — `TASK_RUN(scan, scan_poses=...)` 자리 expand

**deliverable**: scan plan curation 자리 full UX.

### Phase E (후속) — §8 자리 자유 선택

trajectory ghost / Cartesian pose / task argument 통합 등.

### 총 추정 — 5~7일 (단일 개발자, 1주 sprint 자리)

A 자리 small (부팅 unblock 의 cheap 자리). B+C 자리 자세 라이브러리 + ghost — 본 design 의 핵심 가치. D 자리 plan — 사용자 요구 (3)(4) 직접 자리.

## 11. 결정 sourcing trace

본 문서 자리 정답이 *유일하지 않은* 자리. 본 절은 design 결정의 갈래 자리 명시 자리 — 합의 자리 사용자가 짚기 좋게.

| 결정 | 자리 | 대안 | 본 문서 선택 근거 |
|---|---|---|---|
| **yaml vs DB** | §3.1 | StorageNode RDB | 산업 표준 (MoveIt Pro / Isaac), 단일 사용자 시나리오, blob 없음, git audit |
| **단일 layer vs system/user 이원화** | §3.4 | MoveIt SRDF + warehouse 식 분리 | trend 가 collapse 자리 (MoveIt Pro), 마찰 적음, protected flag 로 guard 충분 |
| **pose / plan entity 분리** | §3.2 | 단일 파일 두 섹션 | 산업 표준 (Polyscope / RoboDK), 라이프사이클 다름 |
| **joint-space only (1차)** | §3.3 | Cartesian + joint | reference frame 시스템 없음, calibration invariant, over-engineering 회피 |
| **Foxglove multi-URDF 패턴** | §3.5 | 단순 marker 시각화 | 우리 RobotModel 이 이미 동형, "ghost 자리 실 로봇 모양" 직관 |
| **단일 ghost per robot** | §3.6 | trajectory N-ghost | 단순함, 후속 확장 가능 |
| **TaskNode 가 service host** | §6.4 | 신규 PoseLibraryNode / StorageNode | TaskNode 가 1차 consumer, 작은 entity 별도 노드 과함 |
| **frontend 가 plan expand → TASK_RUN** | §6.5 | backend 가 plan_name 받음 + expand | 첫 사이클 단순, 후속 typed input schema 자리 자연 확장 |
| **file rename** (`robot_poses.yaml` → `poses.yaml`) | §3.7 | 기존 이름 유지 | MoveIt Pro 컨벤션 일치 — *사용자 결정 자리* |

## 12. 합의

본문 §3~§10 의 결정 자리 본 문서 1안 으로 박힘 (§11 의 sourcing trace 가 근거). **다 OK 면 별도 답 없이 "ok 진행"** — 다음 세션에 Phase A 부터 구현.

**사용자에게 묻는 단일 갈래** (§3.7 의 재확인):

- 파일명 `robot_poses.yaml` → `poses.yaml` rename — OK / 유지

읽다가 다른 자리에서 반대 / 수정 의견 자리 section 번호 + 한 줄 짚어주면 본 문서 update 후 재합의.
