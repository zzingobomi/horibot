/**
 * dockview panel 레지스트리 — key → 등록 패널.
 *
 * 패널만 import 하는 순수 map. 각 패널이 useParams(robot context) + scroll
 * container 를 자체 흡수하므로 여기엔 로직·wrapper 없음. 새 패널 추가 =
 * 컴포넌트 만들고 여기 한 줄 + mode 파일 PANELS 한 줄 (frontend.md §2.3).
 */
import type { FunctionComponent } from "react";
import type { IDockviewPanelProps } from "dockview";
import { RobotStatePanel } from "./RobotStatePanel";
import { WaypointScenePart } from "./WaypointPanel/scenePart";
import { CalibrationScenePart } from "./CalibrationPanel/scenePart";
import { MotionPanel } from "./MotionPanel";
import { CalibrationPanel } from "./CalibrationPanel";
import { CameraPanel } from "./CameraPanel";
import { DetectionCameraPanel } from "./DetectionCameraPanel";
import { CalibrationCameraPanel } from "./CalibrationCameraPanel";
import { ScanPanel } from "./ScanPanel";
import { LivePointCloudPanel } from "./LivePointCloudPanel";
import { WaypointPanel } from "./WaypointPanel";
import { PickAndPlacePanel } from "./PickAndPlacePanel";
import { TaskProgressPanel } from "./TaskProgressPanel";
import { withRobotOwnership } from "@/components/shared/robotOwnership";

// 모든 패널은 여기 등록 (§4.1 ② — dockview 가 string key→component 로 인스턴스화).
// pickAndPlace/taskProgress 는 task 페이지(PickAndPlacePage) 의 PANELS 에서 배치.
export const PANEL_COMPONENTS = {
  robotState: RobotStatePanel,
  motion: MotionPanel,
  calibration: CalibrationPanel,
  camera: CameraPanel,
  detectionCamera: DetectionCameraPanel,
  calibrationCamera: CalibrationCameraPanel,
  scan: ScanPanel,
  livePointCloud: LivePointCloudPanel,
  waypoints: WaypointPanel,
  pickAndPlace: PickAndPlacePanel,
  taskProgress: TaskProgressPanel,
} as const;

export type PanelComponentKey = keyof typeof PANEL_COMPONENTS;

/**
 * 패널 카탈로그 — auto-hide 헤더 `+ 패널 추가` 의 모집단 (전 종류).
 * mode 파일의 PANELS 는 그 페이지의 **default 세트**일 뿐이고, 사용자는 어떤
 * 페이지에서든 여기 등록된 모든 패널을 추가할 수 있다 (추가분은 dockview layout
 * localStorage 영속으로 다음 방문에도 유지). title/size 는 추가 시 초기값.
 *
 * `requiredCapabilities` — 이 패널을 쓰려면 robot 이 가져야 하는 capability
 * (robots.yaml SSOT, `useRobots()` 로 노출). **UI 힌트지 권한의 원천이 아니다**
 * — 부족하면 헤더에서 disabled + withRobotOwnership 이 unsupported empty state 로
 * 안내하지만, 최종 사용 가능 여부는 백엔드가 계속 판정한다 ([[lib/capabilities]]).
 * 선언 없는 패널 = 요구 capability 없음 = 항상 활성. `unavailableReason` 은
 * 예외적 UX 자리의 override(기본은 capability 라벨에서 파생, [docs/frontend.md]).
 *
 * `scenePart` — 이 패널이 3D 씬에 **기여하는 조각** (씬 전체가 아니라 부분,
 * [docs/frontend.md]). 패널 폴더의 R3F 컴포넌트를 여기 한
 * 줄로 선언하면 ScenePartHost 가 살아있는 인스턴스마다 RobotProvider 로 감싸
 * Canvas 에 마운트 — Scene.tsx 편집 0. UI(컴포넌트) + 3D(scenePart) 가 한 항목에
 * co-locate 되는 자리.
 */
export const PANEL_CATALOG: Record<
  PanelComponentKey,
  {
    title: string;
    width: number;
    height: number;
    requiredCapabilities?: string[];
    unavailableReason?: string;
    scenePart?: FunctionComponent;
  }
> = {
  robotState: { title: "Robot State", width: 260, height: 300 },
  motion: { title: "Motion", width: 320, height: 380 },
  calibration: {
    title: "Calibration",
    width: 360,
    height: 520,
    scenePart: CalibrationScenePart, // preview PnP 보드 pose — 인식/캘 검증 시각화
  },
  // 카메라 3종 — 구분 단어를 **앞에** (좁은 탭에서 뒤가 잘려도 구분 유지).
  camera: { title: "Camera", width: 420, height: 340 },
  detectionCamera: {
    title: "Detect Camera",
    width: 440,
    height: 330,
    requiredCapabilities: ["rgbd"], // 검출 = depth 투영 필요 (detector/module.py)
  },
  calibrationCamera: { title: "Calib Camera", width: 420, height: 340 },
  scan: { title: "Scan", width: 320, height: 460, requiredCapabilities: ["rgbd"] },
  // 카메라 frustum 은 scenePart 가 아니라 Camera 씬 객체(scene/Cameras.tsx)가 그림
  // — 카메라는 월드 소유, 패널은 cameraStore.showFrustum 토글만 (소유권 모델).
  livePointCloud: {
    title: "Live PointCloud",
    width: 300,
    height: 320,
    requiredCapabilities: ["rgbd"],
  },
  waypoints: {
    title: "Waypoints",
    width: 340,
    height: 560,
    scenePart: WaypointScenePart, // ghost 미리보기 — [보기] 토글 시 반투명 자세
  },
  pickAndPlace: { title: "Pick & Place", width: 340, height: 340 },
  taskProgress: { title: "Task Progress", width: 340, height: 420 },
};

/**
 * robot 을 소유하는(useRobotId 계열) 패널 key 집합 ([[robot_ownership_model]]).
 * 여기 든 패널만 robot 셀렉터 탭 + robot params + Select Robot 빈 상태를 갖는다.
 * task 패널(pickAndPlace/taskProgress)은 robot 을 *고르지* 않고 task 바인딩 계약
 * (list_robots 서비스, useTaskRobots)에서 얻으므로 제외. detectionCamera 는
 * 카메라가 물리적으로 robot 하나에 묶이는 robot-scoped 패널이라 포함.
 */
export const ROBOT_OWNED_PANELS: ReadonlySet<PanelComponentKey> = new Set<PanelComponentKey>([
  "robotState",
  "motion",
  "calibration",
  "camera",
  "detectionCamera",
  "calibrationCamera",
  "scan",
  "livePointCloud",
  "waypoints",
]);

/**
 * dockview 에 넘길 컴포넌트 맵 — robot-owned 패널은 withRobotOwnership 로 감싼
 * 버전. module-scope 상수라 identity 안정(dockview 재마운트 방지). raw
 * PANEL_COMPONENTS 는 테스트/직접 렌더용으로 그대로 export 유지.
 *
 * requiredCapabilities/unavailableReason 은 **wrap 시점 클로저**로 HOC 에 넘긴다
 * (dockview params 에 넣지 않음 — static registry 사실이 localStorage layout 에
 * 영속되면 stale/serialization 오염. [docs/frontend.md]).
 */
export const DOCKVIEW_PANEL_COMPONENTS: Record<
  string,
  FunctionComponent<IDockviewPanelProps>
> = Object.fromEntries(
  (
    Object.entries(PANEL_COMPONENTS) as [PanelComponentKey, FunctionComponent][]
  ).map(([key, Comp]) => {
    if (!ROBOT_OWNED_PANELS.has(key)) return [key, Comp];
    const meta = PANEL_CATALOG[key];
    return [
      key,
      withRobotOwnership(Comp, {
        requiredCapabilities: meta.requiredCapabilities,
        unavailableReason: meta.unavailableReason,
        panelKind: key, // 인스턴스 등록용 — ScenePartHost 가 scene lookup
      }),
    ];
  }),
);
