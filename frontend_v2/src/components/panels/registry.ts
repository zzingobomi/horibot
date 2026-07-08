/**
 * dockview panel 레지스트리 — key → 등록 패널.
 *
 * 패널만 import 하는 순수 map. 각 패널이 useParams(robot context) + scroll
 * container 를 자체 흡수하므로 여기엔 로직·wrapper 없음. 새 패널 추가 =
 * 컴포넌트 만들고 여기 한 줄 + mode 파일 PANELS 한 줄 (frontend_v2.md §2.3).
 */
import type { FunctionComponent } from "react";
import type { IDockviewPanelProps } from "dockview";
import { RobotStatePanel } from "./RobotStatePanel";
import { MotionPanel } from "./MotionPanel";
import { CalibrationPanel } from "./CalibrationPanel";
import { CameraPanel } from "./CameraPanel";
import { DetectionCameraPanel } from "./DetectionCameraPanel";
import { CalibrationCameraPanel } from "./CalibrationCameraPanel";
import { ScanPanel } from "./ScanPanel";
import { LivePointCloudPanel } from "./LivePointCloudPanel";
import { WaypointPanel } from "./WaypointPanel";
import { PromptPanel } from "./PromptPanel";
import { TaskProgressPanel } from "./TaskProgressPanel";
import { withRobotOwnership } from "@/components/shared/robotOwnership";

// 모든 패널은 여기 등록 (§4.1 ② — dockview 가 string key→component 로 인스턴스화).
// prompt/taskProgress 는 최상위 TasksPage(dockview) 의 PANELS 에서 이 key 로 배치.
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
  prompt: PromptPanel,
  taskProgress: TaskProgressPanel,
} as const;

export type PanelComponentKey = keyof typeof PANEL_COMPONENTS;

/**
 * robot 을 소유하는(useRobotId 계열) 패널 key 집합 ([[robot_ownership_model]]).
 * 여기 든 패널만 robot 셀렉터 탭 + robot params + Select Robot 빈 상태를 갖는다.
 * task 바인딩(useTaskRobotId) 패널(detectionCamera/prompt/taskProgress)은 carve-out
 * 이라 제외 — robot 은 task 가 정한다(§7).
 */
export const ROBOT_OWNED_PANELS: ReadonlySet<PanelComponentKey> = new Set<PanelComponentKey>([
  "robotState",
  "motion",
  "calibration",
  "camera",
  "calibrationCamera",
  "scan",
  "livePointCloud",
  "waypoints",
]);

/**
 * dockview 에 넘길 컴포넌트 맵 — robot-owned 패널은 withRobotOwnership 로 감싼
 * 버전. module-scope 상수라 identity 안정(dockview 재마운트 방지). raw
 * PANEL_COMPONENTS 는 테스트/직접 렌더용으로 그대로 export 유지.
 */
export const DOCKVIEW_PANEL_COMPONENTS: Record<
  string,
  FunctionComponent<IDockviewPanelProps>
> = Object.fromEntries(
  (
    Object.entries(PANEL_COMPONENTS) as [PanelComponentKey, FunctionComponent][]
  ).map(([key, Comp]) => [
    key,
    ROBOT_OWNED_PANELS.has(key) ? withRobotOwnership(Comp) : Comp,
  ]),
);
