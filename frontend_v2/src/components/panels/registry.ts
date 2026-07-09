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
 * 패널 카탈로그 — auto-hide 헤더 `+ 패널 추가` 의 모집단 (전 종류).
 * mode 파일의 PANELS 는 그 페이지의 **default 세트**일 뿐이고, 사용자는 어떤
 * 페이지에서든 여기 등록된 모든 패널을 추가할 수 있다 (추가분은 dockview layout
 * localStorage 영속으로 다음 방문에도 유지). title/size 는 추가 시 초기값.
 */
export const PANEL_CATALOG: Record<
  PanelComponentKey,
  { title: string; width: number; height: number }
> = {
  robotState: { title: "Robot State", width: 260, height: 300 },
  motion: { title: "Motion", width: 320, height: 380 },
  calibration: { title: "Calibration", width: 360, height: 520 },
  camera: { title: "Camera", width: 420, height: 340 },
  detectionCamera: { title: "Camera (Detection)", width: 440, height: 330 },
  calibrationCamera: { title: "Camera (Calibration)", width: 420, height: 340 },
  scan: { title: "Scan", width: 320, height: 460 },
  livePointCloud: { title: "Live PointCloud", width: 300, height: 320 },
  waypoints: { title: "Waypoints", width: 340, height: 560 },
  prompt: { title: "Command", width: 340, height: 260 },
  taskProgress: { title: "Task Progress", width: 340, height: 420 },
};

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
