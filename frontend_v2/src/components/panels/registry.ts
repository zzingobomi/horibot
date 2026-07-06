/**
 * dockview panel 레지스트리 — key → 등록 패널.
 *
 * 패널만 import 하는 순수 map. 각 패널이 useParams(robot context) + scroll
 * container 를 자체 흡수하므로 여기엔 로직·wrapper 없음. 새 패널 추가 =
 * 컴포넌트 만들고 여기 한 줄 + mode 파일 PANELS 한 줄 (frontend_v2.md §2.3).
 */
import { RobotStatePanel } from "./RobotStatePanel";
import { MotionPanel } from "./MotionPanel";
import { CalibrationPanel } from "./CalibrationPanel";
import { CameraPanel } from "./CameraPanel";
import { CalibrationCameraPanel } from "./CalibrationCameraPanel";
import { ScanPanel } from "./ScanPanel";
import { LivePointCloudPanel } from "./LivePointCloudPanel";
import { WaypointPanel } from "./WaypointPanel";
import { PromptPanel } from "./PromptPanel";
import { TaskProgressPanel } from "./TaskProgressPanel";

// 모든 패널은 여기 등록 (§4.1 ② — dockview 가 string key→component 로 인스턴스화).
// prompt/taskProgress 는 최상위 TasksPage(dockview) 의 PANELS 에서 이 key 로 배치.
export const PANEL_COMPONENTS = {
  robotState: RobotStatePanel,
  motion: MotionPanel,
  calibration: CalibrationPanel,
  camera: CameraPanel,
  calibrationCamera: CalibrationCameraPanel,
  scan: ScanPanel,
  livePointCloud: LivePointCloudPanel,
  waypoints: WaypointPanel,
  prompt: PromptPanel,
  taskProgress: TaskProgressPanel,
} as const;

export type PanelComponentKey = keyof typeof PANEL_COMPONENTS;
