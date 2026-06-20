import { RobotStatePanel } from "@/components/panels/RobotStatePanel";
import { SceneControlsPanel } from "@/components/panels/SceneControlsPanel";
import { CalibrationPanel } from "@/components/panels/calibration/CalibrationPanel";
import { CameraPanel } from "@/components/panels/calibration/CameraPanel";
import { IntrinsicPanel } from "@/components/panels/calibration/IntrinsicPanel";
import { PromptPanel } from "@/components/panels/PromptPanel";
import { TaskProgressPanel } from "@/components/panels/TaskProgressPanel";
import { CameraFeedPanel } from "@/components/panels/CameraFeedPanel";
import { MotionPanel } from "@/components/panels/motion";

export const PANEL_COMPONENTS = {
  robotState: RobotStatePanel,
  sceneControls: SceneControlsPanel,
  calibration: CalibrationPanel,
  calibrationCamera: CameraPanel,
  intrinsic: IntrinsicPanel,
  prompt: PromptPanel,
  taskProgress: TaskProgressPanel,
  cameraFeed: CameraFeedPanel,
  motion: MotionPanel,
} as const;

export type PanelComponentKey = keyof typeof PANEL_COMPONENTS;
