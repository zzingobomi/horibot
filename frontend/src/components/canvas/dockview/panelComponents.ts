import { RobotStatePanel } from "@/components/panels/RobotStatePanel";
import { SceneControlsPanel } from "@/components/panels/SceneControlsPanel";
import { CalibrationPanel } from "@/components/panels/CalibrationPanel";
import { CalibrationActionsPanel } from "@/components/panels/CalibrationActionsPanel";
import { PointCloudPanel } from "@/components/panels/PointCloudPanel";
import { PromptPanel } from "@/components/panels/PromptPanel";
import { TaskProgressPanel } from "@/components/panels/TaskProgressPanel";
import { CameraFeedPanel } from "@/components/panels/CameraFeedPanel";
import { MotionPanel } from "@/components/panels/MotionPanel";

export const PANEL_COMPONENTS = {
  robotState: RobotStatePanel,
  sceneControls: SceneControlsPanel,
  calibration: CalibrationPanel,
  calibrationActions: CalibrationActionsPanel,
  pointCloud: PointCloudPanel,
  prompt: PromptPanel,
  taskProgress: TaskProgressPanel,
  cameraFeed: CameraFeedPanel,
  motion: MotionPanel,
} as const;

export type PanelComponentKey = keyof typeof PANEL_COMPONENTS;
