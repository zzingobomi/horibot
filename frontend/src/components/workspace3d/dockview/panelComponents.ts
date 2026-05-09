import { RobotStatePanel } from "../panels/RobotStatePanel";
import { SceneControlsPanel } from "../panels/SceneControlsPanel";
import { CalibrationPanel } from "../panels/CalibrationPanel";
import { PointCloudPanel } from "../panels/PointCloudPanel";

export const PANEL_COMPONENTS = {
  robotState: RobotStatePanel,
  sceneControls: SceneControlsPanel,
  calibration: CalibrationPanel,
  pointCloud: PointCloudPanel,
} as const;

export type PanelComponentKey = keyof typeof PANEL_COMPONENTS;
