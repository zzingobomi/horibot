import { RobotStatePanel } from "../panels/RobotStatePanel";
import { SceneControlsPanel } from "../panels/SceneControlsPanel";
import { CalibrationPanel } from "../panels/CalibrationPanel";

export const PANEL_COMPONENTS = {
  robotState: RobotStatePanel,
  sceneControls: SceneControlsPanel,
  calibration: CalibrationPanel,
} as const;

export type PanelComponentKey = keyof typeof PANEL_COMPONENTS;
