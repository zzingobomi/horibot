import { RobotStatePanel } from "@/components/panels/RobotStatePanel";
import { SceneControlsPanel } from "@/components/panels/SceneControlsPanel";
import { CalibrationPanel } from "@/components/panels/CalibrationPanel";
import { PointCloudPanel } from "@/components/panels/PointCloudPanel";

export const PANEL_COMPONENTS = {
  robotState: RobotStatePanel,
  sceneControls: SceneControlsPanel,
  calibration: CalibrationPanel,
  pointCloud: PointCloudPanel,
} as const;

export type PanelComponentKey = keyof typeof PANEL_COMPONENTS;
