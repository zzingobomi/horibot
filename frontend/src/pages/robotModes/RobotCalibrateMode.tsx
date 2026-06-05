import { ModeDockview, type PanelSpec } from "./ModeDockview";

const PANELS: PanelSpec[] = [
  { id: "robot-state", component: "robotState", title: "Robot State", width: 260, height: 270 },
  { id: "calibration", component: "calibration", title: "Calibration", width: 260, height: 260 },
  { id: "calibration-actions", component: "calibrationActions", title: "Calibration Actions", width: 320, height: 360 },
  { id: "scene-controls", component: "sceneControls", title: "Scene Controls", width: 260, height: 300 },
];

export function RobotCalibrateMode() {
  return <ModeDockview mode="calibrate" panels={PANELS} />;
}
