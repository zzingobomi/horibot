import { ModeDockview, type PanelSpec } from "./ModeDockview";

const PANELS: PanelSpec[] = [
  { id: "robot-state", component: "robotState", title: "Robot State", width: 260, height: 270 },
  { id: "point-cloud", component: "pointCloud", title: "Point Cloud", width: 260, height: 240 },
  { id: "scene-controls", component: "sceneControls", title: "Scene Controls", width: 260, height: 300 },
];

export function RobotScanMode() {
  return <ModeDockview mode="scan" panels={PANELS} />;
}
