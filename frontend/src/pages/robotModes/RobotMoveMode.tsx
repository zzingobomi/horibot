import { ModeDockview, type PanelSpec } from "./ModeDockview";

const PANELS: PanelSpec[] = [
  { id: "robot-state", component: "robotState", title: "Robot State", width: 260, height: 270 },
  { id: "motion", component: "motion", title: "Motion", width: 320, height: 360 },
  { id: "scene-controls", component: "sceneControls", title: "Scene Controls", width: 260, height: 300 },
  { id: "live-pointcloud", component: "livePointCloud", title: "Live PointCloud", width: 260, height: 260 },
];

export function RobotMoveMode() {
  return <ModeDockview mode="move" panels={PANELS} />;
}
