import { ModeDockview, type PanelSpec } from "./ModeDockview";

/**
 * Move mode panel set. jog slice 의 RobotStatePanel + Motion(jog) 을 dockview
 * 패널로 fold (옛 MovePage 의 3-column grid 대체).
 *
 * sceneControls / livePointCloud 패널은 Step E+ — backend scene3d module 부재.
 * 박히면 여기 PANELS 에 한 줄 추가 + registry 등록으로 snap-in (frontend_v2.md §2.3).
 */
const PANELS: PanelSpec[] = [
  { id: "robot-state", component: "robotState", title: "Robot State", width: 260, height: 300 },
  { id: "motion", component: "motion", title: "Motion", width: 320, height: 380 },
];

export function RobotMoveMode() {
  return <ModeDockview mode="move" panels={PANELS} />;
}
