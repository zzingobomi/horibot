import { ModeDockview, type PanelSpec } from "@/components/shared/ModeDockview";

/**
 * Move mode panel set. jog slice 의 RobotStatePanel + Motion(jog) 을 dockview
 * 패널로 fold (옛 MovePage 의 3-column grid 대체).
 *
 * sceneControls / livePointCloud 패널은 Step E+ — backend scene3d module 부재.
 * 박히면 여기 PANELS 에 한 줄 추가 + registry 등록으로 snap-in (frontend.md §2.3).
 */
// title/width/height 는 PANEL_CATALOG(SSOT)에서 derive — 여기선 배치 선언만.
const PANELS: PanelSpec[] = [
  { id: "robot-state", component: "robotState" },
  { id: "motion", component: "motion" },
  { id: "move-preview", component: "movePreview" },
];

export function RobotMoveMode() {
  return <ModeDockview mode="move" panels={PANELS} />;
}
