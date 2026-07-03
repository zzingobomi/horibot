import { ModeDockview, type PanelSpec } from "./ModeDockview";

/**
 * Assets mode panel set — Robot Asset Layer (Motion 위).
 *
 * Waypoint(티칭 joint 자세) 라이브러리 + group 관리. PnP / Scan / Inspection 등
 * 여러 워크플로가 공유하는 자산 (docs/backend_v2.md §17.2). 티칭은 현재
 * joint 를 저장 — Robot State 패널로 자세 확인/토크오프하며 티칭.
 */
const PANELS: PanelSpec[] = [
  { id: "robot-state", component: "robotState", title: "Robot State", width: 260, height: 300 },
  { id: "waypoints", component: "waypoints", title: "Waypoints", width: 340, height: 560 },
];

export function RobotAssetsMode() {
  return <ModeDockview mode="assets" panels={PANELS} />;
}
