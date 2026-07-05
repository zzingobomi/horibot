import { ModeDockview, type PanelSpec } from "@/components/shared/ModeDockview";

/**
 * Scan mode panel set — rgbd capability robot 자리 (backend scene3d + scan module).
 *
 * 3D 뷰(라이브 PC / reconstruction mesh)는 RobotsLayout Canvas 의 Scene3DLayer /
 * MeshLayer 가 렌더 — 이 패널은 워크플로 트리거(캡처/빌드/mesh 보기)만.
 *
 * 실용 슬라이스(Task DSL 미사용). Task DSL 도입 시 scan 은 TasksPage 로 흡수 —
 * 그때 이 mode/panel 은 제거 후보 (scan_interactive_design.md §2 C).
 */
const PANELS: PanelSpec[] = [
  { id: "robot-state", component: "robotState", title: "Robot State", width: 260, height: 300 },
  { id: "scan", component: "scan", title: "Scan", width: 320, height: 520 },
];

export function RobotScanMode() {
  return <ModeDockview mode="scan" panels={PANELS} />;
}
