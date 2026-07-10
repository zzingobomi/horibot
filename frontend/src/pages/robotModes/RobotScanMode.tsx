import { ModeDockview, type PanelSpec } from "@/components/shared/ModeDockview";

/**
 * Scan mode panel set — rgbd capability robot 자리 (backend scene3d + scan module).
 *
 * 3D 뷰(라이브 PC / reconstruction mesh)는 RobotsLayout Canvas 의 Camera 씬 객체 /
 * ScanMesh 가 렌더. Camera 패널은 color 카메라 뷰(자세 잡기 확인용) — 3D 점군은
 * "카메라가 본 것의 가공 결과"라 원본 color 를 같이 봐야 어디를 비추는지 알 수 있음.
 *
 * 실용 슬라이스(Task DSL 미사용). Task DSL 도입 시 scan 은 TasksPage 로 흡수 —
 * 그때 이 mode/panel 은 제거 후보 (scan_interactive_design.md §2 C).
 */
// title/width/height 는 PANEL_CATALOG(SSOT)에서 derive — 여기선 배치 선언만.
const PANELS: PanelSpec[] = [
  { id: "robot-state", component: "robotState" },
  { id: "scan", component: "scan" },
  { id: "live-pc", component: "livePointCloud" },
  { id: "camera", component: "camera" },
];

export function RobotScanMode() {
  return <ModeDockview mode="scan" panels={PANELS} />;
}
