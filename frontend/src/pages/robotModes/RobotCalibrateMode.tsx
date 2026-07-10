import { ModeDockview, type PanelSpec } from "@/components/shared/ModeDockview";

/**
 * Calibrate mode panel set. RobotState(torque/home/jog 흡수) + Calibration
 * (preview traffic light + capture 세션 + active bundle + history/rollback) +
 * Camera(color 뷰 + ChArUco 마커 오버레이 — 토크오프 자세 잡기 시 보드 조준 확인).
 *
 * Hand-Eye / Intrinsic 세부 패널은 후속 (capture 세션이 코어 — intrinsic 은 D405
 * factory / USB 캘 자리). 새 패널 = registry 한 줄 + 여기 PANELS 한 줄.
 */
// title/width/height 는 PANEL_CATALOG(SSOT)에서 derive — 여기선 배치 선언만.
// (옛 중복 선언이 캘 카메라 탭을 "Camera" 로 덮어 3종 카메라 구분 불가하던 사고.)
const PANELS: PanelSpec[] = [
  { id: "robot-state", component: "robotState" },
  { id: "calibration", component: "calibration" },
  { id: "camera", component: "calibrationCamera" },
];

export function RobotCalibrateMode() {
  return <ModeDockview mode="calibrate" panels={PANELS} />;
}
