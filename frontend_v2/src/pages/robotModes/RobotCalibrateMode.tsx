import { ModeDockview, type PanelSpec } from "./ModeDockview";

/**
 * Calibrate mode panel set. RobotState(torque/home/jog 흡수) + Calibration
 * (preview traffic light + capture 세션 + active bundle + history/rollback).
 *
 * Hand-Eye / Intrinsic 세부 패널은 후속 (capture 세션이 코어 — intrinsic 은 D405
 * factory / USB 캘 자리). 새 패널 = registry 한 줄 + 여기 PANELS 한 줄.
 */
const PANELS: PanelSpec[] = [
  { id: "robot-state", component: "robotState", title: "Robot State", width: 260, height: 300 },
  { id: "calibration", component: "calibration", title: "Calibration", width: 360, height: 520 },
];

export function RobotCalibrateMode() {
  return <ModeDockview mode="calibrate" panels={PANELS} />;
}
