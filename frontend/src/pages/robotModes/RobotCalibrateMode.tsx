import { useEffect } from "react";
import { ModeDockview, type PanelSpec } from "./ModeDockview";
import { useCalibrationStore } from "@/domain/stores/calibration";

const PANELS: PanelSpec[] = [
  { id: "robot-state", component: "robotState", title: "Robot State", width: 260, height: 270 },
  { id: "calibration", component: "calibration", title: "Calibration", width: 260, height: 260 },
  { id: "calibration-camera", component: "calibrationCamera", title: "Calibration Camera", width: 520, height: 420 },
  { id: "hand-eye", component: "handEye", title: "Hand-Eye", width: 320, height: 640 },
  { id: "intrinsic", component: "intrinsic", title: "Intrinsic", width: 280, height: 420 },
  { id: "rollback", component: "rollback", title: "Rollback", width: 320, height: 360 },
  { id: "scene-controls", component: "sceneControls", title: "Scene Controls", width: 260, height: 300 },
];

export function RobotCalibrateMode() {
  // calibrationStore 라이프사이클 — mode 진입 시 subscribe + initial fetch,
  // 이탈 시 unsubscribe + state clear. 각 panel 의 mount/unmount 와 분리
  // (panel 들이 close 되어도 mode 가 살아있으면 store 유지).
  useEffect(() => {
    useCalibrationStore.getState().bootstrap();
    return () => useCalibrationStore.getState().dispose();
  }, []);

  return <ModeDockview mode="calibrate" panels={PANELS} />;
}
