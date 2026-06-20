import { useEffect } from "react";
import { useParams } from "react-router-dom";

import { useCalibrationStore } from "@/domain/stores/calibration";

import { ModeDockview, type PanelSpec } from "./ModeDockview";

const PANELS: PanelSpec[] = [
  {
    id: "robot-state",
    component: "robotState",
    title: "Robot State",
    width: 260,
    height: 270,
  },
  {
    id: "calibration",
    component: "calibration",
    title: "Calibration",
    width: 300,
    height: 480,
  },
  {
    id: "calibration-camera",
    component: "calibrationCamera",
    title: "Calibration Camera",
    width: 520,
    height: 420,
  },
  {
    id: "intrinsic",
    component: "intrinsic",
    title: "Intrinsic",
    width: 280,
    height: 420,
  },
  {
    id: "motion",
    component: "motion",
    title: "Motion",
    width: 320,
    height: 360,
  },
];

export function RobotCalibrateMode() {
  const { id: robotId } = useParams<{ id: string }>();
  // calibrationStore 라이프사이클 — mode 진입 시 subscribe + initial fetch,
  // 이탈 시 unsubscribe + state clear. 각 panel 의 mount/unmount 와 분리
  // (panel 들이 close 되어도 mode 가 살아있으면 store 유지).
  useEffect(() => {
    if (!robotId) return;
    useCalibrationStore.getState().bootstrap(robotId);
    return () => useCalibrationStore.getState().dispose();
  }, [robotId]);

  return <ModeDockview mode="calibrate" panels={PANELS} />;
}
