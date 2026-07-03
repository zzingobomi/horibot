/**
 * dockview panel 레지스트리 — key → 등록 패널.
 *
 * 패널만 import 하는 순수 map. 각 패널이 useParams(robot context) + scroll
 * container 를 자체 흡수하므로 여기엔 로직·wrapper 없음. 새 패널 추가 =
 * 컴포넌트 만들고 여기 한 줄 + mode 파일 PANELS 한 줄 (frontend_v2.md §2.3).
 */
import { RobotStatePanel } from "./RobotStatePanel";
import { MotionPanel } from "./MotionPanel";
import { CalibrationPanel } from "./CalibrationPanel";
import { ScanPanel } from "./ScanPanel";

export const PANEL_COMPONENTS = {
  robotState: RobotStatePanel,
  motion: MotionPanel,
  calibration: CalibrationPanel,
  scan: ScanPanel,
} as const;

export type PanelComponentKey = keyof typeof PANEL_COMPONENTS;
