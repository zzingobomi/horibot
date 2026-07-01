/**
 * Multi-robot URDF mount — robots.yaml 의 N robot 동시 마운트.
 *
 * frontend_v2 first cut 에선 focus mode 박지 X (Move 페이지 = single robot).
 * Step E+ (Calibration / Scan) 박힐 때 focus / dim opacity 추가.
 */
import { RobotModel } from "./RobotModel";
import type { RobotInfo } from "@/api/generated/contract";

export interface RobotLayerProps {
  robots: RobotInfo[];
  /** focus robot id — null = 모두 동등. */
  focusId?: string | null;
  /** focus robot 의 arm joint name list (backend TcpState.joint_names SSOT). */
  jointNames: string[];
  /** focus robot 의 joint angles (rad). jointNames 와 same index. non-focus = home pose. */
  jointAngles: number[];
  onLinksLoaded?: (linkNames: string[]) => void;
  dimOpacity?: number;
  showRobot?: boolean;
}

const HOME_JOINTS: number[] = [];
const HOME_NAMES: string[] = [];

export function RobotLayer({
  robots,
  focusId = null,
  jointNames,
  jointAngles,
  onLinksLoaded,
  dimOpacity = 0.25,
  showRobot = true,
}: RobotLayerProps) {
  return (
    <>
      {robots.map((r) => {
        const isFocus = focusId === null || r.id === focusId;
        const opacity = isFocus ? 1.0 : dimOpacity;
        const isCallbackTarget =
          focusId === r.id || (focusId === null && r.id === robots[0]?.id);

        return (
          <RobotModel
            key={r.id}
            robotType={r.type}
            basePose={r.base_pose}
            opacity={opacity}
            jointNames={isFocus ? jointNames : HOME_NAMES}
            jointAngles={isFocus ? jointAngles : HOME_JOINTS}
            visible={showRobot}
            onLinksLoaded={isCallbackTarget ? onLinksLoaded : undefined}
          />
        );
      })}
    </>
  );
}
