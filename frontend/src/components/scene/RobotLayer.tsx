/**
 * WorldScene 의 robot URDF 묶음 — robots.yaml 의 N robot 동시 마운트.
 *
 * multi_robot_phase2_frontend.md §2 결정 6 — 도메인 단위 Layer:
 * - RobotsPage (`focusId=<id>`): focus robot 만 불투명, 나머지 dim.
 * - WorldPage (`focusId=null`): 모두 동등하게 불투명.
 *
 * Joint 데이터는 §4 결정 3 의 "임시 호환 코드" — robotStore (focus robot 만
 * subscribe) 의 jointAngles 가 focus robot 에만 전달, 나머지는 home pose.
 * Slice C 에서 robot 별 store dict 화 시 jointAnglesByRobot 으로 확장.
 */
import * as THREE from "three";
import { RobotModel } from "./RobotModel";
import type { RobotInfo } from "@/types/robot";

export interface RobotLayerProps {
  robots: RobotInfo[];
  focusId: string | null;
  jointAngles: number[];
  onLinksLoaded?: (linkNames: string[]) => void;
  onTCPMatrix?: (m: THREE.Matrix4 | null) => void;
  /** focus 모드에서 dim 정도. 0.0–1.0. default 0.25 — 만져보고 조정 자리. */
  dimOpacity?: number;
  /** RobotModel.visible passthrough (focus / world 양쪽 다 hide 가능). */
  showRobot?: boolean;
}

// 빈 배열 → RobotModel 이 motorCfgs 순회 시 setJointValue 호출 안 함 → URDF
// 기본 origin (보통 0 rad) 사용. multi-robot 에서 robot 마다 motor 수 달라도 OK.
const HOME_JOINTS: number[] = [];

export function RobotLayer({
  robots,
  focusId,
  jointAngles,
  onLinksLoaded,
  onTCPMatrix,
  dimOpacity = 0.25,
  showRobot = true,
}: RobotLayerProps) {
  return (
    <>
      {robots.map((r) => {
        const isFocus = focusId === null || r.id === focusId;
        const opacity = isFocus ? 1.0 : dimOpacity;
        // 콜백은 focus robot 1개만 — RobotScene 의 TCP/Camera layer 가 그 한 대
        // 의 데이터로 그려짐. WorldPage(focusId=null) 에선 첫 번째 enabled robot.
        const isCallbackTarget =
          focusId === r.id ||
          (focusId === null && r.id === (robots.find((x) => x.enabled)?.id ?? robots[0]?.id));

        return (
          <RobotModel
            key={r.id}
            robotType={r.type}
            robotId={r.id}
            basePose={r.base_pose}
            opacity={opacity}
            jointAngles={isFocus ? jointAngles : HOME_JOINTS}
            visible={showRobot}
            onTCPMatrix={isCallbackTarget ? onTCPMatrix ?? undefined : undefined}
            onLinksLoaded={isCallbackTarget ? onLinksLoaded : undefined}
          />
        );
      })}
    </>
  );
}
