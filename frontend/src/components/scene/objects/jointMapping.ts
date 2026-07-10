// URDF joint 매핑 helper — RobotModel 에서 뽑음 (react-refresh 규칙 + 단위 테스트 target).
//
// SSOT 원칙: backend `TcpState.joint_names` = motors.yaml arm prefix 순서 (계약).
// frontend 는 URDF 파일 순서 안 믿고 이 name list 로 URDF joint 를 찾아 매핑.
// 2026-07-01 회귀: `Object.entries(robot.joints)` 로 URDF 파일 순서 (SO-101 URDF 는
// joint7→joint1 역순) 에 그냥 index 매핑 → gripper 자리에 J1 rad 들어가 랜더 뒤집힘.

import type { URDFRobot } from "urdf-loader";

/** jointNames[i] 로 URDF joint 를 찾아 angles[i] 로 setJointValue. name 없거나
 *  angle undefined 는 skip (mount 직후 empty state stream 자연스러운 no-op). */
export function applyJoints(
  robot: URDFRobot,
  names: string[],
  angles: number[],
) {
  names.forEach((name, i) => {
    const angle = angles[i];
    if (angle !== undefined && robot.joints?.[name]) {
      robot.setJointValue(name, angle);
    }
  });
}
