/**
 * robot 모터 layout — backend `MOTOR_GET_CONFIG` 서비스 응답에서 derive.
 *
 * 이전엔 `JOINT_CONFIGS` (id/name/type) 가 하드코드 상수였음 — `motors.yaml`
 * 의 frontend 거울. multi-robot (so101 = 6DOF + gripper, id 7) 도입 시 손
 * 동기화 hazard. backend 가 `MotorConfigItem.kind` 로 노출하는 SSOT 를 직접
 * 사용해 frontend 하드코드 제거.
 */
import { useMemo } from "react";
import { useService } from "@/framework";
import { ServiceKey } from "@/constants/topics";
import type { components } from "@/api/generated/types";

export type MotorConfigItem = components["schemas"]["MotorConfigItem"];

// Project-wide URDF 컨벤션: 모든 robot type 의 URDF 는 TCP 를 가리키는 `tcp`
// 이름의 link 를 가져야 함. backend `pybullet_kinematics.TCP_LINK_NAME` 와 같은
// 컨벤션을 frontend `urdf-loader` 가 link lookup 에 사용. 새 robot type 추가
// 시 URDF 에 `<link name="tcp"/>` 를 박으면 양쪽 다 추가 config 없이 동작.
export const TCP_LINK_NAME = "tcp";

/** id 오름차순 정렬된 motor config. robotId 미지정 시 bridge default robot. */
export function useMotorConfigs(robotId?: string): MotorConfigItem[] {
  const motors = useService(ServiceKey.MOTOR_GET_CONFIG, robotId).data?.motors;
  return useMemo(
    () => (motors ? [...motors].sort((a, b) => a.id - b.id) : []),
    [motors],
  );
}

export function useArmJoints(robotId?: string): MotorConfigItem[] {
  return useMotorConfigs(robotId).filter((m) => m.kind === "arm");
}

export function useGripperJoint(robotId?: string): MotorConfigItem | undefined {
  return useMotorConfigs(robotId).find((m) => m.kind === "gripper");
}
