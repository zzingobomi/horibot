import { create } from "zustand";
import type { Joint, MotorConfig } from "@/types/motor";

interface RobotStore {
  joints: Joint[];
  configs: MotorConfig[];
  torqueEnabled: boolean;
  /**
   * Hand-Eye 캘리브레이션이 추정한 조인트 zero offset (라디안).
   * key = motor_id. URDF에 조인트각 적용 시 raw_to_rad에 더해야 함.
   * 백엔드 JointStateCache와 동일한 보정을 프론트에서도 수행해 워크스페이스
   * URDF가 실제 모터 상태와 일치하도록 함.
   */
  jointOffsetsRad: Record<number, number>;
  setJoints: (joints: Joint[]) => void;
  setConfigs: (configs: MotorConfig[]) => void;
  setTorque: (enabled: boolean) => void;
  setJointOffsets: (offsets: Record<number, number>) => void;
}

export const useRobotStore = create<RobotStore>((set) => ({
  joints: [],
  configs: [],
  torqueEnabled: false,
  jointOffsetsRad: {},
  setJoints: (joints) => set({ joints }),
  setConfigs: (configs) => set({ configs }),
  setTorque: (enabled) => set({ torqueEnabled: enabled }),
  setJointOffsets: (offsets) => set({ jointOffsetsRad: offsets }),
}));
