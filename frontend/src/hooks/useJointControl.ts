import { useRobotStore } from "@/store/robotStore";
import { useCallback } from "react";
import { bridge } from "@/api/bridge";
import { Topic, ServiceKey } from "@/constants/topics";
import { useMotion } from "@/hooks/useMotion";
import { loadPose } from "@/lib/robot/robotPoses";

export function useJointControl() {
  const torqueEnabled = useRobotStore((s) => s.torqueEnabled);
  const setTorque = useRobotStore((s) => s.setTorque);
  const motion = useMotion();

  const sendJointCmd = useCallback((id: number, position: number) => {
    bridge.publish(Topic.MOTOR_CMD_JOINT, {
      timestamp: Date.now() / 1000,
      joints: [{ id, position }],
    });
  }, []);

  const sendAllJoints = useCallback(
    (joints: { id: number; position: number }[]) => {
      bridge.publish(Topic.MOTOR_CMD_JOINT, {
        timestamp: Date.now() / 1000,
        joints,
      });
    },
    []
  );

  const enableTorque = useCallback(
    async (enable: boolean) => {
      const res = await bridge.callService(ServiceKey.MOTOR_ENABLE, { enable });
      if (res.success) setTorque(enable);
      return res;
    },
    [setTorque]
  );

  const goHome = useCallback(async () => {
    const joints = await loadPose("home");
    await motion.moveJ({ joints });
  }, [motion]);

  const goRest = useCallback(async () => {
    const joints = await loadPose("rest");
    await motion.moveJ({ joints });
  }, [motion]);

  return {
    torqueEnabled,
    sendJointCmd,
    sendAllJoints,
    enableTorque,
    goHome,
    goRest,
  };
}
