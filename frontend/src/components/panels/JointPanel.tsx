import { useCallback, useState } from "react";
import { useParams } from "react-router-dom";
import { bridge } from "@/api/bridge";
import { useService, useTopic } from "@/framework";
import { ServiceKey, Topic } from "@/constants/topics";
import { JointSlider } from "@/components/shared/JointSlider";
import { PanelButton } from "@/components/shared/PanelButton";
import { loadPose } from "@/lib/robot/robotPoses";
import type { Joint } from "@/types/motor";

const EMPTY_JOINTS: Joint[] = [];

export function JointPanel() {
  const { id: robotId = "" } = useParams<{ id: string }>();
  const joints = useTopic(Topic.MOTOR_STATE_JOINT, robotId)?.joints ?? EMPTY_JOINTS;
  const cfgSvc = useService(ServiceKey.MOTOR_GET_CONFIG, robotId);
  const configs = cfgSvc.data?.motors ?? [];
  const torqueEnabled = cfgSvc.data?.torque_enabled ?? false;
  const enableSvc = useService(ServiceKey.MOTOR_ENABLE, robotId);
  const moveJ = useService(ServiceKey.MOTION_MOVE_J, robotId);

  const [cmdPositions, setCmdPositions] = useState<Record<number, number>>({});

  const handleJointCmd = useCallback(
    (id: number, position: number) => {
      setCmdPositions((prev) => ({ ...prev, [id]: position }));
      bridge.publish(
        Topic.MOTOR_CMD_JOINT,
        {
          timestamp: Date.now() / 1000,
          joints: [{ id, position }],
        },
        robotId,
      );
    },
    [robotId],
  );

  const syncAll = useCallback(() => {
    setCmdPositions(Object.fromEntries(joints.map((j) => [j.id, j.position])));
  }, [joints]);

  const goHome = useCallback(async () => {
    const pose = await loadPose(robotId, "home");
    await moveJ.call({ joints: pose });
  }, [robotId, moveJ]);

  const toggleTorque = useCallback(async () => {
    const next = !torqueEnabled;
    const res = await enableSvc.call({ enable: next });
    if (res.success) await cfgSvc.call({});
  }, [torqueEnabled, enableSvc, cfgSvc]);

  return (
    <div className="flex flex-col gap-3">
      <div className="flex items-center gap-2">
        <PanelButton
          variant="outline"
          onClick={() => void goHome()}
          className="flex-1"
        >
          Home
        </PanelButton>
        <PanelButton
          variant="outline"
          onClick={syncAll}
          className="flex-1"
        >
          Sync
        </PanelButton>
        <PanelButton
          variant={torqueEnabled ? "danger" : "primary"}
          onClick={() => void toggleTorque()}
          className="flex-1"
        >
          Torque {torqueEnabled ? "OFF" : "ON"}
        </PanelButton>
      </div>

      <div className="flex flex-col divide-y divide-zinc-800/60">
        {joints.length === 0 ? (
          <p className="py-3 text-center text-[11px] text-zinc-500 font-mono">
            모터 연결 대기 중...
          </p>
        ) : (
          joints.map((joint) => {
            const cfg = configs.find((c) => c.id === joint.id);
            return (
              <JointSlider
                key={joint.id}
                joint={joint}
                cmdPosition={cmdPositions[joint.id] ?? joint.position}
                limitMin={cfg?.limit.min ?? 0}
                limitMax={cfg?.limit.max ?? 4095}
                onValueChange={handleJointCmd}
              />
            );
          })
        )}
      </div>
    </div>
  );
}
