import { useCallback, useState } from "react";
import { bridge } from "@/api/bridge";
import { useService, useTopic } from "@/framework";
import { ServiceKey, Topic } from "@/constants/topics";
import { JointSlider } from "@/components/shared/JointSlider";
import { Button } from "@/components/ui/button";
import { Separator } from "@/components/ui/separator";
import { loadPose } from "@/lib/robot/robotPoses";
import type { Joint } from "@/types/motor";

const EMPTY_JOINTS: Joint[] = [];

export function JointPanel() {
  const joints = useTopic(Topic.MOTOR_STATE_JOINT)?.joints ?? EMPTY_JOINTS;
  const cfgSvc = useService(ServiceKey.MOTOR_GET_CONFIG);
  const configs = cfgSvc.data?.motors ?? [];
  const torqueEnabled = cfgSvc.data?.torque_enabled ?? false;
  const enableSvc = useService(ServiceKey.MOTOR_ENABLE);
  const moveJ = useService(ServiceKey.MOTION_MOVE_J);

  const [cmdPositions, setCmdPositions] = useState<Record<number, number>>({});

  const handleJointCmd = useCallback((id: number, position: number) => {
    setCmdPositions((prev) => ({ ...prev, [id]: position }));
    bridge.publish(Topic.MOTOR_CMD_JOINT, {
      timestamp: Date.now() / 1000,
      joints: [{ id, position }],
    });
  }, []);

  const syncAll = useCallback(() => {
    setCmdPositions(Object.fromEntries(joints.map((j) => [j.id, j.position])));
  }, [joints]);

  const goHome = useCallback(async () => {
    const pose = await loadPose("home");
    await moveJ.call({ joints: pose });
  }, [moveJ]);

  const toggleTorque = useCallback(async () => {
    const next = !torqueEnabled;
    const res = await enableSvc.call({ enable: next });
    if (res.success) await cfgSvc.call({}); // refresh motor config cache
  }, [torqueEnabled, enableSvc, cfgSvc]);

  return (
    <div className="flex flex-col gap-2">
      <div className="flex items-center justify-between">
        <h2 className="text-sm font-semibold text-muted-foreground uppercase tracking-wider">
          Joint Control
        </h2>
        <div className="flex gap-2">
          <Button size="sm" variant="outline" onClick={() => void goHome()}>
            Home
          </Button>
          <Button size="sm" variant="outline" onClick={syncAll}>
            Sync
          </Button>
          <Button
            size="sm"
            variant={torqueEnabled ? "destructive" : "default"}
            onClick={() => void toggleTorque()}
          >
            {torqueEnabled ? "Torque OFF" : "Torque ON"}
          </Button>
        </div>
      </div>

      <Separator />

      <div className="flex flex-col divide-y">
        {joints.length === 0 ? (
          <p className="py-4 text-center text-sm text-muted-foreground">
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
