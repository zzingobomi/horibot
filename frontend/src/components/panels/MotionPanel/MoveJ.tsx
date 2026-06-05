import { useCallback, useState } from "react";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Slider } from "@/components/ui/slider";
import { Progress } from "@/components/ui/progress";
import { useService, useTopic } from "@/framework";
import { ServiceKey, Topic } from "@/constants/topics";
import { rawToDeg } from "@/lib/robot/utils";
import { ARM_JOINTS } from "@/lib/robot/config";

export function MoveJControl() {
  const joints = useTopic(Topic.MOTOR_STATE_JOINT)?.joints ?? [];
  const configs =
    useService(ServiceKey.MOTOR_GET_CONFIG).data?.motors ?? [];
  const traj = useTopic(Topic.MOTION_STATE_TRAJ);
  const moveJ = useService(ServiceKey.MOTION_MOVE_J);
  const stop = useService(ServiceKey.MOTION_STOP);

  const armIds = new Set(ARM_JOINTS.map((j) => j.id));
  const currentJoints = joints.filter((j) => armIds.has(j.id));

  const [targetDeg, setTargetDeg] = useState<Record<number, number>>(
    Object.fromEntries(ARM_JOINTS.map((j) => [j.id, 0])),
  );
  const [duration, setDuration] = useState(3.0);
  const [error, setError] = useState<string | null>(null);

  const applyCurrentJointAngles = useCallback(() => {
    if (currentJoints.length === 0) return;
    setTargetDeg((prev) => {
      const next = { ...prev };
      currentJoints.forEach((j) => {
        next[j.id] = Math.round(j.degree * 10) / 10;
      });
      return next;
    });
  }, [currentJoints]);

  const handleExecute = async () => {
    setError(null);
    const targetJoints = ARM_JOINTS.map((j) => ({
      id: j.id,
      degree: targetDeg[j.id] ?? 0,
    }));
    const res = await moveJ.call({ joints: targetJoints });
    if (!res.success) setError(res.message || "MoveJ 실패");
  };

  const isRunning = traj?.status === "running";
  const progress = Math.round((traj?.progress ?? 0) * 100);

  return (
    <div className="flex flex-col gap-4">
      <div className="flex flex-col gap-3">
        {ARM_JOINTS.map((j) => {
          const current = currentJoints.find((c) => c.id === j.id);
          const target = targetDeg[j.id] ?? 0;
          const hw = configs.find((c) => c.id === j.id);
          const minDeg = rawToDeg(hw?.limit.min ?? 0);
          const maxDeg = rawToDeg(hw?.limit.max ?? 4095);
          const clipped = Math.max(minDeg, Math.min(maxDeg, target));

          return (
            <div
              key={j.id}
              className="grid grid-cols-[80px_1fr_72px] items-center gap-2"
            >
              <div>
                <Label className="text-xs font-medium">{j.label}</Label>
                {current && (
                  <p className="text-[10px] text-muted-foreground">
                    현재 {current.degree.toFixed(1)}°
                  </p>
                )}
              </div>
              <Slider
                min={minDeg}
                max={maxDeg}
                step={0.5}
                value={[clipped]}
                onValueChange={(v) =>
                  setTargetDeg((p) => ({ ...p, [j.id]: v[0] }))
                }
                className="w-full"
              />
              <div className="flex items-center gap-0.5">
                <Input
                  type="number"
                  value={target}
                  min={minDeg}
                  max={maxDeg}
                  step={0.5}
                  onChange={(e) => {
                    const num = parseFloat(e.target.value);
                    if (!isNaN(num))
                      setTargetDeg((p) => ({ ...p, [j.id]: num }));
                  }}
                  className="h-7 w-full px-1.5 text-xs text-right"
                />
                <span className="text-xs text-muted-foreground">°</span>
              </div>
            </div>
          );
        })}
      </div>

      <div className="flex items-center gap-2">
        <Label className="text-xs whitespace-nowrap">Duration</Label>
        <Slider
          min={0.5}
          max={10}
          step={0.5}
          value={[duration]}
          onValueChange={(v) => setDuration(v[0])}
          className="flex-1"
        />
        <span className="text-xs text-muted-foreground w-12 text-right">
          {duration.toFixed(1)} s
        </span>
      </div>

      {traj && traj.status !== "idle" && (
        <div className="flex flex-col gap-1">
          <div className="flex justify-between text-xs text-muted-foreground">
            <span>
              {traj.status === "running" && "실행 중…"}
              {traj.status === "done" && "완료"}
              {traj.status === "failed" && "IK 실패"}
              {traj.status === "stopped" && "중단됨"}
            </span>
            <span>{progress}%</span>
          </div>
          <Progress value={progress} className="h-1.5" />
        </div>
      )}

      {error && <p className="text-xs text-destructive">{error}</p>}

      <div className="flex gap-2">
        <Button
          variant="outline"
          size="sm"
          className="flex-1"
          onClick={applyCurrentJointAngles}
          disabled={currentJoints.length === 0}
        >
          현재 자세 불러오기
        </Button>
        <Button
          size="sm"
          className="flex-1"
          onClick={handleExecute}
          disabled={moveJ.pending || isRunning}
        >
          {moveJ.pending ? "전송 중…" : "실행"}
        </Button>
        <Button
          variant="destructive"
          size="sm"
          onClick={() => void stop.call({})}
          disabled={!isRunning}
        >
          Stop
        </Button>
      </div>
    </div>
  );
}
