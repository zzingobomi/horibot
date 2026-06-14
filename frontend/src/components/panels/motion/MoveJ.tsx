import { useCallback, useMemo, useState } from "react";
import { useParams } from "react-router-dom";
import * as SliderPrimitive from "@radix-ui/react-slider";
import { PanelButton } from "@/components/shared/PanelButton";
import { useService, useTopic } from "@/framework";
import { ServiceKey, Topic } from "@/constants/topics";
import { rawToDeg } from "@/lib/robot/utils";
import { useArmJoints, useMotorConfigs } from "@/lib/robot/config";

export function MoveJControl() {
  const { id: robotId = "" } = useParams<{ id: string }>();
  const joints = useTopic(Topic.MOTOR_STATE_JOINT, robotId)?.joints ?? [];
  const configs = useMotorConfigs(robotId);
  const armJoints = useArmJoints(robotId);
  const traj = useTopic(Topic.MOTION_STATE_TRAJ, robotId);
  const moveJ = useService(ServiceKey.MOTION_MOVE_J, robotId);
  const stop = useService(ServiceKey.MOTION_STOP, robotId);

  const armIds = useMemo(() => new Set(armJoints.map((j) => j.id)), [armJoints]);
  const currentJoints = joints.filter((j) => armIds.has(j.id));

  const [targetDeg, setTargetDeg] = useState<Record<number, number>>({});
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
    const targetJoints = armJoints.map((j) => ({
      id: j.id,
      degree: targetDeg[j.id] ?? 0,
    }));
    const res = await moveJ.call({ joints: targetJoints });
    if (!res.success) setError(res.message || "MoveJ 실패");
  };

  const isRunning = traj?.status === "running";
  const progress = Math.round((traj?.progress ?? 0) * 100);

  return (
    <div className="flex flex-col gap-3">
      <div className="flex flex-col gap-2">
        {armJoints.map((j) => {
          const current = currentJoints.find((c) => c.id === j.id);
          const target = targetDeg[j.id] ?? 0;
          const hw = configs.find((c) => c.id === j.id);
          const minDeg = rawToDeg(hw?.limit.min ?? 0);
          const maxDeg = rawToDeg(hw?.limit.max ?? 4095);
          const clipped = Math.max(minDeg, Math.min(maxDeg, target));

          return (
            <div key={j.id} className="flex items-center gap-2 font-mono">
              <div className="w-14 shrink-0">
                <div className="text-[11px] text-zinc-300">{j.name}</div>
                {current && (
                  <div className="text-[9px] text-zinc-600 tabular-nums">
                    {current.degree.toFixed(1)}°
                  </div>
                )}
              </div>
              <SliderPrimitive.Root
                className="relative flex items-center select-none touch-none flex-1 h-4"
                min={minDeg}
                max={maxDeg}
                step={0.5}
                value={[clipped]}
                onValueChange={(v) =>
                  setTargetDeg((p) => ({ ...p, [j.id]: v[0] }))
                }
              >
                <SliderPrimitive.Track className="relative h-1 w-full grow rounded-full bg-zinc-800">
                  <SliderPrimitive.Range className="absolute h-full rounded-full bg-blue-500/40" />
                </SliderPrimitive.Track>
                <SliderPrimitive.Thumb className="block h-3.5 w-3.5 rounded-full border border-blue-400 bg-zinc-900 shadow-sm focus-visible:outline-none focus-visible:ring-1 focus-visible:ring-blue-400" />
              </SliderPrimitive.Root>
              <div className="flex items-center gap-0.5 w-16 shrink-0">
                <input
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
                  className="h-6 w-full px-1 text-[10px] text-right text-blue-400 tabular-nums bg-zinc-900 border border-zinc-800 rounded focus:outline-none focus:border-blue-500/60"
                />
                <span className="text-[10px] text-zinc-600">°</span>
              </div>
            </div>
          );
        })}
      </div>

      <div className="flex items-center gap-2 font-mono">
        <span className="text-[10px] uppercase tracking-wide text-zinc-500 w-14 shrink-0">
          Duration
        </span>
        <SliderPrimitive.Root
          className="relative flex items-center select-none touch-none flex-1 h-4"
          min={0.5}
          max={10}
          step={0.5}
          value={[duration]}
          onValueChange={(v) => setDuration(v[0])}
        >
          <SliderPrimitive.Track className="relative h-1 w-full grow rounded-full bg-zinc-800">
            <SliderPrimitive.Range className="absolute h-full rounded-full bg-blue-500/40" />
          </SliderPrimitive.Track>
          <SliderPrimitive.Thumb className="block h-3.5 w-3.5 rounded-full border border-blue-400 bg-zinc-900 shadow-sm focus-visible:outline-none focus-visible:ring-1 focus-visible:ring-blue-400" />
        </SliderPrimitive.Root>
        <span className="text-[10px] text-zinc-400 tabular-nums w-12 text-right">
          {duration.toFixed(1)} s
        </span>
      </div>

      {traj && traj.status !== "idle" && (
        <div className="flex flex-col gap-1 font-mono">
          <div className="flex justify-between text-[10px] text-zinc-500">
            <span>
              {traj.status === "running" && "실행 중…"}
              {traj.status === "done" && "완료"}
              {traj.status === "failed" && "IK 실패"}
              {traj.status === "stopped" && "중단됨"}
            </span>
            <span className="tabular-nums">{progress}%</span>
          </div>
          <div className="h-1 w-full rounded-full bg-zinc-800 overflow-hidden">
            <div
              className="h-full bg-blue-500/70 rounded-full transition-all duration-100"
              style={{ width: `${progress}%` }}
            />
          </div>
        </div>
      )}

      {error && (
        <p className="text-[10px] font-mono text-red-400">{error}</p>
      )}

      <div className="flex gap-2">
        <PanelButton
          variant="outline"
          onClick={applyCurrentJointAngles}
          disabled={currentJoints.length === 0}
          className="flex-1"
        >
          현재 자세
        </PanelButton>
        <PanelButton
          variant="primary"
          onClick={handleExecute}
          disabled={moveJ.pending || isRunning}
          className="flex-1"
        >
          {moveJ.pending ? "전송 중…" : "실행"}
        </PanelButton>
        <PanelButton
          variant="danger"
          onClick={() => void stop.call({})}
          disabled={!isRunning}
        >
          Stop
        </PanelButton>
      </div>
    </div>
  );
}
