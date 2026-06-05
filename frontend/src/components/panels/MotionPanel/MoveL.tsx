import { useCallback, useState } from "react";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Slider } from "@/components/ui/slider";
import { Progress } from "@/components/ui/progress";
import { useService, useTopic } from "@/framework";
import { ServiceKey, Topic } from "@/constants/topics";
import { mmToMVec3, mToMmVec3 } from "@/lib/robot/utils";
import type { Vec3 } from "@/types/motion";

const AXES = ["X", "Y", "Z"] as const;

export function MoveLControl() {
  const tcpSvc = useService(ServiceKey.MOTION_GET_TCP);
  const moveL = useService(ServiceKey.MOTION_MOVE_L);
  const stop = useService(ServiceKey.MOTION_STOP);
  const traj = useTopic(Topic.MOTION_STATE_TRAJ);

  const [targetMm, setTargetMm] = useState<Vec3>([0, 0, 0]);
  const [duration, setDuration] = useState(3.0);
  const [error, setError] = useState<string | null>(null);

  const handleSync = useCallback(async () => {
    const res = await tcpSvc.call({});
    if (res.success) {
      const mm = mToMmVec3(res.data.position);
      setTargetMm([
        Math.round(mm[0] * 10) / 10,
        Math.round(mm[1] * 10) / 10,
        Math.round(mm[2] * 10) / 10,
      ]);
      setError(null);
    } else {
      setError("TCP 읽기 실패");
    }
  }, [tcpSvc]);

  const handleExecute = async () => {
    setError(null);
    const positionM = mmToMVec3(targetMm);
    const res = await moveL.call({ position: positionM });
    if (!res.success) setError(res.message || "MoveL 실패");
  };

  const isRunning = traj?.status === "running";
  const progress = Math.round((traj?.progress ?? 0) * 100);
  const tcpPose = tcpSvc.data;

  return (
    <div className="flex flex-col gap-4">
      {tcpPose && (
        <div className="rounded-md bg-muted px-3 py-2 text-xs font-mono">
          <p className="text-muted-foreground mb-1">현재 TCP (mm)</p>
          <div className="grid grid-cols-3 gap-2">
            {mToMmVec3(tcpPose.position).map((v, i) => (
              <div key={AXES[i]}>
                <span className="text-muted-foreground">{AXES[i]}: </span>
                <span>{v.toFixed(1)}</span>
              </div>
            ))}
          </div>
        </div>
      )}

      <div className="flex flex-col gap-2">
        <Label className="text-xs font-medium">목표 위치 (mm)</Label>
        <div className="grid grid-cols-3 gap-2">
          {AXES.map((ax, i) => (
            <div key={ax} className="flex flex-col gap-1">
              <Label className="text-[10px] text-muted-foreground">{ax}</Label>
              <Input
                type="number"
                step={1}
                value={targetMm[i]}
                onChange={(e) => {
                  const num = parseFloat(e.target.value);
                  if (!isNaN(num))
                    setTargetMm((prev) => {
                      const next: Vec3 = [...prev];
                      next[i] = num;
                      return next;
                    });
                }}
                className="h-8 text-xs text-right"
              />
            </div>
          ))}
        </div>
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
              {traj.status === "running" && "직선 이동 중…"}
              {traj.status === "done" && "완료"}
              {traj.status === "failed" && "IK 실패 — 경로 중단"}
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
          onClick={() => void handleSync()}
          disabled={tcpSvc.pending || isRunning}
        >
          {tcpSvc.pending ? "읽는 중…" : "TCP 동기화"}
        </Button>
        <Button
          size="sm"
          className="flex-1"
          onClick={handleExecute}
          disabled={moveL.pending || isRunning}
        >
          {moveL.pending ? "전송 중…" : "실행"}
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
