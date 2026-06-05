import { useCallback, useState } from "react";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Progress } from "@/components/ui/progress";
import { useService, useTopic } from "@/framework";
import { ServiceKey, Topic } from "@/constants/topics";
import { mmToMVec3, mToMmVec3 } from "@/lib/robot/utils";
import type { Vec3 } from "@/types/motion";

type PointKey = "via" | "end";

const POINT_LABELS: Record<PointKey, string> = {
  via: "경유점 (Via)",
  end: "끝점 (End)",
};

const AXES = ["X", "Y", "Z"] as const;

export function MoveCControl() {
  const tcpSvc = useService(ServiceKey.MOTION_GET_TCP);
  const moveC = useService(ServiceKey.MOTION_MOVE_C);
  const stop = useService(ServiceKey.MOTION_STOP);
  const traj = useTopic(Topic.MOTION_STATE_TRAJ);

  const [points, setPoints] = useState<Record<PointKey, Vec3>>({
    via: [0, 0, 0],
    end: [0, 0, 0],
  });
  const [error, setError] = useState<string | null>(null);

  const syncToPoint = useCallback(
    async (key: PointKey) => {
      const res = await tcpSvc.call({});
      if (res.success) {
        const mm = mToMmVec3(res.data.position);
        setPoints((prev) => ({
          ...prev,
          [key]: [
            Math.round(mm[0] * 10) / 10,
            Math.round(mm[1] * 10) / 10,
            Math.round(mm[2] * 10) / 10,
          ],
        }));
        setError(null);
      } else {
        setError("TCP 읽기 실패");
      }
    },
    [tcpSvc],
  );

  const handleExecute = async () => {
    setError(null);
    const res = await moveC.call({
      via: mmToMVec3(points.via),
      end: mmToMVec3(points.end),
    });
    if (!res.success) setError(res.message || "MoveC 실패");
  };

  const isRunning = traj?.status === "running";
  const progress = Math.round((traj?.progress ?? 0) * 100);
  const tcpMm = tcpSvc.data ? mToMmVec3(tcpSvc.data.position) : null;

  return (
    <div className="flex flex-col gap-4">
      {tcpMm && (
        <div className="rounded-md bg-muted px-3 py-2 text-xs font-mono">
          <p className="text-muted-foreground mb-1">현재 TCP (mm) — Start</p>
          <div className="grid grid-cols-3 gap-2">
            {AXES.map((ax, i) => (
              <div key={ax}>
                <span className="text-muted-foreground">{ax}: </span>
                <span>{tcpMm[i].toFixed(1)}</span>
              </div>
            ))}
          </div>
        </div>
      )}

      {(["via", "end"] as PointKey[]).map((key) => (
        <div key={key} className="flex flex-col gap-2">
          <div className="flex items-center justify-between">
            <Label className="text-xs font-medium">
              {POINT_LABELS[key]} (mm)
            </Label>
            <Button
              variant="ghost"
              size="sm"
              className="h-6 text-[10px] px-2"
              onClick={() => void syncToPoint(key)}
              disabled={tcpSvc.pending || isRunning}
            >
              TCP 복사
            </Button>
          </div>
          <div className="grid grid-cols-3 gap-2">
            {AXES.map((ax, i) => (
              <div key={ax} className="flex flex-col gap-1">
                <Label className="text-[10px] text-muted-foreground">
                  {ax}
                </Label>
                <Input
                  type="number"
                  step={1}
                  value={points[key][i]}
                  onChange={(e) => {
                    const num = parseFloat(e.target.value);
                    if (!isNaN(num))
                      setPoints((prev) => {
                        const next: Vec3 = [...prev[key]];
                        next[i] = num;
                        return { ...prev, [key]: next };
                      });
                  }}
                  className="h-8 text-xs text-right"
                />
              </div>
            ))}
          </div>
        </div>
      ))}

      <p className="text-[10px] text-muted-foreground">
        ※ 현재 TCP(Start) → Via → End 순서로 원호 이동
      </p>

      {traj && traj.status !== "idle" && (
        <div className="flex flex-col gap-1">
          <div className="flex justify-between text-xs text-muted-foreground">
            <span>
              {traj.status === "running" && "원호 이동 중…"}
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
          size="sm"
          className="flex-1"
          onClick={handleExecute}
          disabled={moveC.pending || isRunning}
        >
          {moveC.pending ? "전송 중…" : "실행"}
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
