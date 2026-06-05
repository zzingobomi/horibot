import { useCallback, useState } from "react";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Progress } from "@/components/ui/progress";
import { useService, useTopic } from "@/framework";
import { ServiceKey, Topic } from "@/constants/topics";
import { mmToMVec3, mToMmVec3 } from "@/lib/robot/utils";
import type { Vector3Tuple } from "three";

const AXES = ["X", "Y", "Z"] as const;

interface WaypointRow {
  id: number;
  pos: Vector3Tuple; // mm
}

let _nextId = 1;

export function MovePControl() {
  const tcpSvc = useService(ServiceKey.MOTION_GET_TCP);
  const moveP = useService(ServiceKey.MOTION_MOVE_P);
  const stop = useService(ServiceKey.MOTION_STOP);
  const traj = useTopic(Topic.MOTION_STATE_TRAJ);

  const [rows, setRows] = useState<WaypointRow[]>([
    { id: _nextId++, pos: [0, 0, 0] },
    { id: _nextId++, pos: [0, 0, 0] },
  ]);
  const [syncingId, setSyncingId] = useState<number | null>(null);
  const [error, setError] = useState<string | null>(null);

  const syncRow = useCallback(
    async (id: number) => {
      setSyncingId(id);
      const res = await tcpSvc.call({});
      if (res.success) {
        const mm = mToMmVec3(res.data.position);
        setRows((prev) =>
          prev.map((r) =>
            r.id !== id
              ? r
              : {
                  ...r,
                  pos: [
                    Math.round(mm[0] * 10) / 10,
                    Math.round(mm[1] * 10) / 10,
                    Math.round(mm[2] * 10) / 10,
                  ],
                },
          ),
        );
        setError(null);
      } else {
        setError("TCP 읽기 실패");
      }
      setSyncingId(null);
    },
    [tcpSvc],
  );

  const handleExecute = async () => {
    if (rows.length < 2) {
      setError("경유점 최소 2개 필요");
      return;
    }
    setError(null);
    const waypoints = rows.map((r) => mmToMVec3(r.pos));
    const res = await moveP.call({ waypoints });
    if (!res.success) setError(res.message || "MoveP 실패");
  };

  const isRunning = traj?.status === "running";
  const progress = Math.round((traj?.progress ?? 0) * 100);
  const tcpPose = tcpSvc.data;

  return (
    <div className="flex flex-col gap-4">
      {tcpPose && (
        <div className="rounded-md bg-muted px-3 py-2 text-xs font-mono">
          <p className="text-muted-foreground mb-1">
            현재 TCP (mm) — 자동 Start
          </p>
          <div className="grid grid-cols-3 gap-2">
            {AXES.map((ax, i) => (
              <div key={ax}>
                <span className="text-muted-foreground">{ax}: </span>
                <span>{mToMmVec3(tcpPose.position)[i].toFixed(1)}</span>
              </div>
            ))}
          </div>
        </div>
      )}

      <div className="flex flex-col gap-2">
        <div className="grid grid-cols-[20px_1fr_1fr_1fr_64px] items-center gap-1 px-1">
          <span />
          {AXES.map((ax) => (
            <Label
              key={ax}
              className="text-[10px] text-muted-foreground text-center"
            >
              {ax} (mm)
            </Label>
          ))}
          <span />
        </div>

        {rows.map((row, idx) => (
          <div
            key={row.id}
            className="grid grid-cols-[20px_1fr_1fr_1fr_64px] items-center gap-1"
          >
            <span className="text-[10px] text-muted-foreground text-right">
              {idx + 1}
            </span>
            {AXES.map((_, i) => (
              <Input
                key={i}
                type="number"
                step={1}
                value={row.pos[i]}
                onChange={(e) => {
                  const num = parseFloat(e.target.value);
                  if (!isNaN(num))
                    setRows((prev) =>
                      prev.map((r) => {
                        if (r.id !== row.id) return r;
                        const next: Vector3Tuple = [...r.pos];
                        next[i] = num;
                        return { ...r, pos: next };
                      }),
                    );
                }}
                className="h-7 text-xs text-right px-1.5"
                disabled={isRunning}
              />
            ))}
            <div className="flex gap-0.5">
              <Button
                variant="ghost"
                size="sm"
                className="h-7 w-7 p-0 text-[10px]"
                title="현재 TCP 복사"
                onClick={() => void syncRow(row.id)}
                disabled={syncingId !== null || isRunning}
              >
                {syncingId === row.id ? "…" : "⊕"}
              </Button>
              <Button
                variant="ghost"
                size="sm"
                className="h-7 w-7 p-0 text-destructive"
                onClick={() =>
                  setRows((prev) => prev.filter((r) => r.id !== row.id))
                }
                disabled={rows.length <= 2 || isRunning}
              >
                ✕
              </Button>
            </div>
          </div>
        ))}
      </div>

      <Button
        variant="outline"
        size="sm"
        onClick={() =>
          setRows((prev) => [
            ...prev,
            {
              id: _nextId++,
              pos: [...(prev[prev.length - 1]?.pos ?? [0, 0, 0])] as Vector3Tuple,
            },
          ])
        }
        disabled={isRunning}
        className="text-xs"
      >
        + 경유점 추가
      </Button>

      <p className="text-[10px] text-muted-foreground">
        ※ CubicSpline blending — 경유점에서 멈추지 않고 부드럽게 통과
      </p>

      {traj && traj.status !== "idle" && (
        <div className="flex flex-col gap-1">
          <div className="flex justify-between text-xs text-muted-foreground">
            <span>
              {traj.status === "running" && "경로 이동 중…"}
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
          disabled={moveP.pending || isRunning || rows.length < 2}
        >
          {moveP.pending ? "전송 중…" : "실행"}
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
