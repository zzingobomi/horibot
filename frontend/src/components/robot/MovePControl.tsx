import { useState, useCallback } from "react";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Progress } from "@/components/ui/progress";
import type {
  TCPPose,
  MovePRequest,
  TrajectoryState,
  Vec3,
} from "@/types/motion";
import { mmToMVec3, mToMmVec3 } from "@/lib/robot/utils";

const AXES = ["X", "Y", "Z"] as const;

interface WaypointRow {
  id: number;
  pos: Vec3; // mm
}

let _nextId = 1;

interface Props {
  tcpPose: TCPPose | null;
  trajectoryState: TrajectoryState | null;
  onGetTCP: () => Promise<TCPPose | null>;
  onMoveP: (req: MovePRequest) => Promise<boolean>;
  onStop: () => Promise<void>;
}

export function MovePControl({
  tcpPose,
  trajectoryState,
  onGetTCP,
  onMoveP,
  onStop,
}: Props) {
  const [rows, setRows] = useState<WaypointRow[]>([
    { id: _nextId++, pos: [0, 0, 0] },
    { id: _nextId++, pos: [0, 0, 0] },
  ]);
  const [loading, setLoading] = useState(false);
  const [syncing, setSyncing] = useState<number | null>(null); // 동기화 중인 row id
  const [error, setError] = useState<string | null>(null);

  const handleAxisChange = (id: number, axis: number, value: string) => {
    const num = parseFloat(value);
    if (!isNaN(num)) {
      setRows((prev) =>
        prev.map((r) => {
          if (r.id !== id) return r;
          const next: Vec3 = [...r.pos];
          next[axis] = num;
          return { ...r, pos: next };
        }),
      );
    }
  };

  const addRow = () => {
    const last = rows[rows.length - 1]?.pos ?? [0, 0, 0];
    setRows((prev) => [...prev, { id: _nextId++, pos: [...last] }]);
  };

  const removeRow = (id: number) => {
    setRows((prev) => prev.filter((r) => r.id !== id));
  };

  const syncRow = useCallback(
    async (id: number) => {
      setSyncing(id);
      const pose = await onGetTCP();
      if (pose) {
        const mm = mToMmVec3(pose.position);
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
      setSyncing(null);
    },
    [onGetTCP],
  );

  const handleExecute = async () => {
    if (rows.length < 2) {
      setError("경유점 최소 2개 필요");
      return;
    }

    setLoading(true);
    setError(null);

    const waypoints = rows.map((r) => mmToMVec3(r.pos));

    const ok = await onMoveP({ waypoints });

    if (!ok) setError("MoveP 실패");
    setLoading(false);
  };

  const isRunning = trajectoryState?.status === "running";
  const progress = Math.round((trajectoryState?.progress ?? 0) * 100);

  return (
    <div className="flex flex-col gap-4">
      {/* 현재 TCP 표시 */}
      {tcpPose && (
        <div className="rounded-md bg-muted px-3 py-2 text-xs font-mono">
          <p className="text-muted-foreground mb-1">
            현재 TCP (mm) — 자동 Start
          </p>
          <div className="grid grid-cols-3 gap-2">
            {AXES.map((ax, i) => (
              <div key={ax}>
                <span className="text-muted-foreground">{ax}: </span>
                <span>
                  {tcpPose ? mToMmVec3(tcpPose.position)[i].toFixed(1) : "0.0"}
                </span>
              </div>
            ))}
          </div>
        </div>
      )}

      {/* 경유점 목록 */}
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
                onChange={(e) => handleAxisChange(row.id, i, e.target.value)}
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
                onClick={() => syncRow(row.id)}
                disabled={syncing !== null || isRunning}
              >
                {syncing === row.id ? "…" : "⊕"}
              </Button>
              <Button
                variant="ghost"
                size="sm"
                className="h-7 w-7 p-0 text-destructive"
                onClick={() => removeRow(row.id)}
                disabled={rows.length <= 2 || isRunning}
              >
                ✕
              </Button>
            </div>
          </div>
        ))}
      </div>

      {/* 경유점 추가 */}
      <Button
        variant="outline"
        size="sm"
        onClick={addRow}
        disabled={isRunning}
        className="text-xs"
      >
        + 경유점 추가
      </Button>

      <p className="text-[10px] text-muted-foreground">
        ※ CubicSpline blending — 경유점에서 멈추지 않고 부드럽게 통과
      </p>

      {/* 진행 상황 */}
      {trajectoryState && trajectoryState.status !== "idle" && (
        <div className="flex flex-col gap-1">
          <div className="flex justify-between text-xs text-muted-foreground">
            <span>
              {trajectoryState.status === "running" && "경로 이동 중…"}
              {trajectoryState.status === "done" && "완료"}
              {trajectoryState.status === "failed" && "IK 실패 — 경로 중단"}
              {trajectoryState.status === "stopped" && "중단됨"}
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
          disabled={loading || isRunning || rows.length < 2}
        >
          {loading ? "전송 중…" : "실행"}
        </Button>
        <Button
          variant="destructive"
          size="sm"
          onClick={onStop}
          disabled={!isRunning}
        >
          Stop
        </Button>
      </div>
    </div>
  );
}
