import { useCallback, useState } from "react";
import type {
  TCPPose,
  MoveCRequest,
  TrajectoryState,
  Vec3,
} from "@/types/motion";
import { mmToMVec3, mToMmVec3 } from "@/lib/robot/utils";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Progress } from "@/components/ui/progress";

type PointKey = "via" | "end";

const POINT_LABELS: Record<PointKey, string> = {
  via: "경유점 (Via)",
  end: "끝점 (End)",
};

const AXES = ["X", "Y", "Z"] as const;

interface Props {
  tcpPose: TCPPose | null;
  trajectoryState: TrajectoryState | null;
  onGetTCP: () => Promise<TCPPose | null>;
  onMoveC: (req: MoveCRequest) => Promise<boolean>;
  onStop: () => Promise<void>;
}

export function MoveCControl({
  tcpPose,
  trajectoryState,
  onGetTCP,
  onMoveC,
  onStop,
}: Props) {
  const [points, setPoints] = useState<Record<PointKey, Vec3>>({
    via: [0, 0, 0],
    end: [0, 0, 0],
  });
  const [loading, setLoading] = useState(false);
  const [syncing, setSyncing] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const handleAxisChange = (key: PointKey, axis: number, value: string) => {
    const num = parseFloat(value);
    if (!isNaN(num)) {
      setPoints((prev) => {
        const next: Vec3 = [...prev[key]];
        next[axis] = num;
        return { ...prev, [key]: next };
      });
    }
  };

  const syncToPoint = useCallback(
    async (key: PointKey) => {
      setSyncing(true);
      const pose = await onGetTCP();
      if (pose) {
        const mm = mToMmVec3(pose.position);
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
      setSyncing(false);
    },
    [onGetTCP],
  );

  const handleExecute = async () => {
    setLoading(true);
    setError(null);

    const ok = await onMoveC({
      via: mmToMVec3(points.via),
      end: mmToMVec3(points.end),
    });

    if (!ok) setError("MoveC 실패");
    setLoading(false);
  };

  const isRunning = trajectoryState?.status === "running";
  const progress = Math.round((trajectoryState?.progress ?? 0) * 100);
  const tcpMm = tcpPose ? mToMmVec3(tcpPose.position) : null;

  return (
    <div className="flex flex-col gap-4">
      {/* 현재 TCP */}
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

      {/* Via / End 입력 */}
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
              onClick={() => syncToPoint(key)}
              disabled={syncing || isRunning}
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
                  onChange={(e) => handleAxisChange(key, i, e.target.value)}
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

      {/* 진행 상황 */}
      {trajectoryState && trajectoryState.status !== "idle" && (
        <div className="flex flex-col gap-1">
          <div className="flex justify-between text-xs text-muted-foreground">
            <span>
              {trajectoryState.status === "running" && "원호 이동 중…"}
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
          disabled={loading || isRunning}
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
