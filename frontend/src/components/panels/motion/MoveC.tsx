import { useCallback, useState } from "react";
import { PanelButton } from "@/components/shared/PanelButton";
import { useService, useTopic } from "@/framework";
import { ServiceKey, Topic } from "@/constants/topics";
import { mmToMVec3, mToMmVec3 } from "@/lib/robot/utils";
import type { Vector3Tuple } from "three";

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

  const [points, setPoints] = useState<Record<PointKey, Vector3Tuple>>({
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
    <div className="flex flex-col gap-3">
      {tcpMm && (
        <div className="rounded bg-zinc-900/60 border border-zinc-800/60 px-2.5 py-2 font-mono">
          <p className="text-[9px] uppercase tracking-widest text-zinc-600 mb-1.5">
            현재 TCP (mm) — Start
          </p>
          <div className="grid grid-cols-3 gap-2 text-[10px] tabular-nums">
            {AXES.map((ax, i) => (
              <div key={ax}>
                <span className="text-zinc-500">{ax}: </span>
                <span className="text-zinc-300">{tcpMm[i].toFixed(1)}</span>
              </div>
            ))}
          </div>
        </div>
      )}

      {(["via", "end"] as PointKey[]).map((key) => (
        <div key={key} className="flex flex-col gap-1.5">
          <div className="flex items-center justify-between">
            <p className="text-[9px] uppercase tracking-widest text-zinc-600 font-mono">
              {POINT_LABELS[key]} (mm)
            </p>
            <PanelButton
              variant="ghost"
              onClick={() => void syncToPoint(key)}
              disabled={tcpSvc.pending || isRunning}
              className="!px-2 !py-0.5 !text-[10px]"
            >
              TCP 복사
            </PanelButton>
          </div>
          <div className="grid grid-cols-3 gap-2">
            {AXES.map((ax, i) => (
              <div key={ax} className="flex flex-col gap-1">
                <span className="text-[10px] text-zinc-500 font-mono">
                  {ax}
                </span>
                <input
                  type="number"
                  step={1}
                  value={points[key][i]}
                  onChange={(e) => {
                    const num = parseFloat(e.target.value);
                    if (!isNaN(num))
                      setPoints((prev) => {
                        const next: Vector3Tuple = [...prev[key]];
                        next[i] = num;
                        return { ...prev, [key]: next };
                      });
                  }}
                  className="h-7 w-full px-1.5 text-[11px] text-right text-blue-400 tabular-nums font-mono bg-zinc-900 border border-zinc-800 rounded focus:outline-none focus:border-blue-500/60"
                />
              </div>
            ))}
          </div>
        </div>
      ))}

      <p className="text-[10px] text-zinc-500 font-mono">
        ※ 현재 TCP(Start) → Via → End 순서로 원호 이동
      </p>

      {traj && traj.status !== "idle" && (
        <div className="flex flex-col gap-1 font-mono">
          <div className="flex justify-between text-[10px] text-zinc-500">
            <span>
              {traj.status === "running" && "원호 이동 중…"}
              {traj.status === "done" && "완료"}
              {traj.status === "failed" && "IK 실패 — 경로 중단"}
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
          variant="primary"
          onClick={handleExecute}
          disabled={moveC.pending || isRunning}
          className="flex-1"
        >
          {moveC.pending ? "전송 중…" : "실행"}
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
