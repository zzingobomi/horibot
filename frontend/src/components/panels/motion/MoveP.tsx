import { useCallback, useState } from "react";
import { PanelButton } from "@/components/shared/PanelButton";
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
    <div className="flex flex-col gap-3">
      {tcpPose && (
        <div className="rounded bg-zinc-900/60 border border-zinc-800/60 px-2.5 py-2 font-mono">
          <p className="text-[9px] uppercase tracking-widest text-zinc-600 mb-1.5">
            현재 TCP (mm) — 자동 Start
          </p>
          <div className="grid grid-cols-3 gap-2 text-[10px] tabular-nums">
            {AXES.map((ax, i) => (
              <div key={ax}>
                <span className="text-zinc-500">{ax}: </span>
                <span className="text-zinc-300">
                  {mToMmVec3(tcpPose.position)[i].toFixed(1)}
                </span>
              </div>
            ))}
          </div>
        </div>
      )}

      <div className="flex flex-col gap-1.5">
        <div className="grid grid-cols-[20px_1fr_1fr_1fr_56px] items-center gap-1 px-1">
          <span />
          {AXES.map((ax) => (
            <span
              key={ax}
              className="text-[9px] uppercase tracking-widest text-zinc-600 text-center font-mono"
            >
              {ax} (mm)
            </span>
          ))}
          <span />
        </div>

        {rows.map((row, idx) => (
          <div
            key={row.id}
            className="grid grid-cols-[20px_1fr_1fr_1fr_56px] items-center gap-1"
          >
            <span className="text-[10px] text-zinc-600 text-right font-mono tabular-nums">
              {idx + 1}
            </span>
            {AXES.map((_, i) => (
              <input
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
                className="h-7 w-full px-1.5 text-[10px] text-right text-blue-400 tabular-nums font-mono bg-zinc-900 border border-zinc-800 rounded focus:outline-none focus:border-blue-500/60 disabled:opacity-50"
                disabled={isRunning}
              />
            ))}
            <div className="flex gap-0.5 justify-end">
              <PanelButton
                variant="ghost"
                title="현재 TCP 복사"
                onClick={() => void syncRow(row.id)}
                disabled={syncingId !== null || isRunning}
                className="!h-7 !w-7 !p-0"
              >
                {syncingId === row.id ? "…" : "⊕"}
              </PanelButton>
              <PanelButton
                variant="ghost"
                onClick={() =>
                  setRows((prev) => prev.filter((r) => r.id !== row.id))
                }
                disabled={rows.length <= 2 || isRunning}
                className="!h-7 !w-7 !p-0 hover:!text-red-400"
              >
                ✕
              </PanelButton>
            </div>
          </div>
        ))}
      </div>

      <PanelButton
        variant="outline"
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
      >
        + 경유점 추가
      </PanelButton>

      <p className="text-[10px] text-zinc-500 font-mono">
        ※ CubicSpline blending — 경유점에서 멈추지 않고 부드럽게 통과
      </p>

      {traj && traj.status !== "idle" && (
        <div className="flex flex-col gap-1 font-mono">
          <div className="flex justify-between text-[10px] text-zinc-500">
            <span>
              {traj.status === "running" && "경로 이동 중…"}
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
          disabled={moveP.pending || isRunning || rows.length < 2}
          className="flex-1"
        >
          {moveP.pending ? "전송 중…" : "실행"}
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
