import { useCallback, useEffect, useRef, useState } from "react";
import * as SliderPrimitive from "@radix-ui/react-slider";
import { PanelButton } from "@/components/shared/PanelButton";
import { useService, useTopic } from "@/framework";
import { ServiceKey, Topic } from "@/constants/topics";
import { mmToMVec3, mToMmVec3 } from "@/lib/robot/utils";
import type { Vector3Tuple } from "three";

const AXES = ["X", "Y", "Z"] as const;

function roundMm(v: number): number {
  return Math.round(v * 100) / 100;
}

export function MoveLControl() {
  const tcpSvc = useService(ServiceKey.MOTION_GET_TCP);
  const moveL = useService(ServiceKey.MOTION_MOVE_L);
  const stop = useService(ServiceKey.MOTION_STOP);
  const traj = useTopic(Topic.MOTION_STATE_TRAJ);

  const [targetMm, setTargetMm] = useState<Vector3Tuple>([0, 0, 0]);
  const [duration, setDuration] = useState(3.0);
  const [error, setError] = useState<string | null>(null);

  const handleSync = useCallback(async () => {
    const res = await tcpSvc.call({});
    if (res.success) {
      const mm = mToMmVec3(res.data.position);
      setTargetMm([roundMm(mm[0]), roundMm(mm[1]), roundMm(mm[2])]);
      setError(null);
    } else {
      setError("TCP 읽기 실패");
    }
  }, [tcpSvc]);

  // 탭 mount 시 1회 자동 sync — target 이 [0,0,0] 으로 reset 되어 사용자가 [실행]
  // 누르면 base origin 으로 큰 동작하는 위험 차단.
  const initRef = useRef(false);
  useEffect(() => {
    if (initRef.current) return;
    initRef.current = true;
    // eslint-disable-next-line react-hooks/set-state-in-effect
    void handleSync();
  }, [handleSync]);

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
    <div className="flex flex-col gap-3">
      {tcpPose && (
        <div className="rounded bg-zinc-900/60 border border-zinc-800/60 px-2.5 py-2 font-mono">
          <p className="text-[9px] uppercase tracking-widest text-zinc-600 mb-1.5">
            현재 TCP (mm)
          </p>
          <div className="grid grid-cols-3 gap-2 text-[10px] tabular-nums">
            {mToMmVec3(tcpPose.position).map((v, i) => (
              <div key={AXES[i]}>
                <span className="text-zinc-500">{AXES[i]}: </span>
                <span className="text-zinc-300">{v.toFixed(2)}</span>
              </div>
            ))}
          </div>
        </div>
      )}

      <div className="flex flex-col gap-1.5">
        <p className="text-[9px] uppercase tracking-widest text-zinc-600 font-mono">
          목표 위치 (mm)
        </p>
        <div className="grid grid-cols-3 gap-2">
          {AXES.map((ax, i) => (
            <div key={ax} className="flex flex-col gap-1">
              <span className="text-[10px] text-zinc-500 font-mono">{ax}</span>
              <input
                type="number"
                step={1}
                value={targetMm[i]}
                onChange={(e) => {
                  const num = parseFloat(e.target.value);
                  if (!isNaN(num))
                    setTargetMm((prev) => {
                      const next: Vector3Tuple = [...prev];
                      next[i] = num;
                      return next;
                    });
                }}
                className="h-7 w-full px-1.5 text-[11px] text-right text-blue-400 tabular-nums font-mono bg-zinc-900 border border-zinc-800 rounded focus:outline-none focus:border-blue-500/60"
              />
            </div>
          ))}
        </div>
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
              {traj.status === "running" && "직선 이동 중…"}
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
          variant="outline"
          onClick={() => void handleSync()}
          disabled={tcpSvc.pending || isRunning}
          className="flex-1"
        >
          {tcpSvc.pending ? "읽는 중…" : "TCP 동기화"}
        </PanelButton>
        <PanelButton
          variant="primary"
          onClick={handleExecute}
          disabled={moveL.pending || isRunning}
          className="flex-1"
        >
          {moveL.pending ? "전송 중…" : "실행"}
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
