/**
 * JogJ — joint-space velocity jog (frontend_v2.md §3.3 + motion_taxonomy.md §Jog).
 *
 * Wire: `Motion.Stream.JOG_J` topic publish (fire-and-forget, 50Hz). backend
 * JogJCommand 가 ref latch + 실 dt 적분 → URDF rad target → motor cmd.
 *
 * payload `JogJInput { robot_id, velocities }` — robot_id 필수 (Motion wildcard
 * subscribe 후 payload.robot_id self-filter).
 *
 * IDLE_RESET_S (backend) 가 publish 끊긴 후 fresh latch — encoder-ref drift 차단.
 */
import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { bridge } from "@/api/bridge";
import { useStream, useCapability } from "@/framework";
import {
  MotorKind,
  ServiceKey,
  Topic,
  type JogJInput,
} from "@/api/generated/contract";

const PUBLISH_DT_MS = 20; // 50Hz
const JOINT_VEL_MAX = 0.6; // rad/s

interface JogJProps {
  robotId: string;
}

export function JogJ({ robotId }: JogJProps) {
  const cap = useCapability(ServiceKey.MOTOR_GET_TOPOLOGY, { robotId });
  const armMotors = useMemo(
    () =>
      (cap.value?.motors ?? [])
        .filter((m) => m.kind === MotorKind.JOINT)
        .sort((a, b) => a.id - b.id),
    [cap.value],
  );

  const tcp = useStream(Topic.MOTION_TCP_STATE, { robotId });
  const currentRads = tcp.value?.joints ?? [];

  const [jog, setJog] = useState<{ idx: number; dir: 1 | -1 } | null>(null);
  const [scale, setScale] = useState(0.3);

  const intervalRef = useRef<number | null>(null);
  // jog / scale 을 ref 로 stash — interval closure 가 stale 안 보게.
  const stateRef = useRef<{
    jog: { idx: number; dir: 1 | -1 } | null;
    scale: number;
  }>({ jog: null, scale: 0.3 });
  useEffect(() => {
    stateRef.current = { jog, scale };
  });

  const stopJog = useCallback(() => {
    console.log("[JogJ] stopJog called", new Error().stack?.split("\n")[2]);
    if (intervalRef.current !== null) {
      window.clearInterval(intervalRef.current);
      intervalRef.current = null;
    }
    setJog(null);
  }, []);

  const startJog = useCallback(
    (idx: number, dir: 1 | -1) => {
      const dof = armMotors.length;
      if (dof === 0) return;
      setJog({ idx, dir });
      // stateRef 직접 update — setJog 의 async render 전에 interval 첫 fire 가
      // 옛 stateRef.jog=null 박혀있는 동안 skip 박지 않게 (race fix).
      stateRef.current.jog = { idx, dir };
      if (intervalRef.current !== null) window.clearInterval(intervalRef.current);
      let pubCount = 0;
      intervalRef.current = window.setInterval(() => {
        const s = stateRef.current;
        if (s.jog === null) {
          console.log(`[JogJ] tick ${pubCount++} skip (jog=null)`);
          return;
        }
        const velocities = Array.from({ length: dof }, (_, i) =>
          i === s.jog!.idx ? s.jog!.dir * s.scale * JOINT_VEL_MAX : 0,
        );
        const payload: JogJInput = { robot_id: robotId, velocities };
        bridge.publish(Topic.MOTION_JOG_J, payload, robotId);
        console.log(`[JogJ] tick ${pubCount++} publish v[0]=${velocities[0]}`);
      }, PUBLISH_DT_MS);
    },
    [armMotors.length, robotId],
  );

  // unmount cleanup
  useEffect(() => {
    return () => {
      if (intervalRef.current !== null) {
        window.clearInterval(intervalRef.current);
        intervalRef.current = null;
      }
    };
  }, []);

  // deadman — pointer release / window blur
  useEffect(() => {
    if (jog === null) return;
    const handler = (e: Event) => {
      const pe = e as PointerEvent;
      const tgt = pe.target as Element | null;
      console.log(
        `[JogJ] deadman fire: type=${e.type} isTrusted=${pe.isTrusted} target=${tgt?.tagName ?? "?"}.${tgt?.className?.toString().slice(0, 40) ?? ""} pointerId=${pe.pointerId}`,
      );
      stopJog();
    };
    window.addEventListener("pointerup", handler);
    window.addEventListener("pointercancel", handler);
    window.addEventListener("blur", handler);
    return () => {
      window.removeEventListener("pointerup", handler);
      window.removeEventListener("pointercancel", handler);
      window.removeEventListener("blur", handler);
    };
  }, [jog, stopJog]);

  return (
    <div className="flex flex-col gap-3">
      <p className="text-[10px] text-zinc-500 font-mono leading-relaxed">
        버튼 hold = joint velocity publish (50Hz). backend latch + dt 적분 → URDF
        rad target. cross-process safe.
      </p>

      <div className="flex flex-col gap-1.5">
        {armMotors.map((m, i) => {
          const currentRad = currentRads[i] ?? 0;
          const currentDeg = (currentRad * 180) / Math.PI;
          const isMinus = jog?.idx === i && jog.dir === -1;
          const isPlus = jog?.idx === i && jog.dir === 1;

          return (
            <div key={m.id} className="flex items-center gap-2 font-mono">
              <div className="w-14 shrink-0">
                <div className="text-[11px] text-zinc-300">J{i + 1}</div>
                <div className="text-[9px] text-zinc-600 tabular-nums">
                  {currentDeg.toFixed(1)}°
                </div>
              </div>
              <button
                onPointerDown={(e) => {
                  e.preventDefault();
                  // setPointerCapture — Chromium 이 button class 변경 / hit target
                  // 변동 시 자동 pointercancel → pointerup promote 차단. 사용자가
                  // 실 hardware 박을 때 빠른 손가락 + 누른 채 다른 button 위로
                  // 드래그 박는 시나리오도 동일 fix.
                  e.currentTarget.setPointerCapture(e.pointerId);
                  startJog(i, -1);
                }}
                className={`flex-1 h-7 rounded border text-[11px] font-mono uppercase tracking-wide transition-colors ${
                  isMinus
                    ? "bg-emerald-500/30 border-emerald-500/60 text-emerald-200"
                    : "bg-zinc-900 border-zinc-800 text-zinc-400 hover:bg-zinc-800/60 hover:text-zinc-100"
                }`}
              >
                −
              </button>
              <button
                onPointerDown={(e) => {
                  e.preventDefault();
                  e.currentTarget.setPointerCapture(e.pointerId);
                  startJog(i, 1);
                }}
                className={`flex-1 h-7 rounded border text-[11px] font-mono uppercase tracking-wide transition-colors ${
                  isPlus
                    ? "bg-emerald-500/30 border-emerald-500/60 text-emerald-200"
                    : "bg-zinc-900 border-zinc-800 text-zinc-400 hover:bg-zinc-800/60 hover:text-zinc-100"
                }`}
              >
                +
              </button>
            </div>
          );
        })}
      </div>

      <div className="flex items-center gap-2 font-mono">
        <span className="text-[10px] uppercase tracking-wide text-zinc-500 w-14 shrink-0">
          속도
        </span>
        <input
          type="range"
          min={0.1}
          max={1.0}
          step={0.05}
          value={scale}
          onChange={(e) => setScale(parseFloat(e.target.value))}
          className="flex-1 accent-emerald-400"
        />
        <span className="text-[10px] text-zinc-400 tabular-nums w-16 text-right">
          {(scale * JOINT_VEL_MAX).toFixed(2)} rad/s
        </span>
      </div>
    </div>
  );
}
