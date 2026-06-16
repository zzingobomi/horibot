/**
 * SpeedTcp Jog — TCP twist 추종 (산업 펜던트 cartesian jog 패턴).
 *
 * 버튼 hold 동안 50Hz 로 `MOTION_SPEED_TCP` publish → backend `_velocity_loop` 가
 * Jacobian pseudo-inverse + Ruckig velocity mode + jerk-limited 추종. 손 떼면
 * publish 멈춤 → backend 100ms deadman timeout 자동 정지.
 *
 * 5DOF robot (OMX-F) 자리는 angular 무시 (server side linear-only fallback).
 */
import { useCallback, useEffect, useRef, useState } from "react";
import { useParams } from "react-router-dom";
import * as SliderPrimitive from "@radix-ui/react-slider";
import { PanelButton } from "@/components/shared/PanelButton";
import { useService } from "@/framework";
import { ServiceKey } from "@/constants/topics";

const PUBLISH_DT_MS = 20; // 50Hz
const TCP_LINEAR_MAX = 0.08; // m/s — 게임패드 GamepadNode 와 동일.
const TCP_ANGULAR_MAX = 0.8; // rad/s

type LinearAxis = "X" | "Y" | "Z";
type AngularAxis = "Rx" | "Ry" | "Rz";
type AxisKey = LinearAxis | AngularAxis;
type Dir = 1 | -1;

interface JogState {
  axis: AxisKey;
  dir: Dir;
}

function isAngular(axis: AxisKey): axis is AngularAxis {
  return axis.startsWith("R");
}

function buildTwist(
  jog: JogState,
  linearScale: number,
  angularScale: number,
): { linear: number[]; angular: number[] } {
  const linear = [0, 0, 0];
  const angular = [0, 0, 0];
  if (!isAngular(jog.axis)) {
    const idx = { X: 0, Y: 1, Z: 2 }[jog.axis];
    linear[idx] = jog.dir * linearScale * TCP_LINEAR_MAX;
  } else {
    const idx = { Rx: 0, Ry: 1, Rz: 2 }[jog.axis];
    angular[idx] = jog.dir * angularScale * TCP_ANGULAR_MAX;
  }
  return { linear, angular };
}

export function SpeedTcpControl() {
  const { id: robotId = "" } = useParams<{ id: string }>();
  const speedTcp = useService(ServiceKey.MOTION_SPEED_TCP, robotId);

  const [jogState, setJogState] = useState<JogState | null>(null);
  const [frame, setFrame] = useState<"base" | "tcp">("base");
  const [linearScale, setLinearScale] = useState(0.4);
  const [angularScale, setAngularScale] = useState(0.3);

  const intervalRef = useRef<number | null>(null);
  const stateRef = useRef<{
    jog: JogState | null;
    linear: number;
    angular: number;
    frame: "base" | "tcp";
  }>({ jog: null, linear: 0.4, angular: 0.3, frame: "base" });
  useEffect(() => {
    stateRef.current = {
      jog: jogState,
      linear: linearScale,
      angular: angularScale,
      frame,
    };
  });

  const stopJog = useCallback(() => {
    if (intervalRef.current !== null) {
      window.clearInterval(intervalRef.current);
      intervalRef.current = null;
    }
    setJogState(null);
    // 명시적 stop publish — backend deadman timeout (100ms) 기다리지 말고
    // 즉시 target=0 → joint Ruckig jerk-limited 감속. button up event 가
    // backend 로 *명시*되니 짧은 burst 자리 ramp-down 시작 시점 정확.
    void speedTcp.call({
      linear: [0, 0, 0],
      angular: [0, 0, 0],
      frame: stateRef.current.frame,
    });
  }, [speedTcp]);

  const startJog = useCallback(
    (axis: AxisKey, dir: Dir) => {
      setJogState({ axis, dir });
      if (intervalRef.current !== null) {
        window.clearInterval(intervalRef.current);
      }
      intervalRef.current = window.setInterval(() => {
        const s = stateRef.current;
        if (s.jog === null) return;
        const twist = buildTwist(s.jog, s.linear, s.angular);
        void speedTcp.call({ ...twist, frame: s.frame });
      }, PUBLISH_DT_MS);
    },
    [speedTcp],
  );

  useEffect(() => {
    return () => {
      if (intervalRef.current !== null) {
        window.clearInterval(intervalRef.current);
        intervalRef.current = null;
      }
    };
  }, []);

  useEffect(() => {
    if (jogState === null) return;
    const handler = () => stopJog();
    window.addEventListener("pointerup", handler);
    window.addEventListener("pointercancel", handler);
    return () => {
      window.removeEventListener("pointerup", handler);
      window.removeEventListener("pointercancel", handler);
    };
  }, [jogState, stopJog]);

  const renderAxisPair = (axis: AxisKey, label: string) => {
    const isMinus = jogState?.axis === axis && jogState.dir === -1;
    const isPlus = jogState?.axis === axis && jogState.dir === 1;
    return (
      <div key={axis} className="flex items-center gap-1.5 font-mono">
        <div className="w-7 text-[10px] text-zinc-500 text-right">{label}</div>
        <button
          onPointerDown={(e) => {
            e.preventDefault();
            startJog(axis, -1);
          }}
          className={`flex-1 h-7 rounded border text-[11px] uppercase tracking-wide transition-colors ${
            isMinus
              ? "bg-amber-500/30 border-amber-500/60 text-amber-200"
              : "bg-zinc-900 border-zinc-800 text-zinc-400 hover:bg-zinc-800/60 hover:text-zinc-100"
          }`}
        >
          −
        </button>
        <button
          onPointerDown={(e) => {
            e.preventDefault();
            startJog(axis, 1);
          }}
          className={`flex-1 h-7 rounded border text-[11px] uppercase tracking-wide transition-colors ${
            isPlus
              ? "bg-amber-500/30 border-amber-500/60 text-amber-200"
              : "bg-zinc-900 border-zinc-800 text-zinc-400 hover:bg-zinc-800/60 hover:text-zinc-100"
          }`}
        >
          +
        </button>
      </div>
    );
  };

  return (
    <div className="flex flex-col gap-3">
      <div className="flex items-center justify-between">
        <p className="text-[10px] text-zinc-500 font-mono leading-relaxed">
          ※ 버튼 hold = TCP twist jog. 5DOF 자리는 angular 무시.
        </p>
        <div className="flex items-center gap-1">
          <span className="text-[9px] uppercase tracking-widest text-zinc-600 font-mono">
            frame
          </span>
          <PanelButton
            variant={frame === "base" ? "primary" : "outline"}
            onClick={() => setFrame("base")}
            className="!px-2 !py-0.5 !text-[10px]"
          >
            base
          </PanelButton>
          <PanelButton
            variant={frame === "tcp" ? "primary" : "outline"}
            onClick={() => setFrame("tcp")}
            className="!px-2 !py-0.5 !text-[10px]"
          >
            tcp
          </PanelButton>
        </div>
      </div>

      <div className="flex flex-col gap-1.5">
        <p className="text-[9px] uppercase tracking-widest text-zinc-600 font-mono">
          Linear (m/s)
        </p>
        {(["X", "Y", "Z"] as LinearAxis[]).map((a) => renderAxisPair(a, a))}
      </div>

      <div className="flex items-center gap-2 font-mono">
        <span className="text-[10px] uppercase tracking-wide text-zinc-500 w-14 shrink-0">
          속도 L
        </span>
        <SliderPrimitive.Root
          className="relative flex items-center select-none touch-none flex-1 h-4"
          min={0.1}
          max={1.0}
          step={0.05}
          value={[linearScale]}
          onValueChange={(v) => setLinearScale(v[0])}
        >
          <SliderPrimitive.Track className="relative h-1 w-full grow rounded-full bg-zinc-800">
            <SliderPrimitive.Range className="absolute h-full rounded-full bg-amber-500/40" />
          </SliderPrimitive.Track>
          <SliderPrimitive.Thumb className="block h-3.5 w-3.5 rounded-full border border-amber-400 bg-zinc-900 shadow-sm focus-visible:outline-none focus-visible:ring-1 focus-visible:ring-amber-400" />
        </SliderPrimitive.Root>
        <span className="text-[10px] text-zinc-400 tabular-nums w-16 text-right">
          {(linearScale * TCP_LINEAR_MAX * 1000).toFixed(0)} mm/s
        </span>
      </div>

      <div className="flex flex-col gap-1.5">
        <p className="text-[9px] uppercase tracking-widest text-zinc-600 font-mono">
          Angular (rad/s) — 6DOF only
        </p>
        {(["Rx", "Ry", "Rz"] as AngularAxis[]).map((a) => renderAxisPair(a, a))}
      </div>

      <div className="flex items-center gap-2 font-mono">
        <span className="text-[10px] uppercase tracking-wide text-zinc-500 w-14 shrink-0">
          속도 A
        </span>
        <SliderPrimitive.Root
          className="relative flex items-center select-none touch-none flex-1 h-4"
          min={0.1}
          max={1.0}
          step={0.05}
          value={[angularScale]}
          onValueChange={(v) => setAngularScale(v[0])}
        >
          <SliderPrimitive.Track className="relative h-1 w-full grow rounded-full bg-zinc-800">
            <SliderPrimitive.Range className="absolute h-full rounded-full bg-amber-500/40" />
          </SliderPrimitive.Track>
          <SliderPrimitive.Thumb className="block h-3.5 w-3.5 rounded-full border border-amber-400 bg-zinc-900 shadow-sm focus-visible:outline-none focus-visible:ring-1 focus-visible:ring-amber-400" />
        </SliderPrimitive.Root>
        <span className="text-[10px] text-zinc-400 tabular-nums w-16 text-right">
          {(angularScale * TCP_ANGULAR_MAX).toFixed(2)} rad/s
        </span>
      </div>
    </div>
  );
}
