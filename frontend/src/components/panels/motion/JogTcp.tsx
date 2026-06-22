/**
 * JogTcp — Cartesian human velocity jog (motion_taxonomy.md §Jog).
 *
 * Wire 자리 `MOTION_JOG_TCP_STREAM` topic publish (fire-and-forget, 50Hz). backend
 * JogTcpCommand 가 자기 process 의 joint_cache → fk 로 URDF EE pose fresh latch
 * + 실 측정 dt 자리 SE(3) 적분 → IK → publish_cmd.
 *
 * SE(3) 적분 자리 backend SSOT (scipy.spatial.transform.Rotation) — frontend
 * Three.js / Python gamepad 자리 중복 회피. 동일 wire 자리 gamepad 자리도 사용.
 *
 * 5DOF (OMX-F) 자리 angular 무시 (backend IK 가 position-only fallback).
 * IDLE_RESET_S (backend) 자리 publish 끊긴 후 자동 fresh latch.
 */
import { useCallback, useEffect, useRef, useState } from "react";
import { useParams } from "react-router-dom";
import * as SliderPrimitive from "@radix-ui/react-slider";
import { PanelButton } from "@/components/shared/PanelButton";
import { Topic } from "@/constants/topics";
import { bridge } from "@/api/bridge";

const PUBLISH_DT_MS = 20; // 50Hz
const TCP_LINEAR_MAX = 0.08; // m/s — gamepad 자리 와 같은 권장치
const TCP_ANGULAR_MAX = 0.8; // rad/s

type LinearAxis = "X" | "Y" | "Z";
type AngularAxis = "Rx" | "Ry" | "Rz";
type AxisKey = LinearAxis | AngularAxis;
type Dir = 1 | -1;
type Frame = "base" | "tcp";

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

export function JogTcpControl() {
  const { id: robotId = "" } = useParams<{ id: string }>();

  const [jogState, setJogState] = useState<JogState | null>(null);
  const [frame, setFrame] = useState<Frame>("base");
  const [linearScale, setLinearScale] = useState(0.4);
  const [angularScale, setAngularScale] = useState(0.3);

  const intervalRef = useRef<number | null>(null);
  const stateRef = useRef<{
    jog: JogState | null;
    linear: number;
    angular: number;
    frame: Frame;
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
    // publish 중단 = backend IDLE_RESET_S (0.2s) 후 fresh latch 준비. 모터는
    // *마지막 valid target 머무름* (자연 정지).
  }, []);

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
        bridge.publish(
          Topic.MOTION_JOG_TCP_STREAM,
          { ...twist, frame: s.frame },
          robotId,
        );
      }, PUBLISH_DT_MS);
    },
    [robotId],
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
              ? "bg-emerald-500/30 border-emerald-500/60 text-emerald-200"
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
              ? "bg-emerald-500/30 border-emerald-500/60 text-emerald-200"
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
          ※ 버튼 hold = TCP twist publish (50Hz). backend latch + SE(3) 적분.
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
            <SliderPrimitive.Range className="absolute h-full rounded-full bg-emerald-500/40" />
          </SliderPrimitive.Track>
          <SliderPrimitive.Thumb className="block h-3.5 w-3.5 rounded-full border border-emerald-400 bg-zinc-900 shadow-sm focus-visible:outline-none focus-visible:ring-1 focus-visible:ring-emerald-400" />
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
            <SliderPrimitive.Range className="absolute h-full rounded-full bg-emerald-500/40" />
          </SliderPrimitive.Track>
          <SliderPrimitive.Thumb className="block h-3.5 w-3.5 rounded-full border border-emerald-400 bg-zinc-900 shadow-sm focus-visible:outline-none focus-visible:ring-1 focus-visible:ring-emerald-400" />
        </SliderPrimitive.Root>
        <span className="text-[10px] text-zinc-400 tabular-nums w-16 text-right">
          {(angularScale * TCP_ANGULAR_MAX).toFixed(2)} rad/s
        </span>
      </div>
    </div>
  );
}
