/**
 * JogTcpControl — Cartesian velocity jog (frontend_v2.md §3.3 + motion_taxonomy.md §Jog).
 *
 * MotionPanel 안에서만 쓰는 control (구현 세부). robotId 는 props (순수).
 *
 * Wire: `Motion.Stream.JOG_TCP` topic publish (fire-and-forget, 50Hz). backend
 * JogTcpCommand 가 SE(3) 적분 + IK → motor cmd. frame=base 는 world axes, tcp 는
 * EE-local. 5DOF robot 은 backend IK 가 position-only fallback.
 */
import { useCallback, useEffect, useRef, useState } from "react";
import { bridge } from "@/api/bridge";
import { Button } from "@/components/ui/button";
import { Slider } from "@/components/ui/slider";
import { cn } from "@/lib/utils";
import { Topic, type JogTcpInput } from "@/api/generated/contract";

const PUBLISH_DT_MS = 20; // 50Hz
const TCP_LINEAR_MAX = 0.08; // m/s
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
): { linear: [number, number, number]; angular: [number, number, number] } {
  const linear: [number, number, number] = [0, 0, 0];
  const angular: [number, number, number] = [0, 0, 0];
  if (!isAngular(jog.axis)) {
    const idx = { X: 0, Y: 1, Z: 2 }[jog.axis];
    linear[idx] = jog.dir * linearScale * TCP_LINEAR_MAX;
  } else {
    const idx = { Rx: 0, Ry: 1, Rz: 2 }[jog.axis];
    angular[idx] = jog.dir * angularScale * TCP_ANGULAR_MAX;
  }
  return { linear, angular };
}

interface JogTcpControlProps {
  robotId: string;
}

export function JogTcpControl({ robotId }: JogTcpControlProps) {
  const [jog, setJog] = useState<JogState | null>(null);
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
    stateRef.current = { jog, linear: linearScale, angular: angularScale, frame };
  });

  const stopJog = useCallback(() => {
    if (intervalRef.current !== null) {
      window.clearInterval(intervalRef.current);
      intervalRef.current = null;
    }
    setJog(null);
  }, []);

  const startJog = useCallback(
    (axis: AxisKey, dir: Dir) => {
      setJog({ axis, dir });
      if (intervalRef.current !== null) window.clearInterval(intervalRef.current);
      intervalRef.current = window.setInterval(() => {
        const s = stateRef.current;
        if (s.jog === null) return;
        const twist = buildTwist(s.jog, s.linear, s.angular);
        const payload: JogTcpInput = {
          robot_id: robotId,
          linear: twist.linear,
          angular: twist.angular,
          frame: s.frame,
        };
        bridge.publish(Topic.MOTION_JOG_TCP, payload, robotId);
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
    if (jog === null) return;
    const handler = () => stopJog();
    window.addEventListener("pointerup", handler);
    window.addEventListener("pointercancel", handler);
    window.addEventListener("blur", handler);
    return () => {
      window.removeEventListener("pointerup", handler);
      window.removeEventListener("pointercancel", handler);
      window.removeEventListener("blur", handler);
    };
  }, [jog, stopJog]);

  const renderAxis = (axis: AxisKey) => {
    const isMinus = jog?.axis === axis && jog.dir === -1;
    const isPlus = jog?.axis === axis && jog.dir === 1;
    const activeCls =
      "bg-amber-500/30 border-amber-500/60 text-amber-200 hover:bg-amber-500/30";
    return (
      <div key={axis} className="flex items-center gap-2 font-mono">
        <div className="w-10 shrink-0 text-[11px] text-foreground">{axis}</div>
        <Button
          variant="outline"
          size="sm"
          onPointerDown={(e) => {
            e.preventDefault();
            e.currentTarget.setPointerCapture(e.pointerId);
            startJog(axis, -1);
          }}
          className={cn("flex-1 font-mono", isMinus && activeCls)}
        >
          −
        </Button>
        <Button
          variant="outline"
          size="sm"
          onPointerDown={(e) => {
            e.preventDefault();
            e.currentTarget.setPointerCapture(e.pointerId);
            startJog(axis, 1);
          }}
          className={cn("flex-1 font-mono", isPlus && activeCls)}
        >
          +
        </Button>
      </div>
    );
  };

  return (
    <div className="flex flex-col gap-3">
      <p className="text-[10px] text-muted-foreground font-mono leading-relaxed">
        버튼 hold = TCP twist publish (50Hz). backend SE(3) 적분 + IK → motor cmd.
        frame: <span className="text-foreground">{frame}</span>
      </p>

      <div className="flex gap-2">
        {(["base", "tcp"] as Frame[]).map((f) => (
          <Button
            key={f}
            variant="outline"
            size="xs"
            onClick={() => setFrame(f)}
            className={cn(
              "flex-1 font-mono uppercase",
              frame === f &&
                "bg-emerald-500/20 border-emerald-500/60 text-emerald-200 hover:bg-emerald-500/20",
            )}
          >
            {f}
          </Button>
        ))}
      </div>

      <div className="flex flex-col gap-1.5">
        <div className="text-[9px] uppercase tracking-wide text-muted-foreground">
          linear
        </div>
        {(["X", "Y", "Z"] as LinearAxis[]).map(renderAxis)}
      </div>

      <div className="flex flex-col gap-1.5">
        <div className="text-[9px] uppercase tracking-wide text-muted-foreground">
          angular
        </div>
        {(["Rx", "Ry", "Rz"] as AngularAxis[]).map(renderAxis)}
      </div>

      <div className="flex items-center gap-2 font-mono">
        <span className="text-[10px] uppercase tracking-wide text-muted-foreground w-14 shrink-0">
          lin
        </span>
        <Slider
          min={0.1}
          max={1.0}
          step={0.05}
          value={[linearScale]}
          onValueChange={([v]) => setLinearScale(v)}
          className="flex-1"
        />
        <span className="text-[10px] text-muted-foreground tabular-nums w-16 text-right">
          {(linearScale * TCP_LINEAR_MAX * 1000).toFixed(0)} mm/s
        </span>
      </div>

      <div className="flex items-center gap-2 font-mono">
        <span className="text-[10px] uppercase tracking-wide text-muted-foreground w-14 shrink-0">
          ang
        </span>
        <Slider
          min={0.1}
          max={1.0}
          step={0.05}
          value={[angularScale]}
          onValueChange={([v]) => setAngularScale(v)}
          className="flex-1"
        />
        <span className="text-[10px] text-muted-foreground tabular-nums w-16 text-right">
          {(angularScale * TCP_ANGULAR_MAX).toFixed(2)} rad/s
        </span>
      </div>
    </div>
  );
}
