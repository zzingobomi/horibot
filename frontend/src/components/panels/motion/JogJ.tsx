/**
 * JogJ — joint-space human velocity jog (motion_taxonomy.md §Jog).
 *
 * Wire 자리 `MOTION_JOG_J_STREAM` topic publish (fire-and-forget, 50Hz). backend
 * JogJCommand 가 자기 process joint_cache (joint_offset 적용 URDF rad) 에서 ref
 * latch + 실 측정 dt 자리 적분 → 절대 URDF rad target → publish_cmd.
 *
 * cross-process safe — frontend 는 *velocity 만* 알면 되며 joint_offset / URDF
 * rad 자리 backend SSOT 자리. 동일 wire 자리 gamepad 자리도 사용.
 *
 * IDLE_RESET_S (backend) 자리 publish 끊긴 후 자동 fresh latch — button up 후
 * 모터 settled 자리에 다시 hold 시 인코더 - ref 누적 drift 차단.
 *
 * 한 번에 1 joint 만 jog (산업 펜던트 컨벤션).
 */
import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { useParams } from "react-router-dom";
import * as SliderPrimitive from "@radix-ui/react-slider";
import { useTopic } from "@/framework";
import { Topic } from "@/constants/topics";
import { useArmJoints } from "@/lib/robot/config";
import { useJointOffsetsRad } from "@/hooks/useCalibrationResults";
import { rawToUrdfDeg } from "@/lib/robot/utils";
import { bridge } from "@/api/bridge";

const PUBLISH_DT_MS = 20; // 50Hz
const JOINT_VEL_MAX = 0.6; // rad/s — gamepad 자리 와 같은 권장치

export function JogJControl() {
  const { id: robotId = "" } = useParams<{ id: string }>();
  const armJoints = useArmJoints(robotId);
  const joints = useTopic(Topic.MOTOR_STATE_JOINT, robotId)?.joints ?? [];
  // URDF degree (joint_offset 적용 frame) — 모든 frontend 표시 SSOT.
  const jointOffsetsRad = useJointOffsetsRad(robotId);

  const armIds = useMemo(() => new Set(armJoints.map((j) => j.id)), [armJoints]);
  const currentJoints = joints.filter((j) => armIds.has(j.id));

  const [jogState, setJogState] = useState<{ id: number; dir: 1 | -1 } | null>(
    null,
  );
  const [velocityScale, setVelocityScale] = useState(0.3);

  const intervalRef = useRef<number | null>(null);
  const stateRef = useRef<{
    jog: { id: number; dir: 1 | -1 } | null;
    scale: number;
  }>({ jog: null, scale: 0.3 });
  useEffect(() => {
    stateRef.current = { jog: jogState, scale: velocityScale };
  });

  const stopJog = useCallback(() => {
    if (intervalRef.current !== null) {
      window.clearInterval(intervalRef.current);
      intervalRef.current = null;
    }
    setJogState(null);
    // publish 중단 → backend IDLE_RESET_S (0.2s) 후 fresh latch 준비. 모터는
    // *마지막 target 머무름* (자연 정지).
  }, []);

  const startJog = useCallback(
    (id: number, dir: 1 | -1) => {
      if (armJoints.length === 0) return;
      const targetIdx = armJoints.findIndex((j) => j.id === id);
      if (targetIdx < 0) return;

      setJogState({ id, dir });
      if (intervalRef.current !== null) {
        window.clearInterval(intervalRef.current);
      }
      intervalRef.current = window.setInterval(() => {
        const s = stateRef.current;
        if (s.jog === null) return;
        const velocities = armJoints.map((j) =>
          j.id === s.jog!.id ? s.jog!.dir * s.scale * JOINT_VEL_MAX : 0,
        );
        bridge.publish(
          Topic.MOTION_JOG_J_STREAM,
          { velocities },
          robotId,
        );
      }, PUBLISH_DT_MS);
    },
    [armJoints, robotId],
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

  return (
    <div className="flex flex-col gap-3">
      <p className="text-[10px] text-zinc-500 font-mono leading-relaxed">
        ※ 버튼 hold = joint velocity publish (50Hz). backend latch + dt 적분
        → 직접 publish. cross-process safe (joint_offset SSOT = backend).
      </p>

      <div className="flex flex-col gap-1.5">
        {armJoints.map((j) => {
          const current = currentJoints.find((c) => c.id === j.id);
          const isActiveMinus =
            jogState?.id === j.id && jogState.dir === -1;
          const isActivePlus =
            jogState?.id === j.id && jogState.dir === 1;

          return (
            <div key={j.id} className="flex items-center gap-2 font-mono">
              <div className="w-14 shrink-0">
                <div className="text-[11px] text-zinc-300">{j.name}</div>
                {current && (
                  <div className="text-[9px] text-zinc-600 tabular-nums">
                    {rawToUrdfDeg(current.position, jointOffsetsRad[j.id] ?? 0).toFixed(2)}°
                  </div>
                )}
              </div>
              <button
                onPointerDown={(e) => {
                  e.preventDefault();
                  startJog(j.id, -1);
                }}
                className={`flex-1 h-7 rounded border text-[11px] font-mono uppercase tracking-wide transition-colors ${
                  isActiveMinus
                    ? "bg-emerald-500/30 border-emerald-500/60 text-emerald-200"
                    : "bg-zinc-900 border-zinc-800 text-zinc-400 hover:bg-zinc-800/60 hover:text-zinc-100"
                }`}
              >
                −
              </button>
              <button
                onPointerDown={(e) => {
                  e.preventDefault();
                  startJog(j.id, 1);
                }}
                className={`flex-1 h-7 rounded border text-[11px] font-mono uppercase tracking-wide transition-colors ${
                  isActivePlus
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
        <SliderPrimitive.Root
          className="relative flex items-center select-none touch-none flex-1 h-4"
          min={0.1}
          max={1.0}
          step={0.05}
          value={[velocityScale]}
          onValueChange={(v) => setVelocityScale(v[0])}
        >
          <SliderPrimitive.Track className="relative h-1 w-full grow rounded-full bg-zinc-800">
            <SliderPrimitive.Range className="absolute h-full rounded-full bg-emerald-500/40" />
          </SliderPrimitive.Track>
          <SliderPrimitive.Thumb className="block h-3.5 w-3.5 rounded-full border border-emerald-400 bg-zinc-900 shadow-sm focus-visible:outline-none focus-visible:ring-1 focus-visible:ring-emerald-400" />
        </SliderPrimitive.Root>
        <span className="text-[10px] text-zinc-400 tabular-nums w-16 text-right">
          {(velocityScale * JOINT_VEL_MAX).toFixed(2)} rad/s
        </span>
      </div>
    </div>
  );
}
