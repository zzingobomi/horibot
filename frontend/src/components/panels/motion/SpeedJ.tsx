/**
 * SpeedJ Jog — joint velocity 추종 (산업 펜던트 jog 패턴).
 *
 * 버튼 hold 동안 50Hz 로 `MOTION_SPEED_J` publish → backend `_velocity_loop` 가
 * Ruckig velocity mode + jerk-limited 추종 + 100ms deadman timeout. 손 떼면
 * publish 멈춤 → backend 자동 감속 정지. 게임패드 LT hold 와 동일 메커니즘.
 *
 * 한 번에 1 joint 만 hold (산업 펜던트 컨벤션 — 여러 joint 동시 jog 는 예측 어려움).
 */
import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { useParams } from "react-router-dom";
import * as SliderPrimitive from "@radix-ui/react-slider";
import { PanelButton } from "@/components/shared/PanelButton";
import { useService, useTopic } from "@/framework";
import { ServiceKey, Topic } from "@/constants/topics";
import { useArmJoints } from "@/lib/robot/config";

const PUBLISH_DT_MS = 20; // 50Hz — backend deadman timeout 100ms 안에 갱신.
const JOINT_VEL_MAX = 0.6; // rad/s — 게임패드 GamepadNode 와 동일 권장치.

export function SpeedJControl() {
  const { id: robotId = "" } = useParams<{ id: string }>();
  const armJoints = useArmJoints(robotId);
  const joints = useTopic(Topic.MOTOR_STATE_JOINT, robotId)?.joints ?? [];
  const speedJ = useService(ServiceKey.MOTION_SPEED_J, robotId);

  const armIds = useMemo(() => new Set(armJoints.map((j) => j.id)), [armJoints]);
  const currentJoints = joints.filter((j) => armIds.has(j.id));

  // jog 진행 중인 joint id + 방향. null = idle.
  const [jogState, setJogState] = useState<{ id: number; dir: 1 | -1 } | null>(
    null,
  );
  const [velocityScale, setVelocityScale] = useState(0.3); // 0.1 ~ 1.0

  const intervalRef = useRef<number | null>(null);
  // 최신 jogState/velocityScale 를 setInterval 콜백이 읽어야 함 — closure stale 방지.
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
    // 명시적 stop publish — backend deadman timeout (100ms) 기다리지 말고
    // 즉시 target=0 → joint Ruckig jerk-limited 감속. pendant button up
    // 의 *명시 signal* 자리.
    if (armJoints.length > 0) {
      void speedJ.call({ velocities: armJoints.map(() => 0) });
    }
  }, [armJoints, speedJ]);

  const startJog = useCallback(
    (id: number, dir: 1 | -1) => {
      if (armJoints.length === 0) return;
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
        void speedJ.call({ velocities });
      }, PUBLISH_DT_MS);
    },
    [armJoints, speedJ],
  );

  // 컴포넌트 unmount 시 jog 정지 — 탭 전환해도 robot 안 도망감.
  useEffect(() => {
    return () => {
      if (intervalRef.current !== null) {
        window.clearInterval(intervalRef.current);
        intervalRef.current = null;
      }
    };
  }, []);

  // 전역 pointerup — 버튼 위에서 손 떼지 않고 옆으로 끌고 가도 jog 정지.
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
        ※ 버튼 hold = joint velocity jog. 손 떼면 deadman timeout (100ms) 자동
        정지.
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
                    {current.degree.toFixed(2)}°
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
                    ? "bg-amber-500/30 border-amber-500/60 text-amber-200"
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
                    ? "bg-amber-500/30 border-amber-500/60 text-amber-200"
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
            <SliderPrimitive.Range className="absolute h-full rounded-full bg-amber-500/40" />
          </SliderPrimitive.Track>
          <SliderPrimitive.Thumb className="block h-3.5 w-3.5 rounded-full border border-amber-400 bg-zinc-900 shadow-sm focus-visible:outline-none focus-visible:ring-1 focus-visible:ring-amber-400" />
        </SliderPrimitive.Root>
        <span className="text-[10px] text-zinc-400 tabular-nums w-16 text-right">
          {(velocityScale * JOINT_VEL_MAX).toFixed(2)} rad/s
        </span>
      </div>
    </div>
  );
}
