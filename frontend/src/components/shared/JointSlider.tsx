import * as SliderPrimitive from "@radix-ui/react-slider";
import type { Joint } from "@/types/motor";
import { formatDeg, rawToDeg } from "@/lib/robot/utils";

interface Props {
  joint: Joint;
  cmdPosition: number;
  limitMin: number;
  limitMax: number;
  onValueChange: (id: number, position: number) => void;
}

export function JointSlider({
  joint,
  cmdPosition,
  limitMin,
  limitMax,
  onValueChange,
}: Props) {
  const toPercent = (val: number) =>
    ((val - limitMin) / (limitMax - limitMin)) * 100;

  const isLagging = Math.abs(cmdPosition - joint.position) > 50;

  return (
    <div className="py-2 px-1">
      <div className="flex items-center justify-between mb-1.5 font-mono">
        <span className="text-[11px] text-zinc-300">{joint.name}</span>
        <div className="flex gap-3 text-[10px] tabular-nums">
          <span className="text-blue-400">
            cmd {formatDeg(rawToDeg(cmdPosition))}°
          </span>
          <span className={isLagging ? "text-orange-400" : "text-zinc-500"}>
            act {formatDeg(rawToDeg(joint.position))}°
          </span>
        </div>
      </div>

      <SliderPrimitive.Root
        className="relative flex items-center select-none touch-none w-full h-4"
        min={limitMin}
        max={limitMax}
        step={1}
        value={[cmdPosition]}
        onValueChange={([v]: number[]) => onValueChange(joint.id, v)}
      >
        <SliderPrimitive.Track className="relative h-1 w-full grow rounded-full bg-zinc-800">
          <SliderPrimitive.Range className="absolute h-full rounded-full bg-blue-500/40" />
          <div
            className="absolute top-1/2 -translate-y-1/2 -translate-x-1/2 w-0.5 h-3 rounded-full bg-orange-400 pointer-events-none transition-[left] duration-75"
            style={{ left: `${toPercent(joint.position)}%` }}
          />
        </SliderPrimitive.Track>
        <SliderPrimitive.Thumb className="block h-3.5 w-3.5 rounded-full border border-blue-400 bg-zinc-900 shadow-sm focus-visible:outline-none focus-visible:ring-1 focus-visible:ring-blue-400" />
      </SliderPrimitive.Root>
    </div>
  );
}
