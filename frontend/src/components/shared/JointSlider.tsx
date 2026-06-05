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
    <div className="py-3 px-1">
      <div className="flex items-center justify-between mb-2">
        <span className="text-sm font-medium">{joint.name}</span>
        <div className="flex gap-3 text-xs tabular-nums">
          <span className="text-primary">
            cmd {formatDeg(rawToDeg(cmdPosition))}°
          </span>
          <span
            className={isLagging ? "text-orange-400" : "text-muted-foreground"}
          >
            actual {formatDeg(rawToDeg(joint.position))}°
          </span>
        </div>
      </div>

      <SliderPrimitive.Root
        className="relative flex items-center select-none touch-none w-full h-5"
        min={limitMin}
        max={limitMax}
        step={1}
        value={[cmdPosition]}
        onValueChange={([v]: number[]) => onValueChange(joint.id, v)}
      >
        <SliderPrimitive.Track className="relative h-1.5 w-full grow rounded-full bg-secondary">
          <SliderPrimitive.Range className="absolute h-full rounded-full bg-primary/50" />
          {/* Actual 마커 */}
          <div
            className="absolute top-1/2 -translate-y-1/2 -translate-x-1/2 w-1 h-3.5 rounded-full bg-orange-400 pointer-events-none transition-[left] duration-75"
            style={{ left: `${toPercent(joint.position)}%` }}
          />
        </SliderPrimitive.Track>
        {/* Command thumb */}
        <SliderPrimitive.Thumb className="block h-4 w-4 rounded-full border-2 border-primary bg-background shadow focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-ring" />
      </SliderPrimitive.Root>
    </div>
  );
}
