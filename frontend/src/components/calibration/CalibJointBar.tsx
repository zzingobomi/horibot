import { useCallback, useState } from "react";
import * as SliderPrimitive from "@radix-ui/react-slider";
import { ChevronDown, ChevronUp } from "lucide-react";
import { Button } from "@/components/ui/button";
import { useJointControl } from "@/hooks/useJointControl";
import { useRobotStore } from "@/store/robotStore";
import type { Joint } from "@/types/motor";
import { formatDeg, rawToDeg } from "@/lib/robot/utils";

export function CalibJointBar() {
  const joints = useRobotStore((s) => s.joints);
  const configs = useRobotStore((s) => s.configs);
  const { torqueEnabled, enableTorque, goHome, sendJointCmd } =
    useJointControl();

  const [expanded, setExpanded] = useState(false);
  const [cmdPositions, setCmdPositions] = useState<Record<number, number>>({});

  const getCmdPosition = (joint: { id: number; position: number }) =>
    cmdPositions[joint.id] ?? joint.position;

  const handleJointCmd = useCallback(
    (id: number, position: number) => {
      setCmdPositions((prev) => ({ ...prev, [id]: position }));
      sendJointCmd(id, position);
    },
    [sendJointCmd]
  );

  const syncAll = useCallback(() => {
    const synced = Object.fromEntries(joints.map((j) => [j.id, j.position]));
    setCmdPositions(synced);
  }, [joints]);

  const getLimit = (id: number) => {
    const cfg = configs.find((c) => c.id === id);
    return { min: cfg?.limit.min ?? 0, max: cfg?.limit.max ?? 4095 };
  };

  return (
    <div className="absolute top-2 left-2 right-2 rounded-md border bg-card/90 backdrop-blur shadow max-w-md">
      {/* 헤더 — 항상 보임 */}
      <div className="flex items-center gap-2 px-2 py-1.5">
        <Button
          size="sm"
          variant={torqueEnabled ? "destructive" : "default"}
          onClick={() => enableTorque(!torqueEnabled)}
        >
          Torque {torqueEnabled ? "OFF" : "ON"}
        </Button>
        <Button size="sm" variant="outline" onClick={goHome}>
          Home
        </Button>
        {expanded && (
          <Button size="sm" variant="outline" onClick={syncAll}>
            Sync
          </Button>
        )}
        <div className="flex-1" />
        <button
          onClick={() => setExpanded((v) => !v)}
          className="p-1 rounded hover:bg-muted text-muted-foreground"
          title={expanded ? "슬라이더 접기" : "슬라이더 펼치기"}
        >
          {expanded ? (
            <ChevronUp className="w-4 h-4" />
          ) : (
            <ChevronDown className="w-4 h-4" />
          )}
        </button>
      </div>

      {/* 접혀있을 때: 각도 readout 한 줄 */}
      {!expanded && (
        <div className="px-2 pb-1.5 flex flex-wrap gap-x-2 gap-y-0.5 font-mono text-[11px] tabular-nums text-muted-foreground">
          {joints.length === 0 ? (
            <span className="italic">모터 대기</span>
          ) : (
            joints.map((j) => (
              <span key={j.id}>
                <span className="text-foreground">{j.name}</span>{" "}
                {formatDeg(rawToDeg(j.position))}°
              </span>
            ))
          )}
        </div>
      )}

      {/* 펼쳤을 때: 컴팩트 슬라이더 한 줄씩 */}
      {expanded && (
        <div className="border-t px-2 py-1.5 flex flex-col gap-1">
          {joints.length === 0 ? (
            <p className="py-1 text-center text-xs text-muted-foreground">
              모터 연결 대기 중...
            </p>
          ) : (
            <>
              <div className="flex items-center gap-2 font-mono text-[9px] uppercase tracking-wide text-muted-foreground">
                <span className="w-9 shrink-0" />
                <span className="flex-1" />
                <span className="w-11 text-right text-primary shrink-0">
                  cmd
                </span>
                <span className="w-11 text-right shrink-0">act</span>
              </div>
              {joints.map((joint) => {
                const { min, max } = getLimit(joint.id);
                return (
                  <CompactSliderRow
                    key={joint.id}
                    joint={joint}
                    cmdPosition={getCmdPosition(joint)}
                    limitMin={min}
                    limitMax={max}
                    onValueChange={handleJointCmd}
                  />
                );
              })}
            </>
          )}
        </div>
      )}
    </div>
  );
}

function shortName(name: string): string {
  if (name.startsWith("joint")) return "J" + name.slice(5);
  if (name === "gripper_joint" || name === "gripper") return "Grip";
  return name;
}

function CompactSliderRow({
  joint,
  cmdPosition,
  limitMin,
  limitMax,
  onValueChange,
}: {
  joint: Joint;
  cmdPosition: number;
  limitMin: number;
  limitMax: number;
  onValueChange: (id: number, position: number) => void;
}) {
  const toPercent = (val: number) =>
    ((val - limitMin) / (limitMax - limitMin)) * 100;
  const isLagging = Math.abs(cmdPosition - joint.position) > 50;

  return (
    <div className="flex items-center gap-2 font-mono text-[10px] tabular-nums">
      <span
        className="w-9 text-foreground shrink-0 truncate"
        title={joint.name}
      >
        {shortName(joint.name)}
      </span>

      <SliderPrimitive.Root
        className="relative flex items-center select-none touch-none flex-1 h-4"
        min={limitMin}
        max={limitMax}
        step={1}
        value={[cmdPosition]}
        onValueChange={([v]: number[]) => onValueChange(joint.id, v)}
      >
        <SliderPrimitive.Track className="relative h-1 w-full grow rounded-full bg-secondary">
          <SliderPrimitive.Range className="absolute h-full rounded-full bg-primary/50" />
          {/* actual 위치 마커 (오렌지) */}
          <div
            className="absolute top-1/2 -translate-y-1/2 -translate-x-1/2 w-0.5 h-2.5 rounded-full bg-orange-400 pointer-events-none transition-[left] duration-75"
            style={{ left: `${toPercent(joint.position)}%` }}
          />
        </SliderPrimitive.Track>
        {/* cmd thumb (프라이머리) */}
        <SliderPrimitive.Thumb className="block h-3 w-3 rounded-full border border-primary bg-background shadow-sm focus-visible:outline-none focus-visible:ring-1 focus-visible:ring-ring" />
      </SliderPrimitive.Root>

      <span className="w-11 text-right text-primary shrink-0">
        {formatDeg(rawToDeg(cmdPosition))}°
      </span>
      <span
        className={`w-11 text-right shrink-0 ${
          isLagging ? "text-orange-400" : "text-muted-foreground"
        }`}
      >
        {formatDeg(rawToDeg(joint.position))}°
      </span>
    </div>
  );
}
