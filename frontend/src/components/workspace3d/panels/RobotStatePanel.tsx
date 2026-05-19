import { useMemo } from "react";
import { Activity } from "lucide-react";
import type { IDockviewPanelProps } from "dockview";
import { useRobotStore } from "@/store/robotStore";
import { useSceneStore } from "@/store/sceneStore";
import { useCalibrationResults } from "@/hooks/useCalibrationResults";
import { PanelShell } from "../ui/PanelShell";
import { Section } from "../ui/Section";

export function RobotStatePanel(props: IDockviewPanelProps<object>) {
  const joints = useRobotStore((s) => s.joints);
  const tcpPos = useSceneStore((s) => s.tcpPos);
  const { results } = useCalibrationResults();
  const jointOffsets = results?.joint_offsets_rad;

  const jointAngles = useMemo(() => {
    if (!joints?.length) return Array(5).fill(0) as number[];
    return joints
      .filter((j) => j.id >= 1 && j.id <= 5)
      .sort((a, b) => a.id - b.id)
      .map((j, idx) => {
        const offset = jointOffsets?.[idx] ?? 0;
        if (j.degree !== undefined)
          return (j.degree * Math.PI) / 180 + offset;
        if (j.position !== undefined)
          return ((j.position - 2048) / 4095) * 2 * Math.PI + offset;
        return offset;
      });
  }, [joints, jointOffsets]);

  return (
    <PanelShell
      icon={<Activity className="w-3.5 h-3.5" />}
      title="Robot State"
      panelId={props.api.id}
      api={props.api}
    >
      <Section label="Joint Angles">
        <div className="font-mono text-[11px] space-y-1">
          {jointAngles.map((rad, i) => (
            <div key={i} className="flex items-center gap-2">
              <span className="text-zinc-600 w-4">J{i + 1}</span>
              <div className="flex-1 h-0.5 bg-zinc-800 rounded overflow-hidden">
                <div
                  className="h-full bg-blue-500/70 rounded transition-all duration-100"
                  style={{
                    width: `${((rad + Math.PI) / (2 * Math.PI)) * 100}%`,
                  }}
                />
              </div>
              <span className="text-zinc-300 tabular-nums w-14 text-right">
                {((rad * 180) / Math.PI).toFixed(1)}°
              </span>
            </div>
          ))}
        </div>
      </Section>

      <Section label="TCP Position">
        {tcpPos ? (
          <div className="font-mono text-[11px] space-y-1">
            {(["x", "y", "z"] as const).map((axis, i) => (
              <div key={axis} className="flex justify-between items-center">
                <span className="text-zinc-500">{axis.toUpperCase()}</span>
                <span className="text-emerald-400 tabular-nums">
                  {tcpPos[i].toFixed(4)}
                  <span className="text-zinc-600 ml-1">m</span>
                </span>
              </div>
            ))}
          </div>
        ) : (
          <p className="text-[11px] text-zinc-600 font-mono">No robot loaded</p>
        )}
      </Section>
    </PanelShell>
  );
}
