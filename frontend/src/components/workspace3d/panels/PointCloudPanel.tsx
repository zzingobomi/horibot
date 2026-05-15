import { Cloud } from "lucide-react";
import type { IDockviewPanelProps } from "dockview";
import { usePointCloudStore } from "@/store/pointCloudStore";
import { PanelShell } from "../ui/PanelShell";
import { Section } from "../ui/Section";
import { ToggleRow } from "../ui/ToggleRow";

const VOXEL_PRESETS = [0.003, 0.005, 0.008];

export function PointCloudPanel(props: IDockviewPanelProps<object>) {
  const enabled = usePointCloudStore((s) => s.enabled);
  const voxelSize = usePointCloudStore((s) => s.voxelSize);
  const frame = usePointCloudStore((s) => s.frame);
  const setEnabled = usePointCloudStore((s) => s.setEnabled);
  const setVoxelSize = usePointCloudStore((s) => s.setVoxelSize);

  return (
    <PanelShell
      icon={<Cloud className="w-3.5 h-3.5" />}
      title="Point Cloud"
      panelId={props.api.id}
      api={props.api}
    >
      <Section label="Live Stream">
        <ToggleRow
          label={enabled ? "Streaming" : "Off"}
          checked={enabled}
          onChange={() => setEnabled(!enabled)}
          accentColor="bg-emerald-400"
        />
      </Section>

      <Section label="Voxel Size">
        <div className="grid grid-cols-4 gap-1">
          {VOXEL_PRESETS.map((v) => {
            const active = Math.abs(v - voxelSize) < 1e-6;
            return (
              <button
                key={v}
                onClick={() => setVoxelSize(v)}
                className={`text-[10px] font-mono py-1 rounded transition-colors ${
                  active
                    ? "bg-emerald-500/20 text-emerald-300"
                    : "bg-zinc-900 text-zinc-500 hover:bg-zinc-800"
                }`}
              >
                {(v * 1000).toFixed(0)}mm
              </button>
            );
          })}
        </div>
      </Section>

      <Section label="Stats">
        <div className="text-[11px] font-mono space-y-1">
          <div className="flex justify-between">
            <span className="text-zinc-500">Live</span>
            <span className="text-zinc-300">
              {frame ? frame.count.toLocaleString() : "—"}
            </span>
          </div>
        </div>
      </Section>
    </PanelShell>
  );
}
