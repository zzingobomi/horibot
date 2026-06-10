import { Camera, Loader2, RefreshCw } from "lucide-react";
import type { IDockviewPanelProps } from "dockview";
import { useParams } from "react-router-dom";
import { useCalibrationResults } from "@/hooks/useCalibrationResults";
import { StatusBadge } from "@/components/shared/StatusBadge";
import { PanelShell } from "@/components/shared/PanelShell";
import { Section } from "@/components/shared/Section";
import { MatrixTable } from "@/components/shared/MatrixTable";

export function CalibrationPanel(props: IDockviewPanelProps<object>) {
  const { id: robotId = "" } = useParams<{ id: string }>();
  const { results, loading, error, refetch } = useCalibrationResults(robotId);

  return (
    <PanelShell
      icon={<Camera className="w-3.5 h-3.5" />}
      title="Calibration"
      panelId={props.api.id}
      api={props.api}
    >
      <Section label="Status">
        <div className="flex items-start justify-between">
          <div className="space-y-1.5">
            <StatusBadge ok={!!results?.intrinsic} label="Intrinsic" />
            <StatusBadge ok={!!results?.hand_eye} label="Hand-Eye" />
            {error && (
              <p className="text-[10px] text-red-400 font-mono mt-1">
                ⚠ {error}
              </p>
            )}
          </div>

          <button
            onClick={refetch}
            disabled={loading}
            className="p-1.5 rounded hover:bg-zinc-700/60 text-zinc-400 hover:text-zinc-100 transition-colors disabled:opacity-40"
            title="Reload calibration"
          >
            <RefreshCw
              className={`w-3.5 h-3.5 ${loading ? "animate-spin" : ""}`}
            />
          </button>
        </div>
      </Section>

      {results?.hand_eye?.R && results.hand_eye.t && (
        <Section label="Hand-Eye Transform">
          <div className="space-y-3">
            <MatrixTable data={results.hand_eye.R} label="R (3×3)" />
            <MatrixTable data={results.hand_eye.t} label="t [m]" />
          </div>
        </Section>
      )}

      {results?.intrinsic?.camera_matrix && (
        <Section label="Camera Intrinsics">
          <MatrixTable data={results.intrinsic.camera_matrix} label="K (3×3)" />
          {results.intrinsic.image_size && (
            <p className="font-mono text-[11px] text-zinc-400 mt-2">
              {results.intrinsic.image_size[0]} ×{" "}
              {results.intrinsic.image_size[1]} px
            </p>
          )}
        </Section>
      )}

      {loading && (
        <div className="flex items-center gap-2 px-3 py-2 text-xs text-zinc-500 font-mono">
          <Loader2 className="w-3.5 h-3.5 animate-spin" /> Loading…
        </div>
      )}
    </PanelShell>
  );
}
