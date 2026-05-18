import type { ComputeData, PerPoseResidual } from "./types";

function sigmaRotColor(deg: number): string {
  return deg < 0.5
    ? "text-green-500"
    : deg < 1.5
    ? "text-amber-500"
    : "text-red-500";
}

function sigmaTColor(mm: number): string {
  return mm < 5
    ? "text-green-500"
    : mm < 15
    ? "text-amber-500"
    : "text-red-500";
}

function PerPoseResidualTable({ rows }: { rows: PerPoseResidual[] }) {
  return (
    <div className="max-h-32 overflow-y-auto">
      <table className="w-full text-[11px] font-mono">
        <tbody>
          {rows.map((r) => (
            <tr key={r.id}>
              <td className="py-0.5 text-muted-foreground">#{r.id}</td>
              <td className={`py-0.5 text-right ${sigmaRotColor(r.drot_deg)}`}>
                {r.drot_deg.toFixed(3)}°
              </td>
              <td className="py-0.5 text-right text-muted-foreground">
                {r.dt_mm.toFixed(1)}mm
              </td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
}

export function ComputePreview({ data }: { data: ComputeData }) {
  return (
    <div className="space-y-3 text-xs">
      <div className="rounded-md bg-muted p-2 space-y-1 font-mono">
        <div className="flex justify-between">
          <span className="text-muted-foreground">method</span>
          <span>{data.method}</span>
        </div>
        <div className="flex justify-between">
          <span className="text-muted-foreground">poses</span>
          <span>{data.pose_count}</span>
        </div>
        <div className="flex justify-between">
          <span className="text-muted-foreground">t [mm]</span>
          <span>
            {data.t_cam2gripper.map((v) => (v * 1000).toFixed(1)).join(", ")}
          </span>
        </div>
        <div className="flex justify-between">
          <span className="text-muted-foreground">σ_rot</span>
          <span className={sigmaRotColor(data.sigma_rot_deg)}>
            {data.sigma_rot_deg.toFixed(3)}°
          </span>
        </div>
        <div className="flex justify-between">
          <span className="text-muted-foreground">σ_t</span>
          <span className={sigmaTColor(data.sigma_t_mm)}>
            {data.sigma_t_mm.toFixed(1)}mm
          </span>
        </div>
      </div>

      <div>
        <p className="text-[10px] text-muted-foreground font-mono mb-1">
          method self-consistency
        </p>
        <table className="w-full text-[11px] font-mono">
          <tbody>
            {data.method_compare.map((c) => {
              const isRef = c.ref;
              const drotColor = isRef
                ? "text-muted-foreground"
                : c.drot_deg < 1
                ? "text-green-500"
                : c.drot_deg < 3
                ? "text-amber-500"
                : "text-red-500";
              return (
                <tr key={c.method}>
                  <td className="py-0.5">{c.method}</td>
                  <td className={`py-0.5 text-right ${drotColor}`}>
                    {isRef ? "기준" : `Δ${c.drot_deg.toFixed(3)}°`}
                  </td>
                  <td className="py-0.5 text-right text-muted-foreground">
                    {isRef ? "" : `${c.dt_mm.toFixed(1)}mm`}
                  </td>
                </tr>
              );
            })}
          </tbody>
        </table>
      </div>

      <div>
        <p className="text-[10px] text-muted-foreground font-mono mb-1">
          per-pose residual (T_target←base 분산)
        </p>
        <PerPoseResidualTable rows={data.per_pose_residual} />
      </div>
    </div>
  );
}
