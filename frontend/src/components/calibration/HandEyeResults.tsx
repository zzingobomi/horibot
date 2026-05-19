import { AlertTriangle, CheckCircle2, Info, Trash2 } from "lucide-react";
import type {
  BundleAdjustData,
  ComputeData,
  Diagnosis,
  PerPoseResidual,
} from "./types";

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

function PerPoseResidualTable({
  rows,
  onRemove,
  removeDisabled,
  highlightIds,
}: {
  rows: PerPoseResidual[];
  onRemove?: (id: number) => void;
  removeDisabled?: boolean;
  highlightIds?: number[];
}) {
  const highlight = new Set(highlightIds ?? []);
  return (
    <div className="max-h-32 overflow-y-auto">
      <table className="w-full text-[11px] font-mono">
        <tbody>
          {rows.map((r) => {
            const isOutlier = highlight.has(r.id);
            return (
              <tr
                key={r.id}
                className={isOutlier ? "bg-red-500/10" : undefined}
              >
                <td className="py-0.5 text-muted-foreground">#{r.id}</td>
                <td
                  className={`py-0.5 text-right ${sigmaRotColor(r.drot_deg)}`}
                >
                  {r.drot_deg.toFixed(3)}°
                </td>
                <td className="py-0.5 text-right text-muted-foreground">
                  {r.dt_mm.toFixed(1)}mm
                </td>
                {onRemove && (
                  <td className="py-0.5 pl-2 text-right">
                    <button
                      onClick={() => onRemove(r.id)}
                      disabled={removeDisabled}
                      title="이 포즈 삭제"
                      className="text-muted-foreground hover:text-destructive disabled:opacity-40"
                    >
                      <Trash2 className="w-3 h-3" />
                    </button>
                  </td>
                )}
              </tr>
            );
          })}
        </tbody>
      </table>
    </div>
  );
}

function DiagnosisBanner({ diagnosis }: { diagnosis: Diagnosis }) {
  // status별 스타일 + 아이콘
  const variants: Record<
    Diagnosis["status"],
    {
      Icon: typeof Info;
      cls: string;
      label: string;
    }
  > = {
    good: {
      Icon: CheckCircle2,
      cls: "border-green-500/40 bg-green-500/10 text-green-700 dark:text-green-300",
      label: "GOOD",
    },
    outlier_present: {
      Icon: AlertTriangle,
      cls: "border-red-500/40 bg-red-500/10 text-red-700 dark:text-red-300",
      label: "OUTLIER",
    },
    insufficient_diversity: {
      Icon: AlertTriangle,
      cls: "border-amber-500/40 bg-amber-500/10 text-amber-700 dark:text-amber-300",
      label: "DIVERSITY",
    },
    fk_floor_reached: {
      Icon: Info,
      cls: "border-violet-500/40 bg-violet-500/10 text-violet-700 dark:text-violet-300",
      label: "FK FLOOR",
    },
  };
  const v = variants[diagnosis.status];
  const Icon = v.Icon;
  return (
    <div
      className={`rounded-md border p-2 flex gap-2 text-[11px] ${v.cls}`}
      role="status"
    >
      <Icon className="w-4 h-4 shrink-0 mt-0.5" />
      <div className="flex-1 min-w-0">
        <div className="font-mono font-semibold text-[10px] tracking-wide">
          {v.label}
        </div>
        <div className="leading-snug">{diagnosis.message}</div>
      </div>
    </div>
  );
}

export function ComputePreview({
  data,
  onRemovePose,
  removeDisabled,
}: {
  data: ComputeData;
  onRemovePose?: (id: number) => void;
  removeDisabled?: boolean;
}) {
  return (
    <div className="space-y-3 text-xs">
      <DiagnosisBanner diagnosis={data.diagnosis} />

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
        <PerPoseResidualTable
          rows={data.per_pose_residual}
          onRemove={onRemovePose}
          removeDisabled={removeDisabled}
          highlightIds={data.diagnosis.outlier_ids}
        />
      </div>
    </div>
  );
}

export function BundleAdjustPreview({
  data,
  onRemovePose,
  removeDisabled,
}: {
  data: BundleAdjustData;
  onRemovePose?: (id: number) => void;
  removeDisabled?: boolean;
}) {
  const rotImprovement = data.seed_sigma_rot_deg - data.sigma_rot_deg;
  const tImprovement = data.seed_sigma_t_mm - data.sigma_t_mm;

  return (
    <div className="space-y-3 text-xs">
      {/* BEFORE/AFTER 비교 — BA가 효과 있는지 한눈에 */}
      <div className="rounded-md border border-violet-500/30 bg-violet-500/5 p-2 space-y-1.5 font-mono">
        <div className="text-[10px] text-violet-700 dark:text-violet-300 font-semibold tracking-wide">
          BA: BEFORE → AFTER
        </div>
        <div className="grid grid-cols-3 gap-2 text-[11px]">
          <div></div>
          <div className="text-muted-foreground">seed (TSAI)</div>
          <div className="text-violet-700 dark:text-violet-300">BA</div>
          <div className="text-muted-foreground">σ_rot</div>
          <div>{data.seed_sigma_rot_deg.toFixed(3)}°</div>
          <div className={sigmaRotColor(data.sigma_rot_deg)}>
            {data.sigma_rot_deg.toFixed(3)}°
            <span className="text-[10px] text-muted-foreground ml-1">
              ({rotImprovement >= 0 ? "−" : "+"}
              {Math.abs(rotImprovement).toFixed(2)}°)
            </span>
          </div>
          <div className="text-muted-foreground">σ_t</div>
          <div>{data.seed_sigma_t_mm.toFixed(1)}mm</div>
          <div className={sigmaTColor(data.sigma_t_mm)}>
            {data.sigma_t_mm.toFixed(1)}mm
            <span className="text-[10px] text-muted-foreground ml-1">
              ({tImprovement >= 0 ? "−" : "+"}
              {Math.abs(tImprovement).toFixed(1)}mm)
            </span>
          </div>
        </div>
      </div>

      <div className="rounded-md bg-muted p-2 space-y-1 font-mono">
        <div className="flex justify-between">
          <span className="text-muted-foreground">iter</span>
          <span>
            {data.iterations} · {data.elapsed_sec.toFixed(2)}s
          </span>
        </div>
        <div className="flex justify-between">
          <span className="text-muted-foreground">cost</span>
          <span>
            {data.cost_initial.toExponential(2)} →{" "}
            {data.cost_final.toExponential(2)}
          </span>
        </div>
        <div className="flex justify-between">
          <span className="text-muted-foreground">t [mm]</span>
          <span>
            {data.t_cam2gripper.map((v) => (v * 1000).toFixed(1)).join(", ")}
          </span>
        </div>
      </div>

      <div>
        <p className="text-[10px] text-muted-foreground font-mono mb-1">
          joint_offsets [°] — 모터 horn 조립 보정값
        </p>
        <div className="rounded-md bg-muted p-2 font-mono text-[11px]">
          {data.joint_offsets_deg.map((deg, i) => (
            <div key={i} className="flex justify-between">
              <span className="text-muted-foreground">joint {i + 1}</span>
              <span
                className={
                  Math.abs(deg) > 2
                    ? "text-amber-500"
                    : Math.abs(deg) > 0.5
                    ? "text-foreground"
                    : "text-muted-foreground"
                }
              >
                {deg >= 0 ? "+" : ""}
                {deg.toFixed(3)}°
              </span>
            </div>
          ))}
        </div>
        <p className="text-[10px] text-muted-foreground mt-1 italic">
          Phase 2에서 motor_node에 자동 적용 예정. 지금은 표시만.
        </p>
      </div>

      <div>
        <p className="text-[10px] text-muted-foreground font-mono mb-1">
          per-pose residual (BA 후 T_target←base 분산)
        </p>
        <PerPoseResidualTable
          rows={data.per_pose_residual}
          onRemove={onRemovePose}
          removeDisabled={removeDisabled}
        />
      </div>
    </div>
  );
}

