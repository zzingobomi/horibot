import type {
  CalibThresholds,
  CoachMessage,
  CoachReport,
  ComputeData,
  JointOffsetDelta,
  LinkRotDelta,
  LinkTransDelta,
  PerPoseResidual,
  SagOffsetDelta,
} from "./types";

function makeSigmaRotColor(thr: CalibThresholds) {
  return (deg: number): string =>
    deg < thr.sigma_rot_good_deg
      ? "text-green-500"
      : deg < thr.sigma_rot_warn_deg
      ? "text-amber-500"
      : "text-red-500";
}

function makeSigmaTColor(thr: CalibThresholds) {
  return (mm: number): string =>
    mm < thr.sigma_t_good_mm
      ? "text-green-500"
      : mm < thr.sigma_t_warn_mm
      ? "text-amber-500"
      : "text-red-500";
}

function verdictStyle(verdict: CoachReport["verdict"]): {
  label: string;
  border: string;
  badge: string;
} {
  switch (verdict) {
    case "good":
      return {
        label: "정확도 충분",
        border: "border-green-500/40 bg-green-500/5",
        badge: "bg-green-500 text-white",
      };
    case "needs_work":
      return {
        label: "보완 필요",
        border: "border-amber-500/40 bg-amber-500/5",
        badge: "bg-amber-500 text-white",
      };
    case "bad":
    default:
      return {
        label: "정확도 부족",
        border: "border-red-500/40 bg-red-500/5",
        badge: "bg-red-500 text-white",
      };
  }
}

function messageColor(level: CoachMessage["level"]): string {
  switch (level) {
    case "success":
      return "text-green-600 dark:text-green-400";
    case "warn":
      return "text-amber-600 dark:text-amber-400";
    case "error":
      return "text-red-600 dark:text-red-400";
    case "info":
    default:
      return "text-foreground/80";
  }
}

function CoachPanel({ report }: { report: CoachReport }) {
  const v = verdictStyle(report.verdict);
  return (
    <div className={`rounded-md border p-3 space-y-2 ${v.border}`}>
      <div className="flex items-center gap-2">
        <span
          className={`text-[10px] font-semibold px-2 py-0.5 rounded ${v.badge}`}
        >
          {v.label}
        </span>
        <span className="text-[10px] text-muted-foreground font-mono">
          coach 진단
        </span>
      </div>
      {report.messages.length === 0 ? (
        <p className="text-xs text-muted-foreground italic">의견 없음.</p>
      ) : (
        <ul className="space-y-1">
          {report.messages.map((m, i) => (
            <li
              key={i}
              className={`text-[11px] leading-snug ${messageColor(m.level)}`}
            >
              · {m.text}
            </li>
          ))}
        </ul>
      )}
    </div>
  );
}

function PerPoseResidualTable({
  rows,
  sigmaRotColor,
}: {
  rows: PerPoseResidual[];
  sigmaRotColor: (deg: number) => string;
}) {
  return (
    <div className="max-h-32 overflow-y-auto">
      <table className="w-full text-[11px] font-mono">
        <tbody>
          {rows.map((r) => {
            // excluded 행: 회색 + 취소선 + "excluded" 뱃지. 잔차는 1차 BA 값
            // (왜 빠졌는지 표시). 정확도 σ 계산엔 들어가지 않음.
            if (r.excluded) {
              return (
                <tr key={r.id} className="text-muted-foreground/60">
                  <td className="py-0.5 line-through">#{r.id}</td>
                  <td className="py-0.5 text-right line-through">
                    {r.drot_deg.toFixed(3)}°
                  </td>
                  <td className="py-0.5 text-right line-through">
                    {r.dt_mm.toFixed(1)}mm
                  </td>
                  <td className="py-0.5 text-right">
                    <span className="text-[9px] px-1 py-0.5 rounded bg-muted text-muted-foreground">
                      excluded
                    </span>
                  </td>
                </tr>
              );
            }
            return (
              <tr key={r.id}>
                <td className="py-0.5 text-muted-foreground">#{r.id}</td>
                <td className={`py-0.5 text-right ${sigmaRotColor(r.drot_deg)}`}>
                  {r.drot_deg.toFixed(3)}°
                </td>
                <td className="py-0.5 text-right text-muted-foreground">
                  {r.dt_mm.toFixed(1)}mm
                </td>
                <td />
              </tr>
            );
          })}
        </tbody>
      </table>
    </div>
  );
}

export function ComputePreview({
  data,
  thresholds,
}: {
  data: ComputeData;
  thresholds: CalibThresholds;
}) {
  const sigmaRotColor = makeSigmaRotColor(thresholds);
  const sigmaTColor = makeSigmaTColor(thresholds);
  const usedCount = data.pose_count - data.excluded_pose_ids.length;

  return (
    <div className="space-y-3 text-xs">
      <CoachPanel report={data.coach} />

      <div className="rounded-md bg-muted p-2 space-y-1 font-mono">
        <div className="flex justify-between">
          <span className="text-muted-foreground">method</span>
          <span>{data.method}</span>
        </div>
        {!data.ba_converged && (
          <div className="text-[10px] text-amber-500">
            BA 미수렴 — cv2 seed 결과로 fallback
          </div>
        )}
        <div className="flex justify-between">
          <span className="text-muted-foreground">poses</span>
          <span>
            {usedCount}/{data.pose_count}
            {data.excluded_pose_ids.length > 0 && (
              <span className="text-muted-foreground">
                {" "}
                ({data.excluded_pose_ids.length} excluded)
              </span>
            )}
          </span>
        </div>
        <div className="flex justify-between">
          <span className="text-muted-foreground">t [mm]</span>
          <span>
            {data.t_cam2gripper.map((v) => (v * 1000).toFixed(1)).join(", ")}
          </span>
        </div>
        <div className="flex justify-between">
          <span className="text-muted-foreground">σ_rot (RMS)</span>
          <span className={sigmaRotColor(data.sigma_rot_deg)}>
            {data.sigma_rot_deg.toFixed(3)}°
          </span>
        </div>
        <div className="flex justify-between">
          <span className="text-muted-foreground">σ_t (RMS)</span>
          <span className={sigmaTColor(data.sigma_t_mm)}>
            {data.sigma_t_mm.toFixed(1)}mm
          </span>
        </div>
      </div>

      <div>
        <p className="text-[10px] text-muted-foreground font-mono mb-1">
          cv2 method self-consistency (입력 노이즈 진단)
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
          per-pose residual (BA가 추정한 보드 포즈 기준)
        </p>
        <PerPoseResidualTable
          rows={data.per_pose_residual}
          sigmaRotColor={sigmaRotColor}
        />
      </div>

      {data.joint_offset_estimated &&
        data.joint_offset_delta.length > 0 && (
          <JointOffsetTable rows={data.joint_offset_delta} />
        )}

      {data.link_offset_estimated && data.link_trans_delta.length > 0 && (
        <LinkTransTable rows={data.link_trans_delta} />
      )}
      {data.link_offset_estimated && data.link_rot_delta.length > 0 && (
        <LinkRotTable rows={data.link_rot_delta} />
      )}
      {data.sag_offset_estimated && data.sag_offset_delta.length > 0 && (
        <SagTable rows={data.sag_offset_delta} />
      )}
    </div>
  );
}

function JointOffsetTable({ rows }: { rows: JointOffsetDelta[] }) {
  // BA가 추정한 delta — COMMIT 시 기존 캘에 합산해 저장됨.
  // |offset| > 2°이면 모터 조립이 의심스러우니 강조.
  return (
    <div>
      <p className="text-[10px] text-muted-foreground font-mono mb-1">
        joint offset delta — COMMIT 시 cumulative 저장
      </p>
      <table className="w-full text-[11px] font-mono">
        <tbody>
          {rows.map((r) => {
            const mag = Math.abs(r.offset_deg);
            const cls =
              mag < 0.5
                ? "text-muted-foreground"
                : mag < 2.0
                ? "text-foreground"
                : "text-amber-500";
            const sign = r.offset_deg >= 0 ? "+" : "";
            return (
              <tr key={r.motor_id}>
                <td className="py-0.5 text-muted-foreground">
                  J{r.motor_id}
                </td>
                <td className={`py-0.5 text-right ${cls}`}>
                  {sign}
                  {r.offset_deg.toFixed(3)}°
                </td>
              </tr>
            );
          })}
        </tbody>
      </table>
    </div>
  );
}

/** link translation (확장 BA). |값| > 20mm면 gauge freedom 의심 — 노랑. */
function linkTransColor(mm: number): string {
  const mag = Math.abs(mm);
  if (mag < 5) return "text-muted-foreground";
  if (mag < 20) return "text-foreground";
  return "text-amber-500";
}

function fmtSigned(v: number, frac: number): string {
  return (v >= 0 ? "+" : "") + v.toFixed(frac);
}

function LinkTransTable({ rows }: { rows: LinkTransDelta[] }) {
  return (
    <div>
      <p className="text-[10px] text-muted-foreground font-mono mb-1">
        link translation delta (mm) — joint origin xyz 보정, COMMIT 시 누적
      </p>
      <table className="w-full text-[11px] font-mono">
        <tbody>
          {rows.map((r) => (
            <tr key={r.motor_id}>
              <td className="py-0.5 text-muted-foreground">J{r.motor_id}</td>
              <td className={`py-0.5 text-right ${linkTransColor(r.x_mm)}`}>
                x {fmtSigned(r.x_mm, 2)}
              </td>
              <td className={`py-0.5 text-right ${linkTransColor(r.y_mm)}`}>
                y {fmtSigned(r.y_mm, 2)}
              </td>
              <td className={`py-0.5 text-right ${linkTransColor(r.z_mm)}`}>
                z {fmtSigned(r.z_mm, 2)}
              </td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
}

/** link rotation (확장 BA). |값| > 2°면 small-angle 가정 한계 — 노랑. */
function linkRotColor(deg: number): string {
  const mag = Math.abs(deg);
  if (mag < 0.5) return "text-muted-foreground";
  if (mag < 2.0) return "text-foreground";
  return "text-amber-500";
}

function LinkRotTable({ rows }: { rows: LinkRotDelta[] }) {
  return (
    <div>
      <p className="text-[10px] text-muted-foreground font-mono mb-1">
        link rotation delta (deg) — joint origin rpy 보정 (small-angle)
      </p>
      <table className="w-full text-[11px] font-mono">
        <tbody>
          {rows.map((r) => (
            <tr key={r.motor_id}>
              <td className="py-0.5 text-muted-foreground">J{r.motor_id}</td>
              <td className={`py-0.5 text-right ${linkRotColor(r.rx_deg)}`}>
                rx {fmtSigned(r.rx_deg, 3)}
              </td>
              <td className={`py-0.5 text-right ${linkRotColor(r.ry_deg)}`}>
                ry {fmtSigned(r.ry_deg, 3)}
              </td>
              <td className={`py-0.5 text-right ${linkRotColor(r.rz_deg)}`}>
                rz {fmtSigned(r.rz_deg, 3)}
              </td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
}

/** sag 최대 각 색깔 — |값| > 5°면 비현실적 처짐 의심 — 노랑.
 * XL430 11V + DIY 5축에서 J2/J3 sag ~2-4° 정도가 정상 범위. */
function sagMaxColor(deg: number): string {
  const mag = Math.abs(deg);
  if (mag < 1.0) return "text-muted-foreground";
  if (mag < 5.0) return "text-foreground";
  return "text-amber-500";
}

function SagTable({ rows }: { rows: SagOffsetDelta[] }) {
  return (
    <div>
      <p className="text-[10px] text-muted-foreground font-mono mb-1">
        sag stiffness — 자세 의존 중력 처짐 보정 (J2, J3). COMMIT 시 누적, 즉시 적용
      </p>
      <table className="w-full text-[11px] font-mono">
        <tbody>
          {rows.map((r) => (
            <tr key={r.motor_id}>
              <td className="py-0.5 text-muted-foreground">J{r.motor_id}</td>
              <td className="py-0.5 text-right text-foreground">
                k {fmtSigned(r.k_rad_per_m, 5)}
              </td>
              <td className={`py-0.5 text-right ${sagMaxColor(r.max_sag_deg)}`}>
                max sag {fmtSigned(r.max_sag_deg, 2)}°
              </td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
}
