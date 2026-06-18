import type { HandEyePreview } from "./types";

interface Props {
  preview: HandEyePreview | null;
  stale: boolean;
}

// Phase 1 Traffic Light — verdict 는 backend 가 계산 (검출+tilt+diversity 종합,
// handeye_ux_solver_v3_plan.md §5). frontend 는 색 + 사유만 표시.
const VERDICT_STYLE: Record<string, string> = {
  green: "bg-emerald-500/15 border border-emerald-500/40 text-emerald-300",
  yellow: "bg-amber-500/15 border border-amber-500/40 text-amber-300",
  red: "bg-red-500/15 border border-red-500/40 text-red-300",
};
const VERDICT_HEAD: Record<string, string> = {
  green: "🟢 캡처 권장",
  yellow: "🟡 캡처 가능",
  red: "🔴 캡처 금지",
};

export function CaptureGuideOverlay({ preview, stale }: Props) {
  if (!preview) {
    return (
      <div className="absolute top-2 right-2 rounded border border-zinc-700/60 bg-zinc-900/70 px-2 py-1 font-mono text-[11px] text-zinc-400 backdrop-blur-sm">
        검출 대기중…
      </div>
    );
  }

  const [w, h] = preview.image_size ?? [1280, 720];
  const detected = preview.detected && !stale;
  const corners = preview.corners ?? [];
  const markers = preview.markers ?? [];
  const bbox = preview.bbox;
  const tilt = preview.tilt_deg ?? null;

  // 신호 끊김은 frontend-only (timeout). 그 외엔 backend verdict 사용.
  const verdict = stale ? "red" : (preview.capture_verdict ?? "red");
  const reasons = stale
    ? ["신호 끊김"]
    : preview.capture_reasons ?? (preview.detected ? [] : ["미검출"]);
  const badgeClass = VERDICT_STYLE[verdict] ?? VERDICT_STYLE.red;
  const tiltStr =
    verdict !== "red" && tilt != null ? ` · tilt ${tilt.toFixed(0)}°` : "";
  const reasonStr = reasons.length ? ` · ${reasons.join(" · ")}` : "";
  const badgeText = `${VERDICT_HEAD[verdict] ?? VERDICT_HEAD.red}${tiltStr}${reasonStr}`;

  return (
    <>
      <svg
        className="pointer-events-none absolute inset-0 h-full w-full"
        viewBox={`0 0 ${w} ${h}`}
        preserveAspectRatio="xMidYMid meet"
      >
        {detected && bbox && (
          <rect
            x={bbox[0]}
            y={bbox[1]}
            width={bbox[2]}
            height={bbox[3]}
            fill="none"
            stroke="rgba(16,185,129,0.45)"
            strokeWidth={2}
            strokeDasharray="10 6"
          />
        )}
        {markers.map((m) => {
          const pts = m.corners.map(([x, y]) => `${x},${y}`).join(" ");
          const cx =
            m.corners.reduce((s, [x]) => s + x, 0) / m.corners.length;
          const cy =
            m.corners.reduce((s, [, y]) => s + y, 0) / m.corners.length;
          return (
            <g key={m.id}>
              <polygon
                points={pts}
                fill="rgba(16,185,129,0.12)"
                stroke="rgba(16,185,129,0.85)"
                strokeWidth={2}
              />
              <text
                x={cx}
                y={cy}
                fill="rgba(220,252,231,0.95)"
                stroke="rgba(6,78,59,0.7)"
                strokeWidth={3}
                paintOrder="stroke"
                fontFamily="JetBrains Mono, monospace"
                fontSize={14}
                fontWeight={700}
                textAnchor="middle"
                dominantBaseline="central"
              >
                {m.id}
              </text>
            </g>
          );
        })}
        {detected &&
          corners.map(([x, y], i) => (
            <circle
              key={i}
              cx={x}
              cy={y}
              r={4}
              fill="rgba(52,211,153,0.95)"
              stroke="rgba(16,185,129,0.6)"
              strokeWidth={1}
            />
          ))}
      </svg>

      <div
        className={`absolute top-2 right-2 rounded px-2 py-1 font-mono text-[11px] backdrop-blur-sm ${badgeClass}`}
      >
        {badgeText}
      </div>
    </>
  );
}
