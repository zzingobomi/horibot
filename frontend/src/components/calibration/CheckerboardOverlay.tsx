import type { HandEyePreview } from "./types";

interface Props {
  preview: HandEyePreview | null;
  stale: boolean;
}

export function CheckerboardOverlay({ preview, stale }: Props) {
  if (!preview) {
    return (
      <div className="absolute top-2 right-2 rounded bg-black/60 px-2 py-1 font-mono text-xs text-white/70">
        검출 대기중…
      </div>
    );
  }

  const [w, h] = preview.image_size ?? [1280, 720];
  const detected = preview.detected && !stale;
  const corners = preview.corners ?? [];
  const bbox = preview.bbox;
  const coverage = preview.coverage_ratio;

  const badgeClass = stale
    ? "bg-yellow-600/80"
    : detected
      ? "bg-emerald-600/85"
      : "bg-red-600/85";

  const badgeText = stale
    ? "신호 끊김"
    : detected
      ? `✓ 감지됨${coverage != null ? ` · ${(coverage * 100).toFixed(1)}%` : ""}`
      : "✗ 미감지";

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
            stroke="rgba(16,185,129,0.55)"
            strokeWidth={3}
            strokeDasharray="10 6"
          />
        )}
        {detected && corners.length > 0 && (
          <>
            <polyline
              points={corners.map(([x, y]) => `${x},${y}`).join(" ")}
              fill="none"
              stroke="rgba(16,185,129,0.7)"
              strokeWidth={2}
            />
            {corners.map(([x, y], i) => (
              <circle
                key={i}
                cx={x}
                cy={y}
                r={5}
                fill="rgba(52,211,153,0.95)"
                stroke="white"
                strokeWidth={1}
              />
            ))}
          </>
        )}
      </svg>

      <div
        className={`absolute top-2 right-2 rounded px-2 py-1 font-mono text-xs text-white ${badgeClass}`}
      >
        {badgeText}
      </div>
    </>
  );
}
