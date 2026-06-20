import type { HandEyePreview } from "@/domain/stores/calibration";

interface Props {
  preview: HandEyePreview | null;
  stale: boolean;
  imageWidth?: number;
  imageHeight?: number;
}

// Traffic light verdict — backend 가 계산 (검출 + tilt + pose diversity 종합).
// frontend 는 색 + 사유만 표시 (capture_verdict / capture_reasons).
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

export function CaptureGuideOverlay({
  preview,
  stale,
  imageWidth = 1280,
  imageHeight = 720,
}: Props) {
  if (!preview) {
    return (
      <div className="absolute top-2 right-2 rounded border border-zinc-700/60 bg-zinc-900/70 px-2 py-1 font-mono text-[11px] text-zinc-400 backdrop-blur-sm">
        검출 대기중…
      </div>
    );
  }

  const detected = preview.detected && !stale;
  const corners = preview.corners_2d ?? [];
  const markers = preview.marker_outlines ?? [];
  const tilt = preview.tilt_deg;

  const verdict = stale ? "red" : preview.capture_verdict;
  const reasons = stale
    ? ["신호 끊김"]
    : preview.capture_reasons.length
      ? preview.capture_reasons
      : preview.detected
        ? []
        : ["미검출"];
  const badgeClass = VERDICT_STYLE[verdict] ?? VERDICT_STYLE.red;
  const tiltStr =
    verdict !== "red" && tilt != null ? ` · tilt ${tilt.toFixed(0)}°` : "";
  const reasonStr = reasons.length ? ` · ${reasons.join(" · ")}` : "";
  const badgeText = `${VERDICT_HEAD[verdict] ?? VERDICT_HEAD.red}${tiltStr}${reasonStr}`;

  return (
    <>
      <svg
        className="pointer-events-none absolute inset-0 h-full w-full"
        viewBox={`0 0 ${imageWidth} ${imageHeight}`}
        preserveAspectRatio="xMidYMid meet"
      >
        {markers.map((m, i) => {
          const pts = m.map(([x, y]) => `${x},${y}`).join(" ");
          return (
            <polygon
              key={i}
              points={pts}
              fill="rgba(16,185,129,0.12)"
              stroke="rgba(16,185,129,0.85)"
              strokeWidth={2}
            />
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
