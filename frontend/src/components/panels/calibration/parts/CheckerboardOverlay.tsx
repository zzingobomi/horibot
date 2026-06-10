import { useState } from "react";
import type { HandEyePreview } from "./types";

interface Props {
  preview: HandEyePreview | null;
  stale: boolean;
}

// PnP 권장 tilt 범위 (docs/calibration_workflow.md §2) — 30~70° 안에서 캡처.
// hysteresis 2° margin 으로 chatter 방지.
const TILT_ENTER_MIN = 30;
const TILT_ENTER_MAX = 70;
const TILT_EXIT_MIN = 28;
const TILT_EXIT_MAX = 72;

export function CheckerboardOverlay({ preview, stale }: Props) {
  const tilt = preview?.tilt_deg ?? null;
  const [tiltOk, setTiltOk] = useState(false);
  const [prevTilt, setPrevTilt] = useState<number | null>(null);

  if (tilt !== prevTilt) {
    setPrevTilt(tilt);
    if (tilt == null) {
      if (tiltOk) setTiltOk(false);
    } else if (tiltOk) {
      if (tilt < TILT_EXIT_MIN || tilt > TILT_EXIT_MAX) setTiltOk(false);
    } else {
      if (tilt >= TILT_ENTER_MIN && tilt <= TILT_ENTER_MAX) setTiltOk(true);
    }
  }

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

  let badgeClass: string;
  let badgeText: string;
  if (stale) {
    badgeClass = "bg-red-500/15 border border-red-500/40 text-red-300";
    badgeText = "캡처 금지 · 신호 끊김";
  } else if (!preview.detected) {
    badgeClass = "bg-red-500/15 border border-red-500/40 text-red-300";
    badgeText = "캡처 금지 · 미검출";
  } else if (!tiltOk) {
    badgeClass = "bg-red-500/15 border border-red-500/40 text-red-300";
    // hysteresis 의 EXIT 범위가 아닌 사용자 직관 기준 (ENTER) 으로 판정.
    // tilt < 30° 정면 / tilt > 70° 비스듬 / 사이는 OK (tiltOk=true 라 안 들어옴).
    const reason =
      tilt == null
        ? ""
        : tilt < TILT_ENTER_MIN
        ? ` · tilt ${tilt.toFixed(0)}° 너무 정면`
        : ` · tilt ${tilt.toFixed(0)}° 너무 비스듬`;
    badgeText = `캡처 금지${reason}`;
  } else {
    badgeClass = "bg-emerald-500/15 border border-emerald-500/40 text-emerald-300";
    const tiltStr = tilt != null ? ` · tilt ${tilt.toFixed(0)}°` : "";
    badgeText = `캡처 가능${tiltStr}`;
  }

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
