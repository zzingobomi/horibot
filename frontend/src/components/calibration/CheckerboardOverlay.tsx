import { useState } from "react";
import type { HandEyePreview } from "./types";

interface Props {
  preview: HandEyePreview | null;
  stale: boolean;
}

const TILT_ENTER_MIN = 12;
const TILT_ENTER_MAX = 73;
const TILT_EXIT_MIN = 10;
const TILT_EXIT_MAX = 75;

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
      <div className="absolute top-2 right-2 rounded bg-black/60 px-2 py-1 font-mono text-xs text-white/70">
        검출 대기중…
      </div>
    );
  }

  const [w, h] = preview.image_size ?? [1280, 720];
  const detected = preview.detected && !stale;
  const corners = preview.corners ?? [];
  const bbox = preview.bbox;

  let badgeClass: string;
  let badgeText: string;
  if (stale) {
    badgeClass = "bg-red-600/85";
    badgeText = "캡처 금지 · 신호 끊김";
  } else if (!preview.detected) {
    badgeClass = "bg-red-600/85";
    badgeText = "캡처 금지 · 미검출";
  } else if (!tiltOk) {
    badgeClass = "bg-red-600/85";
    const reason =
      tilt == null
        ? ""
        : tilt < TILT_EXIT_MIN
        ? ` · tilt ${tilt.toFixed(0)}° 너무 정면`
        : ` · tilt ${tilt.toFixed(0)}° 너무 비스듬`;
    badgeText = `캡처 금지${reason}`;
  } else {
    badgeClass = "bg-emerald-600/85";
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
