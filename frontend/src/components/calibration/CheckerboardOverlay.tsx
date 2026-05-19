import { useState } from "react";
import type { HandEyePreview } from "./types";

interface Props {
  preview: HandEyePreview | null;
  stale: boolean;
}

// tilt 게이트: 진입은 보수적으로(12~73°), 이탈은 관대하게(10/75° 밖) — 경계에서 깜빡임 방지
const TILT_ENTER_MIN = 12;
const TILT_ENTER_MAX = 73;
const TILT_EXIT_MIN = 10;
const TILT_EXIT_MAX = 75;

export function CheckerboardOverlay({ preview, stale }: Props) {
  const tilt = preview?.tilt_deg ?? null;
  const [tiltOk, setTiltOk] = useState(false);
  const [prevTilt, setPrevTilt] = useState<number | null>(null);

  // tilt 값이 바뀐 프레임에만 hysteresis 게이트를 재평가. useEffect 안에서
  // setState하면 effect→render 캐스케이드가 생기므로 렌더 중에 직접 처리.
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

  // 캡처 게이트: 검출 OK + tilt 정상 영역 → 캡처 가능. 그 외 전부 캡처 금지.
  // (PnP two-solution ambiguity 영역인 <10°와 코너 부정확 영역인 >75°만 hard block —
  //  그 외 경계 자세는 다양성 확보 위해 허용하고 사후 outlier 제거로 거름.)
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
