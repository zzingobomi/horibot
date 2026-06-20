import type { PoseMeta } from "@/domain/stores/calibration";

/**
 * Capture pose 누적 list — capture-only 시나리오.
 *
 * 표시: pose_index (0-based) + tilt_deg. 상세 (reproj_rms_px, residual, weight)
 * 는 자리 X — offline 분석 스크립트 시점에 자리 자리 자리.
 */
export function HandEyePoseList({ poses }: { poses: PoseMeta[] }) {
  if (poses.length === 0) {
    return (
      <p className="text-[11px] text-zinc-500 italic font-mono">
        포즈 없음 — 첫 자세를 캡처하세요.
      </p>
    );
  }

  return (
    <div className="rounded border border-zinc-800/60 bg-zinc-900/40 p-2 max-h-48 overflow-y-auto">
      <div className="text-[10px] text-zinc-500 font-mono mb-1 px-1 uppercase tracking-wide">
        {poses.length}장 누적
      </div>
      <ul className="space-y-0.5">
        {poses.map((p) => (
          <li
            key={p.pose_index}
            className="flex items-center gap-2 px-2 py-1 rounded text-[11px] font-mono hover:bg-zinc-800/40 text-zinc-300"
          >
            <span className="text-zinc-500 w-8 tabular-nums">#{p.pose_index}</span>
            <span className="flex-1 tabular-nums">
              tilt {p.tilt_deg != null ? `${p.tilt_deg.toFixed(1)}°` : "—"}
            </span>
          </li>
        ))}
      </ul>
    </div>
  );
}
