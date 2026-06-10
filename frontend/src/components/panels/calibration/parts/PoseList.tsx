import type { PoseMeta } from "./types";

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
        {poses.length}개 누적
      </div>
      <ul className="space-y-0.5">
        {poses.map((p) => {
          const ts = new Date(p.timestamp * 1000);
          const time = ts.toLocaleTimeString("ko-KR", { hour12: false });
          return (
            <li
              key={p.id}
              className="flex items-center gap-2 px-2 py-1 rounded text-[11px] font-mono hover:bg-zinc-800/40 text-zinc-300"
            >
              <span className="text-zinc-500 w-8 tabular-nums">#{p.id}</span>
              <span className="flex-1 tabular-nums">{time}</span>
            </li>
          );
        })}
      </ul>
    </div>
  );
}
