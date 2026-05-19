import { Trash2 } from "lucide-react";
import type { PoseMeta } from "./types";

export function HandEyePoseList({
  poses,
  onRemove,
  disabled,
}: {
  poses: PoseMeta[];
  onRemove: (id: number) => void;
  disabled: boolean;
}) {
  if (poses.length === 0) {
    return (
      <p className="text-xs text-muted-foreground italic">
        포즈 없음 — 첫 자세를 캡처하세요.
      </p>
    );
  }

  return (
    <div className="rounded-md bg-muted/40 p-2 max-h-48 overflow-y-auto">
      <div className="text-[10px] text-muted-foreground font-mono mb-1 px-1">
        {poses.length}개 누적
      </div>
      <ul className="space-y-0.5">
        {poses.map((p) => {
          const ts = new Date(p.timestamp * 1000);
          const time = ts.toLocaleTimeString("ko-KR", { hour12: false });
          return (
            <li
              key={p.id}
              className="flex items-center justify-between gap-2 px-2 py-1 rounded text-xs font-mono hover:bg-muted/60"
            >
              <span className="text-muted-foreground">#{p.id}</span>
              <span className="flex-1 text-[11px]">{time}</span>
              <button
                onClick={() => onRemove(p.id)}
                disabled={disabled}
                title="삭제"
                className="text-muted-foreground hover:text-destructive disabled:opacity-40"
              >
                <Trash2 className="w-3.5 h-3.5" />
              </button>
            </li>
          );
        })}
      </ul>
    </div>
  );
}
