import { Trash2 } from "lucide-react";
import type { PoseMeta } from "./types";

export function HandEyePoseList({
  poses,
  onRemove,
  disabled,
}: {
  poses: PoseMeta[];
  onRemove: (index: number) => void;
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
    </div>
  );
}
