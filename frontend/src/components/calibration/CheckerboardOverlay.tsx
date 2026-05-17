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
}
