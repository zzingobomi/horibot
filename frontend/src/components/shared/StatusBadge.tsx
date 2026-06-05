import { AlertCircle, CheckCircle2 } from "lucide-react";

export function StatusBadge({ ok, label }: { ok: boolean; label: string }) {
  return (
    <div
      className={`flex items-center gap-1.5 text-xs font-mono ${
        ok ? "text-emerald-400" : "text-zinc-500"
      }`}
    >
      {ok ? (
        <CheckCircle2 className="w-3.5 h-3.5" />
      ) : (
        <AlertCircle className="w-3.5 h-3.5" />
      )}
      {label}
    </div>
  );
}
