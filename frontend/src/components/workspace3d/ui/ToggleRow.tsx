import { Eye, EyeOff } from "lucide-react";

export function ToggleRow({
  label,
  checked,
  onChange,
  accentColor = "bg-sky-500",
}: {
  label: string;
  checked: boolean;
  onChange: (v: boolean) => void;
  accentColor?: string;
}) {
  return (
    <button
      onClick={() => onChange(!checked)}
      className={`flex items-center justify-between w-full px-2 py-1.5 rounded text-xs font-mono transition-colors
        ${
          checked
            ? "bg-zinc-800 text-zinc-100"
            : "bg-zinc-900 text-zinc-500 hover:bg-zinc-850"
        }`}
    >
      <span className="flex items-center gap-2">
        <span
          className={`w-2 h-2 rounded-full ${
            checked ? accentColor : "bg-zinc-700"
          }`}
        />
        {label}
      </span>
      {checked ? (
        <Eye className="w-3.5 h-3.5" />
      ) : (
        <EyeOff className="w-3.5 h-3.5" />
      )}
    </button>
  );
}
