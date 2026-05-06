export function MatrixTable({
  data,
  label,
}: {
  data: number[][];
  label: string;
}) {
  return (
    <div className="space-y-1">
      <p className="text-[10px] uppercase tracking-widest text-zinc-500 font-mono">
        {label}
      </p>
      <div className="font-mono text-[11px] leading-relaxed">
        {data.map((row, i) => (
          <div key={i} className="flex gap-2 text-zinc-300">
            {row.map((v, j) => (
              <span key={j} className="w-16 text-right tabular-nums">
                {v.toFixed(4)}
              </span>
            ))}
          </div>
        ))}
      </div>
    </div>
  );
}
