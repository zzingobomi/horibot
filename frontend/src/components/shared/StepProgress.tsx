export function StepProgress({
  currentStep,
  totalSteps,
  currentLabel,
}: {
  currentStep: number;
  totalSteps: number;
  currentLabel: string;
}) {
  if (totalSteps === 0)
    return <p className="font-mono text-xs text-zinc-600">—</p>;

  const pct = Math.round((currentStep / totalSteps) * 100);

  return (
    <div className="space-y-2">
      <div className="flex justify-between items-center">
        <span className="font-mono text-xs text-zinc-300">
          {currentLabel || "—"}
        </span>
        <span className="font-mono text-xs text-zinc-500">
          {currentStep} / {totalSteps}
        </span>
      </div>
      <div className="h-1.5 w-full bg-zinc-700 rounded-full overflow-hidden">
        <div
          className="h-full bg-emerald-500 rounded-full transition-all duration-300"
          style={{ width: `${pct}%` }}
        />
      </div>
    </div>
  );
}
