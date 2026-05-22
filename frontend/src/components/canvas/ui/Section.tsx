interface SectionProps {
  label: string;
  children: React.ReactNode;
}

export function Section({ label, children }: SectionProps) {
  return (
    <div className="px-3 py-2.5 border-b border-zinc-800/60 last:border-0">
      <p className="text-[9px] uppercase tracking-widest text-zinc-600 mb-2 font-mono">
        {label}
      </p>
      {children}
    </div>
  );
}
