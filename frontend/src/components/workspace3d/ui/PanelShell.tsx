import { useState } from "react";
import { ChevronDown, ChevronRight } from "lucide-react";

interface PanelShellProps {
  icon?: React.ReactNode;
  title: string;
  children: React.ReactNode;
  defaultCollapsed?: boolean;
}

export function PanelShell({
  icon,
  title,
  children,
  defaultCollapsed = false,
}: PanelShellProps) {
  const [collapsed, setCollapsed] = useState(defaultCollapsed);

  return (
    <div className="h-full flex flex-col bg-[#0d1117]/90 backdrop-blur-sm overflow-hidden">
      <button
        onClick={() => setCollapsed((p) => !p)}
        className="flex items-center gap-2 px-3 py-2 border-b border-zinc-700/50 hover:bg-zinc-800/60 transition-colors w-full text-left shrink-0"
      >
        {icon && <span className="text-zinc-400">{icon}</span>}
        <span className="text-[10px] font-mono font-bold tracking-widest uppercase text-zinc-300 flex-1">
          {title}
        </span>
        {collapsed ? (
          <ChevronRight className="w-3 h-3 text-zinc-500" />
        ) : (
          <ChevronDown className="w-3 h-3 text-zinc-500" />
        )}
      </button>

      {!collapsed && (
        <div className="overflow-y-auto flex-1 scrollbar-thin scrollbar-track-transparent scrollbar-thumb-zinc-700">
          {children}
        </div>
      )}
    </div>
  );
}
