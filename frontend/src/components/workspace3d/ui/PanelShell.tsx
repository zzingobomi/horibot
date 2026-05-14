import { useEffect, useRef, useState } from "react";
import { ChevronDown, ChevronRight } from "lucide-react";
import {
  PANEL_HEADER_HEIGHT,
  loadCollapsed,
  saveCollapsed,
} from "@/lib/workspaceLayout";

interface PanelGroupApi {
  setSize: (event: { height?: number; width?: number }) => void;
}
interface PanelApiLike {
  group: { api: PanelGroupApi };
}

interface PanelShellProps {
  icon?: React.ReactNode;
  title: string;
  panelId: string;
  api?: PanelApiLike;
  children: React.ReactNode;
  expandedHeight?: number;
}

export function PanelShell({
  icon,
  title,
  panelId,
  api,
  children,
  expandedHeight = 280,
}: PanelShellProps) {
  const [collapsed, setCollapsed] = useState(() => loadCollapsed(panelId));
  // 직전 expanded 높이 기억 — 같은 세션 내에서 collapse↔expand 토글 시 사용자가
  // 키웠던 크기로 정확히 복원
  const lastExpandedRef = useRef<number>(expandedHeight);
  const rootRef = useRef<HTMLDivElement | null>(null);

  // mount 시 저장된 collapsed 상태에 맞춰 floating 높이 동기화
  useEffect(() => {
    if (!api) return;
    if (collapsed) {
      api.group.api.setSize({ height: PANEL_HEADER_HEIGHT });
    }
    // expanded 면 dockview 가 layout JSON 으로 이미 복원한 높이를 그대로 사용
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  const toggle = () => {
    const next = !collapsed;
    setCollapsed(next);
    saveCollapsed(panelId, next);
    if (!api) return;
    if (next) {
      const current = rootRef.current?.clientHeight ?? expandedHeight;
      if (current > PANEL_HEADER_HEIGHT) lastExpandedRef.current = current;
      api.group.api.setSize({ height: PANEL_HEADER_HEIGHT });
    } else {
      api.group.api.setSize({ height: lastExpandedRef.current });
    }
  };

  return (
    <div
      ref={rootRef}
      className="h-full flex flex-col bg-[#0d1117]/90 backdrop-blur-sm overflow-hidden"
    >
      <button
        onClick={toggle}
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
