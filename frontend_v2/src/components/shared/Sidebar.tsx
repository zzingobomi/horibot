/**
 * Sidebar — robot enumeration + mode navigation.
 *
 * 옛 frontend Sidebar carry over (frontend_v2.md §2.3), v2 로 적응:
 *   - robots.yaml SSOT 자동 enumeration (useRobots) — capabilities 로 mode 링크.
 *   - Dashboard / World / Settings / Tasks 섹션은 해당 page/backend 부재로 제거
 *     (Step E+ 박히면 nav 추가). SIDEBAR_MODES 는 현재 ["move"].
 *   - 옛 shadcn theme token (bg-background 등) 대신 v2 raw zinc 색.
 */
import { useEffect, useState } from "react";
import { NavLink } from "react-router-dom";
import { Bot, ListTodo, PanelLeftClose, PanelLeftOpen, Share2 } from "lucide-react";
import { cn } from "@/lib/utils";
import { useRobots } from "@/hooks/useRobots";
import { useBridgeConnected } from "@/framework";

// Sidebar mode sub-route 자리 — capability 이름과 1:1 인 UI page mode.
const SIDEBAR_MODES: ReadonlySet<string> = new Set(["move", "calibrate"]);
const MODE_LABELS: Record<string, string> = {
  move: "Move",
  calibrate: "Calibrate",
  scan: "Scan",
  assets: "Assets",
};

/** robot capabilities → sidebar mode 링크 list. rgbd → scan mode (capability 이름과
 *  mode 이름이 다른 유일 케이스). assets(Waypoint) 는 capability 아니라 움직이는
 *  robot 공통 자산 계층 → 항상 노출. */
function sidebarModes(caps: string[]): string[] {
  const modes = caps.filter((c) => SIDEBAR_MODES.has(c));
  if (caps.includes("rgbd")) modes.push("scan");
  modes.push("assets");
  return modes;
}

const COLLAPSED_KEY = "omx.sidebar.collapsed";

export function Sidebar() {
  const { robots } = useRobots();
  const connected = useBridgeConnected();
  const [collapsed, setCollapsed] = useState<boolean>(() => {
    if (typeof window === "undefined") return false;
    return window.localStorage.getItem(COLLAPSED_KEY) === "1";
  });

  useEffect(() => {
    window.localStorage.setItem(COLLAPSED_KEY, collapsed ? "1" : "0");
  }, [collapsed]);

  return (
    <aside
      className={cn(
        "flex h-screen flex-col border-r border-zinc-800 bg-zinc-950 transition-[width] duration-200",
        collapsed ? "w-14" : "w-52",
      )}
    >
      {/* 로고 + 토글 */}
      <div
        className={cn(
          "flex items-center border-b border-zinc-800",
          collapsed ? "justify-center px-2 py-5" : "justify-between px-4 py-5",
        )}
      >
        {!collapsed && (
          <div className="min-w-0">
            <h1 className="text-lg font-semibold tracking-tight text-zinc-100">
              Horibot
            </h1>
            <p className="text-xs text-zinc-500">Robot Arm Controller</p>
          </div>
        )}
        <button
          onClick={() => setCollapsed((c) => !c)}
          title={collapsed ? "사이드바 펼치기" : "사이드바 접기"}
          className="flex h-8 w-8 shrink-0 items-center justify-center rounded-md text-zinc-400 hover:bg-zinc-800 hover:text-zinc-100"
        >
          {collapsed ? (
            <PanelLeftOpen className="h-4 w-4" />
          ) : (
            <PanelLeftClose className="h-4 w-4" />
          )}
        </button>
      </div>

      {/* Robots 섹션 — robots.yaml SSOT 자동 enumeration */}
      <nav className="flex-1 py-4 space-y-1 px-2 overflow-y-auto">
        {!collapsed && (
          <p className="px-3 pb-1 text-xs font-medium text-zinc-500 uppercase tracking-wider">
            Robots
          </p>
        )}
        {robots.map((r) =>
          collapsed ? (
            <NavLink
              key={r.id}
              to={`/robots/${r.id}`}
              title={r.id}
              className={({ isActive }) =>
                cn(
                  "flex items-center justify-center px-2 rounded-md py-2 text-sm transition-colors",
                  isActive
                    ? "bg-zinc-800 text-zinc-100 font-medium"
                    : "text-zinc-400 hover:bg-zinc-800 hover:text-zinc-100",
                )
              }
            >
              <Bot className="h-4 w-4 shrink-0" />
            </NavLink>
          ) : (
            <div key={r.id} className="mb-1">
              <div className="flex items-center gap-3 px-3 py-1.5 text-xs text-zinc-400">
                <Bot className="h-4 w-4 shrink-0" />
                <span className="flex-1 truncate">{r.id}</span>
              </div>
              {sidebarModes(r.capabilities ?? ["move"])
                .map((cap) => (
                  <NavLink
                    key={cap}
                    to={`/robots/${r.id}/${cap}`}
                    className={({ isActive }) =>
                      cn(
                        "flex items-center gap-2 ml-7 mr-2 px-3 py-1.5 rounded-md text-sm transition-colors",
                        isActive
                          ? "bg-zinc-800 text-zinc-100 font-medium"
                          : "text-zinc-400 hover:bg-zinc-800 hover:text-zinc-100",
                      )
                    }
                  >
                    <span>{MODE_LABELS[cap] ?? cap}</span>
                  </NavLink>
                ))}
            </div>
          ),
        )}

        {/* Tasks 섹션 — host-level(robot-agnostic, §2.7). robots 목록 아래 (v1 배치). */}
        <div className="pt-3">
          {!collapsed && (
            <p className="px-3 pb-1 text-xs font-medium text-zinc-500 uppercase tracking-wider">
              Tasks
            </p>
          )}
          <NavLink
            to="/tasks"
            title={collapsed ? "Tasks" : undefined}
            className={({ isActive }) =>
              cn(
                "flex items-center rounded-md py-2 text-sm transition-colors",
                collapsed ? "justify-center px-2" : "gap-3 px-3",
                isActive
                  ? "bg-zinc-800 text-zinc-100 font-medium"
                  : "text-zinc-400 hover:bg-zinc-800 hover:text-zinc-100",
              )
            }
          >
            <ListTodo className="h-4 w-4 shrink-0" />
            {!collapsed && <span>Tasks</span>}
          </NavLink>
        </div>
      </nav>

      {/* Dev 도구 — contract graph viewer (§6.1). 앱 기능 아니라 개발자 도구. */}
      <div className="border-t border-zinc-800 px-2 py-2">
        {!collapsed && (
          <p className="px-3 pb-1 text-xs font-medium text-zinc-500 uppercase tracking-wider">
            Dev
          </p>
        )}
        <NavLink
          to="/contract"
          title="Contract graph"
          className={({ isActive }) =>
            cn(
              "flex items-center gap-2 rounded-md px-3 py-1.5 text-sm transition-colors",
              collapsed && "justify-center px-2",
              isActive
                ? "bg-zinc-800 text-zinc-100 font-medium"
                : "text-zinc-400 hover:bg-zinc-800 hover:text-zinc-100",
            )
          }
        >
          <Share2 className="h-4 w-4 shrink-0" />
          {!collapsed && <span>Contract graph</span>}
        </NavLink>
      </div>

      {/* 연결 상태 */}
      {!collapsed && (
        <div className="border-t border-zinc-800 px-3 py-3">
          <div className="flex items-center gap-2 text-xs font-mono">
            <span
              className={cn(
                "h-2 w-2 rounded-full",
                connected ? "bg-emerald-400" : "bg-red-400",
              )}
            />
            <span className="text-zinc-400">
              {connected ? "online" : "offline"}
            </span>
          </div>
        </div>
      )}
    </aside>
  );
}
