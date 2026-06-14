import { useCallback, useEffect, useState } from "react";
import { NavLink } from "react-router-dom";
import {
  Gamepad2,
  Settings,
  Home,
  Moon,
  Power,
  PanelLeftClose,
  PanelLeftOpen,
  Bot,
  Globe,
  ListChecks,
} from "lucide-react";
import { ConnectionStatus } from "@/components/shared/ConnectionStatus";
import { cn } from "@/lib/utils";
import { useService } from "@/framework";
import { ServiceKey } from "@/constants/topics";
import { useRobots } from "@/hooks/useRobots";
import { useTasks } from "@/hooks/useTasks";
import { loadPose } from "@/lib/robot/robotPoses";
import type { RobotCapability } from "@/types/robot";

const navItems = [
  { to: "/", label: "Dashboard", icon: Gamepad2 },
  { to: "/world", label: "World", icon: Globe },
  { to: "/settings", label: "Settings", icon: Settings },
];

// 단일 source — backend RobotCapability Literal 과 sync.
const CAPABILITY_LABELS: Record<RobotCapability, string> = {
  move: "Move",
  calibrate: "Calibrate",
  scan: "Scan",
};

const COLLAPSED_KEY = "omx.sidebar.collapsed";

export function Sidebar() {
  // Sidebar 는 global (URL param 못 받음) — backend default robot
  // (RobotRegistry().default(), enabled=true 첫 robot) 의 service / pose 사용.
  // multi-robot 시 robot 페이지의 RobotStatePanel 이 명시 robot 의 동등 컨트롤 보유.
  const { robots, defaultId } = useRobots();
  const cfgSvc = useService(ServiceKey.MOTOR_GET_CONFIG, defaultId);
  const enableSvc = useService(ServiceKey.MOTOR_ENABLE, defaultId);
  const moveJ = useService(ServiceKey.MOTION_MOVE_J, defaultId);
  const torqueEnabled = cfgSvc.data?.torque_enabled ?? false;

  const { tasks } = useTasks();
  const [collapsed, setCollapsed] = useState<boolean>(() => {
    if (typeof window === "undefined") return false;
    return window.localStorage.getItem(COLLAPSED_KEY) === "1";
  });

  useEffect(() => {
    window.localStorage.setItem(COLLAPSED_KEY, collapsed ? "1" : "0");
  }, [collapsed]);

  const goHome = useCallback(async () => {
    const pose = await loadPose(defaultId, "home");
    await moveJ.call({ joints: pose });
  }, [defaultId, moveJ]);

  const goRest = useCallback(async () => {
    const pose = await loadPose(defaultId, "rest");
    await moveJ.call({ joints: pose });
  }, [defaultId, moveJ]);

  const toggleTorque = useCallback(async () => {
    const next = !torqueEnabled;
    const res = await enableSvc.call({ enable: next });
    if (res.success) await cfgSvc.call({});
  }, [torqueEnabled, enableSvc, cfgSvc]);

  return (
    <aside
      className={cn(
        "flex h-screen flex-col border-r bg-background transition-[width] duration-200",
        collapsed ? "w-14" : "w-52",
      )}
    >
      {/* 로고 + 토글 */}
      <div
        className={cn(
          "flex items-center border-b",
          collapsed ? "justify-center px-2 py-5" : "justify-between px-4 py-5",
        )}
      >
        {!collapsed && (
          <div className="min-w-0">
            <h1 className="text-lg font-semibold tracking-tight">Horibot</h1>
            <p className="text-xs text-muted-foreground">Robot Arm Controller</p>
          </div>
        )}
        <button
          onClick={() => setCollapsed((c) => !c)}
          title={collapsed ? "사이드바 펼치기" : "사이드바 접기"}
          className="flex h-8 w-8 shrink-0 items-center justify-center rounded-md text-muted-foreground hover:bg-accent hover:text-accent-foreground"
        >
          {collapsed ? (
            <PanelLeftOpen className="h-4 w-4" />
          ) : (
            <PanelLeftClose className="h-4 w-4" />
          )}
        </button>
      </div>

      {/* 네비게이션 */}
      <nav className="flex-1 py-4 space-y-1 px-2 overflow-y-auto">
        {navItems.map(({ to, label, icon: Icon }) => (
          <NavLink
            key={to}
            to={to}
            end={to === "/"}
            title={collapsed ? label : undefined}
            className={({ isActive }) =>
              cn(
                "flex items-center rounded-md py-2 text-sm transition-colors",
                collapsed ? "justify-center px-2" : "gap-3 px-3",
                isActive
                  ? "bg-accent text-accent-foreground font-medium"
                  : "text-muted-foreground hover:bg-accent hover:text-accent-foreground",
              )
            }
          >
            <Icon className="h-4 w-4 shrink-0" />
            {!collapsed && <span>{label}</span>}
          </NavLink>
        ))}

        {/* Robots 섹션 — robots.yaml SSOT 자동 enumeration. */}
        {robots.length > 0 && (
          <div className="pt-3">
            {!collapsed && (
              <p className="px-3 pb-1 text-xs font-medium text-muted-foreground uppercase tracking-wider">
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
                        ? "bg-accent text-accent-foreground font-medium"
                        : "text-muted-foreground hover:bg-accent hover:text-accent-foreground",
                    )
                  }
                >
                  <Bot
                    className={cn("h-4 w-4 shrink-0", !r.enabled && "opacity-40")}
                  />
                </NavLink>
              ) : (
                <div key={r.id} className="mb-1">
                  <div className="flex items-center gap-3 px-3 py-1.5 text-xs text-muted-foreground">
                    <Bot
                      className={cn("h-4 w-4 shrink-0", !r.enabled && "opacity-40")}
                    />
                    <span className={cn("flex-1 truncate", !r.enabled && "opacity-60")}>
                      {r.id}
                    </span>
                    {!r.enabled && (
                      <span className="text-[10px] text-yellow-500/60">viz</span>
                    )}
                  </div>
                  {r.capabilities.map((cap) => (
                    <NavLink
                      key={cap}
                      to={`/robots/${r.id}/${cap}`}
                      className={({ isActive }) =>
                        cn(
                          "flex items-center gap-2 ml-7 mr-2 px-3 py-1.5 rounded-md text-sm transition-colors",
                          isActive
                            ? "bg-accent text-accent-foreground font-medium"
                            : "text-muted-foreground hover:bg-accent hover:text-accent-foreground",
                        )
                      }
                    >
                      <span>{CAPABILITY_LABELS[cap]}</span>
                    </NavLink>
                  ))}
                </div>
              ),
            )}
          </div>
        )}

        {/* Tasks 섹션 — backend `/tasks` 자동 enumeration. */}
        {tasks.length > 0 && (
          <div className="pt-3">
            {!collapsed && (
              <p className="px-3 pb-1 text-xs font-medium text-muted-foreground uppercase tracking-wider">
                Tasks
              </p>
            )}
            {tasks.map((name) => (
              <NavLink
                key={name}
                to={`/tasks/${name}`}
                title={collapsed ? name : undefined}
                className={({ isActive }) =>
                  cn(
                    "flex items-center rounded-md py-2 text-sm transition-colors",
                    collapsed ? "justify-center px-2" : "gap-3 px-3",
                    isActive
                      ? "bg-accent text-accent-foreground font-medium"
                      : "text-muted-foreground hover:bg-accent hover:text-accent-foreground",
                  )
                }
              >
                <ListChecks className="h-4 w-4 shrink-0" />
                {!collapsed && <span>{name}</span>}
              </NavLink>
            ))}
          </div>
        )}
      </nav>

      {/* 전역 로봇 컨트롤 */}
      <div className="px-2 py-3 space-y-2 border-t">
        {!collapsed && (
          <p className="px-3 text-xs font-medium text-muted-foreground uppercase tracking-wider">
            Control
          </p>
        )}
        <button
          onClick={() => void goHome()}
          title={collapsed ? "Go Home" : undefined}
          className={cn(
            "w-full flex items-center rounded-md py-2 text-sm text-muted-foreground hover:bg-accent hover:text-accent-foreground",
            collapsed ? "justify-center px-2" : "gap-3 px-3",
          )}
        >
          <Home className="h-4 w-4" />
          {!collapsed && "Go Home"}
        </button>
        <button
          onClick={() => void goRest()}
          title={collapsed ? "Go Rest" : undefined}
          className={cn(
            "w-full flex items-center rounded-md py-2 text-sm text-muted-foreground hover:bg-accent hover:text-accent-foreground",
            collapsed ? "justify-center px-2" : "gap-3 px-3",
          )}
        >
          <Moon className="h-4 w-4" />
          {!collapsed && "Go Rest"}
        </button>
        <button
          onClick={() => void toggleTorque()}
          title={collapsed ? (torqueEnabled ? "Torque ON" : "Torque OFF") : undefined}
          className={cn(
            "w-full flex items-center rounded-md py-2 text-sm transition-colors",
            collapsed ? "justify-center px-2" : "gap-3 px-3",
            torqueEnabled
              ? "bg-green-500/10 text-green-600 hover:bg-green-500/20"
              : "bg-red-500/20 text-red-600 font-medium hover:bg-red-500/30",
          )}
        >
          <Power className="h-4 w-4" />
          {!collapsed && (torqueEnabled ? "Torque ON" : "Torque OFF")}
        </button>
      </div>

      {/* 연결 상태 */}
      {!collapsed && (
        <div className="border-t">
          <p className="px-3 pt-3 text-xs font-medium text-muted-foreground uppercase tracking-wider">
            Status
          </p>
          <ConnectionStatus />
        </div>
      )}
    </aside>
  );
}
