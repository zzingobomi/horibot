import { useEffect, useState } from "react";
import { NavLink } from "react-router-dom";
import {
  Gamepad2,
  Camera,
  Cpu,
  Settings,
  Box,
  Home,
  Moon,
  Power,
  Hand,
  PanelLeftClose,
  PanelLeftOpen,
} from "lucide-react";
import { ConnectionStatus } from "@/components/common/ConnectionStatus";
import { cn } from "@/lib/utils";
import { useJointControl } from "@/hooks/useJointControl";

const navItems = [
  { to: "/", label: "Dashboard", icon: Gamepad2 },
  { to: "/motion", label: "Motion", icon: Cpu },
  { to: "/calibration", label: "Calibration", icon: Camera },
  { to: "/workspace", label: "Workspace3D", icon: Box },
  { to: "/pick-and-place", label: "Pick & Place", icon: Hand },
  { to: "/settings", label: "Settings", icon: Settings },
];

const COLLAPSED_KEY = "omx.sidebar.collapsed";

export function Sidebar() {
  const { goHome, goRest, torqueEnabled, enableTorque } = useJointControl();
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
            <h1 className="text-lg font-semibold tracking-tight">OMX Control</h1>
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
      <nav className="flex-1 py-4 space-y-1 px-2">
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
      </nav>

      {/* 전역 로봇 컨트롤 */}
      <div className="px-2 py-3 space-y-2 border-t">
        {!collapsed && (
          <p className="px-3 text-xs font-medium text-muted-foreground uppercase tracking-wider">
            Control
          </p>
        )}
        <button
          onClick={goHome}
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
          onClick={goRest}
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
          onClick={() => enableTorque(!torqueEnabled)}
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
