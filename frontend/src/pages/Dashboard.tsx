/**
 * /  — multi_robot_phase2_frontend.md §2 sketch.
 *
 * 시스템 운영 상태 overview. 3D scene 없음.
 *
 *  Robots Online:  2 / 3
 *    omx_f_0     OK
 *    so101_0     OK
 *    so101_1     Offline
 *
 *  Bridge       OK    Zenoh peers: 3
 *  Camera       OK    CPU: 22%    Mem: 1.4GB
 *
 * 첫 프로토타입 scope:
 * - Robots online = useRobots + heartbeat 도착 여부 (per-robot 노드 status 가
 *   아직 미구현이라 enabled 여부만 표시 + 향후 robot 별 motor heartbeat 매칭).
 * - Bridge OK = bridge.connected.
 * - Camera = camera_node heartbeat 도착 여부.
 * - Zenoh peers / CPU / Mem — backend endpoint 없음, 일단 skip (Slice C 자리).
 */
import { Link } from "react-router-dom";
import { Bot, Activity, Camera, Cpu, MemoryStick, Network } from "lucide-react";
import { useSystemStore } from "@/store/systemStore";
import { useRobots } from "@/hooks/useRobots";
import { useSystemMetrics } from "@/hooks/useSystemMetrics";

const HEARTBEAT_TIMEOUT_MS = 5000;

function isLive(timestamp: number | undefined): boolean {
  if (!timestamp) return false;
  return Date.now() / 1000 - timestamp < HEARTBEAT_TIMEOUT_MS / 1000;
}

export function Dashboard() {
  const { robots } = useRobots();
  const bridgeConnected = useSystemStore((s) => s.bridgeConnected);
  const nodesByRobot = useSystemStore((s) => s.nodesByRobot);
  const { metrics } = useSystemMetrics();

  // robot 별 motor / camera 노드 heartbeat 활성도. enabled robot 만 카운트
  // 의미 있음 (viz-only 는 노드 자체가 부팅 안 함).
  function nodeLive(robotId: string, nodeName: string): boolean {
    return isLive(nodesByRobot[robotId]?.[nodeName]?.timestamp);
  }

  const enabledRobots = robots.filter((r) => r.enabled);
  const onlineCount = enabledRobots.filter((r) => nodeLive(r.id, "motor_node")).length;
  // 임의 robot 의 카메라 (N=1 또는 multi-robot 의 default robot 기준).
  const defaultRobotId = robots[0]?.id ?? "";
  const cameraLive = defaultRobotId ? nodeLive(defaultRobotId, "camera_node") : false;

  return (
    <div className="flex h-full flex-col gap-4 p-6 font-mono">
      <h1 className="text-2xl font-semibold tracking-tight">Dashboard</h1>

      {/* Robots */}
      <div className="rounded-lg border bg-card p-4">
        <div className="flex items-center justify-between mb-3">
          <div className="flex items-center gap-2 text-sm font-semibold">
            <Bot className="w-4 h-4" />
            Robots Online
          </div>
          <div className="text-sm text-muted-foreground">
            {onlineCount} / {enabledRobots.length}
          </div>
        </div>
        <div className="flex flex-col gap-1.5 text-xs">
          {robots.length === 0 && (
            <div className="text-muted-foreground">로딩 중...</div>
          )}
          {robots.map((r) => {
            const live = nodeLive(r.id, "motor_node");
            const status = r.enabled
              ? live
                ? { label: "OK", cls: "text-green-400" }
                : { label: "No Heartbeat", cls: "text-yellow-400" }
              : { label: "Viz-only", cls: "text-zinc-500" };
            return (
              <div key={r.id} className="flex items-center justify-between">
                <Link
                  to={`/robots/${r.id}`}
                  className="text-zinc-200 hover:text-blue-400 transition-colors"
                >
                  {r.id}
                </Link>
                <span className={status.cls}>{status.label}</span>
              </div>
            );
          })}
        </div>
      </div>

      {/* System */}
      <div className="rounded-lg border bg-card p-4">
        <div className="flex items-center gap-2 text-sm font-semibold mb-3">
          <Activity className="w-4 h-4" />
          System
        </div>
        <div className="grid grid-cols-2 gap-x-6 gap-y-2 text-xs">
          <div className="flex items-center justify-between">
            <span className="text-muted-foreground">Bridge</span>
            <span className={bridgeConnected ? "text-green-400" : "text-red-400"}>
              {bridgeConnected ? "OK" : "Disconnected"}
            </span>
          </div>
          <div className="flex items-center justify-between">
            <span className="text-muted-foreground flex items-center gap-1">
              <Camera className="w-3 h-3" />
              Camera
            </span>
            <span className={cameraLive ? "text-green-400" : "text-zinc-500"}>
              {cameraLive ? "OK" : "Offline"}
            </span>
          </div>
          <div className="flex items-center justify-between">
            <span className="text-muted-foreground flex items-center gap-1">
              <Network className="w-3 h-3" />
              Zenoh peers
            </span>
            <span className="text-zinc-300 tabular-nums">
              {metrics
                ? `${metrics.zenoh_peers} (+${metrics.zenoh_routers} routers)`
                : "—"}
            </span>
          </div>
          <div className="flex items-center justify-between">
            <span className="text-muted-foreground flex items-center gap-1">
              <Cpu className="w-3 h-3" />
              CPU
            </span>
            <span className="text-zinc-300 tabular-nums">
              {metrics ? `${metrics.cpu_pct.toFixed(1)}%` : "—"}
            </span>
          </div>
          <div className="flex items-center justify-between">
            <span className="text-muted-foreground flex items-center gap-1">
              <MemoryStick className="w-3 h-3" />
              Mem
            </span>
            <span className="text-zinc-300 tabular-nums">
              {metrics
                ? `${(metrics.mem_used_mb / 1024).toFixed(1)} / ${(metrics.mem_total_mb / 1024).toFixed(1)} GB (${metrics.mem_pct.toFixed(0)}%)`
                : "—"}
            </span>
          </div>
        </div>
      </div>
    </div>
  );
}
