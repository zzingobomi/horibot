/**
 * System metrics — backend `/system` HTTP endpoint (psutil CPU/Mem + zenoh peers).
 */

export interface SystemMetrics {
  cpu_pct: number;
  mem_used_mb: number;
  mem_total_mb: number;
  mem_pct: number;
  zenoh_routers: number;
  zenoh_peers: number;
}

export type NodeStatus = "running" | "error" | "stopped";

export interface NodeInfo {
  name: string;
  status: NodeStatus;
  timestamp: number;
  robotId: string | null;
}

export interface LogEntry {
  timestamp: number;
  node: string;
  level: string;
  message: string;
}

export interface TasksResponse {
  tasks: string[];
}
