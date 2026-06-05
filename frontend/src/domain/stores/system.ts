/**
 * System 누적 state — heartbeat 별 node tracker + bounded log buffer.
 *
 * `domain/handlers.ts` 가 SYSTEM_HEARTBEAT / SYSTEM_LOG 토픽에서 갱신. bridge
 * 연결 상태는 framework 가 보관 — `useBridgeConnected` hook.
 */
import { create } from "zustand";
import type { NodeInfo, NodeStatus, LogEntry } from "@/types/system";

interface SystemState {
  /** key = node_name (multi-robot 시 같은 이름 last-wins). 기존 콜사이트 호환. */
  nodes: Record<string, NodeInfo>;
  /** key = robot_id (global = "global"). inner key = node_name. */
  nodesByRobot: Record<string, Record<string, NodeInfo>>;
  logs: LogEntry[];
  updateNode: (
    name: string,
    status: NodeStatus,
    timestamp: number,
    robotId?: string | null,
  ) => void;
  addLog: (log: LogEntry) => void;
}

const MAX_LOGS = 200;

export const useSystemStore = create<SystemState>((set) => ({
  nodes: {},
  nodesByRobot: {},
  logs: [],

  updateNode: (name, status, timestamp, robotId = null) =>
    set((state) => {
      const info: NodeInfo = { name, status, timestamp, robotId };
      const bucket = robotId ?? "global";
      const prevBucket = state.nodesByRobot[bucket] ?? {};
      return {
        nodes: { ...state.nodes, [name]: info },
        nodesByRobot: {
          ...state.nodesByRobot,
          [bucket]: { ...prevBucket, [name]: info },
        },
      };
    }),

  addLog: (log) =>
    set((state) => ({
      logs: [...state.logs.slice(-(MAX_LOGS - 1)), log],
    })),
}));
