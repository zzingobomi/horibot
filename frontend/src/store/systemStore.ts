import { create } from "zustand";

export type NodeStatus = "running" | "error" | "stopped";

export interface NodeInfo {
  name: string;
  status: NodeStatus;
  timestamp: number;
  robotId: string | null;
}

interface SystemStore {
  bridgeConnected: boolean;
  /** key = node_name (multi-robot 시 같은 이름 last-wins). 기존 콜사이트 호환. */
  nodes: Record<string, NodeInfo>;
  /** key = robot_id (global = "global"). inner key = node_name. robot 별 lookup. */
  nodesByRobot: Record<string, Record<string, NodeInfo>>;
  logs: { timestamp: number; node: string; level: string; message: string }[];
  setBridgeConnected: (connected: boolean) => void;
  updateNode: (
    name: string,
    status: NodeStatus,
    timestamp: number,
    robotId?: string | null,
  ) => void;
  addLog: (log: {
    timestamp: number;
    node: string;
    level: string;
    message: string;
  }) => void;
}

const MAX_LOGS = 200;

export const useSystemStore = create<SystemStore>((set) => ({
  bridgeConnected: false,
  nodes: {},
  nodesByRobot: {},
  logs: [],

  setBridgeConnected: (connected) => set({ bridgeConnected: connected }),

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
