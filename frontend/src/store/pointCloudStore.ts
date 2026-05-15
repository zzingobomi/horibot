import { create } from "zustand";
import { bridge } from "@/api/bridge";
import { Topic, ServiceKey } from "@/constants/topics";

export interface PointCloudFrame {
  count: number;
  positions: Float32Array;
  colors: Uint8Array;
}

export interface SessionEntry {
  session_id: string;
  path: string;
  scan_count: number;
}

export interface ScanEntry {
  name: string;
  ply_path: string;
  size: number;
}

interface PointCloudState {
  // 라이브 스트림
  enabled: boolean;
  voxelSize: number;
  frame: PointCloudFrame | null;

  // 세션 / 스냅샷
  currentSessionId: string | null;
  sessions: SessionEntry[];
  scans: ScanEntry[]; // currentSessionId가 가진 스캔 목록
  snapshot: PointCloudFrame | null;
  snapshotLabel: string | null; // 어떤 스캔인지(또는 capture 결과 path)
  busy: boolean;

  // 라이브
  setEnabled: (enabled: boolean) => Promise<void>;
  setVoxelSize: (voxelSize: number) => Promise<void>;

  // 세션 / 캡처 / 라이브러리
  newSession: (
    sessionId?: string
  ) => Promise<{ success: boolean; message: string; sessionId?: string }>;
  capture: (
    numFrames?: number
  ) => Promise<{ success: boolean; message: string; plyPath?: string }>;
  refreshSessions: () => Promise<void>;
  refreshScans: (sessionId: string) => Promise<void>;
  loadScan: (
    plyPath: string
  ) => Promise<{ success: boolean; message: string }>;
  clearSnapshot: () => Promise<void>;
  selectSession: (sessionId: string | null) => Promise<void>;

  _onState: (data: {
    enabled?: boolean;
    voxel_size?: number;
    session_id?: string | null;
  }) => void;
  _onBinary: (buf: ArrayBuffer) => void;
  _onSnapshot: (buf: ArrayBuffer) => void;
  _attach: () => () => void;
}

const HEADER_BYTES = 4;

function decodeFrame(buf: ArrayBuffer): PointCloudFrame | null {
  if (buf.byteLength < HEADER_BYTES) return null;
  const view = new DataView(buf);
  const count = view.getUint32(0, true);
  const xyzBytes = count * 3 * 4;
  const rgbBytes = count * 3;
  if (buf.byteLength < HEADER_BYTES + xyzBytes + rgbBytes) return null;

  const positions = new Float32Array(
    buf.slice(HEADER_BYTES, HEADER_BYTES + xyzBytes)
  );
  const colors = new Uint8Array(
    buf.slice(HEADER_BYTES + xyzBytes, HEADER_BYTES + xyzBytes + rgbBytes)
  );
  return { count, positions, colors };
}

export const usePointCloudStore = create<PointCloudState>((set, get) => ({
  enabled: false,
  voxelSize: 0.008,
  frame: null,

  currentSessionId: null,
  sessions: [],
  scans: [],
  snapshot: null,
  snapshotLabel: null,
  busy: false,

  setEnabled: async (enabled) => {
    set({ enabled });
    if (!enabled) set({ frame: null });
    await bridge.callService(ServiceKey.POINTCLOUD_CONFIGURE, { enabled });
  },

  setVoxelSize: async (voxelSize) => {
    set({ voxelSize });
    await bridge.callService(ServiceKey.POINTCLOUD_CONFIGURE, {
      voxel_size: voxelSize,
    });
  },

  newSession: async (sessionId) => {
    set({ busy: true });
    try {
      const res = await bridge.callService(
        ServiceKey.POINTCLOUD_NEW_SESSION,
        sessionId ? { session_id: sessionId } : {}
      );
      if (res.success) {
        const sid = (res.data?.session_id as string) ?? null;
        set({ currentSessionId: sid, scans: [] });
        await get().refreshSessions();
        if (sid) await get().refreshScans(sid);
      }
      return {
        success: res.success,
        message: res.message,
        sessionId: res.data?.session_id as string | undefined,
      };
    } finally {
      set({ busy: false });
    }
  },

  capture: async (numFrames) => {
    set({ busy: true });
    try {
      const payload: Record<string, unknown> = {};
      if (typeof numFrames === "number") payload.num_frames = numFrames;
      const res = await bridge.callService(
        ServiceKey.POINTCLOUD_CAPTURE,
        payload
      );
      if (res.success) {
        const ply = (res.data?.ply_path as string) ?? null;
        const sid =
          (res.data?.session_id as string) ?? get().currentSessionId;
        set({ snapshotLabel: ply, currentSessionId: sid });
        await get().refreshSessions();
        if (sid) await get().refreshScans(sid);
      }
      return {
        success: res.success,
        message: res.message,
        plyPath: res.data?.ply_path as string | undefined,
      };
    } finally {
      set({ busy: false });
    }
  },

  refreshSessions: async () => {
    const res = await bridge.callService(
      ServiceKey.POINTCLOUD_LIST_SCANS,
      {}
    );
    if (!res.success) return;
    const sessions = (res.data?.sessions as SessionEntry[]) ?? [];
    const current = (res.data?.current_session_id as string | null) ?? null;
    set({ sessions, currentSessionId: current });
  },

  refreshScans: async (sessionId) => {
    const res = await bridge.callService(ServiceKey.POINTCLOUD_LIST_SCANS, {
      session_id: sessionId,
    });
    if (!res.success) return;
    const scans = (res.data?.scans as ScanEntry[]) ?? [];
    set({ scans });
  },

  selectSession: async (sessionId) => {
    set({ currentSessionId: sessionId, scans: [] });
    if (sessionId) await get().refreshScans(sessionId);
  },

  loadScan: async (plyPath) => {
    set({ busy: true });
    try {
      const res = await bridge.callService(ServiceKey.POINTCLOUD_LOAD_SCAN, {
        path: plyPath,
      });
      if (res.success) set({ snapshotLabel: plyPath });
      return { success: res.success, message: res.message };
    } finally {
      set({ busy: false });
    }
  },

  clearSnapshot: async () => {
    await bridge.callService(ServiceKey.POINTCLOUD_CLEAR_SNAPSHOT, {});
    set({ snapshot: null, snapshotLabel: null });
  },

  _onState: (data) => {
    const next: Partial<PointCloudState> = {};
    if (typeof data.enabled === "boolean") next.enabled = data.enabled;
    if (typeof data.voxel_size === "number") next.voxelSize = data.voxel_size;
    if ("session_id" in data) {
      next.currentSessionId = (data.session_id as string | null) ?? null;
    }
    if (Object.keys(next).length) set(next);
  },

  _onBinary: (buf) => {
    const frame = decodeFrame(buf);
    if (frame) set({ frame });
  },

  _onSnapshot: (buf) => {
    const frame = decodeFrame(buf);
    if (!frame) return;
    if (frame.count === 0) set({ snapshot: null, snapshotLabel: null });
    else set({ snapshot: frame });
  },

  _attach: () => {
    const unState = bridge.subscribe(Topic.POINTCLOUD_STATE, (data) =>
      get()._onState(
        data as {
          enabled?: boolean;
          voxel_size?: number;
          session_id?: string | null;
        }
      )
    );
    const unBin = bridge.subscribeBinary(Topic.POINTCLOUD_STREAM, (buf) =>
      get()._onBinary(buf)
    );
    const unSnap = bridge.subscribeBinary(Topic.POINTCLOUD_SNAPSHOT, (buf) =>
      get()._onSnapshot(buf)
    );
    return () => {
      unState();
      unBin();
      unSnap();
    };
  },
}));
