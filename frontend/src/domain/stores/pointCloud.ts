/**
 * PointCloud 도메인 store — live stream / scan capture / mesh build.
 *
 * binary 토픽 (POINTCLOUD_STREAM) 은 framework auto-sub 영역 밖이라 `_attach()`
 * 로 store 가 자체 sub. domain/handlers.ts 의 onConnect 자리에서 호출.
 */
import { create } from "zustand";
import { bridge } from "@/api/bridge";
import type { components } from "@/api/generated/types";
import { Topic, ServiceKey } from "@/constants/topics";

// POINTCLOUD_STREAM 은 binary raw 트랙 (typed 면제) — wire-side 파싱 결과
// frontend 자체 데이터 클래스.
export interface PointCloudFrame {
  count: number;
  positions: Float32Array;
  colors: Uint8Array;
}

// backend `core/transport/messages/pointcloud.py` 의 pydantic 모델에서 자동 생성.
export type ScanMeta = components["schemas"]["ScanMeta"];
export type MeshMeta = components["schemas"]["MeshMeta"];
export type BuildResultSummary = components["schemas"]["PointcloudBuildMeshRes"];

// BuildParams 는 buildMesh service request 에서 session_id 를 frontend 가 따로
// 합쳐 보내는 자리 — backend req 모델에서 session_id 만 제외한 subset.
export type BuildParams = Omit<
  components["schemas"]["PointcloudBuildMeshReq"],
  "session_id"
>;

interface PointCloudState {
  // ── live stream
  enabled: boolean;
  voxelSize: number;
  frame: PointCloudFrame | null;
  setEnabled: (enabled: boolean) => Promise<void>;
  setVoxelSize: (voxelSize: number) => Promise<void>;

  // ── session / scan capture
  currentSessionId: string | null;
  sessions: string[];
  scans: ScanMeta[];
  capturing: boolean;
  lastCaptureMessage: string | null;
  selectSession: (sid: string) => Promise<void>;
  newSession: (sid?: string) => Promise<string | null>;
  refreshSessions: () => Promise<void>;
  refreshScans: () => Promise<void>;
  capture: (numFrames?: number) => Promise<void>;
  deleteScan: (scanId: number) => Promise<void>;

  // ── mesh
  meshes: MeshMeta[];
  meshVisible: boolean;
  meshPath: string | null;
  meshBusy: boolean;
  lastBuildResult: BuildResultSummary | null;
  lastBuildError: string | null;
  refreshMeshes: () => Promise<void>;
  buildMesh: (params?: BuildParams) => Promise<void>;
  showMesh: (path: string) => void;
  hideMesh: () => void;
  setMeshVisible: (v: boolean) => void;

  // ── internal
  _onState: (data: { enabled?: boolean; voxel_size?: number }) => void;
  _onBinary: (buf: ArrayBuffer) => void;
  _attach: () => () => void;
}

const HEADER_BYTES = 4;
const BUILD_TIMEOUT_MS = 120_000;

function decodeFrame(buf: ArrayBuffer): PointCloudFrame | null {
  if (buf.byteLength < HEADER_BYTES) return null;
  const view = new DataView(buf);
  const count = view.getUint32(0, true);
  const xyzBytes = count * 3 * 4;
  const rgbBytes = count * 3;
  if (buf.byteLength < HEADER_BYTES + xyzBytes + rgbBytes) return null;

  const positions = new Float32Array(
    buf.slice(HEADER_BYTES, HEADER_BYTES + xyzBytes),
  );
  const colors = new Uint8Array(
    buf.slice(HEADER_BYTES + xyzBytes, HEADER_BYTES + xyzBytes + rgbBytes),
  );
  return { count, positions, colors };
}

export const usePointCloudStore = create<PointCloudState>((set, get) => ({
  // ── live stream
  enabled: false,
  voxelSize: 0.008,
  frame: null,

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

  // ── session / scan capture
  currentSessionId: null,
  sessions: [],
  scans: [],
  capturing: false,
  lastCaptureMessage: null,

  selectSession: async (sid) => {
    set({ currentSessionId: sid });
    await get().refreshScans();
  },

  newSession: async (sid) => {
    const res = await bridge.callService(ServiceKey.POINTCLOUD_NEW_SESSION, {
      session_id: sid ?? "",
    });
    if (!res.success) {
      set({ lastCaptureMessage: res.message });
      return null;
    }
    const newId = (res.data.session_id as string) ?? null;
    if (newId) {
      set({ currentSessionId: newId, scans: [] });
      await get().refreshSessions();
    }
    return newId;
  },

  refreshSessions: async () => {
    const res = await bridge.callService(
      ServiceKey.POINTCLOUD_LIST_SESSIONS,
      {},
    );
    if (res.success) {
      set({ sessions: (res.data.sessions as string[]) ?? [] });
    }
  },

  refreshScans: async () => {
    const sid = get().currentSessionId;
    if (!sid) {
      set({ scans: [] });
      return;
    }
    const res = await bridge.callService(ServiceKey.POINTCLOUD_LIST_SCANS, {
      session_id: sid,
    });
    if (res.success) {
      set({ scans: (res.data.scans as ScanMeta[]) ?? [] });
    }
  },

  capture: async (numFrames) => {
    const sid = get().currentSessionId;
    if (!sid) {
      set({ lastCaptureMessage: "세션 먼저 생성/선택" });
      return;
    }
    set({ capturing: true, lastCaptureMessage: null });
    try {
      const payload: Record<string, unknown> = { session_id: sid };
      if (numFrames !== undefined) payload.num_frames = numFrames;
      const res = await bridge.callService(
        ServiceKey.POINTCLOUD_CAPTURE,
        payload,
        { timeoutMs: 15_000 },
      );
      set({ lastCaptureMessage: res.message });
      if (res.success) {
        await get().refreshScans();
      }
    } finally {
      set({ capturing: false });
    }
  },

  deleteScan: async (scanId) => {
    const sid = get().currentSessionId;
    if (!sid) return;
    const res = await bridge.callService(ServiceKey.POINTCLOUD_DELETE_SCAN, {
      session_id: sid,
      scan_id: scanId,
    });
    set({ lastCaptureMessage: res.message });
    if (res.success) {
      await get().refreshScans();
    }
  },

  // ── mesh
  meshes: [],
  meshVisible: true,
  meshPath: null,
  meshBusy: false,
  lastBuildResult: null,
  lastBuildError: null,

  refreshMeshes: async () => {
    const res = await bridge.callService(ServiceKey.POINTCLOUD_LIST_MESHES, {});
    if (res.success) {
      set({ meshes: (res.data.meshes as MeshMeta[]) ?? [] });
    }
  },

  buildMesh: async (params) => {
    const sid = get().currentSessionId;
    if (!sid) {
      set({ lastBuildError: "세션 먼저 선택" });
      return;
    }
    set({ meshBusy: true, lastBuildError: null });
    try {
      const payload: Record<string, unknown> = { session_id: sid };
      if (params?.voxel_size !== undefined)
        payload.voxel_size = params.voxel_size;
      if (params?.sdf_trunc !== undefined) payload.sdf_trunc = params.sdf_trunc;
      if (params?.depth_trunc !== undefined)
        payload.depth_trunc = params.depth_trunc;
      if (params?.icp_max_dist !== undefined)
        payload.icp_max_dist = params.icp_max_dist;

      const res = await bridge.callService(
        ServiceKey.POINTCLOUD_BUILD_MESH,
        payload,
        { timeoutMs: BUILD_TIMEOUT_MS },
      );
      if (res.success) {
        const summary = res.data as unknown as BuildResultSummary;
        set({
          lastBuildResult: summary,
          meshPath: summary.path,
          meshVisible: true,
        });
        await get().refreshMeshes();
      } else {
        set({ lastBuildError: res.message });
      }
    } finally {
      set({ meshBusy: false });
    }
  },

  showMesh: (path) => set({ meshPath: path, meshVisible: true }),
  hideMesh: () => set({ meshVisible: false }),
  setMeshVisible: (v) => set({ meshVisible: v }),

  // ── internal
  _onState: (data) => {
    const next: Partial<PointCloudState> = {};
    if (typeof data.enabled === "boolean") next.enabled = data.enabled;
    if (typeof data.voxel_size === "number") next.voxelSize = data.voxel_size;
    if (Object.keys(next).length) set(next);
  },

  _onBinary: (buf) => {
    const frame = decodeFrame(buf);
    if (frame) set({ frame });
  },

  _attach: () => {
    const unState = bridge.subscribe(Topic.POINTCLOUD_STATE, (data) =>
      get()._onState(data as { enabled?: boolean; voxel_size?: number }),
    );
    const unBin = bridge.subscribeBinary(Topic.POINTCLOUD_STREAM, (buf) =>
      get()._onBinary(buf),
    );
    return () => {
      unState();
      unBin();
    };
  },
}));
