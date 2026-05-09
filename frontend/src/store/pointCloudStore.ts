import { create } from "zustand";
import { bridge } from "@/api/bridge";
import { Topic, ServiceKey } from "@/constants/topics";

export interface PointCloudFrame {
  count: number;
  positions: Float32Array;
  colors: Uint8Array;
}

interface PointCloudState {
  enabled: boolean;
  voxelSize: number;
  frame: PointCloudFrame | null;

  setEnabled: (enabled: boolean) => Promise<void>;
  setVoxelSize: (voxelSize: number) => Promise<void>;
  _onState: (data: { enabled?: boolean; voxel_size?: number }) => void;
  _onBinary: (buf: ArrayBuffer) => void;
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
      get()._onState(data as { enabled?: boolean; voxel_size?: number })
    );
    const unBin = bridge.subscribeBinary(Topic.POINTCLOUD_STREAM, (buf) =>
      get()._onBinary(buf)
    );
    return () => {
      unState();
      unBin();
    };
  },
}));
