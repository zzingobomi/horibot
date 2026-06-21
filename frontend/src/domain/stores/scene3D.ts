/**
 * Scene3D 도메인 store — RGBD primitive sensor (라이브 point cloud stream).
 *
 * 책임:
 * - SCENE3D_STATE 구독 (backend 의 enabled / voxel_size 자리)
 * - SCENE3D_STREAM (binary) 구독 → point cloud 자리 decode
 * - SCENE3D_SET_STREAM service 호출 (enable/disable 토글)
 *
 * voxel_size 자리는 backend default — frontend 변경 control 없음 (SSOT).
 * scan workflow / TSDF mesh 자리는 본 store 아님 — TasksPage 의 ScanTask + storage.
 */
import { create } from "zustand";
import { bridge } from "@/api/bridge";
import { Topic, ServiceKey } from "@/constants/topics";

export interface PointCloudFrame {
  count: number;
  positions: Float32Array;
  colors: Uint8Array;
}

interface Scene3DState {
  // ── SCENE3D_STATE topic 의 mirror (backend SSOT)
  enabled: boolean;
  voxelSize: number; // m 단위 — backend 와 동일. UI 는 mm 변환 표시.
  frame: PointCloudFrame | null;

  // ── UI-only (frontend 시각 옵션 — backend 모름)
  pointSize: number; // R3F pointsMaterial.size — slider 1~8 (px-like).

  // ── service / topic
  setEnabled: (enabled: boolean) => Promise<void>;
  setVoxelSize: (voxelSizeM: number) => Promise<void>;
  setPointSize: (pointSize: number) => void;

  // ── internal
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
    buf.slice(HEADER_BYTES, HEADER_BYTES + xyzBytes),
  );
  const colors = new Uint8Array(
    buf.slice(HEADER_BYTES + xyzBytes, HEADER_BYTES + xyzBytes + rgbBytes),
  );
  return { count, positions, colors };
}

export const useScene3DStore = create<Scene3DState>((set, get) => ({
  enabled: false,
  voxelSize: 0.002, // 2mm — Live PointCloud 패널 default (Normal)
  // voxel 2mm default 자체 자리 점들이 촘촘 → ps=1 (px-like) 이 시각상 자연.
  // 사용자 자체 자리 voxel 키우면 (Fast 5mm) slider 자체 자리 직접 키움.
  pointSize: 1,
  frame: null,

  setEnabled: async (enabled) => {
    // optimistic — backend 가 SCENE3D_STATE topic 으로 echo
    set({ enabled });
    if (!enabled) set({ frame: null });
    await bridge.callService(ServiceKey.SCENE3D_SET_STREAM, {
      enabled,
      voxel_size_m: get().voxelSize,
    });
  },

  setVoxelSize: async (voxelSizeM) => {
    set({ voxelSize: voxelSizeM });
    // backend 갱신은 enable 상태일 때만 의미 — disable 자체 자리도 보내면 다음
    // enable 시 backend default 가 아니라 frontend default 가 적용되게 한다.
    await bridge.callService(ServiceKey.SCENE3D_SET_STREAM, {
      enabled: get().enabled,
      voxel_size_m: voxelSizeM,
    });
  },

  setPointSize: (pointSize) => set({ pointSize }),

  _onState: (data) => {
    const next: Partial<Scene3DState> = {};
    if (typeof data.enabled === "boolean") next.enabled = data.enabled;
    if (typeof data.voxel_size === "number") next.voxelSize = data.voxel_size;
    if (Object.keys(next).length) set(next);
  },

  _onBinary: (buf) => {
    const frame = decodeFrame(buf);
    if (frame) set({ frame });
  },

  _attach: () => {
    const unState = bridge.subscribe(Topic.SCENE3D_STATE, (data) =>
      get()._onState(data as { enabled?: boolean; voxel_size?: number }),
    );
    const unBin = bridge.subscribeBinary(Topic.SCENE3D_STREAM, (buf) =>
      get()._onBinary(buf),
    );
    return () => {
      unState();
      unBin();
    };
  },
}));
