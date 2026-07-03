/**
 * scanStore — ScanPanel(dockview overlay) ↔ 3D layer(Canvas) 사이 데이터 브리지.
 *
 * Scene3DLayer / MeshLayer 는 R3F Canvas(RobotsLayout) 안에 있고, ScanPanel 은
 * dockview overlay 라 직접 prop 전달 불가 → 이 store 로 결합.
 *   - liveEnabled : 라이브 PC on/off (Scene3DLayer 가 구독 gate)
 *   - meshPly     : GET_MESH 로 받은 .ply bytes (MeshLayer 가 parse+render)
 */
import { create } from "zustand";

interface ScanMeshMeta {
  vertexCount: number;
  triangleCount: number;
}

interface ScanState {
  liveEnabled: boolean;
  setLiveEnabled: (b: boolean) => void;
  meshPly: Uint8Array | null;
  meshMeta: ScanMeshMeta | null;
  setMesh: (ply: Uint8Array, meta: ScanMeshMeta) => void;
  clearMesh: () => void;
}

export const useScanStore = create<ScanState>((set) => ({
  liveEnabled: false,
  setLiveEnabled: (b) => set({ liveEnabled: b }),
  meshPly: null,
  meshMeta: null,
  setMesh: (ply, meta) => set({ meshPly: ply, meshMeta: meta }),
  clearMesh: () => set({ meshPly: null, meshMeta: null }),
}));
