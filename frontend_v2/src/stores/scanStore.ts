/**
 * scanStore — Scan/LivePointCloud 패널(dockview overlay) ↔ 씬 객체(Canvas) 브리지.
 *
 * Camera 씬 객체(cloud) / ScanMesh 는 R3F Canvas(RobotsLayout) 안에 있고 패널은
 * dockview overlay 라 직접 prop 전달 불가 → 이 store 로 결합.
 *   - liveEnabled : 라이브 PC on/off (Camera 씬 객체의 cloud gate)
 *   - voxelSize   : backend voxel down-sample (m). SET_STREAM 으로 전송 —
 *                   1/2/5mm 3단계, default 2mm (사용자 결정 2026-06-21, v1 정책).
 *   - pointSize   : 렌더 dot 크기 (mm). frontend 시각 옵션 — backend 모름.
 *   - meshPly     : GET_MESH 로 받은 .ply bytes (ScanMesh 가 parse+render)
 *
 * 현재 라이브 뷰는 focus robot 1대 기준 (scan workflow 가 robot-scoped 페이지).
 * N robot 동시 라이브가 필요해지면 liveEnabled/voxelSize 를 dict[robot_id] 화.
 */
import { create } from "zustand";

interface ScanMeshMeta {
  vertexCount: number;
  triangleCount: number;
}

interface ScanState {
  liveEnabled: boolean;
  setLiveEnabled: (b: boolean) => void;
  /** backend voxel down-sample 크기 (m). */
  voxelSize: number;
  setVoxelSize: (m: number) => void;
  /** 렌더 point 크기 (mm) — pointsMaterial.size 는 mm/1000 (world m). */
  pointSize: number;
  setPointSize: (mm: number) => void;
  meshPly: Uint8Array | null;
  meshMeta: ScanMeshMeta | null;
  setMesh: (ply: Uint8Array, meta: ScanMeshMeta) => void;
  clearMesh: () => void;
}

export const useScanStore = create<ScanState>((set) => ({
  liveEnabled: false,
  setLiveEnabled: (b) => set({ liveEnabled: b }),
  voxelSize: 0.002, // 2mm Normal — v1 Live PointCloud 패널 default
  setVoxelSize: (m) => set({ voxelSize: m }),
  pointSize: 2.5, // mm — 기존 하드코딩 0.0025m 와 동일 시각
  setPointSize: (mm) => set({ pointSize: mm }),
  meshPly: null,
  meshMeta: null,
  setMesh: (ply, meta) => set({ meshPly: ply, meshMeta: meta }),
  clearMesh: () => set({ meshPly: null, meshMeta: null }),
}));
