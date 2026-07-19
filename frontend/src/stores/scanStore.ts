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
 * liveEnabled 는 **per-robot dict** — robot-owned 패널의 토글은 자기 robot cloud 만
 * 켠다 (cameraStore.frustum 과 같은 클래스 — 전역 bool 은 cross-robot 오발사).
 * voxelSize/pointSize 는 robot 무관 표시 선호값이라 전역 유지.
 */
import { create } from "zustand";

/**
 * TSDF 빌드 voxel 4단 (SSOT) — ScanPanel 수동 빌드 + PickAndPlacePanel 월드
 * 갱신이 공유. 막연한 low/high 가 아니라 **실제 조절값(mm)** 을 노출한다 (recon
 * DB row 에 저장돼 "이 메시가 왜 이 모양" 분석 데이터가 됨, 2026-07-18). 값은
 * backend BuildRequest.voxel_size 단위(m). 기본 2mm = scan 기본과 일치 (계측 전
 * 기본을 묵시 변경하지 않음 — measure-first). 1mm 는 정밀하나 비용 급증
 * (실측: 2mm 2s/5.9MB vs 1mm 7.7s/29MB).
 */
export const VOXEL_TIERS = [
  { m: 0.001, label: "1mm · 정밀 (느림·무거움)" },
  { m: 0.002, label: "2mm · 표준" },
  { m: 0.004, label: "4mm · 빠름" },
  { m: 0.008, label: "8mm · 초고속 (성김)" },
] as const;
export const DEFAULT_BUILD_VOXEL_M = 0.002;

export interface ScanMeshMeta {
  vertexCount: number;
  triangleCount: number;
  /** 재구성 생성 시각 (ISO) — World 라벨 "N시간 전 스캔" (stale 월드 침묵 금지). */
  createdAt?: string;
  /** 표시 중 reconstruction row id — World 자동 갱신의 dedup 기준. */
  reconstructionId?: number;
  /** 빌드 voxel (m) — World 라벨에 표기 (품질 선택이 실제 반영됐는지 가시화). */
  voxelSizeM?: number;
}

interface ScanState {
  /** robot_id → live cloud 표시 여부 */
  liveEnabled: Record<string, boolean>;
  setLiveEnabled: (robotId: string, on: boolean) => void;
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
  /**
   * World(재구성 메시 배경) 표시 여부 — **workcell 전역** (월드는 robot 이 아니라
   * 작업 셀 소유 — per-robot Record 규칙의 정당한 예외). 표시는 비용 0 (정적
   * 메시) 이라 기본 ON. 갱신(RunRequest.build_world)과는 독립 토글.
   */
  worldVisible: boolean;
  setWorldVisible: (on: boolean) => void;
}

export const useScanStore = create<ScanState>((set) => ({
  liveEnabled: {},
  setLiveEnabled: (robotId, on) =>
    set((s) => ({ liveEnabled: { ...s.liveEnabled, [robotId]: on } })),
  voxelSize: 0.002, // 2mm Normal — v1 Live PointCloud 패널 default
  setVoxelSize: (m) => set({ voxelSize: m }),
  pointSize: 2.5, // mm — 기존 하드코딩 0.0025m 와 동일 시각
  setPointSize: (mm) => set({ pointSize: mm }),
  meshPly: null,
  meshMeta: null,
  setMesh: (ply, meta) => set({ meshPly: ply, meshMeta: meta }),
  clearMesh: () => set({ meshPly: null, meshMeta: null }),
  worldVisible: true,
  setWorldVisible: (on) => set({ worldVisible: on }),
}));
