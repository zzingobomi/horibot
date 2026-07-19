/**
 * World — 작업 셀 배경 레이어 (재구성 메시). 씬 객체 (세계 그 자체 — 자기가
 * 자기를 그림, 패널 수명과 무관). 옛 ScanMesh 의 승격 (2026-07-18).
 *
 * "World 는 Scan 의 부산물이 아니라 레이어" (설계 합의): 오늘의 producer 는
 * scan 재구성뿐이지만 이 컴포넌트는 scanStore 의 mesh slot 만 본다 — 나중에
 * SLAM/수동 임포트가 같은 slot 을 채워도 UI 불변.
 *
 * 데이터 소유 (씬 객체 = 자기 데이터 자기가 구독):
 * - 자동 로드: 마운트 시 최신 reconstruction 조회 → GET_MESH → scanStore.
 * - 성장 UX: BUILD_PROGRESS done 수신 → 최신 재조회 (RunRequest.build_world
 *   편승 스캔이 search pose 마다 빌드 → 월드가 자라는 게 실시간으로 보임).
 * - ScanPanel 수동 로드(옛 recon 열람)는 그대로 존중 — 자동 로드는 마운트/빌드
 *   완료 시에만 개입 (latest-wins).
 * - 표시 게이트 = scanStore.worldVisible (workcell 전역 — 갱신 여부와 독립).
 *
 * mesh 정점은 robot base frame (build 가 base 기준 TSDF) → <RobotFrame> 부모
 * transform 로 배치. 대상 robot = focus ?? 첫 robot.
 */
import { useCallback, useEffect, useMemo, useRef } from "react";
import * as THREE from "three";
import { PLYLoader } from "three/examples/jsm/loaders/PLYLoader.js";
import { useService, useStream } from "@/framework";
import { ServiceKey, Topic } from "@/api/generated/contract";
import type {
  GetMeshResponse,
  ListReconstructionsResponse,
  ListSessionsResponse,
  ReconstructionRecord,
} from "@/api/generated/contract";
import { useScanStore } from "@/stores/scanStore";
import type { SceneObjectProps } from "../sceneTypes";
import { RobotFrame } from "../shared/RobotFrame";

/** 최신 reconstruction 자동 로드 + 빌드 완료 갱신 (World 데이터 소유권). */
function useWorldAutoLoad(robotId: string) {
  const listSessions = useService(ServiceKey.SCAN_LIST_SESSIONS, robotId);
  const listRecons = useService(ServiceKey.SCAN_LIST_RECONSTRUCTIONS, robotId);
  const getMesh = useService(ServiceKey.SCAN_GET_MESH, robotId);
  const setMesh = useScanStore((s) => s.setMesh);
  const progress = useStream(Topic.SCAN_BUILD_PROGRESS, {
    robotId,
    staleMs: 60_000,
  });

  const loadedIdRef = useRef<number | null>(null);
  const busyRef = useRef(false);

  const loadLatest = useCallback(async () => {
    if (!robotId || busyRef.current) return;
    busyRef.current = true;
    try {
      const s = await listSessions.call({ robot_id: robotId });
      const sessions = (s.data as ListSessionsResponse | null)?.sessions ?? [];
      let latest: ReconstructionRecord | null = null;
      for (const sess of sessions) {
        if (sess.id == null) continue;
        const r = await listRecons.call({ session_row_id: sess.id });
        const recons =
          (r.data as ListReconstructionsResponse | null)?.reconstructions ?? [];
        for (const rec of recons) {
          // created_at 은 wire 상 ISO 문자열 (생성 TS 타입은 unknown — datetime)
          if (
            !latest ||
            new Date(String(rec.created_at)).getTime() >
              new Date(String(latest.created_at)).getTime()
          ) {
            latest = rec;
          }
        }
      }
      // 재구성 없음 = 빈 월드가 정상 상태 (안내 라벨은 패널 몫 — "월드 없음")
      if (!latest || latest.id == null) return;
      if (latest.id === loadedIdRef.current) return; // 이미 표시 중 — 재전송 억제
      const m = await getMesh.call(
        { reconstruction_row_id: latest.id },
        { timeoutMs: 30_000 },
      );
      const d = m.data as GetMeshResponse | null;
      if (!d?.ply_bytes || d.ply_bytes.byteLength === 0) return;
      loadedIdRef.current = latest.id;
      setMesh(d.ply_bytes, {
        vertexCount: d.vertex_count,
        triangleCount: d.triangle_count,
        createdAt: String(latest.created_at),
        reconstructionId: latest.id,
        voxelSizeM: latest.voxel_size,
      });
    } finally {
      busyRef.current = false;
    }
    // useService 반환 객체는 render 마다 새 ref — robotId 만이 실제 의존.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [robotId]);

  useEffect(() => {
    void loadLatest();
  }, [loadLatest]);

  // 빌드 완료 → 최신 재조회 (성장 UX). done payload 의 recon id 전이가 트리거.
  const doneId =
    progress.value?.stage === "done"
      ? (progress.value.reconstruction_row_id ?? null)
      : null;
  useEffect(() => {
    if (doneId != null) void loadLatest();
  }, [doneId, loadLatest]);
}

export function World({ robots, focusId }: SceneObjectProps) {
  const ply = useScanStore((s) => s.meshPly);
  const visible = useScanStore((s) => s.worldVisible);
  const robotId = focusId ?? robots[0]?.id ?? "";
  useWorldAutoLoad(robotId);

  const geometry = useMemo(() => {
    if (!ply) return null;
    // Uint8Array → 정확히 tight ArrayBuffer (msgpack view offset 대비 slice).
    const ab = ply.buffer.slice(ply.byteOffset, ply.byteOffset + ply.byteLength);
    const g = new PLYLoader().parse(ab as ArrayBuffer);
    if (!g.getAttribute("normal")) g.computeVertexNormals();
    return g;
  }, [ply]);

  // 메시 교체 시 GPU 자원 해제 (성장 UX = 반복 교체 — three.js leak 방지)
  useEffect(() => {
    return () => {
      geometry?.dispose();
    };
  }, [geometry]);

  const hasColor = geometry?.getAttribute("color") != null;

  if (!visible || !geometry || !robotId) return null;

  return (
    <RobotFrame robotId={robotId}>
      <mesh>
        <primitive object={geometry} attach="geometry" />
        <meshStandardMaterial
          vertexColors={hasColor}
          color={hasColor ? "#ffffff" : "#88aacc"}
          roughness={0.7}
          metalness={0.0}
          side={THREE.DoubleSide}
          flatShading={false}
        />
      </mesh>
    </RobotFrame>
  );
}
