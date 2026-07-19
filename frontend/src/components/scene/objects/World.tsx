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
import { useCallback, useEffect, useRef, useState } from "react";
import * as THREE from "three";
import { PLYLoader } from "three/examples/jsm/loaders/PLYLoader.js";
import type { WorldMeshResult } from "./worldMeshWorker";
import { useService, useStream } from "@/framework";
import { ServiceKey, Topic } from "@/api/generated/contract";
import type {
  GetMeshResponse,
  ListReconstructionsResponse,
  ListSessionsResponse,
  ReconstructionRecord,
} from "@/api/generated/contract";
import { useScanStore } from "@/stores/scanStore";
import { startJankMonitor } from "@/lib/jankMonitor";
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
  // 리로드 스로틀 (2026-07-19 밤): 스윕 중 빌드 done 이 ~7회 — 갱신 1회마다
  // 7.6MB 수신+디코드+GPU 업로드가 메인 스레드를 수십 ms 씩 물어 히칭 (파싱은
  // worker 로 뺐지만 업로드는 못 뺌). 리로드를 최소 간격으로 묶고, 마지막
  // done 은 trailing 타이머가 보장 (최종 메시는 반드시 반영).
  const lastLoadAtRef = useRef(0);
  const trailingRef = useRef<ReturnType<typeof setTimeout> | null>(null);

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

  // 빌드 완료 → 최신 재조회 (성장 UX) — 단, _RELOAD_MIN_GAP_MS 스로틀.
  // done payload 의 recon id 전이가 트리거, 간격 미달이면 trailing 예약
  // (마지막 빌드가 스로틀에 먹혀 최종 메시를 놓치는 일 없음).
  const doneId =
    progress.value?.stage === "done"
      ? (progress.value.reconstruction_row_id ?? null)
      : null;
  useEffect(() => {
    if (doneId == null) return;
    const run = () => {
      lastLoadAtRef.current = Date.now();
      void loadLatest();
    };
    const since = Date.now() - lastLoadAtRef.current;
    if (trailingRef.current) clearTimeout(trailingRef.current);
    if (since >= _RELOAD_MIN_GAP_MS) {
      run();
    } else {
      trailingRef.current = setTimeout(run, _RELOAD_MIN_GAP_MS - since);
    }
    return () => {
      if (trailingRef.current) clearTimeout(trailingRef.current);
    };
  }, [doneId, loadLatest]);
}

// 스윕 중 월드 리로드 최소 간격 — 빌드 done ~7회를 2~3회 갱신으로 묶는다
// (성장 UX 유지 + 갱신당 메인 스레드 비용 노출 횟수 축소). 마지막 done 은
// trailing 으로 보장.
const _RELOAD_MIN_GAP_MS = 15_000;

export function World({ robots, focusId }: SceneObjectProps) {
  const ply = useScanStore((s) => s.meshPly);
  const visible = useScanStore((s) => s.worldVisible);
  const robotId = focusId ?? robots[0]?.id ?? "";
  useWorldAutoLoad(robotId);
  // dev 버벅임 계측 — [jank] 가 [world] 갱신 직후 몰리는지로 범인 분리
  useEffect(() => {
    startJankMonitor();
  }, []);

  // PLY 파싱은 **Web Worker** (2026-07-19) — 실측 7.6MB/106k verts 에 110~127ms,
  // 스윕 중 갱신 6~7회 × 그만큼 메인 스레드가 얼어 3D 뷰 히칭 (사용자 체감).
  // worker 가 파싱 후 typed array 를 transfer(복사 0)로 반환 → 메인은 조립 +
  // GPU 업로드만. seq 가드 = 연속 갱신 시 최신 결과만 채택 (latest-wins).
  const [geometry, setGeometry] = useState<THREE.BufferGeometry | null>(null);
  const workerRef = useRef<Worker | null>(null);
  const seqRef = useRef(0);

  useEffect(() => () => {
    workerRef.current?.terminate();
    workerRef.current = null;
  }, []);

  useEffect(() => {
    if (!ply) {
      // effect 내 동기 setState 금지(lint) — microtask 로 (worker 경로는
      // message 이벤트라 원래 비동기).
      let cancelled = false;
      queueMicrotask(() => {
        if (!cancelled) setGeometry(null);
      });
      return () => {
        cancelled = true;
      };
    }
    // Uint8Array → 정확히 tight ArrayBuffer **복사** (msgpack view offset 대비
    // + worker 로 transfer 해도 store 원본(ply)은 무손상).
    const ab = ply.buffer.slice(ply.byteOffset, ply.byteOffset + ply.byteLength);
    const sizeMb = (ab.byteLength / 1048576).toFixed(1); // transfer 전에 캡처
    const seq = ++seqRef.current;
    if (typeof Worker === "undefined") {
      // 테스트(jsdom)/워커 불가 환경 폴백 — 옛 동기 파싱 (동작 동일).
      // setState 는 microtask 로 미뤄 effect 내 동기 setState lint 준수.
      const g = new PLYLoader().parse(ab as ArrayBuffer);
      if (!g.getAttribute("normal")) g.computeVertexNormals();
      let cancelled = false;
      queueMicrotask(() => {
        if (!cancelled) setGeometry(g);
      });
      return () => {
        cancelled = true;
      };
    }
    workerRef.current ??= new Worker(
      new URL("./worldMeshWorker.ts", import.meta.url),
      { type: "module" },
    );
    const w = workerRef.current;
    const t0 = performance.now();
    const onMsg = (e: MessageEvent<WorldMeshResult>) => {
      if (e.data.seq !== seq) return; // 이전 요청 결과 — 폐기 (latest-wins)
      const t1 = performance.now();
      const g = new THREE.BufferGeometry();
      g.setAttribute("position", new THREE.BufferAttribute(e.data.position, 3));
      if (e.data.normal) {
        g.setAttribute("normal", new THREE.BufferAttribute(e.data.normal, 3));
      }
      if (e.data.color) {
        g.setAttribute("color", new THREE.BufferAttribute(e.data.color, 3));
      }
      if (e.data.index) {
        g.setIndex(new THREE.BufferAttribute(e.data.index, 1));
      }
      setGeometry(g);
      // 계측 (사용자 체감 히칭의 데이터화) — worker 파싱 ms 는 메인 비차단,
      // 메인 스레드 몫은 "조립" 뿐 (+ 첫 렌더 시 GPU 업로드).
      console.info(
        `[world] mesh ${sizeMb}MB — 파싱 ` +
          `${e.data.parseMs.toFixed(0)}ms(worker, 비차단) + 대기 ` +
          `${(t1 - t0 - e.data.parseMs).toFixed(0)}ms + 조립(메인) ` +
          `${(performance.now() - t1).toFixed(1)}ms`,
      );
    };
    w.addEventListener("message", onMsg);
    w.postMessage({ seq, buffer: ab }, [ab as ArrayBuffer]);
    return () => w.removeEventListener("message", onMsg);
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
