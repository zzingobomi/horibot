/**
 * ScanPanel — scan 워크플로 (dockview 패널, RobotScanMode 코어).
 *
 * Task DSL 없이 서비스 직접 호출 (실용 슬라이스):
 *   세션(new/list) → 캡처 반복(CAPTURE) → 빌드(BUILD, 진행 스트림) →
 *   mesh 보기(GET_MESH → scanStore → ScanMesh).
 *
 * 라이브 PC 토글/Density/Point Size 는 LivePointCloudPanel (컨트롤 SSOT 1곳).
 * 자세 잡기는 수동(토크오프) — 이 패널은 캡처/빌드 트리거만. 3D 뷰(라이브 PC/mesh)는
 * RobotsLayout Canvas 의 Camera 씬 객체(cloud)/ScanMesh 가 scanStore/stream 으로 렌더.
 */
import { useCallback, useEffect, useState } from "react";
import { Button } from "@/components/ui/button";
import { useRobotId } from "@/hooks/useRobotId";
import { useService, useStream } from "@/framework";
import { ServiceKey, Topic } from "@/api/generated/contract";
import type {
  GetMeshResponse,
  ReconstructionRecord,
  ScanRecord,
  ScanSessionRecord,
} from "@/api/generated/contract";
import {
  DEFAULT_BUILD_VOXEL_M,
  useScanStore,
  VOXEL_TIERS,
} from "@/stores/scanStore";

// 수동 빌드 voxel 마지막 선택 (m) — pnp 월드 voxel 과 별개 (다른 작업 맥락).
const BUILD_VOXEL_LS_KEY = "scan.buildVoxelM";

export function ScanPanel() {
  const robotId = useRobotId();

  const newSession = useService(ServiceKey.SCAN_NEW_SESSION, robotId);
  const listSessions = useService(ServiceKey.SCAN_LIST_SESSIONS, robotId);
  const capture = useService(ServiceKey.SCAN_CAPTURE, robotId);
  const listScans = useService(ServiceKey.SCAN_LIST_SCANS, robotId);
  const deleteScan = useService(ServiceKey.SCAN_DELETE_SCAN, robotId);
  const build = useService(ServiceKey.SCAN_BUILD, robotId);
  const listRecons = useService(ServiceKey.SCAN_LIST_RECONSTRUCTIONS, robotId);
  const getMesh = useService(ServiceKey.SCAN_GET_MESH, robotId);
  const progress = useStream(Topic.SCAN_BUILD_PROGRESS, { robotId, staleMs: 60_000 });

  const setMesh = useScanStore((s) => s.setMesh);

  const [sessionRowId, setSessionRowId] = useState<number | null>(null);
  const [scans, setScans] = useState<ScanRecord[]>([]);
  const [recons, setRecons] = useState<ReconstructionRecord[]>([]);
  const [building, setBuilding] = useState(false);
  const [msg, setMsg] = useState("");
  const [buildVoxelM, setBuildVoxelMState] = useState(() => {
    const v = Number(localStorage.getItem(BUILD_VOXEL_LS_KEY));
    return VOXEL_TIERS.some((t) => t.m === v) ? v : DEFAULT_BUILD_VOXEL_M;
  });
  const setBuildVoxelM = (m: number) => {
    setBuildVoxelMState(m);
    localStorage.setItem(BUILD_VOXEL_LS_KEY, String(m));
  };

  const refreshScans = useCallback(
    async (sid: number) => {
      const res = await listScans.call({ session_row_id: sid });
      setScans((res.data as { scans?: ScanRecord[] } | null)?.scans ?? []);
    },
    [listScans],
  );
  const refreshRecons = useCallback(
    async (sid: number) => {
      const res = await listRecons.call({ session_row_id: sid });
      setRecons(
        (res.data as { reconstructions?: ReconstructionRecord[] } | null)
          ?.reconstructions ?? [],
      );
    },
    [listRecons],
  );

  // scan/scene3d 는 robot-agnostic — 새 세션/목록/stream 은 req 에 robot_id,
  // 진행 자원(capture/build/mesh 등)은 session/recon row id 에서 파생.
  useEffect(() => {
    void listSessions.call({ robot_id: robotId });
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [robotId]);

  const onNewSession = async () => {
    const res = await newSession.call({ robot_id: robotId, label: null });
    const sess = (res.data as { session?: ScanSessionRecord } | null)?.session;
    if (sess?.id != null) {
      setSessionRowId(sess.id);
      setScans([]);
      setRecons([]);
      setMsg(`세션 시작: ${sess.session_id}`);
    } else {
      setMsg(`세션 실패: ${res.message}`);
    }
  };

  const onSelectSession = async (sid: number, label: string) => {
    setSessionRowId(sid);
    setMsg(`세션 선택: ${label}`);
    await refreshScans(sid);
    await refreshRecons(sid);
  };

  const onCapture = async () => {
    if (sessionRowId == null) return;
    const res = await capture.call(
      { session_row_id: sessionRowId, num_frames: 10 },
      { timeoutMs: 15_000 },
    );
    const d = res.data as { accepted?: boolean; scan_count?: number } | null;
    if (d?.accepted) {
      setMsg(`캡처됨 (총 ${d.scan_count})`);
      await refreshScans(sessionRowId);
    } else {
      setMsg(`캡처 거부: ${res.message}`);
    }
  };

  const onUndo = async () => {
    if (sessionRowId == null || scans.length === 0) return;
    const last = scans[scans.length - 1];
    if (last.id == null) return;
    await deleteScan.call({ scan_row_id: last.id });
    setMsg("마지막 캡처 취소");
    await refreshScans(sessionRowId);
  };

  const onBuild = async () => {
    if (sessionRowId == null) return;
    setBuilding(true);
    setMsg("빌드 시작…");
    try {
      const res = await build.call(
        { session_row_id: sessionRowId, voxel_size: buildVoxelM },
        { timeoutMs: 120_000 },
      );
      const d = res.data as {
        accepted?: boolean;
        reconstruction?: ReconstructionRecord | null;
      } | null;
      if (d?.accepted && d.reconstruction) {
        setMsg(`빌드 완료 (${d.reconstruction.vertex_count} verts)`);
        await refreshRecons(sessionRowId);
      } else {
        setMsg(`빌드 실패: ${res.message}`);
      }
    } finally {
      setBuilding(false);
    }
  };

  const onViewMesh = async (reconId: number) => {
    const res = await getMesh.call({ reconstruction_row_id: reconId });
    const d = res.data as GetMeshResponse | null;
    if (d?.ply_bytes && d.ply_bytes.byteLength > 0) {
      const rec = recons.find((r) => r.id === reconId);
      setMesh(d.ply_bytes, {
        vertexCount: d.vertex_count,
        triangleCount: d.triangle_count,
        // World 라벨("N시간 전 스캔")/자동갱신 dedup 메타 — 수동 로드도 동일 계약
        createdAt: rec ? String(rec.created_at) : undefined,
        reconstructionId: reconId,
      });
      setMsg(`mesh 로드 (${d.vertex_count} verts)`);
    } else {
      setMsg(`mesh 로드 실패: ${res.message}`);
    }
  };

  const sessions =
    (listSessions.data as { sessions?: ScanSessionRecord[] } | null)?.sessions ?? [];
  const pv = progress.value;
  const buildActive = building && pv != null && pv.stage !== "done" && pv.stage !== "failed";

  return (
    <div className="h-full overflow-y-auto p-3 text-[12px]" data-testid="scan-panel">
      {/* 세션 */}
      <section className="mb-3">
        <div className="mb-1 font-mono uppercase text-muted-foreground">session</div>
        <div className="flex flex-wrap items-center gap-2">
          <Button size="sm" onClick={onNewSession} data-testid="new-session">
            새 세션
          </Button>
          <span className="text-muted-foreground" data-testid="session-current">
            {sessionRowId == null ? "선택 안 됨" : `#${sessionRowId}`}
          </span>
        </div>
        {sessions.length > 0 && (
          <div className="mt-1 flex flex-col gap-1" data-testid="session-list">
            {sessions.slice(0, 5).map((s) => (
              <button
                key={s.id}
                onClick={() => s.id != null && onSelectSession(s.id, s.session_id)}
                className={`rounded border px-2 py-1 text-left font-mono ${
                  s.id === sessionRowId ? "border-emerald-500" : "border-zinc-700"
                }`}
              >
                #{s.id} {s.session_id}
              </button>
            ))}
          </div>
        )}
      </section>

      {/* 캡처 */}
      <section className="mb-3">
        <div className="mb-1 font-mono uppercase text-muted-foreground">capture</div>
        <div className="flex flex-wrap gap-2">
          <Button
            size="sm"
            onClick={onCapture}
            disabled={sessionRowId == null}
            data-testid="capture"
          >
            캡처 ({scans.length})
          </Button>
          <Button
            size="sm"
            variant="outline"
            onClick={onUndo}
            disabled={sessionRowId == null || scans.length === 0}
            data-testid="undo"
          >
            취소
          </Button>
        </div>
      </section>

      {/* 빌드 */}
      <section className="mb-3">
        <div className="mb-1 font-mono uppercase text-muted-foreground">build</div>
        <div className="mb-1 flex items-center gap-2">
          <span className="text-muted-foreground">품질 (voxel)</span>
          <select
            value={buildVoxelM}
            onChange={(e) => setBuildVoxelM(Number(e.target.value))}
            data-testid="build-voxel"
            className="rounded border border-zinc-700 bg-zinc-900 px-1 py-0.5 font-mono"
          >
            {VOXEL_TIERS.map((t) => (
              <option key={t.m} value={t.m}>
                {t.label}
              </option>
            ))}
          </select>
        </div>
        <Button
          size="sm"
          variant="secondary"
          onClick={onBuild}
          disabled={sessionRowId == null || scans.length < 2 || building}
          data-testid="build"
        >
          {building ? "빌드 중…" : "TSDF 빌드"}
        </Button>
        {buildActive && pv && (
          <div className="mt-2" data-testid="build-progress">
            <div className="mb-1 flex justify-between text-muted-foreground">
              <span>{pv.stage}</span>
              <span>{Math.round(pv.percent * 100)}%</span>
            </div>
            <div className="h-1.5 w-full overflow-hidden rounded bg-zinc-800">
              <div
                className="h-full bg-emerald-500 transition-all"
                style={{ width: `${Math.round(pv.percent * 100)}%` }}
              />
            </div>
          </div>
        )}
      </section>

      {/* reconstruction 결과 */}
      <section className="mb-3">
        <div className="mb-1 font-mono uppercase text-muted-foreground">
          reconstructions
        </div>
        <div className="flex flex-col gap-1" data-testid="recon-list">
          {recons.length === 0 ? (
            <span className="text-muted-foreground">없음</span>
          ) : (
            recons.map((r) => (
              <div
                key={r.id}
                className="flex items-center justify-between rounded border border-zinc-700 px-2 py-1"
              >
                <span className="font-mono">
                  #{r.id} · {r.vertex_count}v
                </span>
                <Button
                  size="sm"
                  variant="ghost"
                  onClick={() => r.id != null && onViewMesh(r.id)}
                  data-testid="view-mesh"
                >
                  보기
                </Button>
              </div>
            ))
          )}
        </div>
      </section>

      <div className="text-muted-foreground" data-testid="scan-msg">
        {msg}
      </div>
    </div>
  );
}
