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
import { ServiceKey, TaskStatus, Topic } from "@/api/generated/contract";
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

// 빌드 voxel 마지막 선택 (m) — 자동/수동 스캔 공용 품질 노브.
const BUILD_VOXEL_LS_KEY = "scan.buildVoxelM";

/** 재구성 생성 시각 → "N분/시간/일 전" (stale 월드 침묵 금지 라벨). */
function agoLabel(iso: string): string {
  const ms = Date.now() - new Date(iso).getTime();
  if (!Number.isFinite(ms) || ms < 0) return "";
  const min = Math.floor(ms / 60_000);
  if (min < 1) return "방금";
  if (min < 60) return `${min}분 전`;
  const hr = Math.floor(min / 60);
  if (hr < 24) return `${hr}시간 전`;
  return `${Math.floor(hr / 24)}일 전`;
}

export function ScanPanel() {
  const robotId = useRobotId();

  // ── 자동 스캔 (world_scan task — 로봇이 스스로 관측 자세를 돌며 스캔) ──
  const autoRun = useService(ServiceKey.WORLDSCAN_RUN, robotId);
  const autoStop = useService(ServiceKey.WORLDSCAN_STOP, robotId);
  const wsState = useStream(Topic.WORLDSCAN_STATE, { robotId });
  // ── world 표시 (배경 메시) — pick 패널에서 이전 (world 소유 = 스캔) ──
  const worldVisible = useScanStore((s) => s.worldVisible);
  const setWorldVisible = useScanStore((s) => s.setWorldVisible);
  const meshMeta = useScanStore((s) => s.meshMeta);

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

  // 자동 스캔 상태 (world_scan STATE 스트림). RUNNING 이면 시작 잠금 (robot-busy —
  // 한 로봇은 한 번에 한 task. cross-task(pick 동시) 잠금은 후속 arbitration).
  const wsStatus = wsState.value?.status ?? TaskStatus.IDLE;
  const scanning = wsStatus === TaskStatus.RUNNING;

  const onAutoScan = async () => {
    setMsg("자동 스캔 시작…");
    const res = await autoRun.call({ voxel_size: buildVoxelM });
    const d = res.data as { accepted?: boolean; message?: string } | null;
    setMsg(
      d?.accepted ? "자동 스캔 진행 중 — 상태는 아래" : `거부: ${d?.message ?? res.message}`,
    );
  };
  const onAutoStop = async () => {
    const res = await autoStop.call({});
    const d = res.data as { ok?: boolean; message?: string } | null;
    setMsg(d?.ok ? "중지 요청 (모션 정지)" : `중지 실패: ${d?.message ?? res.message}`);
  };

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
      {/* 자동 스캔 (주인공) — 로봇이 스스로 관측 자세를 돌며 캡처 → 끝에 mesh 빌드 */}
      <section className="mb-3">
        <div className="mb-1 font-mono uppercase text-muted-foreground">자동 스캔</div>
        <div className="mb-1 flex items-center gap-2">
          <span className="text-muted-foreground">품질 (voxel)</span>
          <select
            value={buildVoxelM}
            onChange={(e) => setBuildVoxelM(Number(e.target.value))}
            data-testid="auto-voxel"
            className="rounded border border-zinc-700 bg-zinc-900 px-1 py-0.5 font-mono"
          >
            {VOXEL_TIERS.map((t) => (
              <option key={t.m} value={t.m}>
                {t.label}
              </option>
            ))}
          </select>
        </div>
        <div className="flex gap-2">
          <Button
            size="sm"
            onClick={onAutoScan}
            disabled={scanning || !robotId}
            data-testid="auto-scan"
          >
            {scanning ? "스캔 중…" : "자동 스캔 시작"}
          </Button>
          <Button
            size="sm"
            variant="ghost"
            onClick={onAutoStop}
            disabled={!scanning}
            data-testid="auto-stop"
          >
            중지
          </Button>
        </div>
        {/* 진행 상태 (world_scan STATE) — 실패는 사유 표시 (침묵 금지) */}
        <div
          className="mt-1 flex items-center gap-2 text-muted-foreground"
          data-testid="auto-status"
        >
          <span className="font-mono">{wsStatus}</span>
          {wsState.value?.current_title && (
            <span className="truncate">· {wsState.value.current_title}</span>
          )}
        </div>
        {wsState.value?.error && (
          <div
            className="mt-1 rounded border border-red-800/60 bg-red-950/30 p-2 font-mono text-red-300"
            data-testid="auto-error"
          >
            {wsState.value.error}
          </div>
        )}
      </section>

      {/* world 표시 (배경 메시) — pick 패널에서 이전 (world 소유 = 스캔) */}
      <section className="mb-3">
        <div className="mb-1 font-mono uppercase text-muted-foreground">
          world (배경 메시)
        </div>
        <label className="flex items-center gap-2">
          <input
            type="checkbox"
            checked={worldVisible}
            onChange={(e) => setWorldVisible(e.target.checked)}
            data-testid="world-visible"
          />
          <span>월드 표시</span>
        </label>
        <div className="mt-1 text-muted-foreground" data-testid="world-label">
          {meshMeta?.createdAt
            ? `현재 월드: ${agoLabel(meshMeta.createdAt)} 스캔 · ` +
              `${meshMeta.vertexCount.toLocaleString()} verts`
            : "월드 없음 — 위 '자동 스캔'으로 생성"}
        </div>
      </section>

      {/* 수동 스캔 (고급 — 손으로 팔 옮겨 캡처. 자동이 못 담는 각도 톱업용) */}
      <details className="mb-3" data-testid="manual-scan">
        <summary className="mb-2 cursor-pointer font-mono uppercase text-muted-foreground">
          수동 스캔 (고급)
        </summary>
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
      </details>

      <div className="text-muted-foreground" data-testid="scan-msg">
        {msg}
      </div>
    </div>
  );
}
