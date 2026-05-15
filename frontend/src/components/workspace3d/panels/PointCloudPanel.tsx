import { useEffect, useState } from "react";
import {
  Cloud,
  Camera,
  FolderOpen,
  X,
  RefreshCw,
  Plus,
  Boxes,
  Eye,
  EyeOff,
} from "lucide-react";
import type { IDockviewPanelProps } from "dockview";
import { usePointCloudStore } from "@/store/pointCloudStore";
import { PanelShell } from "../ui/PanelShell";
import { Section } from "../ui/Section";
import { ToggleRow } from "../ui/ToggleRow";

const VOXEL_PRESETS = [0.003, 0.005, 0.008];
const CAPTURE_FRAMES = 5;
const MESH_VOXEL_PRESETS = [0.002, 0.003, 0.005]; // 2/3/5mm

export function PointCloudPanel(props: IDockviewPanelProps<object>) {
  const enabled = usePointCloudStore((s) => s.enabled);
  const voxelSize = usePointCloudStore((s) => s.voxelSize);
  const frame = usePointCloudStore((s) => s.frame);
  const snapshot = usePointCloudStore((s) => s.snapshot);
  const snapshotLabel = usePointCloudStore((s) => s.snapshotLabel);
  const sessions = usePointCloudStore((s) => s.sessions);
  const scans = usePointCloudStore((s) => s.scans);
  const currentSessionId = usePointCloudStore((s) => s.currentSessionId);
  const busy = usePointCloudStore((s) => s.busy);

  const meshes = usePointCloudStore((s) => s.meshes);
  const meshPath = usePointCloudStore((s) => s.meshPath);
  const meshVisible = usePointCloudStore((s) => s.meshVisible);
  const meshBusy = usePointCloudStore((s) => s.meshBusy);

  const setEnabled = usePointCloudStore((s) => s.setEnabled);
  const setVoxelSize = usePointCloudStore((s) => s.setVoxelSize);
  const newSession = usePointCloudStore((s) => s.newSession);
  const capture = usePointCloudStore((s) => s.capture);
  const refreshSessions = usePointCloudStore((s) => s.refreshSessions);
  const selectSession = usePointCloudStore((s) => s.selectSession);
  const loadScan = usePointCloudStore((s) => s.loadScan);
  const clearSnapshot = usePointCloudStore((s) => s.clearSnapshot);

  const buildMesh = usePointCloudStore((s) => s.buildMesh);
  const refreshMeshes = usePointCloudStore((s) => s.refreshMeshes);
  const showMesh = usePointCloudStore((s) => s.showMesh);
  const setMeshVisible = usePointCloudStore((s) => s.setMeshVisible);

  const [status, setStatus] = useState<string>("");
  const [meshVoxel, setMeshVoxel] = useState<number>(MESH_VOXEL_PRESETS[0]);

  useEffect(() => {
    refreshSessions();
    refreshMeshes();
  }, [refreshSessions, refreshMeshes]);

  const handleNewSession = async () => {
    const res = await newSession();
    setStatus(
      res.success ? `세션 시작: ${res.sessionId}` : `실패: ${res.message}`
    );
  };

  const handleCapture = async () => {
    const res = await capture(CAPTURE_FRAMES);
    setStatus(
      res.success ? `캡처: ${res.plyPath}` : `실패: ${res.message}`
    );
  };

  const handleLoad = async (plyPath: string) => {
    const res = await loadScan(plyPath);
    setStatus(res.success ? `로드: ${plyPath}` : `실패: ${res.message}`);
  };

  const handleClear = async () => {
    await clearSnapshot();
    setStatus("snapshot 제거");
  };

  const handleBuildMesh = async () => {
    if (!currentSessionId) {
      setStatus("실패: 세션 미선택");
      return;
    }
    setStatus(`메시 빌드 중 (voxel=${(meshVoxel * 1000).toFixed(0)}mm)…`);
    const res = await buildMesh(currentSessionId, { voxelSize: meshVoxel });
    if (res.success) {
      setStatus(
        `메시: ${res.path} (V=${res.vertexCount}, F=${res.triangleCount}, ` +
          `${res.integratedScans}/${res.totalScans} scans, ${res.elapsed?.toFixed(1)}s)`
      );
    } else {
      setStatus(`실패: ${res.message}`);
    }
  };

  return (
    <PanelShell
      icon={<Cloud className="w-3.5 h-3.5" />}
      title="Point Cloud"
      panelId={props.api.id}
      api={props.api}
    >
      <Section label="Live Stream">
        <ToggleRow
          label={enabled ? "Streaming" : "Off"}
          checked={enabled}
          onChange={() => setEnabled(!enabled)}
          accentColor="bg-emerald-400"
        />
      </Section>

      <Section label="Voxel Size">
        <div className="grid grid-cols-3 gap-1">
          {VOXEL_PRESETS.map((v) => {
            const active = Math.abs(v - voxelSize) < 1e-6;
            return (
              <button
                key={v}
                onClick={() => setVoxelSize(v)}
                className={`text-[10px] font-mono py-1 rounded transition-colors ${
                  active
                    ? "bg-emerald-500/20 text-emerald-300"
                    : "bg-zinc-900 text-zinc-500 hover:bg-zinc-800"
                }`}
              >
                {(v * 1000).toFixed(0)}mm
              </button>
            );
          })}
        </div>
      </Section>

      <Section label="Session">
        <div className="flex flex-col gap-2">
          <div className="flex items-center gap-1">
            <select
              value={currentSessionId ?? ""}
              onChange={(e) => selectSession(e.target.value || null)}
              className="flex-1 text-[11px] font-mono bg-zinc-900 text-zinc-300 rounded px-2 py-1.5 border border-zinc-800 focus:outline-none focus:border-zinc-700"
            >
              <option value="">(none)</option>
              {sessions.map((s) => (
                <option key={s.session_id} value={s.session_id}>
                  {s.session_id} ({s.scan_count})
                </option>
              ))}
            </select>
            <button
              onClick={handleNewSession}
              disabled={busy}
              className="flex items-center justify-center px-2 text-[11px] font-mono py-1.5 rounded bg-sky-500/20 text-sky-300 hover:bg-sky-500/30 disabled:opacity-50 transition-colors"
              title="새 세션 시작"
            >
              <Plus className="w-3 h-3" />
            </button>
            <button
              onClick={refreshSessions}
              className="flex items-center justify-center px-2 text-[11px] font-mono py-1.5 rounded bg-zinc-800 text-zinc-400 hover:bg-zinc-700 transition-colors"
              title="세션 목록 새로고침"
            >
              <RefreshCw className="w-3 h-3" />
            </button>
          </div>
        </div>
      </Section>

      <Section label="Capture">
        <div className="flex gap-1">
          <button
            onClick={handleCapture}
            disabled={busy}
            className="flex-1 flex items-center justify-center gap-1 text-[11px] font-mono py-1.5 rounded bg-emerald-500/20 text-emerald-300 hover:bg-emerald-500/30 disabled:opacity-50 transition-colors"
          >
            <Camera className="w-3 h-3" />
            Capture ({CAPTURE_FRAMES})
          </button>
          <button
            onClick={handleClear}
            disabled={!snapshot && !snapshotLabel}
            className="flex items-center justify-center px-2 text-[11px] font-mono py-1.5 rounded bg-zinc-800 text-zinc-400 hover:bg-zinc-700 disabled:opacity-30 transition-colors"
            title="snapshot 제거"
          >
            <X className="w-3 h-3" />
          </button>
        </div>
        {snapshotLabel && (
          <p className="mt-2 text-[10px] font-mono text-zinc-500 truncate">
            {snapshotLabel}
          </p>
        )}
      </Section>

      <Section label="Library">
        <div className="flex items-center justify-between mb-2">
          <span className="text-[10px] font-mono text-zinc-500">
            {scans.length} scan
          </span>
        </div>
        <div className="flex flex-col gap-1 max-h-40 overflow-y-auto">
          {scans.length === 0 ? (
            <p className="text-[10px] font-mono text-zinc-600 text-center py-2">
              {currentSessionId ? "(empty)" : "(no session)"}
            </p>
          ) : (
            scans.map((scan) => (
              <button
                key={scan.ply_path}
                onClick={() => handleLoad(scan.ply_path)}
                disabled={busy}
                className="flex items-center gap-2 px-2 py-1 rounded text-[10px] font-mono text-zinc-300 bg-zinc-900 hover:bg-zinc-800 disabled:opacity-50 transition-colors text-left"
                title={scan.ply_path}
              >
                <FolderOpen className="w-3 h-3 shrink-0 text-zinc-500" />
                <span className="truncate flex-1">{scan.name}</span>
                <span className="text-zinc-600 shrink-0">
                  {(scan.size / 1024).toFixed(0)}KB
                </span>
              </button>
            ))
          )}
        </div>
      </Section>

      <Section label="Mesh (TSDF)">
        <div className="flex flex-col gap-2">
          <div className="grid grid-cols-3 gap-1">
            {MESH_VOXEL_PRESETS.map((v) => {
              const active = Math.abs(v - meshVoxel) < 1e-6;
              return (
                <button
                  key={v}
                  onClick={() => setMeshVoxel(v)}
                  className={`text-[10px] font-mono py-1 rounded transition-colors ${
                    active
                      ? "bg-violet-500/20 text-violet-300"
                      : "bg-zinc-900 text-zinc-500 hover:bg-zinc-800"
                  }`}
                  title="TSDF voxel size"
                >
                  {(v * 1000).toFixed(0)}mm
                </button>
              );
            })}
          </div>
          <div className="flex gap-1">
            <button
              onClick={handleBuildMesh}
              disabled={meshBusy || !currentSessionId}
              className="flex-1 flex items-center justify-center gap-1 text-[11px] font-mono py-1.5 rounded bg-violet-500/20 text-violet-300 hover:bg-violet-500/30 disabled:opacity-50 transition-colors"
              title={
                currentSessionId
                  ? `세션 ${currentSessionId}의 모든 scan을 TSDF로 적분`
                  : "세션 선택 필요"
              }
            >
              <Boxes className="w-3 h-3" />
              {meshBusy ? "Building…" : "Build Mesh"}
            </button>
            <button
              onClick={() => setMeshVisible(!meshVisible)}
              disabled={!meshPath}
              className="flex items-center justify-center px-2 text-[11px] font-mono py-1.5 rounded bg-zinc-800 text-zinc-400 hover:bg-zinc-700 disabled:opacity-30 transition-colors"
              title={meshVisible ? "메시 숨기기" : "메시 보이기"}
            >
              {meshVisible ? (
                <Eye className="w-3 h-3" />
              ) : (
                <EyeOff className="w-3 h-3" />
              )}
            </button>
            <button
              onClick={refreshMeshes}
              className="flex items-center justify-center px-2 text-[11px] font-mono py-1.5 rounded bg-zinc-800 text-zinc-400 hover:bg-zinc-700 transition-colors"
              title="메시 목록 새로고침"
            >
              <RefreshCw className="w-3 h-3" />
            </button>
          </div>
          <div className="flex flex-col gap-1 max-h-32 overflow-y-auto">
            {meshes.length === 0 ? (
              <p className="text-[10px] font-mono text-zinc-600 text-center py-2">
                (no mesh)
              </p>
            ) : (
              meshes.map((m) => {
                const active = m.path === meshPath;
                return (
                  <button
                    key={m.path}
                    onClick={() => showMesh(m.path)}
                    className={`flex items-center gap-2 px-2 py-1 rounded text-[10px] font-mono text-left transition-colors ${
                      active
                        ? "bg-violet-500/20 text-violet-200"
                        : "bg-zinc-900 text-zinc-300 hover:bg-zinc-800"
                    }`}
                    title={m.path}
                  >
                    <Boxes className="w-3 h-3 shrink-0 text-violet-400/70" />
                    <span className="truncate flex-1">{m.name}</span>
                    <span className="text-zinc-600 shrink-0">
                      {(m.size / 1024).toFixed(0)}KB
                    </span>
                  </button>
                );
              })
            )}
          </div>
        </div>
      </Section>

      <Section label="Stats">
        <div className="text-[11px] font-mono space-y-1">
          <div className="flex justify-between">
            <span className="text-zinc-500">Live</span>
            <span className="text-zinc-300">
              {frame ? frame.count.toLocaleString() : "—"}
            </span>
          </div>
          <div className="flex justify-between">
            <span className="text-zinc-500">Snapshot</span>
            <span className="text-zinc-300">
              {snapshot ? snapshot.count.toLocaleString() : "—"}
            </span>
          </div>
        </div>
        {status && (
          <p className="mt-2 text-[10px] font-mono text-zinc-400 break-words">
            {status}
          </p>
        )}
      </Section>
    </PanelShell>
  );
}
