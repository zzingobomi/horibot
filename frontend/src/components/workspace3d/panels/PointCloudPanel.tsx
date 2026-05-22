import { useEffect, useState } from "react";
import { Cloud, Camera, Trash2, Hammer, RefreshCw } from "lucide-react";
import type { IDockviewPanelProps } from "dockview";
import { usePointCloudStore } from "@/store/pointCloudStore";
import { PanelShell } from "../ui/PanelShell";
import { Section } from "../ui/Section";
import { ToggleRow } from "../ui/ToggleRow";

const VOXEL_PRESETS = [0.003, 0.005, 0.008];
const BUILD_VOXEL_PRESETS = [0.001, 0.002, 0.003];

function formatTimestamp(ts: number): string {
  const d = new Date(ts * 1000);
  const pad = (n: number) => n.toString().padStart(2, "0");
  return `${pad(d.getHours())}:${pad(d.getMinutes())}:${pad(d.getSeconds())}`;
}

export function PointCloudPanel(props: IDockviewPanelProps<object>) {
  // live stream
  const enabled = usePointCloudStore((s) => s.enabled);
  const voxelSize = usePointCloudStore((s) => s.voxelSize);
  const frame = usePointCloudStore((s) => s.frame);
  const setEnabled = usePointCloudStore((s) => s.setEnabled);
  const setVoxelSize = usePointCloudStore((s) => s.setVoxelSize);

  // session
  const currentSessionId = usePointCloudStore((s) => s.currentSessionId);
  const sessions = usePointCloudStore((s) => s.sessions);
  const scans = usePointCloudStore((s) => s.scans);
  const capturing = usePointCloudStore((s) => s.capturing);
  const lastCaptureMessage = usePointCloudStore((s) => s.lastCaptureMessage);
  const selectSession = usePointCloudStore((s) => s.selectSession);
  const newSession = usePointCloudStore((s) => s.newSession);
  const refreshSessions = usePointCloudStore((s) => s.refreshSessions);
  const refreshScans = usePointCloudStore((s) => s.refreshScans);
  const capture = usePointCloudStore((s) => s.capture);
  const deleteScan = usePointCloudStore((s) => s.deleteScan);

  // mesh
  const meshes = usePointCloudStore((s) => s.meshes);
  const meshVisible = usePointCloudStore((s) => s.meshVisible);
  const meshPath = usePointCloudStore((s) => s.meshPath);
  const meshBusy = usePointCloudStore((s) => s.meshBusy);
  const lastBuildResult = usePointCloudStore((s) => s.lastBuildResult);
  const lastBuildError = usePointCloudStore((s) => s.lastBuildError);
  const refreshMeshes = usePointCloudStore((s) => s.refreshMeshes);
  const buildMesh = usePointCloudStore((s) => s.buildMesh);
  const showMesh = usePointCloudStore((s) => s.showMesh);
  const setMeshVisible = usePointCloudStore((s) => s.setMeshVisible);

  // build params
  const [buildVoxel, setBuildVoxel] = useState(0.002);

  // 마운트 시 1회 fetch
  useEffect(() => {
    refreshSessions();
    refreshMeshes();
  }, [refreshSessions, refreshMeshes]);

  // 세션 바뀌면 scan 재조회
  useEffect(() => {
    if (currentSessionId) refreshScans();
  }, [currentSessionId, refreshScans]);

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

      <Section label="Live Voxel Size">
        <div className="grid grid-cols-4 gap-1">
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

      <Section label="Stats">
        <div className="text-[11px] font-mono space-y-1">
          <div className="flex justify-between">
            <span className="text-zinc-500">Live</span>
            <span className="text-zinc-300">
              {frame ? frame.count.toLocaleString() : "—"}
            </span>
          </div>
        </div>
      </Section>

      {/* ─── Session ─── */}
      <Section label="Session">
        <div className="space-y-1.5">
          <div className="flex items-center justify-between text-[11px] font-mono">
            <span className="text-zinc-500">Current</span>
            <span className="text-zinc-300 truncate ml-2 max-w-37.5">
              {currentSessionId ?? "—"}
            </span>
          </div>
          <div className="flex gap-1">
            <button
              onClick={() => newSession()}
              className="flex-1 text-[10px] font-mono py-1 rounded bg-sky-500/20 text-sky-300 hover:bg-sky-500/30"
            >
              새 세션
            </button>
            <button
              onClick={() => refreshSessions()}
              className="px-2 py-1 rounded bg-zinc-900 text-zinc-500 hover:bg-zinc-800"
              title="목록 새로고침"
            >
              <RefreshCw className="w-3 h-3" />
            </button>
          </div>
          {sessions.length > 0 && (
            <select
              value={currentSessionId ?? ""}
              onChange={(e) => {
                if (e.target.value) selectSession(e.target.value);
              }}
              className="w-full text-[10px] font-mono bg-zinc-900 text-zinc-300 border border-zinc-800 rounded px-1.5 py-1"
            >
              <option value="" disabled>
                세션 선택…
              </option>
              {sessions.map((s) => (
                <option key={s} value={s}>
                  {s}
                </option>
              ))}
            </select>
          )}
        </div>
      </Section>

      {/* ─── Capture ─── */}
      <Section label={`Capture (${scans.length})`}>
        <div className="space-y-1.5">
          <button
            onClick={() => capture()}
            disabled={!currentSessionId || !enabled || capturing}
            className="w-full text-[11px] font-mono py-1.5 rounded bg-emerald-500/20 text-emerald-300 hover:bg-emerald-500/30 disabled:bg-zinc-900 disabled:text-zinc-600 flex items-center justify-center gap-1.5"
          >
            <Camera className="w-3 h-3" />
            {capturing ? "캡처 중…" : "캡처"}
          </button>
          {!enabled && (
            <p className="text-[9px] text-amber-400/70 font-mono">
              ※ Live Stream 먼저 ON
            </p>
          )}
          {lastCaptureMessage && (
            <p className="text-[9px] text-zinc-500 font-mono break-all">
              {lastCaptureMessage}
            </p>
          )}
          {scans.length > 0 && (
            <div className="max-h-32 overflow-y-auto space-y-0.5">
              {scans.map((scan) => (
                <div
                  key={scan.id}
                  className="flex items-center justify-between text-[10px] font-mono px-1.5 py-1 bg-zinc-900/50 rounded"
                >
                  <span className="text-zinc-400">
                    #{scan.id.toString().padStart(3, "0")}
                  </span>
                  <span className="text-zinc-500">
                    {formatTimestamp(scan.timestamp)}
                  </span>
                  <span className="text-zinc-600">{scan.num_frames}f</span>
                  <button
                    onClick={() => deleteScan(scan.id)}
                    className="text-zinc-600 hover:text-rose-400"
                    title="삭제"
                  >
                    <Trash2 className="w-3 h-3" />
                  </button>
                </div>
              ))}
            </div>
          )}
        </div>
      </Section>

      {/* ─── Build Mesh ─── */}
      <Section label="Build Mesh">
        <div className="space-y-1.5">
          <div>
            <p className="text-[9px] text-zinc-600 mb-1 font-mono">
              voxel_size
            </p>
            <div className="grid grid-cols-3 gap-1">
              {BUILD_VOXEL_PRESETS.map((v) => {
                const active = Math.abs(v - buildVoxel) < 1e-6;
                return (
                  <button
                    key={v}
                    onClick={() => setBuildVoxel(v)}
                    className={`text-[10px] font-mono py-1 rounded ${
                      active
                        ? "bg-violet-500/20 text-violet-300"
                        : "bg-zinc-900 text-zinc-500 hover:bg-zinc-800"
                    }`}
                  >
                    {(v * 1000).toFixed(0)}mm
                  </button>
                );
              })}
            </div>
          </div>
          <button
            onClick={() => buildMesh({ voxel_size: buildVoxel })}
            disabled={!currentSessionId || scans.length < 2 || meshBusy}
            className="w-full text-[11px] font-mono py-1.5 rounded bg-violet-500/20 text-violet-300 hover:bg-violet-500/30 disabled:bg-zinc-900 disabled:text-zinc-600 flex items-center justify-center gap-1.5"
          >
            <Hammer className="w-3 h-3" />
            {meshBusy ? "Building…" : "BUILD"}
          </button>
          {scans.length < 2 && (
            <p className="text-[9px] text-amber-400/70 font-mono">
              ※ scan 2개 이상 필요
            </p>
          )}
          {lastBuildError && (
            <p className="text-[9px] text-rose-400 font-mono break-all">
              {lastBuildError}
            </p>
          )}
          {lastBuildResult && (
            <div className="text-[9px] font-mono text-zinc-500 space-y-0.5">
              <div className="flex justify-between">
                <span>verts/tris</span>
                <span className="text-zinc-300">
                  {lastBuildResult.vertex_count.toLocaleString()} /{" "}
                  {lastBuildResult.triangle_count.toLocaleString()}
                </span>
              </div>
              <div className="flex justify-between">
                <span>scans/edges</span>
                <span className="text-zinc-300">
                  {lastBuildResult.n_scans} / {lastBuildResult.n_edges}
                </span>
              </div>
              <div className="flex justify-between">
                <span>elapsed</span>
                <span className="text-zinc-300">
                  {lastBuildResult.elapsed.toFixed(1)}s
                </span>
              </div>
            </div>
          )}
        </div>
      </Section>

      {/* ─── Meshes ─── */}
      <Section label="Meshes">
        <div className="space-y-1.5">
          <ToggleRow
            label={meshVisible ? "Visible" : "Hidden"}
            checked={meshVisible}
            onChange={setMeshVisible}
            accentColor="bg-violet-400"
          />
          {meshes.length === 0 ? (
            <p className="text-[10px] text-zinc-600 font-mono">
              저장된 mesh 없음
            </p>
          ) : (
            <div className="max-h-32 overflow-y-auto space-y-0.5">
              {meshes.map((m) => {
                const active = m.path === meshPath;
                return (
                  <button
                    key={m.path}
                    onClick={() => showMesh(m.path)}
                    className={`w-full text-left text-[10px] font-mono px-1.5 py-1 rounded ${
                      active
                        ? "bg-violet-500/20 text-violet-300"
                        : "bg-zinc-900/50 text-zinc-400 hover:bg-zinc-800"
                    }`}
                  >
                    <span className="truncate block">{m.session_id}</span>
                  </button>
                );
              })}
            </div>
          )}
        </div>
      </Section>
    </PanelShell>
  );
}
