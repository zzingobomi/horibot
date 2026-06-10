/**
 * Calibration rollback picker — `.history/` snapshot 리스트 + restore.
 *
 * 사용처:
 *   - σ 후퇴 비교 ("지난 주 σ vs 오늘")
 *   - 외부 스크립트로 disk 망친 후 safe baseline 으로 되돌리기
 *   - COMMIT 실수 직전 상태 (pre-commit snapshot) 으로 한 단계 뒤
 *
 * `restart_required=true` 받으면 link_offsets URDF patch 재적용 위해 페이지 reload
 * 권장 메시지.
 */
import { useCallback, useEffect, useState } from "react";
import { History } from "lucide-react";
import type { IDockviewPanelProps } from "dockview";
import { useParams } from "react-router-dom";
import { bridge } from "@/api/bridge";
import { PanelShell } from "@/components/shared/PanelShell";
import { PanelButton } from "@/components/shared/PanelButton";
import { Section } from "@/components/shared/Section";
import { ServiceKey } from "@/constants/topics";
import type { BackupEntry } from "./parts/types";

export function RollbackPanel(props: IDockviewPanelProps<object>) {
  const { id: robotId = "" } = useParams<{ id: string }>();
  const [entries, setEntries] = useState<BackupEntry[]>([]);
  const [loading, setLoading] = useState(false);
  const [status, setStatus] = useState("");
  const [restoring, setRestoring] = useState<string | null>(null);

  const refresh = useCallback(async () => {
    setLoading(true);
    const res = await bridge.callService(ServiceKey.CALIB_BACKUP_LIST, {});
    setLoading(false);
    if (res.success) {
      const data = res.data as unknown as { snapshots: BackupEntry[] };
      setEntries(data.snapshots ?? []);
    } else {
      setStatus(`실패: ${res.message}`);
    }
  }, []);

  // mount 시 fetch — .then 콜백으로 라우팅 (react-hooks/set-state-in-effect).
  useEffect(() => {
    let cancelled = false;
    bridge.callService(ServiceKey.CALIB_BACKUP_LIST, {}).then((res) => {
      if (cancelled || !res.success) return;
      const data = res.data as unknown as { snapshots: BackupEntry[] };
      setEntries(data.snapshots ?? []);
    });
    return () => {
      cancelled = true;
    };
  }, [robotId]);

  const handleRestore = async (entry: BackupEntry) => {
    const σ = formatSigma(entry);
    if (
      !confirm(
        `${entry.timestamp} (${entry.tag}${σ}) 로 복원합니다.\n\n` +
          `링크 URDF 재적용을 위해 백엔드 재시작이 필요할 수 있습니다.\n계속할까요?`,
      )
    ) {
      return;
    }
    setRestoring(entry.timestamp);
    const res = await bridge.callService(ServiceKey.CALIB_BACKUP_RESTORE, {
      timestamp: entry.timestamp,
    });
    setRestoring(null);
    if (res.success) {
      const data = res.data as unknown as {
        restored_timestamp: string;
        restart_required: boolean;
      };
      setStatus(
        `${data.restored_timestamp} 복원 완료` +
          (data.restart_required ? " — 백엔드 재시작 필요" : ""),
      );
      await refresh();
    } else {
      setStatus(`실패: ${res.message}`);
    }
  };

  return (
    <PanelShell
      icon={<History className="w-3.5 h-3.5" />}
      title="Rollback"
      panelId={props.api.id}
      api={props.api}
      expandedHeight={360}
    >
      <Section label="안내">
        <div className="flex items-start justify-between gap-2">
          <p className="text-[11px] text-zinc-500 leading-snug font-mono">
            매 COMMIT 직전 disk 가 자동 백업됩니다. σ 비교 / 후퇴 대비.
          </p>
          <PanelButton
            variant="ghost"
            className="shrink-0 !px-2 !py-0.5 !text-[10px]"
            onClick={() => void refresh()}
            disabled={loading}
          >
            {loading ? "..." : "새로고침"}
          </PanelButton>
        </div>
      </Section>

      <Section label="Snapshots">
        <div className="rounded border border-zinc-800/60 bg-black/20 overflow-hidden">
          {entries.length === 0 ? (
            <p className="text-[11px] text-zinc-500 p-3 font-mono">
              저장된 snapshot 없음. 한 번 COMMIT 하면 자동으로 쌓입니다.
            </p>
          ) : (
            <div className="max-h-72 overflow-y-auto">
              <table className="w-full text-[11px] font-mono">
                <thead className="sticky top-0 bg-zinc-900/80 backdrop-blur">
                  <tr className="text-left text-zinc-500">
                    <th className="px-2 py-1.5 font-normal">timestamp</th>
                    <th className="px-2 py-1.5 font-normal">tag</th>
                    <th className="px-2 py-1.5 font-normal text-right">
                      σ_rot
                    </th>
                    <th className="px-2 py-1.5 font-normal text-right">σ_t</th>
                    <th className="px-2 py-1.5 font-normal text-right">caps</th>
                    <th className="px-2 py-1.5" />
                  </tr>
                </thead>
                <tbody>
                  {entries.map((e) => (
                    <tr
                      key={e.timestamp}
                      className="border-t border-zinc-800/40 text-zinc-300"
                    >
                      <td className="px-2 py-1.5">{formatTs(e.timestamp)}</td>
                      <td className="px-2 py-1.5 text-zinc-500">
                        {e.tag || "—"}
                      </td>
                      <td className="px-2 py-1.5 text-right">
                        {e.sigma_rot_deg !== null
                          ? `${e.sigma_rot_deg.toFixed(2)}°`
                          : "—"}
                      </td>
                      <td className="px-2 py-1.5 text-right">
                        {e.sigma_t_mm !== null
                          ? `${e.sigma_t_mm.toFixed(1)}mm`
                          : "—"}
                      </td>
                      <td className="px-2 py-1.5 text-right text-zinc-500">
                        {e.capture_count ?? "—"}
                      </td>
                      <td className="px-2 py-1.5 text-right">
                        <PanelButton
                          variant="outline"
                          className="!px-2 !py-0.5 !text-[10px]"
                          onClick={() => handleRestore(e)}
                          disabled={restoring !== null}
                        >
                          {restoring === e.timestamp ? "복원중..." : "복원"}
                        </PanelButton>
                      </td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          )}
        </div>
        {status && (
          <p className="text-[11px] text-zinc-400 mt-2 leading-snug font-mono">
            {status}
          </p>
        )}
      </Section>
    </PanelShell>
  );
}

function formatTs(ts: string): string {
  // YYYYMMDDTHHMMSS → YYYY-MM-DD HH:MM:SS
  if (ts.length === 15 && ts[8] === "T") {
    return `${ts.slice(0, 4)}-${ts.slice(4, 6)}-${ts.slice(6, 8)} ${ts.slice(
      9,
      11,
    )}:${ts.slice(11, 13)}:${ts.slice(13, 15)}`;
  }
  return ts;
}

function formatSigma(e: BackupEntry): string {
  if (e.sigma_rot_deg === null && e.sigma_t_mm === null) return "";
  const r =
    e.sigma_rot_deg !== null ? `σ_rot=${e.sigma_rot_deg.toFixed(2)}°` : "";
  const t = e.sigma_t_mm !== null ? `σ_t=${e.sigma_t_mm.toFixed(1)}mm` : "";
  return `, ${[r, t].filter(Boolean).join(", ")}`;
}
