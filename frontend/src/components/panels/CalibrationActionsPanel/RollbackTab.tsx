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
import { useParams } from "react-router-dom";
import { bridge } from "@/api/bridge";
import { Button } from "@/components/ui/button";
import { ServiceKey } from "@/constants/topics";
import type { BackupEntry } from "./types";

export function RollbackTab() {
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
      setStatus(`❌ ${res.message}`);
    }
  }, []);

  // 마운트 시 fetch — refresh() 와 분리. effect body 내 직접 setState 호출 피하려고
  // .then 콜백으로 라우팅 (react-hooks/set-state-in-effect).
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
    const res = await bridge.callService(
      ServiceKey.CALIB_BACKUP_RESTORE,
      { timestamp: entry.timestamp },
    );
    setRestoring(null);
    if (res.success) {
      const data = res.data as unknown as {
        restored_timestamp: string;
        restart_required: boolean;
      };
      setStatus(
        `✅ ${data.restored_timestamp} 복원 완료` +
          (data.restart_required ? " — 백엔드 재시작 필요" : ""),
      );
      await refresh();
    } else {
      setStatus(`❌ ${res.message}`);
    }
  };

  return (
    <div className="flex flex-col gap-3 min-h-0">
      <div className="flex items-center justify-between px-1">
        <p className="text-xs text-muted-foreground">
          매 COMMIT 직전 disk 가 자동 백업됩니다. σ 비교 / 후퇴 대비.
        </p>
        <Button
          size="sm"
          variant="ghost"
          className="h-6 px-2 text-[10px]"
          onClick={() => void refresh()}
          disabled={loading}
        >
          {loading ? "..." : "새로고침"}
        </Button>
      </div>

      <div className="flex-1 min-h-0 overflow-y-auto rounded border bg-card">
        {entries.length === 0 ? (
          <p className="text-xs text-muted-foreground p-3">
            저장된 snapshot 없음. 한 번 COMMIT 하면 자동으로 쌓입니다.
          </p>
        ) : (
          <table className="w-full text-[11px] font-mono">
            <thead className="sticky top-0 bg-muted/60 backdrop-blur">
              <tr className="text-left text-muted-foreground">
                <th className="px-2 py-1.5 font-normal">timestamp</th>
                <th className="px-2 py-1.5 font-normal">tag</th>
                <th className="px-2 py-1.5 font-normal text-right">σ_rot</th>
                <th className="px-2 py-1.5 font-normal text-right">σ_t</th>
                <th className="px-2 py-1.5 font-normal text-right">caps</th>
                <th className="px-2 py-1.5" />
              </tr>
            </thead>
            <tbody>
              {entries.map((e) => (
                <tr key={e.timestamp} className="border-t border-border/30">
                  <td className="px-2 py-1.5">{formatTs(e.timestamp)}</td>
                  <td className="px-2 py-1.5 text-muted-foreground">
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
                  <td className="px-2 py-1.5 text-right text-muted-foreground">
                    {e.capture_count ?? "—"}
                  </td>
                  <td className="px-2 py-1.5 text-right">
                    <Button
                      size="sm"
                      variant="outline"
                      className="h-6 px-2 text-[10px]"
                      onClick={() => handleRestore(e)}
                      disabled={restoring !== null}
                    >
                      {restoring === e.timestamp ? "복원중..." : "복원"}
                    </Button>
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
        )}
      </div>

      {status && (
        <p className="text-[11px] text-muted-foreground px-1">{status}</p>
      )}
    </div>
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
  const r = e.sigma_rot_deg !== null ? `σ_rot=${e.sigma_rot_deg.toFixed(2)}°` : "";
  const t = e.sigma_t_mm !== null ? `σ_t=${e.sigma_t_mm.toFixed(1)}mm` : "";
  return `, ${[r, t].filter(Boolean).join(", ")}`;
}
