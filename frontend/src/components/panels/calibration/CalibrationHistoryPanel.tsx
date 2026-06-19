/**
 * Calibration history — Run 단위 통합 list + 펼침 + ACTIVATE.
 *
 * storage_layer.md §13.7 Stage 4 design A:
 *   - 한 row = 한 Run (= 한 캘 세션). MLflow Model Registry / git history 정합.
 *   - 펼침 → 5 kind result 펼침 → kind 별 ACTIVATE 버튼.
 *   - 활성 (is_active=true) row 강조.
 *   - ACTIVATE 후 backend invalidation publish → useCalibrationRuns 가 자동 refetch.
 *
 * 사용처:
 *   - σ 후퇴 비교 ("지난 주 σ vs 오늘")
 *   - 외부 스크립트로 disk 망친 후 safe baseline 으로 되돌리기
 *   - COMMIT 후 결과 보고 ACTIVATE 결정
 *
 * 옛 RollbackPanel (`.history/<ts>_pre-commit/` 의존) 폐기. storage_node 가 SSOT.
 */
import { useCallback, useMemo, useState } from "react";
import { History, ChevronDown, ChevronRight } from "lucide-react";
import type { IDockviewPanelProps } from "dockview";
import { useParams } from "react-router-dom";
import { PanelShell } from "@/components/shared/PanelShell";
import { PanelButton } from "@/components/shared/PanelButton";
import { Section } from "@/components/shared/Section";
import {
  useCalibrationRuns,
  type CalibrationRunSummary,
  type CalibrationResultRecord,
} from "@/hooks/useCalibrationRuns";

export function CalibrationHistoryPanel(props: IDockviewPanelProps<object>) {
  const { id: robotId = "" } = useParams<{ id: string }>();
  const { runs, loading, error, refetch, activate } = useCalibrationRuns(robotId);
  const [expandedRunIds, setExpandedRunIds] = useState<Set<number>>(new Set());
  const [activatingId, setActivatingId] = useState<number | null>(null);
  const [status, setStatus] = useState<string>("");

  const toggleExpand = useCallback((runId: number | null | undefined) => {
    if (runId === null || runId === undefined) return;
    setExpandedRunIds((prev) => {
      const next = new Set(prev);
      if (next.has(runId)) next.delete(runId);
      else next.add(runId);
      return next;
    });
  }, []);

  const handleActivate = useCallback(
    async (result: CalibrationResultRecord) => {
      if (result.id === null || result.id === undefined) return;
      if (result.is_active) return;
      if (
        !confirm(
          `${result.kind} (id=${result.id}, ${formatTs(result.created_at)}) 를 활성화합니다.\n계속할까요?`,
        )
      ) {
        return;
      }
      setActivatingId(result.id);
      const res = await activate(result.id);
      setActivatingId(null);
      setStatus(res.success ? `✅ ${res.message || "활성화 완료"}` : `❌ ${res.message}`);
    },
    [activate],
  );

  return (
    <PanelShell
      icon={<History className="w-3.5 h-3.5" />}
      title="Calibration History"
      panelId={props.api.id}
      api={props.api}
      expandedHeight={400}
    >
      <Section label="안내">
        <div className="flex items-start justify-between gap-2">
          <p className="text-[11px] text-zinc-500 leading-snug font-mono">
            한 row = 한 캘 세션 (Run). 클릭으로 펼침 → kind 별 ACTIVATE.
            <br />
            현재 활성 result 는 강조됨.
          </p>
          <PanelButton
            variant="ghost"
            className="shrink-0 !px-2 !py-0.5 !text-[10px]"
            onClick={() => void refetch()}
            disabled={loading}
          >
            {loading ? "..." : "새로고침"}
          </PanelButton>
        </div>
      </Section>

      <Section label="Runs">
        <div className="rounded border border-zinc-800/60 bg-black/20 overflow-hidden">
          {error && (
            <p className="text-[11px] text-red-400 p-3 font-mono">{error}</p>
          )}
          {!error && runs.length === 0 ? (
            <p className="text-[11px] text-zinc-500 p-3 font-mono">
              아직 COMMIT 된 캘 없음. 캘 1회 + COMMIT 후 row 가 쌓입니다.
            </p>
          ) : (
            <div className="max-h-80 overflow-y-auto">
              {runs.map((run) => (
                <RunRow
                  key={run.run.id ?? -1}
                  summary={run}
                  expanded={
                    run.run.id !== null &&
                    run.run.id !== undefined &&
                    expandedRunIds.has(run.run.id)
                  }
                  onToggle={() => toggleExpand(run.run.id)}
                  onActivate={handleActivate}
                  activatingId={activatingId}
                />
              ))}
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

interface RunRowProps {
  summary: CalibrationRunSummary;
  expanded: boolean;
  onToggle: () => void;
  onActivate: (result: CalibrationResultRecord) => void;
  activatingId: number | null;
}

function RunRow({
  summary,
  expanded,
  onToggle,
  onActivate,
  activatingId,
}: RunRowProps) {
  const { run, results } = summary;
  const sigmaSummary = useMemo(() => bestSigma(results), [results]);
  const activeKinds = results.filter((r) => r.is_active).length;
  const hasAnyActive = activeKinds > 0;

  return (
    <div className="border-b border-zinc-800/40 last:border-b-0">
      <button
        type="button"
        onClick={onToggle}
        className={
          "w-full flex items-center gap-2 px-2 py-1.5 text-left text-[11px] font-mono " +
          (hasAnyActive ? "bg-emerald-950/20 text-emerald-300" : "text-zinc-300") +
          " hover:bg-zinc-800/40"
        }
      >
        {expanded ? (
          <ChevronDown className="w-3 h-3 shrink-0 text-zinc-500" />
        ) : (
          <ChevronRight className="w-3 h-3 shrink-0 text-zinc-500" />
        )}
        <span className="shrink-0">{formatTs(run.started_at)}</span>
        <span className="shrink-0 text-zinc-500 truncate">{run.algorithm}</span>
        <span className="ml-auto shrink-0 text-zinc-500">
          {results.length} kind{results.length !== 1 ? "s" : ""}
        </span>
        {sigmaSummary && (
          <span className="shrink-0 text-zinc-400">{sigmaSummary}</span>
        )}
        {hasAnyActive && (
          <span className="shrink-0 text-emerald-400 text-[10px]">
            ● {activeKinds}/{results.length}
          </span>
        )}
      </button>
      {expanded && (
        <div className="bg-black/30">
          <table className="w-full text-[11px] font-mono">
            <thead className="text-left text-zinc-500">
              <tr>
                <th className="px-2 py-1 font-normal pl-7">kind</th>
                <th className="px-2 py-1 font-normal text-right">σ_rot</th>
                <th className="px-2 py-1 font-normal text-right">σ_t</th>
                <th className="px-2 py-1 font-normal" />
              </tr>
            </thead>
            <tbody>
              {results.map((r) => (
                <KindResultRow
                  key={r.id ?? -1}
                  result={r}
                  onActivate={onActivate}
                  activating={activatingId === r.id}
                />
              ))}
            </tbody>
          </table>
        </div>
      )}
    </div>
  );
}

interface KindResultRowProps {
  result: CalibrationResultRecord;
  onActivate: (result: CalibrationResultRecord) => void;
  activating: boolean;
}

function KindResultRow({ result, onActivate, activating }: KindResultRowProps) {
  return (
    <tr
      className={
        "border-t border-zinc-800/40 " +
        (result.is_active ? "bg-emerald-950/20 text-emerald-300" : "text-zinc-300")
      }
    >
      <td className="px-2 py-1 pl-7">
        {result.is_active && (
          <span className="text-emerald-400 mr-1">●</span>
        )}
        {result.kind}
      </td>
      <td className="px-2 py-1 text-right">{formatSigma(result.sigma_rot, "deg")}</td>
      <td className="px-2 py-1 text-right">{formatSigma(result.sigma_t, "mm")}</td>
      <td className="px-2 py-1 text-right">
        {result.is_active ? (
          <span className="text-emerald-400 text-[10px]">활성</span>
        ) : (
          <PanelButton
            variant="outline"
            className="!px-2 !py-0.5 !text-[10px]"
            onClick={() => onActivate(result)}
            disabled={activating}
          >
            {activating ? "..." : "ACTIVATE"}
          </PanelButton>
        )}
      </td>
    </tr>
  );
}

function formatTs(epoch: number): string {
  const d = new Date(epoch * 1000);
  const pad = (n: number) => String(n).padStart(2, "0");
  return (
    `${d.getFullYear()}-${pad(d.getMonth() + 1)}-${pad(d.getDate())} ` +
    `${pad(d.getHours())}:${pad(d.getMinutes())}:${pad(d.getSeconds())}`
  );
}

function formatSigma(
  v: number | null | undefined,
  unit: "deg" | "mm",
): string {
  if (v === null || v === undefined) return "—";
  // backend 가 이미 deg / mm 단위로 DB 에 저장 (CalibrationResultRecord.sigma_rot/sigma_t).
  // calibration_node.py 의 finalize_run 자리 sigma_rot_deg / sigma_t_mm 그대로 넣음.
  if (unit === "deg") return `${v.toFixed(2)}°`;
  return `${v.toFixed(1)}mm`;
}

function bestSigma(results: CalibrationResultRecord[]): string | null {
  const he = results.find((r) => r.kind === "hand_eye");
  if (!he || he.sigma_rot === null || he.sigma_rot === undefined) return null;
  const rotDeg = he.sigma_rot.toFixed(2);
  const tMm =
    he.sigma_t !== null && he.sigma_t !== undefined
      ? `/${he.sigma_t.toFixed(1)}mm`
      : "";
  return `σ ${rotDeg}°${tMm}`;
}
