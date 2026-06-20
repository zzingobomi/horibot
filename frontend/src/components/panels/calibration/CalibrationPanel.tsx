import type { IDockviewPanelProps } from "dockview";
import {
  Camera,
  CheckCircle2,
  Loader2,
  PlayCircle,
  RefreshCw,
  Undo2,
} from "lucide-react";
import { useParams } from "react-router-dom";

import { PanelShell } from "@/components/shared/PanelShell";
import { Section } from "@/components/shared/Section";
import { StatusBadge } from "@/components/shared/StatusBadge";
import { Button } from "@/components/ui/button";
import { useCalibrationStore } from "@/domain/stores/calibration";
import { useCalibrationResults } from "@/hooks/useCalibrationResults";

/**
 * Calibration capture flow panel — capture-only 시나리오.
 *
 * 흐름: [캘 시작] → 토크오프 자유 자세 + traffic light → [캡처] N장 → [세션 종료]
 * (in_progress → ready_for_analysis). offline Python 스크립트가 BA + commit.
 */
export function CalibrationPanel(props: IDockviewPanelProps<object>) {
  const { id: robotId = "" } = useParams<{ id: string }>();
  const { results, loading: resultsLoading, error, refetch } =
    useCalibrationResults(robotId);

  const hand_eye_run_id = useCalibrationStore((s) => s.hand_eye_run_id);
  const poseCount = useCalibrationStore((s) => s.poses.length);
  const status = useCalibrationStore((s) => s.status);
  const loading = useCalibrationStore((s) => s.loading);
  const startSession = useCalibrationStore((s) => s.startSession);
  const capture = useCalibrationStore((s) => s.capture);
  const undoLastCapture = useCalibrationStore((s) => s.undoLastCapture);
  const reset = useCalibrationStore((s) => s.reset);
  const finalize = useCalibrationStore((s) => s.finalize);

  const sessionActive = hand_eye_run_id != null;

  return (
    <PanelShell
      icon={<Camera className="w-3.5 h-3.5" />}
      title="Calibration"
      panelId={props.api.id}
      api={props.api}
    >
      {/* 현재 활성 캘 — Intrinsic / Hand-Eye 적용 상태 */}
      <Section label="Active">
        <div className="flex items-start justify-between">
          <div className="space-y-1.5">
            <StatusBadge ok={!!results?.intrinsic} label="Intrinsic" />
            <StatusBadge ok={!!results?.hand_eye} label="Hand-Eye" />
            {error && (
              <p className="text-[10px] text-red-400 font-mono mt-1">⚠ {error}</p>
            )}
          </div>
          <button
            onClick={refetch}
            disabled={resultsLoading}
            className="p-1.5 rounded hover:bg-zinc-700/60 text-zinc-400 hover:text-zinc-100 transition-colors disabled:opacity-40"
            title="Reload active calibration"
          >
            <RefreshCw
              className={`w-3.5 h-3.5 ${resultsLoading ? "animate-spin" : ""}`}
            />
          </button>
        </div>
      </Section>

      {/* Capture flow */}
      <Section label="Capture Session">
        <div className="space-y-2">
          {!sessionActive ? (
            <Button
              size="sm"
              variant="default"
              className="w-full"
              disabled={loading || !results?.intrinsic}
              onClick={() => void startSession(robotId)}
              title={
                !results?.intrinsic
                  ? "Intrinsic 캘 먼저 필요"
                  : "새 hand-eye 세션 시작"
              }
            >
              <PlayCircle className="w-3.5 h-3.5 mr-1.5" />
              캘 시작
            </Button>
          ) : (
            <>
              <div className="text-[11px] font-mono text-zinc-300 px-1">
                run_id={hand_eye_run_id} · {poseCount}장 캡처
              </div>
              <Button
                size="sm"
                variant="default"
                className="w-full"
                disabled={loading}
                onClick={() => void capture(robotId)}
              >
                <Camera className="w-3.5 h-3.5 mr-1.5" />
                캡처
              </Button>
              <div className="flex gap-1.5">
                <Button
                  size="sm"
                  variant="outline"
                  className="flex-1"
                  disabled={loading || poseCount === 0}
                  onClick={() => void undoLastCapture(robotId)}
                >
                  <Undo2 className="w-3.5 h-3.5 mr-1.5" />
                  되돌리기
                </Button>
                <Button
                  size="sm"
                  variant="outline"
                  className="flex-1"
                  disabled={loading}
                  onClick={() => void reset(robotId)}
                >
                  리셋
                </Button>
              </div>
              <Button
                size="sm"
                variant="secondary"
                className="w-full"
                disabled={loading || poseCount < 3}
                onClick={() => void finalize(robotId)}
                title={
                  poseCount < 3
                    ? "최소 3장 이상 필요"
                    : "세션 종료 — offline 분석 대기 (ready_for_analysis)"
                }
              >
                <CheckCircle2 className="w-3.5 h-3.5 mr-1.5" />
                세션 종료
              </Button>
            </>
          )}
          {status && (
            <p className="text-[11px] font-mono text-zinc-400 px-1 break-words">
              {status}
            </p>
          )}
          {loading && (
            <div className="flex items-center gap-2 px-1 text-xs text-zinc-500 font-mono">
              <Loader2 className="w-3.5 h-3.5 animate-spin" /> 처리 중…
            </div>
          )}
        </div>
      </Section>

      <Section label="Offline 분석 안내">
        <p className="text-[11px] font-mono text-zinc-400 leading-relaxed">
          [세션 종료] 후엔 Python 스크립트로 captures + raw blob 읽어 BA → commit.
          frontend 자리는 capture 만 담당.
        </p>
      </Section>
    </PanelShell>
  );
}
