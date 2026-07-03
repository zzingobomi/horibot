/**
 * CalibrationPanel — dockview 등록 패널 (calibration). RobotCalibrateMode 의 코어.
 *
 * backend_v2 calibration 계약 (capture-only flow) wire:
 *   - preview 5Hz stream (Topic.CALIBRATION_PREVIEW) → 실시간 traffic light overlay
 *   - start_run → capture(반복) → finalize_run 세션
 *   - snapshot_bundle → 현재 active 5 kind
 *   - list_runs / list_results + activate_result (rollback)
 *
 * 옛 frontend/ 의 CalibrationPanel 은 옛(online-BA) 계약이라 기계적 복사 X — 새
 * capture-only 계약에 맞춰 rewrite (frontend_v2.md carry-over 원칙).
 */
import { useCallback, useEffect, useState } from "react";
import { useParams } from "react-router-dom";
import { Button } from "@/components/ui/button";
import { DEFAULT_ROBOT_ID } from "@/constants";
import { useService, useStream } from "@/framework";
import { ServiceKey, Topic } from "@/api/generated/contract";
import type {
  CalibrationBundle,
  CalibrationRunRecord,
} from "@/api/generated/contract";

const VERDICT_COLOR: Record<string, string> = {
  green: "bg-green-500",
  yellow: "bg-yellow-500",
  red: "bg-red-500",
};

export function CalibrationPanel() {
  const { id } = useParams<{ id: string }>();
  const robotId = id ?? DEFAULT_ROBOT_ID;

  const snapshot = useService(ServiceKey.CALIBRATION_SNAPSHOT_BUNDLE, robotId);
  const startRun = useService(ServiceKey.CALIBRATION_START_RUN, robotId);
  const capture = useService(ServiceKey.CALIBRATION_CAPTURE, robotId);
  const finalize = useService(ServiceKey.CALIBRATION_FINALIZE_RUN, robotId);
  const undo = useService(ServiceKey.CALIBRATION_UNDO_LAST_CAPTURE, robotId);
  const previewEnable = useService(ServiceKey.CALIBRATION_PREVIEW_ENABLE, robotId);
  const listRuns = useService(ServiceKey.CALIBRATION_LIST_RUNS, robotId);
  const activate = useService(ServiceKey.CALIBRATION_ACTIVATE_RESULT, robotId);

  const preview = useStream(Topic.CALIBRATION_PREVIEW, { robotId, staleMs: 1000 });

  const [runId, setRunId] = useState<number | null>(null);
  const [poseIndex, setPoseIndex] = useState(0);
  const [previewOn, setPreviewOn] = useState(false);
  const [lastMsg, setLastMsg] = useState("");

  // calibration 은 robot-agnostic — 대상 robot 은 req 필드 (키 치환 아님).
  // run 진행 서비스 (capture/finalize/undo/activate) 는 run_id/result_id 에서 파생.
  const refresh = useCallback(() => {
    void snapshot.call({ robot_id: robotId });
    void listRuns.call({ robot_id: robotId });
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [robotId]);

  useEffect(() => {
    refresh();
  }, [refresh]);

  const onStart = async () => {
    const res = await startRun.call({
      robot_id: robotId,
      kind: "hand_eye",
      algorithm: "hand_eye_capture_only",
    });
    const rid = (res.data as { run_id?: number } | null)?.run_id ?? null;
    setRunId(rid);
    setPoseIndex(0);
    setLastMsg(rid ? `run ${rid} 시작` : `실패: ${res.message}`);
    refresh();  // list_runs 갱신 → history 에 새 run 반영
  };

  const onCapture = async () => {
    if (runId == null) return;
    const res = await capture.call({ run_id: runId, pose_index: poseIndex });
    const d = res.data as {
      accepted?: boolean;
      reproj_rms_px?: number | null;
      quality?: { verdict?: string } | null;
    } | null;
    if (d?.accepted) {
      setPoseIndex((p) => p + 1);
      setLastMsg(`캡처됨 (pose ${poseIndex}, RMS ${d.reproj_rms_px?.toFixed(2) ?? "?"}px)`);
    } else {
      setLastMsg(`거부: ${res.message || d?.quality?.verdict || "?"}`);
    }
  };

  const onFinalize = async () => {
    if (runId == null) return;
    const res = await finalize.call({ run_id: runId });
    setLastMsg(res.success ? `run ${runId} 종료 (분석 대기)` : `실패: ${res.message}`);
    setRunId(null);
    refresh();
  };

  const onUndo = async () => {
    if (runId == null) return;
    await undo.call({ run_id: runId });
    setPoseIndex((p) => Math.max(0, p - 1));
    setLastMsg("마지막 캡처 취소");
  };

  const onTogglePreview = async () => {
    const next = !previewOn;
    await previewEnable.call({ robot_id: robotId, enabled: next });
    setPreviewOn(next);
  };

  const bundle = snapshot.data as CalibrationBundle | null;
  const runs = (listRuns.data as { runs?: CalibrationRunRecord[] } | null)?.runs ?? [];
  const pv = preview.value;
  const verdict = pv?.verdict ?? "red";

  return (
    <div className="h-full overflow-y-auto p-3 text-[12px]" data-testid="calibration-panel">
      {/* preview traffic light */}
      <section className="mb-3">
        <div className="mb-1 flex items-center justify-between">
          <span className="font-mono uppercase text-muted-foreground">preview</span>
          <Button size="sm" variant={previewOn ? "default" : "outline"} onClick={onTogglePreview} data-testid="preview-toggle">
            {previewOn ? "ON" : "OFF"}
          </Button>
        </div>
        <div className="flex items-center gap-2 rounded border p-2">
          <span
            className={`inline-block h-3 w-3 rounded-full ${VERDICT_COLOR[verdict] ?? "bg-gray-400"}`}
            data-testid="preview-verdict"
            data-verdict={verdict}
          />
          <span data-testid="preview-detail">
            {pv?.detected ? `검출 ${pv.corner_count} corners` : "미검출"}
            {pv?.tilt_deg != null ? ` · tilt ${pv.tilt_deg.toFixed(0)}°` : ""}
          </span>
          {pv?.reasons?.length ? (
            <span className="text-muted-foreground">— {pv.reasons.join(", ")}</span>
          ) : null}
        </div>
      </section>

      {/* capture session */}
      <section className="mb-3">
        <div className="mb-1 font-mono uppercase text-muted-foreground">capture</div>
        <div className="flex flex-wrap gap-2">
          <Button size="sm" onClick={onStart} data-testid="start-run" disabled={runId != null}>
            세션 시작
          </Button>
          <Button size="sm" onClick={onCapture} data-testid="capture" disabled={runId == null}>
            캡처 ({poseIndex})
          </Button>
          <Button size="sm" variant="outline" onClick={onUndo} data-testid="undo" disabled={runId == null || poseIndex === 0}>
            취소
          </Button>
          <Button size="sm" variant="secondary" onClick={onFinalize} data-testid="finalize" disabled={runId == null}>
            세션 종료
          </Button>
        </div>
        <div className="mt-1 text-muted-foreground" data-testid="capture-msg">
          {lastMsg}
        </div>
      </section>

      {/* active bundle */}
      <section className="mb-3">
        <div className="mb-1 flex items-center justify-between">
          <span className="font-mono uppercase text-muted-foreground">active bundle</span>
          <Button size="sm" variant="ghost" onClick={refresh} data-testid="refresh-bundle">
            ↻
          </Button>
        </div>
        <div className="grid grid-cols-2 gap-1" data-testid="active-bundle">
          {(["intrinsic", "hand_eye", "joint_offset", "link_offset", "sag"] as const).map((k) => {
            const rec = bundle?.[k];
            return (
              <div key={k} className="flex items-center gap-1">
                <span className={`inline-block h-2 w-2 rounded-full ${rec ? "bg-green-500" : "bg-gray-400"}`} />
                <span className="font-mono">{k}</span>
                {k === "hand_eye" && bundle?.hand_eye?.effective_sigma_rot != null ? (
                  <span className="text-muted-foreground">
                    σ{bundle.hand_eye.effective_sigma_rot.toFixed(2)}°
                  </span>
                ) : null}
              </div>
            );
          })}
        </div>
      </section>

      {/* history / rollback */}
      <section>
        <div className="mb-1 font-mono uppercase text-muted-foreground">runs</div>
        <div className="flex flex-col gap-1" data-testid="run-history">
          {runs.length === 0 ? (
            <span className="text-muted-foreground">없음</span>
          ) : (
            runs.map((r) => (
              <div key={r.id} className="flex items-center justify-between rounded border px-2 py-1">
                <span className="font-mono">
                  #{r.id} {r.kind} · {r.status}
                </span>
              </div>
            ))
          )}
        </div>
        {activate.pending ? <span className="text-muted-foreground">activating…</span> : null}
      </section>
    </div>
  );
}
