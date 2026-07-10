/**
 * CalibrationPanel — dockview 등록 패널 (calibration). RobotCalibrateMode 의 코어.
 *
 * backend calibration 계약 (capture-only flow) wire:
 *   - preview 5Hz stream (Topic.CALIBRATION_PREVIEW) → 실시간 traffic light overlay
 *   - start_run(kind) → capture(반복) → finalize_run 세션 — intrinsic/hand_eye
 *     공용 (도메인 = "세션은 한 번에 하나, kind 는 파라미터". 패널 분리 X — preview/
 *     capture/runs 가 90% 공유 + robot-level preview_enable 토글 충돌 방지)
 *   - snapshot_bundle → 현재 active 5 kind + 행 클릭 시 실제 수치 expand
 *   - list_runs / list_results + activate_result (rollback)
 *
 * 세션 UI 는 상태 기반 노출: idle = 시작 버튼 2개만, 세션 중 = kind 배지 +
 * 캡처/취소/종료만 (인지 부하 축소 — 항상 4버튼 나열 X).
 *
 * finalize 분기 주의: intrinsic finalize 는 backend 가 compute 까지 수행하고
 * 캡처 부족이면 ok=false 로 run 을 살려둠 — 이때 세션(runId)을 버리면 진행 중
 * run 을 UI 가 잃어버린다. ok=true 일 때만 세션 종료.
 */
import { useCallback, useEffect, useState } from "react";
import { Button } from "@/components/ui/button";
import { useRobotId } from "@/hooks/useRobotId";
import { useService, useStream } from "@/framework";
import { ServiceKey, Topic } from "@/api/generated/contract";
import type {
  CalibrationBundle,
  CalibrationRunRecord,
} from "@/api/generated/contract";
import { useCameraStore } from "@/stores/cameraStore";

const VERDICT_COLOR: Record<string, string> = {
  green: "bg-green-500",
  yellow: "bg-yellow-500",
  red: "bg-red-500",
};

type SessionKind = "intrinsic" | "hand_eye";

const SESSION_LABEL: Record<SessionKind, string> = {
  intrinsic: "내부캘",
  hand_eye: "핸드아이",
};

// algorithm 은 run 기록용 자유 텍스트 — factory seed("d405_factory")와 구분되는
// 사용자 ChArUco 캡처 표기.
const SESSION_ALGORITHM: Record<SessionKind, string> = {
  intrinsic: "charuco_manual",
  hand_eye: "hand_eye_capture_only",
};

type BundleKind = keyof Pick<
  CalibrationBundle,
  "intrinsic" | "hand_eye" | "joint_offset" | "link_offset" | "sag"
>;

const BUNDLE_KINDS: BundleKind[] = [
  "intrinsic",
  "hand_eye",
  "joint_offset",
  "link_offset",
  "sag",
];

const RAD2DEG = 180 / Math.PI;
const f1 = (n: number) => n.toFixed(1);
const f2 = (n: number) => n.toFixed(2);

/** kind 별 실제 캘 수치 → [label, value] 행 (C — 초록불이 아니라 값을 보여줌). */
function detailRows(
  kind: BundleKind,
  bundle: CalibrationBundle,
): [string, string][] {
  const rec = bundle[kind];
  if (!rec) return [];
  const rows: [string, string][] = [];

  if (kind === "intrinsic" && bundle.intrinsic) {
    const d = bundle.intrinsic.result_data;
    const cm = d.camera_matrix;
    rows.push(["fx / fy", `${f1(cm[0][0])} / ${f1(cm[1][1])} px`]);
    rows.push(["cx / cy", `${f1(cm[0][2])} / ${f1(cm[1][2])} px`]);
    rows.push([
      "해상도",
      d.image_size ? `${d.image_size[0]}×${d.image_size[1]}` : "—",
    ]);
    const k = d.dist_coeffs?.[0];
    rows.push([
      "왜곡 k1,k2",
      k && k.length >= 2 ? `${k[0].toFixed(4)}, ${k[1].toFixed(4)}` : "—",
    ]);
    rows.push(["RMS", d.rms_px != null ? `${f2(d.rms_px)} px` : "— (factory)"]);
  } else if (kind === "hand_eye" && bundle.hand_eye) {
    const d = bundle.hand_eye.result_data;
    const t = d.t_cam2gripper;
    const tv = [t[0][0], t[1][0], t[2][0]];
    const R = d.R_cam2gripper;
    const trace = R[0][0] + R[1][1] + R[2][2];
    const angle =
      Math.acos(Math.min(1, Math.max(-1, (trace - 1) / 2))) * RAD2DEG;
    rows.push(["cam↔EE", `[${tv.map((v) => f1(v * 1000)).join(", ")}] mm`]);
    rows.push([
      "|t| / 회전",
      `${f1(Math.hypot(tv[0], tv[1], tv[2]) * 1000)}mm / ${f1(angle)}°`,
    ]);
    const rec2 = bundle.hand_eye;
    rows.push([
      "σ_eff",
      rec2.effective_sigma_rot != null && rec2.effective_sigma_t != null
        ? `${f2(rec2.effective_sigma_rot)}° / ${f2(rec2.effective_sigma_t)}mm`
        : "—",
    ]);
    rows.push([
      "σ_jac",
      rec2.sigma_rot != null && rec2.sigma_t != null
        ? `${f2(rec2.sigma_rot)}° / ${f2(rec2.sigma_t)}mm`
        : "—",
    ]);
    rows.push(["method", d.method]);
  } else if (kind === "joint_offset" && bundle.joint_offset) {
    const d = bundle.joint_offset.result_data;
    for (const [id, rad] of Object.entries(d.offsets)) {
      rows.push([`J${id}`, `${(rad * RAD2DEG).toFixed(2)}°`]);
    }
    rows.push(["method", d.method]);
  } else if (kind === "link_offset" && bundle.link_offset) {
    const d = bundle.link_offset.result_data;
    for (const e of d.offsets) {
      const dmm = Math.hypot(e.trans_m[0], e.trans_m[1], e.trans_m[2]) * 1000;
      const ddeg =
        Math.hypot(e.rot_rad[0], e.rot_rad[1], e.rot_rad[2]) * RAD2DEG;
      rows.push([`J${e.joint_id}`, `Δ${f1(dmm)}mm / ${f2(ddeg)}°`]);
    }
    rows.push(["method", d.method]);
  } else if (kind === "sag" && bundle.sag) {
    const d = bundle.sag.result_data;
    for (const [id, k] of Object.entries(d.k_rad_per_m)) {
      rows.push([`J${id}`, `k=${k.toExponential(2)}`]);
    }
    rows.push(["method", d.method]);
  }

  rows.push(["run", `#${rec.run_id}`]);
  rows.push(["일시", String(rec.created_at).slice(0, 10)]);
  return rows;
}

export function CalibrationPanel() {
  const robotId = useRobotId();

  const snapshot = useService(ServiceKey.CALIBRATION_SNAPSHOT_BUNDLE, robotId);
  const startRun = useService(ServiceKey.CALIBRATION_START_RUN, robotId);
  const capture = useService(ServiceKey.CALIBRATION_CAPTURE, robotId);
  const finalize = useService(ServiceKey.CALIBRATION_FINALIZE_RUN, robotId);
  const abort = useService(ServiceKey.CALIBRATION_ABORT_RUN, robotId);
  const undo = useService(ServiceKey.CALIBRATION_UNDO_LAST_CAPTURE, robotId);
  const previewEnable = useService(ServiceKey.CALIBRATION_PREVIEW_ENABLE, robotId);
  const listRuns = useService(ServiceKey.CALIBRATION_LIST_RUNS, robotId);
  const activate = useService(ServiceKey.CALIBRATION_ACTIVATE_RESULT, robotId);

  const preview = useStream(Topic.CALIBRATION_PREVIEW, { robotId, staleMs: 1000 });
  // per-robot frustum — 이 패널의 [시야] 는 자기 robot 카메라만 (robot ownership)
  const showFrustum = useCameraStore((s) => !!s.frustum[robotId]);
  const setFrustum = useCameraStore((s) => s.setFrustum);

  const [runId, setRunId] = useState<number | null>(null);
  const [runKind, setRunKind] = useState<SessionKind | null>(null);
  const [poseIndex, setPoseIndex] = useState(0);
  const [previewOn, setPreviewOn] = useState(false);
  const [lastMsg, setLastMsg] = useState("");
  const [openDetail, setOpenDetail] = useState<BundleKind | null>(null);

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

  const onStart = async (kind: SessionKind) => {
    const res = await startRun.call({
      robot_id: robotId,
      kind,
      algorithm: SESSION_ALGORITHM[kind],
    });
    const rid = (res.data as { run_id?: number } | null)?.run_id ?? null;
    setRunId(rid);
    setRunKind(rid != null ? kind : null);
    setPoseIndex(0);
    setLastMsg(
      rid != null ? `${SESSION_LABEL[kind]} run ${rid} 시작` : `실패: ${res.message}`,
    );
    refresh(); // list_runs 갱신 → history 에 새 run 반영
  };

  const onCapture = async () => {
    if (runId == null) return;
    const res = await capture.call({ run_id: runId, pose_index: poseIndex });
    const d = res.data as {
      accepted?: boolean;
      reproj_rms_px?: number | null;
      message?: string;
      quality?: { verdict?: string } | null;
    } | null;
    if (d?.accepted) {
      setPoseIndex((p) => p + 1);
      // intrinsic 세션은 detect-only 라 RMS 없음 — 있을 때만 표시.
      const rms =
        d.reproj_rms_px != null ? `, RMS ${d.reproj_rms_px.toFixed(2)}px` : "";
      setLastMsg(`캡처됨 (#${poseIndex}${rms})`);
    } else {
      setLastMsg(`거부: ${d?.message || d?.quality?.verdict || res.message || "?"}`);
    }
  };

  const onFinalize = async () => {
    if (runId == null) return;
    const res = await finalize.call({ run_id: runId });
    const d = res.data as { ok?: boolean; message?: string } | null;
    if (!res.success) {
      setLastMsg(`실패: ${res.message}`);
      return;
    }
    if (!d?.ok) {
      // intrinsic 캡처 부족 등 — backend 가 run 을 살려둠. 세션 유지 (더 캡처).
      setLastMsg(d?.message || "마감 불가 — 더 캡처 후 재시도");
      return;
    }
    setLastMsg(
      runKind === "intrinsic"
        ? `run ${runId} 종료 — intrinsic 계산·활성 완료`
        : `run ${runId} 종료 (offline 분석 대기)`,
    );
    setRunId(null);
    setRunKind(null);
    refresh();
  };

  const onUndo = async () => {
    if (runId == null) return;
    await undo.call({ run_id: runId });
    setPoseIndex((p) => Math.max(0, p - 1));
    setLastMsg("마지막 캡처 취소");
  };

  const onAbort = async () => {
    if (runId == null) return;
    // 세션 탈출구 — finalize(캡처부족 거부)와 별개. run → failed, 캡처 row 보존.
    // 탈출은 서비스 실패에 막히지 않는다 — 로컬 세션은 무조건 닫고, 백엔드에
    // 남은 run 은 start_run 의 stale 자동 정리가 안전망 (사유 + 다음 행동 안내).
    const res = await abort.call({ run_id: runId });
    setLastMsg(
      res.success
        ? `run ${runId} 중단됨`
        : `중단 요청 실패 (${res.message}) — 세션은 닫음, 잔여 run 은 다음 세션 시작 시 자동 정리`,
    );
    setRunId(null);
    setRunKind(null);
    refresh();
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
          <div className="flex items-center gap-1.5">
            {/* 카메라 시야(frustum) — 렌더는 Camera 씬 객체, 여기는 토글만 (소유권 모델) */}
            <Button
              size="sm"
              variant={showFrustum ? "default" : "outline"}
              onClick={() => setFrustum(robotId, !showFrustum)}
              data-testid="frustum-toggle"
            >
              시야
            </Button>
            <Button size="sm" variant={previewOn ? "default" : "outline"} onClick={onTogglePreview} data-testid="preview-toggle">
              {previewOn ? "ON" : "OFF"}
            </Button>
          </div>
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

      {/* capture session — 상태 기반 노출 (idle=시작 2개 / 세션 중=진행 3개) */}
      <section className="mb-3">
        <div className="mb-1 flex items-center justify-between">
          <span className="font-mono uppercase text-muted-foreground">capture</span>
          {runId != null && runKind != null ? (
            <span className="font-mono text-muted-foreground" data-testid="session-badge">
              {SESSION_LABEL[runKind]} run #{runId}
            </span>
          ) : null}
        </div>
        <div className="flex flex-wrap gap-2">
          {runId == null ? (
            <>
              <Button size="sm" onClick={() => void onStart("intrinsic")} data-testid="start-intrinsic">
                내부캘 시작
              </Button>
              <Button size="sm" onClick={() => void onStart("hand_eye")} data-testid="start-run">
                핸드아이 시작
              </Button>
            </>
          ) : (
            <>
              <Button size="sm" onClick={onCapture} data-testid="capture">
                캡처 ({poseIndex})
              </Button>
              <Button size="sm" variant="outline" onClick={onUndo} data-testid="undo" disabled={poseIndex === 0}>
                한 장 취소
              </Button>
              <Button size="sm" variant="secondary" onClick={onFinalize} data-testid="finalize">
                세션 종료
              </Button>
              <Button
                size="sm"
                variant="ghost"
                className="text-destructive"
                onClick={onAbort}
                data-testid="abort"
              >
                중단
              </Button>
            </>
          )}
        </div>
        <div className="mt-1 text-muted-foreground" data-testid="capture-msg">
          {lastMsg}
        </div>
      </section>

      {/* active bundle — 행 클릭 시 실제 수치 expand */}
      <section className="mb-3">
        <div className="mb-1 flex items-center justify-between">
          <span className="font-mono uppercase text-muted-foreground">active bundle</span>
          <Button size="sm" variant="ghost" onClick={refresh} data-testid="refresh-bundle">
            ↻
          </Button>
        </div>
        <div className="flex flex-col" data-testid="active-bundle">
          {BUNDLE_KINDS.map((k) => {
            const rec = bundle?.[k];
            const open = openDetail === k;
            return (
              <div key={k}>
                <button
                  type="button"
                  className="flex w-full items-center gap-1.5 rounded px-1 py-0.5 text-left hover:bg-muted/40"
                  onClick={() => {
                    if (!rec) return;
                    setOpenDetail(open ? null : k);
                  }}
                  data-testid={`bundle-row-${k}`}
                >
                  <span className={`inline-block h-2 w-2 rounded-full ${rec ? "bg-green-500" : "bg-gray-400"}`} />
                  <span className="font-mono">{k}</span>
                  {k === "hand_eye" && bundle?.hand_eye?.effective_sigma_rot != null ? (
                    <span className="text-muted-foreground">
                      σ{bundle.hand_eye.effective_sigma_rot.toFixed(2)}°
                    </span>
                  ) : null}
                  <span className="ml-auto text-muted-foreground">
                    {rec ? (open ? "▾" : "▸") : ""}
                  </span>
                </button>
                {open && bundle && rec ? (
                  <div
                    className="mb-1 ml-3.5 rounded border bg-muted/20 px-2 py-1 font-mono"
                    data-testid={`bundle-detail-${k}`}
                  >
                    {detailRows(k, bundle).map(([label, val]) => (
                      <div key={label} className="flex justify-between gap-2">
                        <span className="text-muted-foreground">{label}</span>
                        <span>{val}</span>
                      </div>
                    ))}
                  </div>
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
