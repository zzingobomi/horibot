/**
 * TaskProgressPanel — task 진행 표시 + 디버거 (task 공통 부품).
 *
 * TRACE (step 진입 누적 — label/depth/status/detail, 중첩은 depth 들여쓰기) +
 * STATE (status/current_name/error/breakpoints) 로 진행을 그린다. 사전 step
 * 목록은 없음 — imperative 시나리오는 실행해 봐야 경로가 정해지므로, 표시는
 * "실행된 것의 누적" + 직전 run 의 trace 가 다음 run 의 breakpoint 대상 (name
 * 은 run 간 안정 — @step 함수 이름).
 *
 * 디버거: ● dot 클릭 = TOGGLE_BREAKPOINT(label) (없는 run 중에도 미리 박기 —
 * runner 가 run 간 보존) / RUNNING 에서 [일시정지] / PAUSED 에서 [재개]·[한 스텝]·
 * entry 별 [▶ 여기까지] (run-to-cursor). 실패 = error 박스에 사유 (backend 가
 * "다음 행동" 까지 담아 보냄 — 침묵 금지).
 */
import { useState } from "react";
import { Button } from "@/components/ui/button";
import { useService, useStream } from "@/framework";
import { useTaskRobots } from "@/hooks/useTaskRobots";
import {
  ServiceKey,
  TaskStatus,
  Topic,
  type TraceEntry,
} from "@/api/generated/contract";

const STATUS_COLOR: Record<string, string> = {
  [TaskStatus.RUNNING]: "text-sky-400",
  [TaskStatus.SUCCESS]: "text-emerald-400",
  [TaskStatus.FAILED]: "text-red-400",
  [TaskStatus.PAUSED]: "text-amber-400",
  [TaskStatus.STOPPED]: "text-zinc-400",
  [TaskStatus.IDLE]: "text-zinc-500",
};

const ENTRY_DOT: Record<string, string> = {
  running: "bg-sky-400",
  completed: "bg-emerald-400",
  failed: "bg-red-400",
};

export function TaskProgressPanel() {
  // task 참여 robot (계약 조회) — 로드 전 undefined = 스트림 미확장 (데이터 없음).
  const robotId = useTaskRobots(ServiceKey.PICKANDPLACE_LIST_ROBOTS)[0];
  const state = useStream(Topic.PICKANDPLACE_STATE, { robotId });
  const trace = useStream(Topic.PICKANDPLACE_TRACE, { robotId });

  const pauseSvc = useService(ServiceKey.PICKANDPLACE_PAUSE, robotId);
  const resumeSvc = useService(ServiceKey.PICKANDPLACE_RESUME, robotId);
  const stepOnceSvc = useService(ServiceKey.PICKANDPLACE_STEP_ONCE, robotId);
  const runToSvc = useService(ServiceKey.PICKANDPLACE_RUN_TO, robotId);
  const toggleBpSvc = useService(ServiceKey.PICKANDPLACE_TOGGLE_BREAKPOINT, robotId);
  const previewSvc = useService(ServiceKey.PICKANDPLACE_PREVIEW, robotId);

  // 실행 전 전체 단계 미리보기 (dry-run 서비스). live trace 가 생기면 그쪽이 우선.
  const [preview, setPreview] = useState<TraceEntry[]>([]);
  const [previewBusy, setPreviewBusy] = useState(false);
  const onPreview = async () => {
    setPreviewBusy(true);
    const res = await previewSvc.call({});
    setPreviewBusy(false);
    setPreview((res.data as { steps?: TraceEntry[] } | null)?.steps ?? []);
  };

  const st = state.value;
  const entries: TraceEntry[] = trace.value?.entries ?? [];
  const status = st?.status ?? TaskStatus.IDLE;
  const currentName = st?.current_name ?? "";
  const breakpoints = new Set(st?.breakpoints ?? []);
  const running = status === TaskStatus.RUNNING;
  const paused = status === TaskStatus.PAUSED;

  return (
    <div
      className="flex h-full flex-col gap-2 overflow-y-auto p-3 text-[12px]"
      data-testid="task-progress-panel"
    >
      <div className="flex items-center gap-2">
        <span className="font-mono uppercase text-muted-foreground">status</span>
        <span
          className={`font-mono font-semibold ${STATUS_COLOR[status] ?? "text-zinc-400"}`}
          data-testid="task-status"
        >
          {status}
        </span>
        {st?.task_name && (
          <span className="truncate font-mono text-muted-foreground">
            · {st.task_name}
          </span>
        )}
        {paused && currentName && (
          <span className="truncate font-mono text-amber-400">
            ⏸ {st?.current_title || currentName} 직전
          </span>
        )}
      </div>

      {/* 디버거 컨트롤 — VSCode 등가 (pause / resume / step over) */}
      <div className="flex flex-wrap gap-2">
        <Button
          size="sm"
          variant="outline"
          disabled={!running}
          onClick={() => void pauseSvc.call({})}
          data-testid="task-pause"
        >
          일시정지
        </Button>
        <Button
          size="sm"
          variant="outline"
          disabled={!paused}
          onClick={() => void resumeSvc.call({})}
          data-testid="task-resume"
        >
          재개
        </Button>
        <Button
          size="sm"
          variant="outline"
          disabled={!paused}
          onClick={() => void stepOnceSvc.call({})}
          data-testid="task-step-once"
        >
          한 스텝
        </Button>
      </div>

      {st?.error && (
        <div
          className="rounded border border-red-800/60 bg-red-950/30 p-2 font-mono text-red-300"
          data-testid="task-error"
        >
          {st.error}
        </div>
      )}

      <div className="flex items-center justify-between">
        <span className="font-mono uppercase text-muted-foreground">
          {entries.length === 0 && preview.length > 0
            ? `예상 단계 (${preview.length})`
            : `trace (${entries.length})`}
        </span>
        {/* 실행 전 전체 단계 미리보기 — imperative 시나리오라 dry-run 으로 수집.
            live trace 가 없을 때만 의미 (실행 시작하면 실제 trace 가 대체). */}
        <Button
          size="sm"
          variant="outline"
          disabled={previewBusy || running || paused}
          onClick={() => void onPreview()}
          data-testid="task-preview"
        >
          {previewBusy ? "불러오는 중…" : "전체 단계 미리보기"}
        </Button>
      </div>

      {/* live trace 없음 + 미리보기 있음 → 예상 단계 목록 (회색, 미실행). */}
      {entries.length === 0 && preview.length > 0 && (
        <div className="flex flex-col gap-1" data-testid="task-preview-entries">
          {preview.map((e, i) => (
            <div
              key={`p:${i}:${e.name}`}
              className="flex items-center gap-2 rounded border border-zinc-800 px-2 py-1 opacity-70"
              style={{ marginLeft: (e.depth ?? 0) * 14 }}
              data-testid="task-preview-entry"
            >
              <span className="h-2 w-2 shrink-0 rounded-full bg-zinc-600" />
              <span className="flex-1 truncate font-mono">
                {e.title || e.name}
              </span>
            </div>
          ))}
        </div>
      )}

      <div className="flex flex-col gap-1" data-testid="task-entries">
        {entries.length === 0 ? (
          preview.length > 0 ? null : (
            <span className="text-muted-foreground">
              task 대기 중… ([전체 단계 미리보기]로 예상 단계 확인 / 실행하면 실제
              호출이 여기 쌓임)
            </span>
          )
        ) : (
          entries.map((e, i) => {
            const hasBp = breakpoints.has(e.name);
            const isCurrent = e.name === currentName;
            return (
              <div
                key={`${i}:${e.name}`}
                className={`flex items-center gap-2 rounded border px-2 py-1 ${
                  isCurrent ? "border-sky-500" : "border-zinc-700"
                }`}
                // 중첩 step — depth 들여쓰기 (wire 는 flat 리스트 + depth,
                // 트리 표현은 UI 몫. 접기/펼치기는 후속 polish).
                style={{ marginLeft: (e.depth ?? 0) * 14 }}
                data-testid="task-entry"
              >
                {/* dot = 상태색. 클릭 = breakpoint toggle (빨간 ring) — 같은
                    step name 은 다음 run 에서도 유효 (runner 가 run 간 보존). */}
                <button
                  type="button"
                  onClick={() => void toggleBpSvc.call({ name: e.name })}
                  title="브레이크포인트 토글"
                  data-testid="task-entry-bp"
                  className={`h-3 w-3 shrink-0 rounded-full ${ENTRY_DOT[e.status] ?? "bg-zinc-600"} ${
                    hasBp ? "ring-2 ring-red-500" : "hover:ring-2 hover:ring-zinc-500"
                  }`}
                />
                {/* title = 표시 문구 (한글 등), name = 식별자 — title 있으면
                    title 이 주(主), name 은 작은 보조 (breakpoint 대상 확인용) */}
                <span className="flex-1 truncate font-mono">
                  {e.title || e.name}
                  {e.title && (
                    <span className="ml-1 text-[10px] text-muted-foreground">
                      {e.name}
                    </span>
                  )}
                </span>
                {e.detail && (
                  <span
                    className={`max-w-[40%] truncate font-mono text-[10px] ${
                      e.status === "failed" ? "text-red-400" : "text-muted-foreground"
                    }`}
                    title={e.detail}
                  >
                    {e.detail}
                  </span>
                )}
                {paused && (
                  <button
                    type="button"
                    onClick={() => void runToSvc.call({ name: e.name })}
                    title="여기까지 실행 (run to cursor)"
                    data-testid="task-run-to"
                    className="font-mono text-[10px] text-zinc-500 hover:text-sky-400"
                  >
                    ▶│
                  </button>
                )}
                <span className="font-mono text-[10px] text-muted-foreground">
                  {e.status}
                </span>
              </div>
            );
          })
        )}
      </div>
    </div>
  );
}
