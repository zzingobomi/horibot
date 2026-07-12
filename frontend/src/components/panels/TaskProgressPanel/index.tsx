/**
 * TaskProgressPanel — task 진행 표시 + 디버거 (task 공통 부품).
 *
 * TRACE (primitive 호출 누적 — label/kind/status/detail) + STATE (status/
 * current_label/error/breakpoints) 로 진행을 그린다. 옛 DSL 의 TREE(사전 step
 * 목록)는 소멸 — imperative 시나리오는 실행해 봐야 경로가 정해지므로, 표시는
 * "실행된 것의 누적" + 직전 run 의 trace 가 다음 run 의 breakpoint 대상 (label
 * 은 run 간 안정 — 시나리오 코드의 label= 리터럴).
 *
 * 디버거: ● dot 클릭 = TOGGLE_BREAKPOINT(label) (없는 run 중에도 미리 박기 —
 * runner 가 run 간 보존) / RUNNING 에서 [일시정지] / PAUSED 에서 [재개]·[한 스텝]·
 * entry 별 [▶ 여기까지] (run-to-cursor). 실패 = error 박스에 사유 (backend 가
 * "다음 행동" 까지 담아 보냄 — 침묵 금지).
 */
import { Button } from "@/components/ui/button";
import { useService, useStream } from "@/framework";
import { useTaskRobotId } from "@/hooks/useTasks";
import {
  ServiceKey,
  TaskStatus,
  Topic,
  type TraceEntry,
} from "@/api/generated/contract";

const TASK_NAME = "pick_and_place";

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
  const robotId = useTaskRobotId(TASK_NAME) ?? "";
  const state = useStream(Topic.PICKANDPLACE_STATE, { robotId });
  const trace = useStream(Topic.PICKANDPLACE_TRACE, { robotId });

  const pauseSvc = useService(ServiceKey.PICKANDPLACE_PAUSE, robotId);
  const resumeSvc = useService(ServiceKey.PICKANDPLACE_RESUME, robotId);
  const stepOnceSvc = useService(ServiceKey.PICKANDPLACE_STEP_ONCE, robotId);
  const runToSvc = useService(ServiceKey.PICKANDPLACE_RUN_TO, robotId);
  const toggleBpSvc = useService(ServiceKey.PICKANDPLACE_TOGGLE_BREAKPOINT, robotId);

  const st = state.value;
  const entries: TraceEntry[] = trace.value?.entries ?? [];
  const status = st?.status ?? TaskStatus.IDLE;
  const currentLabel = st?.current_label ?? "";
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
        {paused && currentLabel && (
          <span className="truncate font-mono text-amber-400">
            ⏸ {currentLabel} 직전
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

      <div className="font-mono uppercase text-muted-foreground">
        trace ({entries.length})
      </div>
      <div className="flex flex-col gap-1" data-testid="task-entries">
        {entries.length === 0 ? (
          <span className="text-muted-foreground">
            task 대기 중… (실행하면 primitive 호출이 여기 쌓임)
          </span>
        ) : (
          entries.map((e, i) => {
            const hasBp = breakpoints.has(e.label);
            const isCurrent = e.label === currentLabel;
            return (
              <div
                key={`${i}:${e.label}`}
                className={`flex items-center gap-2 rounded border px-2 py-1 ${
                  isCurrent ? "border-sky-500" : "border-zinc-700"
                }`}
                data-testid="task-entry"
              >
                {/* dot = 상태색. 클릭 = breakpoint toggle (빨간 ring) — 같은
                    label 은 다음 run 에서도 유효 (runner 가 run 간 보존). */}
                <button
                  type="button"
                  onClick={() => void toggleBpSvc.call({ label: e.label })}
                  title="브레이크포인트 토글"
                  data-testid="task-entry-bp"
                  className={`h-3 w-3 shrink-0 rounded-full ${ENTRY_DOT[e.status] ?? "bg-zinc-600"} ${
                    hasBp ? "ring-2 ring-red-500" : "hover:ring-2 hover:ring-zinc-500"
                  }`}
                />
                <span className="flex-1 truncate font-mono">
                  {e.label}
                  <span className="ml-1 text-[10px] text-muted-foreground">
                    {e.kind}
                  </span>
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
                    onClick={() => void runToSvc.call({ label: e.label })}
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
