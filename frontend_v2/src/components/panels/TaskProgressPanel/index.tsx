/**
 * TaskProgressPanel — Task 진행 표시 + 디버거 (§17.4 + §17.1.4). RobotTaskMode 코어.
 *
 * TASK_TREE (step 목록 — PREVIEW/실행 시 publish) + TASK_STATE (status + step별
 * 상태 + breakpoints) 로 진행을 그린다. v1 디버거 플로우 계승:
 *   - step 의 ● dot 클릭 = TOGGLE_BREAKPOINT (preview 상태에서 미리 박기 —
 *     backend runner 가 run 간 breakpoint 보존)
 *   - PAUSED 에서 [재개] / [한 스텝] / step 별 [▶ 여기까지] (run-to-cursor)
 *   - RUNNING 에서 [일시정지]
 * robot-scoped 스트림 (stream/task/{robot_id}/...) — useStream 이 robotId expand.
 */
import { useParams } from "react-router-dom";
import { Button } from "@/components/ui/button";
import { DEFAULT_ROBOT_ID } from "@/constants";
import { useService, useStream } from "@/framework";
import { ServiceKey, TaskStatus, Topic } from "@/api/generated/contract";

const STATUS_COLOR: Record<string, string> = {
  [TaskStatus.RUNNING]: "text-sky-400",
  [TaskStatus.SUCCESS]: "text-emerald-400",
  [TaskStatus.FAILED]: "text-red-400",
  [TaskStatus.PAUSED]: "text-amber-400",
  [TaskStatus.STOPPED]: "text-zinc-400",
  [TaskStatus.IDLE]: "text-zinc-500",
};

const STEP_DOT: Record<string, string> = {
  pending: "bg-zinc-600",
  running: "bg-sky-400",
  completed: "bg-emerald-400",
  failed: "bg-red-400",
};

interface StepNode {
  id: string;
  label?: string;
  type?: string;
}

export function TaskProgressPanel() {
  const { id } = useParams<{ id: string }>();
  const robotId = id ?? DEFAULT_ROBOT_ID;
  const state = useStream(Topic.TASK_STATE, { robotId });
  const tree = useStream(Topic.TASK_TREE, { robotId });

  const pauseSvc = useService(ServiceKey.TASK_PAUSE, robotId);
  const resumeSvc = useService(ServiceKey.TASK_RESUME, robotId);
  const stepOnceSvc = useService(ServiceKey.TASK_STEP_ONCE, robotId);
  const runToSvc = useService(ServiceKey.TASK_RUN_TO, robotId);
  const toggleBpSvc = useService(ServiceKey.TASK_TOGGLE_BREAKPOINT, robotId);

  const st = state.value;
  const steps = (tree.value?.steps ?? []) as StepNode[];
  const statuses = st?.step_statuses ?? {};
  const currentId = st?.current_step_id ?? "";
  const status = st?.status ?? TaskStatus.IDLE;
  const breakpoints = new Set(st?.breakpoints ?? []);
  const running = status === TaskStatus.RUNNING;
  const paused = status === TaskStatus.PAUSED;

  const onToggleBp = (stepId: string) =>
    void toggleBpSvc.call({ robot_id: robotId, step_id: stepId });

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
      </div>

      {/* 디버거 컨트롤 — VSCode 등가 (pause / resume / step over) */}
      <div className="flex flex-wrap gap-2">
        <Button
          size="sm"
          variant="outline"
          disabled={!running}
          onClick={() => void pauseSvc.call({ robot_id: robotId })}
          data-testid="task-pause"
        >
          일시정지
        </Button>
        <Button
          size="sm"
          variant="outline"
          disabled={!paused}
          onClick={() => void resumeSvc.call({ robot_id: robotId })}
          data-testid="task-resume"
        >
          재개
        </Button>
        <Button
          size="sm"
          variant="outline"
          disabled={!paused}
          onClick={() => void stepOnceSvc.call({ robot_id: robotId })}
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
        steps ({steps.length})
      </div>
      <div className="flex flex-col gap-1" data-testid="task-steps">
        {steps.length === 0 ? (
          <span className="text-muted-foreground">
            task 대기 중… (파싱하면 step 목록이 여기 뜸)
          </span>
        ) : (
          steps.map((s) => {
            const sstat = statuses[s.id] ?? "pending";
            const hasBp = breakpoints.has(s.id);
            return (
              <div
                key={s.id}
                className={`flex items-center gap-2 rounded border px-2 py-1 ${
                  s.id === currentId ? "border-sky-500" : "border-zinc-700"
                }`}
                data-testid="task-step"
              >
                {/* dot = 상태색. 클릭 = breakpoint toggle (빨간 ring). */}
                <button
                  type="button"
                  onClick={() => onToggleBp(s.id)}
                  title="브레이크포인트 토글"
                  data-testid="task-step-bp"
                  className={`h-3 w-3 shrink-0 rounded-full ${STEP_DOT[sstat] ?? "bg-zinc-600"} ${
                    hasBp ? "ring-2 ring-red-500" : "hover:ring-2 hover:ring-zinc-500"
                  }`}
                />
                <span className="flex-1 truncate font-mono">
                  {s.label || s.type || s.id}
                </span>
                {paused && (
                  <button
                    type="button"
                    onClick={() =>
                      void runToSvc.call({ robot_id: robotId, step_id: s.id })
                    }
                    title="여기까지 실행 (run to cursor)"
                    data-testid="task-run-to"
                    className="font-mono text-[10px] text-zinc-500 hover:text-sky-400"
                  >
                    ▶│
                  </button>
                )}
                <span className="font-mono text-[10px] text-muted-foreground">
                  {sstat}
                </span>
              </div>
            );
          })
        )}
      </div>
    </div>
  );
}
