/**
 * TaskProgressPanel — Task 실행 진행 표시 (§17.4). RobotTaskMode 코어.
 *
 * TASK_TREE (step 목록, 실행/preview 시 1회) + TASK_STATE (status + step별 상태 stream)
 * 로 진행을 그린다. PromptPanel 이 시작한 PnP 의 step 진행/성공/실패/현재 step 을 본다.
 * robot-scoped 스트림 (stream/task/{robot_id}/...) — useStream 이 robotId expand.
 */
import { useParams } from "react-router-dom";
import { DEFAULT_ROBOT_ID } from "@/constants";
import { useStream } from "@/framework";
import { TaskStatus, Topic } from "@/api/generated/contract";

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

  const st = state.value;
  const steps = (tree.value?.steps ?? []) as StepNode[];
  const statuses = st?.step_statuses ?? {};
  const currentId = st?.current_step_id ?? "";
  const status = st?.status ?? TaskStatus.IDLE;

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
          <span className="text-muted-foreground">task 대기 중…</span>
        ) : (
          steps.map((s) => {
            const sstat = statuses[s.id] ?? "pending";
            return (
              <div
                key={s.id}
                className={`flex items-center gap-2 rounded border px-2 py-1 ${
                  s.id === currentId ? "border-sky-500" : "border-zinc-700"
                }`}
                data-testid="task-step"
              >
                <span
                  className={`h-2 w-2 shrink-0 rounded-full ${STEP_DOT[sstat] ?? "bg-zinc-600"}`}
                />
                <span className="flex-1 truncate font-mono">
                  {s.label || s.type || s.id}
                </span>
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
