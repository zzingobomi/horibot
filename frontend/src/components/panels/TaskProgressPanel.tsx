import { useEffect } from "react";
import { ListChecks } from "lucide-react";
import type { IDockviewPanelProps } from "dockview";
import { PanelShell } from "@/components/canvas/ui/PanelShell";
import { Section } from "@/components/canvas/ui/Section";
import { StepProgress } from "@/components/common/StepProgress";
import { useTask } from "@/hooks/useTask";
import type { TaskStatus } from "@/types/task";

const STATUS_COLOR: Record<TaskStatus, string> = {
  idle: "text-zinc-400",
  running: "text-emerald-400",
  paused: "text-amber-400",
  success: "text-sky-400",
  failed: "text-red-400",
  stopped: "text-zinc-500",
};

const STATUS_LABEL: Record<TaskStatus, string> = {
  idle: "IDLE",
  running: "RUNNING",
  paused: "PAUSED",
  success: "SUCCESS",
  failed: "FAILED",
  stopped: "STOPPED",
};

export function TaskProgressPanel(props: IDockviewPanelProps<object>) {
  const { taskState, syncStatus } = useTask();

  useEffect(() => {
    syncStatus();
  }, [syncStatus]);

  return (
    <PanelShell
      icon={<ListChecks className="w-3.5 h-3.5" />}
      title="Task Progress"
      panelId={props.api.id}
      api={props.api}
    >
      <Section label="Status">
        <div className="flex items-center gap-2">
          <span
            className={`font-mono text-[11px] font-bold ${
              STATUS_COLOR[taskState.status]
            }`}
          >
            {STATUS_LABEL[taskState.status]}
          </span>
          {taskState.status === "running" && (
            <span className="w-1.5 h-1.5 rounded-full bg-emerald-400 animate-pulse" />
          )}
          {taskState.task_name && (
            <span className="ml-auto font-mono text-[10px] text-zinc-500 truncate">
              {taskState.task_name}
            </span>
          )}
        </div>
      </Section>

      <Section label="Progress">
        <StepProgress
          currentStep={taskState.current_step}
          totalSteps={taskState.total_steps}
          currentLabel={taskState.current_label}
        />
      </Section>

      {taskState.error && (
        <Section label="Error">
          <p className="font-mono text-[11px] text-red-400 break-all">
            {taskState.error}
          </p>
        </Section>
      )}
    </PanelShell>
  );
}
