import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { createPortal } from "react-dom";
import {
  ListChecks,
  Play,
  Square,
  StepForward,
  Check,
  X as XIcon,
  Loader2,
  Circle,
  ChevronRight,
  ChevronDown,
} from "lucide-react";
import type { IDockviewPanelProps } from "dockview";
import { PanelShell } from "@/components/shared/PanelShell";
import { useService, useTopic } from "@/framework";
import { ServiceKey, Topic } from "@/constants/topics";
import {
  defaultTaskState,
  defaultTaskTree,
  type StepNode,
  type StepStatus,
  type TaskState,
  type TaskStatus,
  type TaskTree,
} from "@/types/task";

const STATUS_TEXT: Record<TaskStatus, string> = {
  idle: "IDLE",
  running: "RUNNING",
  paused: "PAUSED",
  success: "SUCCESS",
  failed: "FAILED",
  stopped: "STOPPED",
};

const STATUS_DOT: Record<TaskStatus, string> = {
  idle: "bg-zinc-500",
  running: "bg-emerald-400 animate-pulse",
  paused: "bg-amber-400",
  success: "bg-sky-400",
  failed: "bg-red-400",
  stopped: "bg-zinc-500",
};

interface ContextMenu {
  stepId: string;
  x: number;
  y: number;
}

export function TaskProgressPanel(props: IDockviewPanelProps<object>) {
  const taskState =
    (useTopic(Topic.TASK_STATE) as TaskState | null) ?? defaultTaskState;
  const taskTree =
    (useTopic(Topic.TASK_TREE) as TaskTree | null) ?? defaultTaskTree;
  const stopSvc = useService(ServiceKey.TASK_STOP);
  const resumeSvc = useService(ServiceKey.TASK_RESUME);
  const stepSvc = useService(ServiceKey.TASK_STEP);
  const runToSvc = useService(ServiceKey.TASK_RUN_TO);
  const toggleBpSvc = useService(ServiceKey.TASK_TOGGLE_BREAKPOINT);

  const isPaused = taskState.status === "paused";
  const isActive =
    taskState.status === "running" || taskState.status === "paused";

  const [expanded, setExpanded] = useState<Set<string>>(() => new Set());
  const [menu, setMenu] = useState<ContextMenu | null>(null);

  const toggleExpand = useCallback((stepId: string) => {
    setExpanded((prev) => {
      const next = new Set(prev);
      if (next.has(stepId)) next.delete(stepId);
      else next.add(stepId);
      return next;
    });
  }, []);

  const breakpointSet = useMemo(
    () => new Set(taskState.breakpoints),
    [taskState.breakpoints],
  );

  useEffect(() => {
    if (!menu) return;
    const close = () => setMenu(null);
    const onKey = (e: KeyboardEvent) => {
      if (e.key === "Escape") setMenu(null);
    };
    window.addEventListener("mousedown", close);
    window.addEventListener("keydown", onKey);
    return () => {
      window.removeEventListener("mousedown", close);
      window.removeEventListener("keydown", onKey);
    };
  }, [menu]);

  const handleContextMenu = useCallback(
    (e: React.MouseEvent, stepId: string) => {
      e.preventDefault();
      setMenu({ stepId, x: e.clientX, y: e.clientY });
    },
    [],
  );

  return (
    <PanelShell
      icon={<ListChecks className="w-3.5 h-3.5" />}
      title="Task"
      panelId={props.api.id}
      api={props.api}
    >
      <div className="px-3 py-2 border-b border-zinc-800/60 flex flex-col gap-2">
        <div className="flex items-center gap-2">
          <span
            className={`w-1.5 h-1.5 rounded-full ${STATUS_DOT[taskState.status]}`}
          />
          <span className="font-mono text-[10px] font-bold tracking-wider text-zinc-300">
            {STATUS_TEXT[taskState.status]}
          </span>
          {taskState.task_name && (
            <span className="ml-auto font-mono text-[10px] text-zinc-500 truncate">
              {taskState.task_name}
            </span>
          )}
        </div>

        <div className="flex gap-1">
          <ControlBtn
            label={isPaused ? "Continue" : "Run"}
            icon={<Play className="w-3 h-3" />}
            onClick={() => void resumeSvc.call({})}
            disabled={!isPaused}
            color="emerald"
          />
          <ControlBtn
            label="Step"
            icon={<StepForward className="w-3 h-3" />}
            onClick={() => void stepSvc.call({})}
            disabled={!isPaused}
            color="amber"
          />
          <ControlBtn
            label="Stop"
            icon={<Square className="w-3 h-3" />}
            onClick={() => void stopSvc.call({})}
            disabled={!isActive}
            color="red"
          />
        </div>
      </div>

      <div className="py-1.5">
        {taskTree.steps.length === 0 ? (
          <p className="px-3 py-4 font-mono text-[10px] text-zinc-600 italic">
            (no task — Prompt 패널에서 Run 으로 시작)
          </p>
        ) : (
          taskTree.steps.map((node) => (
            <StepRow
              key={node.id}
              node={node}
              depth={0}
              taskState={taskState}
              expanded={expanded}
              breakpointSet={breakpointSet}
              onToggleExpand={toggleExpand}
              onContextMenu={handleContextMenu}
              onToggleBreakpoint={(id) => void toggleBpSvc.call({ step_id: id })}
            />
          ))
        )}
      </div>

      {taskState.error && (
        <div className="px-3 py-2 border-t border-red-900/40 bg-red-950/20">
          <p className="text-[9px] font-mono uppercase tracking-widest text-red-500 mb-1">
            Error
          </p>
          <p className="font-mono text-[11px] text-red-400 break-all">
            {taskState.error}
          </p>
        </div>
      )}

      {menu &&
        createPortal(
          <div
            className="fixed z-50 bg-zinc-900 border border-zinc-700 rounded shadow-lg py-1 min-w-[140px]"
            style={{ left: menu.x, top: menu.y }}
            onMouseDown={(e) => e.stopPropagation()}
          >
            <MenuItem
              label="Run to here"
              onClick={() => {
                void runToSvc.call({ step_id: menu.stepId });
                setMenu(null);
              }}
              disabled={!isPaused}
            />
            <MenuItem
              label={
                breakpointSet.has(menu.stepId)
                  ? "Remove breakpoint"
                  : "Add breakpoint"
              }
              onClick={() => {
                void toggleBpSvc.call({ step_id: menu.stepId });
                setMenu(null);
              }}
            />
          </div>,
          document.body,
        )}
    </PanelShell>
  );
}

// ─── 서브 컴포넌트 ─────────────────────────────────────────

interface ControlBtnProps {
  label: string;
  icon: React.ReactNode;
  onClick: () => void;
  disabled?: boolean;
  color: "emerald" | "amber" | "red";
}

const CTRL_COLOR: Record<ControlBtnProps["color"], string> = {
  emerald: "bg-emerald-600 hover:bg-emerald-500",
  amber: "bg-amber-600 hover:bg-amber-500",
  red: "bg-red-700 hover:bg-red-600",
};

function ControlBtn({ label, icon, onClick, disabled, color }: ControlBtnProps) {
  return (
    <button
      onClick={onClick}
      disabled={disabled}
      className={`flex-1 h-7 rounded ${CTRL_COLOR[color]} disabled:bg-zinc-800 disabled:text-zinc-600 disabled:cursor-not-allowed text-white text-[10px] font-mono uppercase tracking-wider flex items-center justify-center gap-1 transition-colors`}
    >
      {icon}
      {label}
    </button>
  );
}

interface StepRowProps {
  node: StepNode;
  depth: number;
  taskState: TaskState;
  expanded: Set<string>;
  breakpointSet: Set<string>;
  onToggleExpand: (id: string) => void;
  onContextMenu: (e: React.MouseEvent, id: string) => void;
  onToggleBreakpoint: (id: string) => void;
}

const HIDDEN_PARAM_KEYS = new Set(["id", "type", "label", "children"]);

function formatParamValue(v: unknown): string {
  if (v === null) return "null";
  if (v === undefined) return "—";
  if (typeof v === "string") return v;
  if (typeof v === "number" || typeof v === "boolean") return String(v);
  try {
    return JSON.stringify(v);
  } catch {
    return String(v);
  }
}

function StepRow({
  node,
  depth,
  taskState,
  expanded,
  breakpointSet,
  onToggleExpand,
  onContextMenu,
  onToggleBreakpoint,
}: StepRowProps) {
  const [hover, setHover] = useState(false);
  const gutterRef = useRef<HTMLButtonElement>(null);

  const status: StepStatus = taskState.step_statuses[node.id] ?? "pending";
  const isCurrent = taskState.current_step_id === node.id;
  const isBreakpoint = breakpointSet.has(node.id);
  const isExpanded = expanded.has(node.id);
  const children = node.children;
  const hasChildren = Array.isArray(children) && children.length > 0;

  const bg = isCurrent ? "bg-amber-900/20" : "hover:bg-zinc-900/40";

  const paramEntries = useMemo(
    () => Object.entries(node).filter(([k]) => !HIDDEN_PARAM_KEYS.has(k)),
    [node],
  );

  const indentPx = depth * 14;

  return (
    <div>
      <div
        className={`flex items-center gap-2 px-2 py-1 cursor-pointer ${bg} transition-colors`}
        style={{ paddingLeft: 8 + indentPx }}
        onClick={() => onToggleExpand(node.id)}
        onContextMenu={(e) => onContextMenu(e, node.id)}
        onMouseEnter={() => setHover(true)}
        onMouseLeave={() => setHover(false)}
      >
        <button
          ref={gutterRef}
          onClick={(e) => {
            e.stopPropagation();
            onToggleBreakpoint(node.id);
          }}
          title={isBreakpoint ? "Remove breakpoint" : "Add breakpoint"}
          className="w-3 h-3 flex items-center justify-center shrink-0 rounded-full"
        >
          {isBreakpoint ? (
            <span className="w-2.5 h-2.5 rounded-full bg-red-500" />
          ) : hover ? (
            <span className="w-2 h-2 rounded-full bg-red-500/30" />
          ) : null}
        </button>

        {isExpanded ? (
          <ChevronDown className="w-3 h-3 text-zinc-600 shrink-0" />
        ) : (
          <ChevronRight className="w-3 h-3 text-zinc-700 shrink-0" />
        )}

        <StatusIcon status={status} isCurrent={isCurrent} />

        <span
          className={`font-mono text-[11px] truncate ${
            status === "completed"
              ? "text-zinc-500"
              : status === "failed"
                ? "text-red-400"
                : isCurrent
                  ? "text-amber-200"
                  : "text-zinc-300"
          }`}
        >
          {node.label || node.type}
          {hasChildren && (
            <span className="ml-1.5 text-zinc-600 text-[9px]">
              ({children!.length})
            </span>
          )}
        </span>
      </div>

      {isExpanded && (
        <div
          className="px-2 pb-1.5 bg-zinc-950/40 border-y border-zinc-800/40"
          style={{ paddingLeft: 40 + indentPx }}
        >
          <div className="grid grid-cols-[auto_1fr] gap-x-3 gap-y-0.5 py-1 font-mono text-[10px]">
            <span className="text-zinc-500">type</span>
            <span className="text-zinc-300">{node.type}</span>
            {paramEntries.map(([k, v]) => (
              <div key={k} className="contents">
                <span className="text-zinc-500">{k}</span>
                <span className="text-zinc-300 break-all">
                  {formatParamValue(v)}
                </span>
              </div>
            ))}
          </div>
        </div>
      )}

      {hasChildren &&
        children!.map((child) => (
          <StepRow
            key={child.id}
            node={child}
            depth={depth + 1}
            taskState={taskState}
            expanded={expanded}
            breakpointSet={breakpointSet}
            onToggleExpand={onToggleExpand}
            onContextMenu={onContextMenu}
            onToggleBreakpoint={onToggleBreakpoint}
          />
        ))}
    </div>
  );
}

function StatusIcon({
  status,
  isCurrent,
}: {
  status: StepStatus;
  isCurrent: boolean;
}) {
  const cls = "w-3 h-3 shrink-0";
  if (status === "completed")
    return <Check className={`${cls} text-emerald-500`} />;
  if (status === "failed") return <XIcon className={`${cls} text-red-500`} />;
  if (status === "running")
    return <Loader2 className={`${cls} text-amber-400 animate-spin`} />;
  return (
    <Circle
      className={`${cls} ${isCurrent ? "text-amber-300" : "text-zinc-700"}`}
    />
  );
}

function MenuItem({
  label,
  onClick,
  disabled,
}: {
  label: string;
  onClick: () => void;
  disabled?: boolean;
}) {
  return (
    <button
      onClick={onClick}
      disabled={disabled}
      className="w-full text-left px-3 py-1.5 text-[11px] font-mono text-zinc-300 hover:bg-zinc-800 disabled:text-zinc-600 disabled:cursor-not-allowed transition-colors"
    >
      {label}
    </button>
  );
}
