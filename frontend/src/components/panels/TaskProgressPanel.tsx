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
import { PanelShell } from "@/components/canvas/ui/PanelShell";
import { useTask } from "@/hooks/useTask";
import type { StepNode, StepStatus, TaskStatus } from "@/types/task";

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
  const {
    taskState,
    taskTree,
    syncStatus,
    stop,
    resume,
    step,
    runTo,
    toggleBreakpoint,
  } = useTask();

  useEffect(() => {
    syncStatus();
  }, [syncStatus]);

  const isPaused = taskState.status === "paused";
  const isActive =
    taskState.status === "running" || taskState.status === "paused";

  // 클릭으로 펼친 step 들 — 디테일 (type / params) 표시
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

  // breakpoints set 으로 변환 — 잦은 lookup
  const breakpointSet = useMemo(
    () => new Set(taskState.breakpoints),
    [taskState.breakpoints],
  );

  // 컨텍스트 메뉴 외부 클릭/Esc 로 닫기
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

  const onMenuRunTo = useCallback(() => {
    if (!menu) return;
    runTo(menu.stepId);
    setMenu(null);
  }, [menu, runTo]);

  const onMenuToggleBp = useCallback(() => {
    if (!menu) return;
    toggleBreakpoint(menu.stepId);
    setMenu(null);
  }, [menu, toggleBreakpoint]);

  return (
    <PanelShell
      icon={<ListChecks className="w-3.5 h-3.5" />}
      title="Task"
      panelId={props.api.id}
      api={props.api}
    >
      {/* 상단: 상태 + 컨트롤 바 */}
      <div className="px-3 py-2 border-b border-zinc-800/60 flex flex-col gap-2">
        <div className="flex items-center gap-2">
          <span
            className={`w-1.5 h-1.5 rounded-full ${
              STATUS_DOT[taskState.status]
            }`}
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
            onClick={() => resume()}
            disabled={!isPaused}
            color="emerald"
          />
          <ControlBtn
            label="Step"
            icon={<StepForward className="w-3 h-3" />}
            onClick={() => step()}
            disabled={!isPaused}
            color="amber"
          />
          <ControlBtn
            label="Stop"
            icon={<Square className="w-3 h-3" />}
            onClick={() => stop()}
            disabled={!isActive}
            color="red"
          />
        </div>
      </div>

      {/* step 트리 */}
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
              status={taskState.step_statuses[node.id] ?? "pending"}
              isBreakpoint={breakpointSet.has(node.id)}
              isCurrent={taskState.current_step_id === node.id}
              isExpanded={expanded.has(node.id)}
              onToggleExpand={() => toggleExpand(node.id)}
              onContextMenu={(e) => handleContextMenu(e, node.id)}
              onToggleBreakpoint={() => toggleBreakpoint(node.id)}
            />
          ))
        )}
      </div>

      {/* 에러 표시 */}
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

      {/* 컨텍스트 메뉴 — dockview 패널이 transform 으로 위치 잡아서
          position:fixed 의 containing block 이 패널 박스가 됨. Portal 로
          document.body 에 직접 렌더해야 viewport 좌표 기준으로 동작. */}
      {menu &&
        createPortal(
          <div
            className="fixed z-50 bg-zinc-900 border border-zinc-700 rounded shadow-lg py-1 min-w-[140px]"
            style={{ left: menu.x, top: menu.y }}
            onMouseDown={(e) => e.stopPropagation()}
          >
            <MenuItem
              label="Run to here"
              onClick={onMenuRunTo}
              disabled={!isPaused}
            />
            <MenuItem
              label={
                breakpointSet.has(menu.stepId)
                  ? "Remove breakpoint"
                  : "Add breakpoint"
              }
              onClick={onMenuToggleBp}
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

const CTRL_COLOR: Record<
  ControlBtnProps["color"],
  string
> = {
  emerald: "bg-emerald-600 hover:bg-emerald-500",
  amber: "bg-amber-600 hover:bg-amber-500",
  red: "bg-red-700 hover:bg-red-600",
};

function ControlBtn({
  label,
  icon,
  onClick,
  disabled,
  color,
}: ControlBtnProps) {
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
  status: StepStatus;
  isBreakpoint: boolean;
  isCurrent: boolean;
  isExpanded: boolean;
  onToggleExpand: () => void;
  onContextMenu: (e: React.MouseEvent) => void;
  onToggleBreakpoint: () => void;
}

// 디테일 표시에서 제외할 step 필드 — 행 자체에 이미 표시되거나 메타.
const HIDDEN_PARAM_KEYS = new Set(["id", "type", "label", "children"]);

function formatParamValue(v: unknown): string {
  if (v === null) return "null";
  if (v === undefined) return "—";
  if (typeof v === "string") return v;
  if (typeof v === "number" || typeof v === "boolean") return String(v);
  // object / array → compact JSON
  try {
    return JSON.stringify(v);
  } catch {
    return String(v);
  }
}

function StepRow({
  node,
  status,
  isBreakpoint,
  isCurrent,
  isExpanded,
  onToggleExpand,
  onContextMenu,
  onToggleBreakpoint,
}: StepRowProps) {
  const [hover, setHover] = useState(false);
  const gutterRef = useRef<HTMLButtonElement>(null);

  const bg = isCurrent
    ? "bg-amber-900/20"
    : "hover:bg-zinc-900/40";

  // 디테일 표시용 params — type/label/id/children 제외, 정의된 순서 유지.
  const paramEntries = useMemo(
    () =>
      Object.entries(node).filter(([k]) => !HIDDEN_PARAM_KEYS.has(k)),
    [node],
  );

  return (
    <div>
      <div
        className={`flex items-center gap-2 px-2 py-1 cursor-pointer ${bg} transition-colors`}
        onClick={onToggleExpand}
        onContextMenu={onContextMenu}
        onMouseEnter={() => setHover(true)}
        onMouseLeave={() => setHover(false)}
      >
        {/* breakpoint gutter — 호버 시 옅은 점 prefill, 토글 시 빨강 채움 */}
        <button
          ref={gutterRef}
          onClick={(e) => {
            e.stopPropagation();
            onToggleBreakpoint();
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

        {/* expand chevron — 미세하게 시각화. 클릭은 행 전체로 받음 */}
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
        </span>
      </div>

      {/* 디테일 — type + params. 들여쓰기로 행 아래에. */}
      {isExpanded && (
        <div className="px-2 pb-1.5 pl-10 bg-zinc-950/40 border-y border-zinc-800/40">
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
  // pending — 현재 위치(PAUSED 대기)면 강조, 아니면 옅게
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
