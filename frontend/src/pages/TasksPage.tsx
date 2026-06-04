/**
 * /tasks/:name — multi_robot_phase2_frontend.md §2 sketch.
 *
 * task 자체가 robot 을 포함 (selector 없음). focus=null = WorldPage 형태의
 * multi-robot 시각화 + task control panel (prompt / progress / camera).
 *
 * 첫 프로토타입: task name 은 사용자가 URL 로 명시. backend 의 task registry
 * 자동 enumeration 은 별도 endpoint 추가 자리 (Slice C 이후).
 */
import { useCallback, useMemo, useRef } from "react";
import { useParams } from "react-router-dom";
import { DockviewReact, type DockviewReadyEvent } from "dockview";
import { RotateCcw } from "lucide-react";
import { RobotSceneContainer } from "@/components/canvas/RobotSceneContainer";
import {
  PANEL_COMPONENTS,
  type PanelComponentKey,
} from "@/components/canvas/dockview/panelComponents";
import {
  PANEL_HEADER_HEIGHT,
  loadCollapsed,
  loadLayout,
  resetWorkspaceLayout,
  saveLayout,
} from "@/lib/workspaceLayout";
import { useTasks } from "@/hooks/useTasks";

type PanelSpec = {
  id: string;
  component: PanelComponentKey;
  title: string;
  width: number;
  height: number;
};

const PANELS: PanelSpec[] = [
  { id: "prompt", component: "prompt", title: "Prompt", width: 280, height: 240 },
  { id: "task-progress", component: "taskProgress", title: "Task Progress", width: 280, height: 200 },
  { id: "camera-feed", component: "cameraFeed", title: "Camera Feed", width: 320, height: 240 },
  { id: "robot-state", component: "robotState", title: "Robot State", width: 260, height: 270 },
];

export function TasksPage() {
  const { name = "pick_and_place" } = useParams<{ name: string }>();
  const { tasks, loading, error } = useTasks();
  const containerRef = useRef<HTMLDivElement | null>(null);
  const layoutKey = `tasks.${name}`;

  const addDefaultLayout = useCallback(
    (event: DockviewReadyEvent) => {
      const MARGIN = 16;
      const GAP_X = 12;
      const GAP_Y = 18;
      const containerWidth =
        containerRef.current?.clientWidth ?? window.innerWidth;

      let x = MARGIN;
      let y = MARGIN;
      let rowHeight = 0;

      for (const p of PANELS) {
        if (x + p.width > containerWidth - MARGIN && x > MARGIN) {
          x = MARGIN;
          y += rowHeight + GAP_Y;
          rowHeight = 0;
        }
        const collapsed = loadCollapsed(p.id);
        event.api.addPanel({
          id: p.id,
          component: p.component,
          title: p.title,
          floating: {
            x,
            y,
            width: p.width,
            height: collapsed ? PANEL_HEADER_HEIGHT : p.height,
          },
          params: {},
        });
        x += p.width + GAP_X;
        rowHeight = Math.max(rowHeight, collapsed ? PANEL_HEADER_HEIGHT : p.height);
      }
    },
    [],
  );

  const onReady = useCallback(
    (event: DockviewReadyEvent) => {
      const saved = loadLayout(layoutKey);
      if (saved) {
        try {
          event.api.fromJSON(saved as Parameters<typeof event.api.fromJSON>[0]);
        } catch {
          addDefaultLayout(event);
        }
      } else {
        addDefaultLayout(event);
      }

      let timer: ReturnType<typeof setTimeout> | null = null;
      event.api.onDidLayoutChange(() => {
        if (timer) clearTimeout(timer);
        timer = setTimeout(() => {
          saveLayout(layoutKey, event.api.toJSON());
        }, 300);
      });
    },
    [addDefaultLayout, layoutKey],
  );

  const handleReset = useCallback(() => {
    resetWorkspaceLayout(layoutKey);
    window.location.reload();
  }, [layoutKey]);

  const dockviewKey = useMemo(() => layoutKey, [layoutKey]);

  if (error) {
    return (
      <div className="p-6 text-red-400 font-mono">/tasks 응답 실패: {error}</div>
    );
  }
  if (loading) {
    return <div className="p-6 text-zinc-400 font-mono">tasks 로드 중...</div>;
  }
  if (!tasks.includes(name)) {
    return (
      <div className="p-6 text-zinc-400 font-mono">
        task <span className="text-red-400">{name}</span> 없음. 등록된 task:{" "}
        {tasks.join(", ") || "(없음)"}
      </div>
    );
  }

  return (
    <div
      ref={containerRef}
      className="relative w-full h-full overflow-hidden bg-[#080c12]"
      style={{ fontFamily: "'JetBrains Mono', 'Fira Code', monospace" }}
    >
      {/* task 는 multi-robot — focusId=null (World overview). */}
      <div className="absolute inset-0 z-0">
        <RobotSceneContainer focusId={null} />
      </div>

      <div className="absolute inset-0 z-10 workspace-dockview pointer-events-none">
        <DockviewReact
          key={dockviewKey}
          className="dockview-theme-dark"
          components={PANEL_COMPONENTS}
          onReady={onReady}
        />
      </div>

      <div className="absolute top-3 right-3 z-20 flex items-center gap-2">
        <div className="px-3 py-2 rounded bg-zinc-900/80 border border-zinc-700/60 text-zinc-300 text-xs font-mono pointer-events-none">
          <div className="text-zinc-100 font-semibold">{name}</div>
          <div className="text-zinc-500">task</div>
        </div>
        <button
          onClick={handleReset}
          title="패널 레이아웃 초기화"
          className="flex items-center gap-1.5 px-2 py-1 rounded bg-zinc-900/80 hover:bg-zinc-800 border border-zinc-700/60 text-zinc-400 hover:text-zinc-100 text-[10px] font-mono pointer-events-auto transition-colors"
        >
          <RotateCcw className="w-3 h-3" />
          Reset layout
        </button>
      </div>
    </div>
  );
}

