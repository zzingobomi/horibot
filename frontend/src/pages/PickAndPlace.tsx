import { useCallback, useMemo, useRef } from "react";
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

const LAYOUT_KEY = "pickandplace";

export function PickAndPlace() {
  const containerRef = useRef<HTMLDivElement | null>(null);

  type PanelSpec = {
    id: string;
    component: PanelComponentKey;
    title: string;
    width: number;
    height: number;
  };

  const PANELS: PanelSpec[] = useMemo(
    () => [
      { id: "prompt", component: "prompt", title: "Prompt", width: 280, height: 240 },
      { id: "task-progress", component: "taskProgress", title: "Task Progress", width: 280, height: 200 },
      { id: "camera-feed", component: "cameraFeed", title: "Camera Feed", width: 320, height: 240 },
      { id: "robot-state", component: "robotState", title: "Robot State", width: 260, height: 270 },
    ],
    [],
  );

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
        rowHeight = Math.max(
          rowHeight,
          collapsed ? PANEL_HEADER_HEIGHT : p.height,
        );
      }
    },
    [PANELS],
  );

  const onReady = useCallback(
    (event: DockviewReadyEvent) => {
      const saved = loadLayout(LAYOUT_KEY);
      if (saved) {
        try {
          event.api.fromJSON(
            saved as Parameters<typeof event.api.fromJSON>[0],
          );
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
          saveLayout(LAYOUT_KEY, event.api.toJSON());
        }, 300);
      });
    },
    [addDefaultLayout],
  );

  const handleReset = useCallback(() => {
    resetWorkspaceLayout(LAYOUT_KEY);
    window.location.reload();
  }, []);

  return (
    <div
      ref={containerRef}
      className="relative w-full h-full overflow-hidden bg-[#080c12]"
      style={{ fontFamily: "'JetBrains Mono', 'Fira Code', monospace" }}
    >
      <div className="absolute inset-0 z-0">
        <RobotSceneContainer />
      </div>

      <div className="absolute inset-0 z-10 workspace-dockview pointer-events-none">
        <DockviewReact
          className="dockview-theme-dark"
          components={PANEL_COMPONENTS}
          onReady={onReady}
        />
      </div>

      <button
        onClick={handleReset}
        title="패널 레이아웃 초기화"
        className="absolute top-3 right-3 z-20 flex items-center gap-1.5 px-2 py-1 rounded bg-zinc-900/80 hover:bg-zinc-800 border border-zinc-700/60 text-zinc-400 hover:text-zinc-100 text-[10px] font-mono pointer-events-auto transition-colors"
      >
        <RotateCcw className="w-3 h-3" />
        Reset layout
      </button>
    </div>
  );
}
