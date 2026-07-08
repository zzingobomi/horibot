/**
 * dockview 패널 workspace wrapper — robot mode sub-route(/robots/:id/{mode}) + 최상위
 * tasks 페이지(/tasks) 공유. registry 패널을 PANELS 로 배치, 이동/레이아웃 localStorage 영속.
 *
 * RobotsLayout 이 R3F Canvas 를 z-0 에 마운트한 상태에서, mode component 가
 * Outlet 으로 이 컴포넌트만 z-10 overlay 로 띄움. mode 전환 시 ModeDockview 는
 * 통째로 unmount + remount (panel set 갈아끼움), R3F 는 그대로.
 *
 * layoutKey 는 mode 별 분리 (`workspace3d.<id>.<mode>`) — 한 robot 안에서도
 * mode 별 panel 배치를 독립적으로 기억.
 *
 * 옛 frontend ModeDockview carry over (frontend_v2.md §2.3). 기능 추가 =
 * PANEL_COMPONENTS 등록 + mode 파일 PANELS 한 줄.
 */
import { useCallback, useRef } from "react";
import { useParams } from "react-router-dom";
import {
  DockviewDefaultTab,
  DockviewReact,
  type DockviewReadyEvent,
  type IDockviewPanelHeaderProps,
} from "dockview";
import { RotateCcw } from "lucide-react";
import {
  DOCKVIEW_PANEL_COMPONENTS,
  ROBOT_OWNED_PANELS,
  type PanelComponentKey,
} from "@/components/panels/registry";
import { RobotTab } from "@/components/shared/robotOwnership";
import { useRobots } from "@/hooks/useRobots";
import {
  loadLayout,
  resetWorkspaceLayout,
  saveLayout,
} from "@/lib/workspaceLayout";

export type PanelSpec = {
  id: string;
  component: PanelComponentKey;
  title: string;
  width: number;
  height: number;
};

interface ModeDockviewProps {
  mode: string;
  panels: PanelSpec[];
}

/**
 * Tab close 버튼 hide — panel close 후 다시 살리는 UI 가 없어서 (Reset layout
 * 버튼이 fallback). 사용자 실수로 panel 잃지 않게 hideClose.
 */
function LockedTab(props: IDockviewPanelHeaderProps) {
  return <DockviewDefaultTab {...props} hideClose />;
}

export function ModeDockview({ mode, panels }: ModeDockviewProps) {
  const { id = "" } = useParams<{ id: string }>();
  const { robots } = useRobots();
  const containerRef = useRef<HTMLDivElement | null>(null);
  // id 없는 최상위 페이지(tasks)는 "global" — robot mode 는 robot 별 배치 기억.
  const layoutKey = `workspace3d.${id || "global"}.${mode}`;

  // 패널 **생성 시 초기값** ([[robot_ownership_model]] §4). route 가 robot 을 주면
  // 그것을, 아니면 후보가 정확히 1개일 때만 그 robot 을, 그 외엔 null(→ Select
  // Robot). 생성 순간 1회 계산해 params 에 박고, 이후 패널은 자기 params 만 본다.
  const initialRobotId = useCallback((): string | null => {
    if (id) return id;
    if (robots.length === 1) return robots[0].id;
    return null;
  }, [id, robots]);

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

      for (const p of panels) {
        if (x + p.width > containerWidth - MARGIN && x > MARGIN) {
          x = MARGIN;
          y += rowHeight + GAP_Y;
          rowHeight = 0;
        }
        const owned = ROBOT_OWNED_PANELS.has(p.component);
        event.api.addPanel({
          id: p.id,
          component: p.component,
          title: p.title,
          floating: { x, y, width: p.width, height: p.height },
          // robot-owned 패널만 robot params + robot 셀렉터 탭. task 패널은 carve-out.
          params: owned ? { robotId: initialRobotId() } : {},
          tabComponent: owned ? "robot" : undefined,
        });
        x += p.width + GAP_X;
        rowHeight = Math.max(rowHeight, p.height);
      }
    },
    [panels, initialRobotId],
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

  return (
    <>
      <div
        ref={containerRef}
        className="absolute inset-0 z-10 pointer-events-none workspace-dockview"
      >
        <DockviewReact
          className="dockview-theme-dark"
          components={DOCKVIEW_PANEL_COMPONENTS}
          tabComponents={{ robot: RobotTab }}
          defaultTabComponent={LockedTab}
          onReady={onReady}
        />
      </div>

      {/* mode 별 layout reset — meta box 옆 자리 */}
      <button
        onClick={handleReset}
        title="이 모드 패널 레이아웃 초기화"
        className="absolute top-3 right-[180px] z-20 flex items-center gap-1.5 px-2 py-1 rounded bg-zinc-900/80 hover:bg-zinc-800 border border-zinc-700/60 text-zinc-400 hover:text-zinc-100 text-[10px] font-mono pointer-events-auto transition-colors"
      >
        <RotateCcw className="w-3 h-3" />
        Reset layout
      </button>
    </>
  );
}
