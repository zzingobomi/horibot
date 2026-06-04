/**
 * /robots/:id — focus mode. multi_robot_phase2_frontend.md §2 sketch:
 * WorldScene (focus on robot) + 기존 panel (Robot State / Scene Controls /
 * Calibration / Point Cloud) dockview.
 *
 * (이전 Workspace3D 페이지의 dockview 인프라가 여기로 흡수됨 — 같은 N=1
 * default focus 가정의 중복 자리였음.)
 *
 * Layout 은 robot 별로 localStorage 분리 (`workspace3d.<id>`) — focus 마다
 * 사용자가 다르게 정렬할 수 있게. 만져보고 불필요하면 단일 키로 통합.
 *
 * 첫 프로토타입 scope — Page Preset / Layer registry / ViewState store /
 * 명령 권한 같은 추상화는 안 만듦. 만져보고 발견.
 */
import { useCallback, useRef } from "react";
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
import { useRobots } from "@/hooks/useRobots";

type PanelSpec = {
  id: string;
  component: PanelComponentKey;
  title: string;
  width: number;
  height: number;
};

// ====== STAGE 3 디버그 — panel 0개 ======
// dockview 컨테이너만 마운트, panel content 0개. 라우팅 되면 어느 panel component
// 가 unmount 막는 것, 안 되면 dockview 컨테이너 자체 (라이브러리) 가 막는 것.
const PANELS: PanelSpec[] = [
  { id: "robot-state", component: "robotState", title: "Robot State", width: 260, height: 270 },
  { id: "motion", component: "motion", title: "Motion", width: 320, height: 360 },
  { id: "scene-controls", component: "sceneControls", title: "Scene Controls", width: 260, height: 300 },
  { id: "calibration", component: "calibration", title: "Calibration", width: 260, height: 260 },
  { id: "calibration-actions", component: "calibrationActions", title: "Calibration Actions", width: 320, height: 360 },
  { id: "point-cloud", component: "pointCloud", title: "Point Cloud", width: 260, height: 240 },
];

export function RobotsPage() {
  const { id = "" } = useParams<{ id: string }>();
  const { robots, loading, error } = useRobots();
  const containerRef = useRef<HTMLDivElement | null>(null);
  const layoutKey = `workspace3d.${id}`;

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

  if (error) {
    return (
      <div className="p-6 text-red-400 font-mono">/robots 응답 실패: {error}</div>
    );
  }
  if (loading) {
    return <div className="p-6 text-zinc-400 font-mono">robots.yaml 로드 중...</div>;
  }

  const found = robots.find((r) => r.id === id);
  if (!found) {
    return (
      <div className="p-6 text-zinc-400 font-mono">
        robot id <span className="text-red-400">{id}</span> 없음. 등록된 robot:{" "}
        {robots.map((r) => r.id).join(", ") || "(없음)"}
      </div>
    );
  }

  return (
    <div
      ref={containerRef}
      className="relative w-full h-full overflow-hidden bg-[#080c12]"
      style={{ fontFamily: "'JetBrains Mono', 'Fira Code', monospace" }}
    >
      <div className="absolute inset-0 z-0">
        <RobotSceneContainer focusId={id} />
      </div>

      {/* dockview overlay — `pointer-events-none` 을 root 에 박지 않는다.
          dockview 가 자기 영역의 mouse/focus event 를 받아야 internal state
          machine (drag, focus, cleanup handshake) 이 정상 동작 → unmount 시
          cleanup 도 정상. 빈 dock 영역의 click 통과는 workspace-dockview.css
          의 .dv-dockview / .dv-groupview 에 pointer-events:none 으로 처리. */}
      <div className="absolute inset-0 z-10 workspace-dockview">
        <DockviewReact
          className="dockview-theme-dark"
          components={PANEL_COMPONENTS}
          onReady={onReady}
        />
      </div>

      {/* 우상단: robot 메타 + layout reset */}
      <div className="absolute top-3 right-3 z-20 flex items-center gap-2">
        <div className="px-3 py-2 rounded bg-zinc-900/80 border border-zinc-700/60 text-zinc-300 text-xs font-mono pointer-events-none">
          <div className="text-zinc-100 font-semibold">{found.id}</div>
          <div className="text-zinc-500">type: {found.type}</div>
          <div className={found.enabled ? "text-green-400" : "text-yellow-400"}>
            {found.enabled ? "enabled" : "viz-only"}
          </div>
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
