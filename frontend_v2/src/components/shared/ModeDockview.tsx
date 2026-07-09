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
 * 패널 관리 UI = AutoHideHeader ([docs/workspace_autohide_header.md]) — 패널 닫기
 * (탭 X) 와 `+ 패널 추가` 가 세트라 실수 복구 가능 (옛 hideClose + Reset 플로팅
 * 버튼 조합 대체. Reset 은 ⋯ 메뉴의 비상용으로 강등).
 *
 * 옛 frontend ModeDockview carry over (frontend_v2.md §2.3). 기능 추가 =
 * PANEL_COMPONENTS 등록 + mode 파일 PANELS 한 줄.
 */
import { useCallback, useRef, useState } from "react";
import { useParams } from "react-router-dom";
import {
  DockviewReact,
  type DockviewApi,
  type DockviewReadyEvent,
} from "dockview";
import {
  DOCKVIEW_PANEL_COMPONENTS,
  PANEL_CATALOG,
  ROBOT_OWNED_PANELS,
  type PanelComponentKey,
} from "@/components/panels/registry";
import { AutoHideHeader } from "@/components/shared/AutoHideHeader";
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

// `+ 패널 추가` 모집단 = 전체 카탈로그 (mode PANELS 는 default 세트일 뿐 — 사용자는
// 어느 페이지에서든 모든 종류를 추가 가능). id 는 `add-` prefix — mode default 의
// id("camera" 등, mode 마다 다른 component 를 가리킴)와 충돌 안 하게. 배치 여부
// 판정은 id 가 아니라 component 기준 (AutoHideHeader).
const CATALOG_SPECS: PanelSpec[] = (
  Object.entries(PANEL_CATALOG) as [
    PanelComponentKey,
    (typeof PANEL_CATALOG)[PanelComponentKey],
  ][]
).map(([key, meta]) => ({
  id: `add-${key}`,
  component: key,
  title: meta.title,
  width: meta.width,
  height: meta.height,
}));

export function ModeDockview({ mode, panels }: ModeDockviewProps) {
  const { id = "" } = useParams<{ id: string }>();
  const { robots } = useRobots();
  const containerRef = useRef<HTMLDivElement | null>(null);
  const [api, setApi] = useState<DockviewApi | null>(null);
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
      setApi(event.api);
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

  // AutoHideHeader `+ 패널 추가` — 닫힌 패널을 다시 배치 (addDefaultLayout 의
  // 단건 버전 + 기존 패널 수 기반 cascade 로 겹침 완화).
  const handleAddPanel = useCallback(
    (p: PanelSpec) => {
      if (!api) return;
      const owned = ROBOT_OWNED_PANELS.has(p.component);
      const n = api.panels.length;
      api.addPanel({
        id: p.id,
        component: p.component,
        title: p.title,
        floating: {
          x: 24 + (n % 5) * 28,
          y: 48 + (n % 5) * 20,
          width: p.width,
          height: p.height,
        },
        params: owned ? { robotId: initialRobotId() } : {},
        tabComponent: owned ? "robot" : undefined,
      });
    },
    [api, initialRobotId],
  );

  return (
    <>
      <div
        ref={containerRef}
        className="absolute inset-0 z-10 pointer-events-none workspace-dockview"
      >
        {/* key=layoutKey — robot 간 이동(:id param 변경)은 mode 전환과 달리 이
            컴포넌트를 remount 하지 않으므로, dockview 가 이전 robot 의 패널/바인딩을
            그대로 들고 감 (cross-page 공유처럼 보임). layout 키가 바뀌면 dockview 를
            remount 해 그 키의 저장 layout 을 새로 로드. */}
        <DockviewReact
          key={layoutKey}
          className="dockview-theme-dark"
          components={DOCKVIEW_PANEL_COMPONENTS}
          tabComponents={{ robot: RobotTab }}
          onReady={onReady}
        />
      </div>

      <AutoHideHeader
        api={api}
        candidates={CATALOG_SPECS}
        onAddPanel={handleAddPanel}
        onResetLayout={handleReset}
      />
    </>
  );
}
