import { useCallback, useMemo, useRef } from "react";
import * as THREE from "three";
import { DockviewReact, type DockviewReadyEvent } from "dockview";
import { RotateCcw } from "lucide-react";
import { RobotScene } from "@/components/workspace3d/3d/RobotScene";
import { useCalibrationResults } from "@/hooks/useCalibrationResults";
import { useRobotStore } from "@/store/robotStore";
import { useSceneStore } from "@/store/sceneStore";
import {
  PANEL_COMPONENTS,
  type PanelComponentKey,
} from "@/components/workspace3d/dockview/panelComponents";
import {
  PANEL_HEADER_HEIGHT,
  loadCollapsed,
  loadLayout,
  resetWorkspaceLayout,
  saveLayout,
} from "@/lib/workspaceLayout";

export function Workspace3D() {
  const { results } = useCalibrationResults();

  const joints = useRobotStore((s) => s.joints);
  const jointAngles = useMemo<number[]>(() => {
    if (!joints?.length) return [0, 0, 0, 0, 0];
    return joints
      .filter((j) => j.id >= 1 && j.id <= 5)
      .sort((a, b) => a.id - b.id)
      .map((j) => {
        if (j.degree !== undefined) return (j.degree * Math.PI) / 180;
        if (j.position !== undefined)
          return ((j.position - 2048) / 4095) * 2 * Math.PI;
        return 0;
      });
  }, [joints]);

  const options = useSceneStore((s) => s.options);
  const linkVisibility = useSceneStore((s) => s.linkVisibility);
  const setLinkNames = useSceneStore((s) => s.setLinkNames);
  const setTcpPos = useSceneStore((s) => s.setTcpPos);

  const handleTCPMatrix = useCallback(
    (m: THREE.Matrix4 | null) => {
      if (!m) {
        setTcpPos(null);
        return;
      }
      const v = new THREE.Vector3().setFromMatrixPosition(m);
      setTcpPos([v.x, v.y, v.z]);
    },
    [setTcpPos]
  );

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
      { id: "robot-state", component: "robotState", title: "Robot State", width: 260, height: 270 },
      { id: "scene-controls", component: "sceneControls", title: "Scene Controls", width: 260, height: 300 },
      { id: "calibration", component: "calibration", title: "Calibration", width: 260, height: 260 },
      { id: "point-cloud", component: "pointCloud", title: "Point Cloud", width: 260, height: 240 },
    ],
    []
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
          // 줄바꿈
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
    [PANELS]
  );

  const onReady = useCallback(
    (event: DockviewReadyEvent) => {
      // 1) 저장된 layout 있으면 그걸로 복원, 없으면 default
      const saved = loadLayout();
      if (saved) {
        try {
          event.api.fromJSON(saved as Parameters<typeof event.api.fromJSON>[0]);
        } catch {
          // 저장본이 깨졌으면 무시하고 default
          addDefaultLayout(event);
        }
      } else {
        addDefaultLayout(event);
      }

      // 2) 사용자 인터랙션(드래그/리사이즈/추가/제거)마다 layout 저장 (debounce)
      let timer: ReturnType<typeof setTimeout> | null = null;
      event.api.onDidLayoutChange(() => {
        if (timer) clearTimeout(timer);
        timer = setTimeout(() => {
          saveLayout(event.api.toJSON());
        }, 300);
      });
    },
    [addDefaultLayout]
  );

  const handleReset = useCallback(() => {
    resetWorkspaceLayout();
    window.location.reload();
  }, []);

  return (
    <div
      ref={containerRef}
      className="relative w-full h-full overflow-hidden bg-[#080c12]"
      style={{ fontFamily: "'JetBrains Mono', 'Fira Code', monospace" }}
    >
      <div className="absolute inset-0 z-0">
        <RobotScene
          jointAngles={jointAngles}
          calibration={results}
          options={options}
          linkVisibility={linkVisibility}
          onLinksLoaded={setLinkNames}
          onTCPMatrix={handleTCPMatrix}
        />
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
