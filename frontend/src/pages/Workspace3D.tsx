import { useCallback, useMemo } from "react";
import * as THREE from "three";
import { DockviewReact, type DockviewReadyEvent } from "dockview";
import { RobotScene } from "@/components/workspace3d/3d/RobotScene";
import { useCalibrationResults } from "@/hooks/useCalibrationResults";
import { useRobotStore } from "@/store/robotStore";
import { useSceneStore } from "@/store/sceneStore";
import { PANEL_COMPONENTS } from "@/components/workspace3d/dockview/panelComponents";

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

  // TODO: panel 들 위치 하드코딩 말고 자동으로 배치하는 로직 필요
  const onReady = useCallback((event: DockviewReadyEvent) => {
    const sidebarWidth = 207;
    const pointCloudWidth = 260;
    const pointCloudX = Math.max(
      16,
      window.innerWidth - pointCloudWidth - 16 - sidebarWidth
    );
    event.api.addPanel({
      id: "robot-state",
      component: "robotState",
      title: "Robot State",
      floating: { x: 16, y: 16, width: 260, height: 270 },
      params: {},
    });
    event.api.addPanel({
      id: "scene-controls",
      component: "sceneControls",
      title: "Scene Controls",
      floating: { x: 16, y: 304, width: 260, height: 300 },
      params: {},
    });
    event.api.addPanel({
      id: "calibration",
      component: "calibration",
      title: "Calibration",
      floating: { x: 16, y: 622, width: 260, height: 260 },
      params: {},
    });
    event.api.addPanel({
      id: "point-cloud",
      component: "pointCloud",
      title: "Point Cloud",
      floating: { x: pointCloudX, y: 16, width: pointCloudWidth, height: 240 },
      params: {},
    });
  }, []);

  return (
    <div
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
    </div>
  );
}
