/**
 * R3F Canvas — 씬의 조립 지점 (소유권 모델, [docs/frontend.md]).
 *
 * - Core chrome  : 조명/grid/BASE 축/OrbitControls — 씬 그 자체.
 * - Scene object : 세계에 있는 것들 — Robot / Camera / ScanMesh. 각 객체가 자기
 *                  시각 요소를 자기 안에서 그림 (패널은 속성 토글만). 새 객체
 *                  종류 추가 = 드문 아키텍처 사건 — 여기 한 줄이 정직함.
 * - Feature overlay: 기능이 보여주는 것 — TaskMarkersOverlay(topic 수명) +
 *                  ScenePartHost(패널 수명). **기능/패널 기여로는 이 파일 diff 0.**
 *
 * robot 상태(joint/TCP frame)는 Robots 가 robot 마다 자기 stream 구독
 * (per-robot — N=2 협동 자리).
 */
import { useCallback, useMemo } from "react";
import { Canvas } from "@react-three/fiber";
import { OrbitControls, Grid, Environment } from "@react-three/drei";
import * as THREE from "three";
import { Robots } from "./objects/Robots";
import { Cameras } from "./objects/Cameras";
import { ScanMesh } from "./objects/ScanMesh";
import { AxisFrame } from "./shared/AxisFrame";
import { TaskMarkersOverlay } from "./overlays/TaskMarkersOverlay";
import { ScenePartHost } from "./overlays/ScenePartHost";
import { DEFAULT_SCENE_OPTIONS, type SceneOptions } from "./sceneOptions";
import type { RobotInfo } from "@/api/generated/contract";

interface RobotSceneProps {
  options?: SceneOptions;
  linkVisibility?: Record<string, boolean>;
  onLinksLoaded?: (names: string[]) => void;
  robots: RobotInfo[];
  focusId?: string | null;
  cameraPosition?: [number, number, number];
  cameraTarget?: [number, number, number];
}

function SceneContent({
  options = DEFAULT_SCENE_OPTIONS,
  onLinksLoaded,
  robots,
  focusId,
  cameraTarget,
}: RobotSceneProps) {
  return (
    <>
      <ambientLight intensity={0.4} color="#b0c8e0" />
      <directionalLight
        position={[0.5, 1, 0.5]}
        intensity={1.2}
        color="#ffffff"
        castShadow
        shadow-mapSize={[1024, 1024]}
      />
      <directionalLight
        position={[-0.5, 0.2, -0.5]}
        intensity={0.3}
        color="#6699bb"
      />
      <Environment preset="city" />

      {options.showGrid && (
        <Grid
          args={[0.6, 0.6]}
          cellSize={0.05}
          cellThickness={0.5}
          cellColor="#1a3a5a"
          sectionSize={0.1}
          sectionThickness={1}
          sectionColor="#2a5a8a"
          fadeDistance={1.5}
          fadeStrength={1}
          followCamera={false}
          position={[0, 0, 0]}
        />
      )}

      {/* z-up → y-up 변환 */}
      {options.showBaseFrame && (
        <group rotation={[-Math.PI / 2, 0, 0]}>
          <AxisFrame size={0.06} label="BASE" labelColor="#ffffff" />
        </group>
      )}

      {/* ── Scene objects — 세계에 있는 것들 (자기가 자기를 그림) ── */}
      <Robots
        robots={robots}
        focusId={focusId ?? null}
        onLinksLoaded={onLinksLoaded}
        showRobot={options.showRobot}
        showTcpFrame={options.showTCPFrame}
      />
      {/* rgbd robot 파생 카메라 — frustum(cameraStore)/live cloud(scanStore) gate */}
      <Cameras robots={robots} focusId={focusId ?? null} />
      <ScanMesh robots={robots} focusId={focusId ?? null} />

      {/* ── Feature overlays — 기능이 보여주는 것 ── */}
      <TaskMarkersOverlay robots={robots} focusId={focusId ?? null} />
      <ScenePartHost />

      <OrbitControls
        makeDefault
        enableDamping
        dampingFactor={0.08}
        minDistance={0.1}
        maxDistance={3}
        target={cameraTarget ?? [0, 0.1, 0]}
      />
    </>
  );
}

// stable refs — Canvas 의 prop 객체가 매 render 새 ref 면 R3F reconciler cycle.
const GL_OPTS = { antialias: true, alpha: false };
const STYLE = { background: "#080c12" };
const NEAR_FAR = { fov: 45, near: 0.001, far: 10 };

export function RobotScene(props: RobotSceneProps) {
  const camPos = useMemo<[number, number, number]>(
    () =>
      props.cameraPosition ??
      (props.focusId === null ? [0.7, 0.6, 0.7] : [0.35, 0.35, 0.35]),
    [props.cameraPosition, props.focusId],
  );
  const cameraOpts = useMemo(
    () => ({ ...NEAR_FAR, position: camPos }),
    [camPos],
  );
  const onCreated = useCallback(({ gl }: { gl: THREE.WebGLRenderer }) => {
    gl.shadowMap.enabled = true;
    gl.shadowMap.type = THREE.PCFShadowMap;
  }, []);
  return (
    <Canvas
      camera={cameraOpts}
      gl={GL_OPTS}
      onCreated={onCreated}
      style={STYLE}
    >
      <SceneContent {...props} />
    </Canvas>
  );
}
