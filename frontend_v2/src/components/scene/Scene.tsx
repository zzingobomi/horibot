/**
 * R3F Canvas + multi-robot URDF + base/TCP axis frame.
 *
 * frontend_v2 first cut — 옛 frontend Scene.tsx 의 simplified carry over:
 *   - 박지 X: Scene3DLayer / DetectionLayer / TaskResultLayer / RobotPreviewLayer /
 *     CameraFrustum / handEyeMatrix / calibration (Step E+ 박힐 때 carry over)
 *   - 박음: lights / Grid / RobotLayer / AxisFrame (BASE + TCP) / OrbitControls
 */
import { useCallback, useMemo } from "react";
import { Canvas } from "@react-three/fiber";
import { OrbitControls, Grid, Environment } from "@react-three/drei";
import * as THREE from "three";
import { RobotLayer } from "./RobotLayer";
import { AxisFrame } from "./AxisFrame";
import { Scene3DLayer } from "./Scene3DLayer";
import { MeshLayer } from "./MeshLayer";
import { DEFAULT_SCENE_OPTIONS, type SceneOptions } from "./sceneOptions";
import type { RobotInfo } from "@/api/generated/contract";

interface RobotSceneProps {
  /** focus robot 의 arm joint name list (backend TcpState.joint_names SSOT). */
  jointNames: string[];
  jointAngles: number[];
  options?: SceneOptions;
  linkVisibility?: Record<string, boolean>;
  onLinksLoaded?: (names: string[]) => void;
  /** focus robot 의 TCP pose (R3F y-up world frame). source = Motion.Stream.TCP_STATE. */
  tcpMatrix?: THREE.Matrix4 | null;
  robots: RobotInfo[];
  focusId?: string | null;
  cameraPosition?: [number, number, number];
  cameraTarget?: [number, number, number];
  /** focus robot base transform (z-up→y-up + base_pose) — scan mesh 배치용. */
  robotBaseMatrix?: THREE.Matrix4 | null;
  /** focus robot id — scan live cloud / hand_eye fetch 용. */
  robotId?: string;
}

function SceneContent({
  jointNames,
  jointAngles,
  options = DEFAULT_SCENE_OPTIONS,
  onLinksLoaded,
  tcpMatrix = null,
  robots,
  focusId,
  cameraTarget,
  robotBaseMatrix = null,
  robotId = "",
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

      <RobotLayer
        robots={robots}
        focusId={focusId ?? null}
        jointNames={jointNames}
        jointAngles={jointAngles}
        onLinksLoaded={onLinksLoaded}
        showRobot={options.showRobot}
      />

      {options.showTCPFrame && tcpMatrix && (
        <AxisFrame
          matrix={tcpMatrix}
          size={0.04}
          label="TCP"
          labelColor="#ffcc44"
        />
      )}

      {/* scan — 라이브 PC (scanStore.liveEnabled gate) + reconstruction mesh */}
      {robotId && <Scene3DLayer tcpMatrix={tcpMatrix} robotId={robotId} />}
      <MeshLayer robotBaseMatrix={robotBaseMatrix} />

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
