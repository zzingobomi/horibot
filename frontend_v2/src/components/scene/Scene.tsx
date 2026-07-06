/**
 * R3F Canvas + multi-robot URDF + base/TCP axis frame.
 *
 * robot 상태(joint/TCP frame)는 RobotLayer 가 robot 마다 자기 stream 구독
 * (per-robot — N=2 협동 자리). Scene 은 조명/grid/카메라 + layer 배치만.
 */
import { useCallback, useMemo } from "react";
import { Canvas } from "@react-three/fiber";
import { OrbitControls, Grid, Environment } from "@react-three/drei";
import * as THREE from "three";
import { RobotLayer } from "./RobotLayer";
import { AxisFrame } from "./AxisFrame";
import { Scene3DLayer } from "./Scene3DLayer";
import { TaskResultLayer } from "./TaskResultLayer";
import { MeshLayer } from "./MeshLayer";
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
  /** focus robot base transform (z-up→y-up + base_pose) — scan mesh 배치용. */
  robotBaseMatrix?: THREE.Matrix4 | null;
  /** focus robot id — scan live cloud (Scene3DLayer 가 stream/hand_eye 자체 구독). */
  robotId?: string;
}

function SceneContent({
  options = DEFAULT_SCENE_OPTIONS,
  onLinksLoaded,
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

      {/* joint + TCP frame — robot 마다 자기 TCP_STATE 구독 (per-robot) */}
      <RobotLayer
        robots={robots}
        focusId={focusId ?? null}
        onLinksLoaded={onLinksLoaded}
        showRobot={options.showRobot}
        showTcpFrame={options.showTCPFrame}
      />

      {/* scan — 라이브 PC (scanStore.liveEnabled gate) + reconstruction mesh */}
      {robotId && <Scene3DLayer robotId={robotId} />}
      {/* task step 결과 (검출 sphere / 목표점 marker) — cloud 와 겹쳐 오차 확인 */}
      {robotId && <TaskResultLayer robotId={robotId} />}
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
