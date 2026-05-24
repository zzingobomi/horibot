import { useCallback, useMemo, useState } from "react";
import type { CalibrationResults } from "@/hooks/useCalibrationResults";
import { Canvas } from "@react-three/fiber";
import { OrbitControls, Grid, Environment } from "@react-three/drei";
import * as THREE from "three";
import { RobotModel } from "./RobotModel";
import { AxisFrame } from "./AxisFrame";
import { CameraFrustum } from "./CameraFrustum";
import { LivePointCloudLayer } from "./PointCloudLayer";
import { MeshLayer } from "./MeshLayer";
import { DetectionLayer } from "./DetectionLayer";
import { SelfPlayLayer } from "./SelfPlayLayer";

export interface SceneOptions {
  showRobot: boolean;
  showBaseFrame: boolean;
  showTCPFrame: boolean;
  showCameraFrame: boolean;
  showGrid: boolean;
}

interface RobotSceneProps {
  jointAngles: number[];
  calibration: CalibrationResults | null;
  options: SceneOptions;
  linkVisibility?: Record<string, boolean>;
  onLinksLoaded?: (names: string[]) => void;
  onTCPMatrix?: (m: THREE.Matrix4 | null) => void;
}

function buildMatrix4(R: number[][], t: number[][]): THREE.Matrix4 {
  const flat_t = t.flat();
  // prettier-ignore
  return new THREE.Matrix4().set(
    R[0][0], R[0][1], R[0][2], flat_t[0],
    R[1][0], R[1][1], R[1][2], flat_t[1],
    R[2][0], R[2][1], R[2][2], flat_t[2],
    0,       0,       0,       1
  );
}

function SceneContent({
  jointAngles,
  calibration,
  options,
  linkVisibility,
  onLinksLoaded,
  onTCPMatrix,
}: RobotSceneProps) {
  const [tcpMatrix, setTcpMatrix] = useState<THREE.Matrix4 | null>(null);

  const handleTCPMatrix = useCallback(
    (m: THREE.Matrix4) => {
      setTcpMatrix(m.clone());
      onTCPMatrix?.(m);
    },
    [onTCPMatrix]
  );

  // tool flange(wrist) 기준과 tool tip 기준(gripper 끝)이 있음
  // 현재 방식은 gripper tip 기준
  const handEyeMatrix = useMemo(() => {
    if (!calibration?.hand_eye?.R || !calibration?.hand_eye?.t) return null;
    return buildMatrix4(calibration.hand_eye.R, calibration.hand_eye.t);
  }, [calibration]);

  // TCP → Camera 변환 행렬
  const cameraMatrix = useMemo(() => {
    if (!tcpMatrix || !handEyeMatrix) return null;
    return tcpMatrix.clone().multiply(handEyeMatrix);
  }, [tcpMatrix, handEyeMatrix]);

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

      {/* group 에서 z-up을 y-up으로 변환 */}
      {options.showBaseFrame && (
        <group rotation={[-Math.PI / 2, 0, 0]}>
          <AxisFrame size={0.06} label="BASE" labelColor="#ffffff" />
        </group>
      )}

      {/* group 에서 z-up을 y-up으로 변환 */}
      <RobotModel
        jointAngles={jointAngles}
        onTCPMatrix={handleTCPMatrix}
        onLinksLoaded={onLinksLoaded}
        linkVisibility={linkVisibility}
        visible={options.showRobot}
      />

      {/* TCP 프레임은 RobotModel에서 계산되므로 변환하지 않음 */}
      {options.showTCPFrame && tcpMatrix && (
        <AxisFrame
          matrix={tcpMatrix}
          size={0.04}
          label="TCP"
          labelColor="#ffcc44"
        />
      )}

      {/* 이미지 픽셀 좌표가 u (x축, 오른쪽으로 증가), (y축, 아래로 증가) */}
      {/* OpenCV 개발자들이 생각한 건 이미지 좌표가 오른쪽 +x, 아래 +y인데, 3D 카메라 좌표계도 똑같이 맞추면 편하지 않을까? */}
      {/* OpenCV로 캘하면 → +Z forward, +X right, +Y down 기준의 R, t가 나옴 */}
      {/* TCP 좌표계를 기준으로 한 상대값이라 y-up 변환 불필요 */}
      {options.showCameraFrame && cameraMatrix && (
        <>
          <AxisFrame
            matrix={cameraMatrix}
            size={0.04}
            label="CAMERA"
            labelColor="#00e5ff"
          />
          {calibration?.intrinsic && (
            <group
              position={new THREE.Vector3()
                .setFromMatrixPosition(cameraMatrix)
                .toArray()}
              quaternion={new THREE.Quaternion().setFromRotationMatrix(
                cameraMatrix
              )}
            >
              <CameraFrustum intrinsic={calibration.intrinsic} />
            </group>
          )}
        </>
      )}

      <LivePointCloudLayer cameraMatrix={cameraMatrix} />

      {/* TSDF mesh — base 프레임 PLY. RobotModel과 동일하게 z-up→y-up 회전 적용 */}
      <group rotation={[-Math.PI / 2, 0, 0]}>
        <MeshLayer />
        {/* Grounded detection 타겟 (base 프레임). store null이면 자동 미렌더. */}
        <DetectionLayer />
        {/* Self-play attempt 마커 (target / spike). store 비면 자동 미렌더 → 다른 페이지 무영향. */}
        <SelfPlayLayer />
      </group>

      <OrbitControls
        makeDefault
        enableDamping
        dampingFactor={0.08}
        minDistance={0.1}
        maxDistance={2}
        target={[0, 0.1, 0]}
      />
    </>
  );
}

export function RobotScene(props: RobotSceneProps) {
  return (
    <Canvas
      camera={{ position: [0.35, 0.35, 0.35], fov: 45, near: 0.001, far: 10 }}
      gl={{ antialias: true, alpha: false }}
      onCreated={({ gl }) => {
        gl.shadowMap.enabled = true;
        gl.shadowMap.type = THREE.PCFShadowMap;
      }}
      style={{ background: "#080c12" }}
    >
      <SceneContent {...props} />
    </Canvas>
  );
}
