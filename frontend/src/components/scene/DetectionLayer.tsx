import { useMemo } from "react";
import * as THREE from "three";
import { Html, Line } from "@react-three/drei";
import { useTopic } from "@/framework";
import { Topic } from "@/constants/topics";
import { useDetectorOverride } from "@/domain/stores/detector";

/**
 * Grounded detection 결과의 3D 시각화.
 *
 * groundedResult.position 은 base frame 좌표. 이 컴포넌트는 RobotScene 의
 * base-frame group(rotation [-π/2, 0, 0]) 안에서 마운트되어야 함 — base 좌표를
 * 그대로 박으면 z-up → y-up 변환은 부모 group 이 처리.
 *
 * `useDetectorOverride.maskBefore` 가 frontend-local hide. PromptPanel 이 새
 * task 시작 시 mask = now 로 갱신 → 이전 결과 가림. backend 의 새 publish 가
 * 도착하면 timestamp 가 더 크니 자동 노출.
 */
export function DetectionLayer() {
  const result = useTopic(Topic.PERCEPTION_GROUNDED_STATE);
  const maskBefore = useDetectorOverride((s) => s.maskBefore);

  const guidePoints = useMemo<[number, number, number][] | null>(() => {
    if (!result) return null;
    const [, , z] = result.position;
    return [
      [0, 0, 0],
      [0, 0, -z],
    ];
  }, [result]);

  if (!result || result.timestamp <= maskBefore || !guidePoints) return null;

  const [x, y, z] = result.position;

  return (
    <group position={new THREE.Vector3(x, y, z)}>
      <mesh>
        <sphereGeometry args={[0.008, 16, 16]} />
        <meshStandardMaterial
          color="#ff3366"
          emissive="#ff3366"
          emissiveIntensity={0.4}
          roughness={0.3}
        />
      </mesh>

      <Line
        points={guidePoints}
        color="#ff3366"
        lineWidth={1.2}
        transparent
        opacity={0.5}
        dashed
        dashSize={0.01}
        gapSize={0.005}
      />

      <Html
        position={[0, 0, 0.03]}
        center
        distanceFactor={0.3}
        style={{ pointerEvents: "none" }}
      >
        <div
          style={{
            fontFamily: "'JetBrains Mono', 'Fira Code', monospace",
            fontSize: 11,
            background: "rgba(255, 51, 102, 0.85)",
            color: "white",
            padding: "3px 8px",
            borderRadius: 4,
            whiteSpace: "nowrap",
            boxShadow: "0 2px 8px rgba(0,0,0,0.4)",
          }}
        >
          {result.prompt}
          <span style={{ opacity: 0.7, marginLeft: 6 }}>
            {(result.confidence * 100).toFixed(0)}%
          </span>
        </div>
      </Html>
    </group>
  );
}
