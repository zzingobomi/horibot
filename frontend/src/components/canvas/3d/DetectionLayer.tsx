import { useMemo } from "react";
import * as THREE from "three";
import { Html, Line } from "@react-three/drei";
import { useDetectorStore } from "@/store/detectorStore";

/**
 * Grounded detection 결과의 3D 시각화.
 *
 * groundedResult.position 은 base frame 좌표. 이 컴포넌트는
 * RobotScene 의 base-frame group(rotation [-π/2, 0, 0]) 안에서 마운트되어야 함
 * — base 좌표를 그대로 박으면 z-up → y-up 변환은 부모 group 이 처리.
 *
 * store 가 null 이면 자동으로 안 보임 → 다른 페이지에 영향 없음.
 */
export function DetectionLayer() {
  const result = useDetectorStore((s) => s.groundedResult);

  const guidePoints = useMemo<[number, number, number][] | null>(() => {
    if (!result) return null;
    const [, , z] = result.position;
    // 마커 → 바닥(local z=0)
    return [
      [0, 0, 0],
      [0, 0, -z],
    ];
  }, [result]);

  if (!result || !guidePoints) return null;

  const [x, y, z] = result.position;

  return (
    <group position={new THREE.Vector3(x, y, z)}>
      {/* 타겟 마커 — 작은 구 */}
      <mesh>
        <sphereGeometry args={[0.008, 16, 16]} />
        <meshStandardMaterial
          color="#ff3366"
          emissive="#ff3366"
          emissiveIntensity={0.4}
          roughness={0.3}
        />
      </mesh>

      {/* 바닥(z=0)까지 가이드 라인 */}
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

      {/* prompt + confidence 라벨 */}
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
