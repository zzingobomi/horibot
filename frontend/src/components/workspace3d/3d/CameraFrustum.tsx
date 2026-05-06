// components/three/CameraFrustum.tsx
import { useMemo } from "react";
import * as THREE from "three";
import { Line } from "@react-three/drei";
import type { IntrinsicData } from "@/hooks/useCalibrationResults";
import type { Vec3 } from "@/types/motion";

interface CameraFrustumProps {
  intrinsic: IntrinsicData;
  /** Frustum depth in meters */
  depth?: number;
  color?: string;
}

/**
 * Draws a camera frustum wireframe based on intrinsic camera_matrix.
 * Assumes image_size from intrinsic data or falls back to 640x480.
 */
export function CameraFrustum({
  intrinsic,
  depth = 0.15,
  color = "#00e5ff",
}: CameraFrustumProps) {
  const lines = useMemo(() => {
    const K = intrinsic.camera_matrix;
    const [w, h] = intrinsic.image_size ?? [640, 480];
    const fx = K[0][0];
    const fy = K[1][1];
    const cx = K[0][2];
    const cy = K[1][2];

    // Corners in camera space at depth d
    const corners = [
      [(0 - cx) / fx, (0 - cy) / fy, 1],
      [(w - cx) / fx, (0 - cy) / fy, 1],
      [(w - cx) / fx, (h - cy) / fy, 1],
      [(0 - cx) / fx, (h - cy) / fy, 1],
    ].map(([x, y, z]) => [x * depth, y * depth, z * depth] as Vec3);

    const o: Vec3 = [0, 0, 0];

    return [
      // From origin to each corner
      [o, corners[0]],
      [o, corners[1]],
      [o, corners[2]],
      [o, corners[3]],
      // Rectangle at far plane
      [corners[0], corners[1]],
      [corners[1], corners[2]],
      [corners[2], corners[3]],
      [corners[3], corners[0]],
    ] as Array<[Vec3, Vec3]>;
  }, [intrinsic, depth]);

  return (
    <group>
      {lines.map((pts, i) => (
        <Line
          key={i}
          points={pts}
          color={color}
          lineWidth={1.2}
          transparent
          opacity={0.7}
        />
      ))}
      {/* Sensor plane marker */}
      <mesh position={[0, 0, 0.005]}>
        <planeGeometry args={[0.015, 0.01]} />
        <meshBasicMaterial
          color={color}
          transparent
          opacity={0.25}
          side={THREE.DoubleSide}
        />
      </mesh>
    </group>
  );
}
