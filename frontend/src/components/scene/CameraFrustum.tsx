import { useMemo } from "react";
import * as THREE from "three";
import { Line } from "@react-three/drei";
import type { IntrinsicData } from "@/types/calibration";
import type { Vec3 } from "@/types/motion";

interface CameraFrustumProps {
  intrinsic: IntrinsicData;
  nearPlane?: number;
  farPlane?: number;
  color?: string;
}

// Intel RealSense D405 specs
// - 해상도: 1280×720 (matched RGB & Depth)
// - FOV: H 87°, V 58°
// - 작동 범위: 7 cm ~ 50 cm (short-range stereo)
const D405_DEFAULT_IMAGE_SIZE: [number, number] = [1280, 720];
const D405_NEAR_PLANE_M = 0.07;
const D405_FAR_PLANE_M = 0.5;

export function CameraFrustum({
  intrinsic,
  nearPlane = D405_NEAR_PLANE_M,
  farPlane = D405_FAR_PLANE_M,
  color = "#00e5ff",
}: CameraFrustumProps) {
  const { nearCorners, farCorners, lines } = useMemo(() => {
    const K = intrinsic.camera_matrix;
    const [w, h] = intrinsic.image_size ?? D405_DEFAULT_IMAGE_SIZE;
    const fx = K[0][0];
    const fy = K[1][1];
    const cx = K[0][2];
    const cy = K[1][2];

    const rays: Vec3[] = [
      [(0 - cx) / fx, (0 - cy) / fy, 1], // top-left
      [(w - cx) / fx, (0 - cy) / fy, 1], // top-right
      [(w - cx) / fx, (h - cy) / fy, 1], // bottom-right
      [(0 - cx) / fx, (h - cy) / fy, 1], // bottom-left
    ];

    const nearCorners = rays.map(
      ([x, y, z]) => [x * nearPlane, y * nearPlane, z * nearPlane] as Vec3,
    );
    const farCorners = rays.map(
      ([x, y, z]) => [x * farPlane, y * farPlane, z * farPlane] as Vec3,
    );

    const lines: Array<[Vec3, Vec3]> = [
      // 광축 원점 → far corners (전체 frustum 윤곽)
      [[0, 0, 0], farCorners[0]],
      [[0, 0, 0], farCorners[1]],
      [[0, 0, 0], farCorners[2]],
      [[0, 0, 0], farCorners[3]],
      // far plane 사각형
      [farCorners[0], farCorners[1]],
      [farCorners[1], farCorners[2]],
      [farCorners[2], farCorners[3]],
      [farCorners[3], farCorners[0]],
      // near plane 사각형
      [nearCorners[0], nearCorners[1]],
      [nearCorners[1], nearCorners[2]],
      [nearCorners[2], nearCorners[3]],
      [nearCorners[3], nearCorners[0]],
    ];

    return { nearCorners, farCorners, lines };
  }, [intrinsic, nearPlane, farPlane]);

  const nearPlaneGeometry = useMemo(() => {
    const geo = new THREE.BufferGeometry();
    const verts = new Float32Array([
      ...nearCorners[0],
      ...nearCorners[1],
      ...nearCorners[2],
      ...nearCorners[0],
      ...nearCorners[2],
      ...nearCorners[3],
    ]);
    geo.setAttribute("position", new THREE.BufferAttribute(verts, 3));
    geo.computeVertexNormals();
    return geo;
  }, [nearCorners]);

  const farPlaneGeometry = useMemo(() => {
    const geo = new THREE.BufferGeometry();
    const verts = new Float32Array([
      ...farCorners[0],
      ...farCorners[1],
      ...farCorners[2],
      ...farCorners[0],
      ...farCorners[2],
      ...farCorners[3],
    ]);
    geo.setAttribute("position", new THREE.BufferAttribute(verts, 3));
    geo.computeVertexNormals();
    return geo;
  }, [farCorners]);

  return (
    <group>
      {/* Frustum 윤곽선 */}
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

      {/* near plane */}
      <mesh geometry={nearPlaneGeometry}>
        <meshBasicMaterial
          color={color}
          transparent
          opacity={0.18}
          side={THREE.DoubleSide}
        />
      </mesh>

      {/* far plane */}
      <mesh geometry={farPlaneGeometry}>
        <meshBasicMaterial
          color={color}
          transparent
          opacity={0.06}
          side={THREE.DoubleSide}
        />
      </mesh>
    </group>
  );
}
