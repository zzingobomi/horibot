import { useEffect, useMemo } from "react";
import * as THREE from "three";
import { useScene3DStore, type PointCloudFrame } from "@/domain/stores/scene3D";

// Point size store value (1~8 slider) → R3F pointsMaterial.size 세계 단위 변환.
// 시각 옵션 — sizeAttenuation true 라 worldspace m. 0.001m ≈ 1px 정도 (카메라 거리 변동).
const POINT_SIZE_SCALE = 0.0015;

function useGeometry(frame: PointCloudFrame | null) {
  const geometry = useMemo(() => {
    if (!frame || frame.count === 0) return null;
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.BufferAttribute(frame.positions, 3));
    g.setAttribute("color", new THREE.BufferAttribute(frame.colors, 3, true));
    g.computeBoundingSphere();
    return g;
  }, [frame]);

  useEffect(() => {
    return () => {
      geometry?.dispose();
    };
  }, [geometry]);

  return geometry;
}

interface LiveProps {
  cameraMatrix: THREE.Matrix4 | null;
}

/** 라이브 RGBD point cloud 자리. scene3DStore.enabled 토글로 보임/숨김. */
export function Scene3DLayer({ cameraMatrix }: LiveProps) {
  const enabled = useScene3DStore((s) => s.enabled);
  const frame = useScene3DStore((s) => s.frame);
  const pointSize = useScene3DStore((s) => s.pointSize);
  const geometry = useGeometry(frame);

  if (!enabled || !cameraMatrix || !geometry) return null;

  const position = new THREE.Vector3().setFromMatrixPosition(cameraMatrix);
  const quaternion = new THREE.Quaternion().setFromRotationMatrix(cameraMatrix);

  return (
    <group position={position.toArray()} quaternion={quaternion}>
      <points geometry={geometry}>
        <pointsMaterial
          size={pointSize * POINT_SIZE_SCALE}
          sizeAttenuation
          vertexColors
        />
      </points>
    </group>
  );
}
