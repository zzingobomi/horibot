import { useEffect, useMemo } from "react";
import * as THREE from "three";
import {
  usePointCloudStore,
  type PointCloudFrame,
} from "@/store/pointCloudStore";

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
  pointSize?: number;
}

export function LivePointCloudLayer({
  cameraMatrix,
  pointSize = 0.003,
}: LiveProps) {
  const enabled = usePointCloudStore((s) => s.enabled);
  const frame = usePointCloudStore((s) => s.frame);
  const geometry = useGeometry(frame);

  if (!enabled || !cameraMatrix || !geometry) return null;

  const position = new THREE.Vector3().setFromMatrixPosition(cameraMatrix);
  const quaternion = new THREE.Quaternion().setFromRotationMatrix(cameraMatrix);

  return (
    <group position={position.toArray()} quaternion={quaternion}>
      <points geometry={geometry}>
        <pointsMaterial size={pointSize} sizeAttenuation vertexColors />
      </points>
    </group>
  );
}

interface SnapshotProps {
  pointSize?: number;
}

// snapshot은 백엔드에서 이미 base frame으로 변환된 상태로 발행 → 추가 transform 불필요
export function SnapshotPointCloudLayer({
  pointSize = 0.003,
}: SnapshotProps) {
  const snapshot = usePointCloudStore((s) => s.snapshot);
  const geometry = useGeometry(snapshot);

  if (!geometry) return null;

  return (
    <points geometry={geometry}>
      <pointsMaterial size={pointSize} sizeAttenuation vertexColors />
    </points>
  );
}
