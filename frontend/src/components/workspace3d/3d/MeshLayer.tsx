import { useEffect, useState } from "react";
import * as THREE from "three";
import { PLYLoader } from "three/examples/jsm/loaders/PLYLoader.js";
import { usePointCloudStore } from "@/store/pointCloudStore";
import { BASE_URL } from "@/constants";

// TSDF 메시는 base frame(z-up)으로 저장 → 씬 y-up 정합을 위해 부모 group에 회전.
// (PointCloudLayer의 Snapshot과 동일 컨벤션)
export function MeshLayer() {
  const meshPath = usePointCloudStore((s) => s.meshPath);
  const meshVisible = usePointCloudStore((s) => s.meshVisible);
  const [geometry, setGeometry] = useState<THREE.BufferGeometry | null>(null);

  useEffect(() => {
    if (!meshPath) {
      // 비동기로 미뤄서 effect 동기 setState 회피 (react-hooks/set-state-in-effect)
      const t = setTimeout(() => setGeometry(null), 0);
      return () => clearTimeout(t);
    }
    let cancelled = false;
    const loader = new PLYLoader();
    // meshPath는 ROBOT_DIR 기준 상대경로 (e.g. "models/mesh_xxx.ply")
    // 백엔드 정적 mount는 /robot 이므로 prefix 추가
    const url = `${BASE_URL}/robot/${meshPath}`;
    loader.load(
      url,
      (geom) => {
        if (cancelled) {
          geom.dispose();
          return;
        }
        if (!geom.attributes.normal) geom.computeVertexNormals();
        setGeometry(geom);
      },
      undefined,
      (err) => {
        if (!cancelled) console.warn(`mesh load 실패 ${url}:`, err);
      }
    );
    return () => {
      cancelled = true;
    };
  }, [meshPath]);

  useEffect(() => {
    return () => {
      geometry?.dispose();
    };
  }, [geometry]);

  if (!meshVisible || !geometry) return null;

  const hasColor = !!geometry.attributes.color;

  return (
    <group rotation={[-Math.PI / 2, 0, 0]}>
      <mesh geometry={geometry}>
        <meshStandardMaterial
          vertexColors={hasColor}
          color={hasColor ? "#ffffff" : "#aab4c2"}
          roughness={0.85}
          metalness={0.0}
          side={THREE.DoubleSide}
          flatShading={false}
        />
      </mesh>
    </group>
  );
}
