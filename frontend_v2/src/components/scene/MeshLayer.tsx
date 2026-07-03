/**
 * MeshLayer — reconstruction 결과 mesh (.ply) 뷰어.
 *
 * ScanPanel 이 GET_MESH 로 받은 ply bytes 를 scanStore 에 넣으면 여기서 PLYLoader
 * 로 parse + render. mesh 정점은 robot base frame (build 가 base 기준 TSDF) →
 * robotBaseMatrix (z-up→y-up + base_pose) 부모 transform 로 배치.
 */
import { useMemo } from "react";
import * as THREE from "three";
import { PLYLoader } from "three/examples/jsm/loaders/PLYLoader.js";
import { useScanStore } from "@/stores/scanStore";

interface MeshLayerProps {
  robotBaseMatrix: THREE.Matrix4 | null;
}

export function MeshLayer({ robotBaseMatrix }: MeshLayerProps) {
  const ply = useScanStore((s) => s.meshPly);

  const geometry = useMemo(() => {
    if (!ply) return null;
    // Uint8Array → 정확히 tight ArrayBuffer (msgpack view offset 대비 slice).
    const ab = ply.buffer.slice(ply.byteOffset, ply.byteOffset + ply.byteLength);
    const g = new PLYLoader().parse(ab as ArrayBuffer);
    if (!g.getAttribute("normal")) g.computeVertexNormals();
    return g;
  }, [ply]);

  const transform = useMemo(() => {
    if (!robotBaseMatrix) return null;
    const p = new THREE.Vector3();
    const q = new THREE.Quaternion();
    const s = new THREE.Vector3();
    robotBaseMatrix.decompose(p, q, s);
    return { position: [p.x, p.y, p.z] as const, quaternion: [q.x, q.y, q.z, q.w] as const };
  }, [robotBaseMatrix]);

  const hasColor = geometry?.getAttribute("color") != null;

  if (!geometry || !transform) return null;

  return (
    <mesh
      position={[transform.position[0], transform.position[1], transform.position[2]]}
      quaternion={[
        transform.quaternion[0],
        transform.quaternion[1],
        transform.quaternion[2],
        transform.quaternion[3],
      ]}
    >
      <primitive object={geometry} attach="geometry" />
      <meshStandardMaterial
        vertexColors={hasColor}
        color={hasColor ? "#ffffff" : "#88aacc"}
        roughness={0.7}
        metalness={0.0}
        side={THREE.DoubleSide}
        flatShading={false}
      />
    </mesh>
  );
}
