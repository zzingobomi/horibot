import { useEffect, useState } from "react";
import * as THREE from "three";
import { PLYLoader } from "three/examples/jsm/loaders/PLYLoader.js";
import { usePointCloudStore } from "@/store/pointCloudStore";
import { BASE_URL } from "@/constants";

/**
 * TSDF mesh. base 프레임 PLY를 그대로 마운트.
 *
 * 부모(RobotScene)에서 `<group rotation={[-π/2, 0, 0]}>` 안에 박혀야 base z-up이
 * three.js y-up과 일치. 이 컴포넌트는 추가 transform 없음.
 */
export function MeshLayer() {
  const meshVisible = usePointCloudStore((s) => s.meshVisible);
  const meshPath = usePointCloudStore((s) => s.meshPath);

  const [geometry, setGeometry] = useState<THREE.BufferGeometry | null>(null);

  useEffect(() => {
    if (!meshPath) return;

    let cancelled = false;
    const loader = new PLYLoader();
    const url = `${BASE_URL}/robot/${meshPath}`;
    loader.load(
      url,
      (geo) => {
        if (cancelled) {
          geo.dispose();
          return;
        }
        geo.computeVertexNormals();
        setGeometry((prev) => {
          prev?.dispose();
          return geo;
        });
      },
      undefined,
      (err) => {
        console.error("[MeshLayer] PLY load 실패:", err);
      }
    );
    return () => {
      cancelled = true;
    };
  }, [meshPath]);

  // 컴포넌트 unmount 시 geometry GPU 메모리 해제
  useEffect(() => {
    return () => {
      geometry?.dispose();
    };
  }, [geometry]);

  // meshPath가 null이면 이전 geometry가 stale 상태로 남아있을 수 있어 render 게이트
  if (!meshVisible || !meshPath || !geometry) return null;

  // mesh가 vertex color를 갖고 있으면 그대로, 아니면 흰색 fallback
  const hasColor = geometry.hasAttribute("color");

  return (
    <mesh geometry={geometry}>
      <meshStandardMaterial
        vertexColors={hasColor}
        color={hasColor ? "white" : "#cccccc"}
        side={THREE.DoubleSide}
        roughness={0.85}
        metalness={0.05}
      />
    </mesh>
  );
}
