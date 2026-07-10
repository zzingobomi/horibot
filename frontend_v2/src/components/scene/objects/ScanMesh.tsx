/**
 * ScanMesh — reconstruction 결과 mesh (.ply) 뷰어. 씬 객체 (월드 산출물 —
 * mesh 는 ScanPanel 닫아도 남는 세션 상태, scanStore gate).
 *
 * ScanPanel 이 GET_MESH 로 받은 ply bytes 를 scanStore 에 넣으면 여기서 PLYLoader
 * 로 parse + render. mesh 정점은 robot base frame (build 가 base 기준 TSDF) →
 * <RobotFrame> 부모 transform 로 배치. 대상 robot = focus ?? 첫 robot
 * (옛 Container 의 scanBaseMatrix 계산 이동).
 */
import { useMemo } from "react";
import * as THREE from "three";
import { PLYLoader } from "three/examples/jsm/loaders/PLYLoader.js";
import { useScanStore } from "@/stores/scanStore";
import type { SceneObjectProps } from "../sceneTypes";
import { RobotFrame } from "../shared/RobotFrame";

export function ScanMesh({ robots, focusId }: SceneObjectProps) {
  const ply = useScanStore((s) => s.meshPly);
  const robotId = focusId ?? robots[0]?.id ?? "";

  const geometry = useMemo(() => {
    if (!ply) return null;
    // Uint8Array → 정확히 tight ArrayBuffer (msgpack view offset 대비 slice).
    const ab = ply.buffer.slice(ply.byteOffset, ply.byteOffset + ply.byteLength);
    const g = new PLYLoader().parse(ab as ArrayBuffer);
    if (!g.getAttribute("normal")) g.computeVertexNormals();
    return g;
  }, [ply]);

  const hasColor = geometry?.getAttribute("color") != null;

  if (!geometry || !robotId) return null;

  return (
    <RobotFrame robotId={robotId}>
      <mesh>
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
    </RobotFrame>
  );
}
