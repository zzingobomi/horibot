import { useCallback, useMemo } from "react";
import * as THREE from "three";
import { RobotScene } from "@/components/canvas/3d/RobotScene";
import { useCalibrationResults } from "@/hooks/useCalibrationResults";
import { useRobotStore } from "@/store/robotStore";
import { useSceneStore } from "@/store/sceneStore";

export function RobotSceneContainer() {
  const { results } = useCalibrationResults();

  const joints = useRobotStore((s) => s.joints);
  const jointOffsetsRad = useRobotStore((s) => s.jointOffsetsRad);
  const jointAngles = useMemo<number[]>(() => {
    if (!joints?.length) return [0, 0, 0, 0, 0, 0];
    return joints
      .filter((j) => j.id >= 1 && j.id <= 6)
      .sort((a, b) => a.id - b.id)
      .map((j) => {
        // 백엔드 JointStateCache와 동일한 보정: raw_to_rad + joint_offset.
        // 캘 안 한 환경에서는 offset이 0이라 기존 동작 그대로.
        const baseRad =
          j.degree !== undefined
            ? (j.degree * Math.PI) / 180
            : j.position !== undefined
            ? ((j.position - 2048) / 4095) * 2 * Math.PI
            : 0;
        return baseRad + (jointOffsetsRad[j.id] ?? 0);
      });
  }, [joints, jointOffsetsRad]);

  const options = useSceneStore((s) => s.options);
  const linkVisibility = useSceneStore((s) => s.linkVisibility);
  const setLinkNames = useSceneStore((s) => s.setLinkNames);
  const setTcpPos = useSceneStore((s) => s.setTcpPos);

  const handleTCPMatrix = useCallback(
    (m: THREE.Matrix4 | null) => {
      if (!m) {
        setTcpPos(null);
        return;
      }
      const v = new THREE.Vector3().setFromMatrixPosition(m);
      setTcpPos([v.x, v.y, v.z]);
    },
    [setTcpPos]
  );

  return (
    <RobotScene
      jointAngles={jointAngles}
      calibration={results}
      options={options}
      linkVisibility={linkVisibility}
      onLinksLoaded={setLinkNames}
      onTCPMatrix={handleTCPMatrix}
    />
  );
}
