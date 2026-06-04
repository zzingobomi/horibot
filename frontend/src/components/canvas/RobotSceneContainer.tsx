import { useCallback, useMemo } from "react";
import * as THREE from "three";
import { RobotScene } from "@/components/canvas/3d/RobotScene";
import { useCalibrationResults } from "@/hooks/useCalibrationResults";
import { useRobotStore } from "@/store/robotStore";
import { useSceneStore } from "@/store/sceneStore";
import { useRobots } from "@/hooks/useRobots";

interface RobotSceneContainerProps {
  /** focus robot id. null=WorldPage(모두 동등). undefined=default(backend default robot). */
  focusId?: string | null;
}

export function RobotSceneContainer({ focusId }: RobotSceneContainerProps = {}) {
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

  const { robots, defaultId } = useRobots();
  // focusId undefined = default. null = WorldPage. string = 명시.
  const effectiveFocus: string | null =
    focusId === undefined ? defaultId : focusId;

  // focus robot 의 base_pose 로 OrbitControls target 잡음 (lookAt 효과).
  // WorldPage(focusId=null) 에선 모든 robot 의 중심.
  const cameraTarget = useMemo<[number, number, number]>(() => {
    if (effectiveFocus !== null) {
      const r = robots.find((x) => x.id === effectiveFocus);
      if (r) return [r.base_pose.x, r.base_pose.y, r.base_pose.z + 0.1];
    }
    if (robots.length === 0) return [0, 0.1, 0];
    const cx = robots.reduce((s, r) => s + r.base_pose.x, 0) / robots.length;
    const cy = robots.reduce((s, r) => s + r.base_pose.y, 0) / robots.length;
    const cz = robots.reduce((s, r) => s + r.base_pose.z, 0) / robots.length;
    return [cx, cy, cz + 0.1];
  }, [robots, effectiveFocus]);

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
      robots={robots}
      focusId={effectiveFocus}
      cameraTarget={cameraTarget}
    />
  );
}
