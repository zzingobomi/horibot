import { useEffect, useRef } from "react";
import * as THREE from "three";
import URDFLoader from "urdf-loader";
import type { URDFRobot } from "urdf-loader";
import { BASE_URL } from "@/constants";
import { TCP_LINK_NAME, JOINT_CONFIGS } from "@/lib/robot/config";

interface URDFRobotProps {
  jointAngles: number[];
  onTCPMatrix?: (matrix: THREE.Matrix4) => void;
  onLinksLoaded?: (linkNames: string[]) => void;
  linkVisibility?: Record<string, boolean>;
  visible?: boolean;
}

function emitTCP(
  robot: URDFRobot,
  cb: ((m: THREE.Matrix4) => void) | undefined,
) {
  if (!cb || !robot.links?.[TCP_LINK_NAME]) return;
  const link = robot.links[TCP_LINK_NAME];
  link.updateWorldMatrix(true, false);
  cb(link.matrixWorld.clone());
}

export function RobotModel({
  jointAngles,
  onTCPMatrix,
  onLinksLoaded,
  linkVisibility,
  visible = true,
}: URDFRobotProps) {
  const groupRef = useRef<THREE.Group>(null);
  const robotRef = useRef<URDFRobot>(null);
  const onTCPMatrixRef = useRef(onTCPMatrix);
  useEffect(() => {
    onTCPMatrixRef.current = onTCPMatrix;
  }, [onTCPMatrix]);

  // URDF 로드
  useEffect(() => {
    let cancelled = false;
    const currentGroup = groupRef.current;

    const loader = new URDFLoader();
    loader.packages = {
      omx_description: `${BASE_URL}/robot`,
      omx_f: `${BASE_URL}/robot`,
    };
    loader.workingPath = `${BASE_URL}/robot/urdf/omx_f/`;

    loader.load(
      `${BASE_URL}/robot/urdf/omx_f/omx_f.urdf`,
      (robot: URDFRobot) => {
        if (cancelled) return;

        robotRef.current = robot;
        currentGroup?.add(robot);

        if (robot.links) {
          const names = Object.keys(robot.links).sort();
          onLinksLoaded?.(names);
        }

        emitTCP(robot, onTCPMatrixRef.current);
      },
      undefined,
      (err: unknown) => console.error("[URDFRobot] load error:", err),
    );

    return () => {
      cancelled = true;
      if (robotRef.current && currentGroup) {
        currentGroup.remove(robotRef.current);
        robotRef.current = null;
      }
    };
  }, [onLinksLoaded]);

  // Joint 각도 적용
  useEffect(() => {
    const robot = robotRef.current;
    if (!robot) return;

    JOINT_CONFIGS.forEach((joint, i) => {
      const angle = jointAngles[i];
      if (angle !== undefined && robot.joints?.[joint.name]) {
        robot.setJointValue(joint.name, angle);
      }
    });

    emitTCP(robot, onTCPMatrix);
  }, [jointAngles, onTCPMatrix]);

  // 전체 visible
  useEffect(() => {
    if (robotRef.current) robotRef.current.visible = visible;
  }, [visible]);

  // 링크별 visibility
  useEffect(() => {
    const robot = robotRef.current;
    if (!robot?.links || !linkVisibility) return;

    Object.entries(linkVisibility).forEach(([name, vis]) => {
      const link = robot.links[name];
      if (link) link.visible = vis;
    });
  }, [linkVisibility]);

  return <group rotation={[-Math.PI / 2, 0, 0]} ref={groupRef} />;
}
