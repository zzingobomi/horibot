import { useCallback, useMemo } from "react";
import * as THREE from "three";
import { RobotScene } from "@/components/scene/Scene";
import { useTopic } from "@/framework";
import { Topic } from "@/constants/topics";
import {
  useCalibrationResults,
  useJointOffsetsRad,
} from "@/hooks/useCalibrationResults";
import { useSceneStore } from "@/domain/stores/scene";
import { useRobots } from "@/hooks/useRobots";
import { useMotorConfigs } from "@/lib/robot/config";
import type { Joint } from "@/types/motor";

const EMPTY_JOINTS: Joint[] = [];

interface RobotSceneContainerProps {
  /** focus robot id. null=WorldPage(모두 동등). undefined=default(backend default robot). */
  focusId?: string | null;
}

export function RobotSceneContainer({ focusId }: RobotSceneContainerProps = {}) {
  const { robots, defaultId } = useRobots();
  const effectiveFocus: string | null =
    focusId === undefined ? defaultId : focusId;
  // calibration / jointOffsets 는 robot-scoped — focus 가 null (WorldPage) 인
  // 경우 default robot 의 캘 보여줌 (transition; multi-robot WorldPage 시
  // 어떻게 표현할지는 별도 자리).
  const calibRobotId = effectiveFocus ?? defaultId ?? "";
  const { results } = useCalibrationResults(calibRobotId);
  const joints = useTopic(Topic.MOTOR_STATE_JOINT)?.joints ?? EMPTY_JOINTS;
  const jointOffsetsRad = useJointOffsetsRad(calibRobotId);
  // motorCfgs (id 오름차순) 가 jointAngles 순서를 결정 — RobotModel 의 setJointValue
  // 매핑과 같은 source 사용해 인덱스 align (motorCfgs.length 가 robot 마다 달라도 OK).
  const motorCfgs = useMotorConfigs(calibRobotId);

  const jointAngles = useMemo<number[]>(() => {
    if (!motorCfgs.length) return [];
    const jointById = new Map(joints.map((j) => [j.id, j]));
    return motorCfgs.map((cfg) => {
      const j = jointById.get(cfg.id);
      if (!j) return 0;
      // 백엔드 JointStateCache 와 동일한 보정: raw_to_rad + joint_offset.
      const baseRad =
        j.degree !== undefined
          ? (j.degree * Math.PI) / 180
          : j.position !== undefined
            ? ((j.position - 2048) / 4095) * 2 * Math.PI
            : 0;
      return baseRad + (jointOffsetsRad[j.id] ?? 0);
    });
  }, [motorCfgs, joints, jointOffsetsRad]);

  const options = useSceneStore((s) => s.options);
  const linkVisibility = useSceneStore((s) => s.linkVisibility);
  const setLinkNames = useSceneStore((s) => s.setLinkNames);
  const setTcpPos = useSceneStore((s) => s.setTcpPos);

  // focus robot 의 base_pose 로 OrbitControls target. WorldPage(null) = 중심.
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
    [setTcpPos],
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
