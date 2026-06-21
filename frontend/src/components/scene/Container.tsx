import { useMemo } from "react";
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
  const joints =
    useTopic(Topic.MOTOR_STATE_JOINT, calibRobotId)?.joints ?? EMPTY_JOINTS;
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

  // TCP pose 의 SSOT = backend MOTION_STATE_TCP topic (sag + link_offset +
  // joint_offset 적용된 corrected FK). 자체 URDF FK 로 cameraMatrix 만들지 X —
  // 그렇게 하면 sag/link_offset 빠져 PC 가 사선으로 나옴 (회귀 차단). URDF
  // visual model 의 시각 위치는 여전히 자체 FK 라 backend tcp 와 미세 mismatch
  // 가능 (≤ 4°), critical 하지 않으므로 별도 작업으로 미룸.
  const tcpState = useTopic(Topic.MOTION_STATE_TCP, calibRobotId);
  const tcpRobotBaseMatrix = useMemo(() => {
    // z-up world (robot frame) → R3F y-up world 변환 + base_pose 적용.
    // RobotModel 의 outer/inner group 구조 (rotation [-π/2, 0, 0] · translate ·
    // rotZ(yaw)) 와 동일 사상 — TCP pose 가 URDF z-up frame 에 있어 같은 chain.
    const r = robots.find((x) => x.id === calibRobotId);
    if (!r) return null;
    const outer = new THREE.Matrix4().makeRotationX(-Math.PI / 2);
    const inner = new THREE.Matrix4().compose(
      new THREE.Vector3(r.base_pose.x, r.base_pose.y, r.base_pose.z),
      new THREE.Quaternion().setFromAxisAngle(
        new THREE.Vector3(0, 0, 1),
        (r.base_pose.yaw_deg * Math.PI) / 180,
      ),
      new THREE.Vector3(1, 1, 1),
    );
    return outer.multiply(inner);
  }, [robots, calibRobotId]);

  const tcpMatrix = useMemo<THREE.Matrix4 | null>(() => {
    if (!tcpState || !tcpRobotBaseMatrix) return null;
    const [px, py, pz] = tcpState.position;
    const [qx, qy, qz, qw] = tcpState.quaternion;
    const local = new THREE.Matrix4().compose(
      new THREE.Vector3(px, py, pz),
      new THREE.Quaternion(qx, qy, qz, qw),
      new THREE.Vector3(1, 1, 1),
    );
    // world (R3F y-up) ← robot_base (z-up) · tcp_local
    return tcpRobotBaseMatrix.clone().multiply(local);
  }, [tcpState, tcpRobotBaseMatrix]);

  // TCP 위치 표시는 RobotStatePanel 이 *직접* MOTION_STATE_TCP topic subscribe.
  // 본 Container 가 setTcpPos useEffect 로 우회한 자리는 매 message 새 array set
  // 자체 자리 React reconciler 자리 cycle / stall (2026-06-21 사선 PC 회귀 사례)
  // → SSOT 정석: backend topic 한 source 만 신뢰, intermediate store 자리 폐기.

  return (
    <RobotScene
      jointAngles={jointAngles}
      calibration={results}
      options={options}
      linkVisibility={linkVisibility}
      onLinksLoaded={setLinkNames}
      tcpMatrix={tcpMatrix}
      robots={robots}
      focusId={effectiveFocus}
      cameraTarget={cameraTarget}
    />
  );
}
