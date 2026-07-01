/**
 * 3D scene container — backend_v2 wire 정합 박은 simplified version.
 *
 * 옛 frontend Container.tsx 의 dependency 큰 정리:
 *   - 박지 X: useCalibrationResults / useJointOffsetsRad / useMotorConfigs / useSceneStore
 *   - 박음: useStream(Motion.Stream.TCP_STATE) — joints rad 직접 + position/quaternion
 *
 * Motion.Stream.TCP_STATE 의 `joints` 가 backend Motion module 의 *URDF rad list*
 * (raw → rad + joint_offset + sag 보정 모두 완료). frontend 자체 변환 박지 X.
 */
import { useMemo } from "react";
import * as THREE from "three";
import { RobotScene } from "./Scene";
import { useStream } from "@/framework";
import { Topic } from "@/api/generated/contract";
import { useRobots } from "@/hooks/useRobots";

const EMPTY_JOINTS: number[] = [];
const EMPTY_NAMES: string[] = [];

interface RobotSceneContainerProps {
  /** focus robot id. null = 모두 동등. undefined = default robot. */
  focusId?: string | null;
}

export function RobotSceneContainer({ focusId }: RobotSceneContainerProps = {}) {
  const { robots, defaultId } = useRobots();
  const effectiveFocus: string | null =
    focusId === undefined ? defaultId : focusId;
  const calibRobotId = effectiveFocus ?? defaultId ?? "";

  const tcp = useStream(Topic.MOTION_TCP_STATE, { robotId: calibRobotId });
  // parallel arrays — backend Motion 이 joint_names + joints 를 same order 로 발행.
  // URDF 파일 순서 안 믿고 이 name list 로 setJointValue (ROS JointState 패턴).
  const jointNames = tcp.value?.joint_names ?? EMPTY_NAMES;
  const jointAngles = tcp.value?.joints ?? EMPTY_JOINTS;

  // focus robot 의 base_pose 로 OrbitControls target.
  const cameraTarget = useMemo<[number, number, number]>(() => {
    if (effectiveFocus !== null) {
      const r = robots.find((x) => x.id === effectiveFocus);
      if (r?.base_pose) {
        return [r.base_pose.x ?? 0, r.base_pose.y ?? 0, (r.base_pose.z ?? 0) + 0.1];
      }
    }
    if (robots.length === 0) return [0, 0.1, 0];
    const acc = robots.reduce(
      (a, r) => {
        a.x += r.base_pose?.x ?? 0;
        a.y += r.base_pose?.y ?? 0;
        a.z += r.base_pose?.z ?? 0;
        return a;
      },
      { x: 0, y: 0, z: 0 },
    );
    const n = robots.length;
    return [acc.x / n, acc.y / n, acc.z / n + 0.1];
  }, [robots, effectiveFocus]);

  // TCP pose = backend Motion.Stream.TCP_STATE (corrected FK 완료, SSOT).
  // 자체 URDF FK 박지 X — sag/link_offset 누락 회귀 차단.
  const tcpRobotBaseMatrix = useMemo(() => {
    const r = robots.find((x) => x.id === calibRobotId);
    if (!r?.base_pose) return null;
    const outer = new THREE.Matrix4().makeRotationX(-Math.PI / 2);
    const inner = new THREE.Matrix4().compose(
      new THREE.Vector3(r.base_pose.x ?? 0, r.base_pose.y ?? 0, r.base_pose.z ?? 0),
      new THREE.Quaternion().setFromAxisAngle(
        new THREE.Vector3(0, 0, 1),
        ((r.base_pose.yaw_deg ?? 0) * Math.PI) / 180,
      ),
      new THREE.Vector3(1, 1, 1),
    );
    return outer.multiply(inner);
  }, [robots, calibRobotId]);

  const tcpMatrix = useMemo<THREE.Matrix4 | null>(() => {
    if (!tcp.value || !tcpRobotBaseMatrix) return null;
    const [px, py, pz] = tcp.value.position;
    const [qx, qy, qz, qw] = tcp.value.quaternion;
    const local = new THREE.Matrix4().compose(
      new THREE.Vector3(px, py, pz),
      new THREE.Quaternion(qx, qy, qz, qw),
      new THREE.Vector3(1, 1, 1),
    );
    return tcpRobotBaseMatrix.clone().multiply(local);
  }, [tcp.value, tcpRobotBaseMatrix]);

  return (
    <RobotScene
      jointNames={jointNames}
      jointAngles={jointAngles}
      tcpMatrix={tcpMatrix}
      robots={robots}
      focusId={effectiveFocus}
      cameraTarget={cameraTarget}
    />
  );
}
