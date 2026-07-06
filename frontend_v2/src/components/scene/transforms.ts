/**
 * Scene 공용 transform 헬퍼 — robot base 배치 수학의 SSOT.
 *
 * world 배치 = [z-up → y-up rotX(-π/2)] · [base_pose translate + yaw].
 * 소비자: RobotLayer(TCP AxisFrame) / Scene3DLayer(라이브 cloud) / Container(MeshLayer).
 * 같은 수학을 각자 복제하면 한 곳만 고쳐질 때 로봇/cloud/mesh 가 어긋난다.
 */
import * as THREE from "three";
import type { BasePoseInfo } from "@/api/generated/contract";

/** robot base 의 world transform (R3F y-up). basePose 없으면 원점. */
export function robotBaseMatrix(basePose?: BasePoseInfo | null): THREE.Matrix4 {
  const outer = new THREE.Matrix4().makeRotationX(-Math.PI / 2);
  const inner = new THREE.Matrix4().compose(
    new THREE.Vector3(basePose?.x ?? 0, basePose?.y ?? 0, basePose?.z ?? 0),
    new THREE.Quaternion().setFromAxisAngle(
      new THREE.Vector3(0, 0, 1),
      ((basePose?.yaw_deg ?? 0) * Math.PI) / 180,
    ),
    new THREE.Vector3(1, 1, 1),
  );
  return outer.multiply(inner);
}

/** base-frame 상대 pose (position + quaternion) 를 world matrix 로. */
export function poseToWorldMatrix(
  base: THREE.Matrix4,
  position: readonly [number, number, number],
  quaternion: readonly [number, number, number, number],
): THREE.Matrix4 {
  const [px, py, pz] = position;
  const [qx, qy, qz, qw] = quaternion;
  const local = new THREE.Matrix4().compose(
    new THREE.Vector3(px, py, pz),
    new THREE.Quaternion(qx, qy, qz, qw),
    new THREE.Vector3(1, 1, 1),
  );
  return base.clone().multiply(local);
}
