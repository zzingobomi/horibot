/**
 * 카메라 pose/frustum 순수 수학 — Camera 씬 객체([Cameras.tsx])가 소비.
 *
 * 카메라 pose(base frame) = tcp(corrected FK, base frame) · hand_eye.
 * base 변환은 <RobotFrame> 이 담당하므로 여기는 base-frame 안 local 만 계산.
 * (옛 LivePointCloudPanel/cameraPose.ts — frustum 이 패널 소유에서 카메라(월드)
 * 소유로 이동하며 함께 이사, [docs/scene_contribution_architecture.md].)
 */
import * as THREE from "three";
import type { CalibrationBundle } from "@/api/generated/contract";
import { handEyeMatrix } from "../shared/transforms";

export interface CameraPose {
  position: [number, number, number];
  quaternion: [number, number, number, number];
}

/** camera-in-base pose = tcpLocal · handEye (캘 전이면 handEye=identity → TCP 위치). */
export function cameraInBase(
  tcpPosition: readonly number[],
  tcpQuaternion: readonly number[],
  bundle: CalibrationBundle | null,
): CameraPose {
  const tcpLocal = new THREE.Matrix4().compose(
    new THREE.Vector3(tcpPosition[0], tcpPosition[1], tcpPosition[2]),
    new THREE.Quaternion(
      tcpQuaternion[0],
      tcpQuaternion[1],
      tcpQuaternion[2],
      tcpQuaternion[3],
    ),
    new THREE.Vector3(1, 1, 1),
  );
  const m = tcpLocal.multiply(handEyeMatrix(bundle));
  const p = new THREE.Vector3();
  const q = new THREE.Quaternion();
  const s = new THREE.Vector3();
  m.decompose(p, q, s);
  return { position: [p.x, p.y, p.z], quaternion: [q.x, q.y, q.z, q.w] };
}

// D405 depth FOV 87°(H) × 58°(V) — 시야 방향 감 잡기용 시각화 (정밀 캘 값 아님)
const HALF_H_RAD = ((87 / 2) * Math.PI) / 180;
const HALF_V_RAD = ((58 / 2) * Math.PI) / 180;

/**
 * 카메라 frame(OpenCV: z 전방) frustum wireframe 선분들 — lineSegments 용
 * position buffer (선분당 점 2개). apex→4 corner + far-rect 4 edge = 8 선분.
 */
export function frustumSegmentPositions(depth = 0.12): Float32Array {
  const hw = depth * Math.tan(HALF_H_RAD);
  const hv = depth * Math.tan(HALF_V_RAD);
  const corners: [number, number, number][] = [
    [-hw, -hv, depth],
    [hw, -hv, depth],
    [hw, hv, depth],
    [-hw, hv, depth],
  ];
  const pts: number[] = [];
  for (const c of corners) pts.push(0, 0, 0, ...c); // apex → corner
  for (let i = 0; i < 4; i++) {
    pts.push(...corners[i], ...corners[(i + 1) % 4]); // far rect
  }
  return new Float32Array(pts);
}
