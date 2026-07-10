/**
 * 카메라 pose/frustum 순수 수학 — Camera 씬 객체([Cameras.tsx])가 소비.
 *
 * 카메라 pose(base frame) = tcp(corrected FK, base frame) · hand_eye.
 * base 변환은 <RobotFrame> 이 담당하므로 여기는 base-frame 안 local 만 계산.
 * (옛 LivePointCloudPanel/cameraPose.ts — frustum 이 패널 소유에서 카메라(월드)
 * 소유로 이동하며 함께 이사, [docs/frontend.md].)
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

export interface FrustumFov {
  halfH: number; // rad
  halfV: number; // rad
}

// D405 depth FOV 87°(H) × 58°(V) — active intrinsic 없을 때 fallback (방향 감용 상수)
export const DEFAULT_FOV: FrustumFov = {
  halfH: ((87 / 2) * Math.PI) / 180,
  halfV: ((58 / 2) * Math.PI) / 180,
};

/**
 * active intrinsic → 실 캘 FOV (halfH = atan(w/2/fx)). intrinsic 없으면 null —
 * caller 가 DEFAULT_FOV fallback. 캘된 카메라는 frustum 이 스펙 상수가 아니라
 * 실측 시야각으로 그려짐 (intrinsic 의 3D 시각화).
 */
export function fovFromIntrinsic(
  bundle: CalibrationBundle | null,
): FrustumFov | null {
  const d = bundle?.intrinsic?.result_data;
  const cm = d?.camera_matrix;
  const size = d?.image_size;
  if (!cm || !size || size.length < 2) return null;
  const fx = cm[0]?.[0];
  const fy = cm[1]?.[1];
  if (!fx || !fy) return null;
  return {
    halfH: Math.atan(size[0] / 2 / fx),
    halfV: Math.atan(size[1] / 2 / fy),
  };
}

/**
 * 카메라 frame(OpenCV: z 전방) frustum wireframe 선분들 — lineSegments 용
 * position buffer (선분당 점 2개). apex→4 corner + far-rect 4 edge = 8 선분.
 */
export function frustumSegmentPositions(
  depth = 0.12,
  fov: FrustumFov = DEFAULT_FOV,
): Float32Array {
  const hw = depth * Math.tan(fov.halfH);
  const hv = depth * Math.tan(fov.halfV);
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
