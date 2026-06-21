import type { Vector3Tuple } from "three";

export const RAW_CENTER = 2048;
export const RAW_RANGE = 4095;

export const MM_TO_M = 0.001;
export const M_TO_MM = 1000;

export function degToRaw(deg: number): number {
  return Math.round((deg / 360) * RAW_RANGE + RAW_CENTER);
}

export function rawToDeg(raw: number): number {
  return ((raw - RAW_CENTER) / RAW_RANGE) * 360;
}

/**
 * raw motor position → URDF degree (joint_offset 적용된 kinematic frame).
 *
 * UI 의 모든 joint 각도 표시 frame 의 SSOT. backend `JointCoordinates.motor_to_urdf`
 * (raw → rad + joint_offset) 와 같은 의미 — 단 frontend 자리는 degree.
 *
 * backend MoveJ handler 가 사용자 input degree 를 *URDF degree* 로 해석하므로,
 * 모든 표시 / input 자리 본 함수 결과를 쓰면 robot 의 실제 자세 ↔ 사용자 input
 * 자리 일관 (= "[현재 자세] → [실행]" 자리 robot 이 그 자리 그대로 머무름).
 */
export function rawToUrdfDeg(raw: number, offsetRad = 0): number {
  return rawToDeg(raw) + (offsetRad * 180) / Math.PI;
}

/**
 * URDF degree → raw motor position (위 함수의 역).
 */
export function urdfDegToRaw(urdfDeg: number, offsetRad = 0): number {
  return degToRaw(urdfDeg - (offsetRad * 180) / Math.PI);
}

export function formatDeg(deg: number): number {
  return Math.round(deg * 10) / 10;
}

// input 은 backend wire (`number[]`) 도 받게 widening — wire ↔ R3F prop tuple narrowing 자리.
export function mmToMVec3(v: ArrayLike<number>): Vector3Tuple {
  return [v[0] * MM_TO_M, v[1] * MM_TO_M, v[2] * MM_TO_M];
}

export function mToMmVec3(v: ArrayLike<number>): Vector3Tuple {
  return [v[0] * M_TO_MM, v[1] * M_TO_MM, v[2] * M_TO_MM];
}
