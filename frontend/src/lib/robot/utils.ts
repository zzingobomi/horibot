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
