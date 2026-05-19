import type { Vec3 } from "@/types/motion";

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

export function rawToRad(raw: number, offsetRad = 0): number {
  return ((raw - RAW_CENTER) / RAW_RANGE) * 2 * Math.PI + offsetRad;
}

export function formatDeg(deg: number): number {
  return Math.round(deg * 10) / 10;
}

export function mmToMVec3(v: Vec3): Vec3 {
  return [v[0] * MM_TO_M, v[1] * MM_TO_M, v[2] * MM_TO_M];
}

export function mToMmVec3(v: Vec3): Vec3 {
  return [v[0] * M_TO_MM, v[1] * M_TO_MM, v[2] * M_TO_MM];
}
