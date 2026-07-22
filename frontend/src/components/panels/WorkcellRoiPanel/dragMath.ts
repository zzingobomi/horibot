/**
 * ROI 박스 드래그 수학 — scenePart 의 포인터 이벤트를 bound 갱신량으로 바꾸는
 * 순수 계산 (R3F/three 객체 무관 부분은 배열 산술 — vitest 단위 대상).
 *
 * 축 드래그의 정의: 화면 포인터 ray 와 "드래그 축 직선"의 최근접점 파라미터.
 * 면 핸들(resize)·중앙 화살표(translate) 모두 이 하나로 — 시작 시점 파라미터를
 * 기억해 두고 (s0), 이동 중 파라미터와의 차(delta)를 bound 에 가산한다.
 * (Ericson, Real-Time Collision Detection §5.1.8 — 두 직선 최근접점.)
 */
import type { WorkcellRoi } from "@/api/generated/contract";

export type Vec3 = readonly [number, number, number];
export type Axis = "x" | "y" | "z";
export type FaceId = "x_min" | "x_max" | "y_min" | "y_max" | "z_min" | "z_max";

/** bound 최소 두께 — min/max 역전을 드래그 단계에서 차단 (backend validator 는
 * min<max 를 거부 — 애초에 못 만들게 clamp 가 UX). */
export const MIN_SPAN_M = 0.01;

export const FACE_AXIS: Record<FaceId, Axis> = {
  x_min: "x",
  x_max: "x",
  y_min: "y",
  y_max: "y",
  z_min: "z",
  z_max: "z",
};

export const AXIS_DIR: Record<Axis, Vec3> = {
  x: [1, 0, 0],
  y: [0, 1, 0],
  z: [0, 0, 1],
};

/**
 * ray(origin,dir) 와 축 직선(anchor,axis) 의 최근접점에서 축 파라미터 s 를 반환.
 * 거의 평행(분모→0)이면 null — 호출부는 그 move 이벤트를 무시 (툭 튀는 것 방지).
 */
export function axisParamAtRay(
  anchor: Vec3,
  axis: Vec3,
  rayOrigin: Vec3,
  rayDir: Vec3,
): number | null {
  // L1 = anchor + s·axis (a=axis·axis), L2 = origin + t·dir (e=dir·dir)
  const r: Vec3 = [
    anchor[0] - rayOrigin[0],
    anchor[1] - rayOrigin[1],
    anchor[2] - rayOrigin[2],
  ];
  const dot = (u: Vec3, v: Vec3) => u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
  const a = dot(axis, axis);
  const e = dot(rayDir, rayDir);
  const b = dot(axis, rayDir);
  const c = dot(axis, r);
  const f = dot(rayDir, r);
  const denom = a * e - b * b;
  if (Math.abs(denom) < 1e-9) return null; // 축 ∥ 시선 — 파라미터 불정
  return (b * f - c * e) / denom;
}

/** 면 핸들 드래그 적용 — 그 면의 bound 하나만, 반대 bound 와 MIN_SPAN 유지. */
export function applyFaceDrag(
  roi: WorkcellRoi,
  face: FaceId,
  delta: number,
): WorkcellRoi {
  const next = { ...roi };
  const axis = FACE_AXIS[face];
  const [lo, hi] = [`${axis}_min`, `${axis}_max`] as [FaceId, FaceId];
  if (face === lo) next[lo] = Math.min(roi[lo] + delta, roi[hi] - MIN_SPAN_M);
  else next[hi] = Math.max(roi[hi] + delta, roi[lo] + MIN_SPAN_M);
  return next;
}

/** 평행이동 적용 — 그 축 min/max 동시 이동 (크기 불변). */
export function applyTranslate(
  roi: WorkcellRoi,
  axis: Axis,
  delta: number,
): WorkcellRoi {
  const next = { ...roi };
  const [lo, hi] = [`${axis}_min`, `${axis}_max`] as [FaceId, FaceId];
  next[lo] = roi[lo] + delta;
  next[hi] = roi[hi] + delta;
  return next;
}

export function roiCenter(roi: WorkcellRoi): [number, number, number] {
  return [
    (roi.x_min + roi.x_max) / 2,
    (roi.y_min + roi.y_max) / 2,
    (roi.z_min + roi.z_max) / 2,
  ];
}

export function roiSize(roi: WorkcellRoi): [number, number, number] {
  return [
    roi.x_max - roi.x_min,
    roi.y_max - roi.y_min,
    roi.z_max - roi.z_min,
  ];
}

/** 면 중심 (base frame) — 핸들/하이라이트 배치 기준. */
export function faceCenter(roi: WorkcellRoi, face: FaceId): [number, number, number] {
  const c = roiCenter(roi);
  const axis = FACE_AXIS[face];
  const idx = axis === "x" ? 0 : axis === "y" ? 1 : 2;
  c[idx] = roi[face];
  return c;
}

export function roiEquals(a: WorkcellRoi, b: WorkcellRoi): boolean {
  return (
    a.x_min === b.x_min && a.x_max === b.x_max &&
    a.y_min === b.y_min && a.y_max === b.y_max &&
    a.z_min === b.z_min && a.z_max === b.z_max
  );
}
