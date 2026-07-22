// ROI 드래그 수학 — 뒤집으면 회귀: 축-ray 최근접 파라미터가 틀리면 드래그가
// 포인터를 안 따라오고, clamp 가 빠지면 min/max 역전 ROI 가 backend validator 에
// 서야 거부된다 (저장 시점 에러 = UX 반쪽 — 드래그 단계에서 못 만들어야 함).

import { describe, expect, it } from "vitest";
import {
  MIN_SPAN_M,
  applyFaceDrag,
  applyTranslate,
  axisParamAtRay,
  faceCenter,
  roiCenter,
  roiEquals,
  roiSize,
} from "./dragMath";

const ROI = {
  x_min: 0.0,
  x_max: 0.35,
  y_min: -0.3,
  y_max: 0.3,
  z_min: -0.05,
  z_max: 0.35,
};

describe("axisParamAtRay", () => {
  it("축을 정면 교차하는 ray — 교차점의 축 파라미터", () => {
    // 축 = x (anchor 원점), ray = (0.2, 0, 1) 에서 -z 로 → x=0.2 에서 교차
    const s = axisParamAtRay([0, 0, 0], [1, 0, 0], [0.2, 0, 1], [0, 0, -1]);
    expect(s).toBeCloseTo(0.2, 9);
  });

  it("비스듬한 ray — 최근접점 (해석 검증: 45° 접근)", () => {
    // ray 가 (0, 1) 에서 (1, -1)/√2 방향 → x 축과 x=1 에서 교차
    const s = axisParamAtRay(
      [0, 0, 0],
      [1, 0, 0],
      [0, 0, 1],
      [Math.SQRT1_2, 0, -Math.SQRT1_2],
    );
    expect(s).toBeCloseTo(1.0, 9);
  });

  it("축과 평행한 ray → null (파라미터 불정 — 드래그 무시)", () => {
    expect(axisParamAtRay([0, 0, 0], [1, 0, 0], [0, 0, 1], [1, 0, 0])).toBeNull();
  });
});

describe("applyFaceDrag", () => {
  it("max 면 드래그 = 그 bound 만 이동 (나머지 5개 불변)", () => {
    const next = applyFaceDrag(ROI, "x_max", 0.05);
    expect(next.x_max).toBeCloseTo(0.4);
    expect(next.x_min).toBe(ROI.x_min);
    expect(next.y_min).toBe(ROI.y_min);
  });

  it("min 면 드래그가 max 를 넘으면 MIN_SPAN 에서 정지 (역전 금지)", () => {
    const next = applyFaceDrag(ROI, "x_min", 10);
    expect(next.x_min).toBeCloseTo(ROI.x_max - MIN_SPAN_M);
  });

  it("max 면을 min 아래로 끌어도 역전 금지", () => {
    const next = applyFaceDrag(ROI, "z_max", -10);
    expect(next.z_max).toBeCloseTo(ROI.z_min + MIN_SPAN_M);
  });
});

describe("applyTranslate", () => {
  it("min/max 동시 이동 — 크기 불변 (사용자 요구: 박스 전체 5cm 이동)", () => {
    const next = applyTranslate(ROI, "y", 0.05);
    expect(next.y_min).toBeCloseTo(-0.25);
    expect(next.y_max).toBeCloseTo(0.35);
    expect(roiSize(next)).toEqual(roiSize(ROI));
  });
});

describe("기하 헬퍼", () => {
  it("center/size/faceCenter 일관", () => {
    expect(roiCenter(ROI)).toEqual([0.175, 0, 0.15]);
    expect(roiSize(ROI)[0]).toBeCloseTo(0.35);
    expect(faceCenter(ROI, "x_max")).toEqual([0.35, 0, 0.15]);
    expect(faceCenter(ROI, "z_min")[2]).toBeCloseTo(-0.05);
  });

  it("roiEquals — 동일/불일치", () => {
    expect(roiEquals(ROI, { ...ROI })).toBe(true);
    expect(roiEquals(ROI, { ...ROI, z_max: 0.4 })).toBe(false);
  });
});
