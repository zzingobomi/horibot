import { describe, it, expect } from "vitest";
import { describeMissing, missingCapabilities } from "./capabilities";

describe("missingCapabilities", () => {
  it("요구 없음 → 항상 빈 배열", () => {
    expect(missingCapabilities(undefined, ["rgbd"])).toEqual([]);
    expect(missingCapabilities([], ["rgbd"])).toEqual([]);
  });

  it("robot 이 요구를 모두 가짐 → 빈 배열", () => {
    expect(missingCapabilities(["rgbd"], ["move", "rgbd"])).toEqual([]);
  });

  it("부족분만 반환 (다중 요구)", () => {
    expect(missingCapabilities(["rgbd", "move"], ["move"])).toEqual(["rgbd"]);
  });

  it("robot capability 미상(undefined) → 요구 전부 부족", () => {
    expect(missingCapabilities(["rgbd"], undefined)).toEqual(["rgbd"]);
  });
});

describe("describeMissing", () => {
  it("override 있으면 그것을 그대로", () => {
    expect(describeMissing(["rgbd"], "커스텀")).toBe("커스텀");
  });

  it("override 없으면 라벨에서 파생", () => {
    expect(describeMissing(["rgbd"])).toBe("RGB-D 카메라 필요");
  });

  it("다중 부족은 라벨을 조립", () => {
    expect(describeMissing(["rgbd", "move"])).toBe("RGB-D 카메라, 모션 필요");
  });

  it("라벨 없는 slug 는 slug 그대로", () => {
    expect(describeMissing(["force_torque"])).toBe("force_torque 필요");
  });

  it("부족 없음 → 빈 문자열", () => {
    expect(describeMissing([])).toBe("");
  });
});
