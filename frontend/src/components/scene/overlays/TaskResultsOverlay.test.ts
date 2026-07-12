// extractMarkers — STEP_RESULT → 3D marker dispatch (순수 함수).
// 컴포넌트 렌더는 R3F Canvas 필요 — 여기선 dispatch 규칙만 계약으로 고정.

import { describe, expect, it } from "vitest";
import { extractMarkers } from "./TaskResultsOverlay";

const det = (x: number, score = 0.9) => ({
  prompt: "white cube",
  position: [x, 0.1, 0.05],
  score,
  base_z: 0,
  height: 0.05,
  grasp_yaw: 0.3,
  footprint: [0.023, 0.022],
});

describe("extractMarkers", () => {
  it("OrientedDetection → detection marker + 라벨(prompt+score)", () => {
    const m = extractMarkers({
      target: { label: "target", type: "OrientedDetection", value: det(0.2) },
    });
    expect(m).toHaveLength(1);
    expect(m[0].kind).toBe("detection");
    expect(m[0].position).toEqual([0.2, 0.1, 0.05]);
    expect(m[0].label).toContain("white cube");
    expect(m[0].label).toContain("90");
  });

  it("list → detection 모양 원소만 candidate marker (그 외 skip)", () => {
    const m = extractMarkers({
      detect_pick: {
        label: "detect_pick",
        type: "list",
        value: [det(0.1), det(0.3, 0.4), "noise", { foo: 1 }],
      },
    });
    expect(m).toHaveLength(2);
    expect(m.every((x) => x.kind === "candidate")).toBe(true);
    expect(m[0].key).toBe("detect_pick:0");
  });

  it("GraspCandidate → grasp 지점 position marker", () => {
    const m = extractMarkers({
      grasp: {
        label: "grasp",
        type: "GraspCandidate",
        value: {
          label: "tilt=+0 yaw=17 flip=0",
          pre: [0.25, -0.05, 0.09],
          grasp: [0.25, -0.05, 0.012],
          quat: [0, 0.7, 0, 0.7],
          lateral: 0.008,
        },
      },
    });
    expect(m).toHaveLength(1);
    expect(m[0].kind).toBe("position");
    expect(m[0].position).toEqual([0.25, -0.05, 0.012]);
  });

  it("PlaceCandidate → place 지점 position marker", () => {
    const m = extractMarkers({
      place: {
        label: "place",
        type: "PlaceCandidate",
        value: {
          label: "tilt=+0 yaw=0 flip=0",
          pre: [0.3, 0.0, 0.12],
          place: [0.3, 0.0, 0.06],
          quat: [0, 0.7, 0, 0.7],
        },
      },
    });
    expect(m).toHaveLength(1);
    expect(m[0].kind).toBe("position");
    expect(m[0].position).toEqual([0.3, 0.0, 0.06]);
  });

  it("None/미지 type → marker 없음", () => {
    const m = extractMarkers({
      w: { label: "w", type: "None", value: null },
      g: { label: "g", type: "str", value: "open" },
    });
    expect(m).toHaveLength(0);
  });
});
