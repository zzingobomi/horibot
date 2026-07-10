// extractMarkers — step 결과 → 3D marker dispatch (순수 함수).
// 컴포넌트 렌더는 R3F Canvas 필요 — 여기선 dispatch 규칙만 계약으로 고정.

import { describe, expect, it } from "vitest";
import { extractMarkers } from "./TaskResultsOverlay";

const det = (x: number, score = 0.9) => ({
  prompt: "white cube",
  position: [x, 0.1, 0.05],
  score,
  base_z: 0,
  height: 0.05,
});

describe("extractMarkers", () => {
  it("Detection → detection marker + 라벨(prompt+score)", () => {
    const m = extractMarkers({
      select_pick: { step_id: "select_pick", type: "Detection", value: det(0.2) },
    });
    expect(m).toHaveLength(1);
    expect(m[0].kind).toBe("detection");
    expect(m[0].position).toEqual([0.2, 0.1, 0.05]);
    expect(m[0].label).toContain("white cube");
    expect(m[0].label).toContain("90");
  });

  it("list → Detection 모양 원소만 candidate marker (그 외 skip)", () => {
    const m = extractMarkers({
      search: {
        step_id: "search",
        type: "list",
        value: [det(0.1), det(0.3, 0.4), "noise", { foo: 1 }],
      },
    });
    expect(m).toHaveLength(2);
    expect(m.every((x) => x.kind === "candidate")).toBe(true);
    expect(m[0].key).toBe("search:0");
  });

  it("Position3 → position marker", () => {
    const m = extractMarkers({
      grasp_policy: {
        step_id: "grasp_policy",
        type: "Position3",
        value: { x: 0.25, y: -0.05, z: 0.03 },
      },
    });
    expect(m).toHaveLength(1);
    expect(m[0].kind).toBe("position");
    expect(m[0].position).toEqual([0.25, -0.05, 0.03]);
  });

  it("None/미지 type → marker 없음", () => {
    const m = extractMarkers({
      w: { step_id: "w", type: "None", value: null },
      g: { step_id: "g", type: "str", value: "open" },
    });
    expect(m).toHaveLength(0);
  });
});
