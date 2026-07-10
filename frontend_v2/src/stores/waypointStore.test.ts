import { describe, it, expect, beforeEach } from "vitest";
import { useWaypointStore } from "./waypointStore";

const P1 = { waypointId: 1, name: "a", jointNames: ["j1"], jointAngles: [0.1] };
const P2 = { waypointId: 2, name: "b", jointNames: ["j1"], jointAngles: [0.2] };

beforeEach(() => {
  useWaypointStore.setState({ previews: {} });
});

describe("waypointStore — ghost preview", () => {
  it("set / clear(null)", () => {
    const s = useWaypointStore.getState();
    s.setPreview("r1", P1);
    expect(useWaypointStore.getState().previews["r1"]).toEqual(P1);
    s.setPreview("r1", null);
    expect(useWaypointStore.getState().previews["r1"]).toBeUndefined();
  });

  it("per-robot 분리 — robot A/B 각자 자기 ghost", () => {
    const s = useWaypointStore.getState();
    s.setPreview("a", P1);
    s.setPreview("b", P2);
    const p = useWaypointStore.getState().previews;
    expect(p["a"]?.waypointId).toBe(1);
    expect(p["b"]?.waypointId).toBe(2);
    s.setPreview("a", null);
    expect(useWaypointStore.getState().previews["b"]?.waypointId).toBe(2);
  });
});
