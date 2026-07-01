import { describe, it, expect } from "vitest";
import { toReactFlow, shortKey } from "./toReactFlow";
import type { ContractGraph } from "./types";

function mod(id: string, over: Partial<ContractGraph["modules"][0]> = {}) {
  return {
    id,
    domain: "x",
    robot_scoped: true,
    services: [],
    publishes: [],
    subscribes: [],
    ...over,
  };
}

const GRAPH: ContractGraph = {
  modules: [
    mod("Motor", {
      domain: "motor",
      publishes: ["stream/motor/{robot_id}/raw_state"],
      subscribes: ["stream/motor/{robot_id}/command"],
    }),
    mod("Motion", {
      domain: "motion",
      publishes: ["stream/motor/{robot_id}/command"],
      subscribes: ["stream/motor/{robot_id}/raw_state"],
    }),
    mod("Cam", { domain: "camera" }),
  ],
  keys: {},
  models: {},
  edges: [
    {
      source: "Motor",
      target: "Motion",
      key: "stream/motor/{robot_id}/raw_state",
      category: "stream",
    },
    {
      source: "Motion",
      target: "Motor",
      key: "stream/motor/{robot_id}/command",
      category: "stream",
    },
  ],
};

describe("toReactFlow", () => {
  it("maps every module to a node", () => {
    const { nodes } = toReactFlow(GRAPH);
    expect(nodes.map((n) => n.id).sort()).toEqual(["Cam", "Motion", "Motor"]);
    expect(nodes.every((n) => n.type === "module")).toBe(true);
    // module data 보존
    const motor = nodes.find((n) => n.id === "Motor")!;
    expect(motor.data.module.domain).toBe("motor");
  });

  it("preserves edge direction (publisher → subscriber)", () => {
    const { edges } = toReactFlow(GRAPH);
    expect(edges).toHaveLength(2);
    const raw = edges.find((e) => e.label === "raw_state")!;
    expect(raw.source).toBe("Motor");
    expect(raw.target).toBe("Motion");
    const cmd = edges.find((e) => e.label === "command")!;
    expect(cmd.source).toBe("Motion");
    expect(cmd.target).toBe("Motor");
  });

  it("assigns finite dagre coordinates", () => {
    const { nodes } = toReactFlow(GRAPH);
    expect(
      nodes.every(
        (n) => Number.isFinite(n.position.x) && Number.isFinite(n.position.y),
      ),
    ).toBe(true);
    // layout 이 노드를 겹치지 않게 벌림 (같은 좌표 X)
    const xs = new Set(nodes.map((n) => `${n.position.x},${n.position.y}`));
    expect(xs.size).toBe(nodes.length);
  });

  it("gives multi-edges between same pair unique ids", () => {
    const multi: ContractGraph = {
      ...GRAPH,
      modules: [mod("Cam", { domain: "camera" }), mod("Dec", { domain: "camera" })],
      edges: [
        { source: "Cam", target: "Dec", key: "stream/camera/{robot_id}/jpeg", category: "stream" },
        { source: "Cam", target: "Dec", key: "stream/camera/{robot_id}/depth_raw", category: "stream" },
      ],
    };
    const { edges } = toReactFlow(multi);
    expect(edges).toHaveLength(2);
    expect(new Set(edges.map((e) => e.id)).size).toBe(2);
  });

  it("carries category into edge data for schema drilldown", () => {
    const { edges } = toReactFlow(GRAPH);
    expect(
      edges.every((e) => (e.data as { key: string }).key.startsWith("stream/")),
    ).toBe(true);
  });
});

describe("shortKey", () => {
  it("takes the last segment", () => {
    expect(shortKey("stream/motor/{robot_id}/raw_state")).toBe("raw_state");
    expect(shortKey("srv/motion/{robot_id}/move_j")).toBe("move_j");
  });
});
