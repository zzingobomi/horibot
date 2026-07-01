/**
 * toReactFlow — 중립 그래프({modules, edges}) → React Flow {nodes, edges} + dagre layout.
 *
 * backend 는 position 을 모르는 중립 그래프만 낸다 (§4). 좌표는 여기서 dagre 로 계산.
 * 순수 함수 (I/O 없음) — L2 test 대상.
 */
import Dagre from "@dagrejs/dagre";
import type { Edge } from "@xyflow/react";
import type { ContractGraph } from "./types";
import type { ModuleNodeType } from "./nodes/ModuleNode";

const NODE_WIDTH = 240;
const ROW_HEIGHT = 18;
const HEADER_HEIGHT = 64;

function nodeHeight(rows: number): number {
  return HEADER_HEIGHT + rows * ROW_HEIGHT;
}

/** wire_key `stream/motor/{robot_id}/raw_state` → 라벨 `raw_state` (마지막 세그먼트). */
export function shortKey(key: string): string {
  const parts = key.split("/");
  return parts[parts.length - 1] || key;
}

export interface FlowGraph {
  nodes: ModuleNodeType[];
  edges: Edge[];
}

export function toReactFlow(graph: ContractGraph): FlowGraph {
  // multigraph — 같은 (src,tgt) 사이 여러 wire (camera jpeg + depth_raw) 를 name 으로
  const g = new Dagre.graphlib.Graph({ multigraph: true }).setDefaultEdgeLabel(
    () => ({}),
  );
  g.setGraph({ rankdir: "LR", nodesep: 50, ranksep: 130, marginx: 20, marginy: 20 });

  for (const m of graph.modules) {
    const rows =
      m.services.length + m.publishes.length + m.subscribes.length;
    g.setNode(m.id, { width: NODE_WIDTH, height: nodeHeight(rows) });
  }
  for (const e of graph.edges) {
    // dagre 는 같은 (src,tgt) multi-edge 를 name 으로 구분
    g.setEdge(e.source, e.target, {}, e.key);
  }

  Dagre.layout(g);

  const nodes: ModuleNodeType[] = graph.modules.map((m) => {
    const pos = g.node(m.id);
    return {
      id: m.id,
      type: "module",
      position: { x: pos.x - pos.width / 2, y: pos.y - pos.height / 2 },
      data: { module: m },
    };
  });

  // 같은 (src,tgt) 사이 여러 wire (예: camera jpeg + depth_raw) — key 로 unique id
  const edges: Edge[] = graph.edges.map((e) => ({
    id: `${e.source}__${e.target}__${e.key}`,
    source: e.source,
    target: e.target,
    label: shortKey(e.key),
    data: { key: e.key, category: e.category },
    animated: e.category === "stream",
    style:
      e.category === "event"
        ? { strokeDasharray: "5 4", stroke: "#a78bfa" }
        : { stroke: "#38bdf8" },
    labelStyle: { fill: "#a1a1aa", fontSize: 10, fontFamily: "monospace" },
    labelBgStyle: { fill: "#18181b" },
  }));

  return { nodes, edges };
}
