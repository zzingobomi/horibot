/**
 * contract graph viewer payload 타입 — GET /contract/graph 전용.
 *
 * contract_graph_viewer.md §4 + §6.3. 앱의 generated `contract.ts` (FRONTEND_EXPOSED
 * subset) 를 import 하지 않는다 — 뷰어는 전 계약(unfiltered)이라 성격이 다르다.
 * 작아서 hand-write (backend build_contract_graph 응답 스키마와 1:1).
 */

export type KeyCategory = "service" | "stream" | "event";
export type EdgeCategory = "stream" | "event";

export interface GraphModule {
  id: string; // module class name
  domain: string; // wire_key prefix 에서 유도 (그룹 색상)
  robot_scoped: boolean;
  services: string[]; // owner(server) — caller 엣지는 없음 (§2 한계)
  publishes: string[]; // output stream/event
  subscribes: string[]; // input stream/event
}

export interface ServiceKeyInfo {
  category: "service";
  req: string;
  res: string;
}

export interface TopicKeyInfo {
  category: EdgeCategory;
  payload: string;
}

export type KeyInfo = ServiceKeyInfo | TopicKeyInfo;

export interface GraphEdge {
  source: string; // publisher module id
  target: string; // subscriber module id
  key: string; // wire_key
  category: EdgeCategory;
}

/** model name → { field: ts_type } (드릴다운 스키마). */
export type ModelSchema = Record<string, string>;

export interface ContractGraph {
  modules: GraphModule[];
  keys: Record<string, KeyInfo>;
  models: Record<string, ModelSchema>;
  edges: GraphEdge[];
}
