/**
 * useContractGraph — GET /contract/graph 를 프레임워크 useResource 로 fetch.
 *
 * 별도 fetch 코드를 새로 짜지 않고 framework 의 HTTP consumer (useResource, useRobots
 * 가 쓰는 것과 동일) 를 재사용. 뷰어 전용 타입만 얹는다 (§6.3). backend 코드/python
 * 환경 접근 0 — 오직 HTTP (§8 경계).
 */
import { useResource } from "@/framework";
import type { ContractGraph } from "./types";

export function useContractGraph() {
  return useResource<ContractGraph>("/contract/graph");
}
