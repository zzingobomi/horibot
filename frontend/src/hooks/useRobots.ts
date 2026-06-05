import { useEffect, useState } from "react";
import { BASE_URL, DEFAULT_ROBOT_ID } from "@/constants";
import { bridge } from "@/api/bridge";
import type { components } from "@/api/generated/types";

// backend `bridge/schemas.py` 의 Pydantic 모델을 `pnpm gen:types` 로 emit 한
// 자리에서 그대로 import — frontend 가 RobotInfo / capabilities union 을 자체
// 선언하지 않음. 새 field / capability 추가 시 backend Pydantic + Literal
// 한 번 갱신 → gen:types 재실행 → frontend 가 자동으로 새 모양 받음.
export type RobotInfo = components["schemas"]["RobotInfo"];
export type RobotBasePose = components["schemas"]["BasePoseSchema"];
export type RobotsListResponse = components["schemas"]["RobotsListResponse"];
export type RobotCapability = RobotInfo["capabilities"][number];

let cached: RobotsListResponse | null = null;
let pending: Promise<RobotsListResponse> | null = null;

async function fetchRobots(): Promise<RobotsListResponse> {
  if (cached) return cached;
  if (pending) return pending;
  pending = fetch(`${BASE_URL}/robots`)
    .then((r) => {
      if (!r.ok) throw new Error(`/robots ${r.status}`);
      return r.json() as Promise<RobotsListResponse>;
    })
    .then((data) => {
      cached = data;
      // BridgeClient 의 default robot 도 backend 와 일치시킴 — N=1 호환 코드
      // (multi_robot_phase2_frontend.md §4 결정 3).
      if (data.default) bridge.setDefaultRobotId(data.default);
      return data;
    })
    .finally(() => {
      pending = null;
    });
  return pending;
}

/**
 * robots.yaml SSOT 를 backend `/robots` 에서 한 번 fetch — 메뉴 / WorldScene
 * enumeration source. backend 재시작 전까지는 변하지 않으니 module-scoped 캐시.
 */
export function useRobots(): {
  robots: RobotInfo[];
  defaultId: string;
  loading: boolean;
  error: string | null;
} {
  const [data, setData] = useState<RobotsListResponse | null>(cached);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    if (cached) return;
    let cancelled = false;
    fetchRobots()
      .then((d) => {
        if (!cancelled) setData(d);
      })
      .catch((e: Error) => {
        if (!cancelled) setError(e.message);
      });
    return () => {
      cancelled = true;
    };
  }, []);

  return {
    robots: data?.robots ?? [],
    defaultId: data?.default ?? DEFAULT_ROBOT_ID,
    loading: data === null && error === null,
    error,
  };
}
