import { useEffect, useState } from "react";
import { BASE_URL, DEFAULT_ROBOT_ID } from "@/constants";
import { bridge } from "@/api/bridge";

export interface RobotBasePose {
  x: number;
  y: number;
  z: number;
  yaw_deg: number;
}

export interface RobotInfo {
  id: string;
  type: string;
  enabled: boolean;
  base_pose: RobotBasePose;
  urdf_url: string;
}

interface RobotsResponse {
  robots: RobotInfo[];
  default: string | null;
}

let cached: RobotsResponse | null = null;
let pending: Promise<RobotsResponse> | null = null;

async function fetchRobots(): Promise<RobotsResponse> {
  if (cached) return cached;
  if (pending) return pending;
  pending = fetch(`${BASE_URL}/robots`)
    .then((r) => {
      if (!r.ok) throw new Error(`/robots ${r.status}`);
      return r.json() as Promise<RobotsResponse>;
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
  const [data, setData] = useState<RobotsResponse | null>(cached);
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
