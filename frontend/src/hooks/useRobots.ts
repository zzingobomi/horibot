import { useEffect } from "react";
import { useResource } from "@/framework";
import { DEFAULT_ROBOT_ID } from "@/constants";
import { bridge } from "@/api/bridge";
import type { RobotInfo, RobotsListResponse } from "@/types/robot";

let defaultRobotIdSet = false;

/**
 * robots.yaml SSOT 를 backend `/robots` 에서 fetch — 메뉴 / WorldScene
 * enumeration source. `useResource` module cache 가 cross-component sync.
 */
export function useRobots(): {
  robots: RobotInfo[];
  defaultId: string;
  loading: boolean;
  error: string | null;
} {
  const { data, loading, error } = useResource<RobotsListResponse>("/robots");

  // 첫 응답 시 BridgeClient default robot id 갱신 — N=1 호환 자리.
  useEffect(() => {
    if (data?.default && !defaultRobotIdSet) {
      defaultRobotIdSet = true;
      bridge.setDefaultRobotId(data.default);
    }
  }, [data?.default]);

  return {
    robots: data?.robots ?? [],
    defaultId: data?.default ?? DEFAULT_ROBOT_ID,
    loading,
    error,
  };
}
