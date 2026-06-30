import { useEffect } from "react";
import { useResource } from "@/framework";
import { DEFAULT_ROBOT_ID } from "@/constants";
import { bridge } from "@/api/bridge";
import type { RobotInfo, RobotsResponse } from "@/api/generated/contract";

let defaultRobotIdSet = false;

/**
 * backend_v2 `/robots` 의 RobotConfig list. multi-robot enumeration source.
 * useResource module cache 가 cross-component sync.
 *
 * backend_v2 의 RobotInfo 는 enabled field 박지 않음 — robots.yaml SSOT
 * (포함된 robot 모두 active).
 */
export function useRobots(): {
  robots: RobotInfo[];
  defaultId: string;
  loading: boolean;
  error: string | null;
} {
  const { data, loading, error } = useResource<RobotsResponse>("/robots");

  useEffect(() => {
    if (data?.default && !defaultRobotIdSet) {
      defaultRobotIdSet = true;
      bridge.setDefaultRobotId(data.default as string);
    }
  }, [data?.default]);

  return {
    robots: data?.robots ?? [],
    defaultId: (data?.default as string | undefined) ?? DEFAULT_ROBOT_ID,
    loading,
    error,
  };
}
