import { useResource } from "@/framework";
import type { RobotInfo, RobotsResponse } from "@/api/generated/contract";

/**
 * backend_v2 `/robots` 의 RobotConfig list. multi-robot enumeration source.
 * useResource module cache 가 cross-component sync.
 *
 * backend_v2 의 RobotInfo 는 enabled field 박지 않음 — robots.yaml SSOT
 * (포함된 robot 모두 active). "기본 로봇" 개념 없음 — robot 은 라우트/ task 바인딩
 * 에서 명시적으로 온다.
 */
export function useRobots(): {
  robots: RobotInfo[];
  loading: boolean;
  error: string | null;
} {
  const { data, loading, error } = useResource<RobotsResponse>("/robots");
  return {
    robots: data?.robots ?? [],
    loading,
    error,
  };
}
