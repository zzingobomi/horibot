import { useResource } from "@/framework";
import type { TaskInfo, TasksResponse } from "@/types/system";

/**
 * backend `/tasks` (TASK_REGISTRY SSOT) fetch. 재시작 전까지 안 변하므로
 * `useResource` module cache 가 충분.
 *
 * 각 TaskInfo 의 required_capabilities 자리 frontend TasksPage 의 robot
 * dropdown filter (rgbd capability robot 만 자체 자리 자리).
 */
export function useTasks(): {
  tasks: TaskInfo[];
  loading: boolean;
  error: string | null;
} {
  const { data, loading, error } = useResource<TasksResponse>("/tasks");
  return { tasks: data?.tasks ?? [], loading, error };
}
