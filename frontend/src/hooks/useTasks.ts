import { useResource } from "@/framework";
import type { TasksResponse } from "@/types/system";

/**
 * backend `/tasks` (TASK_REGISTRY SSOT) fetch. 재시작 전까지 안 변하므로
 * `useResource` module cache 가 충분.
 */
export function useTasks(): {
  tasks: string[];
  loading: boolean;
  error: string | null;
} {
  const { data, loading, error } = useResource<TasksResponse>("/tasks");
  return { tasks: data?.tasks ?? [], loading, error };
}
