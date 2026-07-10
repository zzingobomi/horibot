import { useResource } from "@/framework";
import type { TasksResponse } from "@/api/generated/contract";

/**
 * backend `GET /tasks` — task registry. 각 task 는 자기 참여 robot 을 선언
 * (robot_ids). frontend 는 이 목록으로 task 의 통신 robot 을 정한다 — ambient
 * default 로봇 추측 없음 (task 가 대상 robot 의 SSOT).
 */
export interface TaskInfo {
  name: string;
  robot_ids: string[];
}

export function useTasks(): {
  tasks: TaskInfo[];
  loading: boolean;
  error: string | null;
} {
  const { data, loading, error } = useResource<TasksResponse>("/tasks");
  return {
    tasks: (data?.tasks as TaskInfo[] | undefined) ?? [],
    loading,
    error,
  };
}

/**
 * task 이름 → 첫 참여 robot id (단팔 task 는 1개). 협동 task(robot_ids 여러 개)의
 * 다중 robot 처리는 실제 협동 task 도입 시 별도 설계 — 지금은 단일 robot 진입점.
 * 미로드/미바인딩이면 undefined (호출자가 로딩 처리).
 */
export function useTaskRobotId(taskName: string): string | undefined {
  const { tasks } = useTasks();
  return tasks.find((t) => t.name === taskName)?.robot_ids[0];
}
