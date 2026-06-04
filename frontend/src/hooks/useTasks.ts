import { useEffect, useState } from "react";
import { BASE_URL } from "@/constants";

interface TasksResponse {
  tasks: string[];
}

let cached: TasksResponse | null = null;
let pending: Promise<TasksResponse> | null = null;

async function fetchTasks(): Promise<TasksResponse> {
  if (cached) return cached;
  if (pending) return pending;
  pending = fetch(`${BASE_URL}/tasks`)
    .then((r) => {
      if (!r.ok) throw new Error(`/tasks ${r.status}`);
      return r.json() as Promise<TasksResponse>;
    })
    .then((data) => {
      cached = data;
      return data;
    })
    .finally(() => {
      pending = null;
    });
  return pending;
}

/**
 * backend `/tasks` (TASK_REGISTRY SSOT) 한 번 fetch. backend 재시작 전까지
 * 안 변하니 module-scoped 캐시.
 */
export function useTasks(): {
  tasks: string[];
  loading: boolean;
  error: string | null;
} {
  const [data, setData] = useState<TasksResponse | null>(cached);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    if (cached) return;
    let cancelled = false;
    fetchTasks()
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
    tasks: data?.tasks ?? [],
    loading: data === null && error === null,
    error,
  };
}
