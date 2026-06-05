import { useResource } from "@/framework";
import type { SystemMetrics } from "@/types/system";

const POLL_INTERVAL_MS = 5000;

/**
 * backend `/system` 폴링. Dashboard host metric source.
 */
export function useSystemMetrics(): {
  metrics: SystemMetrics | null;
  error: string | null;
} {
  const { data, error } = useResource<SystemMetrics>("/system", {
    poll: POLL_INTERVAL_MS,
  });
  return { metrics: data, error };
}
