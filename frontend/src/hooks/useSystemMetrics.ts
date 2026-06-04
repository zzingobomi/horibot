import { useEffect, useState } from "react";
import { BASE_URL } from "@/constants";

export interface SystemMetrics {
  cpu_pct: number;
  mem_used_mb: number;
  mem_total_mb: number;
  mem_pct: number;
  zenoh_routers: number;
  zenoh_peers: number;
}

const POLL_INTERVAL_MS = 5000;

/**
 * backend `/system` 폴링 (5초). Dashboard 의 host metric 표시 source.
 * cached 안 함 — *현재* 값이라 갱신되어야.
 */
export function useSystemMetrics(): {
  metrics: SystemMetrics | null;
  error: string | null;
} {
  const [metrics, setMetrics] = useState<SystemMetrics | null>(null);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    let cancelled = false;
    let timer: ReturnType<typeof setInterval> | null = null;

    async function tick() {
      try {
        const r = await fetch(`${BASE_URL}/system`);
        if (!r.ok) throw new Error(`/system ${r.status}`);
        const data = (await r.json()) as SystemMetrics;
        if (!cancelled) {
          setMetrics(data);
          setError(null);
        }
      } catch (e) {
        if (!cancelled) setError((e as Error).message);
      }
    }

    tick();
    timer = setInterval(tick, POLL_INTERVAL_MS);

    return () => {
      cancelled = true;
      if (timer) clearInterval(timer);
    };
  }, []);

  return { metrics, error };
}
