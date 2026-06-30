/**
 * `useResource` — backend HTTP endpoint declarative fetch + module cache.
 *
 *   const { robots } = useResource<RobotsResponse>("/robots").data ?? {};
 *
 *   const { data: metrics } = useResource<SystemMetrics>("/system", { poll: 5000 });
 *
 * - module-scoped cache — 동일 path 호출 자동 sync (cross-component)
 * - select 로 derived 변환 (memo 불필요)
 * - poll option — 주기 갱신
 * - refetch() — 명시적 갱신
 */
import { useCallback, useEffect, useMemo, useState } from "react";
import { BASE_URL } from "@/constants";

type Listener = () => void;

interface CacheEntry<T> {
  data: T | null;
  error: string | null;
  loading: boolean;
  pending: Promise<T> | null;
  listeners: Set<Listener>;
}

const cache = new Map<string, CacheEntry<unknown>>();

function getEntry<T>(path: string): CacheEntry<T> {
  let e = cache.get(path) as CacheEntry<T> | undefined;
  if (!e) {
    e = {
      data: null,
      error: null,
      loading: false,
      pending: null,
      listeners: new Set(),
    };
    cache.set(path, e as CacheEntry<unknown>);
  }
  return e;
}

function notify<T>(entry: CacheEntry<T>) {
  for (const l of entry.listeners) l();
}

async function fetchResource<T>(path: string, force = false): Promise<void> {
  const entry = getEntry<T>(path);
  if (!force && (entry.data !== null || entry.pending)) {
    if (entry.pending) await entry.pending.catch(() => undefined);
    return;
  }
  entry.loading = true;
  notify(entry);
  entry.pending = (async () => {
    const r = await fetch(`${BASE_URL}${path}`);
    if (!r.ok) {
      const err = await r.json().catch(() => null);
      throw new Error(err?.error || `${path} ${r.status}`);
    }
    return (await r.json()) as T;
  })();
  try {
    entry.data = await entry.pending;
    entry.error = null;
  } catch (e) {
    entry.error = (e as Error).message;
  } finally {
    entry.loading = false;
    entry.pending = null;
    notify(entry);
  }
}

export interface UseResourceReturn<S> {
  data: S | null;
  loading: boolean;
  error: string | null;
  refetch: () => Promise<void>;
}

export interface ResourceOptions<T, S = T> {
  poll?: number;
  select?: (raw: T) => S;
}

export function useResource<T, S = T>(
  path: string,
  options?: ResourceOptions<T, S>,
): UseResourceReturn<S> {
  const [, setVersion] = useState(0);
  const selectFn = options?.select;
  const poll = options?.poll;

  useEffect(() => {
    const entry = getEntry<T>(path);
    const listener: Listener = () => setVersion((v) => v + 1);
    entry.listeners.add(listener);
    if (entry.data === null && !entry.pending && !entry.loading) {
      void fetchResource<T>(path);
    }
    let timer: ReturnType<typeof setInterval> | null = null;
    if (poll && poll > 0) {
      timer = setInterval(() => void fetchResource<T>(path, true), poll);
    }
    return () => {
      entry.listeners.delete(listener);
      if (timer) clearInterval(timer);
    };
  }, [path, poll]);

  const entry = getEntry<T>(path);
  const selected = useMemo<S | null>(() => {
    if (entry.data == null) return null;
    return selectFn ? selectFn(entry.data) : (entry.data as unknown as S);
  }, [entry.data, selectFn]);

  const refetch = useCallback(async () => {
    await fetchResource<T>(path, true);
  }, [path]);

  return {
    data: selected,
    loading: entry.loading,
    error: entry.error,
    refetch,
  };
}

// test 자리 cross-test isolation — cache map 비우는 자리.
export function _resetResourceCache(): void {
  cache.clear();
}
