/**
 * `useCapability` — backend_v2 §7 Capability snapshot (boot 1회 fetch, Mirror X).
 *
 * frontend_v2.md §3.5 — D405 vs USB / 5DOF vs 6DOF / RGB vs DEPTH 자리 UI 가
 * 분기 박을 자리 boot 1회 cache. invalidation cycle 박지 X — static fact.
 *
 *   const motorCap = useCapability(ServiceKey.MOTOR_CAPABILITIES);
 *   if (motorCap.value?.flags.includes("torque_toggle")) showTorqueButton();
 *
 * module-scoped cache (wire key 별) — re-mount 시 fetch 안 함.
 */
import { useEffect, useState } from "react";
import { bridge } from "@/api/bridge";
import type { ServiceMap } from "@/api/generated/contract";
import { useBridgeConnected } from "./store";

interface CapEntry<T> {
  value: T | null;
  loading: boolean;
  error: string | null;
  pending: Promise<void> | null;
  listeners: Set<() => void>;
}

const capCache = new Map<string, CapEntry<unknown>>();

function getEntry<T>(wireKey: string): CapEntry<T> {
  let e = capCache.get(wireKey) as CapEntry<T> | undefined;
  if (!e) {
    e = {
      value: null,
      loading: false,
      error: null,
      pending: null,
      listeners: new Set(),
    };
    capCache.set(wireKey, e as CapEntry<unknown>);
  }
  return e;
}

function notify<T>(entry: CapEntry<T>): void {
  for (const l of entry.listeners) l();
}

async function fetchCapability<T, K extends keyof ServiceMap>(
  key: K,
  wireKey: string,
  robotId?: string,
): Promise<void> {
  const entry = getEntry<T>(wireKey);
  if (entry.pending) {
    await entry.pending.catch(() => undefined);
    return;
  }
  entry.loading = true;
  notify(entry);
  entry.pending = (async () => {
    try {
      const res = await bridge.callService(
        key,
        {} as ServiceMap[K]["req"],
        robotId ? { robotId } : undefined,
      );
      if (res.success) {
        entry.value = res.data as T;
        entry.error = null;
      } else {
        entry.error = res.message || "capability fetch fail";
      }
    } catch (e) {
      entry.error = (e as Error).message;
    } finally {
      entry.loading = false;
    }
  })();
  try {
    await entry.pending;
  } finally {
    entry.pending = null;
    notify(entry);
  }
}

export interface UseCapabilityReturn<T> {
  value: T | null;
  loading: boolean;
  error: string | null;
}

export function useCapability<K extends keyof ServiceMap>(
  key: K,
  options?: { robotId?: string },
): UseCapabilityReturn<ServiceMap[K]["res"]> {
  const wireKey = bridge.expand(key, options?.robotId);
  const connected = useBridgeConnected();
  const [, setVersion] = useState(0);

  useEffect(() => {
    const entry = getEntry<ServiceMap[K]["res"]>(wireKey);
    const listener = () => setVersion((v) => v + 1);
    entry.listeners.add(listener);
    // WS 미연결 시 callService 가 drop → timeout — connected 후 fetch.
    if (connected && entry.value === null && !entry.pending && !entry.loading) {
      void fetchCapability(key, wireKey, options?.robotId);
    }
    return () => {
      entry.listeners.delete(listener);
    };
  }, [wireKey, key, options?.robotId, connected]);

  const entry = getEntry<ServiceMap[K]["res"]>(wireKey);
  return {
    value: entry.value,
    loading: entry.loading,
    error: entry.error,
  };
}

// test cross-isolation
export function _resetCapabilityCache(): void {
  capCache.clear();
}
