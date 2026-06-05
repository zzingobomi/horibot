/**
 * `useService` — typed call + 응답 자동 cache.
 *
 *   const moveJ = useService(ServiceKey.MOTION_MOVE_J);
 *   await moveJ.call({ joints });
 *
 *   const config = useService(ServiceKey.MOTOR_GET_CONFIG);
 *   const torque = config.data?.torque_enabled ?? false;  // cross-component cache
 *
 * `bridge.callService` 가 본 store 에 pending → response 갱신 (transport 자리),
 * 본 hook 은 *reactive view* 만 제공.
 */
import { useCallback } from "react";
import { useFrameworkStore, type ServiceEntry } from "./store";
import { bridge } from "@/api/bridge";
import type { ServiceMap } from "@/api/generated/contract";

export interface UseServiceReturn<K extends keyof ServiceMap> {
  call: (
    req: ServiceMap[K]["req"],
    opts?: { timeoutMs?: number; robotId?: string },
  ) => Promise<ServiceEntry<ServiceMap[K]["res"]>>;
  data: ServiceMap[K]["res"] | null;
  success: boolean;
  message: string;
  pending: boolean;
  timestamp: number;
}

export function useService<K extends keyof ServiceMap>(
  key: K,
  robotId?: string,
): UseServiceReturn<K> {
  const wireKey = bridge.expand(key, robotId);
  const entry = useFrameworkStore(
    (s) =>
      s.serviceData[wireKey] as
        | ServiceEntry<ServiceMap[K]["res"]>
        | undefined,
  );

  const call = useCallback(
    async (
      req: ServiceMap[K]["req"],
      opts?: { timeoutMs?: number; robotId?: string },
    ) => {
      const res = await bridge.callService<K>(key, req, {
        timeoutMs: opts?.timeoutMs,
        robotId: opts?.robotId ?? robotId,
      });
      const wk = bridge.expand(key, opts?.robotId ?? robotId);
      // bridge 가 이미 cache 갱신 — getState() 로 최신 entry read.
      return useFrameworkStore.getState().serviceData[wk] as ServiceEntry<
        ServiceMap[K]["res"]
      >;
    },
    [key, robotId],
  );

  return {
    call,
    data: (entry?.data ?? null) as ServiceMap[K]["res"] | null,
    success: entry?.success ?? false,
    message: entry?.message ?? "",
    pending: entry?.pending ?? false,
    timestamp: entry?.timestamp ?? 0,
  };
}
