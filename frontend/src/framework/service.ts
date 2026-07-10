/**
 * `useService` — typed call + 응답 자동 cache.
 *
 *   const moveJ = useService(ServiceKey.MOTION_MOVE_J);
 *   await moveJ.call({ target_joints });
 *
 *   const cap = useService(ServiceKey.MOTOR_CAPABILITIES);
 *   const torqueToggle = cap.data?.flags.includes("torque_toggle");
 *
 * `bridge.callService` 가 본 store 에 pending → response 갱신, 본 hook 은
 * *reactive view* 만 제공.
 *
 * frontend_v2.md §3.1 — backend_v2 의 exception model 은 bridge.ts shim
 * (type=2 → success:true / type=3 → success:false) 로 옛 `{success, message, data}`
 * shape 유지.
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
      await bridge.callService<K>(key, req, {
        timeoutMs: opts?.timeoutMs,
        robotId: opts?.robotId ?? robotId,
      });
      const wk = bridge.expand(key, opts?.robotId ?? robotId);
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
