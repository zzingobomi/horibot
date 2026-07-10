/**
 * `useMirror` — backend §3.3 Mirror[T] frontend 등가.
 *
 * frontend.md §3.4 + §7 — mount 시 snapshot fetch + change event 도착 시
 * 재호출 (payload 박지 X — invalidate+refetch only, backend §3.3.5 정합).
 *
 *   const cal = useMirror({
 *     snapshotService: ServiceKey.CALIBRATION_SNAPSHOT_BUNDLE,
 *     changeTopic: Topic.CALIBRATION_ACTIVATED,
 *     robotId: "so101_6dof_0",
 *   });
 *
 * Step E (Calibration backend) 박힐 때 활성. first cut 에선 motion 이
 * Mirror 사용 안 함 — 단 framework primitive 박혀있어 활성 시 1줄.
 */
import { useEffect, useState } from "react";
import { bridge } from "@/api/bridge";
import type { ServiceMap, TopicPayloadMap } from "@/api/generated/contract";
import { useTopic } from "./topic";
import { useBridgeConnected } from "./store";

export interface UseMirrorReturn<T> {
  value: T | null;
  isReady: boolean;
}

export interface MirrorConfig<S extends keyof ServiceMap, C extends keyof TopicPayloadMap> {
  snapshotService: S;
  snapshotReq?: ServiceMap[S]["req"];
  changeTopic: C;
  robotId?: string;
}

export function useMirror<
  S extends keyof ServiceMap,
  C extends keyof TopicPayloadMap,
>(config: MirrorConfig<S, C>): UseMirrorReturn<ServiceMap[S]["res"]> {
  type T = ServiceMap[S]["res"];
  const [value, setValue] = useState<T | null>(null);
  const event = useTopic(config.changeTopic, config.robotId);
  const connected = useBridgeConnected();

  // ① mount 1회 — Owner 가 떠 있으면 snapshot 받음. 안 떠 있으면 cache=null 유지.
  // WS 미연결 시 callService 가 drop → timeout — connected 후 fetch.
  // 실패는 graceful (fail-fast 박지 X) 하되 **침묵 금지** — snapshot 미도달이
  // identity fallback 류 조용한 오동작으로 이어진 전례 (2026-07-06 hand_eye).
  useEffect(() => {
    if (!connected) return;
    let cancelled = false;
    (async () => {
      try {
        const res = await bridge.callService(
          config.snapshotService,
          (config.snapshotReq ?? {}) as ServiceMap[S]["req"],
          config.robotId ? { robotId: config.robotId } : undefined,
        );
        if (cancelled) return;
        if (res.success) {
          setValue(res.data as T);
        } else {
          console.warn(
            `[useMirror] snapshot 실패: ${String(config.snapshotService)} — ${res.message}`,
          );
        }
      } catch (e) {
        if (!cancelled) {
          console.warn(
            `[useMirror] snapshot 예외: ${String(config.snapshotService)}`,
            e,
          );
        }
      }
    })();
    return () => {
      cancelled = true;
    };
    // snapshotReq 의도적 제외 — caller 가 inline object 넘기면 매 render identity
    // 바뀌어 refetch loop. snapshot 은 mount/event 시점만 (spec §7).
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [config.snapshotService, config.robotId, connected]);

  // ② change event 도착 → fresh refetch (payload 박지 X)
  useEffect(() => {
    if (event === null) return;
    let cancelled = false;
    (async () => {
      try {
        const res = await bridge.callService(
          config.snapshotService,
          (config.snapshotReq ?? {}) as ServiceMap[S]["req"],
          config.robotId ? { robotId: config.robotId } : undefined,
        );
        if (cancelled) return;
        if (res.success) {
          setValue(res.data as T);
        } else {
          console.warn(
            `[useMirror] refetch 실패: ${String(config.snapshotService)} — ${res.message}`,
          );
        }
      } catch (e) {
        if (!cancelled) {
          console.warn(
            `[useMirror] refetch 예외: ${String(config.snapshotService)}`,
            e,
          );
        }
      }
    })();
    return () => {
      cancelled = true;
    };
    // snapshotReq 의도적 제외 (위 mount effect 와 동일 — refetch loop 차단).
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [event, config.snapshotService, config.robotId]);

  return { value, isReady: value !== null };
}
