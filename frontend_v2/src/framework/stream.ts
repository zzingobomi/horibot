/**
 * `useStream` — Stream payload 의 seq monotonic + timestamp_unix lag invariant 검사.
 *
 * frontend_v2.md §3.3 + §6 — backend_v2 §8.5 의 stream payload invariant
 * (`seq: int` + `timestamp_unix: float`) 활용. reconnect / lag / out-of-order
 * detection — 옛 frontend 에서 박지 않은 부분.
 *
 *   const tcp = useStream(Topic.MOTION_TCP_STATE);
 *   if (tcp.stale) showLagBadge();
 *
 * invariant 위반 (seq 역행) 시 console.warn + state 노출. fail-fast 박지 X —
 * 첫 박을 때 graceful 처리.
 */
import { useEffect, useRef, useState } from "react";
import { useTopic } from "./topic";
import type { TopicPayloadMap } from "@/api/generated/contract";

export interface UseStreamReturn<T> {
  value: T | null;
  seq: number;
  lagMs: number;
  stale: boolean;
  outOfOrderCount: number;
}

export function useStream<K extends keyof TopicPayloadMap>(
  topic: K,
  options?: { staleMs?: number; robotId?: string },
): UseStreamReturn<TopicPayloadMap[K]> {
  const value = useTopic(topic, options?.robotId);
  // ref — effect 안 stale closure 차단 (comparison 위 prev seq)
  const lastSeqRef = useRef(-1);
  const [seq, setSeq] = useState(-1);
  const [outOfOrderCount, setOutOfOrderCount] = useState(0);

  useEffect(() => {
    if (!value) return;
    const next = (value as { seq?: number }).seq;
    if (typeof next !== "number") return;
    if (next < lastSeqRef.current) {
      setOutOfOrderCount((c) => c + 1);
      console.warn(
        `[useStream] ${String(topic)} seq 역행: ${lastSeqRef.current} -> ${next}`,
      );
    }
    lastSeqRef.current = next;
    setSeq(next);
  }, [value, topic]);

  const ts = (value as { timestamp_unix?: number } | null)?.timestamp_unix;
  const lagMs = typeof ts === "number" ? Date.now() - ts * 1000 : 0;
  const stale = lagMs > (options?.staleMs ?? 500);

  return {
    value,
    seq,
    lagMs,
    stale,
    outOfOrderCount,
  };
}
