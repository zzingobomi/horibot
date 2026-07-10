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
  const [outOfOrderCount, setOutOfOrderCount] = useState(0);

  // seq 는 최신 value 에서 derive — 별도 state 두고 effect 에서 setSeq 하면
  // cascading render (react-hooks/set-state-in-effect). out-of-order 누적만
  // effect 가 ref 비교로 검출 (accumulator → functional update).
  const rawSeq = (value as { seq?: number } | null)?.seq;
  const seq = typeof rawSeq === "number" ? rawSeq : -1;

  useEffect(() => {
    if (typeof rawSeq !== "number") return;
    if (rawSeq < lastSeqRef.current) {
      setOutOfOrderCount((c) => c + 1);
      console.warn(
        `[useStream] ${String(topic)} seq 역행: ${lastSeqRef.current} -> ${rawSeq}`,
      );
    }
    lastSeqRef.current = rawSeq;
  }, [rawSeq, topic]);

  const ts = (value as { timestamp_unix?: number } | null)?.timestamp_unix;
  // lag = render 시점 wall-clock 의존이 의도 (시간 경과 자체가 staleness). spec §6 가
  // Date.now() in render 를 명시 — purity rule 의 "unstable result" 경고가 여기선
  // 정확히 목적이므로 의도적으로 허용.
  // eslint-disable-next-line react-hooks/purity
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
