/**
 * `useTopic` (declarative read) + `onTopic` (비즈니스 등록).
 *
 *   const tcp = useTopic(Topic.MOTION_TCP_STATE);
 *
 *   onTopic(Topic.SYSTEM_LOG, (log) => {
 *     useSystemStore.getState().addLog(log);
 *   });
 *
 * generated `TopicPayloadMap[K]` 로 자동 typed.
 *
 * frontend.md §3.2 — generic latest-cache. Stream invariant (seq / lag) 검사는
 * `useStream` 자리. event 의 *event-driven refetch* 는 `useMirror` 자리.
 */
import { useFrameworkStore } from "./store";
import { topicFor } from "@/api/bridge";
import type { TopicPayloadMap } from "@/api/generated/contract";

type GenericHandler = (msg: unknown, robotId: string | null) => void;

/** module-scoped — `bootstrap` 가 토픽 도착 시 dispatch. */
export const topicHandlers = new Map<string, GenericHandler[]>();

export function onTopic<K extends keyof TopicPayloadMap>(
  topic: K,
  handler: (msg: TopicPayloadMap[K], robotId: string | null) => void,
  robotId?: string,
): void {
  const wire = topicFor(topic, robotId);
  const arr = topicHandlers.get(wire) ?? [];
  arr.push(handler as GenericHandler);
  topicHandlers.set(wire, arr);
}

export function useTopic<K extends keyof TopicPayloadMap>(
  topic: K,
  robotId?: string,
): TopicPayloadMap[K] | null {
  const wire = topicFor(topic, robotId);
  return useFrameworkStore(
    (s) => (s.topicData[wire] ?? null) as TopicPayloadMap[K] | null,
  );
}
