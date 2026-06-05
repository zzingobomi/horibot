/**
 * `useTopic` (declarative read) + `onTopic` (비즈니스 등록).
 *
 *   const joints = useTopic(Topic.MOTOR_STATE_JOINT)?.joints ?? [];
 *
 *   onTopic(Topic.SYSTEM_LOG, (log) => {
 *     useSystemStore.getState().addLog(log);
 *   });
 *
 * 둘 다 generated `TopicPayloadMap[K]` 로 자동 typed.
 */
import { useFrameworkStore } from "./store";
import { bridge, topicFor } from "@/api/bridge";
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
