/**
 * Framework bootstrap — App.tsx 에 1회.
 *
 *   function AppContent() {
 *     useFrameworkBootstrap();
 *     ...
 *   }
 *
 * bridge 연결 + 모든 토픽 auto-sub + `onTopic` handler dispatch.
 * `onConnect` 등록자는 *연결 시점* 에 호출 (재연결 시 매번).
 */
import { useEffect } from "react";
import { bridge, topicFor } from "@/api/bridge";
import { Topic, BINARY_TOPICS } from "@/constants/topics";
import { useFrameworkStore } from "./store";
import { topicHandlers } from "./topic";

/** module-scoped — `useFrameworkBootstrap` 가 connect 후 호출. */
const connectHandlers: Array<() => void> = [];

export function onConnect(handler: () => void): void {
  connectHandlers.push(handler);
}

export function useFrameworkBootstrap(): void {
  useEffect(() => {
    bridge.connect((connected) => {
      useFrameworkStore.getState().setBridgeConnected(connected);
      if (connected) {
        for (const h of connectHandlers) h();
      }
    });

    const unsubs: Array<() => void> = [];
    for (const tpl of Object.values(Topic)) {
      if (BINARY_TOPICS.has(tpl)) continue;
      const wire = topicFor(tpl);
      const unsub = bridge.subscribe(wire, (data) => {
        useFrameworkStore.getState().setTopicData(wire, data);
        const handlers = topicHandlers.get(wire);
        if (handlers) {
          for (const h of handlers) h(data, null);
        }
      });
      unsubs.push(unsub);
    }

    return () => {
      unsubs.forEach((u) => u());
      bridge.disconnect();
    };
  }, []);
}
