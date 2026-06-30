/**
 * Framework bootstrap — App.tsx 에 1회.
 *
 *   function App() {
 *     useFrameworkBootstrap();
 *     ...
 *   }
 *
 * 흐름:
 * - bridge.connect: mount 1회 (re-mount 시 _resubscribeAll 으로 복원)
 * - robot-scoped subscribe: `useRobots()` 의 robots 변경마다 갱신.
 *   `Topic` 값에 `{robot_id}` placeholder 있는 토픽은 robot 마다 별도 subscribe.
 *   global (placeholder 없음) 은 1회.
 * - onConnect: 연결 시점 호출 (재연결 시 매번).
 *
 * frontend_v2.md §2.1 SSOT — backend `/robots` 의 robots 목록만 구독.
 * backend_v2 의 RobotInfo 는 enabled field 박지 X — 모든 robot active 자체.
 */
import { useEffect } from "react";
import { bridge, topicFor } from "@/api/bridge";
import { Topic } from "@/api/generated/contract";
import { useRobots } from "@/hooks/useRobots";
import { useFrameworkStore } from "./store";
import { topicHandlers } from "./topic";

const connectHandlers: Array<() => void> = [];

export function onConnect(handler: () => void): void {
  connectHandlers.push(handler);
}

export function useFrameworkBootstrap(): void {
  const { robots } = useRobots();

  // (1) bridge 연결 — mount 1회.
  useEffect(() => {
    bridge.connect((connected) => {
      useFrameworkStore.getState().setBridgeConnected(connected);
      if (connected) {
        for (const h of connectHandlers) h();
      }
    });
    return () => bridge.disconnect();
  }, []);

  // (2) robot-scoped 토픽 subscribe — robots 변경마다 갱신.
  useEffect(() => {
    const unsubs: Array<() => void> = [];

    for (const tpl of Object.values(Topic)) {
      const isRobotScoped = tpl.includes("{robot_id}");
      if (isRobotScoped) {
        for (const r of robots) {
          const wire = topicFor(tpl, r.id);
          const robotId = r.id;
          const unsub = bridge.subscribe(wire, (data) => {
            useFrameworkStore.getState().setTopicData(wire, data);
            const handlers = topicHandlers.get(wire);
            if (handlers) {
              for (const h of handlers) h(data, robotId);
            }
          });
          unsubs.push(unsub);
        }
      } else {
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
    }

    return () => unsubs.forEach((u) => u());
  }, [robots]);
}
