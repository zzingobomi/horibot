/**
 * Framework internal cache — 토픽 latest + 서비스 응답 + bridge 연결 상태.
 *
 * 모든 사용처는 `useTopic` / `useService` / `useBridgeConnected` 를 거치고,
 * 본 store 는 *내부 구현*. `bridge.ts` 만 callService 응답 자동 cache 위해
 * 직접 import.
 */
import { create } from "zustand";

export interface ServiceEntry<T = unknown> {
  success: boolean;
  message: string;
  data: T;
  timestamp: number;
  pending: boolean;
}

interface FrameworkState {
  topicData: Record<string, unknown>;
  serviceData: Record<string, ServiceEntry>;
  bridgeConnected: boolean;
  setTopicData: (wireTopic: string, value: unknown) => void;
  setServiceData: (wireKey: string, entry: ServiceEntry) => void;
  setBridgeConnected: (connected: boolean) => void;
}

export const useFrameworkStore = create<FrameworkState>((set) => ({
  topicData: {},
  serviceData: {},
  bridgeConnected: false,
  setTopicData: (k, v) =>
    set((s) => ({ topicData: { ...s.topicData, [k]: v } })),
  setServiceData: (k, e) =>
    set((s) => ({ serviceData: { ...s.serviceData, [k]: e } })),
  setBridgeConnected: (connected) => set({ bridgeConnected: connected }),
}));

/** Bridge WebSocket 연결 상태. `useFrameworkBootstrap` 가 갱신. */
export function useBridgeConnected(): boolean {
  return useFrameworkStore((s) => s.bridgeConnected);
}
