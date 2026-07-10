/**
 * frontend framework — backend 의 4 primitive 를 React hook 으로 노출.
 *
 * frontend.md §3 — 6 hook:
 *   - useService — RPC + auto cache
 *   - useTopic / onTopic — generic latest cache
 *   - useResource — HTTP fetch + cache + poll
 *   - useStream — Stream seq/lag invariant (Step F3)
 *   - useMirror — snapshot + invalidate+refetch (Step F3)
 *   - useCapability — boot 1회 snapshot (Step F3)
 *
 * 새 backend module 추가 시 frontend 자리:
 *   - 새 stream    → `useStream(Topic.X)` 1줄 (seq monotonic + lag 자동)
 *   - 새 event     → `useTopic(Topic.X)` 또는 `useMirror({...})` 1줄
 *   - 새 service   → `useService(ServiceKey.X)` 1줄
 *   - 새 capability → `useCapability(ServiceKey.X)` 1줄
 *   - 새 HTTP endpoint → `useResource<T>("/path")` 1줄
 *   - 새 robot     → `robots.yaml` 1줄. frontend 코드 0.
 */
export { useTopic, onTopic } from "./topic";
export { useService, type UseServiceReturn } from "./service";
export {
  useResource,
  type UseResourceReturn,
  type ResourceOptions,
} from "./resource";
export { useStream, type UseStreamReturn } from "./stream";
export { useMirror, type UseMirrorReturn, type MirrorConfig } from "./mirror";
export { useCapability, type UseCapabilityReturn } from "./capability";
export { useFrameworkBootstrap, onConnect } from "./bootstrap";
export { useBridgeConnected, type ServiceEntry } from "./store";
