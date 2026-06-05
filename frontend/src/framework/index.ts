/**
 * Frontend framework — backend pydantic / api_contract SSOT 가 frontend 의
 * 모든 자리에 자동 propagate. 사용처는 *비즈니스만*, 1줄.
 *
 * FastAPI 의 `@app.get(...)` 식 우아함을 frontend 에 미러:
 *
 * ```tsx
 * function MotionPanel() {
 *   const traj = useTopic(Topic.MOTION_STATE_TRAJ);
 *   const tcp = useService(ServiceKey.MOTION_GET_TCP);
 *   const moveJ = useService(ServiceKey.MOTION_MOVE_J);
 *   const robots = useResource<RobotsListResponse>("/robots");
 *   ...
 * }
 * ```
 *
 * 새 entity 추가 시 frontend 자리:
 *   - 새 토픽 read     → `useTopic(Topic.X)` 1줄
 *   - 새 토픽 비즈니스 → `onTopic(Topic.X, handler)` 1줄 (domain/handlers.ts)
 *   - 새 서비스        → `useService(Key.X)` 1줄 (응답 auto-cache)
 *   - 새 HTTP endpoint → `useResource<T>("/path")` 1줄
 *   - 새 robot         → `robots.yaml` 1줄. frontend 코드 0.
 *
 * hand-write type / store slot / setter / subscribe / cast = 0.
 */
export { useTopic, onTopic } from "./topic";
export { useService, type UseServiceReturn } from "./service";
export {
  useResource,
  type UseResourceReturn,
  type ResourceOptions,
} from "./resource";
export { useFrameworkBootstrap, onConnect } from "./bootstrap";
export { useBridgeConnected, type ServiceEntry } from "./store";
