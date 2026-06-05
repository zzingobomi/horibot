/**
 * Detector latest topic 의 frontend-local hide override.
 *
 * Backend 의 PERCEPTION_GROUNDED_STATE 는 *마지막 publish* 가 항상 latest —
 * "이전 결과 clear" 자리가 backend 에 없음. 새 task 시작 시 이전 detect 결과를
 * 가리려면 frontend-local mask timestamp 가 필요.
 *
 *   - PromptPanel handleRun() → `useDetectorOverride.getState().hideUntil(now)`
 *   - DetectionLayer / CameraFeedPanel → `topic.timestamp > maskBefore` 일 때만 표시
 *
 * Backend 의 새 publish 가 도착하면 (timestamp > maskBefore) 자동으로 다시 노출.
 */
import { create } from "zustand";

interface DetectorOverride {
  /** topic timestamp 가 이 값 이하인 결과는 hide. unit: seconds (backend 와 동일). */
  maskBefore: number;
  /** 현재 시점 이전의 모든 결과 hide. */
  hide: () => void;
}

export const useDetectorOverride = create<DetectorOverride>((set) => ({
  maskBefore: 0,
  hide: () => set({ maskBefore: Date.now() / 1000 }),
}));
