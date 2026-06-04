/**
 * Task step result 누적 store.
 *
 * Backend `horibot/task/step_result` 토픽을 받아 step.id → payload 매핑으로 보관.
 * TaskResultLayer 가 이 store 를 읽어 type 별 (Detection / Position3 / Pose6 / ...)
 * 자동 렌더링. 새 task tree 가 들어오면 누적 결과 클리어 (useBridge 가 호출).
 *
 * payload.value 는 Backend 의 dataclass 를 그대로 직렬화한 모양:
 *   - Detection: { position: Position3, height, base_z, confidence, prompt }
 *   - Position3: { x, y, z }
 *   - Pose6:     { position: Position3, orientation: Quaternion | null }
 *   - None:      null (사이드이펙트만 있는 step — MoveTCP/Gripper/...)
 */
import { create } from "zustand";

export interface StepResultPayload {
  step_id: string;
  type: string; // backend StepResult.type_name
  value: unknown;
}

interface TaskResultStore {
  results: Record<string, StepResultPayload>;
  setStepResult: (payload: StepResultPayload) => void;
  clearAll: () => void;
}

export const useTaskResultStore = create<TaskResultStore>((set) => ({
  results: {},
  setStepResult: (payload) =>
    set((state) => ({
      results: { ...state.results, [payload.step_id]: payload },
    })),
  clearAll: () => set({ results: {} }),
}));
