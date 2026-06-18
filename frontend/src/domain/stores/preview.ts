/**
 * Ghost preview store — 선택한 후보 pose 1개를 현재 robot model 위에 반투명 미리보기.
 *
 * **공통 primitive (calibration 전용 아님)** — docs/pose_library_design.md §5.2.
 * 후보가 여러 개여도 동시에 다 띄우면 헷갈림 → 사용자가 목록에서 *클릭한 1개*만
 * ghost 로 표시 (화면에 고정 → 토크오프로 수동 매칭). 다시 클릭 = 해제.
 * caller 누구든 (캘 후보 / 추후 MoveJ / pose library) 같은 `setGhost` 호출.
 *
 * joints = URDF rad 배열 (RobotModel.jointAngles 와 같은 단위/순서 = motorCfgs order).
 */
import { create } from "zustand";

interface PreviewStore {
  /** robotId → 선택된 ghost joint 각도 (rad). 없으면 미표시. robot 당 최대 1개. */
  ghosts: Record<string, number[]>;
  setGhost: (robotId: string, joints: number[] | null) => void;
  clearAll: () => void;
}

export const usePreviewStore = create<PreviewStore>((set) => ({
  ghosts: {},
  setGhost: (robotId, joints) =>
    set((s) => {
      const next = { ...s.ghosts };
      if (joints === null) delete next[robotId];
      else next[robotId] = joints;
      return { ghosts: next };
    }),
  clearAll: () => set({ ghosts: {} }),
}));
