/**
 * panelInstanceStore — 살아있는 robot-owned 패널 인스턴스의 runtime 목록.
 *
 * scenePart 메커니즘([docs/frontend.md])의 경계 데이터:
 * dockview 패널 트리 ↔ R3F Canvas 트리 사이를 runtime 에 넘는 것은 JSX 가 아니라
 * **이 인스턴스 목록(순수 데이터)뿐**이다. withRobotOwnership HOC(모든 robot-owned
 * 인스턴스의 mount/robotId/capability 를 아는 chokepoint)가 등록/해제하고,
 * Canvas 안 ScenePartHost 가 소비해 scene 컴포넌트를 인스턴스별 마운트한다.
 *
 * runtime 파생 상태 — layout localStorage 에 영속하지 않음 (config vs runtime).
 */
import { create } from "zustand";

export interface PanelInstance {
  /** registry PANEL_COMPONENTS key — ScenePartHost 가 scene 컴포넌트 lookup */
  panelKind: string;
  /** 이 인스턴스가 바인딩한 robot ([[robot_ownership_model]] — 패널이 소유) */
  robotId: string;
}

interface PanelInstanceState {
  instances: Record<string, PanelInstance>;
  register: (key: string, inst: PanelInstance) => void;
  unregister: (key: string) => void;
}

export const usePanelInstanceStore = create<PanelInstanceState>((set) => ({
  instances: {},
  register: (key, inst) =>
    set((s) => {
      const prev = s.instances[key];
      // 동일 값 재등록 no-op — 불안정 ref 로 인한 Canvas 재렌더 방지 (R3F 규율)
      if (prev && prev.panelKind === inst.panelKind && prev.robotId === inst.robotId) {
        return s;
      }
      return { instances: { ...s.instances, [key]: inst } };
    }),
  unregister: (key) =>
    set((s) => {
      if (!(key in s.instances)) return s;
      const next = { ...s.instances };
      delete next[key];
      return { instances: next };
    }),
}));
