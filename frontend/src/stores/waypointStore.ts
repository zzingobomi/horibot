/**
 * waypointStore — WaypointPanel(UI) ↔ WaypointScenePart(Canvas) 브리지.
 *
 * ghost 미리보기: 패널에서 waypoint 의 [보기] 버튼(명시적 토글 — hover X)을 누르면
 * 그 joint 자세의 반투명 로봇이 3D 에 뜬다. 패널 로컬 선택 상태는 트리 경계를 못
 * 넘으므로 store 로 공유 (scanStore 와 같은 패턴).
 *
 * per-robot 키 — robot A/B 패널이 각자 자기 ghost. 같은 robot 패널 2개는
 * last-write-wins (동시 미리보기 2개는 의미 없음). 패널 unmount 시 자기 robot
 * 의 preview clear (scenePart 도 함께 사라지므로 방어적 정리).
 */
import { create } from "zustand";

export interface GhostPreview {
  /** 토글/하이라이트 판정용 waypoint row id */
  waypointId: number;
  name: string;
  jointNames: string[];
  jointAngles: number[];
}

interface WaypointState {
  previews: Record<string, GhostPreview | undefined>;
  setPreview: (robotId: string, preview: GhostPreview | null) => void;
}

export const useWaypointStore = create<WaypointState>((set) => ({
  previews: {},
  setPreview: (robotId, preview) =>
    set((s) => ({
      previews: { ...s.previews, [robotId]: preview ?? undefined },
    })),
}));
