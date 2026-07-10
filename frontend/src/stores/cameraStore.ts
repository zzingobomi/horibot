/**
 * cameraStore — Camera 씬 객체의 표시 옵션 (패널 UI ↔ Canvas 브리지).
 *
 * frustum 렌더는 Camera 씬 객체([scene/Cameras.tsx]) 한 곳 — 패널(캘/라클 등)은
 * 여기 토글만 만진다 ("월드 것은 패널이 그리지 않고 속성만 제어",
 * [docs/frontend.md]). 여러 패널이 같은 토글을 공유해도
 * 렌더는 카메라당 한 번 — 중복이 구조적으로 불가.
 *
 * **per-robot dict** — robot-owned 패널의 토글은 자기 robot 카메라만 켠다
 * ([[robot_ownership_model]]: 바인딩은 오직 panel.robot). 옛 전역 bool 은
 * omx 캘 패널에서 [시야] 누르면 so101 frustum 이 뜨던 사고 (N=2 에서 파탄).
 */
import { create } from "zustand";

interface CameraState {
  /** robot_id → 시야 frustum 표시 여부 */
  frustum: Record<string, boolean>;
  setFrustum: (robotId: string, on: boolean) => void;
}

export const useCameraStore = create<CameraState>((set) => ({
  frustum: {},
  setFrustum: (robotId, on) =>
    set((s) => ({ frustum: { ...s.frustum, [robotId]: on } })),
}));
