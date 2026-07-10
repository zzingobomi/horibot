/**
 * cameraStore — Camera 씬 객체의 표시 옵션 (패널 UI ↔ Canvas 브리지).
 *
 * frustum 렌더는 Camera 씬 객체([scene/Cameras.tsx]) 한 곳 — 패널(캘/라클 등)은
 * 여기 토글만 만진다 ("월드 것은 패널이 그리지 않고 속성만 제어",
 * [docs/scene_contribution_architecture.md]). 여러 패널이 같은 토글을 공유해도
 * 렌더는 카메라당 한 번 — 중복이 구조적으로 불가.
 *
 * 현재 rgbd robot N=1 이라 전역 bool. per-camera 제어가 필요해지면 dict[robot_id]
 * 화 (scanStore.liveEnabled 와 같은 경로).
 */
import { create } from "zustand";

interface CameraState {
  /** D405 시야 frustum 표시 여부 */
  showFrustum: boolean;
  setShowFrustum: (b: boolean) => void;
}

export const useCameraStore = create<CameraState>((set) => ({
  showFrustum: false,
  setShowFrustum: (b) => set({ showFrustum: b }),
}));
