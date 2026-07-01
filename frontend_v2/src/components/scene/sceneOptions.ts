// Scene 표시 옵션 — Scene.tsx 에서 분리 (컴포넌트 파일이 상수/타입도 export 하면
// React Fast Refresh 가 컴포넌트-only export 가정 깨짐 → HMR 경고).

export interface SceneOptions {
  showRobot: boolean;
  showBaseFrame: boolean;
  showTCPFrame: boolean;
  showGrid: boolean;
}

export const DEFAULT_SCENE_OPTIONS: SceneOptions = {
  showRobot: true,
  showBaseFrame: true,
  showTCPFrame: true,
  showGrid: true,
};
