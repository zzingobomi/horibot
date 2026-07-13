// backend wire endpoint.
// (기본 로봇 상수 없음 — robot 은 라우트 param(useRobotId) 또는 task 바인딩
//  (task 페이지 소유 상수)에서 명시적으로 온다. ambient default 로봇 개념 폐기.)

export const BASE_URL = import.meta.env.VITE_BASE_URL ?? "http://localhost:8000";
export const WS_URL = import.meta.env.VITE_WS_URL ?? "ws://localhost:8000/ws";
