// backend wire endpoint.
// (기본 로봇 상수 없음 — robot 은 라우트 param(useRobotId) 또는 task 바인딩
//  (task 페이지 소유 상수)에서 명시적으로 온다. ambient default 로봇 개념 폐기.)

// bridge host 기본값 = **페이지를 서빙한 host** — 같은 내부망의 다른 기기
// (맥북 등)에서 http://<PC IP>:5173 으로 열면 localhost 는 그 기기 자신이라
// bridge 에 못 닿아 로봇 목록이 빈다 (2026-07-17 실사고). 같은 PC 접속이면
// hostname 이 localhost 라 기존과 동일. backend bridge 는 이미 LAN-ready
// (bind 0.0.0.0 + CORS *). env 는 예외 토폴로지(bridge 가 다른 머신) override.
const bridgeHost =
  typeof window !== "undefined" && window.location.hostname
    ? window.location.hostname
    : "localhost";

export const BASE_URL =
  import.meta.env.VITE_BASE_URL ?? `http://${bridgeHost}:8000`;
export const WS_URL = import.meta.env.VITE_WS_URL ?? `ws://${bridgeHost}:8000/ws`;
