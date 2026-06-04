const isDev = import.meta.env.DEV;
const loc = typeof window !== "undefined" ? window.location : undefined;
const httpProto = loc?.protocol ?? "http:";
const wsProto = httpProto === "https:" ? "wss:" : "ws:";
const hostname = loc?.hostname ?? "localhost";
const host = loc?.host ?? "localhost";

const defaultBase = isDev
  ? `${httpProto}//${hostname}:8000`
  : `${httpProto}//${host}`;
const defaultWs = isDev
  ? `${wsProto}//${hostname}:8000/ws`
  : `${wsProto}//${host}/ws`;

export const BASE_URL = import.meta.env.VITE_BASE_URL || defaultBase;
export const WS_URL = import.meta.env.VITE_WS_URL || defaultWs;

// multi_robot_phase2_frontend.md §4 결정 3 — "store 는 임시로 focus robot 만
// 받는 호환 코드로 N=1 검증". BridgeClient 가 robot-scoped template (`horibot/
// {robot_id}/...`) 호출 시 자동으로 이 값 채움. Slice B 에서 robots.yaml
// fetch + ViewState focus 도입 시 dynamic 화.
export const DEFAULT_ROBOT_ID =
  import.meta.env.VITE_DEFAULT_ROBOT_ID || "omx_f_0";
