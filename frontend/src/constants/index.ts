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
