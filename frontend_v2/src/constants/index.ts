// backend_v2 wire endpoint + active robot.
// [[project-active-robot-so101-d405]] — OMX detach, SO-101 + D405 가 현재 활성.

export const BASE_URL = import.meta.env.VITE_BASE_URL ?? "http://localhost:8000";
export const WS_URL = import.meta.env.VITE_WS_URL ?? "ws://localhost:8000/ws";

export const DEFAULT_ROBOT_ID = "so101_6dof_0";
