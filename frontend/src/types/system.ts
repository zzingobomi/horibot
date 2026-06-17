/**
 * System 도메인 type.
 *
 * SystemMetrics / TasksResponse / LogMessage 는 backend pydantic (bridge/schemas.py
 * + api_contract messages) 의 `gen:types` 산출물 alias — SSOT, drift 0.
 *
 * NodeInfo / NodeStatus 는 backend wire 가 아니라 frontend 가 SYSTEM_HEARTBEAT
 * 를 받아 store 에 가공/누적하는 결과 type. backend 가 node status 토픽을
 * 발행하는 게 아니므로 hand-write 가 맞음.
 */
import type { components } from "@/api/generated/types";

export type SystemMetrics = components["schemas"]["SystemMetrics"];
export type TasksResponse = components["schemas"]["TasksResponse"];
export type TaskInfo = components["schemas"]["TaskInfo"];
export type LogMessage = components["schemas"]["LogMessage"];

export type NodeStatus = "running" | "error" | "stopped";

export interface NodeInfo {
  name: string;
  status: NodeStatus;
  timestamp: number;
  robotId: string | null;
}
