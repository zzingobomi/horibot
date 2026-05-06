import type { Vec3 } from "./motion";

export type TaskStatus =
  | "idle"
  | "running"
  | "paused"
  | "success"
  | "failed"
  | "stopped";

export interface TaskState {
  status: TaskStatus;
  task_name: string;
  current_step: number; // 1-based, 0이면 아직 시작 전
  total_steps: number;
  current_label: string;
  error: string | null;
}

export interface RunTaskRequest {
  task: "pick_and_place";
  place_position: Vec3;
}

export const defaultTaskState: TaskState = {
  status: "idle",
  task_name: "",
  current_step: 0,
  total_steps: 0,
  current_label: "",
  error: null,
};
