import type { Vector3Tuple } from "three";

export type TaskStatus =
  | "idle"
  | "running"
  | "paused"
  | "success"
  | "failed"
  | "stopped";

export type StepStatus = "pending" | "running" | "completed" | "failed";

// Backend modules/task/step.py 의 step_to_dict 형식을 그대로 받음.
// type 은 step 클래스 이름 (PascalCase: "MoveTCP", "GroundedDetect", ...).
// Slot 필드는 {step_id: string} 으로 직렬화됨 — frontend 가 producer step 을 lookup 가능.
export interface StepNode {
  id: string;
  type: string;
  label: string;
  // stage 2 의 ForEach/If 도입 시 children: StepNode[] 추가. 지금은 평면 list.
  children?: StepNode[];
  // step type 별 파라미터 — 디버그용 표시 외에는 직접 접근 안 함.
  [key: string]: unknown;
}

export interface TaskTree {
  task_name: string;
  description: string;
  steps: StepNode[];
}

export interface TaskState {
  status: TaskStatus;
  task_name: string;
  current_step: number; // 1-based, 0이면 아직 시작 전
  total_steps: number;
  current_label: string;
  current_step_id: string;
  error: string | null;
  step_statuses: Record<string, StepStatus>;
  breakpoints: string[];
}

export interface RunTaskRequest {
  task: string;
  place_position?: Vector3Tuple;
  prompt?: string;
}

export const defaultTaskState: TaskState = {
  status: "idle",
  task_name: "",
  current_step: 0,
  total_steps: 0,
  current_label: "",
  current_step_id: "",
  error: null,
  step_statuses: {},
  breakpoints: [],
};

export const defaultTaskTree: TaskTree = {
  task_name: "",
  description: "",
  steps: [],
};
