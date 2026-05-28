import type { Vec3 } from "./motion";

export type TaskStatus =
  | "idle"
  | "running"
  | "paused"
  | "success"
  | "failed"
  | "stopped";

export type StepStatus = "pending" | "running" | "completed" | "failed";

// Backend step_types.py 의 dataclass 들을 그대로 직렬화한 모양.
// 모든 step 이 공유하는 공통 필드만 명시 — type 별 추가 필드는 인덱스 시그니처로.
export interface StepNode {
  id: string;
  type: string;        // "move_tcp" | "gripper" | "grounded_detect" | ...
  label: string;
  // 미래 ForEach/If 도입 시 children: StepNode[] 추가. 지금은 평면 list 이므로
  // 모든 step 이 leaf (children 부재).
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

// Backend core/gripper_setup.py 의 GripperSetup 과 동기. None 필드는 backend
// 의 default 사용 (서비스 호출 시 보내지 않으면 됨).
export interface GripperSetupPayload {
  close_current?: number;
  open_position?: number;
  close_position?: number;
  held_threshold?: number;
}

export interface RunTaskRequest {
  task: string;
  // pick_and_place / self_play_pick 공용. task별 필요 필드만 채움.
  place_position?: Vec3;
  prompt?: string;
  // self_play_pick 전용.
  max_attempts?: number;
  gripper_setup?: GripperSetupPayload;
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
