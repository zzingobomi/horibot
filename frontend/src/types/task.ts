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
  error: null,
};
