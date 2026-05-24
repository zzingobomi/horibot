// SelfPlay 토픽(`omx/self_play/state`) 페이로드 타입. backend
// SelfPlayRunner._publish_state 와 동기.

export type SelfPlayStage =
  | "idle"
  | "starting"
  | "detecting"
  | "hovering"
  | "descending"
  | "closing"
  | "lifting"
  | "dropping"
  | "returning_home"
  | "attempt_done"
  | "stopped"
  | "halted"
  | "done";

export type StageResult =
  | "OK"
  | "SPIKE"
  | "EMPTY"
  | "DROPPED"
  | "SKIPPED"
  | "FAIL";

export interface SelfPlayAttemptResult {
  ts: number;
  attempt_id: number;
  prompt: string;

  target_xyz: [number, number, number] | null;
  detect_base_z: number | null;
  detect_height: number | null;
  grasp_z: number | null;
  detect_retries: number;
  search_pose_used: string | null;

  joint_raw: Record<string, number>;

  s1: StageResult;
  s2: StageResult;
  s3: StageResult;

  spike_joint_id: number | null;
  spike_load: number | null;
  spike_baseline: number | null;
  spike_at_z: number | null;
  gripper_pos_after_close: number | null;
  gripper_pos_after_lift: number | null;

  fail_stage: number | null;
  note: string;
}

export interface SelfPlayStats {
  total: number;
  success: number;
  s1_pass: number;
  s2_pass: number;
  s3_pass: number;
}

export interface SelfPlayState {
  prompt: string;
  attempt_id: number;
  max_attempts: number;
  current_stage: SelfPlayStage;
  last_result: SelfPlayAttemptResult | null;
  stats: SelfPlayStats;
}

export const defaultSelfPlayState: SelfPlayState = {
  prompt: "",
  attempt_id: 0,
  max_attempts: 0,
  current_stage: "idle",
  last_result: null,
  stats: { total: 0, success: 0, s1_pass: 0, s2_pass: 0, s3_pass: 0 },
};
