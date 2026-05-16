export type PoseMeta = {
  index: number;
  timestamp: number;
  joint_angles_rad: number[];
};

export type MethodCompareEntry = {
  method: string;
  drot_deg: number;
  dt_mm: number;
  ref?: boolean;
};

export type PerPoseResidual = {
  index: number;
  drot_deg: number;
  dt_mm: number;
};

export type ComputeData = {
  R_cam2gripper: number[][];
  t_cam2gripper: number[];
  method: string;
  pose_count: number;
  method_compare: MethodCompareEntry[];
  per_pose_residual: PerPoseResidual[];
  sigma_rot_deg: number;
  sigma_t_mm: number;
};

export type ValidateData = {
  source: string;
  pose_count: number;
  per_pose_residual: PerPoseResidual[];
  sigma_rot_deg: number;
  sigma_t_mm: number;
};

export type HandEyePreview = {
  timestamp: number;
  detected: boolean;
  image_size?: [number, number];
  corners?: [number, number][];
  bbox?: [number, number, number, number];
  coverage_ratio?: number;
  reason?: string;
};
