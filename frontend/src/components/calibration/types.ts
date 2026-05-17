export type HandEyePreview = {
  timestamp: number;
  detected: boolean;
  image_size?: [number, number];
  corners?: [number, number][];
  bbox?: [number, number, number, number];
  coverage_ratio?: number;
  reason?: string;
};

export type PoseMeta = {
  index: number;
  timestamp: number;
  joint_angles_rad: number[];
};
