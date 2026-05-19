export type HandEyePreview = {
  timestamp: number;
  detected: boolean;
  image_size?: [number, number];
  corners?: [number, number][];
  bbox?: [number, number, number, number];
  coverage_ratio?: number; // 체커보드가 화면에서 차지하는 비율 (너무 작거나 크면 PnP 부정확)
  tilt_deg?: number; // 보드 평면 vs 이미지 평면 각도. 0°=정면(모호), 90°=edge-on. 20~65°가 좋음.
  reason?: string;
};

/**
 * 캡처한 자세 1개의 메타데이터.
 */
export type PoseMeta = {
  id: number;
  timestamp: number;
  joint_angles_rad: number[];
};

/**
 * "알고리즘끼리 결과가 얼마나 일치하나" 표의 한 행.
 *
 * 같은 자세 데이터를 TSAI/PARK/DANIILIDIS로 각각 풀어,
 * 기준(보통 TSAI) 대비 회전/평행이동 차이를 비교한 것.
 *
 * 판정 기준:
 * - 셋 다 Δrot < 1° → 데이터 self-consistent (캘 OK)
 * - 1~3°            → 자세 다양성 부족
 * - > 3°            → 자세 품질 문제 (흔들림 / PnP 실패성 자세)
 */
export type MethodCompareEntry = {
  method: string;
  drot_deg: number; // 기준 대비 회전 차이 (도)
  dt_mm: number; // 기준 대비 평행이동 차이 (mm)
  ref?: boolean; // true면 이 항목이 기준점 (drot/dt는 0)
};

/**
 * 자세별 잔차 한 줄. 값이 크면 outlier 후보 → 개별 삭제 후 재계산.
 * T_base←board의 평균 대비 흩어짐.
 */
export type PerPoseResidual = {
  id: number;
  drot_deg: number;
  dt_mm: number;
};

/**
 * 백엔드 `_diagnose()`의 결과. 다음 액션을 명시적으로 안내.
 *
 * - outlier_present:        한두 포즈가 평균에서 도드라짐 → 삭제 후 재 COMPUTE
 * - insufficient_diversity: joint 1/4/5 회전 범위가 작음 → 자세 추가 캡처
 * - good:                   σ 목표 충족 → COMMIT 권장
 * - fk_floor_reached:       outlier 없음 + 다양성 OK인데 σ 정체 → BA 필요
 */
export type DiagnosisStatus =
  | "outlier_present"
  | "insufficient_diversity"
  | "fk_floor_reached"
  | "good";

export type Diagnosis = {
  status: DiagnosisStatus;
  severity: "action_required" | "success";
  message: string;
  outlier_ids?: number[];
  low_diversity_joints?: number[];
  sigma_rot_deg?: number;
  park_drot_deg?: number;
};

/**
 * `CALIB_HANDEYE_COMPUTE` 응답.
 *
 * "계산만" 수행한 결과 — 파일 저장(COMMIT) 전 미리보기용.
 * 사용자는 이 결과를 보고 outlier 자세를 빼거나 다시 캡처 후 재계산하고,
 * 만족스러우면 별도로 COMMIT을 눌러 hand_eye.npz에 기록.
 */
export type ComputeData = {
  R_cam2gripper: number[][];
  t_cam2gripper: number[];
  method: string;
  pose_count: number;
  method_compare: MethodCompareEntry[];
  per_pose_residual: PerPoseResidual[];
  sigma_rot_deg: number; // T_target←base 회전의 평균 대비 RMS 편차 (도)
  sigma_t_mm: number; // 위치 버전 (mm)
  diagnosis: Diagnosis;
};

/**
 * `CALIB_HANDEYE_COMPUTE_BA` 응답.
 *
 * Bundle Adjustment 결과. joint zero offset 5개 + hand-eye R/t를 동시 최적화한 결과로,
 * cv2.calibrateHandEye(TSAI) seed 대비 σ가 얼마나 줄었는지 비교용 필드(`seed_*`)를 포함.
 */
export type BundleAdjustData = {
  R_cam2gripper: number[][];
  t_cam2gripper: number[];
  joint_offsets_rad: number[];
  joint_offsets_deg: number[];
  method: string;
  pose_count: number;
  sigma_rot_deg: number;
  sigma_t_mm: number;
  per_pose_residual: PerPoseResidual[];
  seed_sigma_rot_deg: number; // BA 이전 (TSAI seed) σ
  seed_sigma_t_mm: number;
  iterations: number;
  cost_initial: number;
  cost_final: number;
  elapsed_sec: number;
  success: boolean;
};

