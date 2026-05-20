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
 * cv2 알고리즘 간 self-consistency 한 행.
 *
 * 같은 입력을 TSAI/PARK/DANIILIDIS로 각각 풀어 기준 대비 차이를 비교.
 * 진단 보조 (BA가 채택되더라도 입력 노이즈 수준을 가늠하는 용도).
 */
export type MethodCompareEntry = {
  method: string;
  drot_deg: number;
  dt_mm: number;
  ref?: boolean;
};

/**
 * 자세별 잔차 한 줄 — BA가 추정한 보드 포즈 대비 예측값 편차.
 *
 * `excluded=true`이면 outlier 자동 제거 단계에서 빠진 포즈.
 * 이 경우 잔차는 1차 BA 값 (왜 빠졌는지 표시용).
 * 사용자는 캡처/계산/커밋만 함 — 직접 삭제 X.
 */
export type PerPoseResidual = {
  id: number;
  drot_deg: number;
  dt_mm: number;
  excluded: boolean;
};

/**
 * 진단 메시지 — 다음에 무엇을 해야 하는지 안내.
 */
export type CoachMessage = {
  level: "success" | "info" | "warn" | "error";
  text: string;
};

export type CoachReport = {
  verdict: "good" | "needs_work" | "bad";
  messages: CoachMessage[];
  // 백엔드는 axis_distributions(각 축의 분포 메타)도 같이 보내지만 현재 UI는
  // coach 메시지 텍스트만 소비. 시각화/히스토그램 도입 시 타입 추가.
};

/**
 * 한 조인트의 zero offset 보정량 — BA가 FK 일관성을 회복하기 위해 추정한 delta.
 * COMMIT 시 기존 joint_offsets.npz에 cumulative하게 합산되어 저장됨.
 */
export type JointOffsetDelta = {
  motor_id: number;
  offset_deg: number;
  offset_rad: number;
};

/**
 * 한 joint origin의 translation 보정. URDF `<joint><origin xyz/>`에 더할 dx,dy,dz.
 * 확장 BA(41 DOF)에서만 채워짐. mm/m 둘 다 제공해 UI 표시 / 추후 계산에 편리.
 * COMMIT 시 link_offsets.npz에 cumulative 합산 — PybulletSolver는 다음 부팅 시 적용.
 */
export type LinkTransDelta = {
  motor_id: number;
  x_mm: number;
  y_mm: number;
  z_mm: number;
  x_m: number;
  y_m: number;
  z_m: number;
};

/**
 * 한 joint origin의 rotation 보정. URDF `<joint><origin rpy/>`에 적용할 rotation
 * vector (small-angle 가정으로 ZYX 오일러 ≈ rotvec). 확장 BA에서만 채워짐.
 */
export type LinkRotDelta = {
  motor_id: number;
  rx_deg: number;
  ry_deg: number;
  rz_deg: number;
  rx_rad: number;
  ry_rad: number;
  rz_rad: number;
};

/**
 * `CALIB_HANDEYE_COMPUTE` 응답.
 *
 * 파이프라인: cv2.calibrateHandEye(TSAI) seed → scipy Huber-BA → 진단.
 * BA가 outlier에 robust하므로 사용자는 캡처만 함 (포즈 삭제 X).
 */
export type ComputeData = {
  R_cam2gripper: number[][];
  t_cam2gripper: number[];
  method: string; // "BA(huber)" 또는 fallback 명
  ba_converged: boolean;
  ba_message?: string;
  pose_count: number;
  method_compare: MethodCompareEntry[];
  per_pose_residual: PerPoseResidual[];
  excluded_pose_ids: number[];
  // σ는 outlier 제거 후 깨끗한 set의 RMS — 정직한 정확도.
  sigma_rot_deg: number;
  sigma_t_mm: number;
  coach: CoachReport;
  joint_offset_estimated: boolean;
  joint_offset_delta: JointOffsetDelta[];
  // 확장 BA(default)에서만 채워짐. standard fallback이면 estimated=false + 빈 배열.
  link_offset_estimated: boolean;
  link_trans_delta: LinkTransDelta[];
  link_rot_delta: LinkRotDelta[];
  // [계산] 응답에 묶이는 다음 자세 후보 리스트. 사용자가 [이동]→카메라 확인→
  // 보이면 [캡처] 흐름으로 소비. 다음 [계산] 전까지 갱신 X.
  recommendations: NextPoseRecommendation[];
};

/**
 * 다음 자세 후보 한 개. 모든 모터(arm 5개)에 대한 목표 각도 + 압축 라벨/이유.
 *
 * 프론트 표시 흐름:
 *   - 리스트 행 헤드라인: label (예: "J4 위쪽 +20°")
 *   - 행 펼침: reason (긴 설명) + joints 5개 풀 값
 *   - [이동] 클릭: motion/move_j 페이로드로 joints 그대로 전송
 *
 * `diagnostics`는 백엔드 내부 진단용 — 현재 UI 미사용.
 */
export type NextPoseRecommendation = {
  joints: { id: number; degree: number }[];
  reason: string;
  label: string;
  primary_axis: number; // 0..4
  source: "high_residual" | "distribution";
};

/**
 * `CALIB_HANDEYE_THRESHOLDS` 응답 — 백엔드 thresholds.py의 단일 출처.
 * 프론트엔드는 HandEyeTab mount 시 1회 fetch.
 */
export type CalibThresholds = {
  sigma_rot_good_deg: number;
  sigma_t_good_mm: number;
  sigma_rot_warn_deg: number;
  sigma_t_warn_mm: number;
  outlier_mod_z_threshold: number;
  outlier_abs_rot_deg: number;
  outlier_abs_t_mm: number;
  outlier_removal_cap_ratio: number;
  min_poses_for_compute: number;
  recommended_poses: number;
  joint_diversity_threshold_deg: number[];
};
