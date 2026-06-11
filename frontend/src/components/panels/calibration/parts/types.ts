export type HandEyePreview = {
  timestamp: number;
  detected: boolean;
  image_size?: [number, number];
  corners?: [number, number][]; // ChArUco inner corner (chessboard 교차점)
  markers?: { corners: [number, number][]; id: number }[]; // ArUco marker quad outline + ID
  bbox?: [number, number, number, number];
  coverage_ratio?: number; // 체커보드가 화면에서 차지하는 비율 (너무 작거나 크면 PnP 부정확)
  tilt_deg?: number; // 보드 평면 vs 이미지 평면 각도. 0°=정면(모호), 90°=edge-on. 30~70°가 좋음.
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

/**
 * coach.py 의 axis 분포 메타 한 행. low_diversity=true 면 *그 축 자세 다양성 부족*.
 * "narrow_sigma_good" verdict 의 dominant 원인이라 UI 표 + 색깔 표시 핵심.
 */
export type AxisDistribution = {
  motor_id: number;
  name_ko: string;
  std_deg: number;
  min_deg: number;
  max_deg: number;
  threshold_deg: number;
  is_low_diversity: boolean;
  motor_limit_min_deg: number;
  motor_limit_max_deg: number;
  suggested_deg: number | null;
  suggestion_text: string;
};

export type CoachVerdict =
  | "good" // σ pass + 자세 다양성 충족 → 초록, COMMIT 권장
  | "narrow_sigma_good" // σ pass + 자세 다양성 부족 → 노랑, COMMIT 가능 + 추가 캡처 권장
  | "needs_work" // σ warn → 노랑, 추가 캡처 권장
  | "bad"; // σ bad → 빨강, COMMIT disable

export type CoachReport = {
  verdict: CoachVerdict;
  messages: CoachMessage[];
  axis_distributions?: AxisDistribution[];
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
 * 자세 의존 중력 처짐 sag stiffness — 물리 sag BA(43 DOF)에서만 채워짐.
 *
 * `sag_J = k * τ_J`   where τ_J = (ee_pos - joint_origin) × g · axis  (m 단위)
 * lumped mass 가정이라 k는 (1/stiffness × effective_mass) 비율을 흡수.
 * `max_sag_deg`는 캡처 자세들 중 최대 sag (디버깅/UI 표시용).
 *
 * 현재 J2, J3에만 적용 (DIY 5축에서 중력 부하 가장 큰 두 joint).
 * COMMIT 시 sag_offsets.npz에 cumulative 합산 — PC 메모리 즉시 갱신
 * (Kinematics 재시작 X), 다른 머신은 git pull + 재시작.
 */
export type SagOffsetDelta = {
  motor_id: number;
  k_rad_per_m: number;
  max_sag_deg: number;
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
  // 물리 sag BA(default, mode="physical_sag")에서만 채워짐. extended로 fallback하면 빈 배열.
  sag_offset_estimated: boolean;
  sag_offset_delta: SagOffsetDelta[];
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
  source: "high_residual" | "distribution" | "geometry";
  // backend next_pose_planner.is_pose_visible() 결과. intrinsic + hand_eye +
  // 보드 base 추정 모두 있을 때만 의미 — 아니면 "unchecked".
  visible?: boolean;
  visibility_reason?: string;
  // 명시 신호 [👎] 누를 때 사용할 anchor 식별자. source="geometry" 면 "geometry_<idx>".
  diagnostics?: {
    mode?: string;
    anchor_id?: string;
    anchor_label?: string;
    [k: string]: unknown;
  };
};

/**
 * `CALIB_HANDEYE_SIGMA` topic payload — capture 후 자동 BA / 수동 COMPUTE 마다 publish.
 * frontend σ live 디스플레이용 (작은 footer / inline 표시).
 */
export type HandEyeSigmaState = {
  timestamp: number;
  sigma_rot_deg: number | null;
  sigma_t_mm: number | null;
  pose_count: number;
  ba_mode: string | null;
  ba_converged: boolean;
  coach_verdict: CoachVerdict | null;
  joint_offset_estimated: boolean;
  link_offset_estimated: boolean;
  sag_offset_estimated: boolean;
  /** capture 시점의 axis 분포. low_diversity 인 축이 있으면 narrow_sigma_good verdict 의 원인. */
  axis_distributions?: AxisDistribution[];
};

/**
 * `CALIB_HANDEYE_RECOMMENDATIONS` topic — 매 capture 후 자동 publish.
 * Phase 1 (n<8) frontend hide, Phase 2 (n>=8) show.
 *
 * `no_candidates_reason` ─ 빈 추천 분기를 분리하는 핵심 (§8.7 deferred fix).
 *   "sigma_sufficient_and_diverse" → COMMIT 권장 양성 메시지
 *   "sigma_sufficient_but_narrow"  → 부족 axis 변주 캡처 안내 (axis_distributions 동반)
 *   "all_invisible"                → 추천 자세에서 보드 시야 밖
 *   "all_ik_fail"                  → 추천 자세 IK 불가
 *   "user_marked_fail"             → 사용자가 명시 [👎] 다수
 *   "insufficient_poses"           → 최소 N 캡처 필요
 *   "no_board_estimate"            → hand_eye / intrinsic / 보드 base 추정 X
 */
export type NoCandidatesReason =
  | "sigma_sufficient_and_diverse"
  | "sigma_sufficient_but_narrow"
  | "all_invisible"
  | "all_ik_fail"
  | "user_marked_fail"
  | "insufficient_poses"
  | "no_board_estimate";

export type HandeyeRecommendationsState = {
  timestamp: number;
  recommendations: NextPoseRecommendation[];
  no_candidates_reason?: NoCandidatesReason | null;
};

/**
 * `CALIB_HANDEYE_SATURATE` topic — σ 변화율 추적 saturate 인지.
 * saturate=true & in_good=true → "sufficient, COMMIT 권장".
 * saturate=true & in_good=false → "floor 도달, escape 시도 (BA mode / 자유 자세 / 외부 도구)".
 */
export type HandeyeSaturateState = {
  timestamp: number;
  saturate: boolean;
  in_good: boolean;
  reason: string;
  sigma_history: number[];
};

/**
 * 사용자 명시 신호 — 추천 자세 fail 기록.
 */
export type RecommendationFailReq = {
  anchor_id: string;
  category: "not_visible" | "red" | "motion_fail";
};

export type RecommendationFailRes = {
  excluded_count: number;
};

/**
 * Multi-start BA — random init 다중 시도 → 가장 좋은 σ.
 * 사용자가 saturate 알림 받고 escape 자체 자리, 또는 [수동 모드 종료] 자동 트리거.
 */
export type MultiStartReq = {
  n_starts?: number;
  mode?: "standard" | "extended" | "physical_sag";
};

export type MultiStartRes = {
  n_tried: number;
  n_converged: number;
  sigma_rot_deg: number | null;
  sigma_t_mm: number | null;
  improvement_rot_deg: number | null;
  improvement_t_mm: number | null;
};

/**
 * `CALIB_BACKUP_LIST` 응답 한 행 — `.history/<ts>_<tag>/meta.json` 의 picker 표시용.
 */
export type BackupEntry = {
  timestamp: string;
  tag: string;
  sigma_rot_deg: number | null;
  sigma_t_mm: number | null;
  capture_count: number | null;
  ba_mode: string | null;
};

/**
 * `CALIB_HANDEYE_THRESHOLDS` 응답 — 백엔드 thresholds.py의 단일 출처.
 * 프론트엔드는 RobotCalibrateMode 진입 시 calibrationStore.bootstrap 안에서 1회 fetch.
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
  min_poses_for_trusted_sigma: number;
  recommended_poses: number;
  joint_diversity_threshold_deg: number[];
  tilt_min_deg?: number;
  tilt_max_deg?: number;
  recommend_distance_m?: number;
  recommend_side_offset_m?: number;
  intrinsic_rms_good_px?: number;
  intrinsic_rms_warn_px?: number;
  intrinsic_min_captures?: number;
  intrinsic_recommended_captures?: number;
  intrinsic_grid_coverage_good?: number;
};
